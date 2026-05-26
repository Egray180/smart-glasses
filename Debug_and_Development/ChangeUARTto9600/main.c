#include <msp430.h>
#include <stdint.h>

// ===================== CONFIG =====================
#define F_CPU_HZ 8000000UL

// Mapping you want:
//   BLE  = UCA0 on P2.0 (TXD) / P2.1 (RXD)
//   PC   = UCA1 on P2.5 (TXD) / P2.6 (RXD)

// PC UART (UCA1) 9600 @ 8 MHz (known-good)
#define PC_BAUD_9600_BRW   52
#define PC_BAUD_9600_MCTL  (UCOS16 | UCBRF0 | 0x4900)

// BLE UART (UCA0) starts at 115200 @ 8 MHz
#define BLE_BAUD_115200_BRW  4
#define BLE_BAUD_115200_MCTL (UCOS16 | UCBRF_5 | 0x5500)

// BLE UART (UCA0) after SB,09 + reboot: 9600 @ 8 MHz
#define BLE_BAUD_9600_BRW   52
#define BLE_BAUD_9600_MCTL  (UCOS16 | UCBRF0 | 0x4900)

#define TXQ_SIZE   128u      // MUST be power of 2

// Your data-mode frame format
#define PKT_LEN    14u
#define PKT_SOF    0xFF

// BLE reset pin
#define BLE_RST_PORT_OUT  P2OUT
#define BLE_RST_PORT_DIR  P2DIR
#define BLE_RST_PORT_SEL0 P2SEL0
#define BLE_RST_PORT_SEL1 P2SEL1
#define BLE_RST_PIN       BIT2

// ===================== UART error counters =====================
static volatile uint16_t uca0_ovr = 0; // BLE (UCA0)
static volatile uint16_t uca1_ovr = 0; // PC  (UCA1)

// ===================== BLE TX QUEUE (MSP->BLE on UCA0) =====================
static volatile uint8_t  txq[TXQ_SIZE];
static volatile uint8_t  txq_w = 0;
static volatile uint8_t  txq_r = 0;
static volatile uint16_t txq_drop = 0;
static volatile uint32_t ble_tx_isr = 0;

// ===================== Packet capture (BLE->MSP) =====================
// Simple 14-byte framed parser: 0xFF + 13 bytes
static volatile uint8_t  rx_pkt[PKT_LEN];
static volatile uint8_t  rx_pkt_ready = 0;
static volatile uint32_t rx_pkt_count = 0;

static volatile uint8_t  parse_active = 0;
static volatile uint8_t  parse_idx = 0;
static volatile uint8_t  parse_buf[PKT_LEN];

// ===================== MAIN-LOOP FLAGS =====================
static volatile uint8_t do_stats = 0;

// ===================== FORWARD DECLS =====================
static void initClock(void);

// PC = UCA1 on P2.5/P2.6
static void initUART_PC(void);

// BLE = UCA0 on P2.0/P2.1
static void initUART_BLE_115200(void);
static void initUART_BLE_9600(void);

static void ble_reset_init_pin(void);
static void ble_hard_reset_pulse(void);

static void txq_push_str(const char *s);
static void kick_ble_tx(void);

static void delay_ms(uint16_t ms);

static void pc_putc(char c);
static void pc_puts(const char *s);
static void pc_putu16(uint16_t v);
static void pc_putu32(uint32_t v);

static void pc_print_banner(void);
static void pc_print_stats_blocking(void);
static void pc_print_packet_blocking(const uint8_t *p, uint8_t n);

static void ble_send_cmd(const char *cmd_with_cr);

static void setupLED(void)
{
    P3DIR |= BIT0;
    P3OUT |= BIT0; // don't clobber other P3 pins
}

static void BlinkLED(void)
{
    P3OUT ^= BIT0;
}

// ===================== small delay helper =====================
static void delay_ms(uint16_t ms)
{
    while (ms--) {
        __delay_cycles(F_CPU_HZ / 1000);
    }
}

// ===================== PC (UCA1) BLOCKING PRINT =====================
static void pc_putc(char c)
{
    while(!(UCA1IFG & UCTXIFG)) {}
    UCA1TXBUF = (uint8_t)c;
}

static void pc_puts(const char *s)
{
    while(*s) pc_putc(*s++);
}

static void pc_putu16(uint16_t v)
{
    char buf[6];
    int i = 0;
    if (v == 0) { pc_putc('0'); return; }
    while (v && i < 5) { buf[i++] = (char)('0' + (v % 10)); v /= 10; }
    while (i--) pc_putc(buf[i]);
}

static void pc_putu32(uint32_t v)
{
    char buf[11];
    int i = 0;
    if (v == 0) { pc_putc('0'); return; }
    while (v && i < 10) { buf[i++] = (char)('0' + (v % 10)); v /= 10; }
    while (i--) pc_putc(buf[i]);
}

static void pc_print_packet_blocking(const uint8_t *p, uint8_t n)
{
    uint8_t i;
    pc_puts("PKT ");
    for (i = 0; i < n; i++) {
        pc_putu16(p[i]);
        pc_putc(i == (n-1) ? '\r' : ' ');
    }
    pc_putc('\n');
}

// ===================== TXQ HELPERS =====================
static inline uint8_t txq_count(void) { return (uint8_t)(txq_w - txq_r); }

static inline uint8_t txq_push(uint8_t b)
{
    if (txq_count() >= TXQ_SIZE) { txq_drop++; return 0; }
    txq[txq_w & (TXQ_SIZE - 1)] = b;
    txq_w++;
    return 1;
}

static inline uint8_t txq_pop(uint8_t *b)
{
    if (txq_count() == 0) return 0;
    *b = txq[txq_r & (TXQ_SIZE - 1)];
    txq_r++;
    return 1;
}

static void txq_push_str(const char *s)
{
    while (*s) {
        (void)txq_push((uint8_t)(*s++));
    }
}

// IMPORTANT: prime TX immediately if TXIFG is already set
static void kick_ble_tx(void)
{
    UCA0IE  |= UCTXIE;
    UCA0IFG |= UCTXIFG;
}

// Send a command string that already includes <CR> as '\r'
static void ble_send_cmd(const char *cmd_with_cr)
{
    txq_push_str(cmd_with_cr);
    kick_ble_tx();
}

// ===================== GPIO reset =====================
static void ble_reset_init_pin(void)
{
    BLE_RST_PORT_SEL0 &= ~BLE_RST_PIN;
    BLE_RST_PORT_SEL1 &= ~BLE_RST_PIN;
    BLE_RST_PORT_DIR  |=  BLE_RST_PIN;  // output
    BLE_RST_PORT_OUT  |=  BLE_RST_PIN;  // idle high (not in reset)
}

static void ble_hard_reset_pulse(void)
{
    BLE_RST_PORT_OUT &= ~BLE_RST_PIN;   // assert reset (active low)
    delay_ms(50);
    BLE_RST_PORT_OUT |= BLE_RST_PIN;    // deassert reset
}

// ===================== banner / stats =====================
static void pc_print_banner(void)
{
    pc_puts("\r\nFW: AUTO BLE INIT + PACKET CAPTURE\r\n");
    pc_puts("PC(UCA1 @ P2.5/2.6)=9600\r\n");
    pc_puts("BLE(UCA0 @ P2.0/2.1)=115200 initially; switch to 9600 after SB,09 + reboot\r\n\r\n");

    pc_puts("PC single-byte control codes:\r\n");
    pc_puts(" 00 = R,1<CR> reboot RN4871\r\n");
    pc_puts(" 01 = $$$ enter CMD mode\r\n");
    pc_puts(" 02 = D<CR>\r\n");
    pc_puts(" 03 = ---<CR>\r\n");
    pc_puts(" 07 = SS,C0<CR>\r\n");
    pc_puts(" 09 = A<CR> advertise\r\n");
    pc_puts(" 0A = ST,0010,0030,0000,01F4<CR>\r\n");
    pc_puts(" 0B = STA,0020,0010,00A0<CR>\r\n");
    pc_puts(" 0C = JC<CR>\r\n");
    pc_puts(" 0D = SB,09<CR> set UART baud to 9600 (takes effect after reboot)\r\n");
    pc_puts(" 06 = STATS\r\n\r\n");
}

static void pc_print_stats_blocking(void)
{
    uint16_t o0, o1, tdrop;
    uint8_t  tqc;
    uint32_t txisr, pcnt;

    __disable_interrupt();
    o0    = uca0_ovr;
    o1    = uca1_ovr;
    tdrop = txq_drop;
    tqc   = txq_count();
    txisr = ble_tx_isr;
    pcnt  = rx_pkt_count;
    __enable_interrupt();

    pc_puts("\r\nSTATS\r\n");
    pc_puts("uca0_ovr="); pc_putu16(o0);
    pc_puts("  uca1_ovr="); pc_putu16(o1);
    pc_puts("\r\ntxq_drop="); pc_putu16(tdrop);
    pc_puts("  txq_count="); pc_putu16(tqc);
    pc_puts("\r\nble_tx_isr="); pc_putu32(txisr);
    pc_puts("\r\nrx_pkt_count="); pc_putu32(pcnt);
    pc_puts("\r\n\r\n");
}

// ===================== CLOCK / UART INIT =====================
static void initClock(void)
{
    CSCTL0 = 0xA500;
    CSCTL1 = DCOFSEL0 + DCOFSEL1; // 8 MHz
    CSCTL2 = SELM__DCOCLK | SELS__DCOCLK | SELA__DCOCLK;
    CSCTL3 = DIVS__1;
    CSCTL0_H = 0x01;
}

static void initUART_PC(void)
{
    // PC = UCA1 : P2.5=UCA1TXD, P2.6=UCA1RXD
    P2SEL0 &= ~(BIT5 | BIT6);
    P2SEL1 |=  (BIT5 | BIT6);

    UCA1CTLW0 = UCSWRST | UCSSEL__SMCLK;
    UCA1BRW   = PC_BAUD_9600_BRW;
    UCA1MCTLW = PC_BAUD_9600_MCTL;
    UCA1CTLW0 &= ~UCSWRST;

    UCA1IE |= UCRXIE; // RX interrupt only
}

static void initUART_BLE_115200(void)
{
    // BLE = UCA0 : P2.0=UCA0TXD, P2.1=UCA0RXD
    P2SEL0 &= ~(BIT0 | BIT1);
    P2SEL1 |=  (BIT0 | BIT1);

    UCA0CTLW0 = UCSWRST | UCSSEL__SMCLK;
    UCA0BRW   = BLE_BAUD_115200_BRW;
    UCA0MCTLW = BLE_BAUD_115200_MCTL;
    UCA0CTLW0 &= ~UCPEN;
    UCA0CTLW0 &= ~UCSWRST;

    UCA0IE |= UCRXIE; // RX interrupt
}

static void initUART_BLE_9600(void)
{
    // BLE stays on UCA0, just change baud safely
    UCA0CTLW0 |= UCSWRST;
    UCA0CTLW0 = (UCA0CTLW0 & ~UCSSEL_3) | UCSSEL__SMCLK;
    UCA0BRW   = BLE_BAUD_9600_BRW;
    UCA0MCTLW = BLE_BAUD_9600_MCTL;
    UCA0CTLW0 &= ~UCPEN;
    UCA0CTLW0 &= ~UCSWRST;

    // RX interrupt already enabled; TX interrupt is on-demand
}

// ===================== AUTO-INIT SEQUENCE =====================
static void ble_auto_init_sequence(void)
{
    delay_ms(200);

    ble_hard_reset_pulse();
    delay_ms(300);

    // Enter CMD mode: requires quiet time before first '$'
    delay_ms(120);
    ble_send_cmd("$$$");   // NO CR for $$$
    delay_ms(200);

    ble_send_cmd("SS,C0\r");
    delay_ms(120);

    ble_send_cmd("ST,0010,0030,0000,01F4\r");
    delay_ms(120);

    ble_send_cmd("STA,0020,0010,00A0\r");
    delay_ms(120);

    ble_send_cmd("JC\r");
    delay_ms(120);

    // Set UART baud to 9600 AFTER reboot
    ble_send_cmd("SB,09\r");       // 09 = 9600
    delay_ms(120);

    BlinkLED();

    // Reboot to apply set commands
    ble_send_cmd("R,1\r");
    delay_ms(800);

    // After reboot, module UART is now 9600 if SB,09 was accepted
    initUART_BLE_9600();
    delay_ms(200);

    // Start advertising
    ble_send_cmd("A\r");
    delay_ms(100);
}

// ===================== MAIN =====================
int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;

#ifdef PM5CTL0
    // Unlock GPIO from high-Z on FRAM parts (safe even if not needed)
    PM5CTL0 &= ~LOCKLPM5;
#endif

    initClock();
    initUART_PC();         // PC  = UCA1 @ P2.5/2.6
    initUART_BLE_115200(); // BLE = UCA0 @ P2.0/2.1
    ble_reset_init_pin();

    setupLED();

    __enable_interrupt();

    pc_print_banner();

    ble_auto_init_sequence();

    pc_puts("Auto-init complete.\r\n");

    while(1)
    {
        if (rx_pkt_ready) {
            uint8_t pkt[PKT_LEN];
            uint8_t i;

            __disable_interrupt();
            for (i = 0; i < PKT_LEN; i++) pkt[i] = rx_pkt[i];
            rx_pkt_ready = 0;
            __enable_interrupt();

            pc_print_packet_blocking(pkt, PKT_LEN);
        }

        if (do_stats) {
            do_stats = 0;
            pc_print_stats_blocking();
        }

        __no_operation();
    }
}

// ===================== ISRs =====================

// BLE ISR = UCA0 (RX capture + TX drain)
#pragma vector = USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
{
    switch(__even_in_range(UCA0IV, 4))
    {
        case 2: // RXIFG (BLE -> MSP)
        {
            uint8_t in = UCA0RXBUF;

            if (UCA0STATW & UCOE) { uca0_ovr++; UCA0STATW &= ~UCOE; }

            // Frame parser for 14-byte packets starting with 0xFF
            if (!parse_active) {
                if (in == PKT_SOF) {
                    parse_active = 1;
                    parse_idx = 0;
                    parse_buf[parse_idx++] = in;
                }
            } else {
                parse_buf[parse_idx++] = in;
                if (parse_idx >= PKT_LEN) {
                    if (!rx_pkt_ready) {
                        uint8_t i;
                        for (i = 0; i < PKT_LEN; i++) rx_pkt[i] = parse_buf[i];
                        rx_pkt_ready = 1;
                        rx_pkt_count++;
                    }
                    parse_active = 0;
                    parse_idx = 0;
                }
            }
            break;
        }

        case 4: // TXIFG (MSP -> BLE)
        {
            ble_tx_isr++;

            uint8_t out;
            if (txq_pop(&out)) {
                UCA0TXBUF = out;
            } else {
                UCA0IE &= ~UCTXIE;
            }
            break;
        }

        default: break;
    }
}

// PC ISR = UCA1 (single-byte control codes)
#pragma vector = USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
{
    switch(__even_in_range(UCA1IV, 4))
    {
        case 2: // RXIFG (PC -> commands)
        {
            uint8_t b = UCA1RXBUF;

            if (UCA1STATW & UCOE) { uca1_ovr++; UCA1STATW &= ~UCOE; }

            // Manual command injections (debug)
            if (b == 0x00)      ble_send_cmd("R,1\r");
            else if (b == 0x01) ble_send_cmd("$$$");
            else if (b == 0x02) ble_send_cmd("D\r");
            else if (b == 0x03) ble_send_cmd("---\r");
            else if (b == 0x07) ble_send_cmd("SS,C0\r");
            else if (b == 0x09) ble_send_cmd("A\r");
            else if (b == 0x0A) ble_send_cmd("ST,0010,0030,0000,01F4\r");
            else if (b == 0x0B) ble_send_cmd("STA,0020,0010,00A0\r");
            else if (b == 0x0C) ble_send_cmd("JC\r");
            else if (b == 0x0D) ble_send_cmd("SB,09\r");
            else if (b == 0x06) do_stats = 1;

            break;
        }

        default: break;
    }
}
