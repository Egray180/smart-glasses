#include <msp430.h>
#include <stdint.h>

// ===================== CONFIG =====================
#define PC_BAUD_9600_BRW    52
#define PC_BAUD_9600_MCTL   (UCOS16 | UCBRF0 | 0x4900)     // known-good 9600 @ 8 MHz

// NOTE: BLE UART is ALSO 9600 now (renamed from 115200 to avoid confusion)
#define BLE_BAUD_9600_BRW   52
#define BLE_BAUD_9600_MCTL  (UCOS16 | UCBRF0 | 0x4900)     // known-good 9600 @ 8 MHz

#define CAP_SIZE    256u     // capture buffer for BLE responses
#define TXQ_SIZE    64u      // TX queue for UCA1 commands, MUST be power of 2

// Packet extraction
#define PKT_SIZE            14u
#define PKT_START_BYTE      0xFFu
#define ACCEPTED_PKTS_MAX   16u   // store up to 16 packets (16*14=224 bytes)

// RN4871 RESET pin on MSP430
#define RN_RST_PORT_DIR   P1DIR
#define RN_RST_PORT_OUT   P1OUT
#define RN_RST_PORT_SEL0  P1SEL0
#define RN_RST_PORT_SEL1  P1SEL1
#define RN_RST_BIT        BIT7

// ===================== CAPTURE BUFFER (BLE->MSP) =====================
static volatile uint8_t  cap[CAP_SIZE];
static volatile uint16_t cap_len  = 0;
static volatile uint16_t cap_drop = 0;

// UART hardware overrun counters
static volatile uint16_t uca0_ovr = 0;
static volatile uint16_t uca1_ovr = 0;

// ===================== BLE TX QUEUE (MSP->BLE on UCA1) =====================
static volatile uint8_t  txq[TXQ_SIZE];
static volatile uint8_t  txq_w = 0;
static volatile uint8_t  txq_r = 0;
static volatile uint16_t txq_drop = 0;

// Useful counters to diagnose "TX not draining"
static volatile uint32_t uca1_tx_isr = 0;   // increments every UCA1 TXIFG ISR entry

// ===================== ACCEPTED PACKETS (for OLED, etc.) =====================
// Flat storage: accepted_pkts[(pkt_index * 14) + byte_index]
static volatile uint8_t  accepted_pkts[ACCEPTED_PKTS_MAX * PKT_SIZE];
static volatile uint8_t  accepted_w = 0;     // write index (wraps)
static volatile uint8_t  accepted_r = 0;     // read index (wraps)
static volatile uint16_t accepted_drop = 0;  // packets dropped when full

// ===================== MAIN-LOOP FLAGS =====================
static volatile uint8_t do_dump      = 0;   // 0x05: dump capture then process into accepted packets
static volatile uint8_t do_stats     = 0;
static volatile uint8_t do_hwreset   = 0;
static volatile uint8_t do_autosetup = 0;
static volatile uint8_t do_dump_acc  = 0;   // 0x14 (decimal 20): dump accepted packets

// ===================== FORWARD DECLS =====================
static void initClock(void);
static void initUART_PC(void);
static void initUART_BLE(void);

static void cap_clear_all(void);
static void cap_clear_len_only(void);

static void pc_print_banner(void);
static void pc_print_stats_blocking(void);
static void pc_dump_capture_blocking(void);

static void process_capture_into_packets(void);
static void pc_dump_accepted_packets_blocking(void);

static void rn_reset_init_pin(void);
static void rn_reset_pulse(void);
static void delay_ms(uint16_t ms);
static void rn_auto_setup_sequence(void);

// ===================== PC (UCA0) BLOCKING PRINT =====================
static void pc_putc(char c)
{
    while(!(UCA0IFG & UCTXIFG)) {}
    UCA0TXBUF = (uint8_t)c;
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

static void pc_puthex8(uint8_t v)
{
    const char *h = "0123456789ABCDEF";
    pc_putc(h[(v >> 4) & 0x0F]);
    pc_putc(h[v & 0x0F]);
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
        txq_push((uint8_t)(*s++));
    }
}

static inline void kick_uart1_tx(void)
{
    UCA1IE |= UCTXIE;
    UCA1IFG |= UCTXIFG;   // force UCA1 TX ISR to run soon
}

// ===================== ACCEPTED PACKETS HELPERS =====================
static inline uint8_t accepted_count(void) { return (uint8_t)(accepted_w - accepted_r); }

static inline uint8_t accepted_push_packet(const uint8_t *pkt14)
{
    uint16_t base;
    uint8_t i;

    if (accepted_count() >= ACCEPTED_PKTS_MAX) {
        accepted_drop++;
        return 0;
    }

    base = (uint16_t)(accepted_w & (ACCEPTED_PKTS_MAX - 1u)) * PKT_SIZE;
    for (i = 0; i < PKT_SIZE; i++) {
        accepted_pkts[base + i] = pkt14[i];
    }
    accepted_w++;
    return 1;
}

// ===================== CAPTURE / STATS =====================
static void cap_clear_all(void)
{
    __disable_interrupt();
    cap_len = 0;
    cap_drop = 0;
    uca1_ovr = 0;
    txq_drop = 0;
    txq_w = txq_r = 0;
    uca1_tx_isr = 0;

    accepted_w = accepted_r = 0;
    accepted_drop = 0;
    __enable_interrupt();
}

static void cap_clear_len_only(void)
{
    __disable_interrupt();
    cap_len = 0;
    __enable_interrupt();
}

static void pc_print_banner(void)
{
    pc_puts("\r\nDECOUPLED BRIDGE TEST (NO UCA0 TX ISR)\r\n");
    pc_puts("PC(UCA0)=9600, BLE(UCA1)=9600\r\n");
    pc_puts("Commands from PC (single byte control codes):\r\n");
    pc_puts(" 00 = R,1<CR>     (reboot RN4871)\r\n");
    pc_puts(" 01 = $$$         (enter CMD mode; guard time handled on PC side)\r\n");
    pc_puts(" 02 = D<CR>\r\n");
    pc_puts(" 03 = ---<CR>\r\n");
    pc_puts(" 04 = CLEAR CAPTURE + QUEUES\r\n");
    pc_puts(" 05 = DUMP CAPTURE + PROCESS -> acceptedPackets (14B starting with 0xFF)\r\n");
    pc_puts(" 06 = STATS\r\n");
    pc_puts(" 07 = SS,C0<CR>   (enable UART Transparent + Device Info; needs reboot)\r\n");
    pc_puts(" 08 = +<CR>       (echo on)\r\n");
    pc_puts(" 09 = A<CR>       (start advertising)\r\n");
    pc_puts(" 0A = ST,0010,0030,0000,01F4<CR>   (iOS-friendly conn params)\r\n");
    pc_puts(" 0B = STA,0020,0010,00A0<CR>       (adv params)\r\n");
    pc_puts(" 0C = JC<CR>      (clear whitelist)\r\n");
    pc_puts(" 0D = HW RESET pulse on P1.7 (active low)\r\n");
    pc_puts(" 0E = AUTO SETUP sequence (reset + $$$ + SS,C0 + ST + STA + JC + R,1 + A)\r\n");
    pc_puts(" 14 = DUMP acceptedPackets (decimal 20)\r\n");
    pc_puts("\r\nManual suggested setup:\r\n");
    pc_puts(" 0D (reset), wait, 01, 07, 0A, 0B, 0C, 00, 09\r\n\r\n");
}

static void pc_print_stats_blocking(void)
{
    uint16_t len, drop, o0, o1, tdrop;
    uint8_t  tqc;
    uint32_t txisr;

    uint8_t  ap_cnt;
    uint16_t ap_drop;

    __disable_interrupt();
    len   = cap_len;
    drop  = cap_drop;
    o0    = uca0_ovr;
    o1    = uca1_ovr;
    tdrop = txq_drop;
    tqc   = txq_count();
    txisr = uca1_tx_isr;

    ap_cnt  = accepted_count();
    ap_drop = accepted_drop;
    __enable_interrupt();

    pc_puts("\r\nSTATS\r\n");
    pc_puts("cap_len="); pc_putu16(len);
    pc_puts("  cap_drop="); pc_putu16(drop);
    pc_puts("\r\nuca0_ovr="); pc_putu16(o0);
    pc_puts("  uca1_ovr="); pc_putu16(o1);
    pc_puts("\r\ntxq_drop="); pc_putu16(tdrop);
    pc_puts("  txq_count="); pc_putu16(tqc);
    pc_puts("\r\nuca1_tx_isr="); pc_putu32(txisr);

    pc_puts("\r\naccepted_count="); pc_putu16(ap_cnt);
    pc_puts("  accepted_drop="); pc_putu16(ap_drop);

    pc_puts("\r\n\r\n");
}

static void pc_dump_capture_blocking(void)
{
    uint16_t n, i;

    __disable_interrupt();
    n = cap_len;
    __enable_interrupt();

    pc_puts("\r\nDUMP:\r\n");

    for (i = 0; i < n; i++) {
        pc_putc((char)cap[i]);   // blocking TX, but interrupts remain enabled
    }
    pc_puts("\r\n\r\n");
}

static void pc_dump_accepted_packets_blocking(void)
{
    uint8_t  cnt, r, w;
    uint16_t drop;
    uint8_t  p;
    uint8_t  i;
    uint8_t  idx;
    uint16_t base;

    __disable_interrupt();
    r = accepted_r;
    w = accepted_w;
    cnt = accepted_count();
    drop = accepted_drop;
    __enable_interrupt();

    pc_puts("\r\nACCEPTED PACKETS\r\n");
    pc_puts("count="); pc_putu16(cnt);
    pc_puts("  drop="); pc_putu16(drop);
    pc_puts("\r\n");

    (void)w; // silence unused warning if any

    for (p = 0; p < cnt; p++) {
        idx = (uint8_t)(r + p);
        base = (uint16_t)(idx & (ACCEPTED_PKTS_MAX - 1u)) * PKT_SIZE;

        pc_puts("pkt["); pc_putu16(p); pc_puts("]: ");

        for (i = 0; i < PKT_SIZE; i++) {
            uint8_t b;
            __disable_interrupt();
            b = accepted_pkts[base + i];
            __enable_interrupt();

            pc_puthex8(b);
            pc_putc(' ');
        }
        pc_puts("\r\n");
    }

    pc_puts("\r\n");
}

// ===================== CAPTURE -> PACKETS (main-loop) =====================
static void process_capture_into_packets(void)
{
    uint8_t  shadow[CAP_SIZE];
    uint16_t n;
    uint16_t i;
    uint16_t accepted_here;

    // quick snapshot
    __disable_interrupt();
    n = cap_len;
    if (n > CAP_SIZE) n = CAP_SIZE;
    for (i = 0; i < n; i++) shadow[i] = cap[i];
    cap_len = 0; // free capture immediately
    __enable_interrupt();

    // parse snapshot for packets
    i = 0;
    accepted_here = 0;

    while (i < n) {
        if (shadow[i] == (uint8_t)PKT_START_BYTE) {
            if ((uint16_t)(i + PKT_SIZE) <= n) {
                __disable_interrupt();
                (void)accepted_push_packet(&shadow[i]);
                __enable_interrupt();

                accepted_here++;
                i = (uint16_t)(i + PKT_SIZE);
                continue;
            } else {
                break;
            }
        }
        i++;
    }

    pc_puts("\r\n[PROCESS] accepted="); pc_putu16(accepted_here);
    pc_puts("  remaining_discarded="); pc_putu16((uint16_t)(n - (uint16_t)(accepted_here * PKT_SIZE)));
    pc_puts("\r\n");
}

// ===================== RESET + DELAY + AUTO SETUP =====================
static void rn_reset_init_pin(void)
{
    RN_RST_PORT_SEL0 &= ~RN_RST_BIT;
    RN_RST_PORT_SEL1 &= ~RN_RST_BIT;

    RN_RST_PORT_DIR |= RN_RST_BIT;
    RN_RST_PORT_OUT |= RN_RST_BIT;
}

static void rn_reset_pulse(void)
{
    RN_RST_PORT_OUT &= ~RN_RST_BIT;
    delay_ms(50);
    RN_RST_PORT_OUT |= RN_RST_BIT;
}

static void delay_ms(uint16_t ms)
{
    while (ms--) {
        __delay_cycles(8000);
    }
}

static void rn_auto_setup_sequence(void)
{
    pc_puts("\r\n[AUTO] reset...\r\n");
    rn_reset_pulse();
    delay_ms(600);

    pc_puts("[AUTO] enter CMD ($$$)...\r\n");
    delay_ms(250);
    txq_push('$'); txq_push('$'); txq_push('$');
    kick_uart1_tx();
    delay_ms(350);

    pc_puts("[AUTO] SS,C0\r\n");
    txq_push_str("SS,C0\r");
    kick_uart1_tx();
    delay_ms(200);

    pc_puts("[AUTO] ST,...\r\n");
    txq_push_str("ST,0010,0030,0000,01F4\r");
    kick_uart1_tx();
    delay_ms(200);

    pc_puts("[AUTO] STA,...\r\n");
    txq_push_str("STA,0020,0010,00A0\r");
    kick_uart1_tx();
    delay_ms(200);

    pc_puts("[AUTO] JC\r\n");
    txq_push_str("JC\r");
    kick_uart1_tx();
    delay_ms(150);

    pc_puts("[AUTO] R,1 (reboot)\r\n");
    txq_push_str("R,1\r");
    kick_uart1_tx();
    delay_ms(1400);

    pc_puts("[AUTO] A (advertise)\r\n");
    txq_push_str("A\r");
    kick_uart1_tx();
    delay_ms(100);

    pc_puts("[AUTO] done.\r\n\r\n");
}

// ===================== MAIN =====================
int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;

    initClock();
    initUART_PC();
    initUART_BLE();
    rn_reset_init_pin();

    cap_clear_all();
    __enable_interrupt();

    pc_print_banner();

    while(1)
    {
        if (do_hwreset) {
            do_hwreset = 0;
            rn_reset_pulse();
            pc_puts("\r\n[HW RESET PULSE]\r\n");
        }

        if (do_autosetup) {
            do_autosetup = 0;
            rn_auto_setup_sequence();
        }

        if (do_dump) {
            do_dump = 0;
            pc_dump_capture_blocking();      // keep visibility of capture contents
            process_capture_into_packets();   // then extract packets
        }

        if (do_dump_acc) {
            do_dump_acc = 0;
            pc_dump_accepted_packets_blocking();
        }

        if (do_stats) {
            do_stats = 0;
            pc_print_stats_blocking();
        }

        __no_operation();
    }
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
    P2SEL0 &= ~(BIT0 | BIT1);
    P2SEL1 |=  (BIT0 | BIT1);

    UCA0CTLW0 = UCSWRST | UCSSEL__SMCLK;
    UCA0BRW   = PC_BAUD_9600_BRW;
    UCA0MCTLW = PC_BAUD_9600_MCTL;
    UCA0CTLW0 &= ~UCSWRST;

    UCA0IE |= UCRXIE;
}

static void initUART_BLE(void)
{
    P2SEL0 &= ~(BIT5 | BIT6);
    P2SEL1 |=  (BIT5 | BIT6);

    UCA1CTLW0 = UCSWRST | UCSSEL__SMCLK;
    UCA1BRW   = BLE_BAUD_9600_BRW;
    UCA1MCTLW = BLE_BAUD_9600_MCTL;
    UCA1CTLW0 &= ~UCPEN;
    UCA1CTLW0 &= ~UCSWRST;

    UCA1IE |= UCRXIE;
}

// ===================== ISRs =====================
#pragma vector = USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
{
    switch(__even_in_range(UCA0IV, 4))
    {
        case 2: // RXIFG
        {
            uint8_t b = UCA0RXBUF;

            if (UCA0STATW & UCOE) { uca0_ovr++; UCA0STATW &= ~UCOE; }

            if (b == 0x00) {
                txq_push_str("R,1\r"); kick_uart1_tx();
            } else if (b == 0x01) {
                txq_push('$'); txq_push('$'); txq_push('$'); kick_uart1_tx();
            } else if (b == 0x02) {
                txq_push('D'); txq_push('\r'); kick_uart1_tx();
            } else if (b == 0x03) {
                txq_push('-'); txq_push('-'); txq_push('-'); txq_push('\r'); kick_uart1_tx();
            } else if (b == 0x04) {
                cap_clear_all();
                pc_putc('!');
            } else if (b == 0x05) {
                do_dump = 1;
            } else if (b == 0x06) {
                do_stats = 1;
            } else if (b == 0x07) {
                txq_push_str("SS,C0\r"); kick_uart1_tx();
            } else if (b == 0x08) {
                txq_push_str("+\r"); kick_uart1_tx();
            } else if (b == 0x09) {
                txq_push_str("A\r"); kick_uart1_tx();
            } else if (b == 0x0A) {
                txq_push_str("ST,0010,0030,0000,01F4\r"); kick_uart1_tx();
            } else if (b == 0x0B) {
                txq_push_str("STA,0020,0010,00A0\r"); kick_uart1_tx();
            } else if (b == 0x0C) {
                txq_push_str("JC\r"); kick_uart1_tx();
            } else if (b == 0x0D) {
                do_hwreset = 1;
            } else if (b == 0x0E) {
                do_autosetup = 1;
            } else if (b == 0x0F) {
                txq_push_str("V\r"); kick_uart1_tx();
            } else if (b == 0x14) {        // decimal 20
                do_dump_acc = 1;
            }
            break;
        }
        default: break;
    }
}

#pragma vector = USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
{
    switch(__even_in_range(UCA1IV, 4))
    {
        case 2: // RXIFG (BLE -> capture buffer)
        {
            uint8_t b = UCA1RXBUF;

            if (UCA1STATW & UCOE) { uca1_ovr++; UCA1STATW &= ~UCOE; }

            if (cap_len < CAP_SIZE) {
                cap[cap_len++] = b;
            } else {
                cap_drop++;
            }
            break;
        }

        case 4: // TXIFG (send queued bytes to BLE)
        {
            uint8_t out;

            uca1_tx_isr++;

            if (txq_pop(&out)) {
                UCA1TXBUF = out;
            } else {
                UCA1IE &= ~UCTXIE;
            }
            break;
        }

        default: break;
    }
}
