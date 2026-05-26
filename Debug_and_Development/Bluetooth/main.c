#include <msp430.h>
#include <stdint.h>

// ===================== CONFIG =====================
#define PC_BAUD_9600_BRW   52
#define PC_BAUD_9600_MCTL  (UCOS16 | UCBRF0 | 0x4900)        // known-good 9600 @ 8 MHz

#define BLE_BAUD_115200_BRW  52
#define BLE_BAUD_115200_MCTL (UCOS16 | UCBRF0 | 0x4900)    // 9600 baud

#define CAP_SIZE   256u     // capture buffer for BLE responses
#define TXQ_SIZE   64u      // TX queue for UCA1 commands, MUST be power of 2

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

// ===================== MAIN-LOOP FLAGS =====================
static volatile uint8_t do_dump  = 0;
static volatile uint8_t do_stats = 0;

// ===================== FORWARD DECLS =====================
static void initClock(void);
static void initUART_PC(void);
static void initUART_BLE(void);

static void cap_clear(void);
static void pc_print_banner(void);
static void pc_print_stats_blocking(void);
static void pc_dump_capture_blocking(void);

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

// Push a null-terminated ASCII string into txq (best-effort).
static void txq_push_str(const char *s)
{
    while (*s) {
        txq_push((uint8_t)(*s++));
    }
}

// IMPORTANT: prime TX immediately if TXIFG is already set
static inline void kick_uart1_tx(void)
{
    UCA1IE |= UCTXIE;
    UCA1IFG |= UCTXIFG;   // force UCA1 TX ISR to run soon
}

// ===================== CAPTURE / STATS =====================
static void cap_clear(void)
{
    __disable_interrupt();
    cap_len = 0;
    cap_drop = 0;
    uca1_ovr = 0;
    txq_drop = 0;
    txq_w = txq_r = 0;
    uca1_tx_isr = 0;
    __enable_interrupt();
}

static void pc_print_banner(void)
{
    pc_puts("\r\nDECOUPLED BRIDGE TEST (NO UCA0 TX ISR)\r\n");
    pc_puts("PC(UCA0)=9600, BLE(UCA1)=115200\r\n");
    pc_puts("Commands from PC (single byte control codes):\r\n");
    pc_puts(" 00 = R,1<CR>     (reboot RN4871)\r\n");
    pc_puts(" 01 = $$$         (enter CMD mode; guard time handled on PC side)\r\n");
    pc_puts(" 02 = D<CR>\r\n");
    pc_puts(" 03 = ---<CR>\r\n");
    pc_puts(" 04 = CLEAR CAPTURE\r\n");
    pc_puts(" 05 = DUMP CAPTURE\r\n");
    pc_puts(" 06 = STATS\r\n");
    pc_puts(" 07 = SS,C0<CR>   (enable UART Transparent + Device Info; needs reboot)\r\n");
    pc_puts(" 08 = +<CR>       (echo on)\r\n");
    pc_puts(" 09 = A<CR>       (start advertising)\r\n");
    pc_puts(" 0A = ST,0010,0030,0000,01F4<CR>   (iOS-friendly conn params; reboot recommended)\r\n");
    pc_puts(" 0B = STA,0020,0010,00A0<CR>       (fast/slow adv params; reboot recommended)\r\n");
    pc_puts(" 0C = JC<CR>      (clear whitelist; reboot recommended)\r\n");
    pc_puts("\r\nSuggested setup sequence:\r\n");
    pc_puts(" 01 (enter CMD), 07 (SS,C0), 0A (ST...), 0B (STA...), 0C (JC), 00 (R,1), 09 (A)\r\n\r\n");
}

static void pc_print_stats_blocking(void)
{
    uint16_t len, drop, o0, o1, tdrop;
    uint8_t  tqc;
    uint32_t txisr;

    __disable_interrupt();
    len   = cap_len;
    drop  = cap_drop;
    o0    = uca0_ovr;
    o1    = uca1_ovr;
    tdrop = txq_drop;
    tqc   = txq_count();
    txisr = uca1_tx_isr;
    __enable_interrupt();

    pc_puts("\r\nSTATS\r\n");
    pc_puts("cap_len="); pc_putu16(len);
    pc_puts("  cap_drop="); pc_putu16(drop);
    pc_puts("\r\nuca0_ovr="); pc_putu16(o0);
    pc_puts("  uca1_ovr="); pc_putu16(o1);
    pc_puts("\r\ntxq_drop="); pc_putu16(tdrop);
    pc_puts("  txq_count="); pc_putu16(tqc);
    pc_puts("\r\nuca1_tx_isr="); pc_putu32(txisr);
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

// ===================== MAIN =====================
int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;

    initClock();
    initUART_PC();
    initUART_BLE();

    cap_clear();
    __enable_interrupt();

    pc_print_banner();

    while(1)
    {
        if (do_dump) {
            do_dump = 0;
            pc_dump_capture_blocking();
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
    // P2.0=UCA0TXD, P2.1=UCA0RXD
    P2SEL0 &= ~(BIT0 | BIT1);
    P2SEL1 |=  (BIT0 | BIT1);

    UCA0CTLW0 = UCSWRST | UCSSEL__SMCLK;
    UCA0BRW   = PC_BAUD_9600_BRW;
    UCA0MCTLW = PC_BAUD_9600_MCTL;
    UCA0CTLW0 &= ~UCSWRST;

    UCA0IE |= UCRXIE; // RX interrupt only
}

static void initUART_BLE(void)
{
    // P2.5=UCA1TXD, P2.6=UCA1RXD
    P2SEL0 &= ~(BIT5 | BIT6);
    P2SEL1 |=  (BIT5 | BIT6);

    UCA1CTLW0 = UCSWRST | UCSSEL__SMCLK;
    UCA1BRW   = PC_BAUD_9600_BRW;
    UCA1MCTLW = PC_BAUD_9600_MCTL;
    //UCA1BRW   = BLE_BAUD_115200_BRW;
    //UCA1MCTLW = BLE_BAUD_115200_MCTL;
    UCA1CTLW0 &= ~UCPEN;
    UCA1CTLW0 &= ~UCSWRST;

    UCA1IE |= UCRXIE; // RX interrupt
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

            // Count UART hardware overrun errors on UCA0
            if (UCA0STATW & UCOE) { uca0_ovr++; UCA0STATW &= ~UCOE; }

            // Single-byte control codes from PC:
            if (b == 0x00) {
                // R,1<CR> (reboot)
                txq_push_str("R,1\r");
                kick_uart1_tx();

            } else if (b == 0x01) {
                // $$$ (guard time handled on PC side; no CR)
                txq_push('$'); txq_push('$'); txq_push('$');
                kick_uart1_tx();

            } else if (b == 0x02) {
                txq_push('D'); txq_push('\r');
                kick_uart1_tx();

            } else if (b == 0x03) {
                txq_push('-'); txq_push('-'); txq_push('-'); txq_push('\r');
                kick_uart1_tx();

            } else if (b == 0x04) {
                cap_clear();
                pc_putc('!'); // small ack

            } else if (b == 0x05) {
                do_dump = 1; // dump in main

            } else if (b == 0x06) {
                do_stats = 1; // stats in main

            } else if (b == 0x07) {
                // SS,C0<CR> enable Device Info (0x80) + UART Transparent (0x40) = C0
                txq_push_str("SS,C0\r");
                kick_uart1_tx();

            } else if (b == 0x08) {
                // +<CR> enable echo (optional)
                txq_push_str("+\r");
                kick_uart1_tx();

            } else if (b == 0x09) {
                // A<CR> start advertising (keep this even if auto-adv works)
                txq_push_str("A\r");
                kick_uart1_tx();

            } else if (b == 0x0A) {
                // ST,<min>,<max>,<latency>,<timeout><CR> (iOS-friendly)
                txq_push_str("ST,0010,0030,0000,01F4\r");
                kick_uart1_tx();

            } else if (b == 0x0B) {
                // STA,<fast_int>,<fast_timeout>,<slow_int><CR>
                txq_push_str("STA,0020,0010,00A0\r");
                kick_uart1_tx();

            } else if (b == 0x0C) {
                // JC<CR> clear whitelist
                txq_push_str("JC\r");
                kick_uart1_tx();

            } else {
                // ignore
            }
            break;
        }

        case 4: // TXIFG (unused)
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

            // Count UART hardware overrun errors on UCA1
            if (UCA1STATW & UCOE) { uca1_ovr++; UCA1STATW &= ~UCOE; }

            // Minimal constant-time capture
            if (cap_len < CAP_SIZE) {
                cap[cap_len++] = b;
            } else {
                cap_drop++;
            }
            break;
        }

        case 4: // TXIFG (send queued bytes to BLE)
        {
            uca1_tx_isr++;

            uint8_t out;
            if (txq_pop(&out)) {
                UCA1TXBUF = out;
            } else {
                UCA1IE &= ~UCTXIE; // done
            }
            break;
        }

        default: break;
    }
}
