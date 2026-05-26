#include <msp430.h>
#include <stdint.h>

// Configure UART
#define PC_BAUD_9600_BRW    52
#define PC_BAUD_9600_MCTL   (UCOS16 | UCBRF0 | 0x4900)     // 9600 @ 8 MHz

#define BLE_BAUD_9600_BRW   52
#define BLE_BAUD_9600_MCTL  (UCOS16 | UCBRF0 | 0x4900)     // 9600 @ 8 MHz

// Steup buffers
#define CAP_SIZE        64u      // circular capture buffer (BLE->MSP)
#define TXQ_SIZE        64u      // BLE TX queue (MSP->BLE)

#define PKT_SIZE        14u      // each received packet is 14 bytes. Might change to add an end byte
#define PKT_START_BYTE  0xFFu    // start byte of packet is 255 (0xFF)

#define ACCEPTED_PKTS_MAX  4u    // store 4 accepted bytes

// RN4871 RESET pin on MSP430, set P1.7 as an output, pull-down for reset
#define RN_RST_PORT_DIR   P1DIR
#define RN_RST_PORT_OUT   P1OUT
#define RN_RST_PORT_SEL0  P1SEL0
#define RN_RST_PORT_SEL1  P1SEL1
#define RN_RST_BIT        BIT7

// -------------- CAPTURE BUFFER (BLE->MSP) ----------------------
// Circular buffer: producer = UCA1 RX ISR, consumer = main loop
// All received bytes in UCA1 RX ISR are added to the capture buffer
// Contents are sorted and kept/discarded in main
static volatile uint8_t  cap[CAP_SIZE];
static volatile uint16_t cap_w = 0;
static volatile uint16_t cap_r = 0;
static volatile uint16_t cap_drop = 0;

// UART hardware overrun counters (not using now, were handy for debug)
static volatile uint16_t uca0_ovr = 0;
static volatile uint16_t uca1_ovr = 0;

// --------------- BLE TX QUEUE (MSP->BLE on UCA1) --------------
// used to send commands to BLE module
static volatile uint8_t  txq[TXQ_SIZE];
static volatile uint8_t  txq_w = 0;
static volatile uint8_t  txq_r = 0;
static volatile uint16_t txq_drop = 0;

// ---------------- ACCEPTED PACKETS FIFO ----------------------
// stores accepted data packets sent over BLE, will be used to updated OLED
static volatile uint8_t  accepted_pkts[ACCEPTED_PKTS_MAX * PKT_SIZE];
static volatile uint8_t  accepted_w = 0;
static volatile uint8_t  accepted_r = 0;
static volatile uint16_t accepted_drop = 0;

// ------------------ MAIN-LOOP FLAGS ------------------------------
// set in UCA0 RX ISR based on input from PC (used for debug)
static volatile uint8_t do_hwreset   = 0;
static volatile uint8_t do_autosetup = 0;
static volatile uint8_t do_clear     = 0;

// ------------------- FORWARD DECLARATIONS ------------------------
static void initClock(void);
static void initUART_PC(void);
static void initUART_BLE(void);

static void rn_reset_init_pin(void);
static void rn_reset_pulse(void);
static void delay_ms(uint16_t ms);
static void rn_auto_setup_sequence(void);

// PC TX (blocking)
static void pc_putc(uint8_t c);

// Helpers
static inline void kick_uart1_tx(void);

// CAP ring helpers (consumer-side)
static inline uint16_t cap_count_snapshot(void);
static inline uint8_t  cap_peek0(uint8_t *b);
static inline uint8_t  cap_pop(uint8_t *b);

// Accepted FIFO helpers
static inline uint8_t accepted_count(void);
static inline uint8_t accepted_push_packet(const uint8_t *pkt14);
static inline uint8_t accepted_pop_packet(uint8_t *pkt14);

// Packet parsing
static inline uint8_t validate_packet14(const uint8_t *pkt14);
static void parse_cap_into_accepted(void);

// Clear
static void clear_all(void);

// ---------------- PC (UCA0) BLOCKING TX ------------------------------
static void pc_putc(uint8_t c)
{
    while(!(UCA0IFG & UCTXIFG)) {}
    UCA0TXBUF = c;
}

static void pc_send_bytes_blocking(const uint8_t *buf, uint8_t n)
{
    uint8_t i;
    for (i = 0; i < n; i++) pc_putc(buf[i]);
}

// ------------------- TXQ HELPERS (BLE TX)-------------------------------
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
    while (*s) (void)txq_push((uint8_t)(*s++));
}

static inline void kick_uart1_tx(void)
{
    UCA1IE |= UCTXIE;
    UCA1IFG |= UCTXIFG;
}

// ------------------------ CAP RING (BLE->MSP) ------------------------------
// Snapshot count in a short critical section (main reads r/w; ISR updates w)
static inline uint16_t cap_count_snapshot(void)
{
    uint16_t w, r;
    __disable_interrupt();
    w = cap_w;
    r = cap_r;
    __enable_interrupt();
    return (uint16_t)(w - r);
}

static inline uint8_t cap_peek0(uint8_t *b)
{
    uint16_t w, r;
    __disable_interrupt();
    w = cap_w;
    r = cap_r;
    if (w == r) { __enable_interrupt(); return 0; }
    *b = cap[r & (CAP_SIZE - 1u)];
    __enable_interrupt();
    return 1;
}

static inline uint8_t cap_pop(uint8_t *b)
{
    uint16_t w, r;
    __disable_interrupt();
    w = cap_w;
    r = cap_r;
    if (w == r) { __enable_interrupt(); return 0; }
    *b = cap[r & (CAP_SIZE - 1u)];
    cap_r = (uint16_t)(r + 1u);
    __enable_interrupt();
    return 1;
}

// ---------------------------------- ACCEPTED PACKET FIFO ---------------------------
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

static inline uint8_t accepted_pop_packet(uint8_t *pkt14)
{
    uint16_t base;
    uint8_t i;

    if (accepted_count() == 0) return 0;

    base = (uint16_t)(accepted_r & (ACCEPTED_PKTS_MAX - 1u)) * PKT_SIZE;
    for (i = 0; i < PKT_SIZE; i++) {
        pkt14[i] = accepted_pkts[base + i];
    }
    accepted_r++;
    return 1;
}

// --------------------- PACKET VALIDATION ----------------------------
// This may need to be updated by defining what the next 13 bytes look like (or by adding an end byte)
// For now: accept anything with correct start byte
static inline uint8_t validate_packet14(const uint8_t *pkt14)
{
    (void)pkt14;
    return 1;
}

// =------------------- PARSING (main loop) -----------------------
// Discard until 0xFF, then only extract when 14 bytes are available.
static void parse_cap_into_accepted(void)
{
    uint16_t cnt;

    while (1) {
        uint8_t b;

        cnt = cap_count_snapshot();
        if (cnt == 0) return;

        // Discard junk/unwanted bytes until start byte
        if (!cap_peek0(&b)) return;
        if (b != PKT_START_BYTE) {
            (void)cap_pop(&b);
            continue;
        }

        // Start byte found. Wait for full 14 bytes.
        if (cnt < PKT_SIZE) return;

        // Pop 14 bytes into a local packet buffer
        {
            uint8_t pkt[PKT_SIZE];
            uint8_t i;

            for (i = 0; i < PKT_SIZE; i++) {
                (void)cap_pop(&pkt[i]);
            }

            // Validate and enqueue
            if (validate_packet14(pkt)) {
                __disable_interrupt();
                (void)accepted_push_packet(pkt);
                __enable_interrupt();
            }
            // else drop silently
        }
    }
}

// --------------- CLEAR ---------------------
static void clear_all(void)
{
    __disable_interrupt();
    cap_w = cap_r = 0;
    cap_drop = 0;

    txq_w = txq_r = 0;
    txq_drop = 0;

    accepted_w = accepted_r = 0;
    accepted_drop = 0;

    uca0_ovr = 0;
    uca1_ovr = 0;
    __enable_interrupt();
}

// Hardware reset pin setup
static void rn_reset_init_pin(void)
{
    RN_RST_PORT_SEL0 &= ~RN_RST_BIT;
    RN_RST_PORT_SEL1 &= ~RN_RST_BIT;

    RN_RST_PORT_DIR |= RN_RST_BIT;
    RN_RST_PORT_OUT |= RN_RST_BIT; // idle high
}

// Send hardware reset pulse
static void rn_reset_pulse(void)
{
    RN_RST_PORT_OUT &= ~RN_RST_BIT;
    delay_ms(50);
    RN_RST_PORT_OUT |= RN_RST_BIT;
}

// Delay, useful for time between command inputs to BLE module
static void delay_ms(uint16_t ms)
{
    while (ms--) __delay_cycles(8000); // 1 ms at 8 MHz
}

// Auto setup sequence, can run on start (after delay)
static void rn_auto_setup_sequence(void)
{
    // minimal blocking delays; all BLE sends go via txq + UCA1 TX ISR
    rn_reset_pulse();
    delay_ms(600);

    // enter CMD ($$$)
    delay_ms(250);
    txq_push('$'); txq_push('$'); txq_push('$');
    kick_uart1_tx();
    delay_ms(350);

    // enable transparent UART + device info (as before)
    txq_push_str("SS,C0\r");
    kick_uart1_tx();
    delay_ms(200);

    // iOS-friendly conn params
    txq_push_str("ST,0010,0030,0000,0C80\r");
    kick_uart1_tx();
    delay_ms(200);

    // adv params
    txq_push_str("STA,0020,0010,00A0\r");
    kick_uart1_tx();
    delay_ms(200);

    // clear whitelist
    txq_push_str("JC\r");
    kick_uart1_tx();
    delay_ms(150);

    // NEW: echo on
    txq_push_str("+\r");
    kick_uart1_tx();
    delay_ms(150);

    // reboot
    txq_push_str("R,1\r");
    kick_uart1_tx();
    delay_ms(1400);

    // advertise
    txq_push_str("A\r");
    kick_uart1_tx();
    delay_ms(100);
}

// ===================== MAIN =====================
int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;

    initClock();
    initUART_PC();
    initUART_BLE();
    rn_reset_init_pin();

    clear_all();
    __enable_interrupt();

    // TO ADD HERE: delay, followed by rn_auto_setup_sequence();
    // for now, I am setting this manually over UART from PC

    while (1)
    {
        if (do_clear) {
            do_clear = 0;
            clear_all();
        }

        if (do_hwreset) {
            do_hwreset = 0;
            rn_reset_pulse();
        }

        if (do_autosetup) {
            do_autosetup = 0;
            rn_auto_setup_sequence();
        }

        // Parse any newly received BLE bytes into accepted FIFO
        parse_cap_into_accepted();

        // If we have an accepted packet, forward it to PC (UCA0), used to validate it is working
        if (accepted_count() != 0) {
            uint8_t pkt[PKT_SIZE];
            __disable_interrupt();
            (void)accepted_pop_packet(pkt);
            __enable_interrupt();

            // Forward raw 14 bytes to PC
            pc_send_bytes_blocking(pkt, PKT_SIZE);
        }

        __no_operation();
    }
}

// ===================== CLOCK / UART INIT =====================
static void initClock(void)
{
    CSCTL0 = 0xA500;
    CSCTL1 = DCOFSEL0 + DCOFSEL1; // 8 MHz
    CSCTL2 = SELM__DCOCLK | SELS__DCOCLK | SELA__DCOCLK; // setting all clocks to 8 MHz
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

            // Keeping only a few commands from the debug code
            // 0x01 (CMD mode), 0x0E (autosetup), 0x0D (HW RST), 0x04 (clear)
            if (b == 0x01) {
                txq_push('$'); txq_push('$'); txq_push('$');
                kick_uart1_tx();
            } else if (b == 0x0E) {
                do_autosetup = 1;
            } else if (b == 0x0D) {
                do_hwreset = 1;
            } else if (b == 0x04) {
                do_clear = 1;
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
        case 2: // RXIFG (BLE -> cap ring)
        {
            uint8_t b = UCA1RXBUF;

            if (UCA1STATW & UCOE) { uca1_ovr++; UCA1STATW &= ~UCOE; }

            // ring buffer push (producer only)
            // full when count == CAP_SIZE
            {
                uint16_t w = cap_w;
                uint16_t r = cap_r;
                uint16_t cnt = (uint16_t)(w - r);

                if (cnt < CAP_SIZE) {
                    cap[w & (CAP_SIZE - 1u)] = b;
                    cap_w = (uint16_t)(w + 1u);
                } else {
                    cap_drop++;
                }
            }
            break;
        }

        case 4: // TXIFG (send queued bytes to BLE)
        {
            uint8_t out;
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
