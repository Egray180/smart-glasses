#include <msp430.h>
#include <stdint.h>
#include <stddef.h>

// ===================== UART CONFIG =====================
#define PC_BAUD_9600_BRW    52
#define PC_BAUD_9600_MCTL   (UCOS16 | UCBRF0 | 0x4900)     // 9600 @ 8 MHz

#define BLE_BAUD_9600_BRW   52
#define BLE_BAUD_9600_MCTL  (UCOS16 | UCBRF0 | 0x4900)     // 9600 @ 8 MHz

// ===================== BUFFERS / PACKETS =====================
#define CAP_SIZE        64u      // circular capture buffer (BLE->MSP), power of 2
#define TXQ_SIZE        64u      // BLE TX queue (MSP->BLE), power of 2

#define PKT_SIZE        14u
#define PKT_START_BYTE  0xFFu

#define ACCEPTED_PKTS_MAX  4u    // power of 2

// ===================== RN4871 RESET PIN =====================
#define RN_RST_PORT_DIR   P2DIR
#define RN_RST_PORT_OUT   P2OUT
#define RN_RST_PORT_SEL0  P2SEL0
#define RN_RST_PORT_SEL1  P2SEL1
#define RN_RST_BIT        BIT2

// ===================== CAPTURE BUFFER (BLE->MSP) =====================
static volatile uint8_t  cap[CAP_SIZE];
static volatile uint16_t cap_w = 0;
static volatile uint16_t cap_r = 0;
static volatile uint16_t cap_drop = 0;

// UART hardware overrun counters
static volatile uint16_t uca0_ovr = 0; // BLE (UCA0)
static volatile uint16_t uca1_ovr = 0; // PC  (UCA1)

// ===================== BLE TX QUEUE (MSP->BLE on UCA0) =====================
static volatile uint8_t  txq[TXQ_SIZE];
static volatile uint8_t  txq_w = 0;
static volatile uint8_t  txq_r = 0;
static volatile uint16_t txq_drop = 0;

// ===================== ACCEPTED PACKETS FIFO =====================
static volatile uint8_t  accepted_pkts[ACCEPTED_PKTS_MAX * PKT_SIZE];
static volatile uint8_t  accepted_w = 0;
static volatile uint8_t  accepted_r = 0;
static volatile uint16_t accepted_drop = 0;

// ===================== MAIN-LOOP FLAGS (PC commands) =====================
static volatile uint8_t do_hwreset   = 0;
static volatile uint8_t do_autosetup = 0;
static volatile uint8_t do_clear     = 0;

// ===================== FORWARD DECLARATIONS =====================
static void initClock(void);
static void initUART_PC(void);   // PC = UCA1
static void initUART_BLE(void);  // BLE = UCA0

static void rn_reset_init_pin(void);
static void rn_reset_pulse(void);
static void delay_ms(uint16_t ms);
static void rn_auto_setup_sequence(void);

// PC TX (blocking)  (PC = UCA1)
static void pc_putc(uint8_t c);
static void pc_send_bytes_blocking(const uint8_t *buf, uint8_t n);

// Helpers
static inline void kick_ble_tx(void); // BLE TX uses UCA0 TX ISR

// CAP ring helpers
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

// =====================================================================================
// ===================== OLED (FULL IMPLEMENTATION INCLUDED) ============================
// =====================================================================================

// ---------------- Pin mapping ----------------
#define OLED_SCL_BIT   BIT7   // (your comment said P1.5, but BIT7 is P1.7) keep as you provided
#define OLED_SDA_BIT   BIT6
#define OLED_RES_BIT   BIT5

#define OLED_PORT_DIR  P1DIR
#define OLED_PORT_OUT  P1OUT
#define OLED_PORT_IN   P1IN
#define OLED_PORT_REN  P1REN

#define USE_INTERNAL_PULLUPS 0

// SSD1315 address (8-bit write form) (7-bit 0x3C -> 0x78 write)
#define SSD1315_ADDR_WRITE  0x78

// Control bytes
#define OLED_CTRL_CMD   0x00
#define OLED_CTRL_DATA  0x40

#define OLED_CMD   0
#define OLED_DATA  1

// Prototypes (fixes your implicit-declare warnings + linker errors)
void OLED_WR_Byte(uint8_t dat, uint8_t mode);
void OLED_Set_Pos(uint8_t x, uint8_t y);
void OLED_Clear(void);
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t sizey);
void OLED_ShowString(uint8_t x, uint8_t y, const uint8_t *chr, uint8_t sizey);
void OLED_ShowStringCentered(uint8_t y, const uint8_t *chr, uint8_t sizey);
void OLED_ShowStringCenteredLeft(uint8_t y, const uint8_t *chr, uint8_t sizey);
void OLED_DrawBMP(uint8_t x, uint8_t y, uint8_t sizex, uint8_t sizey, const uint8_t *BMP);
void OLED_Init_DCDC(void);

static void oled_update_from_packet14(const uint8_t *pkt);

// 12x16 arrow head, page-major (2 pages × 12 columns)
static const unsigned char arrow_head_right_12x16[24] = {
  0x08,0x14,0x24,0x48,0x88,0x10,0x10,0x20,0x20,0x40,0x40,0x80,
  0x08,0x14,0x12,0x09,0x08,0x04,0x04,0x02,0x02,0x01,0x01,0x00
};
static const unsigned char arrow_head_left_12x16[24] = {
  0x80,0x40,0x40,0x20,0x20,0x10,0x10,0x88,0x48,0x24,0x14,0x08,
  0x00,0x01,0x01,0x02,0x02,0x04,0x04,0x08,0x09,0x12,0x14,0x08
};
static const unsigned char checkmark_16x16[32] = {
  // Page 0 (rows 1-8), 16 columns
  0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x80, 0x40, 0x20, 0x10, 0x88, 0x44, 0x28, 0x10,
  // Page 1 (rows 9-16), 16 columns  (UPDATED)
  0x02, 0x05, 0x08, 0x11, 0x22, 0x44, 0x22, 0x11,
  0x08, 0x04, 0x02, 0x01, 0x00, 0x00, 0x00, 0x00
};

// -----------------------------------------------------------------------------
// Font table (your Code 2; unchanged)
// -----------------------------------------------------------------------------
static const uint8_t asc2_0806[][6] ={
{0x00, 0x00, 0x00, 0x00, 0x00, 0x00},// sp
{0x00, 0x00, 0x00, 0x2f, 0x00, 0x00},// !
{0x00, 0x00, 0x07, 0x00, 0x07, 0x00},// "
{0x00, 0x14, 0x7f, 0x14, 0x7f, 0x14},// #
{0x00, 0x24, 0x2a, 0x7f, 0x2a, 0x12},// $
{0x40, 0x26, 0x16, 0x68, 0x64, 0x02},// %
{0x00, 0x36, 0x49, 0x55, 0x22, 0x50},// &
{0x00, 0x00, 0x05, 0x03, 0x00, 0x00},// '
{0x00, 0x00, 0x1c, 0x22, 0x41, 0x00},// (
{0x00, 0x00, 0x41, 0x22, 0x1c, 0x00},// )
{0x00, 0x14, 0x08, 0x3E, 0x08, 0x14},// *
{0x00, 0x08, 0x08, 0x3E, 0x08, 0x08},// +
{0x00, 0x00, 0x00, 0xA0, 0x60, 0x00},// ,
{0x00, 0x08, 0x08, 0x08, 0x08, 0x08},// -
{0x00, 0x00, 0x60, 0x60, 0x00, 0x00},// .
{0x00, 0x20, 0x10, 0x08, 0x04, 0x02},// /
{0x00, 0x3E, 0x51, 0x49, 0x45, 0x3E},// 0
{0x00, 0x00, 0x42, 0x7F, 0x40, 0x00},// 1
{0x00, 0x42, 0x61, 0x51, 0x49, 0x46},// 2
{0x00, 0x21, 0x41, 0x45, 0x4B, 0x31},// 3
{0x00, 0x18, 0x14, 0x12, 0x7F, 0x10},// 4
{0x00, 0x27, 0x45, 0x45, 0x45, 0x39},// 5
{0x00, 0x3C, 0x4A, 0x49, 0x49, 0x30},// 6
{0x00, 0x01, 0x71, 0x09, 0x05, 0x03},// 7
{0x00, 0x36, 0x49, 0x49, 0x49, 0x36},// 8
{0x00, 0x06, 0x49, 0x49, 0x29, 0x1E},// 9
{0x00, 0x00, 0x36, 0x36, 0x00, 0x00},// :
{0x00, 0x00, 0x56, 0x36, 0x00, 0x00},// ;
{0x00, 0x08, 0x14, 0x22, 0x41, 0x00},// <
{0x00, 0x14, 0x14, 0x14, 0x14, 0x14},// =
{0x00, 0x00, 0x41, 0x22, 0x14, 0x08},// >
{0x00, 0x02, 0x01, 0x51, 0x09, 0x06},// ?
{0x00, 0x32, 0x49, 0x59, 0x51, 0x3E},// @
{0x00, 0x7C, 0x12, 0x11, 0x12, 0x7C},// A
{0x00, 0x7F, 0x49, 0x49, 0x49, 0x36},// B
{0x00, 0x3E, 0x41, 0x41, 0x41, 0x22},// C
{0x00, 0x7F, 0x41, 0x41, 0x22, 0x1C},// D
{0x00, 0x7F, 0x49, 0x49, 0x49, 0x41},// E
{0x00, 0x7F, 0x09, 0x09, 0x09, 0x01},// F
{0x00, 0x3E, 0x41, 0x49, 0x49, 0x7A},// G
{0x00, 0x7F, 0x08, 0x08, 0x08, 0x7F},// H
{0x00, 0x00, 0x41, 0x7F, 0x41, 0x00},// I
{0x00, 0x20, 0x40, 0x41, 0x3F, 0x01},// J
{0x00, 0x7F, 0x08, 0x14, 0x22, 0x41},// K
{0x00, 0x7F, 0x40, 0x40, 0x40, 0x40},// L
{0x00, 0x7F, 0x02, 0x0C, 0x02, 0x7F},// M
{0x00, 0x7F, 0x04, 0x08, 0x10, 0x7F},// N
{0x00, 0x3E, 0x41, 0x41, 0x41, 0x3E},// O
{0x00, 0x7F, 0x09, 0x09, 0x09, 0x06},// P
{0x00, 0x3E, 0x41, 0x51, 0x21, 0x5E},// Q
{0x00, 0x7F, 0x09, 0x19, 0x29, 0x46},// R
{0x00, 0x46, 0x49, 0x49, 0x49, 0x31},// S
{0x00, 0x01, 0x01, 0x7F, 0x01, 0x01},// T
{0x00, 0x3F, 0x40, 0x40, 0x40, 0x3F},// U
{0x00, 0x1F, 0x20, 0x40, 0x20, 0x1F},// V
{0x00, 0x3F, 0x40, 0x38, 0x40, 0x3F},// W
{0x00, 0x63, 0x14, 0x08, 0x14, 0x63},// X
{0x00, 0x07, 0x08, 0x70, 0x08, 0x07},// Y
{0x00, 0x61, 0x51, 0x49, 0x45, 0x43},// Z
{0x00, 0x00, 0x7F, 0x41, 0x41, 0x00},// [
{0x00, 0x55, 0x2A, 0x55, 0x2A, 0x55},// 55
{0x00, 0x00, 0x41, 0x41, 0x7F, 0x00},// ]
{0x00, 0x04, 0x02, 0x01, 0x02, 0x04},// ^
{0x00, 0x40, 0x40, 0x40, 0x40, 0x40},// _
{0x00, 0x00, 0x01, 0x02, 0x04, 0x00},// `
{0x00, 0x20, 0x54, 0x54, 0x54, 0x78},// a
{0x00, 0x7F, 0x48, 0x44, 0x44, 0x38},// b
{0x00, 0x38, 0x44, 0x44, 0x44, 0x20},// c
{0x00, 0x38, 0x44, 0x44, 0x48, 0x7F},// d
{0x00, 0x38, 0x54, 0x54, 0x54, 0x18},// e
{0x00, 0x08, 0x7E, 0x09, 0x01, 0x02},// f
{0x00, 0x18, 0xA4, 0xA4, 0xA4, 0x7C},// g
{0x00, 0x7F, 0x08, 0x04, 0x04, 0x78},// h
{0x00, 0x00, 0x44, 0x7D, 0x40, 0x00},// i
{0x00, 0x40, 0x80, 0x84, 0x7D, 0x00},// j
{0x00, 0x7F, 0x10, 0x28, 0x44, 0x00},// k
{0x00, 0x00, 0x41, 0x7F, 0x40, 0x00},// l
{0x00, 0x7C, 0x04, 0x18, 0x04, 0x78},// m
{0x00, 0x7C, 0x08, 0x04, 0x04, 0x78},// n
{0x00, 0x38, 0x44, 0x44, 0x44, 0x38},// o
{0x00, 0xFC, 0x24, 0x24, 0x24, 0x18},// p
{0x00, 0x18, 0x24, 0x24, 0x18, 0xFC},// q
{0x00, 0x7C, 0x08, 0x04, 0x04, 0x08},// r
{0x00, 0x48, 0x54, 0x54, 0x54, 0x20},// s
{0x00, 0x04, 0x3F, 0x44, 0x40, 0x20},// t
{0x00, 0x3C, 0x40, 0x40, 0x20, 0x7C},// u
{0x00, 0x1C, 0x20, 0x40, 0x20, 0x1C},// v
{0x00, 0x3C, 0x40, 0x30, 0x40, 0x3C},// w
{0x00, 0x44, 0x28, 0x10, 0x28, 0x44},// x
{0x00, 0x1C, 0xA0, 0xA0, 0xA0, 0x7C},// y
{0x00, 0x44, 0x64, 0x54, 0x4C, 0x44},// z
{0x14, 0x14, 0x14, 0x14, 0x14, 0x14},// horiz lines
};

static void setupLED(void)
{
    P3DIR |= BIT0;
    P3OUT &= ~BIT0;
}

static void BlinkLED(void)
{
    P3OUT ^= BIT0;
}

// x_offset matches the sample code (module-specific column mapping)
static uint8_t x_offset = 0x22;

// ---------- I2C timing ----------
static void delay_short(void) { __delay_cycles(100); } // conservative

// Open-drain helpers (drive low by DIR=1+OUT=0; release by DIR=0)
static void SCL_low(void)      { OLED_PORT_OUT &= ~OLED_SCL_BIT; OLED_PORT_DIR |=  OLED_SCL_BIT; }
static void SCL_release(void)  { OLED_PORT_DIR &= ~OLED_SCL_BIT; }

static void SDA_low(void)      { OLED_PORT_OUT &= ~OLED_SDA_BIT; OLED_PORT_DIR |=  OLED_SDA_BIT; }
static void SDA_release(void)  { OLED_PORT_DIR &= ~OLED_SDA_BIT; }

static uint8_t SDA_read(void)  { return (OLED_PORT_IN & OLED_SDA_BIT) ? 1 : 0; }

// I2C bit-bang
static void i2c_idle(void)
{
    SDA_release();
    SCL_release();
    delay_short();
}

static void i2c_start(void)
{
    SDA_release();
    SCL_release();
    delay_short();

    SDA_low();
    delay_short();

    SCL_low();
    delay_short();
}

static void i2c_stop(void)
{
    SDA_low();
    delay_short();

    SCL_release();
    delay_short();

    SDA_release();
    delay_short();
}

static void i2c_write_bit(uint8_t bit)
{
    if (bit) SDA_release();
    else     SDA_low();

    delay_short();
    SCL_release();
    delay_short();
    SCL_low();
    delay_short();
}

static uint8_t i2c_read_ack(void)
{
    uint8_t ack;

    SDA_release();
    delay_short();

    SCL_release();
    delay_short();

    ack = (SDA_read() == 0);

    SCL_low();
    delay_short();

    return ack;
}

static uint8_t i2c_write_byte(uint8_t b)
{
    uint8_t i;
    for (i = 0; i < 8; i++)
    {
        i2c_write_bit((b & 0x80u) != 0);
        b <<= 1;
    }
    return i2c_read_ack();
}

// OLED low-level write helpers
static uint8_t oled_write_byte(uint8_t control, uint8_t data)
{
    uint8_t ok;

    i2c_start();

    ok = i2c_write_byte(SSD1315_ADDR_WRITE);
    if (!ok) { i2c_stop(); return 0; }

    ok = i2c_write_byte(control);
    if (!ok) { i2c_stop(); return 0; }

    ok = i2c_write_byte(data);

    i2c_stop();
    return ok;
}

static uint8_t oled_write_data_bytes(const uint8_t *data, size_t len)
{
    uint8_t ok;
    size_t i;

    i2c_start();

    ok = i2c_write_byte(SSD1315_ADDR_WRITE);
    if (!ok) { i2c_stop(); return 0; }

    ok = i2c_write_byte(OLED_CTRL_DATA);
    if (!ok) { i2c_stop(); return 0; }

    for (i = 0; i < len; i++)
    {
        ok = i2c_write_byte(data[i]);
        if (!ok) { i2c_stop(); return 0; }
    }

    i2c_stop();
    return 1;
}

static uint8_t oled_cmd(uint8_t cmd)  { return oled_write_byte(OLED_CTRL_CMD,  cmd); }
static uint8_t oled_data(uint8_t dat) { return oled_write_byte(OLED_CTRL_DATA, dat); }

// GPIO + Reset
static void oled_gpio_init(void)
{
#ifdef PM5CTL0
    PM5CTL0 &= ~LOCKLPM5; // unlock GPIO from high-Z
#endif

    OLED_PORT_OUT &= ~(OLED_SCL_BIT | OLED_SDA_BIT | OLED_RES_BIT);

    // RES push-pull output
    OLED_PORT_DIR |= OLED_RES_BIT;

#if USE_INTERNAL_PULLUPS
    OLED_PORT_REN |=  (OLED_SCL_BIT | OLED_SDA_BIT);
    OLED_PORT_OUT |=  (OLED_SCL_BIT | OLED_SDA_BIT);
#else
    OLED_PORT_REN &= ~(OLED_SCL_BIT | OLED_SDA_BIT);
#endif

    OLED_PORT_OUT |= OLED_RES_BIT; // RES high
    i2c_idle();
}

static void oled_reset_pulse(void)
{
    OLED_PORT_OUT &= ~OLED_RES_BIT;
    delay_ms(50);
    OLED_PORT_OUT |= OLED_RES_BIT;
    delay_ms(50);
}

// High-level API
void OLED_WR_Byte(uint8_t dat, uint8_t mode)
{
    if (mode == OLED_DATA) (void)oled_data(dat);
    else                  (void)oled_cmd(dat);
}

void OLED_Set_Pos(uint8_t x, uint8_t y)
{
    uint8_t col = (uint8_t)(x + x_offset);
    OLED_WR_Byte((uint8_t)(0xB0 + y), OLED_CMD);
    OLED_WR_Byte((uint8_t)(0x10 | (col >> 4)), OLED_CMD);
    OLED_WR_Byte((uint8_t)(col & 0x0F), OLED_CMD);
}

void OLED_Clear(void)
{
    static const uint8_t zeros[16] = {0};
    uint8_t page;
    uint8_t n;

    for (page = 0; page < 4; page++)
    {
        OLED_WR_Byte((uint8_t)(0xB0 + page), OLED_CMD);
        {
            uint8_t col = x_offset;
            OLED_WR_Byte((uint8_t)(col & 0x0F), OLED_CMD);
            OLED_WR_Byte((uint8_t)(0x10 | (col >> 4)), OLED_CMD);
        }

        for (n = 0; n < 8; n++)
        {
            (void)oled_write_data_bytes(zeros, sizeof(zeros));
        }
    }
}

void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t sizey)
{
    uint8_t c;
    uint16_t i;
    uint16_t size1;
    uint8_t sizex;

    c = 0;
    sizex = (uint8_t)(sizey / 2);

    if (sizey == 8) size1 = 6;
    else size1 = (uint16_t)((sizey / 8 + ((sizey % 8) ? 1 : 0)) * (sizey / 2));

    c = (uint8_t)(chr - ' ');
    OLED_Set_Pos(x, y);

    for (i = 0; i < size1; i++)
    {
        if ((i % sizex) == 0 && sizey != 8) OLED_Set_Pos(x, y++);
        if (sizey == 8) OLED_WR_Byte(asc2_0806[c][i], OLED_DATA);
        else return;
    }
}

void OLED_ShowString(uint8_t x, uint8_t y, const uint8_t *chr, uint8_t sizey)
{
    uint8_t j = 0;
    while (chr[j] != '\0')
    {
        OLED_ShowChar(x, y, chr[j++], sizey);
        if (sizey == 8) x = (uint8_t)(x + 6);
        else x = (uint8_t)(x + sizey / 2);
    }
}

void OLED_ShowStringCentered(uint8_t y, const uint8_t *chr, uint8_t sizey)
{
    uint8_t len = 0;
    while (chr[len] != '\0') len++;

    if (sizey == 8)
        OLED_ShowString((uint8_t)((60u - (uint8_t)(len * 6u)) / 2u), y, chr, sizey);
    else
        OLED_ShowString((uint8_t)((60u - (uint8_t)(len * 16u)) / 2u), y, chr, sizey);
}


void OLED_ShowStringCenteredLeft(uint8_t y, const uint8_t *chr, uint8_t sizey)
{
    uint8_t len = 0;
    while (chr[len] != '\0') len++;

    if (sizey == 8)
        OLED_ShowString((uint8_t)((26u - (uint8_t)(len * 6u)) / 2u), y, chr, sizey);
    else
        OLED_ShowString((uint8_t)((26u - (uint8_t)(len * 16u)) / 2u), y, chr, sizey);
}

void OLED_DrawBMP(uint8_t x, uint8_t y, uint8_t sizex, uint8_t sizey, const uint8_t *BMP)
{
    uint16_t j = 0;
    uint8_t i, m;
    uint8_t pages;

    pages = (uint8_t)(sizey / 8 + ((sizey % 8) ? 1 : 0));
    for (i = 0; i < pages; i++)
    {
        OLED_Set_Pos(x, (uint8_t)(i + y));
        for (m = 0; m < sizex; m++)
        {
            OLED_WR_Byte(BMP[j++], OLED_DATA);
        }
    }
}

void OLED_Init_DCDC(void)
{
    oled_gpio_init();
    delay_ms(3000); // Vbat/Vcc settle
    oled_reset_pulse();

    OLED_WR_Byte(0xAE, OLED_CMD); // display off
    OLED_WR_Byte(0xD5, OLED_CMD); // clock divide
    OLED_WR_Byte(0x80, OLED_CMD);
    OLED_WR_Byte(0xA8, OLED_CMD); // multiplex
    OLED_WR_Byte(0x1F, OLED_CMD); // 1/32 duty
    OLED_WR_Byte(0xD3, OLED_CMD); // display offset
    OLED_WR_Byte(0x00, OLED_CMD);
    OLED_WR_Byte(0x00, OLED_CMD); // start line

    //OLED_WR_Byte(0xA1, OLED_CMD); // seg remap left/right
    OLED_WR_Byte(0xC8, OLED_CMD); // com scan dir
    OLED_WR_Byte(0xDA, OLED_CMD); // com pins
    OLED_WR_Byte(0x12, OLED_CMD);
    OLED_WR_Byte(0x81, OLED_CMD); // contrast
    OLED_WR_Byte(0x18, OLED_CMD);
    OLED_WR_Byte(0xD9, OLED_CMD); // pre-charge
    OLED_WR_Byte(0xF1, OLED_CMD);
    OLED_WR_Byte(0xDB, OLED_CMD); // vcomh
    OLED_WR_Byte(0x40, OLED_CMD);
    OLED_WR_Byte(0xA4, OLED_CMD); // resume RAM

    OLED_Clear();

    OLED_WR_Byte(0x8D, OLED_CMD); // charge pump
    OLED_WR_Byte(0x14, OLED_CMD); // enable
    OLED_WR_Byte(0xAF, OLED_CMD); // display on

    delay_ms(150);
}

// ===================== PACKET -> OLED STRING HELPERS =====================
static void unpack_two_digits(uint8_t b, uint8_t *tensChar, uint8_t *onesChar)
{
    uint8_t tens = (uint8_t)(b / 10u);
    uint8_t ones = (uint8_t)(b % 10u);

    if (tens > 9u) tens = 0u;
    if (ones > 9u) ones = 0u;

    *tensChar = (uint8_t)('0' + tens);
    *onesChar = (uint8_t)('0' + ones);
}

static int utoa_no_leading_zeros(unsigned int v, uint8_t *out)
{
    uint8_t tmp[5];
    int n = 0;
    int i;

    if (v == 0u) { out[0] = '0'; return 1; }

    while (v > 0u && n < 4)
    {
        tmp[n++] = (uint8_t)('0' + (v % 10u));
        v /= 10u;
    }
    for (i = 0; i < n; i++)
        out[i] = tmp[n - 1 - i];

    return n;
}

static void build_distance_string(uint8_t dist1, uint8_t dist2, uint8_t unit01, uint8_t *out)
{
    int idx = 0;

    if (unit01 == 0u)
    {
        uint8_t d0, d1, d2, d3;
        uint8_t digits[4];
        int started = 0;
        int i;

        unpack_two_digits(dist1, &d0, &d1);
        unpack_two_digits(dist2, &d2, &d3);

        digits[0] = d0; digits[1] = d1; digits[2] = d2; digits[3] = d3;

        for (i = 0; i < 4; i++)
        {
            if (!started)
            {
                if (digits[i] != '0' || i == 3) started = 1;
                else continue;
            }
            out[idx++] = digits[i];
        }

        //out[idx++] = ' ';
        out[idx++] = 'm';
        out[idx]   = '\0';
        return;
    }
    else if (unit01 == 1u)
    {
        uint8_t tenths;
        uint8_t hundredths;

        idx += utoa_no_leading_zeros(dist1, &out[idx]);

        if (dist2 < 10u)
        {
            tenths = dist2;
            hundredths = 0u;
        }
        else
        {
            tenths = (uint8_t)(dist2 / 10u);
            hundredths = (uint8_t)(dist2 % 10u);
        }

        if (!(tenths == 0u && hundredths == 0u))
        {
            out[idx++] = '.';
            out[idx++] = (uint8_t)('0' + (tenths % 10u));
            if (hundredths != 0u)
                out[idx++] = (uint8_t)('0' + (hundredths % 10u));
        }

        //out[idx++] = ' ';
        out[idx++] = 'k';
        out[idx++] = 'm';
        out[idx]   = '\0';
        return;
    }
    else
    {
        out[0] = '?';
        out[1] = '\0';
    }
}

static void build_street_string(const uint8_t *streetBytes8, uint8_t *out)
{
    int idx = 0;
    int i;

    for (i = 0; i < 8; i++)
        out[idx++] = streetBytes8[i];

    i = 7;
    while (i >= 0)
    {
        if (out[i] == ' ')
        {
            i--;
            continue;
        }
        out[i + 1] = '\0';
        return;
    }
    out[0] = '\0';
}

static void oled_update_from_packet14(const uint8_t *pkt)
{
    uint8_t dir;
    uint8_t dist1, dist2, unit;
    uint8_t streetBytes[8];
    uint8_t special;
    uint8_t distanceString[16];
    uint8_t streetString[9];
    uint8_t i;

    if (pkt[0] != PKT_START_BYTE) return;

    dir   = pkt[1];
    dist1 = pkt[2];
    dist2 = pkt[3];
    unit  = pkt[4];

    for (i = 0; i < 8; i++) streetBytes[i] = pkt[5 + i];

    special = pkt[13];
    (void)special;

    build_distance_string(dist1, dist2, unit, distanceString);
    build_street_string(streetBytes, streetString);

    OLED_Clear();

    if(special == 1)
    {
        //OLED_DrawBMP(22,0,16,16, checkmark_16x16);
        OLED_DrawBMP(6,1,16,16, checkmark_16x16);
        return;
    }

    if (dir == 1u)
        //OLED_DrawBMP(24, 0, 12, 16, arrow_head_left_12x16);
        OLED_DrawBMP(6, 0, 12, 16, arrow_head_left_12x16);
    else
        //OLED_DrawBMP(24, 0, 12, 16, arrow_head_right_12x16);
        OLED_DrawBMP(6, 0, 12, 16, arrow_head_right_12x16);

    //OLED_ShowStringCentered(2, distanceString, 8);
    //OLED_ShowStringCentered(3, streetString, 8);
    OLED_ShowStringCenteredLeft(2, distanceString, 8);
    OLED_ShowStringCenteredLeft(3, streetString, 8);
}

// =====================================================================================
// ===================== END OLED =======================================================
// =====================================================================================


// ===================== PC (UCA1) BLOCKING TX =====================
static void pc_putc(uint8_t c)
{
    while(!(UCA1IFG & UCTXIFG)) {}
    UCA1TXBUF = c;
}

static void pc_send_bytes_blocking(const uint8_t *buf, uint8_t n)
{
    uint8_t i;
    for (i = 0; i < n; i++) pc_putc(buf[i]);
}

// ===================== TXQ HELPERS (BLE TX on UCA0) =====================
static inline uint8_t txq_count(void) { return (uint8_t)(txq_w - txq_r); }

static inline uint8_t txq_push(uint8_t b)
{
    if (txq_count() >= TXQ_SIZE) { txq_drop++; return 0; }
    txq[txq_w & (TXQ_SIZE - 1u)] = b;
    txq_w++;
    return 1;
}

static inline uint8_t txq_pop(uint8_t *b)
{
    if (txq_count() == 0) return 0;
    *b = txq[txq_r & (TXQ_SIZE - 1u)];
    txq_r++;
    return 1;
}

static void txq_push_str(const char *s)
{
    while (*s) (void)txq_push((uint8_t)(*s++));
}

static inline void kick_ble_tx(void)
{
    UCA0IE  |= UCTXIE;
    UCA0IFG |= UCTXIFG;
}

// ===================== CAP RING (BLE->MSP) =====================
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

// ===================== ACCEPTED FIFO =====================
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

// ===================== PACKET VALIDATION =====================
static inline uint8_t validate_packet14(const uint8_t *pkt14)
{
    (void)pkt14;
    return 1;
}

// ===================== PARSING (main loop) =====================
static void parse_cap_into_accepted(void)
{
    while (1) {
        uint16_t cnt;
        uint8_t b;

        cnt = cap_count_snapshot();
        if (cnt == 0) return;

        if (!cap_peek0(&b)) return;
        if (b != PKT_START_BYTE) {
            (void)cap_pop(&b);
            continue;
        }

        if (cnt < PKT_SIZE) return;

        {
            uint8_t pkt[PKT_SIZE];
            uint8_t i;

            for (i = 0; i < PKT_SIZE; i++) {
                (void)cap_pop(&pkt[i]);
            }

            if (validate_packet14(pkt)) {
                __disable_interrupt();
                (void)accepted_push_packet(pkt);
                __enable_interrupt();
            }
        }
    }
}

// ===================== CLEAR =====================
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

// ===================== RN RESET =====================
static void rn_reset_init_pin(void)
{
    RN_RST_PORT_SEL0 &= ~RN_RST_BIT;
    RN_RST_PORT_SEL1 &= ~RN_RST_BIT;

    RN_RST_PORT_DIR |= RN_RST_BIT;
    RN_RST_PORT_OUT |= RN_RST_BIT; // idle high
}

static void rn_reset_pulse(void)
{
    RN_RST_PORT_OUT &= ~RN_RST_BIT;
    delay_ms(50);
    RN_RST_PORT_OUT |= RN_RST_BIT;
}

static void delay_ms(uint16_t ms)
{
    while (ms--) __delay_cycles(8000); // 1 ms at 8 MHz
}

static void rn_auto_setup_sequence(void)
{
    rn_reset_pulse();
    delay_ms(600);

    delay_ms(250);
    txq_push('$'); txq_push('$'); txq_push('$');
    kick_ble_tx();
    delay_ms(350);

    txq_push_str("SS,C0\r");
    kick_ble_tx();
    delay_ms(200);

    txq_push_str("ST,0010,0030,0000,0C80\r");
    kick_ble_tx();
    delay_ms(200);

    txq_push_str("STA,0020,0010,00A0\r");
    kick_ble_tx();
    delay_ms(200);

    txq_push_str("JC\r");
    kick_ble_tx();
    delay_ms(150);

    txq_push_str("+\r");
    kick_ble_tx();
    delay_ms(150);

    txq_push_str("R,1\r");
    kick_ble_tx();
    delay_ms(1400);

    txq_push_str("A\r");
    kick_ble_tx();
    delay_ms(100);
}

// ===================== MAIN =====================
int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;

    initClock();
    initUART_PC();   // UCA1
    initUART_BLE();  // UCA0
    rn_reset_init_pin();
    setupLED();

    OLED_Init_DCDC();
    OLED_ShowStringCentered(0, (const uint8_t*)"Welcome", 8);
    OLED_ShowStringCentered(1, (const uint8_t*)"to", 8);
    OLED_ShowStringCentered(2, (const uint8_t*)"EYEWHERE", 8);

    clear_all();
    __enable_interrupt();

    rn_auto_setup_sequence();

    while (1)
    {
        if (do_clear) {
            do_clear = 0;
            clear_all();
            OLED_Clear();
        }

        if (do_hwreset) {
            do_hwreset = 0;
            rn_reset_pulse();
        }

        if (do_autosetup) {
            do_autosetup = 0;
            rn_auto_setup_sequence();
        }

        parse_cap_into_accepted();

        if (accepted_count() != 0) {
            //BlinkLED();
            uint8_t pkt[PKT_SIZE];

            __disable_interrupt();
            (void)accepted_pop_packet(pkt);
            __enable_interrupt();

            oled_update_from_packet14(pkt);
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
    CSCTL2 = SELM__DCOCLK | SELS__DCOCLK | SELA__DCOCLK;
    CSCTL3 = DIVS__1;
    CSCTL0_H = 0x01;
}

// PC = UCA1 (pins already remapped by you; keep the P2SEL settings you used)
static void initUART_PC(void)
{
    // PC pins (as you had): P2.5=UCA1TXD, P2.6=UCA1RXD
    P2SEL0 &= ~(BIT5 | BIT6);
    P2SEL1 |=  (BIT5 | BIT6);

    UCA1CTLW0 = UCSWRST | UCSSEL__SMCLK;
    UCA1BRW   = PC_BAUD_9600_BRW;
    UCA1MCTLW = PC_BAUD_9600_MCTL;
    UCA1CTLW0 &= ~UCSWRST;

    UCA1IE |= UCRXIE;
}

// BLE = UCA0 (pins already remapped by you; keep the P2SEL settings you used)
static void initUART_BLE(void)
{
    // BLE pins (as you had): P2.0=UCA0TXD, P2.1=UCA0RXD
    P2SEL0 &= ~(BIT0 | BIT1);
    P2SEL1 |=  (BIT0 | BIT1);

    UCA0CTLW0 = UCSWRST | UCSSEL__SMCLK;
    UCA0BRW   = BLE_BAUD_9600_BRW;
    UCA0MCTLW = BLE_BAUD_9600_MCTL;
    UCA0CTLW0 &= ~UCPEN;
    UCA0CTLW0 &= ~UCSWRST;

    UCA0IE |= UCRXIE;
}

// ===================== ISRs =====================

// PC ISR = UCA1
#pragma vector = USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
{
    switch(__even_in_range(UCA1IV, 4))
    {
        case 2: // RXIFG (PC -> commands)
        {
            uint8_t b = UCA1RXBUF;

            if (UCA1STATW & UCOE) { uca1_ovr++; UCA1STATW &= ~UCOE; }

            // debug commands from PC:
            // 0x01 (CMD mode), 0x0E (autosetup), 0x0D (HW RST), 0x04 (clear)
            if (b == 0x01) {
                txq_push('$'); txq_push('$'); txq_push('$');
                kick_ble_tx();
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

// BLE ISR = UCA0 (RX capture + TX drain)
#pragma vector = USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
{
    switch(__even_in_range(UCA0IV, 4))
    {
        case 2: // RXIFG (BLE -> cap ring)
        {
            uint8_t b = UCA0RXBUF;

            if (UCA0STATW & UCOE) { uca0_ovr++; UCA0STATW &= ~UCOE; }

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
                UCA0TXBUF = out;
            } else {
                UCA0IE &= ~UCTXIE;
            }
            break;
        }

        default: break;
    }
}
