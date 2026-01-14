#include <msp430.h>
#include <stdint.h>

// ---------------- Pin mapping ----------------
#define OLED_SCL_BIT   BIT1   // P2.1
#define OLED_SDA_BIT   BIT2   // P2.2
#define OLED_RES_BIT   BIT0   // P2.0

#define OLED_PORT_DIR  P2DIR
#define OLED_PORT_OUT  P2OUT
#define OLED_PORT_IN   P2IN
#define OLED_PORT_REN  P2REN

// Using external 10k pull-ups already
#define USE_INTERNAL_PULLUPS 0

// SSD1315 address (8-bit write form) from STC example (7-bit 0x3C -> 0x78 write)
#define SSD1315_ADDR_WRITE  0x78

// Control bytes
#define OLED_CTRL_CMD   0x00
#define OLED_CTRL_DATA  0x40

// ---- Choose power mode ----
// 1 = enable charge pump in code (recommended for first light)
// 0 = assume external VCC and pause to switch VCC on manually
#define USE_CHARGE_PUMP 0

// ---------------- Simple delay helpers ----------------
// DCO ~5.33 MHz => 5330 cycles/ms approximately
// We'll just approximate with constant-cycle blocks.
// These are NOT precise; they are "safe & slow" for bring-up.

static void delay_cycles_const(uint16_t nBlocks)
{
    // each block ~ 1000 cycles
    while (nBlocks--)
        __delay_cycles(1000);
}

static void delay_ms(uint16_t ms)
{
    // ~5 blocks per ms @5.33MHz (5000 cycles)
    while (ms--)
        delay_cycles_const(5);
}

static void delay_s(uint16_t s)
{
    while (s--)
        delay_ms(1000);
}

// I2C edge timing: ~10 us-ish (very conservative / slow)
static void delay_short(void)
{
    // 10 us @5.33MHz ~53 cycles. Use 100 cycles to be safe/visible-ish.
    __delay_cycles(100);
}

// ---------------- Open-drain helpers ----------------
static void SCL_low(void)
{
    OLED_PORT_OUT &= ~OLED_SCL_BIT;
    OLED_PORT_DIR |=  OLED_SCL_BIT;
}

static void SCL_release(void)
{
    OLED_PORT_DIR &= ~OLED_SCL_BIT;
}

static void SDA_low(void)
{
    OLED_PORT_OUT &= ~OLED_SDA_BIT;
    OLED_PORT_DIR |=  OLED_SDA_BIT;
}

static void SDA_release(void)
{
    OLED_PORT_DIR &= ~OLED_SDA_BIT;
}

static uint8_t SDA_read(void)
{
    return (OLED_PORT_IN & OLED_SDA_BIT) ? 1 : 0;
}

// ---------------- I2C bit-bang ----------------
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

    SDA_release();   // release so OLED can pull low
    delay_short();

    SCL_release();   // clock high for ACK bit
    delay_short();

    ack = (SDA_read() == 0); // ACK = SDA low

    SCL_low();
    delay_short();

    return ack;
}

static uint8_t i2c_write_byte(uint8_t b)
{
    uint8_t i;
    for (i = 0; i < 8; i++)
    {
        i2c_write_bit((b & 0x80) != 0);
        b <<= 1;
    }
    return i2c_read_ack();
}

// ---------------- OLED primitives ----------------
static void oled_gpio_init(void)
{
#ifdef PM5CTL0
    PM5CTL0 &= ~LOCKLPM5; // unlock GPIO from high-Z
#endif

    // Prepare outputs low
    OLED_PORT_OUT &= ~(OLED_SCL_BIT | OLED_SDA_BIT | OLED_RES_BIT);

    // RES push-pull output
    OLED_PORT_DIR |= OLED_RES_BIT;

#if USE_INTERNAL_PULLUPS
    OLED_PORT_REN |=  (OLED_SCL_BIT | OLED_SDA_BIT);
    OLED_PORT_OUT |=  (OLED_SCL_BIT | OLED_SDA_BIT); // pull-up
#else
    OLED_PORT_REN &= ~(OLED_SCL_BIT | OLED_SDA_BIT);
#endif

    // RES high
    OLED_PORT_OUT |= OLED_RES_BIT;

    // I2C idle released
    i2c_idle();
}

static void oled_reset_pulse(void)
{
    OLED_PORT_OUT &= ~OLED_RES_BIT; // RES low
    delay_ms(50);
    OLED_PORT_OUT |= OLED_RES_BIT;  // RES high
    delay_ms(50);
}

static uint8_t oled_write(uint8_t control, uint8_t data)
{
    uint8_t ok;

    i2c_start();

    ok = i2c_write_byte(SSD1315_ADDR_WRITE);
    if (!ok) { i2c_stop(); return 0; }

    ok = i2c_write_byte(control);
    if (!ok) { i2c_stop(); return 0; }

    ok = i2c_write_byte(data);

    if (ok) {
        P3OUT |= BIT4; // turn on LED if ack
    } else {
        P3OUT &= !BIT4; 
    }

    i2c_stop();
    return ok;
}

static uint8_t oled_cmd(uint8_t cmd)
{
    return oled_write(OLED_CTRL_CMD, cmd);
}

// Minimal bring-up: prove the panel responds
static void oled_init_minimal(void)
{
#if USE_CHARGE_PUMP
    // Charge pump enable sequence (matches typical SSD13xx style)
    oled_cmd(0x8D);      // Charge Pump Setting
    oled_cmd(0x14);      // Enable IGNORE
    delay_ms(100);
#else
    // External VCC: pause so you can turn it ON now (after reset)
    delay_s(8);
#endif

    // Display ON
    oled_cmd(0xAF);
    delay_ms(150);
}

// ---------------- Clock setup ----------------
static void clock_init_dco_default(void)
{
    CSCTL0_H = CSKEY_H;
    //CSCTL1 |= DCORSEL;
    CSCTL1 = DCOFSEL_0;  // ~5.33 MHz on MSP430FR5739
    //CSCTL1 = DCOFSEL0 + DCOFSEL1; // 8 MHz
    CSCTL2 = SELA__VLOCLK | SELS__DCOCLK | SELM__DCOCLK;
    CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1;
    CSCTL0_H = 0;
}

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;

    P3DIR |= BIT4; // LED to check ack
    P3OUT &= !BIT4; 


    clock_init_dco_default();
    oled_gpio_init();

    delay_ms(200);

    oled_reset_pulse();
    oled_init_minimal();
    oled_cmd(0xA5);

    while (1)
    {
        // Toggle between "all pixels on" and "normal display"
        //oled_cmd(0xA5);  // All pixels ON
        //delay_ms(5);

        //oled_cmd(0xA4);  // Resume RAM display
        //delay_ms(5);
    }
}
