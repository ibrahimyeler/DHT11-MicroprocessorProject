#include <xc.h>
#include <stdint.h>

#define _XTAL_FREQ 4000000  // 4 MHz crystal

// CONFIG
#pragma config FOSC = HS     // High Speed Oscillator
#pragma config WDTE = OFF    // Watchdog Disabled
#pragma config PWRTE = ON    // Power-up Timer Enabled
#pragma config BOREN = ON    // Brown-out Reset Enabled
#pragma config LVP = OFF     // Low Voltage Programming Disabled
#pragma config CPD = OFF     // Data EE Memory Code Protection Off
#pragma config WRT = OFF     // Flash Program Memory Write Enable Off
#pragma config CP = OFF      // Flash Program Memory Code Protection Off

// LCD Settings
#define LCD_ADDR 0x27        // Common I2C addresses: 0x27 or 0x3F
#define LCD_BACKLIGHT 0x08   // Backlight control bit

// I2C Functions
void I2C_Init(void);
void I2C_Start(void);
void I2C_Stop(void);
void I2C_Write(uint8_t data);
uint8_t I2C_Check_Address(uint8_t address);

// LCD Functions
void LCD_Send_Nibble(uint8_t nibble, uint8_t rs);
void LCD_Send_Byte(uint8_t data, uint8_t rs);
void LCD_Cmd(uint8_t cmd);
void LCD_Write_Char(char c);
void LCD_Write_String(const char *str);
void LCD_Init(void);
void LCD_Set_Cursor(uint8_t row, uint8_t col);
void LCD_Clear(void);

void main(void) {
    // Disable analog functions
    ADCON1 = 0x06;  // Set all PORTA pins as digital
    
    // Initialize I2C
    I2C_Init();
    
    // Check if LCD is present
    if(!I2C_Check_Address(LCD_ADDR)) {
        // Flash LED on PORTD if LCD not found
        TRISD = 0x00;
        while(1) {
            PORTD = 0xFF;
            __delay_ms(500);
            PORTD = 0x00;
            __delay_ms(500);
        }
    }
    
    // Initialize LCD
    LCD_Init();
    
    // Display messages
    LCD_Set_Cursor(1, 1);
    LCD_Write_String("Hello World!");
    LCD_Set_Cursor(2, 1);
    LCD_Write_String("PIC16F877A I2C");

    while(1) {
        // Optional: Blink backlight to show it's working
        LCD_Cmd(0x08);  // Display off, backlight off
        __delay_ms(100);
        LCD_Cmd(0x0C);  // Display on, backlight on
        __delay_ms(1000);
    }
}

// I2C Initialization
void I2C_Init(void) {
    TRISC3 = 1;  // SCL as input
    TRISC4 = 1;  // SDA as input
    SSPCON = 0x28;   // I2C Master mode
    SSPCON2 = 0x00;
    SSPADD = 9;      // 100kHz clock with 4MHz OSC: (4e6/(4*100000))-1 = 9
    SSPSTAT = 0x00;
}

// Check if device exists at address
uint8_t I2C_Check_Address(uint8_t address) {
    I2C_Start();
    SSPBUF = (address << 1);  // Write operation
    while(SSPSTATbits.BF);    // Wait for transmission
    I2C_Stop();
    return !SSPCON2bits.ACKSTAT;  // 0 if NACK (not found), 1 if ACK (found)
}

// I2C Start condition
void I2C_Start(void) {
    SEN = 1;
    while(SEN);
}

// I2C Stop condition
void I2C_Stop(void) {
    PEN = 1;
    while(PEN);
}

// I2C Write byte
void I2C_Write(uint8_t data) {
    SSPBUF = data;
    while(SSPSTATbits.BF);  // Wait for transmission to complete
    while(SSPCON2bits.ACKSTAT);  // Wait for ACK
}

// Send 4-bit nibble to LCD
void LCD_Send_Nibble(uint8_t nibble, uint8_t rs) {
    // Data format: [D7 D6 D5 D4 BL EN RW RS]
    uint8_t data = nibble | LCD_BACKLIGHT | (rs ? 0x01 : 0x00);
    
    // Send with EN high
    I2C_Write(data | 0x04);  // Set EN high
    __delay_us(1);
    I2C_Write(data & ~0x04); // Set EN low
    __delay_us(50);          // Command needs >37us to settle
}

// Send full byte to LCD (as two nibbles)
void LCD_Send_Byte(uint8_t data, uint8_t rs) {
    LCD_Send_Nibble(data >> 4, rs);  // High nibble
    LCD_Send_Nibble(data & 0x0F, rs); // Low nibble
}

// Send command to LCD
void LCD_Cmd(uint8_t cmd) {
    LCD_Send_Byte(cmd, 0);
    if(cmd == 0x01 || cmd == 0x02) __delay_ms(2); // Extra delay for clear/home
}

// Write character to LCD
void LCD_Write_Char(char c) {
    LCD_Send_Byte(c, 1);
}

// Write string to LCD
void LCD_Write_String(const char *str) {
    while(*str) LCD_Write_Char(*str++);
}

// Set cursor position
void LCD_Set_Cursor(uint8_t row, uint8_t col) {
    col--;
    uint8_t address = (row == 1) ? 0x80 : 0xC0;
    LCD_Cmd(address + col);
}

// Clear LCD display
void LCD_Clear(void) {
    LCD_Cmd(0x01);
    __delay_ms(2);
}

// Initialize LCD
void LCD_Init(void) {
    __delay_ms(50);  // Wait for LCD to power up
    
    // Start I2C communication
    I2C_Start();
    I2C_Write(LCD_ADDR << 1);  // Send address with write bit
    
    // Initialization sequence
    LCD_Send_Nibble(0x03, 0);  // Try to set 8-bit mode
    __delay_ms(5);
    LCD_Send_Nibble(0x03, 0);
    __delay_us(150);
    LCD_Send_Nibble(0x03, 0);
    LCD_Send_Nibble(0x02, 0);  // Set to 4-bit mode
    
    // Function set: 4-bit, 2 lines, 5x8 font
    LCD_Cmd(0x28);
    __delay_us(50);
    
    // Display control: Display on, cursor off, blink off
    LCD_Cmd(0x0C);
    __delay_us(50);
    
    // Entry mode set: Increment cursor, no display shift
    LCD_Cmd(0x06);
    __delay_us(50);
    
    // Clear display
    LCD_Clear();
    
    I2C_Stop();
}