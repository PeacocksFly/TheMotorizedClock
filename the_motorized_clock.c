/*
 * File:   the_motorized_clock.c
 * Author: Quentin
 *
 * Created on 20 February 2020, 18:20
 */

// PIC18F45K22 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1H
#pragma config FOSC = HSMP      // Oscillator Selection bits (Internal oscillator block)
#pragma config PLLCFG = OFF     // 4X PLL Enable (Oscillator used directly)
#pragma config PRICLKEN = ON    // Primary clock enable bit (Primary clock is always enabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRTEN = OFF     // Power-up Timer Enable bit (Power up timer disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 190       // Brown Out Reset Voltage bits (VBOR set to 1.90 V nominal)

// CONFIG2H
#pragma config WDTEN = OFF      // Watchdog Timer Enable bits (Watch dog timer is always disabled. SWDTEN has no effect.)
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC1  // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<5:0> pins are configured as analog input channels on Reset)
#pragma config CCP3MX = PORTB5  // P3A/CCP3 Mux bit (P3A/CCP3 input/output is multiplexed with RB5)
#pragma config HFOFST = ON      // HFINTOSC Fast Start-up (HFINTOSC output and ready status are not delayed by the oscillator stable status)
#pragma config T3CMX = PORTC0   // Timer3 Clock input mux bit (T3CKI is on RC0)
#pragma config P2BMX = PORTD2   // ECCP2 B output mux bit (P2B is on RD2)
#pragma config MCLRE = EXTMCLR  // MCLR Pin Enable bit (MCLR pin enabled, RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection Block 0 (Block 0 (000800-001FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection Block 1 (Block 1 (002000-003FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection Block 2 (Block 2 (004000-005FFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection Block 3 (Block 3 (006000-007FFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection Block 0 (Block 0 (000800-001FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection Block 1 (Block 1 (002000-003FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection Block 2 (Block 2 (004000-005FFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection Block 3 (Block 3 (006000-007FFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection Block 0 (Block 0 (000800-001FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection Block 1 (Block 1 (002000-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection Block 2 (Block 2 (004000-005FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection Block 3 (Block 3 (006000-007FFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>
#include <pic18f45k22.h>
#define _XTAL_FREQ 8000000

#define EN LATEbits.LATE2
#define RW LATEbits.LATE1
#define RS LATEbits.LATE0

#define SLAVE_ADR_WRITE 0xD0
#define SLAVE_ADR_READ 0xD1

#define CLK_SEC_ADR 0x00                 //second register address for DS3231 
#define CLK_MIN_ADR 0x01                 //minute register address for DS3231
#define CLK_HR_ADR 0x02                  //hour register address for DS3231
#define CTRL_REG_ADR 0x0E                //control register address for DS3231
#define STATUS_REG_ADR 0x0F              //status register address for DS3231

#define NOT_LAST_BYTE 0
#define LAST_BYTE 1

typedef enum{   
    HR_SETTING = 0,
    MIN_SETTING = 1,
    SEC_SETTING = 2,   
    NOT_DEFINED = 3,
    NOT_SETTING = 4,
}clk_state;

typedef enum{   
    HR_POS = 1,
    MIN_POS = 4,
    SEC_POS = 7,   
}cursor_pos;

//LCD Functions
void stringToUSB(uint8_t*);
void charToUSB(uint8_t);
void LCDCommand(uint8_t);
void LCDData(uint8_t);
void writeToLCD(uint8_t* p, uint8_t);

//I2C Functions
void startI2C(void);
void repeatStartI2C(void);
void writeInitI2C(void);
void readInitI2C(void);
void stopI2C(void);
void writeByteI2C(uint8_t);
void readByteI2C(uint8_t*, uint8_t);

//RTC Clock Functions
void writeToRTC(uint8_t*, uint8_t, uint8_t);
void writeSingleByteToRTC(uint8_t*);
void writeMultipleByteToRTC(uint8_t*, uint8_t);
void readFromRTC(uint8_t*, uint8_t, uint8_t);
void readSingleByteFromRTC(uint8_t*);
void readMultipleByteFromRTC(uint8_t*, uint8_t);
void requestTime(void);
void displayTime(void);
void displayFormatting(uint8_t*);

//Clock Display Functions
void moveCursor(uint8_t);
void displayCursor(void);
void removeCursor(void);
void digitUp(void);
void digitDown(void);

uint8_t time[4]={0x00, 0x00, 0x00, '\0'};
uint8_t display[9]={0x00, 0x00, 0x3A, 0x00, 0x00, 0x3A, 0x00, 0x00, '\0'};
uint8_t clk_reg_adr[3]={CLK_HR_ADR, CLK_MIN_ADR, CLK_SEC_ADR};
uint8_t cursor_positions[3]={HR_POS, MIN_POS, SEC_POS};


uint8_t low_nibble __at(0x050);
uint8_t high_nibble __at(0x060);

clk_state state = NOT_SETTING;
        
void main(void) {
    
    //Oscillator configuration
    OSCTUNEbits.PLLEN = 0;
    OSCCONbits.SCS = 0b00;
            
    //EUSART Reception Control Register Configuration
    TRISCbits.TRISC6 = 1;              //RC6 as an output - TX
    TRISCbits.TRISC7 = 1;              //RC7 as an input  - RX
    ANSELCbits.ANSC6 = 0;              //digital enabled
    ANSELCbits.ANSC7 = 0;              //digital enabled
    
    RCSTA1bits.SPEN = 1;               //serial port enabled
    RCSTA1bits.RX9 = 0;                //8-bits reception
    RCSTA1bits.CREN = 1;               //receiver enabled
       
    //EUSART Transmit Control Register Configuration
    TXSTA1bits.SYNC = 0;               //asynchronous mode
    TXSTA1bits.TXEN = 1;               //transmitter enabled
    TXSTA1bits.TX9 = 0;                //8-bits transmission
    TXSTA1bits.BRGH = 0;               //low speed baud rate selected 
    BAUDCON1bits.BRG16 = 0;            //8-bit baud rate generator
    SPBRG1 = 12;                       //baud rate = 9600
    
    //LCD Configuration
    TRISD = 0;                         //PORTD as an output
    TRISE = 0;                         //PORTE as an output
    
    EN = 0;  
    __delay_ms(100);
    LCDCommand(0x38);                  //LCD 2 lines, 5x7 matrix
    __delay_ms(100);
    LCDCommand(0x0C);                  //display on, cursor off
    __delay_ms(5);
       
    writeToLCD((uint8_t*)"The Motor Clock ", 0x80);
    
    //I2C Configuration
    ANSELCbits.ANSC3 = 0;              //digital enabled
    ANSELCbits.ANSC4 = 0;              //digital enabled
    TRISCbits.RC3 = 1;
    TRISCbits.RC4 = 1;
    SSP1CON1bits.SSPM = 0b1000;        // I2C Master mode, clock = FOSC / (4 * (SSPxADD+1))
    SSP1ADD = 0x09;                    // i2c at 200 kHz
    SSP1CON1bits.SSPEN = 1;            //enables i2c pins
    
    //1 Hz SQW Pin Configuration
    TRISCbits.RC0 = 1;
    INTCONbits.GIE = 1;                //general interrupt enabled
    INTCONbits.PEIE = 1;               //peripheral interrupt enabled
    T3CONbits.T3SOSCEN = 0;            //secondary oscillator disabled
    T3CONbits.TMR3CS = 0b10;           //T3CKI pin enabled
    T3CONbits.T3CKPS1 = 0b00;          //prescaler = 0
    TMR3 = 0xFFFF;
     
    //Set, Up and Down Buttons Configuration

    TRISBbits.RB0 = 1;                 //INT1 as input
    TRISBbits.RB1 = 1;                 //INT1 as input
    TRISBbits.RB2 = 1;                 //INT1 as input
    ANSELBbits.ANSB0 = 0;
    ANSELBbits.ANSB1 = 0;
    ANSELBbits.ANSB2 = 0;
    
    INTCON2bits.INTEDG0 = 0;                //interrupt on falling edge
    INTCON2bits.INTEDG1 = 0;
    INTCON2bits.INTEDG2 = 0;
    INTCON2bits.NOT_RBPU = 1;               //disable all weak pull-ups
    INTCONbits.INT0IF = 0;                  //interrupt flag on RB0 reset
    INTCON3bits.INT1IF = 0;                 //interrupt flag on RB1 reset
    INTCON3bits.INT2IF = 0;                 //interrupt flag on RB2 reset
    INTCONbits.INT0IE = 1;                  //interrupt on RB0 enabled
    INTCON3bits.INT1IE = 1;                 //interrupt on RB1 enabled
    INTCON3bits.INT2IE = 1;                 //interrupt on RB2 enabled
    
    uint8_t reg = 0x00;
    uint8_t init = 0x09;
    writeToRTC(&reg, CTRL_REG_ADR, 1);
    writeToRTC(&init, CLK_SEC_ADR, 1);
    writeToRTC(&init, CLK_MIN_ADR, 1);
    writeToRTC(&init, CLK_HR_ADR, 1);
    
    PIR2bits.TMR3IF = 0;
    PIE2bits.TMR3IE = 1;               //counter 1 interrupt enabled
    T3CONbits.TMR3ON = 1;
    
    __delay_ms(1000);
    while(1)
    {
//       __delay_ms(2000);
//       stringToUSB((uint8_t*)"Hello Motor\r\n");
        

        
    }
    
    return;
}


void __interrupt(low_priority) myTimer(void)
{

    if(PIR2bits.TMR3IF)                   
    {    
        requestTime();        
        displayTime();
        TMR3 = 0xFFFF;
        PIR2bits.TMR3IF = 0;
    }
     
    if(INTCONbits.INT0IF)                          
    {               
        if(state == NOT_SETTING)
        {
           T3CONbits.TMR3ON = 0;                  
           state = HR_SETTING;
           displayCursor();
           moveCursor(cursor_positions[state]);
        }        
        else
        {          
           writeToRTC(&time[clk_reg_adr[state]], clk_reg_adr[state], 1);
           state += 1;                            
           if(state!=NOT_DEFINED)                          
           {
               moveCursor(cursor_positions[state]);          
           }
           else
           {
               removeCursor();
               state = NOT_SETTING;
               TMR3 = 0xFFFF;
               PIR2bits.TMR3IF = 0;
               T3CONbits.TMR3ON = 1;             
           }                                
        }             
        INTCONbits.INT0IF = 0;
    }
    
    //up
    if(INTCON3bits.INT1IF)                        
    {
        if(state!=NOT_SETTING)               
        {
           digitUp();
           displayTime();
           moveCursor(cursor_positions[state]);          
        }
        INTCON3bits.INT1IF = 0;       
    }
    
    //down
    if(INTCON3bits.INT2IF)                        
    {
        if(state!=NOT_SETTING)               
        {
           digitDown();
           displayTime();
           moveCursor(cursor_positions[state]);          
        }
        INTCON3bits.INT2IF = 0;       
    }
     
   
      
}

void digitDown()
{
    low_nibble = time[clk_reg_adr[state]] & 0x0F;
    high_nibble = (time[clk_reg_adr[state]] & 0xF0) >> 4;
    
    low_nibble--;
    if(state==HR_SETTING)
    {
        if(low_nibble == 0xFF && high_nibble == 0x00)
        {
            low_nibble = 0x03;
            high_nibble = 0x02;
        }
        if(low_nibble == 0xFF && high_nibble > 0x00)
        {
            low_nibble = 0x09;
            high_nibble--;
        }
    }
    else
    {
        if(low_nibble == 0xFF)
        {
            low_nibble = 0x09;
            high_nibble--;
            if(high_nibble == 0xFF)
                high_nibble = 0x05;  
        }
    }   
    time[clk_reg_adr[state]] =  high_nibble << 4 | low_nibble;
}


void digitUp()
{
    low_nibble = time[clk_reg_adr[state]] & 0x0F;
    high_nibble = (time[clk_reg_adr[state]] & 0xF0) >> 4;
    
    low_nibble++;
    if(state==HR_SETTING)
    {
        if(high_nibble < 0x02 && low_nibble == 0x0A)
        {
            low_nibble = 0x00;
            high_nibble++;
        }  
        if(high_nibble == 0x02 && low_nibble == 0x04)
        {
            low_nibble = 0x00;
            high_nibble = 0x00;      
        }
    }
    else
    {
        if(low_nibble== 0x0A)
        {
            low_nibble = 0x00;
            high_nibble++;
            if(high_nibble==0x06)
                high_nibble=0x00;
        }  
    }   
    time[clk_reg_adr[state]] =  high_nibble << 4 | low_nibble;
}

void moveCursor(uint8_t state)
{
    LCDCommand(0xC0 | state);
}

void displayCursor()
{
    LCDCommand(0x0E);
}

void removeCursor()
{
    LCDCommand(0x0C);
}

void requestTime(void)
{    
    readFromRTC(&time[0], CLK_SEC_ADR, 3);                                            
}

void displayFormatting(uint8_t* data)
{
    uint8_t temp;
    for(int8_t i = 7; i>0; i-=3)
    {
        temp = *data++;
        display[i] = 0x30 | (temp & 0x0F);                
        display[i-1] = 0x30 | temp>>4;       
    }        
}
void displayTime(void)
{
    displayFormatting(&time[0]);
    writeToLCD(&display[0], 0xC0);
}


void readFromRTC(uint8_t* byte, uint8_t adr, uint8_t msg_lgth)
{  
    startI2C();                                        //start condition                                 
    writeInitI2C();                                       //write slave to transmit address
    writeByteI2C(adr);                                //write address of register to be read, i.e. seconds address
    repeatStartI2C();                                  //repeat start condition 
    readInitI2C();                                        //tell the slave we wanna read from it
    if(msg_lgth>1)
        readMultipleByteFromRTC(byte, msg_lgth);
    else
        readSingleByteFromRTC(byte);    
    stopI2C(); 
}

void readSingleByteFromRTC(uint8_t* byte)
{
    readByteI2C(byte, LAST_BYTE);                               //write the byte into the buffer                                      //stop condition
}

void readMultipleByteFromRTC(uint8_t* byte, uint8_t msg_lgth)
{
    uint8_t last_byte;
    for(uint8_t i=0; i<msg_lgth; i++)
    {
        last_byte = (i == msg_lgth-1)? LAST_BYTE : NOT_LAST_BYTE;
        readByteI2C(byte++, last_byte);                       //write the byte into the buffer            
    }        
}

void readByteI2C(uint8_t* recbyte, uint8_t last)
{ 
    SSP1CON2bits.RCEN = 1;             //set receive enable bit to indicate a reception operation
    while(!PIR1bits.SSP1IF);           //wait for MSSP interrupt flag bit to indicate one byte received
    PIR1bits.SSP1IF = 0;               //reset MSSP interrupt flag bit
    *recbyte = SSP1BUF;  
      
    SSP1CON2bits.ACKDT = last ? 1 : 0;
    SSP1CON2bits.ACKEN = 1;            //ACK initiation
    while(!PIR1bits.SSP1IF);
    PIR1bits.SSP1IF = 0;
}

void writeToRTC(uint8_t* byte, uint8_t adr, uint8_t msg_lgth)
{
    startI2C();                                       //start condition
    writeInitI2C();                                   //write slave to transmit address
    writeByteI2C(adr);                                //write address of register to be written to
    if(msg_lgth>1)
        writeMultipleByteToRTC(byte, msg_lgth);
    else
        writeSingleByteToRTC(byte);
    stopI2C();                                        //stop condition
}

void writeSingleByteToRTC(uint8_t* byte)
{
    writeByteI2C(*byte);                               //write the byte into the buffer                                      //stop condition
}

void writeMultipleByteToRTC(uint8_t* byte, uint8_t msg_lgth)
{
    for(uint8_t i=0; i<msg_lgth; i++)
         writeByteI2C(*byte++);                       //write the byte into the buffer
}

void writeByteI2C(uint8_t byte)
{
    
    SSP1BUF = byte;                     //slave address to transmit
    while(!PIR1bits.SSP1IF);            //MSSP interrupt flag bit set by hardware after 9th clock
    PIR1bits.SSP1IF = 0;                //reset MSSP interrupt flag bit
    if(SSP1CON2bits.ACKSTAT)
        writeToLCD((uint8_t*)"Acknowledgement failed", 0xC0);
}

void startI2C(void)
{  
    SSP1STATbits.S = 0;                 //reset start bit
    SSP1CON2bits.SEN = 1;               //initiate a start condition
    while(!PIR1bits.SSP1IF);            //MSSP interrupt flag bit set by hardware following a start condition completion 
    PIR1bits.SSP1IF = 0;                //clear MSSP interrupt flag bit
    if(!SSP1STATbits.S)
        writeToLCD((uint8_t*)"Start condition error", 0xC0);
}

void repeatStartI2C(void)
{    
    SSP1STATbits.S = 0;                 //reset start bit
    SSP1CON2bits.RSEN = 1;              //initiate a restart condition  
    while(!PIR1bits.SSP1IF);            //MSSP interrupt flag bit set by hardware following a start condition completion  
    PIR1bits.SSP1IF = 0;                //clear MSSP interrupt flag bit
    if(!SSP1STATbits.S)
        writeToLCD((uint8_t*)"Restart condition error", 0xC0);
}

void stopI2C(void)
{   
    SSP1STATbits.P = 0;                 //reset stop bit
    SSP1CON2bits.PEN = 1;               //initiate a stop condition
    while(!PIR1bits.SSP1IF);            //MSSP interrupt flag bit set by hardware following a stop condition completion 
    PIR1bits.SSP1IF = 0;                //clear MSSP interrupt flag bit     
    if(!SSP1STATbits.P)
        writeToLCD((uint8_t*)"Stop condition error", 0xC0);  
}

void writeInitI2C(void)
{   
    SSP1BUF = SLAVE_ADR_WRITE;          //slave address to transmit
    while(!PIR1bits.SSP1IF);            //MSSP interrupt flag bit set by hardware after 9th clock
    PIR1bits.SSP1IF = 0;                //reset MSSP interrupt flag bit
    if(SSP1CON2bits.ACKSTAT)
        writeToLCD((uint8_t*)"Acknowledgement failed", 0xC0);
}

void readInitI2C(void)
{   
    SSP1BUF = SLAVE_ADR_READ ;          //slave address to receive from
    while(!PIR1bits.SSP1IF);            //MSSP interrupt flag bit set by hardware after 9th clock
    PIR1bits.SSP1IF = 0;                //reset MSSP interrupt flag bit
    if(SSP1CON2bits.ACKSTAT)
        writeToLCD((uint8_t*)"Acknowledgement failed", 0xC0);
}

void stringToUSB(uint8_t* strg)
{
     while(*strg)    
         charToUSB(*strg++);
}

void charToUSB(uint8_t chtr)
{
    while(!PIR1bits.TX1IF);        
    TXREG1 = chtr;
}

void LCDCommand(uint8_t cmd)
{
    LATD = cmd;
    RS = 0;
    RW = 0;
    EN = 1;
    __delay_ms(1);
    EN = 0;
}

void LCDData(uint8_t cmd)
{
    LATD = cmd;
    RS = 1;
    RW = 0;
    EN = 1;
    __delay_ms(1);
    EN = 0;
}

void writeToLCD(uint8_t* p, uint8_t pos)
{
     LCDCommand(pos);
     __delay_ms(5);
     while(*p)
     {
           LCDData(*p++);
           __delay_ms(5);
     }
}