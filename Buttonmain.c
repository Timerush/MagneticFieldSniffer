/* 
 * File:   Buttonmain.c
 * Author: timerush
 *
 * Created on April 4, 2019, 3:29 PM
 */
// the necessary parts are really only the voids that are not already given also the var modify
#define FSY 3655000UL
#include <stdio.h>
#include <stdlib.h>
#include <libpic30.h>
#include <xc.h>
#include <p33EP64MC502.h>
#include <math.h>
#include<string.h>
/* define constants for programming the LCD module */
#define pgm_delay_ms 2
#define INSTRUCTION 0
#define DATA 1
int enter; // rotary encoder push button
STATES state = S1;
double DC,Freq,BField,Digit;
/* set Configuration Bits */
#pragma config ICS = PGD2    // Communicate on PGED2 (pin 14) and PGEC2 (pin 15)
#pragma config JTAGEN = OFF  // Disable JTAG inorder to use RB8 through RB11
/*
 * 
 */
void __attribute__((__interrupt__,auto_psv)) _INT1Interrupt (void); // interrupt 1
void __attribute__((__interrupt__,auto_psv)) _INT2Interrupt (void); // interrupt 2
void Init_DIO_Ports (void);
void Toggle_Enable_line (void);
void Write_LCD_Nibble(int data, int cmd);
void Write_LCD_Byte(int data, int cmd);
void Init_LCD_Module(void);
void Init_Int(void); // initialize interrupts
void Init_Encoder(void); //initialize encoder
void Poll_Encoder(void);// poll encoer btn
void State1(void);

typedef enum {
    S0, S1, S2
} STATES;

void __attribute__((__interrupt__,auto_psv)) _INT1Interrupt (void){ // interrupt 1 on/off
    // change state between S0 and S1
    switch (state) {
        case S0:
            //initialize display
            state = S1; break;
        case S1:
            state = S0; break;
        case S2:
            state = S0; break;
    }
    IFS1bits.INT1IF = 0;
}
void __attribute__((__interrupt__,auto_psv)) _INT2Interrupt (void){ // interrupt 2 set/mes
    // change state between S1 and S2
    switch (state) {
        case S0:
            state = S0; break;
        case S1:
            state = S2; break;
        case S2:
            state = S1; break;
    IFS1bits.INT2IF = 0;
}
}

void Init_Int (void){
    
    RPINR0bits.INT1R = 0x22; //assign int 1 to pin # 6
    RPINR1bits.INT2R = 0x23; // assign int 2 to pin # 7
    INTCON2bits.GIE = 1;
    INTCON2bits.INT1EP = 1;
    INTCON2bits.INT2EP = 1;
    IFS1bits.INT1IF = 0;
    IFS1bits.INT2IF = 0;
    IEC1bits.INT1IE = 1;
    IEC1bits.INT2IE = 1;
    
}
void Init_Encoder (void){
    ANSELBbits.ANSB0 = 0; 
    TRISBbits.TRISB0 = 1; // QE B pin #4
    ANSELBbits.ANSB1 = 0;
    TRISBbits.TRISB1 = 1;// QE A pin #5
    ANSELAbits.ANSA1 = 0;
    TRISAbits.TRISA1 = 1; // rotary encoder push button pin #3
    
    
}
Init_QEI_module (void){
    RPINR14 = 0x2021;
    QEI1CONbits.QEIEN = 0;
    QEI1CONbits.PIMOD = 6;
    QEI1LECH = 0;
    QEI1LECL = 0;
    QEI1GECH = 0;
    QEI1GECL = 80;
    QEI1CONbits.QEIEN = 1;
    
}
void Init_DIO_Ports (void) {
    ANSELB = 0;     // use peripheral pins associted with PORTB for digital I/O
    TRISB = 0xC0FF; // set RB8 to RB13 for output, the rest for input
    PORTB =  0;     // set all LCD inputs low
  }

void Toggle_Enable_line (void) {
    __delay_ms(pgm_delay_ms);   // delay
    PORTBbits.RB13 = 1;         // set E high
    __delay_ms(pgm_delay_ms);   // delay
    PORTBbits.RB13 = 0;         // set E low
    __delay_ms(pgm_delay_ms);   // delay
}

void Write_LCD_Nibble(int data, int cmd) {
 PORTB =  data << 8;     // place nibble at LCD inputs DB4 through DB7
 PORTBbits.RB12 = cmd;   // set RS; cmd = 0 for instruction, 1 for data
 Toggle_Enable_line ();  // strobe data into LCD module
 PORTB =  0;             // set all LCD inputs low
}

void Write_LCD_Byte(int data, int cmd) {
    Write_LCD_Nibble((data & 0x00F0) >> 4, cmd); // write upper nibble
    Write_LCD_Nibble( data & 0x000F, cmd);       // write lower nibble
}

void Init_LCD_Module(void) {
    Write_LCD_Nibble(0b0011, INSTRUCTION);  // Initialize the LCD Module
    Write_LCD_Nibble(0b0011, INSTRUCTION);
    Write_LCD_Nibble(0b0011, INSTRUCTION);
    Write_LCD_Nibble(0b0010, INSTRUCTION);  // invoke 4-bit mode
    Write_LCD_Byte(0b00101000, INSTRUCTION);// 4-bit mode, two-line,5X7 dot
    Write_LCD_Byte(0b00000001, INSTRUCTION);// clear display, cursor at 0x00
    Write_LCD_Byte(0b00001111, INSTRUCTION);// display on,cursor blink/underline
}
void Poll_Encoder(void){
   enter = PORTAbits.RA1 ^1;
   Digit= (double) (POS1CNTL/8);
    __delay_ms(50);
}

void State1(void){
      Poll_Encoder();
    
}
int main(void){
    Init_DIO_Ports ();
    Init_LCD_Module();
    Init_Int();
    Init_Encoder();
    while(1){
        switch(state) {
            case S0:
                //clear LCD function
                while (1) { } ; break;
            case S1:
                State1();
                //update display
                //
                break;
            case S2:
                //help me
                break;
        }  
    ClrWdt();
    }
}