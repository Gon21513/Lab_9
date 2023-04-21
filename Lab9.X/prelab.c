/* 
 * File:   prelab9.c
 * Author: Luis Pedro Gonzalez 21513
 *
 * Created on 19 de abril de 2023, 03:34 PM
 */


// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

#include <xc.h>
#include <stdint.h>
#include <stdio.h>
#include <pic16f887.h>

//variable
#define _XTAL_FREQ 4000000 //frec de 1mhz
#define dirEEPROM 0x04

//uint8_t pot = 0; //alamcena el valor del potenciometro

uint8_t address = 0x01; // variable con la deireccion de los datodos en epeprom
int dormir = 0 ; //bandera para indicar si esta en modo sleep 

uint8_t potValue;
uint8_t botonPrevState;

//prototipos
void setup(void);
void EEPROMWRITE(uint8_t data, uint8_t address); //lectura de eeprom
uint8_t EEPROMREAD(uint8_t adress); //lectura del eeprom

void __interrupt() isr(void){
    if (INTCONbits.RBIF){
        PORTB = PORTB; //lectura del puertp b
        INTCONbits.RBIF = 0;
    }
}


//----------MAIN---------

void main(void){
    
    setup();
    ADCON0bits.GO = 1;
    
    //-----looop------
    while(1){
        if (ADCON0bits.GO == 0){
       potValue = ADRESH; 
       PORTD = potValue;
       __delay_us(50);
       ADCON0bits.GO = 1;
        }
    PORTC = EEPROMREAD(dirEEPROM);  
    
    //REVISAR LOS BOTONES
    if (RB0 == 0){
        botonPrevState = 1;
    }
       
    if (RB0 == 1 && botonPrevState ==1){
        EEPROMWRITE(potValue, dirEEPROM);
        botonPrevState = 0;
    }
    
    if (RB1 == 0){
        INTCONbits.RBIF = 0;
        SLEEP();
    }
     
    } 
    
}


//-------------setup---
void setup(void){
   
//----------configuracion de puertos--------------
    ANSEL = 0b00100000; //canal an5 re0 como entrada analogica
    ANSELH = 0x00;
    
    TRISA = 0x00;
    TRISB = 0x03; //rb0 y rb1 como entradas
    TRISC = 0x00;
    TRISD = 0x00;
    TRISE = 0x01; //re0 como entrada
    
    PORTA = 0x00;
    PORTB = 0x00;
    PORTC = 0x00;
    PORTD = 0x00;
    PORTE = 0x00;
   
//----------pullups------------------
    OPTION_REGbits.nRBPU = 0; //habilitarr pullups
    WPUBbits.WPUB0 = 1;
    WPUBbits.WPUB1 = 1; 
    
//------------interrupciones-----------------
    INTCONbits.RBIE = 1;
    INTCONbits.RBIF = 0;
    IOCBbits.IOCB0 = 1;
    
//--------------oscilador-------------
    OSCCONbits.IRCF2 = 1;//4MHZ
    OSCCONbits.IRCF1 = 1;
    OSCCONbits.IRCF0 = 0;
    
    OSCCONbits.SCS = 1;//INTERNO
    
//---------ADC----------------
    ADCON0bits.CHS = 5; // seleccionar AN0
    
    ADCON1bits.VCFG1 = 0; // Voltaje de referencia de 0V
    ADCON1bits.VCFG0 = 0; // Voltaje de referencia de 5V
            
    ADCON0bits.ADCS = 1; // Fosc/8
            
    
    ADCON1bits.ADFM = 0;        
            
    __delay_ms(5);
    ADCON0bits.ADON = 1;  


}



//----------funciones--------------
void EEPROMWRITE(uint8_t data, uint8_t address){
    EEADR = address;
    EEDAT = data;
    
    EECON1bits.EEPGD = 0; //escribe en la memoria de datos
    EECON1bits.WREN = 1; // habiliota escritura en eeprom 
    
    INTCONbits.GIE = 0; //deshabilita las interrupciones 
    
    
    //obligatorio
    EECON2 = 0x55;
    EECON2 = 0xAA;
    EECON1bits.WR = 1; //habilitar escritua
    
    EECON1bits.WREN = 0; //apagamos la escritura
            
}

uint8_t EEPROMREAD(uint8_t address){
    EEADR = address ;
    EECON1bits.EEPGD = 0;
    EECON1bits.RD = 1;
    return EEDAT;
}