/* 
 * File:   lab9.c
 * Author: Luis Pedro Gonzalez 21513
 *
 * Created on 20 de abril de 2023, 21:16 PM
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
#define _XTAL_FREQ 1000000 //frec de 1mhz
//#define dirEEPROM 0x01 // variable con la direccion del epprom

int dormir = 0 ; //bandera para indicar si esta en modo sleep 
uint8_t potValue = 0;

//uint8_t pot = 0; //alamcena el valor del potenciometro
uint8_t address = 0x04; // variable con la deireccion de los datodos en epeprom



//prototipos
void setup(void);
void EEPROMWRITE(uint8_t address, uint8_t data); //lectura de eeprom
uint8_t EEPROMREAD(uint8_t address); //lectura del eeprom

void __interrupt() isr(void){
    
    if (PIR1bits.ADIF){//revisar las interrupciones en el adc
        if (ADCON0bits.CHS == 0){
            potValue = ADRESH; //pasar valor de adresh a potenciometro
            PORTC = potValue; // enviar vzlor del potenciometro al puerto c
        }
        PIR1bits.ADIF = 0; //se limpia la bandera del adc
        
    }
    
    if (INTCONbits.RBIF){//revisar las interrupciones del portb
        if (PORTBbits.RB0 == 0){//enceder
            dormir = 0; //apagar la bander de dormir 
            PORTEbits.RE0 = 1; //limpia el bits que inica el sleeps
        }
        
        else if (PORTBbits.RB1 == 0){
            dormir = 1; //encender la bandera de dormir
            PORTEbits.RE0 = 0 ; //encender un led para indicarme que sleep esta encdedio
            SLEEP(); //poner el pic en modo sleep
        }
        
        else if (PORTBbits.RB2 == 0){
            dormir = 0 ; //apagar bandera de dormir
            PORTEbits.RE0 = 1; //apagar led que indica si esta dormido 
            EEPROMWRITE(address, potValue); //escribir valor del potenciometro en el eeprom
        }
        INTCONbits.RBIF = 0; 

    }
    return;
    
}


//---------------main----------------

void main(void){
    setup();
    
    while(1){
        
        //PORTEbits.RE1 = 1; //apagar led que indica si esta dormido 

        if (dormir == 0){//revisar si el pic esta en modo sleep
            if (ADCON0bits.GO == 0){//revisar si se inicial la ocnversion adc
                ADCON0bits.GO = 1; //iniciar la conversion 
                __delay_us(40);
            }
            
        }
        PORTD = EEPROMREAD(address); //mostrar el valor del eeprom en portd 
    }
    
}


//-------------setup---
void setup(void){
   
//----------configuracion de puertos--------------
    ANSEL = 0b00000001; //canal an1 como entrada analogica
    ANSELH = 0x00;
    
    TRISA = 0b00000001; // ra0 como entradaa
    
    //botones
    TRISBbits.TRISB0 = 1; //rb0 como entrada
    TRISBbits.TRISB1 = 1; //rb1 como entrada 
    TRISBbits.TRISB2 = 1; //rb1 como entrada 

    
    TRISC = 0x00;
    TRISD = 0x00;
    TRISE = 0x00; 
    
    //limpiar puertos 
    PORTA = 0x00;
    PORTB = 0x00;
    PORTC = 0x00;
    PORTD = 0x00;
    PORTE = 0x00;
   
//----------pullups------------------
    OPTION_REGbits.nRBPU = 0; //habilitarr pullups
    WPUBbits.WPUB0 = 1;
    WPUBbits.WPUB1 = 1; 
    WPUBbits.WPUB2 = 1; 

    
//------------interrupciones-----------------
    INTCONbits.GIE = 1; //habilitar interrupciones globales
    INTCONbits.PEIE = 1; //habilitar interrupciones de perifericos
    INTCONbits.RBIE = 1; //habilitar interrupciones en portb
    
    IOCBbits.IOCB0 = 1; //habilitar interrupciones en rb0
    IOCBbits.IOCB1 = 1; // habilitar interrupciones en rb1
    IOCBbits.IOCB2 = 1; // habilitar interrupcion en el rb2
    
    INTCONbits.RBIF = 0; //limpirar bander de interrupcion de portb
    PIR1bits.ADIF = 0; //limpieza de bandera de interrupcion del adc
    PIE1bits.ADIE = 1; //habilitar interrupcion del adc 
    
//--------------oscilador-------------
    OSCCONbits.IRCF2 = 1;//4MHZ
    OSCCONbits.IRCF1 = 0;
    OSCCONbits.IRCF0 = 0;
    
    OSCCONbits.SCS = 1;//INTERNO
    
//---------ADC----------------
    ADCON0bits.CHS = 0b0000; // seleccionar AN0
    
    ADCON1bits.VCFG1 = 0; // Voltaje de referencia de 0V
    ADCON1bits.VCFG0 = 0; // Voltaje de referencia de 5V
            
    ADCON0bits.ADCS = 0b01; // Fosc/8
            
    
    ADCON1bits.ADFM = 0;        
    ADCON0bits.ADON = 1;  

    __delay_ms(5);

}



////----------funciones--------------
void EEPROMWRITE(uint8_t address, uint8_t data){
    EEADR = address;//asignar direccin de datos 
    EEDAT = data;//datos 
    
    EECON1bits.EEPGD = 0; //escribe en la memoria de datos
    EECON1bits.WREN = 1; // habilita escritura en eeprom 
    
    INTCONbits.GIE = 0; //deshabilita las interrupciones 
    

    //obligatorio
    EECON2 = 0x55;
    EECON2 = 0xAA;
    EECON1bits.WR = 1; //habilitar escritua
    
    EECON1bits.WREN = 0; //apagamos la escritura
    
    INTCONbits.RBIF = 0; // limpiamos bandera en el puerto b
    INTCONbits.GIE = 1; //habilita interrupciones globales 
            
}

uint8_t EEPROMREAD(uint8_t address){
    EEADR = address ;//asgina la direccin 
    EECON1bits.EEPGD = 0;//selecciona la menoria eeprom
    EECON1bits.RD = 1;//habilita lectura de eeprom
    return EEDAT;//retorna el valor de la direccion leida
}
