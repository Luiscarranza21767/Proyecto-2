/* Universidad del Valle de Guatemala
 IE2023 Programación de Microcontroladores
 Autor: Luis Pablo Carranza
 Compilador: XC8, MPLAB X IDE (v6.00)
 Proyecto: Proyecto de laboratorio 2
 Hardware PIC16F887
 Creado: 3/11/22
 Última Modificación: 03/11/22*/

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT // Oscillator Selection bits (INTOSC 
//oscillator without clock out)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and 
//can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR 
//pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code 
//protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code 
//protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/
//External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-
//Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin 
//has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit 
//(Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits 
//(Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include "oscilador.h"
#define _XTAL_FREQ 8000000

//******************************************************************************
// Definición de funciones y variables
//******************************************************************************
void setup(void);
void initUART(void);        
void cadena(char *puntero);
void ComSerial(void);
void setup_portb(void);
//******************************************************************************
// Función de interrupciones
//******************************************************************************
void __interrupt() isr (void){
    if (RBIF == 1){             // Revisa si hay interrupción del puerto B
    if (PORTBbits.RB7 == 0)     // Si hay revisa si se presionó RB6
    {
        __delay_ms(50);
        if (PORTAbits.RA1 == 0){
            PORTAbits.RA1 = 1;
        }
        else {
            PORTAbits.RA1 = 0;
        }
    }
    RBIF = 0;
    }
}
//******************************************************************************
// Función principal
//******************************************************************************
void main(void) {
    setup();            // Realiza la configuración de puertos
    setupINTOSC(7);     // Oscilador a 8 MHz
    initUART();         // Configuración para el módulo UART
    setup_portb();      // Configuración de interrupción del puerto B
    
    while(1){
        if (PIR1bits.RCIF == 1){
            ComSerial();
        } // Cuando hay interrupción continua
        __delay_ms(100); 
    }
}
//******************************************************************************
// Configuración de puertos
//******************************************************************************
void setup(void){
    ANSEL = 0;
    ANSELH = 0;
    
    TRISA = 0;
    PORTA = 0;
    TRISB = 0b11000000;
    PORTB = 0;
    TRISD = 0;
    PORTD = 0; 
}
//******************************************************************************
// Configuración de módulo UART
//******************************************************************************
void initUART(void){
    // Configuración velocidad de baud rate
    SPBRG = 12;
    
    TXSTAbits.SYNC = 0;     // Modo asíncrono
    RCSTAbits.SPEN = 1;     // Habilitar módulo UART
    
    TXSTAbits.TXEN = 1;     // Habilitar la transmisión
    PIR1bits.TXIF = 0;
    
    RCSTAbits.CREN = 1;     // Habilitar la recepción
}
//******************************************************************************
// Configuración del puerto B
//******************************************************************************
void setup_portb(void){
    INTCONbits.GIE = 1;     // Habilita interrupciones globales
    INTCONbits.RBIE = 1;    // Habilita interrupción del puerto B
    INTCONbits.RBIF = 0;    // Apaga la bandera de interrupción del puerto B
    IOCB = 0b11000000;      // Habilita la interrupción en cambio
    WPUB = 0b11000000;      // Habilita el Weak Pull-Up en el puerto B
    OPTION_REGbits.nRBPU = 0;   // Deshabilita el bit de RBPU
}
//******************************************************************************
// Función para comunicación terminal - PIC
//******************************************************************************
void ComSerial(void){
    if (RCREG == 0b00110001){  // Si es 1 en ASCII ejecuta
        PORTAbits.RA0 = 1;
        }
    if (RCREG == 0b00110010){   // Si es 2 en ASCII ejecuta
        PORTAbits.RA0 = 0;       
    }
    TXREG = RCREG;
    PIR1bits.RCIF = 0;
}


