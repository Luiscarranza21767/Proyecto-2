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
#define _XTAL_FREQ 1000000

//******************************************************************************
// Definición de funciones y variables
//******************************************************************************
void setup(void);
void setup_ADC(void);
void setup_PWM1(void);
void setup_PWM2(void);
void setupTMR0(void);
void setupTMR1(void);
void mapeo(void);
void modomanual(void);
void PWMtmr0(void);
void PWMtmr1(void);

// Variables para la configuración del PWM
unsigned int valADC;
unsigned int vPWM;
unsigned int vPWMl;
unsigned int vPWMh;
unsigned int HIGHpulse;
unsigned int HIGHpulse1;
//******************************************************************************
// Función para interrupciones
//******************************************************************************
void __interrupt() isr (void){
    if (INTCONbits.T0IF){
        // Revisa si hay interrupción de TMR0
        if (PORTCbits.RC3){
            // Si el bit está encendido lo apaga y carga el valor al TMR0
            TMR0 = 255-HIGHpulse;
            PORTCbits.RC3 = 0;
        }
        else {
            // Si el bit está apagado lo enciente y carga el valor al TMR0
            TMR0 = HIGHpulse;
            PORTCbits.RC3 = 1;
        }
        // Apaga la bandera de interrupción del TMR0
        INTCONbits.T0IF = 0;
    }
    if (PIR1bits.TMR1IF){
        // Revisa si hay interrupción de TMR0
        if (PORTCbits.RC0){
            // Si el bit está encendido lo apaga y carga el valor al TMR1
            TMR1H = ((61314+(65535-HIGHpulse1)) & 0xFF00) >> 8;
            TMR1L = (61314+(65535-HIGHpulse1)) & 0x00FF;
            PORTCbits.RC0 = 0;
        }
        else {
            // Si el bit está apagado lo enciende y carga el valor al TMR1
            TMR1H = (HIGHpulse1&0xFF00) >> 8;
            TMR1L = HIGHpulse1&0x00FF;
            PORTCbits.RC0 = 1;
        }
        // Apaga la bandera de interrupción del TMR1
        PIR1bits.TMR1IF = 0; 
    }
}

//******************************************************************************
// Función principal
//******************************************************************************
void main(void) {
    setup();
    setupINTOSC(4);     // Oscilador a 1 MHz
    setup_ADC();
    setup_PWM1();
    setup_PWM2();
    setupTMR0();
    setupTMR1();
    HIGHpulse = 241;
    HIGHpulse1 = 65411;
    while(1){
        modomanual();        
        __delay_ms(1);
    }
}
//******************************************************************************
// Configuración de puertos
//******************************************************************************
void setup(void){
    ANSELH = 0;
    TRISB = 0;
    TRISC = 0; 
    TRISD = 0;
    PORTC = 0;
    PORTD = 0;
}
//******************************************************************************
// Configuración del ADC
//******************************************************************************
void setup_ADC(void){
    PORTAbits.RA0 = 0;      // Inicia el bit 0 de PORTA en 0
    TRISAbits.TRISA0 = 1;   // RA0 es entrada
    ANSELbits.ANS0 = 1;     // RA0 es analógico
    
    PORTAbits.RA1 = 0;      // Configuración del canal analógico RA1
    TRISAbits.TRISA1 = 1;
    ANSELbits.ANS1 = 1;
    
    PORTAbits.RA2 = 0;      // Configuración del canal analógico RA2
    TRISAbits.TRISA2 = 1;
    ANSELbits.ANS2 = 1;
    
    PORTAbits.RA3 = 0;      // Configuración del canal analógico RA2
    TRISAbits.TRISA3 = 1;
    ANSELbits.ANS3 = 1;
    
    ADCON0bits.ADCS1 = 0;
    ADCON0bits.ADCS0 = 1;   // Fosc/8
    
    ADCON1bits.VCFG1 = 0;   // Ref VSS
    ADCON1bits.VCFG0 = 0;   // Ref VDD
    
    ADCON1bits.ADFM = 0;    // Justificado a la izquierda
    
    ADCON0bits.ADON = 1;    // Habilitar el convertidor ADC
    __delay_us(100);
}
//******************************************************************************
// Configuración del PWM
//******************************************************************************
void setup_PWM1(void){
    TRISCbits.TRISC2 = 1;       // CCP1
    PR2 = 254;                  // Periodo de 16.32 ms
    CCP1CON = 0b00001100;       // P1A como PWM
    TMR2IF = 0;                 // bandera de TMR2 apagada
    T2CONbits.T2CKPS = 0b11;    // Prescaler 1:16
    TMR2ON = 1;                 // Habilitar TMR2
    while(!TMR2IF);             //
    TRISCbits.TRISC2 = 0;       // Habilitar la salida del PWM
}
void setup_PWM2(void){
    TRISCbits.TRISC1 = 1;       // CCP0
    PR2 = 254;                  // Periodo de 16.32 ms
    CCP2CON = 0b00001100;       // P2A como PWM
    TMR2IF = 0;                 // bandera de TMR2 apagada
    while(!TMR2IF);
    TRISCbits.TRISC1 = 0;       // Habilitar la salida del PWM
}
//******************************************************************************
// Configuración del TMR0
//******************************************************************************
void setupTMR0(void){
    INTCONbits.GIE = 1;         // Habilitar interrupciones globales
    INTCONbits.T0IE = 1;        // Habilitar interrupción de TMR0
    INTCONbits.T0IF = 0;        // Desactivar la bandera de TMR0
    
    OPTION_REGbits.T0CS = 0;    // Fosc/4
    OPTION_REGbits.PSA = 0;     // Prescaler para TMR0
    OPTION_REGbits.PS = 0b011;  // Prescaler 1:16
    TMR0 = 0;                   // Valor inicial del TMR0
}
//******************************************************************************
// Configuración del TMR1
//******************************************************************************
void setupTMR1(void){
    T1CONbits.T1CKPS = 0;
    T1CONbits.T1OSCEN = 0;
    T1CONbits.TMR1CS = 0;    // Internal clock (FOSC/4)
    T1CONbits.TMR1ON = 1;    // bit 0 enables timer
    // Valor inicial del TMR1
    TMR1H = 0xEF;         // preset for timer1 MSB register
    TMR1L = 0x82;         // preset for timer1 LSB register
    INTCONbits.PEIE = 1; // Habilitar interrupción de periféricos
    PIE1bits.TMR1IE = 1; // Habilitar interrupción del timer 1
    PIR1bits.TMR1IF = 0; // Apagar bandera de interrupción del timer 1
    
}
//******************************************************************************
// Función para PWM con TMR0
//******************************************************************************
void PWMtmr0(void){
    valADC = ((ADRESH << 2) + (ADRESL >> 6));
    HIGHpulse = (0.017*valADC + 226); // mapea el valor de 0-1023 a 224-241
}
//******************************************************************************
// Función para PWM con TMR1
//******************************************************************************
void PWMtmr1(void){
    valADC = ((ADRESH << 2) + (ADRESL >> 6));
    HIGHpulse1 = (0.37*valADC + 65036); // mapea el valor a 0xFF0C-FF06
    PORTD = valADC;
}
//******************************************************************************
// Función para el mapeo de variables para el módulo PWM
//******************************************************************************
void mapeo(void){
    // Carga el resultado a valADC en una variable de 1024 bits
    valADC = ((ADRESH << 2) + (ADRESL >> 6));
    // Mapea el resultado a los valores calculados para 1 y 2ms
    vPWM = (0.061*valADC + 63);
    // Obtiene los 2 bits bajos de la variable vPWM
    vPWMl = vPWM & 0x003;
    // Obtiene los 8 bits más altos de vPWM 
    vPWMh = (vPWM & 0x3FC) >> 2;
}

//******************************************************************************
// Función para el modo manual del proyecto
//******************************************************************************
void modomanual(void){
    // Iniciar la conversión ADC
    ADCON0bits.CHS = 0b0000;
    __delay_us(100);
    ADCON0bits.GO = 1;
    while (ADCON0bits.GO == 1); // Revisa si ya terminó la conversión ADC
    ADIF = 0;                   // Apaga la bandera del ADC
    // Hará el mapeo del ADC a valores para el servo
    mapeo();
    // Carga los bits bajos a CCP1CON <5:4>
    CCP1CONbits.DC1B = vPWMl;
    // Carga los bits altos a CCPR1L
    CCPR1L = vPWMh;
    
    // Cambia a canal analógico 1
    ADCON0bits.CHS = 0b0001;
    __delay_us(100);
    ADCON0bits.GO = 1;
    while (ADCON0bits.GO == 1); // Revisa si ya terminó la conversión ADC
    ADIF = 0;               // Apaga la bandera del ADC
    // Hará el mapeo del ADC a valores para el servo
    mapeo();
    // Carga los bits bajos a CCP2CON <5:4>
    CCP2CONbits.DC2B0 = vPWMl & 0x01;
    CCP2CONbits.DC2B1 = ((vPWMl & 0x02) >> 1);
    // Carga los bits altos a CCPR2L
    CCPR2L = vPWMh;
    
    // Cambia a canal analógico 2
    ADCON0bits.CHS = 0b0010;
    __delay_us(100);
    ADCON0bits.GO = 1;
    while (ADCON0bits.GO == 1); // Revisa si ya terminó la conversión ADC
    ADIF = 0; 
    PWMtmr0();
    __delay_us(100);
    
     // Cambia a canal analógico 3
    ADCON0bits.CHS = 0b0011;
    __delay_us(100);
    ADCON0bits.GO = 1;
    while (ADCON0bits.GO == 1); // Revisa si ya terminó la conversión ADC
    ADIF = 0; 
    PWMtmr1();
    __delay_us(100);
        
}