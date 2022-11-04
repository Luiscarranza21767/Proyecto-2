/*
 * File:   oscilador.c
 * Author: Luis Pablo Carranza
 *
 * Created on 24 de octubre de 2022, 11:37 PM
 */

#include "oscilador.h"

void setupINTOSC(uint8_t IRCF){
    if (IRCF == 7){
        OSCCONbits.IRCF = 0b111;    // Oscilador 8 MHz
    }
    if (IRCF == 6){
        OSCCONbits.IRCF = 0b110;    // Oscilador 4 MHz
    }
    OSCCONbits.SCS = 1;         // Oscilador interno
}
