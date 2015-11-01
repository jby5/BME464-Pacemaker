/* 
 * File:   pacemaker_A.c
 * Author: Jessica
 *
 * Created on November 1, 2015, 1:36 AM

 * 1. Detect atrial EGM by detecting slew rate + threshold
 * 2. if EGM detected, output 10 cycles of a 50kHz PWM signal 
 */

#include <stdio.h>
#include <stdlib.h>
//#include <delays.h>
#include <htc.h>
//#include <pic16f1827.h>
#pragma config1 FOSC = FOSC_INTOSC //internal oscillator, I/O function on CLKIN pin 

int processA(void);
void output(void);
void SysInit(void);

void main(void) {
    SysInit();
    while(1){
    int detected; 
    detected = processA();
        if (detected == 1){
            output();
            //Delay10KTCYx(30); //delay 300ms to "debounce"
        }
    }
}

void SysInit(void){
    //clock source
    OSCCONbits.IRCF = 0b1101; //use 4MHz system clock
    OSCCONbits.SCS = 0b10; 
    
    
    //ADC
    
    //PWM
    T2CONbits.TMR2ON = 1; //1MHz timer
    CCP3CONbits.DC3B = 0b10;
    
    CCPTMRSbits.C3TSEL = 0b00; //use Timer 2
    PR2 = 0x04; //period 
    CCPR3L = 0b00000010; //8 MSBs of period 
    
    TRISAbits.TRISA3 = 0; //CCP3 output
    
    
}
int processA(void){ //ADC and calculation of slew rate/threshold
    return 1;
}

void output(void){
    CCP3CONbits.CCP3M = 0b1100; //PWM mode enabled
    //set a delay
    CCP3CONbits.CCP3M = 0b0000; //PWM disabled
}
