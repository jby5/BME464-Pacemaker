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
//#pragma config1 FOSC = FOSC_INTOSC //internal oscillator, I/O function on CLKIN pin 
#define _XTAL_FREQ 0x3D0900//4MHz system clock

void processA(void);
void output(void);
void SysInit(void);

int detected; 
int stateA;

void main(void) {
    SysInit();
    
    while(1){
        
        processA();
        
        if (stateA == 1){
            output();
            stateA = 0;
        __delay_ms(200); //delay 200ms to "debounce"    
            
        }
        
    }
     
}

void SysInit(void){
    //clock source
    OSCCONbits.IRCF = 0b1101; //use 4MHz system clock
    OSCCONbits.SCS = 0b10; 
    
    //ADC
    ANSELAbits.ANSA0 = 1; //pin A0 is analog 
    TRISAbits.TRISA0 = 1; //A0 input
    ADCON0 = 0x00; // clear ADCON0, select AN0
    ADCON1 = 0b01010000; //left justified, FOSC/16, Vdd and Vss ref
    ADCON0bits.ADON = 0x01; //enable ADC module 
    
    TRISBbits.TRISB0 = 0; 
    
    
    //PWM
    T2CONbits.TMR2ON = 1; //1MHz timer
    CCP3CONbits.DC3B = 0b00;
    
    CCPTMRSbits.C3TSEL = 0b00; //use Timer 2
    PR2 = 0x13; //period 
    CCPR3L = 0b00001010; //8 MSBs of period 
    
    TRISAbits.TRISA3 = 0; //CCP3 output
    
}

void processA(void){ //ADC and calculation of slew rate/threshold
    
    int slewThresh = 0; 
    int ampThresh = 4*255/5;  
    int dt = 1; //10ms
    int EGMVals[5];
    int slewVals[4];
    int slewSum = 0;
    int i;
    int slewAvg;
    
    //ADC conversion and array storage
    for(i=0; i<5; i++){
        ADCON0bits.GO = 1; 
        while(ADCON0bits.GO==0){}; //wait to finish conversion
        EGMVals[i] = ADRESH; //replace with correct pin
        
        if(i>0){
            slewVals[i] = (EGMVals[i]-EGMVals[i-1])/dt; //replace with correct pin    
            slewSum += slewVals[i];
        }
        //slewVal = EGMVals[4]-EGMVals[0]; 
        __delay_ms(1); 
    }

    //calculate avg slew rate
    slewAvg = slewSum/4;
    //slewAvg = EGMVals[4] - EGMVals[0];
    
    //if amplitude + slew rate passed threshold, stateA = 1  
    if(EGMVals[4]>ampThresh){   
        
        if (slewAvg>slewThresh){
            stateA = 1;
            //LATBbits.LATB0 = 1;
        } else{
            //LATBbits.LATB0 = 0;
            stateA = 0;
        }
    } else {
        stateA = 0;
        //LATBbits.LATB0 = 0; 
    }
              
}

void output(void){
    CCP3CONbits.CCP3M = 0b1100; //PWM mode enabled
    __delay_ms(1); 
    CCP3CONbits.CCP3M = 0b0000; //PWM disabled
    LATAbits.LATA3 = 0;
}
