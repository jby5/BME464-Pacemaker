/* BME 464 - Ventricle processor code 
 * Student: Jessica Yan
 * PIC18F46K22 USED FOR THIS ELECTRODE
 */
//#include "Lcd.h"
#include <delays.h>
#include <p18f46k22.h>
#include <stdio.h>
#include "adc.h"

#pragma config FOSC = INTIO67   // Internal OSC block, Port Function on RA6/7
#pragma config WDTEN = OFF      // Watch Dog Timer disabled. SWDTEN no effect
#pragma config XINST = OFF      // Instruction set Extension and indexed Addressing mode disabled

//Define statements
//#define One_Sec  0x80 	//Load high byte of timer 1 with this for 1 second
// Timer 1 clock source is crystal oscillator on T1OS1/T1OS0, 1:1,
// Dedicated enabled, Do Not Synch, Enable Timer1
//#define Timer1  0x89

//Variable definitions
int stateV;
int detected;  
int slewThresh = 0; 
int ampThresh = 3.7*255/5.5; 

#define EGMThresh 
//Function definitions
void processV(void);
void producePulse(void);
void detect(void);
void SysInit(void);
void High_Priority_ISR(void);
void RTC_ISR(void);

//High priority interrupt
#pragma code InterruptVectorHigh = 0x08
void InterruptVectorHigh (void){
  _asm
    goto High_Priority_ISR
  _endasm
}
#pragma interrupt High_Priority_ISR
void High_Priority_ISR(void){
    RTC_ISR(); //Call real-time clock service routine
}

//Called every second by the interrupt
void RTC_ISR (void){
    if (INTCONbits.TMR0IF)            // If timer overflowed
    {
        producePulse();
        T0CONbits.TMR0ON = 0;         // turn timer off
        TMR0H = 0;
        TMR0L = 0;
        
        INTCONbits.TMR0IF = 0;        // Clear timer flag
        INTCONbits.INT0IF = 0;      // Clear interrupt flag
    }
}


void main(void)
{
    SysInit();
    stateV = 0;
    while(1)
    {   
        //check for the signal from atrial processor
        detect();
        if (detected == 1){                       
            //check for normal ventricle signal
            processV(); 
             
        }
	}
}

//Initialize necessary systems
void SysInit(void){
    OSCCON=0b01010110; //4 MHz internal oscillator

    //Set up analog in for hi-f signal detection on A2 
    ANSELBbits.ANSB3 = 1; //pin B3 is analog
    TRISBbits.RB3 = 1; //B3 input
    
    //Set up comparator for hi-f signal detection  
    CM2CON0bits.C2OE = 1; //enable output onto C2OUT (A5)
    CM2CON0bits.C2POL = 1; //inverted
    CM2CON0bits.C2SP = 1; //normal power mode
    CM2CON0bits.C2R = 1; //Vin+ at FVR 
    CM2CON0bits.C2CH = 0b10; //Vin- at C12IN2- pin (RB3)
    CM2CON1bits.C2RSEL = 1; //select FVR
    CM2CON1bits.MC2OUT = 1; 
    CM2CON1bits.C2SYNC = 0;
    CM2CON0bits.C2OUT = 0;
    
    TRISAbits.TRISA5 = 0; //A5 (C2OUT) output
    
    //Set up fixed voltage reference of 2.048V
    VREFCON0bits.FVREN = 1;
    VREFCON0bits.FVRS = 0b01;
    
    //Set up analog in for EGM on A0
    ANSELAbits.ANSA0 = 1; //pin A0 is analog 
    TRISAbits.RA0 = 1; //A0 input
    
    //Set up ADC for EGM input
    ADCON0 = 0x00; // clear ADCON0, select AN0
    ADCON1 = 0b00000000;  //VSS, VDD ref, AN0 analog
    ADCON2 = 0b00001000; //left justified
    ADCON2bits.ACQT=001; //2 TAD
    ADCON2bits.ADCS=0b101; //FOSC/32, Tacq = 2Tad, Tad = 2*Tosc
    ADCON0bits.ADON = 0x01; //enable ADC module 
    
    //Set up digital out for pulse (5V)) on B1
    ANSELBbits.ANSB0 = 0; //digital
    TRISBbits.RB0 = 0; //output
    ANSELBbits.ANSB1 = 0;
    TRISBbits.RB1 = 0;
    ANSELBbits.ANSB2 = 0;
    TRISBbits.RB2 = 0;
    
    //Set up Timer0
    T0CONbits.T08BIT = 0; //16-bit counter
    T0CONbits.T0CS = 0; //use instruction cycle clock
    T0CONbits.PSA = 0; //use prescaler
    T0CONbits.T0PS = 0b001; //1:4 prescaler
    T0CONbits.TMR0ON = 0;
    TMR0H  = 0; 
    TMR0L  = 0;
    RCONbits.IPEN=1;            // Allow interrupt priorities
    INTCONbits.TMR0IF = 0;        // Clear any pending Timer 0 Interrupt indication
    INTCONbits.TMR0IE = 1;        // enable Timer 0 overflow Interrupt
    INTCONbits.GIE=1;           // enable interrupts

    /*
    //Set up timer1
    TMR1H  = One_Sec; 
    TMR1L  = 0;
    T1CON  = Timer1;		    // Configure Timer 1, timer enabled
    T1GCON = 0;                 // Timer 1 Gate function disabled
    RCONbits.IPEN=1;            // Allow interrupt priorities
    PIR1bits.TMR1IF = 0;        // Clear any pending Timer 1 Interrupt indication
    PIE1bits.TMR1IE = 1;        // enable Timer 1 Interrupt
    INTCONbits.GIE=1;           // enable interrupts 
    T1CONbits.TMR1ON=1;         //Turn timer on
     */
}

void processV(void){
    //slewThresh = 0; 
    //ampThresh = 3.7*255/5.5; 
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
            slewVals[i] = EGMVals[i]-EGMVals[i-1]; //replace with correct pin    
            slewSum += slewVals[i];
        }
        Delay1KTCYx(1); //1ms 
    }

    //calculate avg slew rate
    slewAvg = slewSum/4;
    //slewAvg = EGMVals[4] - EGMVals[0];
    
    //if amplitude + slew rate passed threshold, ventricle EGM detected/reset timer0 
    if(EGMVals[4]>ampThresh){   
        if (slewAvg>slewThresh){
            //LATBbits.LATB2 = 1; 
            stateV = 0;
            detected = 0;
            TMR0H = 0; //reset timer when detected
            TMR0L = 0;
            T0CONbits.TMR0ON = 0; //disable timer when detected
        } else{
            //LATBbits.LATB2 = 0;
            
        }
    } else {  //ventricular EGM not detected
        //LATBbits.LATB2 = 0; 
    }         
}

void producePulse(void){  
    //generate square pulse with PWM
    LATBbits.LATB0 = 1; //output to pin RB0, according to book ~0.1-2ms

    Delay10TCYx(100); //1ms pulse
    LATBbits.LATB0 = 0;
    TMR0L = 0; //reset everything
    TMR0H = 0;
    detected = 0;
    stateV = 0;
}

void detect(void){  //check for values past threshold on digital output A4 
    CM2CON0bits.C2ON = 1;
    if(CM2CON0bits.C2OUT ==1){ //result of comparator 
        detected = 1;
        LATBbits.LATB1 =1;
        T0CONbits.TMR0ON = 1; //start timer0 if detected
    }
    else{ 
        LATBbits.LATB1 = 0;
    }
    CM1CON0bits.C1ON = 0;
}
