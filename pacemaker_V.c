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
#define One_Sec  0x80 	//Load high byte of timer 1 with this for 1 second
// Timer 1 clock source is crystal oscillator on T1OS1/T1OS0, 1:1,
// Dedicated enabled, Do Not Synch, Enable Timer1
#define Timer1  0x89

//Variable definitions
unsigned char A_thresh; //threshold for positive detection of atrial electrogram
unsigned char V_thresh; //threshold for positive detection of ventricle electrogram
unsigned char pulse_level; //generated pulse signal strength
int stateV;
int signalThresh = 3; //3V 
int timer1;
int timer2; 
int RP;
int EGMmax;

#define EGMThresh 
//Function definitions
void processV(void);
void producePulse(void);
int detect(void);
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
    if (PIR1bits.TMR1IF)            // If timer overflowed
    {
        TMR1H  |= One_Sec;          // Reset timer to one second
        timer1 = 1;                 //this says that ventricular event has been detected 
        PIR1bits.TMR1IF = 0;        // Clear timer flag
        INTCONbits.INT0IF = 0;      // Clear interrupt flag
    }
}


void main(void)
{
    SysInit();
    timer2 = 0; //implement this w/ internal oscillator
    timer1 = 0;
    while(1)
    {
        
        
        stateV = 0;
        //check for the signal from atrial processor
        if (detect()){ 
            //restart timer1
            //TMR1H = 0; 
            //TMR1L = 0; 
            
            //while(timer2<RP){}; //don't do anything during refractory period
            
            //check for normal ventricle signal
            processV();    
            //producePulse();
        }	
	}
}

//Initialize necessary systems
void SysInit(void){
    OSCCON=0b01010110; //4 MHz internal oscillator

    //Set up analog in for hi-f signal on A2 
    //ANSELAbits.ANSA2 = 1; //pin A2 is analog
    //TRISAbits.RA2 = 1; //A2 input
    //ADCON0bits.CHS=0b00010; //AN2
    
    //Set up ADC for hi-f signal 
    

    //Set up analog in for EGM on A0
    ANSELAbits.ANSA0 = 1; //pin A0 is analog 
    TRISAbits.RA0 = 1; //PORTA all input
    
    //Set up ADC for EGM input
    ADCON0 = 0x00; // clear ADCON0, select AN0
    ADCON1 = 0b00000000;  //VSS, VDD ref, AN0 analog
    ADCON2 = 0b00001000; //left justified
    ADCON2bits.ACQT=001; //2 TAD
    ADCON2bits.ADCS=010; //FOSC/32tified, Tacq = 2Tad, Tad = 2*Tosc
    ADCON0bits.ADON = 0x01; //enable ADC module 
    
    //Set up DAC for output
    //VREFCON1 = 0b11000001; //enable dac, use Vdd/Vref- as refs, don't output on DACOUT pin
    //VREFCON2 = ADRESH>>3; //set DAC level equal to 5MSB of ADC result
    
    //Set up digital out for pulse (5V))
    ANSELB = 0x00; //digital
    TRISB = 0x00; //output
    
    //Set up LCD
    ANSELD = 0x00;
    TRISD = 0x00; //Digital out

    //LCDInit(); //Start LCD
    
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
    
    //Set up timer2 //need to do this
}

void processV(void){
    int slewThresh = 0; 
    int ampThresh = 1*255/5; //1V 
    //int dt = 0.01;
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
        if(i<4){
            slewVals[i] = (EGMVals[i+1]-EGMVals[i]); //replace with correct pin    
            slewSum += slewVals[i];
        }
        //Delay10KTCYx(1); //10ms 
        Delay10TCYx(1); //1ms?
    }

    //calculate avg slew rate
    slewAvg = slewSum/4;
    //if amplitude + slew rate passed threshold, stateV = 1  
    if(EGMVals[4]>ampThresh && slewAvg>slewThresh){   
        stateV = 1;
        LATBbits.LATB1 = 1; 
        //restart timer2
    } else {
        stateV = 0;
        LATBbits.LATB1 = 0; 
    } 
         
}

void producePulse(void){  //this could be done on an interrupt?
    //if atrial sensed but ventricle not sensed and timer1 too long, output signal
        LATBbits.LATB0 = 1; //output to pin RB0, according to book ~0.1-2ms
        timer2 = 0;
        Delay10KTCYx(10); //delay 1s I think
        //Delay10TCYx(100); //every 100 = 1ms
        LATBbits.LATB0 = 0;
    /*
    int expected = 0;
    if(timer1>expected && stateV = 0){ //need to define expected
        //generate square pulse with PWM
        LATBbits.LATB0 = 1; //output to pin RB0, according to book ~0.1-2ms
        timer2 = 0;
        Delay10TCYx(100); //every 100 = 1ms
        LATBbits.LATB0 = 0;        
    }
    else{
        timer2 = 0;
    }
     */
}

int detect(void){  //check for values past threshold on analog pin A2
    //THIS ADC DOESN'T WORK YET
    //ADCON0bits.CHS=0b00010; //AN2
    //ADCON0bits.GO=1; //Start conversion
    //while(ADCON0bits.GO==1){}; //Wait for finish
    
    if(ADRESH > 128){ //digital in of signal 
        Delay10TCYx(100); //every 100 = 1ms
        return 1;
    }
    else {
        return 0; 
    }
    
    //return 1;
}
