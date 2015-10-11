/* BME 464 - Ventricle processor code 
 * Student: Jessica Yan
 * PIC18F46K22 USED FOR THIS ELECTRODE
 */
#include "Lcd.h"
#include <delays.h>
#include <p18f46k22.h>
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
int timer2; 

#define EGMThresh 
//Function definitions
void processV(void);
void producePulse(void);
int detect(void);
void SysInit(void);



void main(void)
{
    SysInit();
    //LCDClear();
    timer2 = 0; //implement this w/ internal oscillator
    //LCDGoto(0,0);
    //LCDWriteStr("Started");
    while(1)
    {
        stateV = 0;
        //ADC conversion
        ADCON0bits.GO_DONE = 1; 
        while(ADCON0bits.GO_DONE!=0); //wait to finish conversion
        
        if(ADRESH>200){ //ADRESH = 8MSB
            LATBbits.LATB0 = 1; //output to pin RB0
            //LCDGoto(0,0);
            //LCDPutByte(ADRESH);
        }
        else{
            LATBbits.LATB0 = 0;
        }
      
        //check for the signal from atrial processor
    /*    if (detect()){ 
            //start timer1
            
            
            while(timer2<RP); //don't do anything 
            
            //check for normal ventricle signal
            processV();    
            producePulse();
        }
	*/
	};
}

//Initialize necessary systems
void SysInit(void){
    OSCCON=0b01010110; //4 MHz internal oscillator

    //Set up analog in for hi-f signal 
    //ANSELBbits.ANSB0=0; //Digital
    //TRISAbits.RA4=1; //Input
    //TRISBbits.RB0=1; //Input

    //Set up ADC for hi-f signal 
    

    //Set up analog in for EGM
    //ANSELA=0b11111111; //PORTA analog
    ANSELAbits.ANSA0 = 1; //pin A0 is analog 
    TRISA = 0b11111111; //PORTA all input
    
    //Set up ADC for EGM
    ADCON1 = 0b00001101;  //Vref-, VDD ref, AN0 analog
    ADCON0 = 0x00; // clear ADCON0, select AN0
    ADCON2 = 0b00001000; //left justified                                                                                                                                       tified, Tacq = 2Tad, Tad = 2*Tosc
    ADCON0bits.ADON = 0x01; //enable ADC module 
    
    //Set up DAC to test ADC
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
    T1CON  = Timer1;		// Configure Timer 1
    T1GCON = 0;                 // Timer 1 Gate function disabled
    RCONbits.IPEN=1;            // Allow interrupt priorities
    PIR1bits.TMR1IF = 0;        // Clear any pending Timer 1 Interrupt indication
    PIE1bits.TMR1IE = 1;        // Enable Timer 1 Interrupt
    INTCONbits.GIE=1;           // Enable interrupts

    //Set up timer2 //need to do this
}



void processV(void){
    int slewThresh = 0; 
    int ampThresh = 0;
    int dt = 0.01;
    int EGMVals[5];
    int slewVals[4];
    int slewSum = 0;
    int i;
    for(i=0; i<5; i++){
            EGMVals[i] = PORTB; //replace with correct pin
        if(i<4){
            slewVals[i] = (EGMVals[i+1]-EGMVals[i])/dt; //replace with correct pin    
            slewSum += slewVals[i];
        }
        Delay10KTCYx(1); //10ms 
    }
 
    /*
    //calculate avg slew rate
    int slewAvg = slewSum/4;
    //if amplitude + slew rate passed threshold, stateA = 1
    if(EGMVals[5]>ampThresh && slewAvg>slewThresh){
        stateV = 1;
    }
     */
}

/*void producePulse(void){
    //if atrial sensed but ventricle not sensed and timer too long, output signal
    if(timer1>expected && stateV = 0){ //need to define expected
        //generate square pulse with PWM
        timer2 = 0;
    }
    else{ timer2 = 0};
}*/

/*
int detect(void){
    //check for values past threshold for certain duration
    int i=0; 
    while (i<10){
        if (PORTC > signalThresh){ //analog in of signal 
            i++;
            Delay10TCYx(100); //every 100 = 1ms
        }
        else {
            return 0; 
        }
    }
    return 1;
}
*/

/*
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
        RTCIncSec();                // Increment count
        PIR1bits.TMR1IF = 0;        // Clear timer flag
        INTCONbits.INT0IF = 0;      // Clear interrupt flag
    }
}
*/
