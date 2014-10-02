// ******************************************************************************************* //
// Include file for PIC24FJ64GA002 microcontroller. This include file defines
// MACROS for special function registers (SFR) and control bits within those
// registers.

// NAMES: JOSH HURLEY, URIEL GARCIA AND JESUS CHAVOLLA

#include "p24fj64ga002.h"
#include <stdio.h>
#include "lcd.h"

// ******************************************************************************************* //
// Configuration bits for CONFIG1 settings.
//
// Make sure "Configuration Bits set in code." option is checked in MPLAB.
//
// These settings are appropriate for debugging the PIC microcontroller. If you need to
// program the PIC for standalone operation, change the COE_ON option to COE_OFF.

_CONFIG1( JTAGEN_OFF & GCP_OFF & GWRP_OFF &
          BKBUG_ON & COE_ON & ICS_PGx1 &
          FWDTEN_OFF & WINDIS_OFF & FWPSA_PR128 & WDTPS_PS32768 )

// ******************************************************************************************* //
// Configuration bits for CONFIG2 settings.
// Make sure "Configuration Bits set in code." option is checked in MPLAB.

_CONFIG2( IESO_OFF & SOSCSEL_SOSC & WUTSEL_LEG & FNOSC_PRIPLL & FCKSM_CSDCMD & OSCIOFNC_OFF &
          IOL1WAY_OFF & I2C1SEL_PRI & POSCMOD_XT )

// ******************************************************************************************* //
// Defines to simply UART's baud rate generator (BRG) regiser
// given the osicllator freqeuncy and PLLMODE.

#define XTFREQ          7372800         	  // On-board Crystal frequency
#define PLLMODE         4               	  // On-chip PLL setting (Fosc)
#define FCY             (XTFREQ*PLLMODE)/2    // Instruction Cycle Frequency (Fosc/2)

#define BAUDRATE         115200
#define BRGVAL          ((FCY/BAUDRATE)/16)-1

// ******************************************************************************************* //

// Variable utilized to store the count that is incremented within the Timer 1 interrupt
// service routine every second.
// Notes:
//        1. This variable is declared are volatile becuase it is updated in the interrupt
//           service routine but will be read in the main execution loop.
//        2. Declared as unsigned char as the varaible will only store the values between
//           0 and 10.

volatile unsigned char cnt;
volatile unsigned char tens;
volatile unsigned char mins;
volatile unsigned char tenmins;

volatile unsigned char tenths;
volatile unsigned int hundredths;
volatile unsigned int state=0;
volatile unsigned int click = 0;
unsigned char command;
// ******************************************************************************************* //
// ******************************************************************************************* //

int main(void)
{
//    // TODO: Configure AD1PCFG register for configuring input pins between analog input
	// and digital IO.
    AD1PCFGbits.PCFG0=0;//sets RA0 to analog (RIGHT LED)
    AD1PCFGbits.PCFG1=0;//sets RA1 to analog ( LEFT LED)
    AD1PCFGbits.PCFG4=1;//sets RB2 to digital (SWITCH)

	// TODO: Configure TRIS register bits for Right and Left LED outputs.
    TRISAbits.TRISA0=0;//sets RIGHT LED as output
    TRISAbits.TRISA1=0;//sets LEFT LED as output
    TRISBbits.TRISB5=1; //sets sw1 as input

	// TODO: Configure LAT register bits to initialize Right LED to on.
    LATA=0;
    LATAbits.LATA0=0;//Green LED on (RIGHT LED)
    LATAbits.LATA1=1;//RED LED off

	// TODO: Configure ODC register bits to use open drain configuration for Right
	// and Left LED output.
    ODCAbits.ODA0=1;//enables open drain configuration for Green LED
    ODCAbits.ODA1=1;// enables open drain configuration for RED LED

	// TODO: Configure TRIS register bits for swtich input.
    TRISBbits.TRISB2=1;//input switch

	// TODO: Configure CNPU register bits to enable internal pullup resistor for switch
	// input.
    CNPU1bits.CN2PUE=1;
    CNPU1bits.CN3PUE=1;
    CNEN2bits.CN27IE = 1;
    CNPU1bits.CN6PUE = 1;
    CNEN1bits.CN6IE = 1;
    IFS1bits.CNIF = 0;
    IEC1bits.CNIE = 1;


	// TODO: Setup Timer 1 to use internal clock (Fosc/2).

        //    Fosc     = XTFREQ * PLLMODE
	//             = 7372800 * 4
	//             = 29491200
	//
	//    Fosc/2   = 29491200 / 2
	//             = 14745600

	// TODO: Setup Timer 1's prescaler to 1:256.

        //    Timer 1 Freq = (Fosc/2) / Prescaler
	//                 = 14745600 / 256
	//                 = 57600
	//

 	// TODO: Set Timer 1 to be initially off.


	// TODO: Clear Timer 1 value and reset interrupt flag
            TMR3=0;//Clears timer1
            IFS0bits.T1IF = 0;//clear interrupt flag timer1


	// TODO: Set Timer 1's period value register to value for 5 ms.
            //    PR1 = 5 ms / (1 / (T1 Freq))
            //        = 5e-3 / (1 / 57600)
            //        = 5e-3 * 57600
            //        = 288
             T3CONbits.TON = 1;//     (sets timer off)
              T3CONbits.TCKPS1=1;//    (sets timer prescaler to 1:256)
              T3CONbits.TCKPS0=1;//   (set timer prescaler to 1:256)
              T3CONbits.TCS= 0;//     (Fosc/2)
              PR3 = 288;//5ms delay


//

	// The following provides a demo configuration of Timer 1 in which
	// the Timer 1 interrupt service routine will be executed every 1 second
	PR1 = 57599;//1 second delay
	TMR1 = 0;//resets timer 1
	IFS0bits.T1IF = 0;
	IEC0bits.T1IE = 1;
	T1CONbits.TCKPS = 3;
	T1CONbits.TON = 1;

	// printf by default is mapped to serial communication using UART1.
	// NOTES:
	//        1. You must specify a heap size for printf. This is required
	//           becuase printf needs to allocate its own memory, which is
	//           allocated on the heap. This can be set in MPLAB by:
	//           a.) Selecting Build Options...->Project from the Project menu.
	//           b.) Selecting the MPLABLINK30 Tab.
	//           c.) Entering the size of heap, e.g. 512, under Heap Size
	//        2. printf function is advanced and using printf may require
	//           significant code size (6KB-10KB).
	//printf("Lab 2: Debugging Statements\n\r");

	// The following code will not work until you have implemented the
	// the required LCD functions defined within lcd.c
	LCDInitialize();
/*******************************/
////	//below is for testing MoceCursor command
//	LCDMoveCursor(0,2);
//	LCDPrintString("Hello");
//	LCDMoveCursor(1,2);
//	LCDPrintString("Test");
//	command = 0xC;
/*******************************/

	LCDPrintString("Running:");
	LCDMoveCursor(1,0);
	LCDPrintString("00:00.00");

        state=0;
        click = 0;

	while(1)
	{
             if(state==0)
              {
                            LCDMoveCursor(0,0);
                                 LCDPrintString("Running:");

                              if(click%2 == 1){
                                 LCDMoveCursor(0,0);
                                 LCDPrintString("Stopped:");
                             }


                        }


            LCDMoveCursor(1,0);
            LCDPrintChar(tenmins+'0');
            LCDMoveCursor(1,1);
            LCDPrintChar(mins+'0');
            LCDMoveCursor(1,2);
            LCDPrintChar(':');

            LCDMoveCursor(1,3);
            LCDPrintChar(tens+'0');
            //given
            LCDMoveCursor(1,4);
            LCDPrintChar(cnt+'0');
//            given above
            LCDMoveCursor(1,5);
             LCDPrintChar('.');

            tenths = TMR1/5759.9;
            LCDMoveCursor(1,6);
            LCDPrintChar((int)(tenths)+'0');



            if((TMR1/575.99)<10)
                hundredths=(int)(TMR1/575.99);

            if((TMR1/575.99)>=10)
            hundredths=((int)(TMR1/575.99))-(((int)(TMR1/5759.9))*10);

            LCDMoveCursor(1,7);
             LCDPrintChar(hundredths+'0');



        }
	return 0;
}

// ******************************************************************************************* //
// Defines an interrupt service routine that will execute whenever Timer 1's
// count reaches the specfied period value defined within the PR1 register.
//
//     _ISR and _ISRFAST are macros for specifying interrupts that
//     automatically inserts the proper interrupt into the interrupt vector
//     table
//
//     _T1Interrupt is a macro for specifying the interrupt for Timer 1
//
// The functionality defined in an interrupt should be a minimal as possible
// to ensure additional interrupts can be processed.
void __attribute__((interrupt, auto_psv)) _CNInterrupt(void) {

                        if(PORTBbits.RB2 == 0&&state==0)
                        {//if the button is pressed start running time
                            LCDMoveCursor(0,0);
                                 LCDPrintString("Running:");
                            click++;
                             LATAbits.LATA0=~(LATAbits.LATA0);//alternates behavior of GREEN LED
                             LATAbits.LATA1=~(LATAbits.LATA1);//alternates behavior of RED LED
                              TMR3 = 0;//resets timer 3
                              while(TMR3 < PR3);//5ms delay between button pressed

                             state=1;
                             T1CONbits.TON = 1;//turns timer 1 on
                              if(click%2 == 1){//if the button is pressed twice
                                 LCDMoveCursor(0,0);
                                 LCDPrintString("Stopped:");
                                 T1CONbits.TON = 0;//stops timer 1
                             }


                        }
                        

                        if(PORTBbits.RB2== 1 && state==1)//if the button is not pressed
                        {   state=0;
                            TMR3 = 0;
                            while(TMR3 < PR3);//5ms delay
                        }

                      
                     if(PORTBbits.RB5==0 && T1CONbits.TON==0){
        //if the watch is stopped and the reset button is pressed, everything in watch is resetted
                            TMR1=0;
                            cnt=0;
                            mins=0;
                            tens=0;
                            tenmins = 0;
                            hundredths=0;
                            tenths=0;

                        }
    IFS1bits.CNIF = 0;

}

void __attribute__((interrupt,auto_psv)) _T1Interrupt(void)
//void _ISR _T1Interrupt(void)

{
	// Clear Timer 1 interrupt flag to allow another Timer 1 interrupt to occur.
	IFS0bits.T1IF = 0;

	// Updates cnt to wraparound from 9 to 0 for this demo.
        if(mins==9&&tens==5&&cnt==9)
        {
            tenmins=(tenmins<9)?(tenmins+1):0;//counts every ten minutes
        }
        if(tens==5&&cnt==9)
        {
            mins=(mins<9)?(mins+1):0;//counts every minute
        }
        if(cnt==9)
        { tens=(tens<5)?(tens+1):0;//counts every ten seconds
        }

        cnt = (cnt<9)?(cnt+1):0;//counts the seconds


/*******************************/
	//make the LCD blink;
////	command ^= 0x4;
////	WriteLCD(command, 0, 40);
/*******************************/
}

// ******************************************************************************************* //

