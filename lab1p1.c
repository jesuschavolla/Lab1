// ******************************************************************************************* //
// Include file for PIC24FJ64GA002 microcontroller. This include file defines
// MACROS for special function registers (SFR) and control bits within those
// registers.

#include "p24fj64ga002.h"
#include <stdio.h>

// ******************************************************************************************* //
// Configuration bits for CONFIG1 settings.
//
// Make sure "Configuration Bits set in code." option is checked in MPLAB.
// This option can be set by selecting "Configuration Bits..." under the Configure
// menu in MPLAB.
//
// These settings are appropriate for debugging the PIC microcontroller. If you need to
// program the PIC for standalone operation, change the COE_ON option to COE_OFF.

_CONFIG1( JTAGEN_OFF & GCP_OFF & GWRP_OFF &
          BKBUG_ON & COE_ON & ICS_PGx1 &
          FWDTEN_OFF & WINDIS_OFF & FWPSA_PR128 & WDTPS_PS32768 )

// ******************************************************************************************* //
// Configuration bits for CONFIG2 settings.
// Make sure "Configuration Bits set in code." option is checked in MPLAB.
// This option can be set by selecting "Configuration Bits..." under the Configure
// menu in MPLAB.

_CONFIG2( IESO_OFF & SOSCSEL_SOSC & WUTSEL_LEG & FNOSC_PRIPLL & FCKSM_CSDCMD & OSCIOFNC_OFF &
          IOL1WAY_OFF & I2C1SEL_PRI & POSCMOD_XT )
// ******************************************************************************************* //

void DebounceDelay() {

	// TODO: Use Timer 1 to create a precise 5 ms delay.
	int i;
	for(i=0; i<PR3; i++);
}

// ******************************************************************************************* //

int main(void)
{
	// ****************************************************************************** //

	// TODO: Configure AD1PCFG register for configuring input pins between analog input
	// and digital IO.
    AD1PCFGbits.PCFG0=0;//sets RA0 to analog (RIGHT LED)
    AD1PCFGbits.PCFG1=0;//sets RA1 to analog ( LEFT LED)
    AD1PCFGbits.PCFG4=1;//sets RB2 to digital (SWITCH)

	// TODO: Configure TRIS register bits for Right and Left LED outputs.
    TRISAbits.TRISA0=0;//sets RIGHT LED as output
    TRISAbits.TRISA1=0;//sets LEFT LED as output
   
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
    CNPU1bits.CN6PUE=1;
  
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
            T3CONbits.TON = 0;//     (sets timer off)
	    T3CONbits.TCKPS1=1;//    (sets timer prescaler to 1:256)
            T3CONbits.TCKPS0=1;//   (set timer prescaler to 1:256)
	    T3CONbits.TCS= 0;//     (Fosc/2)

	// TODO: Clear Timer 1 value and reset interrupt flag
            TMR3=0;//Clears timer1
            IFS0bits.T1IF = 0;//clear interrupt flag timer1
            
            
	// TODO: Set Timer 1's period value register to value for 5 ms.
            //    PR1 = 5 ms / (1 / (T1 Freq))
            //        = 5e-3 / (1 / 57600)
            //        = 5e-3 * 57600
            //        = 288
            PR3 = 288;//timer 1's period for 5ms
           
           int state=0;

            while(1)
            {

               
                switch(state)
                {
                    case 0:
                        DebounceDelay();//5ms delay
                        while(PORTBbits.RB2 == 0)
                        {
                             LATAbits.LATA0=1;//turn off GREEn
                             LATAbits.LATA1=0;//TURN on red
                        }
                        
                        
                        state = 1;
                    break;
                    case 1:
                        DebounceDelay();//5ms delay
                        while(PORTBbits.RB2 == 1);
                        
                            state = 2;
                    break;
                    case 2:
                       DebounceDelay();//5ms delay
                        while(PORTBbits.RB2 == 0)
                        {
                            LATAbits.LATA0=0;//turn off GREEn
                            LATAbits.LATA1=1;//TURN on red
                        }
                        
                       
                        state=3;
                    break;
                    case 3:
                         DebounceDelay();//5ms delay
                        while(PORTBbits.RB2 == 1);
                       
                        state = 0;
                    break;
//
                }
            }
	return 0;
}


// *******************************************************************************************