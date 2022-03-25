/*******************************************************************************
  Input / Output Port COnfiguration Routine source File

  File Name:
    port_config.c

  Summary:
    This file includes subroutine for initializing GPIO pins as analog/digital,
    input or output etc. Also to PPS functionality to Remap-able input or output 
    pins

  Description:
    Definitions in the file are for dsPIC33CK256MP508 on Motor Control 
    Development board from Microchip
 
*******************************************************************************/
/*******************************************************************************
* © [2022] Microchip Technology Inc. and its subsidiaries

* Subject to your compliance with these terms, you may use Microchip software 
* and any derivatives exclusively with Microchip products. You are responsible
* for complying with 3rd party license terms applicable to your use of 3rd party
* software (including open source software) that may accompany Microchip 
* software. SOFTWARE IS ?AS IS.? NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR 
* STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF 
* NON-INFRINGEMENT, MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. 
* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP 
* HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.
* TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP?S TOTAL LIABILITY ON ALL
* CLAIMS RELATED TO THE SOFTWARE WILL NOT EXCEED AMOUNT OF FEES, IF ANY, YOU 
* PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
*
*******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include <xc.h>
#include "port_config.h"
#include "userparms.h"

// *****************************************************************************
/* Function:
    SetupGPIOPorts()

  Summary:
    Routine to set-up GPIO ports

  Description:
    Function initializes GPIO pins for input or output ports,analog/digital pins,
    remap the peripheral functions to desires RPx pins.

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */

void SetupGPIOPorts(void)
{
    // Reset all PORTx register (all inputs)
    #ifdef TRISA
        TRISA = 0xFFFF;
        LATA  = 0x0000;
    #endif
    #ifdef ANSELA
        ANSELA = 0x0000;
    #endif

    #ifdef TRISB
        TRISB = 0xFFFF;
        LATB  = 0x0000;
    #endif
    #ifdef ANSELB
        ANSELB = 0x0000;
    #endif

    #ifdef TRISC
        TRISC = 0xFFFF;
        LATC  = 0x0000;
    #endif
    #ifdef ANSELC
        ANSELC = 0x0000;
    #endif

    #ifdef TRISD
        TRISD = 0xFFFF;
        LATD  = 0x0000;
    #endif
    #ifdef ANSELD
        ANSELD = 0x0000;
    #endif

    #ifdef TRISE
        TRISE = 0xFFFF;
        LATE  = 0x0000;
    #endif
    #ifdef ANSELE
        ANSELE = 0x0000;
    #endif

    MapGPIOHWFunction();

    return;
}
// *****************************************************************************
/* Function:
    Map_GPIO_HW_Function()

  Summary:
    Routine to setup GPIO pin used as input/output analog/digital etc

  Description:
    Function initializes GPIO pins as input or output port pins,analog/digital 
    pins,remap the peripheral functions to desires RPx pins.

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */

void MapGPIOHWFunction(void)
{
    
    /* ANALOG SIGNALS */

    // Configure Port pins for Motor Current Sensing
    
    //Ia Out
    ANSELBbits.ANSELB2 = 1;
    TRISBbits.TRISB2 = 1;   //Pin 41: OA2OUT/AN1/AN7/ANA0/CMP1D/CMP2D/CMP3D/RP34/SCL3/INT0/RB2
    
    //Ib Out
    ANSELAbits.ANSELA4 = 1;
    TRISAbits.TRISA4 = 1;   //Pin 23: OA3OUT/AN4/CMP3B/IBIAS3/RA4
    
    
#ifdef INTERNAL_OPAMP_CONFIG
    
    //Ia- 
    ANSELBbits.ANSELB3 = 1;
    TRISBbits.TRISB3 = 1;   //Pin 43: PGD2/OA2IN-/AN8/RP35/RB3
    
    //Ia+ 
    ANSELBbits.ANSELB4 = 1;
    TRISBbits.TRISB4 = 1;   //Pin 45: PGC2/OA2IN+/RP36/RB4
    
    //Ib- 
    ANSELCbits.ANSELC1 = 1;
    TRISCbits.TRISC1 = 1;   //Pin 28: OA3IN-/AN13/CMP1B/ISRC0/RP49/PMA7/RC1
    
    //Ib+ 
    ANSELCbits.ANSELC2 = 1;
    TRISCbits.TRISC2 = 1;   //Pin 29: OA3IN+/AN14/CMP2B/ISRC1/RP50/PMD13/PMA13/RC2
    //Op-Amp Configuration
    AMPCON1Hbits.NCHDIS2 = 0;    //Wide input range for Op Amp #2
    AMPCON1Lbits.AMPEN2 = 1;     //Enables Op Amp #2
        
    AMPCON1Hbits.NCHDIS3 = 0;    //Wide input range for Op Amp #3
    AMPCON1Lbits.AMPEN3 = 1;     //Enables Op Amp #3
    
    AMPCON1Lbits.AMPON = 1;      //Enables op amp modules if their respective AMPENx bits are also asserted
     
#endif
    
    // Potentiometer #1 input - used as Speed Reference
    // POT1 
    ANSELDbits.ANSELD11 = 1;
    TRISDbits.TRISD11 = 1;   // PIN36: AN19/CMP2C/RP75/PMA0/PMALL/PSA0/RD11
    
    /*DC Bus Voltage Signals*/
    ANSELDbits.ANSELD10 = 1;
    TRISDbits.TRISD10 = 1; //PIN 38:AN18/CMP3C/ISRC3/RP74/PMD9/PMA9/RD10

    /* Digital SIGNALS */   
    // DIGITAL INPUT/OUTPUT PINS

    // Inverter Control - PWM Outputs
    // PWM1L : PIM #93  RP47/PWM1L/RB15
    // PWM1H : PIM #94  RP46/PWM1H/RB14
    // PWM2L : PIM #98  RP45/PWM2L/RB13
    // PWM2H : PIM #99  TDI/RP44/PWM2H/RB12
    // PWM3L : PIM #100 TCK/RP43/PWM3L/RB11
    // PWM3H : PIM #03  TMS/RP42/PWM3H/RB10
    TRISBbits.TRISB14 = 0 ;          
    TRISBbits.TRISB15 = 0 ;         
    TRISBbits.TRISB12 = 0 ;          
    TRISBbits.TRISB13 = 0 ;           
    TRISBbits.TRISB10 = 0 ;          
    TRISBbits.TRISB11 = 0 ;             
    
    // FAULT Pins
    // FAULT : PIM #18
    TRISDbits.TRISD8 = 1;           //PIN:49 - RP72/SDO2/PCI19/RD8

    // Debug LEDs
    // LED2 : PIM #01
     TRISEbits.TRISE9 = 0;          // PIN:44 - RE9
    // LED1 : PIM #60
    TRISEbits.TRISE8 = 0;           // PIN:42 - RE8

    // Push button Switches
            
    // Push Button : PIM #68
    TRISEbits.TRISE5 = 1;   // PIN24: RE5
	
	/** Diagnostic Interface for MCLV-2,MCHV-2/3,LVMCDB etc.
        Re-map UART Channels to the device pins connected to the following 
        PIM pins on the Motor Control Development Boards .
        UART_RX : PIM #49 (Input)
        UART_TX : PIM #50 (Output)   */
    _U1RXR = 71;
    _RP70R = 0b000001;
}
