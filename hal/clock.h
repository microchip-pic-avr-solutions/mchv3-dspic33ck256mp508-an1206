/*******************************************************************************
  Oscillator Configuration Routine Header File

  File Name:
    clock.h

  Summary:
    This header file lists Clock Configuration related functions and definitions

  Description:
    Definitions in the file are for dsPIC33CK256MP508 MC PIM plugged onto
    Motor Control Development board from Microchip

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
#ifndef _CLOCK_H
#define _CLOCK_H

#ifdef __cplusplus  // Provide C++ Compatability
    extern "C" {
#endif
// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include <xc.h>
#include <stdint.h>
// *****************************************************************************
// *****************************************************************************
// Section: Constants
// *****************************************************************************
// *****************************************************************************
// OSCILLATOR Related Definitions
// Oscillator frequency (Hz) - 200,000,000 Hz
#define FOSC                    200000000UL
// Oscillator frequency (MHz) - 200MHz
#define FOSC_MHZ                200U     
// Instruction cycle frequency (Hz) - 100,000,000 Hz
#define FCY                     100000000UL
// Instruction cycle frequency (MHz) - 100 MHz
#define FCY_MHZ                 100U  
        
// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines
// *****************************************************************************
// *****************************************************************************
void InitOscillator(void);
void EnableREFCLKOutput(uint16_t);        
        
#ifdef __cplusplus  // Provide C++ Compatibility
    }
#endif
#endif      // end of CLOCK_H


