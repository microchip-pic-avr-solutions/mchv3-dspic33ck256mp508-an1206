/*******************************************************************************
  ADC Configuration Routine Header File

  File Name:
    adc.h

  Summary:
    This header file lists ADC Configuration related functions and definitions

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
#ifndef _ADC_H
#define _ADC_H

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
#include "userparms.h"
// *****************************************************************************
// *****************************************************************************
// Section: Constants
// *****************************************************************************
// *****************************************************************************
// ADC MODULE Related Definitions
#define ADCBUF_INV_A_IPHASE1    -ADCBUF1
#define ADCBUF_INV_A_IPHASE2    -ADCBUF4

        
#define ADCBUF_SPEED_REF_A      (int16_t)(ADCBUF19 >> 1)
#define ADCBUF_VBUS_A           (int16_t)(ADCBUF18 >> 1)

/* This defines number of current offset samples for averaging 
 * If the 2^n samples are considered specify n(in this case 2^7(= 128)=> 7*/
#define  CURRENT_OFFSET_SAMPLE_SCALER         7
        
#define EnableADCInterrupt()   _ADCAN19IE = 1
#define DisableADCInterrupt()  _ADCAN19IE = 0
#define ClearADCIF()           _ADCAN19IF = 0
#define ClearADCIF_ReadADCBUF() ADCBUF19
        
#define _ADCInterrupt _ADCAN19Interrupt  

        
// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines
// *****************************************************************************
// *****************************************************************************
void InitializeADCs(void);

#ifdef __cplusplus  // Provide C++ Compatibility
    }
#endif
#endif      // end of ADC_H

