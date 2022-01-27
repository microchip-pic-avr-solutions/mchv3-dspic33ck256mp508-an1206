
/**
 * @file measure.c
 *
 * @brief This module has functions for signal conditioning of measured
 *        analog feedback signals.
 *
 * Component: MEASURE
 *
 */

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

#include <stdint.h>
#include <stdbool.h>

#include "measure.h"
#include "adc.h"

/**
* <B> Function: MeasureCurrentInit(MEASURE_CURRENT_T *)  </B>
*
* @brief Function to reset variables used for current offset measurement.
*        .
*
* @param Pointer to the data structure containing measured currents.
* @return none.
* @example
* <CODE> MeasureCurrentInit(&current); </CODE>
*
*/
void MeasureCurrentInit(MEASURE_T *pMotorInputs)
{
    MEASURE_CURRENT_T *pCurrent;
    
    pCurrent = &pMotorInputs->current;
    
    pCurrent->counter = 0;
    pCurrent->sumIa = 0;
    pCurrent->sumIb = 0;
    pCurrent->status = 0;
}

/**
* <B> Function: MeasureCurrentOffset(MEASURE_CURRENT_T *)  </B>
*
* @brief Function to compute current offset after measuring specified number of
*        current samples and averaging them.
*        .
* @param Pointer to the data structure containing measured current.
* @return none.
* @example
* <CODE> MeasureCurrentOffset(&current); </CODE>
*
*/
void MeasureCurrentOffset(MEASURE_T *pMotorInputs)
{
    MEASURE_CURRENT_T *pCurrent;
    
    pCurrent = &pMotorInputs->current;
    
    pCurrent->sumIa += pCurrent->Ia;
    pCurrent->sumIb += pCurrent->Ib;
    pCurrent->counter++;

    if (pCurrent->counter >= OFFSET_COUNT_MAX)
    {
        pCurrent->offsetIa = (int16_t)(pCurrent->sumIa >> OFFSET_COUNT_BITS);
        pCurrent->offsetIb = (int16_t)(pCurrent->sumIb >> OFFSET_COUNT_BITS);
        
        pCurrent->counter = 0;
        pCurrent->sumIa = 0;
        pCurrent->sumIb = 0;
        pCurrent->status = 1;
    }
}
/**
* <B> Function: MeasureCurrentCalibrate(MEASURE_CURRENT_T *)  </B>
*
* @brief Function to compensate offset from measured current samples.
*        .
* @param Pointer to the data structure containing measured current.
* @return none.
* @example
* <CODE> MeasureCurrentCalibrate(&current); </CODE>
*
*/
void MeasureCurrentCalibrate(MEASURE_T *pMotorInputs)
{
    MEASURE_CURRENT_T *pCurrent;
    
    pCurrent = &pMotorInputs->current;
    
    pCurrent->Ia = pCurrent->Ia - pCurrent->offsetIa;
    pCurrent->Ib = pCurrent->Ib  - pCurrent->offsetIb;
}

int16_t MeasureCurrentOffsetStatus (MEASURE_T *pMotorInputs)
{
    return pMotorInputs->current.status;
}
