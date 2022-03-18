/**
 * @file measure.h
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


#ifndef __MEASURE_H
#define __MEASURE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "general.h"

#define OFFSET_COUNT_BITS   (int16_t)10
#define OFFSET_COUNT_MAX    (int16_t)(1 << OFFSET_COUNT_BITS)
        
typedef struct
{
    int16_t
        offsetIa,       /* A phase current offset */
        offsetIb,       /* B phase current offset */
        Ia,             /* A phase Current Feedback */
        Ib,             /* B phase Current Feedback */
        counter,        /* counter */
        status;         /* flag to indicate offset measurement completion */

    int32_t
        sumIa,          /* Accumulation of Ia */
        sumIb;          /* Accumulation of Ib */
       
} MEASURE_CURRENT_T;



typedef struct
{
    int16_t 
        potValue;         /* Measure potentiometer */
    int16_t
        dcBusVoltage; 
    MEASURE_CURRENT_T
        current;     /* Current measurement parameters */
            
}MEASURE_T;

extern MEASURE_T measureInputs;

void MeasureCurrentOffset (MEASURE_T *);
void MeasureCurrentCalibrate (MEASURE_T *);
void MeasureCurrentInit (MEASURE_T *);
int16_t MeasureCurrentOffsetStatus (MEASURE_T *);


#ifdef __cplusplus
}
#endif

#endif /* end of __MEASURE_H */
