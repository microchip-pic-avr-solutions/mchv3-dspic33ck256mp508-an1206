
/**
 * @file id_ref.h
 *
 * @brief This module implements Id current reference generation of Induction Motor.
 *
 * Component: ID REFERENCE GENERATION
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


#ifndef __ID_REF_H
#define __ID_REF_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "measure.h"

typedef struct
{
    int16_t   
        OmegaLs,    
        VoltStatorFlux,        /* BACK EMF voltage along Q-axis */
        SigmaLsDIqs,            /* Voltage drop across Q-axis */
        SigmaLsDIqsFiltConst,   /* Filter constant for Q-axis Inductive Drop */
        prevIqs,            /* Previous Iqs Reference */
        IdRef,              /* Id Current reference */
        IdRefFiltConst,     /* Filter constant for Id */
        IdRefMax,           /* upper limit on IdRef */
        RsIqs;              /* Stator Resistance voltage drop along Q-axis */
} FDWEAK_T;

typedef struct
{
    int16_t 
        IdRef,              /* Id Current reference */
        IdRefFilt;          /* Filtered Id Current reference */
    
    int32_t
        IdRefFiltStateVar;  /* Accumulation variable for IdRef filter */
    
    FDWEAK_T
        fdWeak;             /* Flux Weakening Structure */
    
} IDREFGEN_T;


extern IDREFGEN_T idRefGen;

void IdRefGenerationInit(IDREFGEN_T *);
void IdRefGeneration(IDREFGEN_T *);



#ifdef __cplusplus
}
#endif

#endif /* end of __ID_REF_H */
