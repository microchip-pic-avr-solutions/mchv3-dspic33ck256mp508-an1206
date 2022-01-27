/**
 * @file id_ref.c
 *
 * @brief This module implements field weakening  algorithm to 
 * generate id current reference (FOC) required for extended speed operation of 
 * Induction Motor.
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


#include <stdint.h>

/* _Q15abs and _Q15sqrt function use */
#include <libq.h>
#include "motor_control_noinline.h"
#include "id_ref.h"
#include "userparms.h"
#include "estim.h"
#include "control.h"

IDREFGEN_T idRefGen;
static void FdWeakControl(FDWEAK_T *);

/**
* <B> Function: IdRefGenerationInit(IDREFGEN_T * )  </B>
*
* @brief Function to reset variables used for Field Weakening Control.
*        .
*
* @param Pointer to the data structure containing Field Weakening Control
*        parameters.
* @return   none.
* @example
* <CODE> IdRefGenerationInit(&IdRefGen); </CODE>
*
*/
void  IdRefGenerationInit(IDREFGEN_T *pIdRefGen)
{
    pIdRefGen->fdWeak.IdRefMax = Q15_IDREF_BASESPEED;
    pIdRefGen->fdWeak.IdRef = Q15_IDREF_BASESPEED;
    pIdRefGen->IdRefFiltStateVar = pIdRefGen->fdWeak.IdRef<<15;
    pIdRefGen->fdWeak.IdRefFiltConst = IDREF_FILT_CONST;
    pIdRefGen->fdWeak.SigmaLsDIqsFiltConst = SIGMALSIQS_FILT_CONST;
}

/**
* <B> Function: IdRefGeneration(IDREFGEN_T * )  </B>
*
* @brief Function implementing Field Weakening Control
*
* @param Pointer to the data structure containing Field Weakening Control
*        parameters.
* @return   none.
* @example
* <CODE> IdRefGeneration(&idRefGen); </CODE>
*
*/
void IdRefGeneration(IDREFGEN_T *pIdRefGen)
{
    
    /* Calculate Flux Weakening Control current */
    
    FdWeakControl(&pIdRefGen->fdWeak);
            
  
    pIdRefGen->IdRef = pIdRefGen->fdWeak.IdRef;
   
    /* filtering the generated id reference before giving it to control loop */
    pIdRefGen->IdRefFiltStateVar +=
            __builtin_mulss((pIdRefGen->IdRef - pIdRefGen->IdRefFilt),
                                pIdRefGen->fdWeak.IdRefFiltConst);

    pIdRefGen->IdRefFilt =
            (int16_t)(pIdRefGen->IdRefFiltStateVar >> 15);
}

/**
* <B> Function: FdWeakControl(FDWEAK_T * )  </B>
*
* @brief Function implementing Field Weakening Control
*
* @param Pointer to the data structure containing Field Weakening Control
*        parameters.
* @return   none.
* @example
* <CODE> FdWeakControl(&fieldWeak); </CODE>
*
*/
static void FdWeakControl(FDWEAK_T *pFdWeak)
{    
    
    int16_t
        dIqs,           /* Delta_iqs = Current - previous iqs */
        sigmaLsDIqs,        /* Inductance drop along q axis */
        mds_sq,         /* Square of modulation index along D axis */
        mqs_ref,        /* Modulation index reference along Q axis */
        vqs_ref;        /* Q-axis voltage reference */

    /* calculating available Vq in the system, Vq = sqrt(Vmax^2- Vd^2) */
    mds_sq = (int16_t)(__builtin_mulss(vdq.d,vdq.d) >> 15);

    if(mds_sq >= VMAX_SQR_FDWEAK)
    {
        mds_sq = VMAX_SQR_FDWEAK;
    }

    mqs_ref = _Q15sqrt(VMAX_SQR_FDWEAK - mds_sq);

    /* vqs_ref is multiplied by DC bus voltage for normalization and scale it 
     * down by a factor of two. This scaling is required 
     * to prevent overflow/saturation of vqs_ref calculation in a few corner cases*/
    vqs_ref = (int16_t)(__builtin_mulss(mqs_ref, measureInputs.dcBusVoltage) >> (15+1));
    /*
     * stator voltage equations:
     * Vds = Rs * Ids ++ SigmaLs*dIds/dt - Omega * SigmaLs * Iqs
     * Vqs = Rs * Iqs + SigmaLs*dIqs/dt + Omega * Ls * Ids
     * 
     */
    
     /*stator voltage drop along q-axis*/
    pFdWeak->RsIqs =
            (int16_t)(__builtin_mulss(ctrlParm.qVqRef, motorParm.qRs) >>
                            NORM_RS_SCALE_SHIFT);
    
     dIqs = ctrlParm.qVqRef - pFdWeak->prevIqs;
    sigmaLsDIqs =
            (int16_t)(__builtin_mulss(dIqs, motorParm.qSigmaLsDt) >>
            NORM_SIGMA_LSDTBASE_SCALE_SHIFT);
    
    /* filtering the sigmaLsDIqs magnitude */
    pFdWeak->SigmaLsDIqs =
            (int16_t)((__builtin_mulss(pFdWeak->SigmaLsDIqs,
                                        (32767-pFdWeak->SigmaLsDIqsFiltConst)) +
                    __builtin_mulss(sigmaLsDIqs,pFdWeak->SigmaLsDIqsFiltConst)) >> 15);
    
    /* Calculatin Omega * Ls * Ids 
     * Omega * Ls * Ids = Vqs - Rs * Iqs - SigmaLs*dIqs/dt
     * since vqs_ref is in Q14 Foramt,converted Rs * Iqs and SigmaLs*dIqs/dt 
     * to Q14 format and the result comes converted back to Q15 format   */
    pFdWeak->VoltStatorFlux = 
            ((vqs_ref - 
                    ((pFdWeak->RsIqs + pFdWeak->SigmaLsDIqs) >> 1))<<1);
    /* omega*Ls is right shifted with 15, but not NORM_LS_SCALE to avoid 
     * numerical overflow/saturation. */
    pFdWeak->OmegaLs =  (int16_t)(__builtin_mulss(estimator.qOmegaMr, motorParm.qLs)>> 15);
    
    /* Since OmegaLs is in Q.NORM_LS_SCALE system, shifting VoltStatorFlux
     * to match the number system for the division operation later 
     * Computing Idref only if speed is more than half of rated speed to avoid overflows*/
    if(estimator.qOmegaMr > (Q15_NOMINALSPEED>>1))
    {    
        pFdWeak->IdRef = (int16_t)(__builtin_divsd(((int32_t)pFdWeak->VoltStatorFlux <<(15-NORM_LS_SCALE)),
                                        pFdWeak->OmegaLs));
    }
    else
    {
        pFdWeak->IdRef = Q15_IDREF_BASESPEED;
    }

    /* Clamp id_ref to 0 on Negative side */
    if(pFdWeak->IdRef < 0)
    {
        pFdWeak->IdRef = 0;
    }
    else if(pFdWeak->IdRef > pFdWeak->IdRefMax)
    {
        /* Clamp id_ref to Q15_IDREF_BASESPEED on Positive side  */
        pFdWeak->IdRef = pFdWeak->IdRefMax;
    }
    
    pFdWeak->prevIqs = ctrlParm.qVqRef;
}

