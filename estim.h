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
#ifndef __ESTIM_H
#define __ESTIM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "motor_control_noinline.h"
/* Estimator Parameter data type

  Description:
    This structure will host parameters related to angle/speed estimator
    parameters.
 */
typedef struct
{
    /* Integration constant */
    int16_t qDeltaT;
    /* angle of estimation */
    int16_t qRho;
    /* internal variable for angle */
    int32_t qRhoStateVar;
    /* synchronous speed estimation */
    int16_t qOmegaMr;
    /* difference Ialpha */
    int16_t qDIalpha;
    /* difference Ibeta */
    int16_t qDIbeta;
    /* Phase voltage valpha*/
    int16_t qValpha;
    /* Phase voltage Vbeta*/
    int16_t qVbeta;
    /* counter in Last DI tables */
    int16_t qDiCounter;
    /* dI*Ls/dt alpha */
    int16_t qVIndalpha;
    /* dI*Ls/dt beta */
    int16_t qVIndbeta;
    /* BEMF d filtered */
    int16_t qEsdf;
    /* state variable for BEMF d Filtered */
    int32_t qEsdStateVar;
    /* BEMF q filtered */
    int16_t qEsqf;
    /* state variable for BEMF q Filtered */
    int32_t qEsqStateVar;
    /* filter constant for d-q BEMF */
    int16_t qKfilterEsdq;
    /* Estimated filtered rotor speed */
    int16_t qVelEstim;
    /* Filter constant for Estimated rotor speed */
    int16_t qVelEstimFilterK;
    /* State Variable for Estimated rotor speed */
    int32_t qVelEstimStateVar;
    /* dIalphabeta/dt */
    int16_t qDIlimitLS;
    /* dIalphabeta/dt */
    int16_t qDIlimitHS;
    /*  last  value for Ialpha */
    int16_t qLastIalphaHS[8];
    /* last  value for Ibeta */
    int16_t qLastIbetaHS[8];
    /* estimator angle initial offset */
    int16_t qRhoOffset;
    /* Rotor D-axis flux*/
    int16_t qinvPsiRd;
    /* Slip Speed Estimation*/
    int16_t qOmegaSlip;
    /* Magnetizing Component of Current Imr Calculation*/
    int16_t qImrEstim;
    /* Filter constant for Estimated Imr */
    int16_t qImrEstimFilterK;
    /* State Variable for Estimated Imr */
    int32_t qImrEstimStateVar;
    /* Estimated Rotor Speed*/
    int16_t qOmegaRotor;

} ESTIM_PARM_T;
/* Motor Estimator Parameter data type

  Description:
    This structure will host motor parameters parameters required by angle
    estimator.
 */
typedef struct
{
    /* Rs value - stator resistance */
    int16_t qRs;
    /* Ls/dt value - stator inductance / dt - variable with speed */
    int16_t qSigmaLsDt;
    /* Ls/dt value - stator inductance / dt for base speed (nominal) */
    int16_t qSigmaLsDtBase;
    /* Lr/Lm^2 Value */
    int16_t qInvLmSqrbyLr;
    /* Lr/Lm^2 base Value */
    int16_t qInvLmSqrbyLrBase; 
    /*inverse Rotor Time Constant (Tr)*/
    int16_t qInvTr;
    /* Ls Value - Stator Inductance*/
    int16_t qLs;
} MOTOR_ESTIM_PARM_T;

extern ESTIM_PARM_T estimator;
extern MOTOR_ESTIM_PARM_T motorParm;

void Estim(void);
void InitEstimParm(void);


#ifdef __cplusplus
}
#endif

#endif /* __ESTIM_H */
