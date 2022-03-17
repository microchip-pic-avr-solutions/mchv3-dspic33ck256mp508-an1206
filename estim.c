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

#include <libq.h>
#include "motor_control_noinline.h"
#include "userparms.h"
#include "estim.h"
#include "control.h"
#include "measure.h"

#define DECIMATE_NOMINAL_SPEED    NORM_VALUE(NOMINAL_SPEED_RPM/10,MC1_PEAK_SPEED_RPM)

/** Variables */
ESTIM_PARM_T estimator;
MOTOR_ESTIM_PARM_T motorParm;
MC_ALPHABETA_T bemfAlphaBeta;
MC_DQ_T bemfdq;
MC_SINCOS_T sincosThetaEstimator;

// *****************************************************************************

/* Function:
    Estim()

  Summary:
    Motor speed and angle estimator

  Description:
    Estimation of the speed of the motor and field angle based on inverter
    voltages and motor currents.

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
void Estim(void) 
{
    int16_t deltaEs;
    uint16_t index = (estimator.qDiCounter - 7)&0x0007;
    /* valphabeta.alpha and valphabeta.beta are modulation index and multiplying with 
     * dcbus voltage to convert to voltage in q15 for*/
    estimator.qValpha = (int16_t) (__builtin_mulss(valphabeta.alpha, 
                                  measureInputs.dcBusVoltage) >> 15);
    
    estimator.qVbeta = (int16_t) (__builtin_mulss(valphabeta.beta, 
                                  measureInputs.dcBusVoltage) >> 15);
    
    /*dIalpha = Ialpha-oldIalpha,  dIbeta  = Ibeta-oldIbeta*/
    
    if (_Q15abs(estimator.qVelEstim) < PLL_VELOCITY_FILTER_THRESHOLD) 
    {
        /*At lower speed the granularity of change is higher. Hence, the difference
        * is calculated between two sampled values that are 8 
        * number of PLL execution steps apart */
        estimator.qDIalpha = (ialphabeta.alpha -
                estimator.qLastIalphaHS[index]);
        /* Limit the change in current to dIlimitLS in order to
         * reduce the effect of noisy current measurements */
        if (estimator.qDIalpha > estimator.qDIlimitLS) 
        {
            estimator.qDIalpha = estimator.qDIlimitLS;
        }
        if (estimator.qDIalpha<-estimator.qDIlimitLS) 
        {
            estimator.qDIalpha = -estimator.qDIlimitLS;
        }
        estimator.qVIndalpha = (int16_t) (__builtin_mulss(motorParm.qSigmaLsDt,
                estimator.qDIalpha) >> (NORM_SIGMA_LSDTBASE_SCALE_SHIFT+3));

        estimator.qDIbeta = (ialphabeta.beta - estimator.qLastIbetaHS[index]);
        /* Limit the change in current to dIlimitLS in order to
         * reduce the effect of noisy current measurements */
        if (estimator.qDIbeta > estimator.qDIlimitLS) 
        {
            estimator.qDIbeta = estimator.qDIlimitLS;
        }
        if (estimator.qDIbeta<-estimator.qDIlimitLS) 
        {
            estimator.qDIbeta = -estimator.qDIlimitLS;
        }
        estimator.qVIndbeta = (int16_t) (__builtin_mulss(motorParm.qSigmaLsDt,
                estimator.qDIbeta) >> (NORM_SIGMA_LSDTBASE_SCALE_SHIFT+3));
    
    }
    else
    {
        /* At higher speed the granularity of change is insignificant. Hence, the
         * difference can be calculated between two sampled values that are one PLL
         * execution steps apart */
        estimator.qDIalpha = (ialphabeta.alpha -
                estimator.qLastIalphaHS[(estimator.qDiCounter)]);
         /* Limit the change in current to dIlimitHS in order to
         * reduce the effect of noisy current measurements */
        if (estimator.qDIalpha > estimator.qDIlimitLS) 
        {
            estimator.qDIalpha = estimator.qDIlimitLS;
        }
        if (estimator.qDIalpha<-estimator.qDIlimitLS) 
        {
            estimator.qDIalpha = -estimator.qDIlimitLS;
        }
        estimator.qVIndalpha = (int16_t) (__builtin_mulss(motorParm.qSigmaLsDt,
                estimator.qDIalpha) >> NORM_SIGMA_LSDTBASE_SCALE_SHIFT);

        estimator.qDIbeta = (ialphabeta.beta - estimator.qLastIbetaHS[(estimator.qDiCounter)]);
        /* Limit the change in current to dIlimitHS in order to
         * reduce the effect of noisy current measurements */
        if (estimator.qDIbeta > estimator.qDIlimitLS) 
        {
            estimator.qDIbeta = estimator.qDIlimitLS;
        }
        if (estimator.qDIbeta<-estimator.qDIlimitLS) 
        {
            estimator.qDIbeta = -estimator.qDIlimitLS;
        }
        estimator.qVIndbeta = (int16_t) (__builtin_mulss(motorParm.qSigmaLsDt,
                estimator.qDIbeta) >> NORM_SIGMA_LSDTBASE_SCALE_SHIFT);
    }    
   
    /* Update the sample history of Ialpha and Ibeta */
    estimator.qDiCounter = (estimator.qDiCounter + 1) & 0x0007;
    estimator.qLastIalphaHS[estimator.qDiCounter] = ialphabeta.alpha;
    estimator.qLastIbetaHS[estimator.qDiCounter] = ialphabeta.beta;

    
    
   /* Calculate the BEMF voltage:
     *  Ealphabeta = Valphabeta - Rs*Ialphabeta - SigmaLs*(dIalphabeta/dt)
     * and Ealphabeta scale it down by a factor of two. This scaling is required 
     * to prevent overflow/saturation of BEMF calculation in a few corner cases.
     * Use the previous value of Valphabeta here since this value corresponds to
     * the current value of phase current sample */
    
    bemfAlphaBeta.alpha =  (estimator.qValpha - (int16_t) (__builtin_mulss(motorParm.qRs, 
                                  ialphabeta.alpha) >> NORM_RS_SCALE_SHIFT) -
                        estimator.qVIndalpha)>>1;

    bemfAlphaBeta.beta =   (estimator.qVbeta -
                        (int16_t) (__builtin_mulss(motorParm.qRs,
                                 ialphabeta.beta) >> NORM_RS_SCALE_SHIFT) -
                        estimator.qVIndbeta)>>1;
    
    /* Calculate sine and cosine components of the rotor flux angle */
    MC_CalculateSineCosine_Assembly_Ram((estimator.qRho + estimator.qRhoOffset),
                                        &sincosThetaEstimator);

    /*  Park_BEMF.d =  Clark_BEMF.alpha*cos(Angle) + Clark_BEMF.beta*sin(Rho)
       Park_BEMF.q = -Clark_BEMF.alpha*sin(Angle) + Clark_BEMF.beta*cos(Rho)*/
    MC_TransformPark_Assembly(&bemfAlphaBeta, &sincosThetaEstimator, &bemfdq);

    /* Filter the BEMF voltage using a first order low pass filter:
     *  Edqfiltered = 1/TFilterd * Integral{ (Esd-EsdFilter).dt }*/
    
    const int16_t ddiff = (int16_t) (bemfdq.d - estimator.qEsdf);
    estimator.qEsdStateVar += __builtin_mulss(ddiff, estimator.qKfilterEsdq);
    estimator.qEsdf = (int16_t) (estimator.qEsdStateVar >> 15);

    const int16_t qdiff = (int16_t) (bemfdq.q - estimator.qEsqf);
    estimator.qEsqStateVar += __builtin_mulss(qdiff, estimator.qKfilterEsdq);
    estimator.qEsqf = (int16_t) (estimator.qEsqStateVar >> 15);
    /* To avoid Math Trap Error*/
    if(ctrlParm.qVdRef > 0)
    {    /* Calculating Inverse of Rotor Flux linkage*/
        estimator.qinvPsiRd = __builtin_divf(motorParm.qInvLmSqrbyLr,ctrlParm.qVdRef);
    }
   
     /*  For stability the condition for low speed */
    if (_Q15abs(estimator.qVelEstim) > DECIMATE_NOMINAL_SPEED) 
    {
        /* At speed greater than decimation speed, calculate the estimated
         * velocity based on:
         *  OmegaMr = Invpsi * (Eqfiltered - sgn(Eqfiltered) * Edfiltered)
         */
        if (estimator.qEsqf > 0) 
        {
            deltaEs = (int16_t) (estimator.qEsqf - estimator.qEsdf);
            estimator.qOmegaMr =  (int16_t) (__builtin_mulss(estimator.qinvPsiRd,
                                    deltaEs) >> 15);
        } 
        else 
        {
            deltaEs = (int16_t) (estimator.qEsqf + estimator.qEsdf);
            estimator.qOmegaMr = (int16_t) (__builtin_mulss(estimator.qinvPsiRd,
                                    deltaEs) >> 15);
        }
    }        
    /* if estimator speed<10% => condition VelRef<>0 */
    else 
    {
        /* At speed lower than or equal to decimation speed, calculate the estimated
         * velocity based on:
         *  OmegaMr = (1/ke) * (Eqfiltered - sgn(omega) * Edfiltered)
         * to improve stability.
         */
        if (estimator.qVelEstim > 0) 
        {
            deltaEs = (int16_t) (estimator.qEsqf - estimator.qEsdf);
            estimator.qOmegaMr = (int16_t) (__builtin_mulss(estimator.qinvPsiRd,
                                    deltaEs) >> 15);
        } 
        else 
        {
            deltaEs = (int16_t) (estimator.qEsqf + estimator.qEsdf);
            estimator.qOmegaMr = (int16_t) (__builtin_mulss(estimator.qinvPsiRd,
                                    deltaEs) >> 15);
        }
    }
    /* Adjusting the shift count by 1 to compensate for Q14 scaling of the estimated back EMF 
        and Shift Count by NORM_INV_LMSQREBYLR_BASE_SCALE  */ 
    estimator.qOmegaMr = estimator.qOmegaMr << (NORM_INV_LMSQREBYLR_BASE_SCALE+1);
        
     /* Integrate the estimated rotor flux velocity to get estimated rotor angle */
    estimator.qRhoStateVar += __builtin_mulss(estimator.qOmegaMr,
                                estimator.qDeltaT);
    estimator.qRho = (int16_t) (estimator.qRhoStateVar >> 15);
    
    /* Estimate the Magnetizing Current Imr = (Id/(TrS + 1))*/
    const int16_t imrdiff = (int16_t) (ctrlParm.qVdRef - estimator.qImrEstim);
    estimator.qImrEstimStateVar += __builtin_mulss(imrdiff,
                                    estimator.qImrEstimFilterK);
    estimator.qImrEstim = (int16_t) (estimator.qImrEstimStateVar >> 15);
    
    /* Estimate the slip value wslip = ((1/Tr) * (iq/imr))*/
    const int32_t iqTr = __builtin_mulss(motorParm.qInvTr,idq.q);
    
    if(estimator.qImrEstim > 0)
    {    
        estimator.qOmegaSlip = __builtin_divsd(iqTr,estimator.qImrEstim);
    }

    /* Estimate the rotor velocity by subtracting slip from synchronous Speed(rotor flux) */
    estimator.qOmegaRotor = estimator.qOmegaMr - estimator.qOmegaSlip ;
            
    /* Filter the estimated  rotor velocity using a first order low-pass filter */
    
    const int16_t Omegadiff = (int16_t) (estimator.qOmegaRotor - estimator.qVelEstim);
    estimator.qVelEstimStateVar += __builtin_mulss(Omegadiff,
                                    estimator.qVelEstimFilterK);
    estimator.qVelEstim = (int16_t) (estimator.qVelEstimStateVar >> 15);

}
// *****************************************************************************

/* Function:
    InitEstimParm ()

  Summary:
    Initializes Motor speed and angle estimator parameters

  Description:
    Initialization of the parameters of the estimator.

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
void InitEstimParm(void) 
{
    /* Constants are defined in usreparms.h */

    motorParm.qSigmaLsDtBase = NORM_SIGMA_LSDTBASE;
    motorParm.qSigmaLsDt = motorParm.qSigmaLsDtBase;
    motorParm.qRs = NORM_RS;

    motorParm.qInvLmSqrbyLrBase = NORM_INV_LMSQREBYLR_BASE;
    motorParm.qInvLmSqrbyLr = motorParm.qInvLmSqrbyLrBase;

    motorParm.qInvTr = NORM_INVTR;
    
    motorParm.qLs = NORM_LS;    
    estimator.qRhoStateVar = 0;
    estimator.qOmegaMr = 0;
    estimator.qDiCounter = 0;
    estimator.qEsdStateVar = 0;
    estimator.qEsqStateVar = 0;
    estimator.qOmegaSlip = 0;
    estimator.qImrEstim = 0;
    estimator.qImrEstimStateVar = 0;
    estimator.qOmegaRotor = 0;
    estimator.qinvPsiRd = 0;
    estimator.qDIlimitHS = D_ILIMIT_HS;
    estimator.qDIlimitLS = D_ILIMIT_LS;

    estimator.qKfilterEsdq = KFILTER_ESDQ;
    estimator.qVelEstimFilterK = KFILTER_VELESTIM;
    estimator.qImrEstimFilterK = KFILTER_IMRESTIM;
    
    estimator.qDeltaT = NORM_DELTAT;
    estimator.qRhoOffset = INITOFFSET_TRANS_OPEN_CLSD;

}
