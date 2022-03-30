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
#ifndef USERPARMS_H
#define USERPARMS_H

#ifdef __cplusplus
extern "C" {
#endif
// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include <stdint.h>
#include <xc.h>
#include "general.h"

// *****************************************************************************
// *****************************************************************************
// Section: Constants
// *****************************************************************************
// *****************************************************************************
/* Define for open loop(V/F) continuous functioning and closed loop transition disabled*/
/* undef to run in Closed Loop FOC Control */
#undef OPEN_LOOP_FUNCTIONING

/* Definition for torque mode - for a separate tuning of the current PI
controllers, tuning mode will disable the speed PI controller and undef to run in speed control mode*/
#undef TORQUE_MODE
    
/* Define to run in Internal Op Amp Configuration, undef to run in External Op Amp Configuration*/    
#define INTERNAL_OPAMP_CONFIG   
       
/****************************** Motor Parameters ******************************/
/********************  support xls file definitions begin *********************/
/* The following values are given in the xls attached file */
    
/** Board Parameters */
/* Hardware Measurable Rated Voltage*/    
#define MC1_PEAK_VOLTAGE            453     /* Vdc Base */
/* Hardware Measurable Rated Current*/ 
#ifdef INTERNAL_OPAMP_CONFIG
    #define MC1_PEAK_CURRENT            1    /* I Base *//* 11A for Unmodified Gain*/ 
#else
    #define MC1_PEAK_CURRENT            16.5    /* I Base */
#endif    

/* Define to run ZD Induction motor*/    
#define ZD_MOTOR_HV

#ifdef ZD_MOTOR_HV
    /* Motor's number of pole pairs */
    #define NOPOLESPAIRS 2
    /* Nominal speed of the motor in RPM */
    #define NOMINAL_SPEED_RPM    1300 
    /* Maximum speed of the motor in RPM - given by the motor's manufacturer */
    #define MAXIMUM_SPEED_RPM    1300 
    /* Base Speed */
    #define MC1_PEAK_SPEED_RPM      2600
    /* MOTOR Rated Phase Current RMS in Amps*/
    #define NOMINAL_CURRENT_PHASE_RMS (float) 0.25  
    /* Motor Rated Magnetization Current RMS in Amps */
    #define NOMINAL_MAGNITIZING_CURRENT_RMS (float) 0.14 
    /* Motor Rated Torque Component of Current RMS in Amps */
    #define NOMINAL_TORQUE_COMPONENT_CURRENT_RMS (float) 0.2071 
    /*Motor Rated Line - Line RMS Voltage*/
    #define NOMINAL_VOLTAGE_L_L  (float)220 
    /*Minium Line - Line Peak Voltage to start the motor (approximately 10% of NOMINAL_VOLTAGE_L_L_Peak) */
    #define MIN_VOLTAGE_L_L_Peak  (float)(20.0) 
    /*Modulation index Limit in V/F Control (open loop Control)*/
    #define MI_LIMIT_VBYF_CONTROL   0.9
    #ifdef INTERNAL_OPAMP_CONFIG
        /* The following values are given in the xls attached file */
        /* Normalized rs value */
        /* the calculation of Rs gives a value exceeding the Q15 range so,
         the normalized value is further divided by 2^NORM_RS_SCALE to fit the 32768 limit
         this is taken care in the estim.c  */
        #define NORM_RS 16037
        #define NORM_RS_SCALE   0   /* 2^NORM_RS_SCALE is the scaling */ 
        #define NORM_RS_SCALE_SHIFT (15 - NORM_RS_SCALE)
        /* Normalized Sigmals/dt value */
        /* the calculation of NORM_SIGMA_LSDTBASE gives a value exceeding the Q15 range so,
         the normalized value is further divided by 2^NORM_SIGMA_LSDTBASE_SCALE to fit the 32768 limit
         this is taken care in the estim.c  */
        #define NORM_SIGMA_LSDTBASE 20929   
        #define NORM_SIGMA_LSDTBASE_SCALE 6    /* 2^NORM_LSDTBASE_SCALE is the scaling */
        #define NORM_SIGMA_LSDTBASE_SCALE_SHIFT   (15- NORM_SIGMA_LSDTBASE_SCALE)      
        /* Normalized 1/(Lm^2/Lr) value */
        /* Make sure that  NORM_INV_LMSQREBYLR_BASE is less than ctrlParm.qVdRef else 
         * Increase the NORM_INV_LMSQREBYLR_BASE_SCALE in power of 2 and decrease the 
         value of NORM_INV_LMSQREBYLR_BASE in power of 2 */
        #define NORM_INV_LMSQREBYLR_BASE 1546
        #define NORM_INV_LMSQREBYLR_BASE_SCALE  2 /* 2^NORM_INV_LMSQREBYLR_BASE_SCALE is the scaling*/

        /* Normalized delta T value*/
        #define NORM_DELTAT  284
        /* Normalized filter constant of Imr(magnetization current) */
        #define KFILTER_IMRESTIM 68
        /* Normalized Inverse rotor time constant*/
        #define NORM_INVTR 2501    
        /* Normalized Stator Inductance (Lm + Lls)*/
        #define NORM_LS 13133
        #define NORM_LS_SCALE 4    
        /* Limitation constants */
        /* di = i(t1)-i(t2) limitation
         high speed limitation, for dt 50us 
         the value can be taken from attached xls file */
        #define D_ILIMIT_HS 142
        /* low speed limitation, for dt 8*50us */
        #define D_ILIMIT_LS 1136
        /**********************  support xls file definitions end *********************/
    
        /* Open loop speed ramp up end value Value in RPM*/
        #define MIN_SPEED_RPM 300     

        /* Filters constants definitions  */
        /* BEMF filter for d-q components @ low speeds */
        #define KFILTER_ESDQ 164
        /* BEMF filter for d-q components @ high speed - Flux Weakening case */
        #define KFILTER_ESDQ_FW 164
        /* Estimated speed filter constatn */
        #define KFILTER_VELESTIM 1*120

        /* In case of the potentiometer speed reference, a reference ramp
        is needed for assuring the motor can follow the reference imposed /
        minimum value accepted */
        #define SPEEDREFRAMP   1  
        /* The Speed Control Loop Executes every  SPEEDREFRAMP_COUNT */
        #define SPEEDREFRAMP_COUNT   4 

        /* PI controllers tuning values - */     
        /* D Control Loop Coefficients */
        #define D_CURRCNTR_PTERM       Q15(0.001)
        #define D_CURRCNTR_ITERM       Q15(0.0005)
        #define D_CURRCNTR_CTERM       Q15(0.999)
        #define D_CURRCNTR_OUTMAX      0x7FFF

        /* Q Control Loop Coefficients */
        #define Q_CURRCNTR_PTERM       Q15(0.001)
        #define Q_CURRCNTR_ITERM       Q15(0.0005)
        #define Q_CURRCNTR_CTERM       Q15(0.999)
        #define Q_CURRCNTR_OUTMAX      0x7FFF

        /* Velocity Control Loop Coefficients */
        #define SPEEDCNTR_PTERM        Q15(0.1)
        #define SPEEDCNTR_ITERM        10
        #define SPEEDCNTR_CTERM        Q15(0.999)
        #define SPEEDCNTR_OUTMAX       0x5000
    #else
        /* The following values are given in the xls attached file */
        /* Normalized rs value */
        /* the calculation of Rs gives a value exceeding the Q15 range so,
         the normalized value is further divided by 2^NORM_RS_SCALE to fit the 32768 limit
         this is taken care in the estim.c  */
        #define NORM_RS 16538
        #define NORM_RS_SCALE   4   /* 2^NORM_RS_SCALE is the scaling */ 
        #define NORM_RS_SCALE_SHIFT (15 - NORM_RS_SCALE)  
        /* Normalized Sigmals/dt value */
        /* the calculation of NORM_SIGMA_LSDTBASE gives a value exceeding the Q15 range so,
         the normalized value is further divided by 2^NORM_SIGMA_LSDTBASE_SCALE to fit the 32768 limit
         this is taken care in the estim.c  */
        #define NORM_SIGMA_LSDTBASE 21583   
        #define NORM_SIGMA_LSDTBASE_SCALE 10    /* 2^NORM_LSDTBASE_SCALE is the scaling */
        #define NORM_SIGMA_LSDTBASE_SCALE_SHIFT   (15- NORM_SIGMA_LSDTBASE_SCALE)    
        /* Normalized 1/(Lm^2/Lr) value */
        /* Make sure that  NORM_INV_LMSQREBYLR_BASE is less than ctrlParm.qVdRef else 
         * Increase the NORM_INV_LMSQREBYLR_BASE_SCALE in power of 2 and decrease the 
         value of NORM_INV_LMSQREBYLR_BASE in power of 2 */
        #define NORM_INV_LMSQREBYLR_BASE 94
        #define NORM_INV_LMSQREBYLR_BASE_SCALE  2 /* 2^NORM_INV_LMSQREBYLR_BASE_SCALE is the scaling*/

        /* Normalized delta T value*/
        #define NORM_DELTAT  284
        /* Normalized filter constant of Imr(magnetization current) */
        #define KFILTER_IMRESTIM 68
        /* Normalized Inverse rotor time constant*/
        #define NORM_INVTR 2501    
        /* Normalized Stator Inductance (Lm + Lls)*/
        #define NORM_LS 27088
        #define NORM_LS_SCALE 7    
        /* Limitation constants */
        /* di = i(t1)-i(t2) limitation
         high speed limitation, for dt 50us 
         the value can be taken from attached xls file */
        #define D_ILIMIT_HS 142
        /* low speed limitation, for dt 8*50us */
        #define D_ILIMIT_LS 1136
        /**********************  support xls file definitions end *********************/

        /* Open loop speed ramp up end value Value in RPM*/
        #define MIN_SPEED_RPM 300     

        /* Filters constants definitions  */
        /* BEMF filter for d-q components @ low speeds */
        #define KFILTER_ESDQ 164
        /* BEMF filter for d-q components @ high speed - Flux Weakening case */
        #define KFILTER_ESDQ_FW 164
        /* Estimated speed filter constatn */
        #define KFILTER_VELESTIM 1*120

        /* In case of the potentiometer speed reference, a reference ramp
        is needed for assuring the motor can follow the reference imposed /
        minimum value accepted */
        #define SPEEDREFRAMP   1  
        /* The Speed Control Loop Executes every  SPEEDREFRAMP_COUNT */
        #define SPEEDREFRAMP_COUNT   4 

        /* PI controllers tuning values - */          
        /* D Control Loop Coefficients */
        #define D_CURRCNTR_PTERM       Q15(0.5)
        #define D_CURRCNTR_ITERM       Q15(0.1)
        #define D_CURRCNTR_CTERM       Q15(0.999)
        #define D_CURRCNTR_OUTMAX      0x7FFF

        /* Q Control Loop Coefficients */
        #define Q_CURRCNTR_PTERM       Q15(0.5)
        #define Q_CURRCNTR_ITERM       Q15(0.1)
        #define Q_CURRCNTR_CTERM       Q15(0.999)
        #define Q_CURRCNTR_OUTMAX      0x7FFF

        /* Velocity Control Loop Coefficients */
        #define SPEEDCNTR_PTERM        Q15(0.01)
        #define SPEEDCNTR_ITERM        1
        #define SPEEDCNTR_CTERM        Q15(0.999)
        #define SPEEDCNTR_OUTMAX       0x5000
    #endif

#endif
        
/* initial offset added to estimated value, 
 when transitioning from open loop to closed loop */
#define INITOFFSET_TRANS_OPEN_CLSD 0x0000
    
/* Field Weakening Parameters*/
/* Fraction of dc link voltage expressed as a squared amplitude. */
#define VMAX_SQR_FDWEAK  Q15(0.5)  
/* Filter Constant for LqsIqs  */    
#define SIGMALSIQS_FILT_CONST   1700
/* Filter Constant for Idref Generation*/    
#define IDREF_FILT_CONST    500
/* Field Weakening Parameters*/
    
/* MOTOR Rated Phase Current Peak in Amps*/
#define NOMINAL_CURRENT_PHASE_PEAK (float) (NOMINAL_CURRENT_PHASE_RMS * 1.414) 
/* Motor Rated Magnetization Current Peak in Amps */
#define NOMINAL_MAGNITIZING_CURRENT_PEAK (float) (NOMINAL_MAGNITIZING_CURRENT_RMS * 1.414) 
/* Motor Rated Torque Component of Current Peak in Amps */
#define NOMINAL_TORQUE_COMPONENT_CURRENT_PEAK (float) (NOMINAL_TORQUE_COMPONENT_CURRENT_RMS * 1.414)
/*Motor Rated Line - Line Peak Voltage*/
#define NOMINAL_VOLTAGE_L_L_Peak  (float)(NOMINAL_VOLTAGE_L_L*1.414)
/* Minimum motor open loop speed converted into Normalized Minimum speed */    
#define Q15_MINIMUMSPEED  NORM_VALUE(MIN_SPEED_RPM,MC1_PEAK_SPEED_RPM)   
/* Maximum motor speed converted into Normalized Maximum speed */
#define Q15_MAXIMUMSPEED NORM_VALUE(MAXIMUM_SPEED_RPM,MC1_PEAK_SPEED_RPM)
/* Nominal motor speed converted into Normalized Nominal speed */
#define Q15_NOMINALSPEED NORM_VALUE(NOMINAL_SPEED_RPM,MC1_PEAK_SPEED_RPM)
/* Nominal motor Line to Line Voltage Peak converted into Normalized Line to Line Voltage Peak */
#define Q15_NOMINAL_VOLTAGE_L_L_Peak NORM_VALUE(NOMINAL_VOLTAGE_L_L_Peak,MC1_PEAK_VOLTAGE) 
/* Nominal motor Minimum Line to Line Voltage Peak converted into Normalized Minimum Line to Line Voltage Peak */    
#define NORM_MIN_VOLTAGE_L_L_Peak  NORM_VALUE(MIN_VOLTAGE_L_L_Peak,MC1_PEAK_VOLTAGE)  
/* Normalized value of V by F constant in q13 format*/    
#define VBYF_CONSTANT    (int16_t)(((int32_t)(Q15_NOMINAL_VOLTAGE_L_L_Peak-NORM_MIN_VOLTAGE_L_L_Peak)<<13)/(int32_t)Q15_NOMINALSPEED)
/* Modulation limit in Q13 format*/
#define VQ_LIMIT_OPENLOOP   (int16_t)(MI_LIMIT_VBYF_CONTROL * 8192)
/* Normalized value of Veloctiy Threshhold used in Estimator*/    
#define PLL_VELOCITY_FILTER_THRESHOLD Q15_NOMINALSPEED
/* Normalized value of Iq Limit*/
#define Q15_TORQUE_COMPONENT_CURRENT_PEAK   NORM_VALUE(NOMINAL_TORQUE_COMPONENT_CURRENT_PEAK,MC1_PEAK_CURRENT)   
/******************************** Field Weakening *****************************/
/* Field Weakening constant for constant torque range 
   Flux reference value */
#define Q15_IDREF_BASESPEED         NORM_VALUE(NOMINAL_MAGNITIZING_CURRENT_PEAK,MC1_PEAK_CURRENT)   

#ifdef __cplusplus
}
#endif

#endif /* __USERPARMS_H */
