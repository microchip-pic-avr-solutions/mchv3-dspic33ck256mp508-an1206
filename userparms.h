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
    #define MC1_PEAK_CURRENT            16.4    /* I Base */
#endif    

/* Define to run washing machine Induction Motor */    
#undef WASHING_MACHINE_MOTOR_HV
/* Define to run oriental Induction motor*/    
#undef ORIENTAL_MOTOR_HV
/* Define to run Leeson Induction motor*/    
#undef LEESON_MOTOR_HV
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
        #define NORM_RS 16438
        #define NORM_RS_SCALE   4   /* 2^NORM_RS_SCALE is the scaling */ 
        #define NORM_RS_SCALE_SHIFT (15 - NORM_RS_SCALE)  
        /* Normalized Sigmals/dt value */
        /* the calculation of NORM_SIGMA_LSDTBASE gives a value exceeding the Q15 range so,
         the normalized value is further divided by 2^NORM_SIGMA_LSDTBASE_SCALE to fit the 32768 limit
         this is taken care in the estim.c  */
        #define NORM_SIGMA_LSDTBASE 21452   
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
        #define NORM_LS 26923
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
    
#ifdef WASHING_MACHINE_MOTOR_HV
     /* Motor's number of pole pairs */
    #define NOPOLESPAIRS    1
    /* Nominal speed of the motor in RPM */
    #define NOMINAL_SPEED_RPM   6000 
    /* Maximum speed of the motor in RPM - given by the motor's manufacturer */
    #define MAXIMUM_SPEED_RPM   18000 
    /* Base Speed */
    #define MC1_PEAK_SPEED_RPM  30000
    /* MOTOR Rated Phase Current RMS in Amps*/
    #define NOMINAL_CURRENT_PHASE_RMS (float) 3.0 
    /* Motor Rated Magnetization Current RMS in Amps */
    #define NOMINAL_MAGNITIZING_CURRENT_RMS (float) 1.35 
    /* Motor Rated Torque Component of Current RMS in Amps */
    #define NOMINAL_TORQUE_COMPONENT_CURRENT_RMS (float) 2.6791 
    /*Motor Rated Line - Line RMS Voltage*/
    #define NOMINAL_VOLTAGE_L_L  (float)195 
    /*Minium Line - Line Peak Voltage to start the motor (approximately 10% of NOMINAL_VOLTAGE_L_L_Peak) */
    #define MIN_VOLTAGE_L_L_Peak  (float)(30.0)  
    /*Modulation index Limit in V/F Control (open loop Control)*/
    #define MI_LIMIT_VBYF_CONTROL   0.9
    /* The following values are given in the xls attached file */
    #ifdef INTERNAL_OPAMP_CONFIG
        /* Normalized rs value */
        /* the calculation of Rs gives a value exceeding the Q15 range so,
         the normalized value is further divided by 2^NORM_RS_SCALE to fit the 32768 limit
         this is taken care in the estim.c  */
        #define NORM_RS 4961
        #define NORM_RS_SCALE   0   /* 2^NORM_RS_SCALE is the scaling */ 
        #define NORM_RS_SCALE_SHIFT (15 - NORM_RS_SCALE)
        /* Normalized Sigmals/dt value */
        /* the calculation of NORM_SIGMA_LSDTBASE gives a value exceeding the Q15 range so,
         the normalized value is further divided by 2^NORM_SIGMA_LSDTBASE_SCALE to fit the 32768 limit
         this is taken care in the estim.c  */
        #define NORM_SIGMA_LSDTBASE 13155   
        #define NORM_SIGMA_LSDTBASE_SCALE 6    /* 2^NORM_LSDTBASE_SCALE is the scaling */
        #define NORM_SIGMA_LSDTBASE_SCALE_SHIFT   (15- NORM_SIGMA_LSDTBASE_SCALE)      
        /* Normalized 1/(Lm^2/Lr) value */
        /* Make sure that  NORM_INV_LMSQREBYLR_BASE is less than ctrlParm.qVdRef else 
         * Increase the NORM_INV_LMSQREBYLR_BASE_SCALE in power of 2 and decrease the 
         value of NORM_INV_LMSQREBYLR_BASE in power of 2 */
        #define NORM_INV_LMSQREBYLR_BASE 852
        #define NORM_INV_LMSQREBYLR_BASE_SCALE  1 /* 2^NORM_INV_LMSQREBYLR_BASE_SCALE is the scaling*/

        /* Normalized delta T value*/
        #define NORM_DELTAT  1638
        /* Normalized filter constant of Imr(magnetization current) */
        #define KFILTER_IMRESTIM 34
        /* Normalized Inverse rotor time constant*/
        #define NORM_INVTR 213    
        /* Normalized Stator Inductance (Lm + Lls)*/
        #define NORM_LS 11907
        #define NORM_LS_SCALE 6    
        /* Limitation constants */
        /* di = i(t1)-i(t2) limitation
         high speed limitation, for dt 50us 
         the value can be taken from attached xls file */
        #define D_ILIMIT_HS 928
        /* low speed limitation, for dt 8*50us */
        #define D_ILIMIT_LS 2621
    #else
        /* Normalized rs value */
        /* the calculation of Rs gives a value exceeding the Q15 range so,
         the normalized value is further divided by 2^NORM_RS_SCALE to fit the 32768 limit
         this is taken care in the estim.c  */
        #define NORM_RS 7397
        #define NORM_RS_SCALE   0   /* 2^NORM_RS_SCALE is the scaling */ 
        #define NORM_RS_SCALE_SHIFT (15 - NORM_RS_SCALE)
        /* Normalized Sigmals/dt value */
        /* the calculation of NORM_SIGMA_LSDTBASE gives a value exceeding the Q15 range so,
         the normalized value is further divided by 2^NORM_SIGMA_LSDTBASE_SCALE to fit the 32768 limit
         this is taken care in the estim.c  */
        #define NORM_SIGMA_LSDTBASE 19613   
        #define NORM_SIGMA_LSDTBASE_SCALE 6    /* 2^NORM_LSDTBASE_SCALE is the scaling */
        #define NORM_SIGMA_LSDTBASE_SCALE_SHIFT   (15- NORM_SIGMA_LSDTBASE_SCALE)      
        /* Normalized 1/(Lm^2/Lr) value */
        /* Make sure that  NORM_INV_LMSQREBYLR_BASE is less than ctrlParm.qVdRef else 
         * Increase the NORM_INV_LMSQREBYLR_BASE_SCALE in power of 2 and decrease the 
         value of NORM_INV_LMSQREBYLR_BASE in power of 2 */
        #define NORM_INV_LMSQREBYLR_BASE 572
        #define NORM_INV_LMSQREBYLR_BASE_SCALE  1 /* 2^NORM_INV_LMSQREBYLR_BASE_SCALE is the scaling*/

        /* Normalized delta T value*/
        #define NORM_DELTAT  1638
        /* Normalized filter constant of Imr(magnetization current) */
        #define KFILTER_IMRESTIM 34
        /* Normalized Inverse rotor time constant*/
        #define NORM_INVTR 213    
        /* Normalized Stator Inductance (Lm + Lls)*/
        #define NORM_LS 17752
        #define NORM_LS_SCALE 6    
        /* Limitation constants */
        /* di = i(t1)-i(t2) limitation
         high speed limitation, for dt 50us 
         the value can be taken from attached xls file */
        #define D_ILIMIT_HS 928
        /* low speed limitation, for dt 8*50us */
        #define D_ILIMIT_LS 2621
    #endif
    
    /**********************  support xls file definitions end *********************/

    /* Open loop speed ramp up end value Value in RPM*/
    #define MIN_SPEED_RPM 2000     

    /* Filters constants definitions  */
    /* BEMF filter for d-q components @ low speeds */
    #define KFILTER_ESDQ 600
    /* BEMF filter for d-q components @ high speed - Flux Weakening case */
    #define KFILTER_ESDQ_FW 164
    /* Estimated speed filter constatn */
    #define KFILTER_VELESTIM 1*120

    /* In case of the potentiometer speed reference, a reference ramp
    is needed for assuring the motor can follow the reference imposed /
    minimum value accepted */
    #define SPEEDREFRAMP   1  
    /* The Speed Control Loop Executes every  SPEEDREFRAMP_COUNT */
    #define SPEEDREFRAMP_COUNT   120 

    /* PI controllers tuning values - */     
    /* D Control Loop Coefficients */
    #define D_CURRCNTR_PTERM       Q15(0.05)
    #define D_CURRCNTR_ITERM       Q15(0.005)
    #define D_CURRCNTR_CTERM       Q15(0.999)
    #define D_CURRCNTR_OUTMAX      0x7FFF

    /* Q Control Loop Coefficients */
    #define Q_CURRCNTR_PTERM       Q15(0.05)
    #define Q_CURRCNTR_ITERM       Q15(0.005)
    #define Q_CURRCNTR_CTERM       Q15(0.999)
    #define Q_CURRCNTR_OUTMAX      0x7FFF

    /* Velocity Control Loop Coefficients */
    #define SPEEDCNTR_PTERM        Q15(0.1)
    #define SPEEDCNTR_ITERM        5
    #define SPEEDCNTR_CTERM        Q15(0.999)
    #define SPEEDCNTR_OUTMAX       0x5000

#endif
    
#ifdef ORIENTAL_MOTOR_HV
    /* Motor's number of pole pairs */
    #define NOPOLESPAIRS 2
    /* Nominal speed of the motor in RPM */
    #define NOMINAL_SPEED_RPM    1600 
    /* Maximum speed of the motor in RPM - given by the motor's manufacturer */
    #define MAXIMUM_SPEED_RPM    1600 
    /* Base Speed */
    #define MC1_PEAK_SPEED_RPM      3200
    /* MOTOR Rated Phase Current RMS in Amps*/
    #define NOMINAL_CURRENT_PHASE_RMS (float) 0.21  
    /* Motor Rated Magnetization Current RMS in Amps */
    #define NOMINAL_MAGNITIZING_CURRENT_RMS (float) 0.13 
    /* Motor Rated Torque Component of Current RMS in Amps */
    #define NOMINAL_TORQUE_COMPONENT_CURRENT_RMS (float) 0.1649 
    /*Motor Rated Line - Line RMS Voltage*/
    #define NOMINAL_VOLTAGE_L_L  (float)220 
    /*Minium Line - Line Peak Voltage to start the motor (approximately 10% of NOMINAL_VOLTAGE_L_L_Peak) */
    #define MIN_VOLTAGE_L_L_Peak  (float)(30.0)  
    /*Modulation index Limit in V/F Control (open loop Control)*/
    #define MI_LIMIT_VBYF_CONTROL   0.9
    /* The following values are given in the xls attached file */
    #ifdef INTERNAL_OPAMP_CONFIG
        /* Normalized rs value */
        /* the calculation of Rs gives a value exceeding the Q15 range so,
         the normalized value is further divided by 2^NORM_RS_SCALE to fit the 32768 limit
         this is taken care in the estim.c  */
        #define NORM_RS 11652
        #define NORM_RS_SCALE   0   /* 2^NORM_RS_SCALE is the scaling */ 
        #define NORM_RS_SCALE_SHIFT (15 - NORM_RS_SCALE)
        /* Normalized Sigmals/dt value */
        /* the calculation of NORM_SIGMA_LSDTBASE gives a value exceeding the Q15 range so,
         the normalized value is further divided by 2^NORM_SIGMA_LSDTBASE_SCALE to fit the 32768 limit
         this is taken care in the estim.c  */
        #define NORM_SIGMA_LSDTBASE 16444   
        #define NORM_SIGMA_LSDTBASE_SCALE 6    /* 2^NORM_LSDTBASE_SCALE is the scaling */
        #define NORM_SIGMA_LSDTBASE_SCALE_SHIFT   (15- NORM_SIGMA_LSDTBASE_SCALE)      
        /* Normalized 1/(Lm^2/Lr) value */
        /* Make sure that  NORM_INV_LMSQREBYLR_BASE is less than ctrlParm.qVdRef else 
         * Increase the NORM_INV_LMSQREBYLR_BASE_SCALE in power of 2 and decrease the 
         value of NORM_INV_LMSQREBYLR_BASE in power of 2 */
        #define NORM_INV_LMSQREBYLR_BASE 1598
        #define NORM_INV_LMSQREBYLR_BASE_SCALE  2 /* 2^NORM_INV_LMSQREBYLR_BASE_SCALE is the scaling*/

        /* Normalized delta T value*/
        #define NORM_DELTAT  350
        /* Normalized filter constant of Imr(magnetization current) */
        #define KFILTER_IMRESTIM 63
        /* Normalized Inverse rotor time constant*/
        #define NORM_INVTR 1879    
        /* Normalized Stator Inductance (Lm + Lls)*/
        #define NORM_LS 12700
        #define NORM_LS_SCALE 4    
        /* Limitation constants */
        /* di = i(t1)-i(t2) limitation
         high speed limitation, for dt 50us 
         the value can be taken from attached xls file */
        #define D_ILIMIT_HS 175
        /* low speed limitation, for dt 8*50us */
        #define D_ILIMIT_LS 1398
        /* Open loop speed ramp up end value Value in RPM*/
        #define MIN_SPEED_RPM 500     

        /* Filters constants definitions  */
        /* BEMF filter for d-q components @ low speeds */
        #define KFILTER_ESDQ 600
        /* BEMF filter for d-q components @ high speed - Flux Weakening case */
        #define KFILTER_ESDQ_FW 164
        /* Estimated speed filter constatn */
        #define KFILTER_VELESTIM 1*120

        /* In case of the potentiometer speed reference, a reference ramp
        is needed for assuring the motor can follow the reference imposed /
        minimum value accepted */
        #define SPEEDREFRAMP   1  
        /* The Speed Control Loop Executes every  SPEEDREFRAMP_COUNT */
        #define SPEEDREFRAMP_COUNT   6 

        /* PI controllers tuning values - */     
        /* D Control Loop Coefficients */
        #define D_CURRCNTR_PTERM       Q15(0.05)
        #define D_CURRCNTR_ITERM       Q15(0.005)
        #define D_CURRCNTR_CTERM       Q15(0.999)
        #define D_CURRCNTR_OUTMAX      0x7FFF

        /* Q Control Loop Coefficients */
        #define Q_CURRCNTR_PTERM       Q15(0.05)
        #define Q_CURRCNTR_ITERM       Q15(0.005)
        #define Q_CURRCNTR_CTERM       Q15(0.999)
        #define Q_CURRCNTR_OUTMAX      0x7FFF

        /* Velocity Control Loop Coefficients */
        #define SPEEDCNTR_PTERM        Q15(0.1)
        #define SPEEDCNTR_ITERM        10
        #define SPEEDCNTR_CTERM        Q15(0.999)
        #define SPEEDCNTR_OUTMAX       0x5000
    #else
        /* Normalized rs value */
        /* the calculation of Rs gives a value exceeding the Q15 range so,
         the normalized value is further divided by 2^NORM_RS_SCALE to fit the 32768 limit
         this is taken care in the estim.c  */
        #define NORM_RS 11943
        #define NORM_RS_SCALE   4   /* 2^NORM_RS_SCALE is the scaling */ 
        #define NORM_RS_SCALE_SHIFT (15 - NORM_RS_SCALE)
        /* Normalized Sigmals/dt value */
        /* the calculation of NORM_SIGMA_LSDTBASE gives a value exceeding the Q15 range so,
         the normalized value is further divided by 2^NORM_SIGMA_LSDTBASE_SCALE to fit the 32768 limit
         this is taken care in the estim.c  */
        #define NORM_SIGMA_LSDTBASE 16855   
        #define NORM_SIGMA_LSDTBASE_SCALE 10    /* 2^NORM_LSDTBASE_SCALE is the scaling */
        #define NORM_SIGMA_LSDTBASE_SCALE_SHIFT   (15- NORM_SIGMA_LSDTBASE_SCALE)      
        /* Normalized 1/(Lm^2/Lr) value */
        /* Make sure that  NORM_INV_LMSQREBYLR_BASE is less than ctrlParm.qVdRef else 
         * Increase the NORM_INV_LMSQREBYLR_BASE_SCALE in power of 2 and decrease the 
         value of NORM_INV_LMSQREBYLR_BASE in power of 2 */
        #define NORM_INV_LMSQREBYLR_BASE 97
        #define NORM_INV_LMSQREBYLR_BASE_SCALE  2 /* 2^NORM_INV_LMSQREBYLR_BASE_SCALE is the scaling*/

        /* Normalized delta T value*/
        #define NORM_DELTAT  350
        /* Normalized filter constant of Imr(magnetization current) */
        #define KFILTER_IMRESTIM 63
        /* Normalized Inverse rotor time constant*/
        #define NORM_INVTR 1879    
        /* Normalized Stator Inductance (Lm + Lls)*/
        #define NORM_LS 26036
        #define NORM_LS_SCALE 7    
        /* Limitation constants */
        /* di = i(t1)-i(t2) limitation
         high speed limitation, for dt 50us 
         the value can be taken from attached xls file */
        #define D_ILIMIT_HS 175
        /* low speed limitation, for dt 8*50us */
        #define D_ILIMIT_LS 1398
        /* Open loop speed ramp up end value Value in RPM*/
        #define MIN_SPEED_RPM 500     

        /* Filters constants definitions  */
        /* BEMF filter for d-q components @ low speeds */
        #define KFILTER_ESDQ 600
        /* BEMF filter for d-q components @ high speed - Flux Weakening case */
        #define KFILTER_ESDQ_FW 164
        /* Estimated speed filter constatn */
        #define KFILTER_VELESTIM 1*120

        /* In case of the potentiometer speed reference, a reference ramp
        is needed for assuring the motor can follow the reference imposed /
        minimum value accepted */
        #define SPEEDREFRAMP   1  
        /* The Speed Control Loop Executes every  SPEEDREFRAMP_COUNT */
        #define SPEEDREFRAMP_COUNT   6 

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
    /**********************  support xls file definitions end *********************/
#endif  
    
#ifdef LEESON_MOTOR_HV
    /* Motor's number of pole pairs */
    #define NOPOLESPAIRS 1
    /* Nominal speed of the motor in RPM */
    #define NOMINAL_SPEED_RPM    3600 
    /* Maximum speed of the motor in RPM - given by the motor's manufacturer */
    #define MAXIMUM_SPEED_RPM    3600 
    /* Base Speed */
    #define MC1_PEAK_SPEED_RPM      7200
    /* MOTOR Rated Phase Current RMS in Amps*/
    #define NOMINAL_CURRENT_PHASE_RMS (float) 1.3  
    /* Motor Rated Magnetization Current RMS in Amps */
    #define NOMINAL_MAGNITIZING_CURRENT_RMS (float) 0.75 
    /* Motor Rated Torque Component of Current RMS in Amps */
    #define NOMINAL_TORQUE_COMPONENT_CURRENT_RMS (float) 1.0618 
    /*Motor Rated Line - Line RMS Voltage*/
    #define NOMINAL_VOLTAGE_L_L  (float)208 
    /*Minium Line - Line Peak Voltage to start the motor (approximately 10% of NOMINAL_VOLTAGE_L_L_Peak) */
    #define MIN_VOLTAGE_L_L_Peak  (float)(30.0)  
    /* The following values are given in the xls attached file */
    /*Modulation index Limit in V/F Control (open loop Control)*/
    #define MI_LIMIT_VBYF_CONTROL   0.9
    #ifdef INTERNAL_OPAMP_CONFIG
        /* Normalized rs value */
        /* the calculation of Rs gives a value exceeding the Q15 range so,
         the normalized value is further divided by 2^NORM_RS_SCALE to fit the 32768 limit
         this is taken care in the estim.c  */
        #define NORM_RS  11025
        #define NORM_RS_SCALE       0   /* 2^NORM_RS_SCALE is the scaling */ 
        #define NORM_RS_SCALE_SHIFT   (15 - NORM_RS_SCALE) 
        /* Normalized ls/dt value */
        /* the calculation of NORM_SIGMA_LSDTBASE gives a value exceeding the Q15 range so,
         the normalized value is further divided by 2^NORM_SIGMA_LSDTBASE_SCALE to fit the 32768 limit
         this is taken care in the estim.c  */
        #define NORM_SIGMA_LSDTBASE 7975   
        #define NORM_SIGMA_LSDTBASE_SCALE 8    /* 2^NORM_LSDTBASE_SCALE is the scaling */
        #define NORM_SIGMA_LSDTBASE_SCALE_SHIFT   (15- NORM_SIGMA_LSDTBASE_SCALE)     
        /* Normalized 1/(Lm^2/Lr) value */
        /* Make sure that  NORM_INV_LMSQREBYLR_BASE is less than ctrlParm.qVdRef else 
         * Increase the NORM_INV_LMSQREBYLR_BASE_SCALE in power of 2 and decrease the 
         value of NORM_INV_LMSQREBYLR_BASE in power of 2 */
        #define NORM_INV_LMSQREBYLR_BASE 732
        #define NORM_INV_LMSQREBYLR_BASE_SCALE  2 /* 2^NORM_INV_LMSQREBYLR_BASE_SCALE is the scaling*/

        /* Normalized delta T value*/
        #define NORM_DELTAT  393
        /* Normalized filter constant of Imr(magnetization current) */
        #define KFILTER_IMRESTIM 31
        /* Normalized Inverse rotor time constant*/
        #define NORM_INVTR 815    
        /* Normalized Stator Inductance (Lm + Lls)*/
        #define NORM_LS 13859
        #define NORM_LS_SCALE 5    
        /* Limitation constants */
        /* di = i(t1)-i(t2) limitation
         high speed limitation, for dt 50us 
         the value can be taken from attached xls file */
        #define D_ILIMIT_HS 928
        /* low speed limitation, for dt 8*50us */
        #define D_ILIMIT_LS 2621
    #else
        /* Normalized rs value */
        /* the calculation of Rs gives a value exceeding the Q15 range so,
         the normalized value is further divided by 2^NORM_RS_SCALE to fit the 32768 limit
         this is taken care in the estim.c  */
        #define NORM_RS  16438
        #define NORM_RS_SCALE       0   /* 2^NORM_RS_SCALE is the scaling */ 
        #define NORM_RS_SCALE_SHIFT   (15 - NORM_RS_SCALE)
        /* Normalized ls/dt value */
        /* the calculation of NORM_SIGMA_LSDTBASE gives a value exceeding the Q15 range so,
         the normalized value is further divided by 2^NORM_SIGMA_LSDTBASE_SCALE to fit the 32768 limit
         this is taken care in the estim.c  */
        #define NORM_SIGMA_LSDTBASE 11891   
        #define NORM_SIGMA_LSDTBASE_SCALE 8    /* 2^NORM_LSDTBASE_SCALE is the scaling */
        #define NORM_SIGMA_LSDTBASE_SCALE_SHIFT   (15- NORM_SIGMA_LSDTBASE_SCALE)      
        /* Normalized 1/(Lm^2/Lr) value */
        /* Make sure that  NORM_INV_LMSQREBYLR_BASE is less than ctrlParm.qVdRef else 
         * Increase the NORM_INV_LMSQREBYLR_BASE_SCALE in power of 2 and decrease the 
         value of NORM_INV_LMSQREBYLR_BASE in power of 2 */
        #define NORM_INV_LMSQREBYLR_BASE 491
        #define NORM_INV_LMSQREBYLR_BASE_SCALE  2 /* 2^NORM_INV_LMSQREBYLR_BASE_SCALE is the scaling*/

        /* Normalized delta T value*/
        #define NORM_DELTAT  393
        /* Normalized filter constant of Imr(magnetization current) */
        #define KFILTER_IMRESTIM 31
        /* Normalized Inverse rotor time constant*/
        #define NORM_INVTR 815    
        /* Normalized Stator Inductance (Lm + Lls)*/
        #define NORM_LS 20663
        #define NORM_LS_SCALE 5    
        /* Limitation constants */
        /* di = i(t1)-i(t2) limitation
         high speed limitation, for dt 50us 
         the value can be taken from attached xls file */
        #define D_ILIMIT_HS 197
        /* low speed limitation, for dt 8*50us */
        #define D_ILIMIT_LS 1573
    #endif
    
    /**********************  support xls file definitions end *********************/

    /* Open loop speed ramp up end value Value in RPM*/
    #define MIN_SPEED_RPM 500     

    /* Filters constants definitions  */
    /* BEMF filter for d-q components @ low speeds */
    #define KFILTER_ESDQ 600
    /* BEMF filter for d-q components @ high speed - Flux Weakening case */
    #define KFILTER_ESDQ_FW 164
    /* Estimated speed filter constatn */
    #define KFILTER_VELESTIM 1*120

    /* In case of the potentiometer speed reference, a reference ramp
    is needed for assuring the motor can follow the reference imposed /
    minimum value accepted */
    #define SPEEDREFRAMP   1  
    /* The Speed Control Loop Executes every  SPEEDREFRAMP_COUNT */
    #define SPEEDREFRAMP_COUNT   60 

    /* PI controllers tuning values - */     
    /* D Control Loop Coefficients */
    #define D_CURRCNTR_PTERM       Q15(0.05)
    #define D_CURRCNTR_ITERM       Q15(0.005)
    #define D_CURRCNTR_CTERM       Q15(0.999)
    #define D_CURRCNTR_OUTMAX      0x7FFF

    /* Q Control Loop Coefficients */
    #define Q_CURRCNTR_PTERM       Q15(0.05)
    #define Q_CURRCNTR_ITERM       Q15(0.005)
    #define Q_CURRCNTR_CTERM       Q15(0.999)
    #define Q_CURRCNTR_OUTMAX      0x7FFF

    /* Velocity Control Loop Coefficients */
    #define SPEEDCNTR_PTERM        Q15(0.1)
    #define SPEEDCNTR_ITERM        10
    #define SPEEDCNTR_CTERM        Q15(0.999)
    #define SPEEDCNTR_OUTMAX       0x5000

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
