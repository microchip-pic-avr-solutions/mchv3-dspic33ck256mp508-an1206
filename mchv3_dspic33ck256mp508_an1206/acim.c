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
// FSEC
#pragma config BWRP = OFF               // Boot Segment Write-Protect bit (Boot Segment may be written)
#pragma config BSS = DISABLED           // Boot Segment Code-Protect Level bits (No Protection (other than BWRP))
#pragma config BSEN = OFF               // Boot Segment Control bit (No Boot Segment)
#pragma config GWRP = OFF               // General Segment Write-Protect bit (General Segment may be written)
#pragma config GSS = DISABLED           // General Segment Code-Protect Level bits (No Protection (other than GWRP))
#pragma config CWRP = OFF               // Configuration Segment Write-Protect bit (Configuration Segment may be written)
#pragma config CSS = DISABLED           // Configuration Segment Code-Protect Level bits (No Protection (other than CWRP))
#pragma config AIVTDIS = OFF            // Alternate Interrupt Vector Table bit (Disabled AIVT)

// FBSLIM
#pragma config BSLIM = 0x1FFF           // Boot Segment Flash Page Address Limit bits (Boot Segment Flash page address  limit)

// FSIGN

// FOSCSEL
#pragma config FNOSC = FRC              // Oscillator Source Selection (Internal Fast RC (FRC))
#pragma config IESO = OFF               // Two-speed Oscillator Start-up Enable bit (Start up with user-selected oscillator source)

// FOSC
#pragma config POSCMD = NONE            // Primary Oscillator Mode Select bits (Primary Oscillator disabled)
#pragma config OSCIOFNC = ON            // OSC2 Pin Function bit (OSC2 is general purpose digital I/O pin)
#pragma config FCKSM = CSECMD           // Clock Switching Mode bits (Clock switching is enabled,Fail-safe Clock Monitor is disabled)
#pragma config XTCFG = G3               // XT Config (24-32 MHz crystals)
#pragma config XTBST = DISABLE           // XT Boost (Boost the kick-start)

// FWDT
#pragma config RWDTPS = PS1048576       // Run Mode Watchdog Timer Post Scaler select bits (1:1048576)
#pragma config RCLKSEL = LPRC           // Watchdog Timer Clock Select bits (Always use LPRC)
#pragma config WINDIS = ON              // Watchdog Timer Window Enable bit (Watchdog Timer operates in Non-Window mode)
#pragma config WDTWIN = WIN25           // Watchdog Timer Window Select bits (WDT Window is 25% of WDT period)
#pragma config SWDTPS = PS1048576       // Sleep Mode Watchdog Timer Post Scaler select bits (1:1048576)
#pragma config FWDTEN = ON_SW           // Watchdog Timer Enable bit (WDT controlled via SW, use WDTCON.ON bit)

// FPOR
#pragma config BISTDIS = DISABLED       // Memory BIST Feature Disable (mBIST on reset feature disabled)

// FICD
#pragma config ICS = PGD3               // ICD Communication Channel Select bits (Communicate on PGEC3 and PGED3)
#pragma config JTAGEN = OFF             // JTAG Enable bit (JTAG is disabled)
#pragma config NOBTSWP = DISABLED       // BOOTSWP instruction disable bit (BOOTSWP instruction is disabled)

// FDMTIVTL
#pragma config DMTIVTL = 0x0         // Dead Man Timer Interval low word (Lower 16 bits of 32 bitDMT window interval (0-0xFFFF))

// FDMTIVTH
#pragma config DMTIVTH = 0x0        // Dead Man Timer Interval high word (Uper 16 bits of 32 bitDMT window interval (0-0xFFFF))

// FDMTCNTL
#pragma config DMTCNTL = 0x0         // Lower 16 bits of 32 bit DMT instruction count time-out value (0-0xFFFF) (Lower 16 bits of 32 bit DMT instruction count time-out value (0-0xFFFF))

// FDMTCNTH
#pragma config DMTCNTH = 0x0         // Upper 16 bits of 32 bit DMT instruction count time-out value (0-0xFFFF) (Upper 16 bits of 32 bit DMT instruction count time-out value (0-0xFFFF))

// FDMT
#pragma config DMTDIS = OFF             // Dead Man Timer Disable bit (Dead Man Timer is Disabled and can be enabled by software)

// FDEVOPT
#pragma config ALTI2C1 = OFF            // Alternate I2C1 Pin bit (I2C1 mapped to SDA1/SCL1 pins)
#pragma config ALTI2C2 = OFF            // Alternate I2C2 Pin bit (I2C2 mapped to SDA2/SCL2 pins)
#pragma config ALTI2C3 = OFF            // Alternate I2C3 Pin bit (I2C3 mapped to SDA3/SCL3 pins)
#pragma config SMBEN = SMBUS            // SM Bus Enable (SMBus input threshold is enabled)
#pragma config SPI2PIN = PPS            // SPI2 Pin Select bit (SPI2 uses I/O remap (PPS) pins)

// FALTREG
#pragma config CTXT1 = OFF              // Specifies Interrupt Priority Level (IPL) Associated to Alternate Working Register 1 bits (Not Assigned)
#pragma config CTXT2 = OFF              // Specifies Interrupt Priority Level (IPL) Associated to Alternate Working Register 2 bits (Not Assigned)
#pragma config CTXT3 = OFF              // Specifies Interrupt Priority Level (IPL) Associated to Alternate Working Register 3 bits (Not Assigned)
#pragma config CTXT4 = OFF              // Specifies Interrupt Priority Level (IPL) Associated to Alternate Working Register 4 bits (Not Assigned)

// FBTSEQ
#pragma config BSEQ = 0xFFF             // Relative value defining which partition will be active after device Reset; the partition containing a lower boot number will be active (Boot Sequence Number bits)
#pragma config IBSEQ = 0xFFF            // The one's complement of BSEQ; must be calculated by the user and written during device programming. (Inverse Boot Sequence Number bits)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#include <libq.h>      
#include "motor_control_noinline.h"

#include "general.h"   
#include "userparms.h"

#include "control.h"   
#include "estim.h"

#include "clock.h"
#include "pwm.h"
#include "adc.h"
#include "port_config.h"
#include "delay.h"
#include "board_service.h"
#include "diagnostics.h"
#include "measure.h"
#include "id_ref.h"


volatile UGF_T uGF;

CTRL_PARM_T ctrlParm;

volatile int16_t thetaElectrical = 0,thetaElectricalOpenLoop = 0;
uint16_t pwmPeriod;

MC_ALPHABETA_T valphabeta,ialphabeta;
MC_SINCOS_T sincosTheta;
MC_DQ_T vdq,idq;
MC_DUTYCYCLEOUT_T pwmDutycycle;
MC_ABC_T   vabc,iabc;

MC_PIPARMIN_T piInputIq;
MC_PIPARMOUT_T piOutputIq;
MC_PIPARMIN_T piInputId;
MC_PIPARMOUT_T piOutputId;
MC_PIPARMIN_T piInputOmega;
MC_PIPARMOUT_T piOutputOmega;

volatile uint16_t adcDataBuffer;
MEASURE_T measureInputs;
/** Definitions */
/* Fraction of dc link voltage(expressed as a squared amplitude) to set the limit for current controllers PI Output */
#define MAX_VOLTAGE_VECTOR                      0.98

void InitControlParameters(void);
void DoControl( void );
void CalculateParkAngle(void);
void ResetParmeters(void);
void pwmDutyCycleSet(MC_DUTYCYCLEOUT_T *);
void pwmDutyCycleLimit(MC_DUTYCYCLEOUT_T *,uint16_t,uint16_t);

// *****************************************************************************
/* Function:
   main()

  Summary:
    main() function

  Description:
    program entry point, calls the system initialization function group 
    containing the buttons polling loop

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */

int main ( void )
{
    InitOscillator();
    SetupGPIOPorts();
    /* Turn on LED2 to indicate the device is programmed */
    LED2 = 1;
    /* Initialize Peripherals */
    InitPeripherals();
    DiagnosticsInit();
    
    BoardServiceInit();
    CORCONbits.SATA = 0;
    while(1)
    {        
        /* Reset parameters used for running motor through Inverter A*/
        ResetParmeters();
        
        while(1)
        {
            DiagnosticsStepMain();
            BoardService();
            
            if(IsPressed_Button1())
            {
                if(uGF.bits.RunMotor == 1)
                {
                    ResetParmeters();
                }
                else
                {
                    EnablePWMOutputsInverterA();
                    uGF.bits.RunMotor = 1;
                }

            }
           
            /* LED1 is used as motor run Status */
            LED1 = uGF.bits.RunMotor;
        }

    } // End of Main loop
    // should never get here
    while(1){}
}
// *****************************************************************************
/* Function:
    ResetParmsA()

  Summary:
    This routine resets all the parameters required for Motor through Inv-A

  Description:
    Reinitializes the duty cycle,resets all the counters when restarting motor

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
void ResetParmeters(void)
{
    /* Make sure ADC does not generate interrupt while initializing parameters*/
	DisableADCInterrupt();
    

    INVERTERA_PWM_TRIGA = ADC_SAMPLING_POINT;

    /* Re initialize the duty cycle to minimum value */
    INVERTERA_PWM_PDC3 = MIN_DUTY;
    INVERTERA_PWM_PDC2 = MIN_DUTY;
    INVERTERA_PWM_PDC1 = MIN_DUTY;
    
    DisablePWMOutputsInverterA();
    
    /* Stop the motor   */
    uGF.bits.RunMotor = 0;        
    /* Set the reference speed value to 0 */
    ctrlParm.qVelRef = 0;
    /* Restart in open loop */
    uGF.bits.OpenLoop = 1;
    /* Change mode */
    uGF.bits.ChangeMode = 1;
     
    /* Initialize PI control parameters */
    InitControlParameters();        
    /* Initialize estimator parameters */
    InitEstimParm();
    /* Initialize FieldWeakening parameters */
    IdRefGenerationInit(&idRefGen);
    /* Initialize measurement parameters */
    MeasureCurrentInit(&measureInputs);
    /* Initialize the Id reference*/
    ctrlParm.qVdRef = Q15_IDREF_BASESPEED;
    /* Enable ADC interrupt and begin main loop timing */
    ClearADCIF();
    adcDataBuffer = ClearADCIF_ReadADCBUF();
    EnableADCInterrupt();
}
// *****************************************************************************
/* Function:
    DoControl()

  Summary:
    Executes one PI iteration for each of the three loops Id,Iq,Speed

  Description:
    This routine executes one PI iteration for each of the three loops
    Id,Iq,Speed

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
void DoControl( void )
{
    /* Temporary variables */
    int16_t vdSquared,vqSquaredLimit;
    int16_t voltRefOpenLoop,vqRefOpenLoop;
    
    if(uGF.bits.OpenLoop)
    {
        /* OPENLOOP:  force rotating angle,Vd and Vq */
        if(uGF.bits.ChangeMode)
        {
            /* Just changed to open loop */
            uGF.bits.ChangeMode = 0;

            /* Synchronize angles */
            /* VqRef & VdRef not used */
            ctrlParm.qVqRef = 0;
            ctrlParm.qVdRef = Q15_IDREF_BASESPEED;

            /* Reinitialize variables for initial speed ramp */
            ctrlParm.startupRamp = 0;
        }
        /* Potentiometer value is scaled between Minimum Speed 
             * and NOMINALSPEED to set the speed reference*/
            
        ctrlParm.qtargetSpeed = (__builtin_mulss(measureInputs.potValue,
                    Q15_NOMINALSPEED-Q15_MINIMUMSPEED)>>15) +
                    Q15_MINIMUMSPEED;
        
        if(ctrlParm.speedRampCount < SPEEDREFRAMP_COUNT)
        {
           ctrlParm.speedRampCount++; 
        }
        else
        {
            /* Ramp generator to limit the change of the speed reference
              the rate of change is defined by CtrlParm.qRefRamp */
            ctrlParm.qDiff = ctrlParm.qVelRef - ctrlParm.qtargetSpeed;
            /* Speed Ref Ramp */
            if (ctrlParm.qDiff < 0)
            {
                /* Set this cycle reference as the sum of
                previously calculated one plus the reference ramp value */
                ctrlParm.qVelRef = ctrlParm.qVelRef+ctrlParm.qRefRamp;
            }
            else
            {
                /* Same as above for speed decrease */
                ctrlParm.qVelRef = ctrlParm.qVelRef-ctrlParm.qRefRamp;
            }
            /* If difference less than half of ref ramp, set reference
            directly from the pot */
            if (_Q15abs(ctrlParm.qDiff) < (ctrlParm.qRefRamp << 1))
            {
                ctrlParm.qVelRef = ctrlParm.qtargetSpeed;
            }
            ctrlParm.speedRampCount = 0;
        }
        
        vdq.d = 0;
        /* Multiplying the V/F constant with Speed ref to get required voltage to apply
         and the output is in Q13 Format*/
        voltRefOpenLoop = (int16_t) (__builtin_mulss(VBYF_CONSTANT, 
                                  ctrlParm.qVelRef) >> 15);
        /* Adding the Minimum Voltage to voltRefOpenLoop to generate 
         * required torque during very low speeds and Converted Minimum Voltage from 
         * Q15 Format to Q13 Format */
        voltRefOpenLoop = voltRefOpenLoop + (NORM_MIN_VOLTAGE_L_L_Peak>>2);
        
        /* Dividing the voltRefOpenLoop with dc bus voltage to compensate the 
         * changes in dc bus voltage  */
        if(measureInputs.dcBusVoltage >0)
        {
            vqRefOpenLoop = (int16_t)__builtin_divf(voltRefOpenLoop,measureInputs.dcBusVoltage);
        }
        else
        {
            vqRefOpenLoop = voltRefOpenLoop;
        }    
        /* Limit the vqRefOpenLoop to Open Loop Maximum Modulation index */
        if(vqRefOpenLoop >= VQ_LIMIT_OPENLOOP)
        {
           vqRefOpenLoop =  VQ_LIMIT_OPENLOOP;
        } 
        /* Converting the vqRefOpenLoop back to Q15 Format from Q13 Format*/ 
        vdq.q  = vqRefOpenLoop<<2;

    }
    else
    /* Closed Loop Vector Control */
    {
        
        /* Potentiometer value is scaled between Minimum Speed
         * and Maximum Speed to set the speed reference*/

        ctrlParm.qtargetSpeed = (__builtin_mulss(measureInputs.potValue,
                Q15_MAXIMUMSPEED-Q15_MINIMUMSPEED)>>15) +
                Q15_MINIMUMSPEED;  
             
        if(ctrlParm.speedRampCount < SPEEDREFRAMP_COUNT)
        {
           ctrlParm.speedRampCount++; 
        }
        else
        {
            /* Ramp generator to limit the change of the speed reference
              the rate of change is defined by CtrlParm.qRefRamp */
            ctrlParm.qDiff = ctrlParm.qVelRef - ctrlParm.qtargetSpeed;
            /* Speed Ref Ramp */
            if (ctrlParm.qDiff < 0)
            {
                /* Set this cycle reference as the sum of
                previously calculated one plus the reference ramp value */
                ctrlParm.qVelRef = ctrlParm.qVelRef+ctrlParm.qRefRamp;
            }
            else
            {
                /* Same as above for speed decrease */
                ctrlParm.qVelRef = ctrlParm.qVelRef-ctrlParm.qRefRamp;
            }
            /* If difference less than half of ref ramp, set reference
            directly from the pot */
            if (_Q15abs(ctrlParm.qDiff) < (ctrlParm.qRefRamp << 1))
            {
                ctrlParm.qVelRef = ctrlParm.qtargetSpeed;
            }
            ctrlParm.speedRampCount = 0;
        }

        if( uGF.bits.ChangeMode )
        {
            /* Just changed from open loop */
            uGF.bits.ChangeMode = 0;
            /* Updating the Speed Control Integral Output*/
            piInputOmega.piState.integrator = (((int32_t)idq.q<<15)<<1);
            /* Setting the speed reference to Minimum Speed*/
            ctrlParm.qVelRef = estimator.qVelEstim;
            /* Updating the D,Q Current Control Integrator outputs*/
            piInputId.piState.integrator = (((int32_t)vdq.d<<15)<<1);
            piInputIq.piState.integrator = (((int32_t)vdq.q<<15)<<1);
            /* Initializing the Flux weakening output Id filter values  */
            idRefGen.IdRefFilt = idRefGen.fdWeak.IdRefMax;
            idRefGen.IdRefFiltStateVar = ((int32_t)idRefGen.IdRefFilt)<<15;
        }

        /* If TORQUE MODE skip the speed controller */
        #ifndef	TORQUE_MODE
            /* Execute the velocity control loop */
            piInputOmega.inMeasure = estimator.qVelEstim;
            piInputOmega.inReference = ctrlParm.qVelRef;
            MC_ControllerPIUpdate_Assembly(piInputOmega.inReference,
                                           piInputOmega.inMeasure,
                                           &piInputOmega.piState,
                                           &piOutputOmega.out);
            ctrlParm.qVqRef = piOutputOmega.out;
        #else
            /* Potentiometer value is scaled to Maximum Torque Component Current
             *  to set the Current reference*/
            ctrlParm.qVqRef =(__builtin_mulss(measureInputs.potValue,
                    Q15_TORQUE_COMPONENT_CURRENT_PEAK)>>15); 
        #endif
        
        /* Flux weakening control - the actual speed is replaced 
        with the reference speed for stability 
        reference for d current component 
        adapt the estimator parameters in concordance with the speed */
        IdRefGeneration(&idRefGen);
        ctrlParm.qVdRef=idRefGen.IdRefFilt;

        /* PI control for D */
        piInputId.inMeasure = idq.d;
        piInputId.inReference  = ctrlParm.qVdRef;
        MC_ControllerPIUpdate_Assembly(piInputId.inReference,
                                       piInputId.inMeasure,
                                       &piInputId.piState,
                                       &piOutputId.out);
        vdq.d    = piOutputId.out;

        /* Dynamic d-q adjustment
         with d component priority 
         vq=sqrt (vs^2 - vd^2) 
        limit vq maximum to the one resulting from the calculation above */
        vdSquared  = (int16_t)(__builtin_mulss(piOutputId.out ,
                                                      piOutputId.out) >> 15);
        vqSquaredLimit = Q15(MAX_VOLTAGE_VECTOR) - vdSquared;
        piInputIq.piState.outMax = Q15SQRT (vqSquaredLimit);
        piInputIq.piState.outMin = - piInputIq.piState.outMax;
        /* PI control for Q */
        piInputIq.inMeasure  = idq.q;
        piInputIq.inReference  = ctrlParm.qVqRef;
        MC_ControllerPIUpdate_Assembly(piInputIq.inReference,
                                       piInputIq.inMeasure,
                                       &piInputIq.piState,
                                       &piOutputIq.out);
        vdq.q = piOutputIq.out;
    }
}
// *****************************************************************************
/* Function:
   _ADCInterrupt()

  Summary:
   _ADCInterrupt() ISR routine

  Description:
    Does speed calculation and executes the vector update loop
    The ADC sample and conversion is triggered by the PWM period.
    The speed calculation assumes a fixed time interval between calculations.

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
void __attribute__((__interrupt__,no_auto_psv)) _ADCInterrupt()
{  

    if(uGF.bits.RunMotor)
    {
        measureInputs.current.Ia = ADCBUF_INV_A_IPHASE1;
        measureInputs.current.Ib = ADCBUF_INV_A_IPHASE2; 
   
        MeasureCurrentCalibrate(&measureInputs);
        iabc.a = measureInputs.current.Ia;
        iabc.b = measureInputs.current.Ib;

        /* Calculate qId,qIq from qSin,qCos,qIa,qIb */
        MC_TransformClarke_Assembly(&iabc,&ialphabeta);
        MC_TransformPark_Assembly(&ialphabeta,&sincosTheta,&idq);

        /* Speed and field angle estimation */
        Estim();
        /* Calculate control values */
        DoControl();
        /* Calculate qAngle */
        CalculateParkAngle();
        /* if open loop */
        if(uGF.bits.OpenLoop == 1)
        {
            /* the angle is given by park parameter */
            thetaElectrical = thetaElectricalOpenLoop;
        }
        else
        {
            /* if closed loop, angle generated by estimator */
            thetaElectrical = estimator.qRho + estimator.qRhoOffset;
        }
        MC_CalculateSineCosine_Assembly_Ram(thetaElectrical,&sincosTheta);
        MC_TransformParkInverse_Assembly(&vdq,&sincosTheta,&valphabeta);

        MC_TransformClarkeInverseSwappedInput_Assembly(&valphabeta,&vabc);


        MC_CalculateSpaceVectorPhaseShifted_Assembly(&vabc,pwmPeriod,
                                                    &pwmDutycycle);
        pwmDutyCycleLimit(&pwmDutycycle,MIN_DUTY,MAX_DUTY);
        pwmDutyCycleSet(&pwmDutycycle);
                        
    }
    else
    {
        INVERTERA_PWM_TRIGA = ADC_SAMPLING_POINT;
        pwmDutycycle.dutycycle3 = MIN_DUTY;
        pwmDutycycle.dutycycle2 = MIN_DUTY;
        pwmDutycycle.dutycycle1 = MIN_DUTY;
        pwmDutyCycleLimit(&pwmDutycycle,MIN_DUTY,MAX_DUTY);
        pwmDutyCycleSet(&pwmDutycycle);
    } 
    
    if(uGF.bits.RunMotor == 0)
    {
        measureInputs.current.Ia = ADCBUF_INV_A_IPHASE1;
        measureInputs.current.Ib = ADCBUF_INV_A_IPHASE2;   
    }
    if(MeasureCurrentOffsetStatus(&measureInputs) == 0)
    {
        MeasureCurrentOffset(&measureInputs);
    }
    else
    {
        BoardServiceStepIsr(); 
    }
    measureInputs.potValue = (int16_t)( ADCBUF_SPEED_REF_A);
    measureInputs.dcBusVoltage = (int16_t)( ADCBUF_VBUS_A);

    DiagnosticsStepIsr();
    
    /* Read ADC Buffet to Clear Flag */
	adcDataBuffer = ClearADCIF_ReadADCBUF();
    ClearADCIF();   
}
// *****************************************************************************
/* Function:
    CalculateParkAngle ()

  Summary:
    Function calculates the angle for open loop control

  Description:
    Generate the start sine waves feeding the motor terminals
    Open loop control, forcing the motor to align and to start speeding up .
 
  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
void CalculateParkAngle(void)
{   
    /* if open loop */
    if(uGF.bits.OpenLoop)
    {
        if(ctrlParm.qVelRef >= Q15_MINIMUMSPEED) 
        {    
            #ifndef OPEN_LOOP_FUNCTIONING
               uGF.bits.ChangeMode = 1;
               uGF.bits.OpenLoop = 0;
               estimator.qRhoOffset = thetaElectricalOpenLoop - estimator.qRho;
           #endif
        }
            /* the integral of the angle is the estimated angle */
        ctrlParm.startupRamp += __builtin_mulss(ctrlParm.qVelRef,
                                    estimator.qDeltaT);
        thetaElectricalOpenLoop = (int16_t) (ctrlParm.startupRamp >> 15);
    }
    /* Switched to closed loop */
    else 
    {
        /* In closed loop slowly decrease the offset add to the estimated angle */
        if (estimator.qRhoOffset > 0)
        {
            estimator.qRhoOffset--;
        }
        else if(estimator.qRhoOffset < 0)
        {
            estimator.qRhoOffset++;
        }
    }

}
// *****************************************************************************
/* Function:
    InitControlParameters()

  Summary:
    Function initializes control parameters

  Description:
    Initialize control parameters: PI coefficients, scaling constants etc.

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
void InitControlParameters(void)
{
    
    ctrlParm.qRefRamp = SPEEDREFRAMP;
    ctrlParm.speedRampCount = SPEEDREFRAMP_COUNT;
    /* Set PWM period to Loop Time */
    pwmPeriod = LOOPTIME_TCY;
 
    /* PI - Id Current Control */
    piInputId.piState.kp = D_CURRCNTR_PTERM;
    piInputId.piState.ki = D_CURRCNTR_ITERM;
    piInputId.piState.kc = D_CURRCNTR_CTERM;
    piInputId.piState.outMax = D_CURRCNTR_OUTMAX;
    piInputId.piState.outMin = -piInputId.piState.outMax;
    piInputId.piState.integrator = 0;
    piOutputId.out = 0;

    /* PI - Iq Current Control */
    piInputIq.piState.kp = Q_CURRCNTR_PTERM;
    piInputIq.piState.ki = Q_CURRCNTR_ITERM;
    piInputIq.piState.kc = Q_CURRCNTR_CTERM;
    piInputIq.piState.outMax = Q_CURRCNTR_OUTMAX;
    piInputIq.piState.outMin = -piInputIq.piState.outMax;
    piInputIq.piState.integrator = 0;
    piOutputIq.out = 0;

    /* PI - Speed Control */
    piInputOmega.piState.kp = SPEEDCNTR_PTERM;
    piInputOmega.piState.ki = SPEEDCNTR_ITERM;
    piInputOmega.piState.kc = SPEEDCNTR_CTERM;
    piInputOmega.piState.outMax = SPEEDCNTR_OUTMAX;
    piInputOmega.piState.outMin = -piInputOmega.piState.outMax;
    piInputOmega.piState.integrator = 0;
    piOutputOmega.out = 0;
}

void pwmDutyCycleSet(MC_DUTYCYCLEOUT_T *pPwmDutycycle)
{  
    INVERTERA_PWM_PDC3 = pPwmDutycycle->dutycycle3;
    INVERTERA_PWM_PDC2 = pPwmDutycycle->dutycycle2;
    INVERTERA_PWM_PDC1 = pPwmDutycycle->dutycycle1;
}

void pwmDutyCycleLimit (MC_DUTYCYCLEOUT_T *pPwmDutycycle,uint16_t min,uint16_t max)
{
    pPwmDutycycle->dutycycle1 = UTIL_LimitU16(pPwmDutycycle->dutycycle1,min,max);
    pPwmDutycycle->dutycycle2 = UTIL_LimitU16(pPwmDutycycle->dutycycle2,min,max);
    pPwmDutycycle->dutycycle3 = UTIL_LimitU16(pPwmDutycycle->dutycycle3,min,max);
}
