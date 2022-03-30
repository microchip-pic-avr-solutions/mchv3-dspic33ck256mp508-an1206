![image](images/microchip.jpg) 

# MCHV3 dsPIC33CK256MP508 AN1206

## INTRODUCTION
<p style='text-align: justify;'>
The requirement of low-cost, low maintenance, robust electrical motors has resulted in the emergence of the AC Induction Motor (ACIM). The AC Induction Motor (ACIM) is the workhorse of industrial and domestic motor applications due to its simple construction and durability. These motors have no brushes to wear out or magnets to add to the cost. The induction motors are available in a variety of power and sizes.</p>
<p style='text-align: justify;'>
The control of induction machines can be classified into ‘scalar (v/f)’ and ‘vector’ controls. Scalar controls are simple to implement and offer good steady-state response. However, the dynamics are slow because the transients are not controlled. To obtain high precision and good dynamics, the Field Oriented Control (FOC), also known as vector control, provides the best solution. Flux position information is required to implement vector control. Using position/speed sensors may result in many practical problems, such as complexity of hardware, difficulties in application meant for hostile environments, increased cost, reduced reliability due to the cables and sensor itself, difficulties of mechanical attachment of the sensor to the electric machine, increased axial length of the machine and electromagnetic noise interference. These problems with a physical sensor can be mitigated by estimating the speed/ position information (sensorless FOC).</p>
<p style='text-align: justify;'>
The intent of this application note is to present sensorless Field Oriented Control (FOC) of a three-phase AC Induction Motor using a PLL estimator.</p>
<p align="center">
  <img src="images/focblockdiagram.PNG">
</p>

### SPEED AND ANGLE ESTIMATOR
<p style='text-align: justify;'>
A Phase-Locked Loop (PLL) estimator is implemented to estimate the speed and position of the rotor flux. Back-Electromagnetic Force (Back-EMF or BEMF) is used for estimating the speed and position of the rotor flux. When rotor flux is oriented along the d-axis, then the d-component of BEMF (ed) would be equal to 0. The estimator algorithm operates on the principle that the steady-state value of the d-axis component of the Back-EMF is equal to zero. This estimator implementation is similar to the PLL estimator employed in Microchip Application Note: AN1292, “Sensorless Field Oriented Control (FOC) for a Permanent Magnet Synchronous Motor (PMSM) Using a PLL Estimator and Field Weakening (FW)”. The block diagram of the PLL estimator is shown in Figure below.</p>
<p align="center">
  <img src="images/pllestimatorblockdiagram.PNG">
</p>

### The following hardware tools are used for the demonstration:

Use one of the High-Voltage Motor Control Development Boards mentioned below:
- dsPICDEM™ MCHV-2 Development Board, [DM330023-2](https://www.microchip.com/en-us/development-tool/dm330023-2)
- dsPICDEM™ MCHV-3 Development Board, [DM330023-3](https://www.microchip.com/en-us/development-tool/dm330023-3)
- High Voltage 3-Phase AC Induction Motor, [AC300023](https://www.microchip.com/en-us/development-tool/AC300023)
- dsPIC33CK256MP508 Internal Op-Amp Motor Control Plug-in module, [MA330041-2](https://www.microchip.com/en-us/development-tool/MA330041-2)
- dsPIC33CK256MP508 External Op-Amp Motor Control Plug-in module, [MA330041-1](https://www.microchip.com/en-us/development-tool/MA330041-1)

Maximum rated current of the induction motor (AC300023) used for the demonstration is 0.25A.  The default gain of the internal amplifier is 15, capable of measuring a maximum peak current of 11A. To improve the measurement accuracy, modify the amplifier gain so that the maximum peak current is 1A. 

- Readme document for Internal op-amp configuration [Readme for Internal Op-Amp](mchv3_dspic33ck256mp508_an1206/docs/README_internal.md)

The default gain of the external amplifiers on the MCHV-2 or MCHV-3 is 10 (the maximum measurable peak current is 16.5A). 

- Readme document for External op-amp configuration [Readme for External Op-Amp](mchv3_dspic33ck256mp508_an1206/docs/README_external.md)

</br>

> **NOTE :**
> - <p style='text-align: justify;'> The hardware setting for internal opamp configuration is altered to get a peak current of 1A. This is done to enable MCHV-2 or MCHV-3 board to run the  induction motor -AC300023 whose maximum rated current is only 0.25A.</p>
> - <p style='text-align: justify;'>The hardware setting for external opamp configuration is left unaltered (with a peak current of 16.5A). This enables user to run higher power motors on MCHV-2 or MCHV-3 board. But, with the default external amplifier gain settings, the induction motor - AC300023 may not run properly or may stall at certain speeds, due to lower current resolution.</p>





