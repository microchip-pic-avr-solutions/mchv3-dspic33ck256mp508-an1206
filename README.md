![image](images/microchip.jpg) 

# MCHV3 DSPIC33CK256MP508 AN1206

## INTRODUCTION
<p style='text-align: justify;'>
This document describes the setup requirements for running the Sensor-less FOC algorithm for an Induction Motor, which is referenced in AN1206 “Sensorless Field Oriented Control (FOC) of a Three-Phase AC Induction Motor (ACIM)” </p>
<p style='text-align: justify;'>
The demonstration is configured to run on either the dsPICDEM™ MCHV-2 Development Board or the dsPICDEM™ MCHV-3 Development Board in the Internal Op-Amp configuration with the dsPIC33CK256MP508 Internal Op-Amp Motor Control Plug-In Module (PIM).
</p>
The control of induction machines can be classified into
‘scalar (v/f)’ and ‘vector’ controls. Scalar controls are
simple to implement and offer good steady-state
response. However, the dynamics are slow because
the transients are not controlled. To obtain high precision
and good dynamics, the Field Oriented Control
(FOC), also known as vector control, provides the best
solution. Flux position information is required to implement
vector control. Using position/speed sensors may
result in many practical problems, such as complexity
of hardware, difficulties in application meant for hostile
environments, increased cost, reduced reliability due to
the cables and sensor itself, difficulties of mechanical
attachment of the sensor to the electric machine,
increased axial length of the machine and electromagnetic
noise interference. These problems with a
physical sensor can be mitigated by estimating the speed/
position information.

- Readme document for external op-amp configuration [Readme for external opamp](mchv3_dspic33ck256mp508_an1206/docs/README_external.md)
- Readme document for internal op-amp configuration [Readme for internal opamp](mchv3_dspic33ck256mp508_an1206/docs/README_internal.md)

## Summary


## Related Documentation


## Software Used 


## Hardware Used


## Setup


## Operation



