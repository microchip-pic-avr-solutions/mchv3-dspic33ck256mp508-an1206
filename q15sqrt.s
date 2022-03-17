;*******************************************************************************
; Copyright (c) 2017 released Microchip Technology Inc.  All rights reserved.
;
; SOFTWARE LICENSE AGREEMENT:
;
; Microchip Technology Incorporated ("Microchip") retains all ownership and
; intellectual property rights in the code accompanying this message and in all
; derivatives hereto.  You may use this code, and any derivatives created by
; any person or entity by or on your behalf, exclusively with Microchip's
; proprietary products.  Your acceptance and/or use of this code constitutes
; agreement to the terms and conditions of this notice.
;
; CODE ACCOMPANYING THIS MESSAGE IS SUPPLIED BY MICROCHIP "AS IS".  NO
; WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
; TO, IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A
; PARTICULAR PURPOSE APPLY TO THIS CODE, ITS INTERACTION WITH MICROCHIP'S
; PRODUCTS, COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.
;
; YOU ACKNOWLEDGE AND AGREE THAT, IN NO EVENT, SHALL MICROCHIP BE LIABLE,
; WHETHER IN CONTRACT, WARRANTY, TORT (INCLUDING NEGLIGENCE OR BREACH OF
; STATUTORY DUTY),STRICT LIABILITY, INDEMNITY, CONTRIBUTION, OR OTHERWISE,
; FOR ANY INDIRECT, SPECIAL,PUNITIVE, EXEMPLARY, INCIDENTAL OR CONSEQUENTIAL
; LOSS, DAMAGE, FOR COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE CODE,
; HOWSOEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR
; THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT ALLOWABLE BY LAW,
; MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO THIS CODE,
; SHALL NOT EXCEED THE PRICE YOU PAID DIRECTLY TO MICROCHIP SPECIFICALLY TO
; HAVE THIS CODE DEVELOPED.
;
; You agree that you are solely responsible for testing the code and
; determining its suitability.  Microchip has no obligation to modify, test,
; certify, or support the code.
;
;******************************************************************************/

.global   _Q15SQRT
.global   Q15SQRT

_Q15SQRT:
Q15SQRT:
    mov.w w0,w7
    clr.w w0
    cpsgt.w w0,w7
    nop
    nop
    nop
    mov.d w8,[w15++]
    ff1l w7,w3
    sub.w w3,#2,w1
    sl w7,w1,w2
    mov.w #0x8000,w0
    sub.w w2,w0,w5
    mov.w w5,w4
    sl w5,#1,w5
    mov.w #0x4000,w6
    mul.ss w4,w6,w6
    mul.ss w4,w5,w8
    mov.w #0xf000,w0
    mul.ss w0,w9,w2
    add.w w2,w6,w6
    addc.w w3,w7,w7
    mul.ss w9,w5,w8
    mov.w #0x800,w0
    mul.ss w0,w9,w2
    add.w w2,w6,w6
    addc.w w3,w7,w7
    mul.ss w9,w5,w8
    mov.w #0xfb00,w0
    mul.ss w0,w9,w2
    add.w w2,w6,w6
    addc.w w3,w7,w7
    mul.ss w9,w5,w8
    mov.w #0x380,w0
    mul.ss w0,w9,w2
    add.w w2,w6,w6
    addc.w w3,w7,w7
    mul.ss w9,w5,w8
    mov.w #0xfd60,w0
    mul.ss w0,w9,w2
    add.w w2,w6,w6
    addc.w w3,w7,w7
    mul.ss w9,w5,w8
    mov.w #0x210,w0
    mul.ss w0,w9,w2
    add.w w2,w6,w6
    addc.w w3,w7,w7
    mul.ss w9,w5,w8
    mov.w #0xfe53,w0
    mul.ss w0,w9,w2
    add.w w2,w6,w6
    addc.w w3,w7,w7
    lsr w6,#15,w6
    sl w7,#1,w0
    ior.w w6,w0,w6
    asr w7,#15,w7
    mov.w #0x8000,w0
    add.w w0,w6,w6
    addc.w w7,#0,w7
    lsr w1,#1,w2
    subr.w w2,#16,w0
    lsr w6,w2,w6
    sl w7,w0,w0
    ior.w w6,w0,w6
    asr w7,w2,w7
    btst.c w1,#0
    bra nc, Sqrt_else
    mov.w #0x5a82,w4
    mul.ss w6,w4,w0
    lsr w0,#15,w0
    sl w1,#1,w1
    ior.w w0,w1,w6
Sqrt_else:
    mov.w w6,w0
    mov.d [--w15],w8
    return
    
.end
