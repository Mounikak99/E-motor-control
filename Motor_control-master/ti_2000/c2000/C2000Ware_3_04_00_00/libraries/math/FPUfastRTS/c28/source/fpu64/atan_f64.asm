;;#############################################################################
;;! \file source/fpu64/atan_f64.asm
;;!
;;! \brief  Fast arctangent function for the C28x + FPU64
;;! \author Vishal Coelho   
;;! \date   03/02/2016
;;
;; DESCRIPTION:
;;
;;   This function computes a 64-bit floating point arctangent given a
;;   numerical input.  This function uses the FPU64 math tables to
;;   compute the atan.  
;;
;; FUNCTIONS:
;;
;;  float64 atan (float64 Y) 
;;  
;; ASSUMPTIONS:
;;
;;     * FPU64atan2Table is linked into the project.
;;
;; ALGORITHM:
;;   Step(1):   if( 1.0 >= abs(Y) )
;;                  Numerator   = abs(Y)
;;                  Denominator = 1.0
;;              else
;;                  Numerator   = 1.0
;;                  Denominator = abs(Y)
;;
;;   Step(2):   Ratio = Numerator/Denominator
;;
;;              Note: Ratio range = 0.0 to 1.0
;;
;;   Step(3):   Use the upper 6-bits of the "Ratio" value as an
;;              index into the table to obtain the coefficients
;;              for a second order equation:
;;
;;              _FPU64atan2Table:
;;                   CoeffA0[0]
;;                   CoeffA1[0]
;;                   CoeffA2[0]
;;                      .
;;                      .
;;                   CoeffA0[63]
;;                   CoeffA1[63]
;;                   CoeffA2[63]
;;
;;   Step(4):   Calculate the angle using the folowing equation:
;;
;;              arctan(Ratio) = A0 + A1*Ratio + A2*Ratio*Ratio
;;              arctan(Ratio) = A0 + Ratio(A1 + A2*Ratio)
;;
;;   Step(5):   The final angle is determined as follows:
;;
;;              if( Y >= 0 and 1.0 >= abs(Y) )
;;                  Angle = arctan(abs(Y)/1.0)
;;              if( Y >= 0 and 1.0 <  abs(Y) )
;;                  Angle = PI/2 - arctan(1.0/abs(Y))
;;              if( Y < 0 )
;;                  Angle = -Angle
;;
;;  Group:            C2000
;;  Target Family:    C28x+FPU64
;;
;;#############################################################################
;; $TI Release: C28x Floating Point Unit Library V2.04.00.00 $
;; $Release Date: Feb 12, 2021 $
;; $Copyright: Copyright (C) 2021 Texas Instruments Incorporated -
;;             http://www.ti.com/ ALL RIGHTS RESERVED $
;;#############################################################################


;; the includes
    .cdecls C, LIST, "fastrts.h"

;; external references
    .if __TI_EABI__
    .asg    FPU64atan2HalfPITable, _FPU64atan2HalfPITable
    .asg    FPU64atan2Table, _FPU64atan2Table
    .asg    atan, _atanl
    .endif
    .ref    _FPU64atan2HalfPITable
    .ref    _FPU64atan2Table
       
;;*****************************************************************************
;; Register Usage:
;;    AR0 : index into the arctan table
;;   XAR1 : 
;;   XAR2 : points to table entries A0.L
;;   XAR3 : points to table entries A0.H
;;   XAR4 : points to table entries A1.L
;;   XAR5 : points to table entries A1.H
;;   XAR6 : points to table entries A2.L
;;   XAR7 : points to table entries A2.H & PI/2
;;
;; Stack Usage:
;;
;;   |_______|<- Stack Pointer                    (SP)
;;   |_______|<- R0H                              (SP-2) ------>Local Frame
;;   |_______|<- R4H                              (SP-4)
;;   |_______|<- XAR3                             (SP-6)
;;   |_______|<- XAR2                             (SP-8)
;;   |_______|<- rpc calling function             (SP-10)
;;*****************************************************************************
    .page
    .global   _atanl
    .sect     ".text"
_atanl:        
    .asmfunc             
    .asg      XAR2, P_A0_L
    .asg      XAR3, P_A0_H
    .asg      XAR4, P_A1_L
    .asg      XAR5, P_A1_H
    .asg      XAR6, P_A2_L
    .asg      XAR7, P_A2_H
    .asg      XAR7, PI_O_2
    .if CONVERT_F32_TO_F64 == 1
    F32TOF64  R0, R0H             ; convert argument to double precision
    .endif
;;-----------------------------------------------------------------------------
;; Context Save
;;-----------------------------------------------------------------------------
    PUSH      XAR2
    PUSH      XAR3
    MOV32     *SP++, R4H

;;-----------------------------------------------------------------------------
;; atan
;;-----------------------------------------------------------------------------
    ; Perform Step (1): R0 = Y (on entry)
    ABSF64    R3, R0              ; R3 = abs(Y)
    ZERO      R4
    MOVIZ     R4H, #0x3FF0        ; R4 = 1.0

    MINF64    R3, R4              ; R3 = Numerator = min(abs(Y),1.0)
 || MOV64     R4, R3              ; R4 = Denominator = max(abs(Y),1.0)
    ;<<VC160304 looks like a NOP is required here for EINVF64 to work properly>>
    .if       0
    NOP
    .endif
    ; Perform Step (2) and (3):
    NOP
    EINVF64   R2, R4              ; *| R2 = Ye = Estimate(1/Numerator)
    TESTTF    LEQ                 ; 1| Set TF if 1.0 >= abs(Y)
    MPYF64    R0, R2, R4          ; *| R0H = Ye*B
 || MOV32     *SP++, R0H          ;  | Store Y (*-SP[2] = Y.H)
    MPYF64    R1, R2, R3          ; 1|*| R1 = 16-bit accurate estimate
    ; Points to A2.L                2|1|
    MOVL      P_A2_L, #_FPU64atan2Table+8
    SUBF64    R0, #2.0, R0        ; *|2| R0 = 2.0 - Ye*B
    MPYF64    R1, R1, #64.0       ; 1|*| 64 = Elements In Table
    ; Points to A2.H                2|1|
    MOVL      P_A2_H, #_FPU64atan2Table+10
    ; R2 = Ye = Ye*(2.0 - Ye*B)  (first estimate)
    MPYF64    R2, R2, R0          ; *|2| 
    ; R1H = int32_t(64*ratio) (32-bit accurate)
    F64TOUI32 R1H, R1             ; 1|*|
    ; Points to A1.L                2|1|
    MOVL      P_A1_L, #_FPU64atan2Table+4
    MPYF64    R0, R2, R4          ; *|   R0= Ye*B
    ; Points to A1.H                1|
    MOVL      P_A1_H, #_FPU64atan2Table+6
    MOV32     ACC, R1H            ; 2|   ACC = int(64*Ratio)
    SUBF64    R0, #2.0, R0        ; *|   R0 = 2.0 - Ye*B
    NOP                           ; 1|
    MPY       P, @AL, #12         ; 2|   P = 12 * int(64*Ratio)
    MPYF64    R2, R2, R0          ; *|   R2 = Ye = Ye*(2.0 - Ye*B)
    MOVZ      AR0, @PL            ; 1|   AR0 = _FPU64atan2Table index
    ; Points to A0.L                2|
    MOVL      P_A0_L, #_FPU64atan2Table
    MPYF64    R0, R2, R4          ; *|   R0= Ye*B    
    ; Points to A0.H                1|
    MOVL      P_A0_H, #_FPU64atan2Table+2
    NOP                           ; 2|
    SUBF64    R0, #2.0, R0        ; *|   R0 = 2.0 - Ye*B
    NOP                           ; 1|
    NOP                           ; 2|
    MPYF64    R2, R2, R0          ; *|   R2 = Ye = Ye*(2.0 - Ye*B)
    NOP                           ; 1|
    NOP                           ; 2|
    MPYF64    R0, R2, R4          ; *|   R0= Ye*B    
    NOP                           ; 1|
    NOP                           ; 2|
    SUBF64    R0, #2.0, R0        ; *|   R0 = 2.0 - Ye*B
    NOP                           ; 1|
    NOP                           ; 2|
    MPYF64    R2, R2, R0          ; *|   R2 = Ye = Ye*(2.0 - Ye*B)
    MOV32     R4L, *+P_A0_L[AR0]  ; 1|   R4 = A0
    MOV32     R4H, *+P_A0_H[AR0]  ; 2|
    
    MPYF64    R0, R3, R2          ; *|   R0 = Ratio = A*Ye = A/B
    ; Perform Step (4):
    ; arctan(Ratio) = A0 + Ratio(A1 + A2*Ratio)
    MOV32     R1L, *+P_A2_L[AR0]  ; 1|   R1 = A2
    MOV32     R1H, *+P_A2_H[AR0]  ; 2|
    MPYF64    R3, R1, R0          ; *|   R3 = A2*Ratio
 || MOV32     R1L, *+P_A1_L[AR0]  ;  |   R1 = A1
    MOV32     R1H, *+P_A1_H[AR0]  ; 1|   
    ; *XAR7[2] == pi/2            ; 2|
    MOVL      XAR7, #_FPU64atan2HalfPITable
    ADDF64    R3, R1, R3          ; *|   R3 = A1 + A2*Ratio
    MOV32     R1L, *+PI_O_2[4]    ; 1|   R1 = pi/2 (no flag change)
    MOV32     R1H, *+PI_O_2[6]    ; 2| 
    MPYF64    R3, R0, R3          ; *|   R3 = Ratio*(A1 + A2*Ratio)
    NOP                           ; 1|
    NOP                           ; 2|
    ; R0 = arctan(Ratio) = A0 + Ratio(A1 + A2*Ratio)
    ADDF64    R0, R4, R3          ; *|   
    NOP                           ; 1|
    ; Perform Step (5):
    ZERO      R3                  ; 2|
    NEGF64    R0, R0, UNC         ; R0 = flip sign of atan(Ratio)
    NEGF64    R0, R0, TF          ; if (1.0 >= abs(Y)) flip sign of atan(Ratio)                            

    ; <<VC160303
    ; we only care about the leading 32-bits as they contain the sign and 
    ; exponent; the sign bit will set/clear the NF while the exponent sets
    ; the ZF when its value is 0 - we dont care about a non-zero mantissa in 
    ; this case since we treat subnormal numbers as 0 anyway
    ;  VC160303>>
    MOV32     R2H, *--SP          ; R2H = Y.H (set/clear NF,ZF)     

    MOV64     R3, R1, NTF         ; if(1.0 < abs(Y) R3 = pi/2, else R3 = 0.0
    ADDF64    R0, R0, R3          ; *|   R0 = Angle
;;-----------------------------------------------------------------------------
;; Context Restore (and final operations)
;;-----------------------------------------------------------------------------
    ; Restore save on entry registers
    MOV32     R4H, *--SP, UNC     ; 1|
    POP       XAR3                ; 2| 
    NEGF64    R0, R0, LT          ; if (Y < 0) Angle = -Angle
    POP       XAR2
    LRETR                   
    .unasg    P_A0_L
    .unasg    P_A0_H
    .unasg    P_A1_L
    .unasg    P_A1_H
    .unasg    P_A2_L
    .unasg    P_A2_H    
    .unasg    PI_O_2
    .endasmfunc

;;*****************************************************************************
;; This function will run the function above taking the address of the input  
;; and output as arguments         
;; 
;; Register Usage:
;;   XAR4 : points to the destination argument
;;   XAR5 : points to the source argument
;;
;; Stack Usage:
;;
;;   |_______|<- Stack Pointer (SP)
;;   |_______|<- XAR4          (SP-2)
;;
;;*****************************************************************************
    .if __TI_EABI__
    .asg    run_atan, _run_atan
    .endif
    .global _run_atan
    .sect   ".text"
_run_atan:  .asmfunc
    PUSH    XAR4
    MOV32   R0L, *+XAR5[0]
    MOV32   R0H, *+XAR5[2]
    LCR     #_atanl
    POP     XAR4
    MOV32   *+XAR4[0], R0L
    MOV32   *+XAR4[2], R0H
    LRETR
    .endasmfunc
;; End of File
