;======================================================================
;
; File Name     : SGTI3C.ASM
; 
; Originator    : Advanced Embedded Control (AEC)
;                 Texas Instruments Inc.
; 
; Description   : This file contain source code for Three channel
;                 SIN generator module(Using Table look-up and Linear Interpolation)
;                 * Table look-up and linear interpolation provides, low
;                 THD and high phase Resolution
;               
; Routine Type  : CcA
;               
; Date          : 28/12/2001 (DD/MM/YYYY)
;======================================================================
; Description:
;                            ___________________
;                           |                   |
;                           |                   |------o OUT1
;        gain   o---------->|                   |
;        offset o---------->|     SGENTI_3      |------o OUT2
;        freq   o---------->|                   |
;                           |                   |------o OUT3
;                           |___________________|
;
;======================================================================

;======================================================================
; Function Local Frame
;======================================================================
;    _______
;   |_______|<- Stack Pointer                           (FP+2) <---AR1
;   |_______|<- Temp                                    (FP+1) 
;   |_______|<- Register to Register Tfr & Computation  (FP)   <---AR0 
;   |_______|<- Old FP                                  (FP-1)
;   |_______|<- Return Address of the Caller            (FP-2) 
;   |_______|<- Module Handle                           (FP-3) 
;======================================================================
; ###########################################################################
; $TI Release: C28x SGEN Library v1.02.00.00 $
; $Release Date: Fri Feb 12 19:16:58 IST 2021 $
; $Copyright: Copyright (C) 2011-2021 Texas Instruments Incorporated -
;             http://www.ti.com/ ALL RIGHTS RESERVED $
; ###########################################################################

; Module definition for external reference
                .def    _SGENTI_3_calc     
                .ref    SINTAB_360   
ALPHA120        .set    05555h 
ALPHA240        .set    0AAABh 
                
__SGEN3_calc_frs    .set    00002h  ; Local frame size for this routine

_SGENTI_3_calc: 
            SETC    SXM,OVM         ; XAR4->freq
            MOVL    XAR5,#SINTAB_360

; Obtain the step value in pro-rata with the freq input         
            MOV     T,*XAR4++       ; XAR4->step_max, T=freq
            MPY     ACC,T,*XAR4++   ; XAR4->alpha, ACC=freq*step_max (Q15)
            MOVH    AL,ACC<<1           
            
; Increment the angle "alpha" by step value             
            ADD     AL,*XAR4        ; AL=(freq*step_max)+alpha  (Q0)
            MOV     *XAR4,AL        ; XAR4->alpha, alpha=alpha+step (Unsigned 8.8 format)   

; OUT1 Computation
; Obtain the SIN of the angle "X=alpha" using direct Look-up Table            
            MOVB    XAR0,#0     
            MOV     T,#0          
            MOVB    AR0,AL.MSB      ; AR0=indice (alpha/2^8)
            MOVB    T,AL.LSB        ; T=(X-X1) in Q8 format

                       
            MOV     ACC,*+XAR5[AR0] ; ACC=Y1=*(SINTAB_360 + indice)
            ADDB    XAR0,#1
            MOV     PL,*+XAR5[AR0]  ; PL=Y2
            SUB     PL,AL           ; PL=Y2-Y1 in Q15 
            MPY     P,T,PL          ; P=Y2-Y1 in Q23
            LSL     ACC,8           ; ACC=Y1 in Q23
            ADDL    ACC,P           ; Y=Y1+(Y2-Y1)*(X-X1)
            MOVH    T,ACC<<8        ; T=Y in Q15 format
        
            MPY     ACC,T,*+XAR4[1] 
            LSL     ACC,#1          ; ACC=Y*gain (Q31)
            ADD     ACC,*+XAR4[2]<<16 ; ACC=Y*gain+offset
            MOV     *+XAR4[3],AH    ; out1=Y*gain+offset

; OUT2 Computation    
; Add 120deg phase with the "alpha" of OUT1   
            MOVU    ACC,*XAR4       ; ACC=alpha
            ADD     ACC,#ALPHA120   ; ACC=alpha+120
               
            MOVB    XAR0,#0    
            MOV     T,#0    
            MOVB    AR0,AL.MSB      ; AR0=indice (alpha/2^8)
            MOVB    T,AL.LSB        ; T=(X-X1) in Q8 format

                       
            MOV     ACC,*+XAR5[AR0] ; ACC=Y1=*(SINTAB_360 + indice)
            ADDB    XAR0,#1
            MOV     PL,*+XAR5[AR0]  ; PL=Y2
            SUB     PL,AL           ; PL=Y2-Y1 in Q15 
            MPY     P,T,PL          ; P=Y2-Y1 in Q23
            LSL     ACC,8           ; ACC=Y1 in Q23
            ADDL    ACC,P           ; Y=Y1+(Y2-Y1)*(X-X1)
            MOVH    T,ACC<<8        ; T=Y in Q15 format
         
            MPY     ACC,T,*+XAR4[1] 
            LSL     ACC,#1          ; ACC=Y*gain (Q31)
            ADD     ACC,*+XAR4[2]<<16 ; ACC=Y*gain+offset
            MOV     *+XAR4[4],AH    ; out2=Y*gain+offset 
            
; OUT3 Computation    
; Add 240deg phase with the "alpha" of OUT1   
            MOVU    ACC,*XAR4       ; ACC=alpha
            ADD     ACC,#ALPHA240   ; ACC=alpha+120
               
            MOVB    XAR0,#0    
            MOV     T,#0    
            MOVB    AR0,AL.MSB      ; AR0=indice (alpha/2^8)
            MOVB    T,AL.LSB        ; T=(X-X1) in Q8 format

                       
            MOV     ACC,*+XAR5[AR0] ; ACC=Y1=*(SINTAB_360 + indice)
            ADDB    XAR0,#1
            MOV     PL,*+XAR5[AR0]  ; PL=Y2
            SUB     PL,AL           ; PL=Y2-Y1 in Q15 
            MPY     P,T,PL          ; P=Y2-Y1 in Q23
            LSL     ACC,8           ; ACC=Y1 in Q23
            ADDL    ACC,P           ; Y=Y1+(Y2-Y1)*(X-X1)
            MOVH    T,ACC<<8        ; T=Y in Q15 format
         
            MPY     ACC,T,*+XAR4[1] 
            LSL     ACC,#1          ; ACC=Y*gain (Q31)
            ADD     ACC,*+XAR4[2]<<16 ; ACC=Y*gain+offset
            MOV     *+XAR4[5],AH    ; out3=Y*gain+offset                  
        	CLRC 	OVM
        	LRETR
            
            

