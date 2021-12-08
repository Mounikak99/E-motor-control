;;#############################################################################
;;! \file source/sel_q.asm
;;!
;;! \brief  Selects the Q format for the twiddle factors
;;!
;;! \date   Feb 25, 2002
;;! 
;;
;;  Group:            C2000
;;  Target Family:    C28x
;;
;;#############################################################################
;;$TI Release: C28x Fixed Point DSP Library v1.22.00.00.21 $
;;$Release Date: Fri Feb 12 19:16:27 IST 2021 $
;;$Copyright: Copyright (C) 2014-2021 Texas Instruments Incorporated -
;;            http://www.ti.com/ ALL RIGHTS RESERVED $
;;#############################################################################
;;
;;*****************************************************************************
;; defines
;;*****************************************************************************
Q30             .set    30
Q31             .set    31

; Select Twiddle factor Q format
TF_QFMAT        .set    Q30    
        
;;#############################################################################
;;  End of File
;;#############################################################################