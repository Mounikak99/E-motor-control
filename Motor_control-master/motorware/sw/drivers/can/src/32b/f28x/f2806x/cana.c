/* --COPYRIGHT--,BSD
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
//! \file    sw/drivers/can/src/32b/f28x/f2806x/cana.c
//! \brief   The functions in this file are used to configure the configure
//!          eCAN registers.
//! \author  Vivishekh sivan
//! \date    25.05.2021
//! \company Mofu eDrives and Controls pvt. ltd.
//!
//! NOTE: #define ECANA2_0B in hal_obj.h and clk.h file to enable CANA module.
//! (C) Copyright 2015, Texas Instruments, Inc.


// **************************************************************************
// the includes

#include "sw/drivers/can/src/32b/f28x/f2806x/can.h"

// **************************************************************************
// the defines


// **************************************************************************
// the globals

//Only 32-bit accesses are allowed to the Control and Status registers.
//16-bit access to these registers could potentially corrupt the register contents or return false data.
//The C2000Ware files released by TI employs a **shadow register structure that aids in 32-bit access.

struct ECAN_REGS  EcanaShadowReg;

// **************************************************************************
// the functions


//Initializes the eCAN Control and Status Registers object handle
ECAN_REGS_Handle ECAN_REGS_init(void *pMemory,const size_t numBytes)
{
    ECAN_REGS_Handle    ecanRegsHandle;

    if(numBytes < sizeof(struct ECAN_REGS))
        return((ECAN_REGS_Handle)NULL);

    // assign the handle
    ecanRegsHandle = (ECAN_REGS_Handle)pMemory;

    return(ecanRegsHandle);
} // end of ECAN_REGS_init() function


//Initializes the eCAN Message Mailbox Registers object handle
ECAN_MBOXES_Handle ECAN_MBOXES_init(void *pMemory,const size_t numBytes)
{
    ECAN_MBOXES_Handle    ecanMboxesHandle;


    if(numBytes < sizeof(struct ECAN_MBOXES))
        return((ECAN_MBOXES_Handle)NULL);

    // assign the handle
    ecanMboxesHandle = (ECAN_MBOXES_Handle)pMemory;

    return(ecanMboxesHandle);
} // end of ECAN_MBOXES_init() function


// **********************************************************************************
//                      Configure the eCAN for operation

//1a.Set the CANTX  pins to CAN functions
//The CANTX pin is used for the CAN transmit functions.
void CAN_setTxPin(ECAN_REGS_Handle ecanRegsHandle)
{
    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    EcanaShadowReg.CANTIOC.all = ecanRegsHandle->CANTIOC.all;
    EcanaShadowReg.CANTIOC.bit.TXFUNC = 1;           //CAN transmit functions
    ecanRegsHandle->CANTIOC.all = EcanaShadowReg.CANTIOC.all;

    DISABLE_PROTECTED_REGISTER_WRITE_MODE;
    return;
} //end of CAN_setTxPin  () function


//1b.Set the CANRX pins to CAN functions
//The CANRX pin is used for the CAN receive functions.
void CAN_setRxPin(ECAN_REGS_Handle ecanRegsHandle)
{
    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    EcanaShadowReg.CANRIOC.all = ecanRegsHandle->CANRIOC.all;
    EcanaShadowReg.CANRIOC.bit.RXFUNC = 1;           //CAN receive functions
    ecanRegsHandle->CANRIOC.all = EcanaShadowReg.CANRIOC.all;

    DISABLE_PROTECTED_REGISTER_WRITE_MODE;
    return;
} // end of CAN_setRxPin () function


//2.Reset CAN module
void CAN_SoftReset(ECAN_REGS_Handle ecanRegsHandle)
{
    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    EcanaShadowReg.CANMC.all = ecanRegsHandle->CANMC.all;
    EcanaShadowReg.CANMC.bit.SRES = 1;               //Software reset of the module
    ecanRegsHandle->CANMC.all = EcanaShadowReg.CANMC.all;

    DISABLE_PROTECTED_REGISTER_WRITE_MODE;

    return;
}//end of CAN_SoftReset() function


//3.After a reset, bit CCR (CANMC.12) and bit CCE (CANES.4) are set to 1.
//This allows the user to configure the bit-timing configuration register (CANBTC).
//If the CCE bit is set (CANES.4 = 1), proceed to next step; otherwise,
//set the CCR bit(CANMC.12 = 1) and wait until CCE bit is set (CANES.4 = 1).
void CAN_checkForCANBTC(ECAN_REGS_Handle ecanRegsHandle)
{

    do
    {
        EcanaShadowReg.CANES.all = ecanRegsHandle->CANES.all;

        if(EcanaShadowReg.CANES.bit.CCE != 1 || EcanaShadowReg.CANMC.bit.CCR != 1 )
        {
            ENABLE_PROTECTED_REGISTER_WRITE_MODE;

            EcanaShadowReg.CANMC.all = ecanRegsHandle->CANMC.all;
            EcanaShadowReg.CANMC.bit.CCR = 1;
            ecanRegsHandle->CANMC.all = EcanaShadowReg.CANMC.all;

            DISABLE_PROTECTED_REGISTER_WRITE_MODE;
        }
    }while(EcanaShadowReg.CANES.bit.CCE != 1);

    return;
}//end of CAN_checkForCANBTC() function


//4.Program the CANBTC register with the appropriate timing values. Make sure that the values
//TSEG1 and TSEG2 are not 0. If they are 0, the module does not leave the initialization mode
void CAN_congif_CANBTC(ECAN_REGS_Handle ecanRegsHandle)
{
    // The following block is for 90 MHz SYSCLKOUT.
    // (45 MHz CAN module clock Bit rate = 1 Mbps)
    // See Note at end of file.

    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    EcanaShadowReg.CANBTC.all = ecanRegsHandle->CANBTC.all;
    EcanaShadowReg.CANBTC.bit.BRPREG = 2;    //Baud rate prescaler
    EcanaShadowReg.CANBTC.bit.TSEG1REG = 10; //Time segment 1
    EcanaShadowReg.CANBTC.bit.TSEG2REG = 2;  //Time segment 2
    EcanaShadowReg.CANBTC.bit.SAM = 1;       //number of samples
    ecanRegsHandle->CANBTC.all = EcanaShadowReg.CANBTC.all;

    DISABLE_PROTECTED_REGISTER_WRITE_MODE;

    return;
}//end of CAN_congif_CANBTC() function

//5.Set SCC: that is not required for this setup.

//6.Program the master control register (CANMC).
void CAN_config_CANMC(ECAN_REGS_Handle ecanRegsHandle)
{
    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    EcanaShadowReg.CANMC.bit.CCR = 0;  //Change-configuration request
    EcanaShadowReg.CANMC.bit.PDR = 0;  //Power down mode request
    EcanaShadowReg.CANMC.bit.DBO = 0;  //Data byte order
    EcanaShadowReg.CANMC.bit.WUBA = 0; //Wake up on bus activity
    EcanaShadowReg.CANMC.bit.CDR = 0;  //Change data field request
    EcanaShadowReg.CANMC.bit.ABO = 0;  //Auto bus on
    EcanaShadowReg.CANMC.bit.STM = 0;  //Self test mode
    EcanaShadowReg.CANMC.bit.SRES = 0; //Software reset of the module
    EcanaShadowReg.CANMC.bit.SCB = 1;  //Std or ext CAN option

    DISABLE_PROTECTED_REGISTER_WRITE_MODE;

    EcanaShadowReg.CANMC.bit.MBNR = 0; //Mailbox for which the CPU requests a write access to the data field
    EcanaShadowReg.CANMC.bit.SUSP = 1; //SUSPEND

    ecanRegsHandle->CANMC.all = EcanaShadowReg.CANMC.all;

    return;
}// end of CAN_config_CANMC() function


//7.Initialize all bits of MSGCTRLn registers to zero.
void CAN_Reset_MSGCTRL(ECAN_MBOXES_Handle ecanMboxesHandle)
{
    ecanMboxesHandle->MBOX0.MSGCTRL.all = 0x00000000;
    ecanMboxesHandle->MBOX1.MSGCTRL.all = 0x00000000;
    ecanMboxesHandle->MBOX2.MSGCTRL.all = 0x00000000;
    ecanMboxesHandle->MBOX3.MSGCTRL.all = 0x00000000;
    ecanMboxesHandle->MBOX4.MSGCTRL.all = 0x00000000;
    ecanMboxesHandle->MBOX5.MSGCTRL.all = 0x00000000;
    ecanMboxesHandle->MBOX6.MSGCTRL.all = 0x00000000;
    ecanMboxesHandle->MBOX7.MSGCTRL.all = 0x00000000;
    ecanMboxesHandle->MBOX8.MSGCTRL.all = 0x00000000;
    ecanMboxesHandle->MBOX9.MSGCTRL.all = 0x00000000;
    ecanMboxesHandle->MBOX10.MSGCTRL.all = 0x00000000;
    ecanMboxesHandle->MBOX11.MSGCTRL.all = 0x00000000;
    ecanMboxesHandle->MBOX12.MSGCTRL.all = 0x00000000;
    ecanMboxesHandle->MBOX13.MSGCTRL.all = 0x00000000;
    ecanMboxesHandle->MBOX14.MSGCTRL.all = 0x00000000;
    ecanMboxesHandle->MBOX15.MSGCTRL.all = 0x00000000;
    ecanMboxesHandle->MBOX16.MSGCTRL.all = 0x00000000;
    ecanMboxesHandle->MBOX17.MSGCTRL.all = 0x00000000;
    ecanMboxesHandle->MBOX18.MSGCTRL.all = 0x00000000;
    ecanMboxesHandle->MBOX19.MSGCTRL.all = 0x00000000;
    ecanMboxesHandle->MBOX20.MSGCTRL.all = 0x00000000;
    ecanMboxesHandle->MBOX21.MSGCTRL.all = 0x00000000;
    ecanMboxesHandle->MBOX22.MSGCTRL.all = 0x00000000;
    ecanMboxesHandle->MBOX23.MSGCTRL.all = 0x00000000;
    ecanMboxesHandle->MBOX24.MSGCTRL.all = 0x00000000;
    ecanMboxesHandle->MBOX25.MSGCTRL.all = 0x00000000;
    ecanMboxesHandle->MBOX26.MSGCTRL.all = 0x00000000;
    ecanMboxesHandle->MBOX27.MSGCTRL.all = 0x00000000;
    ecanMboxesHandle->MBOX28.MSGCTRL.all = 0x00000000;
    ecanMboxesHandle->MBOX29.MSGCTRL.all = 0x00000000;
    ecanMboxesHandle->MBOX30.MSGCTRL.all = 0x00000000;
    ecanMboxesHandle->MBOX31.MSGCTRL.all = 0x00000000;

    return;
}// end of CAN_Reset_MSGCTRL() function


//8.Verify the CCE(Change configuration enable) bit is cleared (CANES.4 = 0),
//indicating that the CAN module has been configured.
void CAN_checkForCCEBit(ECAN_REGS_Handle ecanRegsHandle)
{
    do
    {
        EcanaShadowReg.CANES.all = ecanRegsHandle->CANES.all;

    }while (EcanaShadowReg.CANES.bit.CCE != 0);

    return;
}// end of CAN_checkForCCEBit() function

//                      End of configure the eCAN for operation
// ******************************************************************************

// ******************************************************************************
//                        Configuring mailbox to transmit

//1.Clear the appropriate bit in the CANTRS register:
//Clear CANTRS.x = 0 (Writing a 0 to TRS has no effect; instead, set TRR.x and wait until TRS.x
//clears.) If the RTR bit is set, the TRS bit can send a remote frame. Once the remote frame is sent, the
//TRS bit of the mailbox is cleared by the CAN module. The same node can be used to request a data
//frame from another node.
// Note: x = 0-31 mailbox
void CAN_ClearCANTRSBit(ECAN_REGS_Handle ecanRegsHandle )
{
    do
    {
        EcanaShadowReg.CANTRR.all = ecanRegsHandle->CANTRR.all;
        EcanaShadowReg.CANTRR.bit.TRR0 = 1;
        ecanRegsHandle->CANTRR.all =  EcanaShadowReg.CANTRR.all;

        EcanaShadowReg.CANTRS.all = ecanRegsHandle->CANTRS.all;

    }while(EcanaShadowReg.CANTRS.bit.TRS0 == 1);

    return;
}// end of CAN_ClearCANTRSBit() function


//2.Disable the mailbox by clearing the corresponding bit in the mailbox enable (CANME) register.
void CAN_DisableMailbox(ECAN_REGS_Handle ecanRegsHandle)
{
    EcanaShadowReg.CANME.all = ecanRegsHandle->CANME.all;
    EcanaShadowReg.CANME.bit.ME0 = 0;                  // disable mailbox0
    ecanRegsHandle->CANME.all = EcanaShadowReg.CANME.all;

    return;
}// end of CAN_DisableMailbox() function


//3.Load the message identifier (MSGID) register of the mailbox. Clear the AME (MSGID.30) and AAM
//(MSGID.29) bits for a normal send mailbox (MSGID.30 = 0 and MSGID.29 = 0). This register is usually
//not modified during operation. It can only be modified when the mailbox is disabled.
void CAN_LoadIdentifier(ECAN_REGS_Handle ecanRegsHandle,ECAN_MBOXES_Handle ecanMboxesHandle )
{
    ecanMboxesHandle->MBOX0.MSGID.all = 0x95555555; //Cleared AME and AAM bit for MBOX0
    ecanMboxesHandle->MBOX0.MSGCTRL.bit.DLC = 8;    //data length MBOX0

    EcanaShadowReg.CANMD.all = ecanRegsHandle->CANMD.all;
    EcanaShadowReg.CANMD.bit.MD0 = 0;                //Set the mailbox0 direction {0-Tx,1-Rx}
    ecanRegsHandle->CANMD.all = EcanaShadowReg.CANMD.all;

    return;
}// end of CAN_LoadIdentifier() function


//4.Set the mailbox enable by setting the corresponding bit in the CANME register
void CAN_EnableMailBox(ECAN_REGS_Handle ecanRegsHandle)
{
    EcanaShadowReg.CANME.all = ecanRegsHandle->CANME.all;
    EcanaShadowReg.CANME.bit.ME0 = 1;                    //mailbox0 is enable
    ecanRegsHandle->CANME.all = EcanaShadowReg.CANME.all;

    return;
}// end of CAN_EnableMailBox() function

//                      End of Configuring mailbox to transmit
// ***********************************************************************************


// ***********************************************************************************
//                       Transmitting a Message

//2.Set the corresponding flag in the transmit request register (CANTRS.1 = 1)
//to start the transmission of the message.
//The CAN module now handles the complete transmission of the CAN message.
void CAN_enableTx(ECAN_REGS_Handle ecanRegsHandle)
{
    EcanaShadowReg.CANTRS.all = ecanRegsHandle->CANTRS.all;
    EcanaShadowReg.CANTRS.bit.TRS0 = 1;
    ecanRegsHandle->CANTRS.all = EcanaShadowReg.CANTRS.all;

    return;
}//end of CAN_enableTx() function


//3.Wait until the transmit-acknowledge flag of the corresponding mailbox is set (TA.1 = 1).
//After a successful transmission, this flag is set by the CAN module.
void CAN_ackTx(ECAN_REGS_Handle ecanRegsHandle)
{
    do
    {
        EcanaShadowReg.CANTA.all = ecanRegsHandle->CANTA.all;

    }while(EcanaShadowReg.CANTA.bit.TA0 == 0);

    return;
}// end of CAN_ackTx() function


//4. The TRS flag is reset to 0 by the module after a successful or aborted transmission.
//Nothing to do.


//5. The transmit acknowledge must be cleared for the next transmission (from the same mailbox).
void CAN_clearAckTx(ECAN_REGS_Handle ecanRegsHandle)
{
    EcanaShadowReg.CANTA.all = ecanRegsHandle->CANTA.all;
    EcanaShadowReg.CANTA.bit.TA0 = 1;
    ecanRegsHandle->CANTA.all = EcanaShadowReg.CANTA.all;
    do
    {

        EcanaShadowReg.CANTA.all = ecanRegsHandle->CANTA.all;

    }while(EcanaShadowReg.CANTA.bit.TA0 == 1);

    return;
}

//6.To transmit another message in the same mailbox,
//Repeat from step 1.

//1.Write the message data into the mailbox data field (added in main.c file)
//ecanMboxesHandle->MBOX0.MDL.all = 0x55555555;
//ecanMboxesHandle->MBOX0.MDH.all = 0x55555555;


//                     End of Transmitting a Message
// ***************************************************************************************

//
// Note: Bit timing parameters must be chosen based on the network parameters
// such as the sampling point desired and the propagation delay of the network.
// The propagation delay is a function of length of the cable, delay introduced
// by the transceivers and opto/galvanic-isolators (if any).
//
// The parameters used in this file must be changed taking into account the
// above mentioned factors in order to arrive at the bit-timing parameters
// suitable for a network.
//

