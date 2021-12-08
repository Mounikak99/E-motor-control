/*
 * engine.c
 *
 *  Created on: 04-Sep-2020
 *
 */

/* --COPYRIGHT--,BSD
 * (C) Copyright 2020, Mofu eDrive and Controls (OPC) Private Ltd
 * All rights reserved.
 */


// **************************************************************************


#include "engine.h"


ENGINE_Handle ENGINE_init(void *pMemory,const size_t numBytes)
{
    ENGINE_Handle engineHandle;


    if(numBytes < sizeof(ENGINE_Obj))
    {
        return((ENGINE_Handle)NULL);
    }

    // assign the handle
    engineHandle = (ENGINE_Handle)pMemory;

    return(engineHandle);
} // end of ENGINE_init() function


//! \brief      Gets the throttle value
//! \param[in]  handle  The Engine handle
//! \return     The throttle position value
extern _iq ENGINE_getTpsValue(ENGINE_Handle handle)
{
    ENGINE_Obj *obj = (ENGINE_Obj *)handle;
    return(obj->TpsValue);
}// end of ENGINE_getTpsValue() function


//! \brief      Gets the Engine error
//! \param[in]  handle  The Engine handle
//! \return     The error code of engine
extern ENGINE_ErrorCode_e ENGINE_getErrorCode(ENGINE_Handle handle)
{
    ENGINE_Obj *obj = (ENGINE_Obj *)handle;
    return(obj->error);
}// end of ENGINE_getErrorCode() function


//! \brief      Gets the drive mode
//! \param[in]  handle  The Engine handle
//! \return     The drive mode of engine
extern ENGINE_DriveMode_e ENGINE_getDriveMode(ENGINE_Handle handle)
{
    ENGINE_Obj *obj = (ENGINE_Obj *)handle;
    return(obj->mode);
}// end of ENGINE_DriveMode() function


//! \brief      Determines if the throttle has been identified
//! \param[in]  handle  The Engine handle
//! \return     A boolean value denoting if the throttle is identified (true) or not (false)
extern bool ENGINE_isTpsOnOff(ENGINE_Handle handle)
{
    ENGINE_Obj *obj = (ENGINE_Obj *)handle;
    return(obj->TpsOnOff);
}// end of ENGINE_isTpsOnOff() function


//! \brief      Sets the enable/disable of throttle input in the engine
//! \param[in]  handle  The Engine handle
extern void ENGINE_setTpsOnOff(ENGINE_Handle handle, const bool state)
{
    ENGINE_Obj *obj = (ENGINE_Obj *)handle;
    obj->TpsOnOff = state;

    return;
}// end of ENGINE_setTpsOnOff() function


//! \brief      Checks for the engine error
//! \param[in]  handle  The Engine handle
extern void ENGINE_checkForErrors(ENGINE_Handle handle)
{
    ENGINE_setErrorCode(handle, ENGINE_ErrorCode_NoError);

    if(0)
    {
        ENGINE_setErrorCode(handle, ENGINE_ErrorCode_OverTemprature);
    }

    if(0)
    {
        ENGINE_setErrorCode(handle, ENGINE_ErrorCode_StandCondition);
    }

    if(0)
    {
        ENGINE_setErrorCode(handle, ENGINE_ErrorCode_OverCurrent);
    }

    return;
}// end of ENGINE_checkForErrors() function


//! \brief      Sets the Engine error
//! \param[in]  handle  The Engine handle

extern void ENGINE_setErrorCode(ENGINE_Handle handle, const ENGINE_ErrorCode_e errorCode)
{
    ENGINE_Obj *obj = (ENGINE_Obj *)handle;
    obj->error = errorCode;

    return;
}// end of ENGINE_setErrorCode() function


//! \brief      Sets the drive mode of engine
//! \param[in]  handle  The Engine handle


extern void ENGINE_setDriveMode(ENGINE_Handle handle, const ENGINE_DriveMode_e driveMode)
{
    ENGINE_Obj *obj = (ENGINE_Obj *)handle;
    obj-> mode = driveMode;

    return;
}// end of ENGINE_setDriveMode() function





