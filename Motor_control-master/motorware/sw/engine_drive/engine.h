/*
 * engine.h
 *
 *  Created on: 04-Sep-2020
 */

/* --COPYRIGHT--,BSD
 * (C) Copyright 2020, Mofu eDrive and Controls (OPC) Private Ltd
 * All rights reserved.
 */


// **************************************************************************
#ifndef ENGINE_H_
#define ENGINE_H_

#include "sw/modules/math/src/32b/math.h"
#include "sw/modules/iqmath/src/32b/IQmathLib.h"
#include "sw/modules/types/src/types.h"

#ifdef __cplusplus
extern "C" {
#endif


#define START_DEADBAND_IN_PERCENTAGE    (float_t)(42.4242424)       //(28.1818182)
#define END_DEADBAND_IN_PERCENTAGE  (float_t)(24.545455)
#define NO_OF_SPEED_DIVISION    (float_t)(10)
#define POTENTIOMETER_RANGE (float_t)(3.3)
#define ADC_VALUE   (float_t)(3200)

#define START_TPS ((START_DEADBAND_IN_PERCENTAGE / 100) * POTENTIOMETER_RANGE)
#define END_TPS (POTENTIOMETER_RANGE -((END_DEADBAND_IN_PERCENTAGE / 100) * POTENTIOMETER_RANGE))
#define CONVERSION_SCALE ((END_TPS) / NO_OF_SPEED_DIVISION)

#define ENGINE_VARS_DEFAULTS {false, \
    ENGINE_DriveMode_Forward, \
    ENGINE_ErrorCode_NoError, \
    0, \
    0}


//! \brief Enumeration for Engine error codes
typedef enum
{
    ENGINE_ErrorCode_NoError=0,                 //!< No error
    ENGINE_ErrorCode_OverTemprature=1,          //!< engine over heated
    ENGINE_ErrorCode_StandCondition=2,          //!< vehicle stand condition
    ENGINE_ErrorCode_OverCurrent=3              //!< over current
}ENGINE_ErrorCode_e;

//! \brief Enumeration for Engine drive mode
typedef enum
{
    ENGINE_DriveMode_Forward=0,                 //!< Engine moves in forward direction
    ENGINE_DriveMode_Reverse=1,                 //!< Engine moves in revrese direction
}ENGINE_DriveMode_e;

//! \brief  Defines the Engine objec
//! \detail Contains configuration parameters of Engine component
typedef struct _ENGINE_Obj_
{
    bool    TpsOnOff;                           //!< Throttle state false: Off  true: On
    ENGINE_DriveMode_e  mode;                   //!< drive mode
    ENGINE_ErrorCode_e  error;                  //!< error type
    _iq TpsValue;                               //!< throttle position value numerical value 1,2,....
    _iq SpeedFeedback;                          //!< engine speed feedback
}ENGINE_Obj;

typedef struct _ENGINE_Obj_ *ENGINE_Handle;

extern ENGINE_Handle ENGINE_init(void *pMemory, const size_t numBytes);


//! \brief      Gets the throttle value
//! \param[in]  handle  The Engine handle
//! \return     The throttle position value
extern _iq ENGINE_getTpsValue(ENGINE_Handle handle);



//! \brief      Gets the Engine error
//! \param[in]  handle  The Engine handle
//! \return     The error code of engine
extern ENGINE_ErrorCode_e ENGINE_getErrorCode(ENGINE_Handle handle);


//! \brief      Gets the drive mode
//! \param[in]  handle  The Engine handle
//! \return     The drive mode of engine
extern ENGINE_DriveMode_e ENGINE_getDriveMode(ENGINE_Handle handle);


//! \brief      Determines if the throttle has been identified
//! \param[in]  handle  The Engine handle
//! \return     A boolean value denoting if the throttle is identified (true) or not (false)
extern bool ENGINE_isTpsOnOff(ENGINE_Handle handle);


//! \brief      Sets the enable/disable of throttle input in the engine
//! \param[in]  handle  The Engine handle
extern void ENGINE_setTpsOnOff(ENGINE_Handle handle, const bool state);


//! \brief      Checks for the engine error
//! \param[in]  handle  The Engine handle
extern void ENGINE_checkForErrors(ENGINE_Handle handle);


//! \brief      Sets the Engine error
//! \param[in]  handle  The Engine handle

extern void ENGINE_setErrorCode(ENGINE_Handle handle, const ENGINE_ErrorCode_e errorCode);


//! \brief      Sets the drive mode of engine
//! \param[in]  handle  The Engine handle


extern void ENGINE_setDriveMode(ENGINE_Handle handle, const ENGINE_DriveMode_e driveMode);

// extern void ENGINE_setTpsValue(ENGINE_Handle handle, _iq gPot);



#ifdef __cplusplus
}
#endif // extern "C"


#endif /* ENGINE_H_ */

