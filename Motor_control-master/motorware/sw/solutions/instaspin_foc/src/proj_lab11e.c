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
//! \file   solutions/instaspin_foc/src/proj_lab011e.c
//! \brief Hall sensor start-up with transition to FAST for InstaSPIN-FOC
//!
//! (C) Copyright 2015, Texas Instruments, Inc.

//! \defgroup PROJ_LAB11e PROJ_LAB11e
//@{

//! \defgroup PROJ_LAB11e_OVERVIEW Project Overview
//!
//! Hall sensor start-up with transition to FAST for InstaSPIN-FOC
//! 
//!

// **************************************************************************
// the includes

// system includes
#include <math.h>
#include "main.h"

#ifdef FLASH
#pragma CODE_SECTION(mainISR,"ramfuncs");
#pragma CODE_SECTION(runSetTrigger,"ramfuncs");
#pragma CODE_SECTION(runFieldWeakening,"ramfuncs");
#pragma CODE_SECTION(runCurrentReconstruction,"ramfuncs");
#pragma CODE_SECTION(angleDelayComp,"ramfuncs");
#pragma CODE_SECTION(HALLBLDC_State_Check,"ramfuncs");
#pragma CODE_SECTION(HALLBLDC_Ctrl_PwmSet,"ramfuncs");
#pragma CODE_SECTION(HALLBLDC_Ctrl_Run,"ramfuncs");
#endif

// Include header files used in the main function

// **************************************************************************
// the defines

#define LED_BLINK_FREQ_Hz   5

#define HAL_GPIO_SW1 GPIO_Number_9 // Used for gEnginestartSW (SW1 in board)
#define HAL_GPIO_SW2 GPIO_Number_7 // For future use (SW2 in board)
#define HAL_GPIO_LED3 GPIO_Number_34 // used for timer blinking

//SAKI-1 Voltage limit configuration
#define OVER_VOLTAGE_Kv  _IQ(0.060)    //over voltage above 55V
#define UNDER_VOLTAGE_Kv _IQ(0.050)    //under voltage set based on the design

//SAKI-1 Temperature of the motor configuration
#define COPPER_TEMP_COEF_INV_C    (0.00393)
#define ROOM_TEMP_C               (35.0)
#define INV_COPPER_TEMP_COEF_C     (1.0/COPPER_TEMP_COEF_INV_C)
#define INV_RS_AT_ROOM_TEMP_INV_OHMS (1.0/USER_MOTOR_Rs)
#define MOTOR_MAX_TEMP_C          (100.0)    //Motor max temperature

//SAKI-1 Start-up with hall configuration
#define ONE_HALL_ROTATION    (156)   //total hall count for one rotation
#define NO_OF_REVOLUTION     (3)    //number of rotation needed
#define HALL_COUNT           (ONE_HALL_ROTATION * NO_OF_REVOLUTION)   //total hall count

//SAKI-1 Throttle configuration
#define START_BAND                     (1150)    //1063  value for old throttle
#define END_BAND                       (3180)
#define TPS_TOLERENCE_IN_PERCENTAGE    (5)
#define BASE_CURRENT_A                 (12)    //Default (1) ,min current needed to run the motor at no-load
#define NO_OF_DIVISION                 (15)
#define WIDTH_PER_DIVISION             ((END_BAND - START_BAND)/NO_OF_DIVISION)
#define MOTOR_MAX_CURRENT              (40)
#define MOTOR_LOWSPEED_CURRENT         (40)
#define MAX_TIMER_CNT_SEC              (20)

#define STALL_IN_SEC    (5)    //Stall condition check in sec

#define STARTUP


// **************************************************************************
// the globals

CLARKE_Handle   clarkeHandle_I;               //!< the handle for the current Clarke transform
CLARKE_Obj      clarke_I;                     //!< the current Clarke transform object

CLARKE_Handle   clarkeHandle_V;               //!< the handle for the voltage Clarke transform
CLARKE_Obj      clarke_V;                     //!< the voltage Clarke transform object

EST_Handle      estHandle;                    //!< the handle for the estimator

FW_Handle       fwHandle;
FW_Obj          fw;

PID_Obj         pid[4];                       //!< three handles for PID controllers 0 - Speed, 1 - Id, 2 - Iq
PID_Handle      pidHandle[4];                 //!< three objects for PID controllers 0 - Speed, 1 - Id, 2 - Iq
uint16_t        pidCntSpeed;                  //!< count variable to decimate the execution of the speed PID controller

IPARK_Handle    iparkHandle;                  //!< the handle for the inverse Park transform
IPARK_Obj       ipark;                        //!< the inverse Park transform object

FILTER_FO_Handle  filterHandle[6];            //!< the handles for the 3-current and 3-voltage filters for offset calculation
FILTER_FO_Obj     filter[6];                  //!< the 3-current and 3-voltage filters for offset calculation

SVGENCURRENT_Obj     svgencurrent;
SVGENCURRENT_Handle  svgencurrentHandle;

SVGEN_Handle    svgenHandle;                  //!< the handle for the space vector generator
SVGEN_Obj       svgen;                        //!< the space vector generator object

TRAJ_Handle     trajHandle_Id;                //!< the handle for the id reference trajectory
TRAJ_Obj        traj_Id;                      //!< the id reference trajectory object

TRAJ_Handle     trajHandle_spd;               //!< the handle for the speed reference trajectory
TRAJ_Obj        traj_spd;                     //!< the speed reference trajectory object

// define Flying Start (FS) variables
FS_Obj fs;
FS_Handle fsHandle;
CTRL_Handle handle;

#ifdef CSM_ENABLE
#pragma DATA_SECTION(halHandle,"rom_accessed_data");
#endif

HAL_Handle      halHandle;                    //!< the handle for the hardware abstraction layer (HAL)

HAL_PwmData_t   gPwmData = {_IQ(0.0), _IQ(0.0), _IQ(0.0)};       //!< contains the three pwm values -1.0 - 0%, 1.0 - 100%

HAL_AdcData_t   gAdcData;                     //!< contains three current values, three voltage values and one DC buss value

MATH_vec3       gOffsets_I_pu = {_IQ(0.0), _IQ(0.0), _IQ(0.0)};  //!< contains the offsets for the current feedback

MATH_vec3       gOffsets_V_pu = {_IQ(0.0), _IQ(0.0), _IQ(0.0)};  //!< contains the offsets for the voltage feedback

MATH_vec2       gIdq_ref_pu = {_IQ(0.0), _IQ(0.0)};              //!< contains the Id and Iq references

MATH_vec2       gVdq_out_pu = {_IQ(0.0), _IQ(0.0)};              //!< contains the output Vd and Vq from the current controllers

MATH_vec2       gIdq_pu = {_IQ(0.0), _IQ(0.0)};                  //!< contains the Id and Iq measured values

#ifdef CSM_ENABLE
#pragma DATA_SECTION(gUserParams,"rom_accessed_data");
#endif

USER_Params     gUserParams;

volatile MOTOR_Vars_t gMotorVars = MOTOR_Vars_INIT;   //!< the global motor variables that are defined in main.h and used for display in the debugger's watch window

#ifdef FLASH
// Used for running BackGround in flash, and ISR in RAM
extern uint16_t *RamfuncsLoadStart, *RamfuncsLoadEnd, *RamfuncsRunStart;

#ifdef CSM_ENABLE
extern uint16_t *econst_start, *econst_end, *econst_ram_load;
extern uint16_t *switch_start, *switch_end, *switch_ram_load;
#endif
#endif


MATH_vec3 gIavg = {_IQ(0.0), _IQ(0.0), _IQ(0.0)};
uint16_t gIavg_shift = 1;
MATH_vec3 gPwmData_prev = {_IQ(0.0), _IQ(0.0), _IQ(0.0)};

#ifdef DRV8301_SPI
// Watch window interface to the 8301 SPI
DRV_SPI_8301_Vars_t gDrvSpi8301Vars;
#endif

#ifdef DRV8305_SPI
// Watch window interface to the 8305 SPI
DRV_SPI_8305_Vars_t gDrvSpi8305Vars;
#endif

// define CPU time
CPU_TIME_Obj     cpu_time;
CPU_TIME_Handle  cpu_timeHandle;

HAL_DacData_t gDacData;

_iq gFlux_pu_to_Wb_sf;

_iq gFlux_pu_to_VpHz_sf;

_iq gTorque_Ls_Id_Iq_pu_to_Nm_sf;

_iq gTorque_Flux_Iq_pu_to_Nm_sf;

_iq gSpeed_krpm_to_pu_sf = _IQ((float_t)USER_MOTOR_NUM_POLE_PAIRS * 1000.0 / (USER_IQ_FULL_SCALE_FREQ_Hz * 60.0));

_iq gSpeed_hz_to_krpm_sf = _IQ(60.0 / (float_t)USER_MOTOR_NUM_POLE_PAIRS / 1000.0);

_iq gIs_Max_squared_pu = _IQ((USER_MOTOR_MAX_CURRENT * USER_MOTOR_MAX_CURRENT) / (USER_IQ_FULL_SCALE_CURRENT_A * USER_IQ_FULL_SCALE_CURRENT_A));

uint32_t gOffsetCalcCount = 0;

uint32_t gtimer1Cnt = 0;
uint16_t gTrjCnt = 0;

volatile bool gFlag_enableRsOnLine = true;

volatile bool gFlag_updateRs = false;

volatile _iq gRsOnLineFreq_Hz = _IQ(0.2);

volatile _iq gRsOnLineId_mag_A = _IQ(5.0); // SAKI default 0.5A

volatile _iq gRsOnLinePole_Hz = _IQ(0.2);

//variable
_iq speed_pid_out = _IQ(0.0);
_iq angle_est_pu = _IQ(0.0);
_iq speed_est_pu = _IQ(0.0);
_iq angle_pu = _IQ(0.0);
_iq speed_pu = _IQ(0.0);

//BLDC current loop
_iq gHall_BLDC_Is_fdb_pu = _IQ(0.0);
_iq gHall_BLDC_Is_ref_pu = _IQ(0.0);
_iq gHall_PwmDuty = _IQ(0.0);

// Coefficient for pid regulator switch from Hall to Fast
_iq gHall2Fast_Spd_Coef = _IQ(1.0);
_iq gHall2Fast_Iq_coef = _IQ(0.50);
_iq gHall2Fast_Ui_coef = _IQ(0.50);

// Coefficient for pid regulator switch from Fast to Hall
_iq gFast2Hall_Spd_coef = _IQ(1.0);
_iq gFast2Hall_Iq_coef = _IQ(0.0);
_iq gFast2Hall_Ui_coef = _IQ(0.02); //default (2.0)

// BLDC speed
_iq gHall_speed_fdb_pu = _IQ(0.0);
_iq gHall_speed_FastToBldc_low_pu  = _IQ(0.0200*USER_MOTOR_NUM_POLE_PAIRS*1000.0/(USER_IQ_FULL_SCALE_FREQ_Hz * 60.0));		// 400rpm   //30rpm
_iq gHall_speed_BldcToFast_high_pu = _IQ(0.0300*USER_MOTOR_NUM_POLE_PAIRS*1000.0/(USER_IQ_FULL_SCALE_FREQ_Hz * 60.0));		// 750rpm   //50rpm

uint16_t gHall_PwmState = 0;

uint16_t gHall_GpioData = 0;
int16_t gHall_State = 0;
int16_t gHall_PrevState = 0;
int16_t gHall_LastState = 0;
int16_t gHall_State_delta = 0;

uint16_t gHall_dir = 0;
uint16_t gHall_dir_prev = 0;
uint16_t gHall_dir_change = 0;

uint16_t gHall_BLDC_Flag_Is_fdb = 0;
//uint16_t gHall_PwmIndex[8] = {4, 1, 5, 6, 3, 2, 4, 1}; //TI
uint16_t gHall_PwmIndex[8] = {1, 4, 2, 3, 6, 5, 1, 4}; //SAKI-1 // for forward direction

bool gHall_Flag_EnableBldc = true;
//bool gHall_Flag_EnableBldc = false;
bool gHall_Flag_EnableStartup = true;		// true->enable hall startup
//bool gHall_Flag_EnableStartup = false;	// false->disable hall startup
bool gHall_Flag_CurrentCtrl = true;
bool gHall_Flag_State_Change = false;

uint32_t gHall_timer_now= 0;
uint32_t gHall_timer_prev = 0;
uint32_t gHall_time_delta_now = 0;
uint32_t gHall_time_delta_prev = 0;
uint32_t gHall_time_delta = 0;

uint32_t gHall_speed_scale = USER_SYSTEM_FREQ_MHz*1000000*100/6;
uint32_t gHall_speed_fdb_0p01Hz = 0;
_iq gHall_Speed_0p01hz_to_pu_sf = _IQ((float_t)0.01/(USER_IQ_FULL_SCALE_FREQ_Hz));

uint16_t gHall_PwmCnt = 0;
uint16_t gHall_PwmCntMax = 10000;

uint32_t gHall_Bldc_Cnt = 0;
uint32_t gHall_Fast_Cnt = 0;


void HALLBLDC_Ctrl_Run(void);
void HALLBLDC_State_Check(void);
void HALLBLDC_Ctrl_Stop(void);
void HALLBLDC_Ctrl_PwmSet(uint16_t PwmState, _iq PwmDuty);

//SAKI-1
uint16_t gLEDcnt   =  0;
uint16_t gHallCount = 0;
uint16_t gstallCount = 0;
uint16_t gReadyCount = 0;

_iq gTps           = _IQ(0.0);
_iq togglingPt     = _IQ(0.0);
bool gStartFlag     = false;
_iq gHall_krpm     = _IQ(0.0);


float_t gMotorTemperature_C = 0.0;

bool gEngineStartSW = false;    //Engine start button
bool gEngineStopSW = false;
bool gEngineStatusError = false;
bool EngineReady = false;
bool gstop = false;
bool gFlag_Motor_Run = false;
_iq gValue = _IQ(0.0);
uint16_t gTimerCnt = 0;
bool gflag = false;
bool gInBLDC = true;

bool EnablePeakCurrentMode = true;
bool EnablePeakCurrent = true;




// **************************************************************************
// the functions
void main(void)

{
    // IMPORTANT NOTE: If you are not familiar with MotorWare coding guidelines
    // please refer to the following document:
    // C:/ti/motorware/motorware_1_01_00_1x/docs/motorware_coding_standards.pdf

    // Only used if running from FLASH
    // Note that the variable FLASH is defined by the project

#ifdef FLASH
    // Copy time critical code and Flash setup code to RAM
    // The RamfuncsLoadStart, RamfuncsLoadEnd, and RamfuncsRunStart
    // symbols are created by the linker. Refer to the linker files.
    memCopy((uint16_t *)&RamfuncsLoadStart,(uint16_t *)&RamfuncsLoadEnd,
            (uint16_t *)&RamfuncsRunStart);

#ifdef CSM_ENABLE
    //copy .econst to unsecure RAM
    if(*econst_end - *econst_start)
    {
        memCopy((uint16_t *)&econst_start,(uint16_t *)&econst_end,(uint16_t *)&econst_ram_load);
    }

    //copy .switch ot unsecure RAM
    if(*switch_end - *switch_start)
    {
        memCopy((uint16_t *)&switch_start,(uint16_t *)&switch_end,(uint16_t *)&switch_ram_load);
    }
#endif
#endif

    // initialize the Hardware Abstraction Layer  (HAL)
    // halHandle will be used throughout the code to interface with the HAL
    // (set parameters, get and set functions, etc) halHandle is required since
    // this is how all objects are interfaced, and it allows interface with
    // multiple objects by simply passing a different handle. The use of
    // handles is explained in this document:
    // C:/ti/motorware/motorware_1_01_00_1x/docs/motorware_coding_standards.pdf
    halHandle = HAL_init(&hal,sizeof(hal));

    // check for errors in user parameters
    USER_checkForErrors(&gUserParams);

    // store user parameter error in global variable
    gMotorVars.UserErrorCode = USER_getErrorCode(&gUserParams);

    // do not allow code execution if there is a user parameter error
    if(gMotorVars.UserErrorCode != USER_ErrorCode_NoError)
    {
        for(;;)
        {
            gMotorVars.Flag_enableSys = false;
        }
    }

    // initialize the Clarke modules
    clarkeHandle_I = CLARKE_init(&clarke_I,sizeof(clarke_I));
    clarkeHandle_V = CLARKE_init(&clarke_V,sizeof(clarke_V));

    // initialize the estimator
    estHandle = EST_init((void *)USER_EST_HANDLE_ADDRESS, 0x200);

    // initialize the user parameters
    USER_setParams(&gUserParams);

    // set the hardware abstraction layer parameters
    HAL_setParams(halHandle,&gUserParams);

#ifdef FAST_ROM_V1p6
    {
        CTRL_Handle ctrlHandle = CTRL_init((void *)USER_CTRL_HANDLE_ADDRESS, 0x200);
        CTRL_Obj *obj = (CTRL_Obj *)ctrlHandle;
        obj->estHandle = estHandle;

        // initialize the estimator through the controller
        CTRL_setParams(ctrlHandle,&gUserParams);
        CTRL_setUserMotorParams(ctrlHandle);
        CTRL_setupEstIdleState(ctrlHandle);
    }
#else
    {
        // initialize the estimator
        EST_setEstParams(estHandle,&gUserParams);
        EST_setupEstIdleState(estHandle);
    }
#endif

    // disable Rs recalculation by default
    gMotorVars.Flag_enableRsRecalc = false;
    EST_setFlag_enableRsRecalc(estHandle,false);

    // configure RsOnLine
    EST_setFlag_enableRsOnLine(estHandle,gFlag_enableRsOnLine);
    EST_setFlag_updateRs(estHandle,gFlag_updateRs);
    EST_setRsOnLineAngleDelta_pu(estHandle,_IQmpy(gRsOnLineFreq_Hz, _IQ(1.0/USER_ISR_FREQ_Hz)));
    EST_setRsOnLineId_mag_pu(estHandle,_IQmpy(gRsOnLineId_mag_A, _IQ(1.0/USER_IQ_FULL_SCALE_CURRENT_A)));

    // Calculate coefficients for all filters
    {
        _iq b0 = _IQmpy(gRsOnLinePole_Hz, _IQ(1.0/USER_ISR_FREQ_Hz));
        _iq a1 = b0 - _IQ(1.0);
        EST_setRsOnLineFilterParams(estHandle,EST_RsOnLineFilterType_Current,b0,a1,_IQ(0.0),b0,a1,_IQ(0.0));
        EST_setRsOnLineFilterParams(estHandle,EST_RsOnLineFilterType_Voltage,b0,a1,_IQ(0.0),b0,a1,_IQ(0.0));
    }

    // set the number of current sensors
    setupClarke_I(clarkeHandle_I,USER_NUM_CURRENT_SENSORS);

    // set the number of voltage sensors
    setupClarke_V(clarkeHandle_V,USER_NUM_VOLTAGE_SENSORS);

    // set the pre-determined current and voltage feeback offset values
    gOffsets_I_pu.value[0] = _IQ(I_A_offset);
    gOffsets_I_pu.value[1] = _IQ(I_B_offset);
    gOffsets_I_pu.value[2] = _IQ(I_C_offset);
    gOffsets_V_pu.value[0] = _IQ(V_A_offset);
    gOffsets_V_pu.value[1] = _IQ(V_B_offset);
    gOffsets_V_pu.value[2] = _IQ(V_C_offset);

    // initialize the PID controllers
    {
        _iq maxCurrent_pu = _IQ(USER_MOTOR_MAX_CURRENT / USER_IQ_FULL_SCALE_CURRENT_A);
        _iq maxVoltage_pu = _IQ(USER_MAX_VS_MAG_PU * USER_VD_SF);
        float_t fullScaleCurrent = USER_IQ_FULL_SCALE_CURRENT_A;
        float_t fullScaleVoltage = USER_IQ_FULL_SCALE_VOLTAGE_V;
        float_t IsrPeriod_sec = 1.0 / USER_ISR_FREQ_Hz;
        float_t Ls_d = USER_MOTOR_Ls_d;
        float_t Ls_q = USER_MOTOR_Ls_q;
        float_t Rs = USER_MOTOR_Rs;
        float_t RoverLs_d = Rs/Ls_d;
        float_t RoverLs_q = Rs/Ls_q;
        _iq Kp_Id = _IQ((0.314*Ls_d*fullScaleCurrent)/(IsrPeriod_sec*fullScaleVoltage));    //SAK1 default value is 0.25
        _iq Ki_Id = _IQ(RoverLs_d*IsrPeriod_sec);
        _iq Kp_Iq = _IQ((0.314*Ls_q*fullScaleCurrent)/(IsrPeriod_sec*fullScaleVoltage));    //SAK1 default value is 0.25
        _iq Ki_Iq = _IQ(RoverLs_q*IsrPeriod_sec);
        gMotorVars.Kp_Idq = Kp_Id;
        gMotorVars.Ki_Idq = Ki_Id;

        pidHandle[0] = PID_init(&pid[0],sizeof(pid[0]));
        pidHandle[1] = PID_init(&pid[1],sizeof(pid[1]));
        pidHandle[2] = PID_init(&pid[2],sizeof(pid[2]));
        pidHandle[3] = PID_init(&pid[3],sizeof(pid[3]));

        PID_setGains(pidHandle[0],_IQ(0.00002),_IQ(0.005),_IQ(0.0));    //SAKI-1 default value kp=0.2 ki=0.005
        PID_setMinMax(pidHandle[0],-maxCurrent_pu,maxCurrent_pu);
        PID_setUi(pidHandle[0],_IQ(0.0));
        pidCntSpeed = 0;

        PID_setGains(pidHandle[1],Kp_Id,Ki_Id,_IQ(0.0));
        PID_setMinMax(pidHandle[1],-maxVoltage_pu,maxVoltage_pu);
        PID_setUi(pidHandle[1],_IQ(0.0));

        PID_setGains(pidHandle[2],Kp_Iq,Ki_Iq,_IQ(0.0));
        PID_setMinMax(pidHandle[2],_IQ(0.0),_IQ(0.0));
        PID_setUi(pidHandle[2],_IQ(0.0));

        PID_setGains(pidHandle[3],Kp_Iq,Ki_Iq,_IQ(0.0));
        PID_setMinMax(pidHandle[3],-maxVoltage_pu,maxVoltage_pu);    //SAKI-1 default PID_setMinMax(pidHandle[3],_IQ(0.0),_IQ(0.0));
        PID_setUi(pidHandle[3],_IQ(0.0));
    }

    // initialize the speed reference in kilo RPM where base speed is USER_IQ_FULL_SCALE_FREQ_Hz
    gMotorVars.SpeedRef_krpm = _IQmpy(_IQ(5.0), gSpeed_hz_to_krpm_sf);

    // initialize the inverse Park module
    iparkHandle = IPARK_init(&ipark,sizeof(ipark));

    // initialize and configure offsets using filters
    {
        uint16_t cnt = 0;
        _iq b0 = _IQ(gUserParams.offsetPole_rps/(float_t)gUserParams.ctrlFreq_Hz);
        _iq a1 = (b0 - _IQ(1.0));
        _iq b1 = _IQ(0.0);

        for(cnt=0;cnt<6;cnt++)
        {
            filterHandle[cnt] = FILTER_FO_init(&filter[cnt],sizeof(filter[0]));
            FILTER_FO_setDenCoeffs(filterHandle[cnt],a1);
            FILTER_FO_setNumCoeffs(filterHandle[cnt],b0,b1);
            FILTER_FO_setInitialConditions(filterHandle[cnt],_IQ(0.0),_IQ(0.0));
        }

        gMotorVars.Flag_enableOffsetcalc = false;
    }

    // initialize the space vector generator module
    svgenHandle = SVGEN_init(&svgen,sizeof(svgen));

    // Initialize and setup the 100% SVM generator
    svgencurrentHandle = SVGENCURRENT_init(&svgencurrent,sizeof(svgencurrent));

    // setup svgen current
    {
        float_t minWidth_microseconds = 2.0;
        uint16_t minWidth_counts = (uint16_t)(minWidth_microseconds * USER_SYSTEM_FREQ_MHz);
        float_t fdutyLimit = 0.5-(2.0*minWidth_microseconds*USER_PWM_FREQ_kHz*0.001);
        _iq dutyLimit = _IQ(fdutyLimit);

        SVGENCURRENT_setMinWidth(svgencurrentHandle, minWidth_counts);
        SVGENCURRENT_setIgnoreShunt(svgencurrentHandle, use_all);
        SVGENCURRENT_setMode(svgencurrentHandle,all_phase_measurable);
        SVGENCURRENT_setVlimit(svgencurrentHandle,dutyLimit);
    }

    // initialize the speed reference trajectory
    trajHandle_spd = TRAJ_init(&traj_spd,sizeof(traj_spd));

    // configure the speed reference trajectory
    TRAJ_setTargetValue(trajHandle_spd,_IQ(0.0));
    TRAJ_setIntValue(trajHandle_spd,_IQ(0.0));
    TRAJ_setMinValue(trajHandle_spd,_IQ(-1.0));
    TRAJ_setMaxValue(trajHandle_spd,_IQ(1.0));
    TRAJ_setMaxDelta(trajHandle_spd,_IQ(USER_MAX_ACCEL_Hzps / USER_IQ_FULL_SCALE_FREQ_Hz / USER_ISR_FREQ_Hz));

    // initialize the Id reference trajectory
    trajHandle_Id = TRAJ_init(&traj_Id,sizeof(traj_Id));

    if(USER_MOTOR_TYPE == MOTOR_Type_Pm)
    {
        // configure the Id reference trajectory
        TRAJ_setTargetValue(trajHandle_Id,_IQ(0.0));
        TRAJ_setIntValue(trajHandle_Id,_IQ(0.0));
        TRAJ_setMinValue(trajHandle_Id,_IQ(-USER_MOTOR_MAX_CURRENT / USER_IQ_FULL_SCALE_CURRENT_A));
        TRAJ_setMaxValue(trajHandle_Id,_IQ(USER_MOTOR_MAX_CURRENT / USER_IQ_FULL_SCALE_CURRENT_A));
        TRAJ_setMaxDelta(trajHandle_Id,_IQ(USER_MOTOR_RES_EST_CURRENT / USER_IQ_FULL_SCALE_CURRENT_A / USER_ISR_FREQ_Hz));

        // Initialize field weakening
        fwHandle = FW_init(&fw,sizeof(fw));
        FW_setFlag_enableFw(fwHandle, false); // Disable field weakening
        FW_clearCounter(fwHandle); // Clear field weakening counter
        FW_setNumIsrTicksPerFwTick(fwHandle, FW_NUM_ISR_TICKS_PER_CTRL_TICK); // Set the number of ISR per field weakening ticks
        FW_setDeltas(fwHandle, FW_INC_DELTA, FW_DEC_DELTA); // Set the deltas of field weakening
        FW_setOutput(fwHandle, _IQ(0.0)); // Set initial output of field weakening to zero
        FW_setMinMax(fwHandle,_IQ(USER_MAX_NEGATIVE_ID_REF_CURRENT_A/USER_IQ_FULL_SCALE_CURRENT_A),_IQ(0.0)); // Set the field weakening controller limits
    }
    else
    {
        // configure the Id reference trajectory
        TRAJ_setTargetValue(trajHandle_Id,_IQ(0.0));
        TRAJ_setIntValue(trajHandle_Id,_IQ(0.0));
        TRAJ_setMinValue(trajHandle_Id,_IQ(0.0));
        TRAJ_setMaxValue(trajHandle_Id,_IQ(USER_MOTOR_MAGNETIZING_CURRENT / USER_IQ_FULL_SCALE_CURRENT_A));
        TRAJ_setMaxDelta(trajHandle_Id,_IQ(USER_MOTOR_MAGNETIZING_CURRENT / USER_IQ_FULL_SCALE_CURRENT_A / USER_ISR_FREQ_Hz));
    }

    // Initialize Flying Start (FS)
    fsHandle = FS_init(&fs,sizeof(fs));

    // Disable Flying Start (FS)
    FS_setFlag_enableFs(fsHandle, false);

    // Clear Flying Start(FS) check time count
    FS_clearCntCheckTime(fsHandle);

    // Set Flying Start(FS) minimum transition speed
    //FS_setSpeedFsMin_krpm(fsHandle, ctrlHandle, FS_SPEED_MIN);

    FS_setSpeedFsMin(fsHandle,FS_SPEED_MIN); //SAKI-1

    // set Flying Start(FS) maximum check time
    FS_setMaxCheckTime(fsHandle, FS_MAX_CHECK_TIME);

    gMotorVars.Flag_enableFlyingStart = true;       // enable Flying Start

    // initialize the CPU usage module
    cpu_timeHandle = CPU_TIME_init(&cpu_time,sizeof(cpu_time));
    CPU_TIME_setParams(cpu_timeHandle, (PWM_getPeriod(halHandle->pwmHandle[0])*USER_NUM_PWM_TICKS_PER_ISR_TICK));

    // setup faults
    HAL_setupFaults(halHandle);

    // initialize the interrupt vector table
    HAL_initIntVectorTable(halHandle);

    // enable the ADC interrupts
    HAL_enableAdcInts(halHandle);

    // enable global interrupts
    HAL_enableGlobalInts(halHandle);

    // enable debug interrupts
    HAL_enableDebugInt(halHandle);

    // disable the PWM
    HAL_disablePwm(halHandle);

    // compute scaling factors for flux and torque calculations
    gFlux_pu_to_Wb_sf = USER_computeFlux_pu_to_Wb_sf();
    gFlux_pu_to_VpHz_sf = USER_computeFlux_pu_to_VpHz_sf();
    gTorque_Ls_Id_Iq_pu_to_Nm_sf = USER_computeTorque_Ls_Id_Iq_pu_to_Nm_sf();
    gTorque_Flux_Iq_pu_to_Nm_sf = USER_computeTorque_Flux_Iq_pu_to_Nm_sf();

#ifdef DRV8301_SPI
    // turn on the DRV8301 if present
    HAL_enableDrv(halHandle);
    // initialize the DRV8301 interface
    HAL_setupDrvSpi(halHandle,&gDrvSpi8301Vars);
#endif

#ifdef DRV8305_SPI
    // turn on the DRV8305 if present
    HAL_enableDrv(halHandle);
    // initialize the DRV8305 interface
    HAL_setupDrvSpi(halHandle,&gDrvSpi8305Vars);
#endif

    //gMotorVars.Flag_enableOffsetcalc = true;
    gMotorVars.Flag_enableOffsetcalc = false;    //SAKI-1
    gMotorVars.Flag_enableSpeedCtrl = false;    //SAKI-1
    HAL_setGpioLow(halHandle,(GPIO_Number_e)GPIO_Number_51);   //SAKI-1 put drv in sleep mode

#ifdef TIMER0
    HAL_enableTimer0Int(halHandle);    //SAKI-1 timer
#endif

    // while(gMotorVars.Flag_enableOffsetcalc == true) gMotorVars.Flag_Run_Identify = 0;

    // Below two lines code for Flash Testing, need to be commented
    //  gMotorVars.Flag_enableSys = true;
    //  gMotorVars.Flag_Run_Identify = true


    // Begin the background loop
    for(;;)
    {
        //SAKI-1
        while(!(getStartSwitch()));    // default while(!(gMotorVars.Flag_enableSys));

        if (gStartFlag != true)        //execute for one time
        {
            disableLpmDrv8301();

            gMotorVars.Flag_enableSys = true;

            HAL_setGpioHigh(halHandle,(GPIO_Number_e)GPIO_Number_15); //LED-11 On

            gStartFlag = true;
        }

        // loop while the enable system flag is true
        while(gMotorVars.Flag_enableSys)
        {

            //SAKI-1
            gEngineStatusError = ENGINE_checkForErrors();

            //SAKI-1
            //ENGINE_IgnitionSwitch();

            // Control motor Start or Stop with Flying Start
            motor_RunCtrl();

            //SAKI-1
            ENGINE_ReadyToRun();



            if(gMotorVars.Flag_Run_Identify)
            {
                //SAKI-1
                if (computeOverLoad())gMotorVars.Flag_Run_Identify = false;

                // disable Rs recalculation
                EST_setFlag_enableRsRecalc(estHandle,false);

                // update estimator state
                EST_updateState(estHandle,0);

#ifdef FAST_ROM_V1p6
                // call this function to fix 1p6
                softwareUpdate1p6(estHandle);
#endif

                // enable the PWM
                // HAL_enablePwm(halHandle);

                if(gHall_Flag_EnableStartup == false)
                {
                    // enable the PWM
                    HAL_enablePwm(halHandle);
                }

                // set trajectory target for speed reference
                TRAJ_setTargetValue(trajHandle_spd,_IQmpy(gMotorVars.SpeedRef_krpm, gSpeed_krpm_to_pu_sf));

                // SAKI-1
                ENGINE_StartUp();
              //new_startup();


                // Enable RsOnLine flags
                gFlag_enableRsOnLine = true;
                gFlag_updateRs = false;


                if(USER_MOTOR_TYPE == MOTOR_Type_Pm)
                {
                    // set trajectory target for Id reference
                    TRAJ_setTargetValue(trajHandle_Id,gIdq_ref_pu.value[0]);
                }
                else
                {
                    if(gMotorVars.Flag_enablePowerWarp)
                    {
                        _iq Id_target_pw_pu = EST_runPowerWarp(estHandle,TRAJ_getIntValue(trajHandle_Id),gIdq_pu.value[1]);
                        TRAJ_setTargetValue(trajHandle_Id,Id_target_pw_pu);
                        TRAJ_setMinValue(trajHandle_Id,_IQ(USER_MOTOR_MAGNETIZING_CURRENT * 0.3 / USER_IQ_FULL_SCALE_CURRENT_A));
                        TRAJ_setMaxDelta(trajHandle_Id,_IQ(USER_MOTOR_MAGNETIZING_CURRENT * 0.3 / USER_IQ_FULL_SCALE_CURRENT_A / USER_ISR_FREQ_Hz));
                    }
                    else
                    {
                        // set trajectory target for Id reference
                        TRAJ_setTargetValue(trajHandle_Id,_IQ(USER_MOTOR_MAGNETIZING_CURRENT / USER_IQ_FULL_SCALE_CURRENT_A));
                        TRAJ_setMinValue(trajHandle_Id,_IQ(0.0));
                        TRAJ_setMaxDelta(trajHandle_Id,_IQ(USER_MOTOR_MAGNETIZING_CURRENT / USER_IQ_FULL_SCALE_CURRENT_A / USER_ISR_FREQ_Hz));
                    }
                }

            }
            else if(gMotorVars.Flag_enableRsRecalc)
            {
                // set angle to zero
                EST_setAngle_pu(estHandle,_IQ(0.0));

                // enable or disable Rs recalculation
                EST_setFlag_enableRsRecalc(estHandle,true);

                // update estimator state
                EST_updateState(estHandle,0);

#ifdef FAST_ROM_V1p6
                // call this function to fix 1p6
                softwareUpdate1p6(estHandle);
#endif

                // enable the PWM
                HAL_enablePwm(halHandle);

                // set trajectory target for speed reference
                TRAJ_setTargetValue(trajHandle_spd,_IQ(0.0));

                // set trajectory target for Id reference
                TRAJ_setTargetValue(trajHandle_Id,_IQ(USER_MOTOR_RES_EST_CURRENT / USER_IQ_FULL_SCALE_CURRENT_A));

                // if done with Rs recalculation, disable flag
                if(EST_getState(estHandle) == EST_State_OnLine) gMotorVars.Flag_enableRsRecalc = false;
            }
            else
            {
                // set estimator to Idle
                EST_setIdle(estHandle);

                // disable the PWM
                if(!gMotorVars.Flag_enableOffsetcalc) HAL_disablePwm(halHandle);

                // clear the speed reference trajectory
                TRAJ_setTargetValue(trajHandle_spd,_IQ(0.0));
                TRAJ_setIntValue(trajHandle_spd,_IQ(0.0));

                // clear the Id reference trajectory
                TRAJ_setTargetValue(trajHandle_Id,_IQ(0.0));
                TRAJ_setIntValue(trajHandle_Id,_IQ(0.0));

                // configure trajectory Id defaults depending on motor type
                if(USER_MOTOR_TYPE == MOTOR_Type_Pm)
                {
                    TRAJ_setMinValue(trajHandle_Id,_IQ(-USER_MOTOR_MAX_CURRENT / USER_IQ_FULL_SCALE_CURRENT_A));
                    TRAJ_setMaxDelta(trajHandle_Id,_IQ(USER_MOTOR_RES_EST_CURRENT / USER_IQ_FULL_SCALE_CURRENT_A / USER_ISR_FREQ_Hz));
                }
                else
                {
                    TRAJ_setMinValue(trajHandle_Id,_IQ(0.0));
                    TRAJ_setMaxDelta(trajHandle_Id,_IQ(USER_MOTOR_MAGNETIZING_CURRENT / USER_IQ_FULL_SCALE_CURRENT_A / USER_ISR_FREQ_Hz));
                }

                // clear integral outputs
                PID_setUi(pidHandle[0],_IQ(0.0));
                PID_setUi(pidHandle[1],_IQ(0.0));
                PID_setUi(pidHandle[2],_IQ(0.0));

                // clear Id and Iq references
                gIdq_ref_pu.value[0] = _IQ(0.0);
                gIdq_ref_pu.value[1] = _IQ(0.0);

                //SAKI-1
                gMotorVars.IqRef_A = _IQ(0.0);
                gHallCount = 0;    //reset the hall count
                gHall_krpm = 0;

                // disable RsOnLine flags
                gFlag_enableRsOnLine = false;
                gFlag_updateRs = false;

                // disable PowerWarp flag
                gMotorVars.Flag_enablePowerWarp = false;
            }

            // update the global variables
            updateGlobalVariables(estHandle);

            // set field weakening enable flag depending on user's input
            FW_setFlag_enableFw(fwHandle,gMotorVars.Flag_enableFieldWeakening);

            // set the speed acceleration
            TRAJ_setMaxDelta(trajHandle_spd,_IQmpy(MAX_ACCEL_KRPMPS_SF,gMotorVars.MaxAccel_krpmps));

            // enable/disable the forced angle
            EST_setFlag_enableForceAngle(estHandle,gMotorVars.Flag_enableForceAngle);

            // enable or disable RsOnLine
            EST_setFlag_enableRsOnLine(estHandle,gFlag_enableRsOnLine);

            // set slow rotating frequency for RsOnLine
            EST_setRsOnLineAngleDelta_pu(estHandle,_IQmpy(gRsOnLineFreq_Hz, _IQ(1.0/USER_ISR_FREQ_Hz)));

            // set current amplitude for RsOnLine
            EST_setRsOnLineId_mag_pu(estHandle,_IQmpy(gRsOnLineId_mag_A, _IQ(1.0/USER_IQ_FULL_SCALE_CURRENT_A)));

            // set flag that updates Rs from RsOnLine value
            EST_setFlag_updateRs(estHandle,gFlag_updateRs);

            // clear Id for RsOnLine if disabled
            if(!gFlag_enableRsOnLine) EST_setRsOnLineId_pu(estHandle,_IQ(0.0));

#ifdef DRV8301_SPI
            HAL_writeDrvData(halHandle,&gDrvSpi8301Vars);

            HAL_readDrvData(halHandle,&gDrvSpi8301Vars);
#endif
#ifdef DRV8305_SPI
            HAL_writeDrvData(halHandle,&gDrvSpi8305Vars);

            HAL_readDrvData(halHandle,&gDrvSpi8305Vars);
#endif


            //SAKI-1 Emergency stop
            if(getStopSwitch())
            {
                gstop = getStopSwitch();

                enableLpmDrv8301();

                gMotorVars.Flag_Run_Identify = false;

                gMotorVars.Flag_enableSys = false;
                HAL_setGpioLow(halHandle,(GPIO_Number_e)GPIO_Number_22); //tune off STATUS LED

            }

        } // end of while(gFlag_enableSys) loop


        // disable the PWM
        HAL_disablePwm(halHandle);

        gMotorVars.Flag_Run_Identify = false;

    } // end of for(;;) loop
} // end of main() function


//! \brief     The main ISR that implements the motor control.
interrupt void mainISR(void)
{
    // read the timer 1 value and update the CPU usage module
    uint32_t timer1Cnt = HAL_readTimerCnt(halHandle,2);
    CPU_TIME_updateCnts(cpu_timeHandle,timer1Cnt);

    if(++gLEDcnt >= (uint_least32_t)(USER_ISR_FREQ_Hz / LED_BLINK_FREQ_Hz))
    {
        HAL_toggleLed(halHandle,(GPIO_Number_e)HAL_Gpio_LED2);
        gLEDcnt = 0;

        //SAKI-1
        if(gstop)HAL_toggleLed(halHandle,(GPIO_Number_e)GPIO_Number_15); //LED-11 toggle
    }

    _iq oneOverDcBus;
    MATH_vec2 Iab_pu;
    MATH_vec2 Vab_pu;
    MATH_vec2 phasor;

    // acknowledge the ADC interrupt
    HAL_acqAdcInt(halHandle,ADC_IntNumber_1);

    // convert the ADC data
    HAL_readAdcDataWithOffsets(halHandle,&gAdcData);


    // remove offsets
    gAdcData.I.value[0] = gAdcData.I.value[0] - gOffsets_I_pu.value[0];
    gAdcData.I.value[1] = gAdcData.I.value[1] - gOffsets_I_pu.value[1];
    gAdcData.I.value[2] = gAdcData.I.value[2] - gOffsets_I_pu.value[2];
    gAdcData.V.value[0] = gAdcData.V.value[0] - gOffsets_V_pu.value[0];
    gAdcData.V.value[1] = gAdcData.V.value[1] - gOffsets_V_pu.value[1];
    gAdcData.V.value[2] = gAdcData.V.value[2] - gOffsets_V_pu.value[2];

    // run the current reconstruction algorithm
    runCurrentReconstruction();

    // SAKI-1 run the flying start
    FS_run(handle,fsHandle);

    // run Hall state check for startup and running
    HALLBLDC_State_Check();

    // run Clarke transform on current
    CLARKE_run(clarkeHandle_I,&gAdcData.I,&Iab_pu);

    // run Clarke transform on voltage
    CLARKE_run(clarkeHandle_V,&gAdcData.V,&Vab_pu);

    // run a trajectory for Id reference, so the reference changes with a ramp instead of a step
    TRAJ_run(trajHandle_Id);

    // run the estimator
    EST_run(estHandle,
            &Iab_pu,
            &Vab_pu,
            gAdcData.dcBus,
            TRAJ_getIntValue(trajHandle_spd));

    // generate the motor electrical angle
    angle_est_pu = EST_getAngle_pu(estHandle);
    speed_est_pu = EST_getFm_pu(estHandle);

    // get Idq from estimator to avoid sin and cos
    EST_getIdq_pu(estHandle,&gIdq_pu);

    // run the appropriate controller
    if((gMotorVars.Flag_Run_Identify) || (gMotorVars.Flag_enableRsRecalc))
    {
        _iq refValue;
        _iq fbackValue;
        _iq outMax_pu;


        if (gMotorVars.Flag_enableSpeedCtrl == true)    //SAKI-1 Speed loop
        {
            // when appropriate, run the PID speed controller
            if(pidCntSpeed++ >= USER_NUM_CTRL_TICKS_PER_SPEED_TICK)
            {
                // calculate Id reference squared
                _iq Id_ref_squared_pu = _IQmpy(PID_getRefValue(pidHandle[1]),PID_getRefValue(pidHandle[1]));

                // Take into consideration that Iq^2+Id^2 = Is^2
                _iq Iq_Max_pu = _IQsqrt(gIs_Max_squared_pu - Id_ref_squared_pu);

                // clear counter
                pidCntSpeed = 0;

                // Set new min and max for the speed controller output
                PID_setMinMax(pidHandle[0], -Iq_Max_pu, Iq_Max_pu);

                // run speed controller
                PID_run_spd(pidHandle[0],TRAJ_getIntValue(trajHandle_spd),speed_pu, &speed_pid_out);

                gIdq_ref_pu.value[1] = speed_pid_out;
            }
        }
        else    // SAKI-1 Torque mode
        {
            gIdq_ref_pu.value[1] = _IQmpy(gMotorVars.IqRef_A,_IQ(1.0/USER_IQ_FULL_SCALE_CURRENT_A));
        }

        // get the reference value from the trajectory module
        refValue = TRAJ_getIntValue(trajHandle_Id) + EST_getRsOnLineId_pu(estHandle);

        // get the feedback value
        fbackValue = gIdq_pu.value[0];

        // run the Id PID controller
        PID_run(pidHandle[1],refValue,fbackValue,&(gVdq_out_pu.value[0]));

        // set Iq reference to zero when doing Rs recalculation
        if(gMotorVars.Flag_enableRsRecalc) gIdq_ref_pu.value[1] = _IQ(0.0);

        // get the Iq reference value
        refValue = gIdq_ref_pu.value[1];

        // get the feedback value
        fbackValue = gIdq_pu.value[1];

        // calculate Iq controller limits, and run Iq controller
        //SAKI-1 outMax_pu value will decide the max speed a motor can reach
        _iq max_vs = _IQmpy(_IQ(USER_MAX_VS_MAG_PU),EST_getDcBus_pu(estHandle));
        outMax_pu = _IQsqrt(_IQmpy(max_vs,max_vs) - _IQmpy(gVdq_out_pu.value[0],gVdq_out_pu.value[0]));
        PID_setMinMax(pidHandle[2],-outMax_pu,outMax_pu);
        PID_run(pidHandle[2],refValue,fbackValue,&(gVdq_out_pu.value[1]));


        // compensate angle for PWM delay
        angle_est_pu = angleDelayComp((speed_est_pu), angle_est_pu);

        // compute the sin/cos phasor
        phasor.value[0] = _IQcosPU(angle_pu);
        phasor.value[1] = _IQsinPU(angle_pu);

        // set the phasor in the inverse Park transform
        IPARK_setPhasor(iparkHandle,&phasor);

        // run the inverse Park module
        IPARK_run(iparkHandle,&gVdq_out_pu,&Vab_pu);

        // run the space Vector Generator (SVGEN) module
        oneOverDcBus = EST_getOneOverDcBus_pu(estHandle);
        Vab_pu.value[0] = _IQmpy(Vab_pu.value[0],oneOverDcBus);
        Vab_pu.value[1] = _IQmpy(Vab_pu.value[1],oneOverDcBus);

        SVGEN_run(svgenHandle,&Vab_pu,&(gPwmData.Tabc));

        // run the PWM compensation and current ignore algorithm
        SVGENCURRENT_compPwmData(svgencurrentHandle,&(gPwmData.Tabc),&gPwmData_prev);

        // Run BLDC
        HALLBLDC_Ctrl_Run();

        // increase traj count
        gTrjCnt++;

    }
    else if(gMotorVars.Flag_enableOffsetcalc == true)
    {
        runOffsetsCalculation();
    }
    else
    {
        // disable the PWM
        HAL_disablePwm(halHandle);

        // Set the PWMs to 50% duty cycle
        gPwmData.Tabc.value[0] = _IQ(0.0);
        gPwmData.Tabc.value[1] = _IQ(0.0);
        gPwmData.Tabc.value[2] = _IQ(0.0);

        HALLBLDC_Ctrl_Stop();
    }

    // write the PWM compare values
    HAL_writePwmData(halHandle,&gPwmData);

    if(gTrjCnt >= gUserParams.numCtrlTicksPerTrajTick)
    {
        // clear counter
        gTrjCnt = 0;

        // run a trajectory for speed reference, so the reference changes with a ramp instead of a step
        TRAJ_run(trajHandle_spd);
    }

    // run function to set next trigger
    if(!gMotorVars.Flag_enableRsRecalc) runSetTrigger();

    // run field weakening
    if(USER_MOTOR_TYPE == MOTOR_Type_Pm) runFieldWeakening();

    gMotorVars.angle_est_pu = angle_pu;
    gMotorVars.speed_est_pu = speed_est_pu;

#ifndef F2802xF
    // connect inputs of the PWMDAC module.
    gDacData.value[0] = gAdcData.I.value[0];
    gDacData.value[1] = gIdq_ref_pu.value[0];
    gDacData.value[2] = gMotorVars.speed_est_pu;
    gDacData.value[3] = gMotorVars.angle_est_pu;

    // run PwmDAC
    HAL_writeDacData(halHandle,&gDacData);

#endif

    // read the timer 2 value and update the CPU usage module
    timer1Cnt = HAL_readTimerCnt(halHandle,2);
    CPU_TIME_run(cpu_timeHandle,timer1Cnt);
    gtimer1Cnt = HAL_readTimerCnt(halHandle,2);


    return;
} // end of mainISR() function

#ifdef TIMER0
interrupt void timer0ISR(void)
{

    //ack the timer interrupt

    //HAL_acqTimer0Int(halHandle);
    gTimerCnt++;


    //toggle status LED
    HAL_toggleLed(halHandle,HAL_GPIO_LED3);

    return;
}
#endif

_iq angleDelayComp(const _iq fm_pu, const _iq angleUncomp_pu)
{
    _iq angleDelta_pu = _IQmpy(fm_pu,_IQ(USER_IQ_FULL_SCALE_FREQ_Hz/(USER_PWM_FREQ_kHz*1000.0)));
    _iq angleCompFactor = _IQ(1.0 + (float_t)USER_NUM_PWM_TICKS_PER_ISR_TICK * 0.5);
    _iq angleDeltaComp_pu = _IQmpy(angleDelta_pu, angleCompFactor);
    uint32_t angleMask = ((uint32_t)0xFFFFFFFF >> (32 - GLOBAL_Q));
    _iq angleComp_pu;
    _iq angleTmp_pu;

    // increment the angle
    angleTmp_pu = angleUncomp_pu + angleDeltaComp_pu;

    // mask the angle for wrap around
    // note: must account for the sign of the angle
    angleComp_pu = _IQabs(angleTmp_pu) & angleMask;

    // account for sign
    if(angleTmp_pu < _IQ(0.0))
    {
        angleComp_pu = -angleComp_pu;
    }

    return(angleComp_pu);
} // end of angleDelayComp() function


void runCurrentReconstruction(void)
{
    SVGENCURRENT_MeasureShunt_e measurableShuntThisCycle = SVGENCURRENT_getMode(svgencurrentHandle);
    SVGENCURRENT_RunRegenCurrent(svgencurrentHandle, (MATH_vec3 *)(gAdcData.I.value));

    gIavg.value[0] += (gAdcData.I.value[0] - gIavg.value[0])>>gIavg_shift;
    gIavg.value[1] += (gAdcData.I.value[1] - gIavg.value[1])>>gIavg_shift;
    gIavg.value[2] += (gAdcData.I.value[2] - gIavg.value[2])>>gIavg_shift;

    if(measurableShuntThisCycle > two_phase_measurable)
    {
        gAdcData.I.value[0] = gIavg.value[0];
        gAdcData.I.value[1] = gIavg.value[1];
        gAdcData.I.value[2] = gIavg.value[2];
    }

    return;
} // end of runCurrentReconstruction() function


void runSetTrigger(void)
{
    SVGENCURRENT_IgnoreShunt_e ignoreShuntNextCycle = SVGENCURRENT_getIgnoreShunt(svgencurrentHandle);
    SVGENCURRENT_VmidShunt_e midVolShunt = SVGENCURRENT_getVmid(svgencurrentHandle);

    // Set trigger point in the middle of the low side pulse
    HAL_setTrigger(halHandle,ignoreShuntNextCycle,midVolShunt);

    return;
} // end of runSetTrigger() function

void runFieldWeakening(void)
{
    if(FW_getFlag_enableFw(fwHandle) == true)
    {
        FW_incCounter(fwHandle);

        if(FW_getCounter(fwHandle) > FW_getNumIsrTicksPerFwTick(fwHandle))
        {
            _iq refValue;
            _iq fbackValue;

            FW_clearCounter(fwHandle);

            refValue = gMotorVars.VsRef;
            fbackValue =_IQmpy(gMotorVars.Vs,EST_getOneOverDcBus_pu(estHandle));

            FW_run(fwHandle, refValue, fbackValue, &(gIdq_ref_pu.value[0]));

            gMotorVars.IdRef_A = _IQmpy(gIdq_ref_pu.value[0], _IQ(USER_IQ_FULL_SCALE_CURRENT_A));
        }
    }
    else
    {
        gIdq_ref_pu.value[0] = _IQmpy(gMotorVars.IdRef_A, _IQ(1.0/USER_IQ_FULL_SCALE_CURRENT_A));
    }

    return;
} // end of runFieldWeakening() function


void runOffsetsCalculation(void)
{
    uint16_t cnt;

    // enable the PWM
    HAL_enablePwm(halHandle);

    for(cnt=0;cnt<3;cnt++)
    {
        // Set the PWMs to 50% duty cycle
        gPwmData.Tabc.value[cnt] = _IQ(0.0);

        // reset offsets used
        gOffsets_I_pu.value[cnt] = _IQ(0.0);
        gOffsets_V_pu.value[cnt] = _IQ(0.0);

        // run offset estimation
        FILTER_FO_run(filterHandle[cnt],gAdcData.I.value[cnt]);
        FILTER_FO_run(filterHandle[cnt+3],gAdcData.V.value[cnt]);
    }

    if(gOffsetCalcCount++ >= gUserParams.ctrlWaitTime[CTRL_State_OffLine])
    {
        gMotorVars.Flag_enableOffsetcalc = false;
        gOffsetCalcCount = 0;

        for(cnt=0;cnt<3;cnt++)
        {
            // get calculated offsets from filter
            gOffsets_I_pu.value[cnt] = FILTER_FO_get_y1(filterHandle[cnt]);
            gOffsets_V_pu.value[cnt] = FILTER_FO_get_y1(filterHandle[cnt+3]);

            // clear filters
            FILTER_FO_setInitialConditions(filterHandle[cnt],_IQ(0.0),_IQ(0.0));
            FILTER_FO_setInitialConditions(filterHandle[cnt+3],_IQ(0.0),_IQ(0.0));
        }
    }

    return;
} // end of runOffsetsCalculation() function


void softwareUpdate1p6(EST_Handle handle)
{
    float_t fullScaleInductance = USER_IQ_FULL_SCALE_VOLTAGE_V/(USER_IQ_FULL_SCALE_CURRENT_A*USER_VOLTAGE_FILTER_POLE_rps);
    float_t Ls_coarse_max = _IQ30toF(EST_getLs_coarse_max_pu(handle));
    int_least8_t lShift = ceil(log(USER_MOTOR_Ls_d/(Ls_coarse_max*fullScaleInductance))/log(2.0));
    uint_least8_t Ls_qFmt = 30 - lShift;
    float_t L_max = fullScaleInductance * pow(2.0,lShift);
    _iq Ls_d_pu = _IQ30(USER_MOTOR_Ls_d / L_max);
    _iq Ls_q_pu = _IQ30(USER_MOTOR_Ls_q / L_max);


    // store the results
    EST_setLs_d_pu(handle,Ls_d_pu);
    EST_setLs_q_pu(handle,Ls_q_pu);
    EST_setLs_qFmt(handle,Ls_qFmt);

    return;
} // end of softwareUpdate1p6() function

//! \brief     Setup the Clarke transform for either 2 or 3 sensors.
//! \param[in] handle             The clarke (CLARKE) handle
//! \param[in] numCurrentSensors  The number of current sensors
void setupClarke_I(CLARKE_Handle handle,const uint_least8_t numCurrentSensors)
{
    _iq alpha_sf,beta_sf;

    // initialize the Clarke transform module for current
    if(numCurrentSensors == 3)
    {
        alpha_sf = _IQ(MATH_ONE_OVER_THREE);
        beta_sf = _IQ(MATH_ONE_OVER_SQRT_THREE);
    }
    else if(numCurrentSensors == 2)
    {
        alpha_sf = _IQ(1.0);
        beta_sf = _IQ(MATH_ONE_OVER_SQRT_THREE);
    }
    else
    {
        alpha_sf = _IQ(0.0);
        beta_sf = _IQ(0.0);
    }

    // set the parameters
    CLARKE_setScaleFactors(handle,alpha_sf,beta_sf);
    CLARKE_setNumSensors(handle,numCurrentSensors);

    return;
} // end of setupClarke_I() function


//! \brief     Setup the Clarke transform for either 2 or 3 sensors.
//! \param[in] handle             The clarke (CLARKE) handle
//! \param[in] numVoltageSensors  The number of voltage sensors
void setupClarke_V(CLARKE_Handle handle,const uint_least8_t numVoltageSensors)
{
    _iq alpha_sf,beta_sf;

    // initialize the Clarke transform module for voltage
    if(numVoltageSensors == 3)
    {
        alpha_sf = _IQ(MATH_ONE_OVER_THREE);
        beta_sf = _IQ(MATH_ONE_OVER_SQRT_THREE);
    }
    else
    {
        alpha_sf = _IQ(0.0);
        beta_sf = _IQ(0.0);
    }

    // set the parameters
    CLARKE_setScaleFactors(handle,alpha_sf,beta_sf);
    CLARKE_setNumSensors(handle,numVoltageSensors);

    return;
} // end of setupClarke_V() function


//! \brief     Update the global variables (gMotorVars).
//! \param[in] handle  The estimator (EST) handle
void updateGlobalVariables(EST_Handle handle)
{
    // get the speed estimate
    gMotorVars.Speed_krpm = EST_getSpeed_krpm(handle);



    // get the torque estimate
    {
        _iq Flux_pu = EST_getFlux_pu(handle);
        _iq Id_pu = PID_getFbackValue(pidHandle[1]);
        _iq Iq_pu = PID_getFbackValue(pidHandle[2]);
        _iq Ld_minus_Lq_pu = _IQ30toIQ(EST_getLs_d_pu(handle)-EST_getLs_q_pu(handle));
        _iq Torque_Flux_Iq_Nm = _IQmpy(_IQmpy(Flux_pu,Iq_pu),gTorque_Flux_Iq_pu_to_Nm_sf);
        _iq Torque_Ls_Id_Iq_Nm = _IQmpy(_IQmpy(_IQmpy(Ld_minus_Lq_pu,Id_pu),Iq_pu),gTorque_Ls_Id_Iq_pu_to_Nm_sf);
        _iq Torque_Nm = Torque_Flux_Iq_Nm + Torque_Ls_Id_Iq_Nm;

        gMotorVars.Torque_Nm = Torque_Nm;
    }

    // get the magnetizing current
    gMotorVars.MagnCurr_A = EST_getIdRated(handle);

    // get the rotor resistance
    gMotorVars.Rr_Ohm = EST_getRr_Ohm(handle);

    // get the stator resistance
    gMotorVars.Rs_Ohm = EST_getRs_Ohm(handle);

    // get the online stator resistance
    gMotorVars.RsOnLine_Ohm = EST_getRsOnLine_Ohm(handle);

    // get the stator inductance in the direct coordinate direction
    gMotorVars.Lsd_H = EST_getLs_d_H(handle);

    // get the stator inductance in the quadrature coordinate direction
    gMotorVars.Lsq_H = EST_getLs_q_H(handle);

    // get the flux in V/Hz in floating point
    gMotorVars.Flux_VpHz = EST_getFlux_VpHz(handle);

    // get the flux in Wb in fixed point
    gMotorVars.Flux_Wb = _IQmpy(EST_getFlux_pu(handle),gFlux_pu_to_Wb_sf);

    // get the estimator state
    gMotorVars.EstState = EST_getState(handle);

    // Get the DC buss voltage
    gMotorVars.VdcBus_kV = _IQmpy(gAdcData.dcBus,_IQ(USER_IQ_FULL_SCALE_VOLTAGE_V/1000.0));

    // read Vd and Vq vectors per units
    gMotorVars.Vd = gVdq_out_pu.value[0];
    gMotorVars.Vq = gVdq_out_pu.value[1];

    // calculate vector Vs in per units
    gMotorVars.Vs = _IQsqrt(_IQmpy(gMotorVars.Vd, gMotorVars.Vd) + _IQmpy(gMotorVars.Vq, gMotorVars.Vq));

    // read Id and Iq vectors in amps
    gMotorVars.Id_A = _IQmpy(gIdq_pu.value[0], _IQ(USER_IQ_FULL_SCALE_CURRENT_A));
    gMotorVars.Iq_A = _IQmpy(gIdq_pu.value[1], _IQ(USER_IQ_FULL_SCALE_CURRENT_A));

    // calculate vector Is in amps
    gMotorVars.Is_A = _IQsqrt(_IQmpy(gMotorVars.Id_A, gMotorVars.Id_A) + _IQmpy(gMotorVars.Iq_A, gMotorVars.Iq_A));

    return;
} // end of updateGlobalVariables() function


//! \brief
//  if startup mode has been enabled, store the position of the hall state
//      if estimated speed is less than low_pu, check if bldc mode is disabled
//          if the bldc count is greater than 20, enable bldc mode
//              and if currentctrl flag is enabled, motor runs in torque mode
//                   else it runs in speed mode
//          it increment the count until it reaches 20
//      else if the speed feedback is greater than high_pu, check if the bldc is enabled and if the count is greater than 20
//      disable the bldc mode
//      if currentctrl flag is enabled, motor runs in torque mode
//      else it runs in speed mode


void HALLBLDC_Ctrl_Run(void)
{
    if(gHall_Flag_EnableStartup == true) //  Enable  BLDC mode
    {
        gHall_PwmState = gHall_PwmIndex[gHall_State]; // Store the position of the Hall state

        if(_IQabs(speed_est_pu)  < gHall_speed_FastToBldc_low_pu)	// FAST to Hall
        {
            if(gHall_Flag_EnableBldc == false)
            {
                gInBLDC = false;
                if(gHall_Bldc_Cnt > 20)
                {
                    gHall_Flag_EnableBldc = true;
                    gHall_Bldc_Cnt = 0;

                    if(gHall_Flag_CurrentCtrl == true)		// Torque Control Mode
                    {
                        // The following instructions load the parameters for the speed PI
                        // controller.
                        PID_setGains(pidHandle[0],_IQ(0.784),_IQ(0.183),_IQ(0.0));// From SAKI-1  //10.1 & 0.005
                        //PID_setGains(pidHandle[0],_IQ(2.0),_IQ(0.005),_IQ(0.0)); //From TI

                        // Set the initial condition value for the integrator output to 0
                        PID_setUi(pidHandle[3], _IQmpy(pid[2].Ui, gFast2Hall_Ui_coef));

                    }
                    else		                            // Speed Control Mode
                    {
                        // The following instructions load the parameters for the speed PI
                        // controller.
                        PID_setGains(pidHandle[0],_IQ(0.1),_IQ(0.005),_IQ(0.0));    // From TI
                        //PID_setGains(pidHandle[0],_IQ(0.00002),_IQ(0.005),_IQ(0.0));  //SAKI-1

                        gHall_PwmDuty = pid[2].Ui;

                        // Set the initial condition value for the integrator output to 0
                        PID_setUi(pidHandle[0], _IQmpy(pid[2].Ui, gFast2Hall_Spd_coef));
                    }
                }
                else
                    gHall_Bldc_Cnt++;
            }
        }
        else if(_IQabs(gHall_speed_fdb_pu) > gHall_speed_BldcToFast_high_pu)			// Hall to FAST
        {

            if(gHall_Flag_EnableBldc == true)
            {
                gInBLDC = true;
                if(gHall_Fast_Cnt > 20)
                {
                    gHall_Flag_EnableBldc = false;
                    gHall_Fast_Cnt = 0;

                    if(gHall_Flag_CurrentCtrl == true)		// Torque Control
                    {
                        // The following instructions load the parameters for the speed PI
                        // controller.
                        PID_setGains(pidHandle[0],_IQ(0.784),_IQ(0.183),_IQ(0.0));//SAKI-1  //Old value 10.0 & 0.02
                        //PID_setGains(pidHandle[0],_IQ(1.0),_IQ(0.02),_IQ(0.0));//From TI

                        // Set the initial condition value for the integrator output to 0, Id
                        PID_setUi(pidHandle[1],_IQ(0.0));

                        // Set the initial condition value for the integrator output to 0, Iq
                        PID_setUi(pidHandle[2], _IQmpy(pid[3].Ui, _IQ(0.025))); //default (0.25)


                    }
                    else				                        // speed control
                    {
                        // The following instructions load the parameters for the speed PI
                        // controller.
                        PID_setGains(pidHandle[0],_IQ(2.0),_IQ(0.02),_IQ(0.0));    // SAKI-1 Default value kp=0.2, ki=0.005

                        // Set the initial condition value for the integrator output to 0, speed
                        PID_setUi(pidHandle[0], _IQmpy(gHall_PwmDuty, gHall2Fast_Spd_Coef));

                        // Set the initial condition value for the integrator output to 0, Iq
                        PID_setUi(pidHandle[2], _IQmpy(gHall_PwmDuty, gHall2Fast_Iq_coef));

                        // Set the initial condition value for the integrator output to 0, Id
                        PID_setUi(pidHandle[1],_IQmpy(gHall_PwmDuty, _IQ(0.0)));
                    }

                    PWM_setSocAPulseSrc(hal.pwmHandle[PWM_Number_1],PWM_SocPulseSrc_CounterEqualZero);

                    HAL_enablePwm(halHandle);
                }
                else
                    gHall_Fast_Cnt++;
            }
        }

        if(gHall_Flag_EnableBldc)
        {
            angle_pu = angle_est_pu;
            speed_pu = gHall_speed_fdb_pu;

            if(gHall_Flag_CurrentCtrl == true)		// Torque Control Mode
            {
                gHall_BLDC_Is_fdb_pu = gAdcData.I.value[gHall_BLDC_Flag_Is_fdb];
                //gHall_BLDC_Is_ref_pu = speed_pid_out;
                gHall_BLDC_Is_ref_pu = gIdq_ref_pu.value[1];    // SAKI-1 FOC in torque mode

                // BLDC current loop
                PID_run(pidHandle[3],gHall_BLDC_Is_ref_pu,gHall_BLDC_Is_fdb_pu,&gHall_PwmDuty);

                HALLBLDC_Ctrl_PwmSet(gHall_PwmState, gHall_PwmDuty);
            }
            else	// Speed Control Mode
            {
                gHall_PwmDuty = speed_pid_out;
                HALLBLDC_Ctrl_PwmSet(gHall_PwmState, gHall_PwmDuty);
            }
        }
        else
        {
            angle_pu = angle_est_pu;
            speed_pu = speed_est_pu;
            //PID_setGains(pidHandle[0],_IQ(8.0),_IQ(0.005),_IQ(0.0));    //SAKI-1 set kp and ki value for FOC
        }
    }
    else	//(gHall_Flag_EnableStartup == false)
    {
        angle_pu = angle_est_pu;
        speed_pu = speed_est_pu;
        PeakTorqueMode();
    }

    return;
}

//! \brief
void HALLBLDC_State_Check(void)
{
    // Hall_A, Hall_B, Hall_C
    gHall_GpioData  = (~HAL_readGpio(halHandle, (GPIO_Number_e)HAL_HallGpio_C) & 0x1) << 2;   //CAP1->J10/J4_1->green
    gHall_GpioData += (~HAL_readGpio(halHandle, (GPIO_Number_e)HAL_HallGpio_B) & 0x1) << 1;   //CAP2->J10/J4_2->green&white
    gHall_GpioData += (~HAL_readGpio(halHandle, (GPIO_Number_e)HAL_HallGpio_A) & 0x1);        //CAP3->J10/J4_3->gray&white

    gHall_State = gHall_GpioData;

    if(gHall_State != gHall_PrevState)
    {
        gHallCount++;

        gHall_timer_now = HAL_readTimerCnt(halHandle,2);
        gHall_time_delta_now = gHall_timer_prev - gHall_timer_now;
        gHall_timer_prev = gHall_timer_now;

        gHall_time_delta = (gHall_time_delta_now + gHall_time_delta_prev)>>1;
        gHall_time_delta_prev = gHall_time_delta_now;

        gHall_speed_fdb_0p01Hz = gHall_speed_scale/gHall_time_delta;
        gHall_speed_fdb_pu = gHall_speed_fdb_0p01Hz*gHall_Speed_0p01hz_to_pu_sf;

        gHall_krpm = perUnit_to_Krpm(gHall_speed_fdb_pu);

        //	   	if(TRAJ_getIntValue(trajHandle_spd) < _IQ(0.0))
        //		{
        //			gHall_speed_fdb_pu = -gHall_speed_fdb_pu;
        //		}

        //direction check
        gHall_State_delta = gHall_PwmIndex[gHall_State] - gHall_PwmIndex[gHall_PrevState];

        if((gHall_State_delta == -1) || (gHall_State_delta == 5))
        {
            gHall_dir = 1; 		// positive direction
        }
        else if((gHall_State_delta == 1) || (gHall_State_delta == -5))
        {
            gHall_dir = 2; 		// negative direction
            gHall_speed_fdb_pu = -gHall_speed_fdb_pu;
        }
        else
        {
            gHall_dir = 0; 		// direction change
        }

        //check if dirction is chagned.
        //if direction is changed, speed feedback is reset.
        if(gHall_dir != gHall_dir_prev)
        {
            gHall_speed_fdb_pu = _IQ(0.0);
            gHall_dir_change = 1;
        }
        else
        {
            // gHall_dir_change = 0;
        }

        gHall_LastState = gHall_PrevState;
        gHall_PrevState = gHall_State;
        gHall_dir_prev = gHall_dir;

        gHall_Flag_State_Change = true;
        gHall_PwmCnt = 0;
        gHall_State_delta = 0;
        gstallCount = 0;
    }
    else
    {

        // SAKI-1
        if(gMotorVars.Flag_Run_Identify == true)
        {
            gHall_PwmCnt++;
        }//SAKI-1

        if(gHall_PwmCnt > gHall_PwmCntMax)
        {
            gstallCount++;

            gHall_speed_fdb_pu = _IQ(0.0);
            gHall_PwmCnt = 0;
            gHallCount = 0;
        }
    }

    return;
}

//! \brief
void HALLBLDC_Ctrl_Stop(void)
{
    gHall_Flag_EnableBldc = true;
    gHall_Flag_State_Change = false;
    gHall_speed_fdb_pu = _IQ(0.0);
    EnablePeakCurrent = true;

    // Set the initial condition value for the integrator output to 0
    PID_setUi(pidHandle[0],_IQ(0.0));
    PID_setUi(pidHandle[1],_IQ(0.0));
    PID_setUi(pidHandle[2],_IQ(0.0));
    PID_setUi(pidHandle[3],_IQ(0.0));
}

//! \brief

// This function finds the sensors direction

void HALLBLDC_Ctrl_PwmSet(uint16_t PwmState, _iq PwmDuty)
{
    switch(PwmState)
    {
    case 0:		// V+/W-
    {
        PWM_setOneShotTrip(hal.pwmHandle[PWM_Number_1]);
        PWM_clearOneShotTrip(hal.pwmHandle[PWM_Number_2]);
        PWM_clearOneShotTrip(hal.pwmHandle[PWM_Number_3]);

        gPwmData.Tabc.value[0] = _IQ(0.0);
        gPwmData.Tabc.value[1] = PwmDuty;
        gPwmData.Tabc.value[2] = -PwmDuty;

        gHall_BLDC_Flag_Is_fdb = 1;
        break;
    }
    case 1:		// U+/W-
    {
        PWM_clearOneShotTrip(hal.pwmHandle[PWM_Number_1]);
        PWM_setOneShotTrip(hal.pwmHandle[PWM_Number_2]);
        PWM_clearOneShotTrip(hal.pwmHandle[PWM_Number_3]);

        gPwmData.Tabc.value[0] = PwmDuty;
        gPwmData.Tabc.value[1] = _IQ(0.0);
        gPwmData.Tabc.value[2] = -PwmDuty;

        gHall_BLDC_Flag_Is_fdb = 0;
        break;
    }
    case 2:		//U+/V-
    {
        PWM_clearOneShotTrip(hal.pwmHandle[PWM_Number_1]);
        PWM_clearOneShotTrip(hal.pwmHandle[PWM_Number_2]);
        PWM_setOneShotTrip(hal.pwmHandle[PWM_Number_3]);

        gPwmData.Tabc.value[0] = PwmDuty;
        gPwmData.Tabc.value[1] = -PwmDuty;
        gPwmData.Tabc.value[2] = _IQ(0.0);

        gHall_BLDC_Flag_Is_fdb = 0;
        break;
    }
    case 3:		//W+/V-
    {
        PWM_setOneShotTrip(hal.pwmHandle[PWM_Number_1]);
        PWM_clearOneShotTrip(hal.pwmHandle[PWM_Number_2]);
        PWM_clearOneShotTrip(hal.pwmHandle[PWM_Number_3]);

        gPwmData.Tabc.value[0] = _IQ(0.0);
        gPwmData.Tabc.value[1] = -PwmDuty;
        gPwmData.Tabc.value[2] = PwmDuty;

        gHall_BLDC_Flag_Is_fdb = 2;
        break;
    }
    case 4:		//W+/U-
    {
        PWM_clearOneShotTrip(hal.pwmHandle[PWM_Number_1]);
        PWM_setOneShotTrip(hal.pwmHandle[PWM_Number_2]);
        PWM_clearOneShotTrip(hal.pwmHandle[PWM_Number_3]);

        gPwmData.Tabc.value[0] = -PwmDuty;
        gPwmData.Tabc.value[1] = _IQ(0.0);
        gPwmData.Tabc.value[2] = PwmDuty;

        gHall_BLDC_Flag_Is_fdb = 2;
        break;
    }
    case 5:		// V+/U-
    {
        PWM_clearOneShotTrip(hal.pwmHandle[PWM_Number_1]);
        PWM_clearOneShotTrip(hal.pwmHandle[PWM_Number_2]);
        PWM_setOneShotTrip(hal.pwmHandle[PWM_Number_3]);

        gPwmData.Tabc.value[0] = -PwmDuty;
        gPwmData.Tabc.value[1] = PwmDuty;
        gPwmData.Tabc.value[2] = _IQ(0.0);

        gHall_BLDC_Flag_Is_fdb = 1;
        break;
    }
    case 6:		// V+/W-
    {
        PWM_setOneShotTrip(hal.pwmHandle[PWM_Number_1]);
        PWM_clearOneShotTrip(hal.pwmHandle[PWM_Number_2]);
        PWM_clearOneShotTrip(hal.pwmHandle[PWM_Number_3]);

        gPwmData.Tabc.value[0] = _IQ(0.0);
        gPwmData.Tabc.value[1] = PwmDuty;
        gPwmData.Tabc.value[2] = -PwmDuty;

        gHall_BLDC_Flag_Is_fdb = 1;
        break;
    }
    default:	// N/A
        break;
    }
}


//***********************************SAKI-1*****************************************************

//! \brief          Engine error check
//! return          Status of the error {true: error, false: no error}

//  Get the status of battery, throttle and motor temperature
//  If any one of the status exhibits error, then errorStatus will be true
bool ENGINE_checkForErrors(void)
{
    bool errorStatus ;
    bool batteryStatus;
    bool throttleStatus;
    bool temperatureStatus;

    batteryStatus = getBatteryStatus();

    throttleStatus = getThrottleCalibration();

    temperatureStatus = getMotorTemperature();

    if(batteryStatus || throttleStatus || temperatureStatus)    //if either battery, throttle or temperature exhibits error
    {
        errorStatus = true;         //Error
    }
    else
    {
        errorStatus = false;        //No error
    }

    return(errorStatus);
}//end of ENGINE_checkForErrors()


// gHall_Flag_EnableBldc - Sets the motor to bldc mode
//! \brief              Start the motor from zero speed and
//!                     Set the motor current based on the tps value
void ENGINE_StartUp(void)
{
    if(gHall_Flag_EnableBldc == true)       //Checks whether motor is in bldc mode
    {
        gMotorVars.IqRef_A = _IQ(computeTps(&gAdcData));        //Applied tps value
       //gMotorVars.IqRef_A = _IQ(USER_MOTOR_MAX_CURRENT);      //Max current is applied
    }
    else
    {
        gMotorVars.IqRef_A = _IQ(computeTps(&gAdcData));        //Applied tps value
    }


   /* //check for the hall count
    if(gHallCount > HALL_COUNT )
    {
        gMotorVars.IqRef_A = _IQ(computeTps(&gAdcData));
    }
    else
    {
        gMotorVars.IqRef_A = _IQ(USER_MOTOR_MAX_CURRENT);     //peak current is applied

    }*/

    return;
}//end of ENGINE_StartUp()


//! \brief          Enable the PWM to run the motor based on tps value

//  if the motor is idle/OFF and the throttle is ON
//  make the Run_Identify and Motor_Run flag as true
//  Once the throttle is OFF, reset all the parameters

void ENGINE_ReadyToRun(void)
{
    //check for the start switch  pressed for two times
    //if (gReadyCount == 2)
    //{
    //HAL_setGpioLow(halHandle,(GPIO_Number_e)GPIO_Number_15);        //LED-11 off

    if(gFlag_Motor_Run != true)
    {
        if(ENGINE_isTpsON(&gAdcData))       //Checks whether tps is in ON condition
        {
            gMotorVars.Flag_Run_Identify = true;
            gFlag_Motor_Run = true;
        }
    }

    if(ENGINE_isTpsON(&gAdcData) != true)
    {
        //reset all the parameters
        gMotorVars.Flag_Run_Identify = false;
        gstallCount = 0;
        gTps = _IQ(0.0);
        gMotorTemperature_C = 0.0;
        gFlag_Motor_Run = false;
        gHallCount = 0;
        gTimerCnt = 0;
        gHall_krpm = 0;
        speed_est_pu = 0;
        speed_pu = 0;
        gflag = false;
    }
    //}

    return;
}//end of ENGINE_ReadyToRun()


//! \brief              Check for the tps is ON/OFF
//! \param[in]          pAdcData       The pointer to the ADC data

//  if the data is greater than start band
//      throttle is ON
//  else throttle is OFF

//! \return             tpsOnOff       status of the tps  {true:tpsOn , false: tpsOff}
bool ENGINE_isTpsON(HAL_AdcData_t *pAdcData)
{
    bool tpsOnOff;

    if(pAdcData->potentiometer > START_BAND)
    {
        tpsOnOff = true;                //tpsOn

        //if (gTimerCnt<= MAX_TIMER_CNT_SEC)HAL_acqTimer0Int(halHandle);
    }
    else
    {
        tpsOnOff = false;               //tpsOff
    }

    return(tpsOnOff);
}//end of ENGINE_isTpsON()


//! \brief          Check for the IgnitionSwitch
//!                 {true:motor ready to run,  false: waiting for the IgnitionSwitch}

//  Check for the start switch condition
//  if start switch is pressed and the if engine is not ready
//      Increments the count value until it becomes true
//  otherwise engine is in OFF state

void ENGINE_IgnitionSwitch(void)
{
    //check for the start switch pressed for two times
    if(getStartSwitch())
    {
        if(EngineReady == false)    //Engine is not ready
        {
            gReadyCount++;          //Increments the count value until it becomes true

            EngineReady = true;       //Engine is ON
        }
    }
    else
    {
        EngineReady = false;          //Engine is OFF
    }

    return;
}//end of ENGINE_IgnitionSwitch()


//  Flag_Run_Identify - Identifies the motor, defined in main.h
//  gstallCount - vehicle is in stale condition (not running for a certain amount of time/ idle)
//  STALL_IN_SEC - Specific time period where the vehicle can be in stall state, already defined

//! \brief          Check for the motor overload condition
//                  if a running motor becomes idle for certain amount of time, that means it is overloaded
//                  else it works normal
//! \return         stallCondition       Status of the stall condition {true:overloaded , false: normal}
bool computeOverLoad(void)
{
    bool stallCondition;

    if(gMotorVars.Flag_Run_Identify == true)
    {

        if (gstallCount > STALL_IN_SEC)     //check for stall seconds
        {
            stallCondition = true;
        }
        else
        {
            stallCondition = false;
        }
    }
    return(stallCondition);
}//end of computeOverLoad()


//! \brief          Get the state of the start switch
//! \return         gEngineStartSW       status of the switch {true:pressed , false: not pressed}
bool getStartSwitch(void)
{
    gEngineStartSW = (~HAL_readGpio(halHandle,HAL_GPIO_SW1)& 0x01);

    return (gEngineStartSW);
}//end of getStartSwitch()


//! \brief          Get the state of the stop switch
//! \return         gEngineStopSW       status of the switch {true:pressed , false: not pressed}
bool getStopSwitch(void)
{
    gEngineStopSW = (~HAL_readGpio(halHandle,HAL_GPIO_SW2)&0x01);

    return(gEngineStopSW);
}// end of getStopSwitch()


//! \brief          Compute the tps value from the potentiometer
//! \param[in]      pAdcData     The pointer to the ADC data
//                  check for the start band condition
//                  if the throttle value is higher than start band and
//                      if throttle value is greater or less than the calculated toggling value
//                          if throttle value greater than end band
//                              then Apply Max current
//                          else get the tps value and if tps value is greater than max current
//                              Apply Max current
//                      Assign the throttle value for toggling point
//                  else throttle & toggling point value will be zero when throttle is in zero position
//
//! \return         gTps         Get the throttle position value (tps)
_iq computeTps(HAL_AdcData_t *pAdcData)
{
    _iq throttleValue = pAdcData->potentiometer;

    _iq startValue     = START_BAND - WIDTH_PER_DIVISION;

    //check for start band
    if((throttleValue > START_BAND))
    {
        //remove toggling of throttle value
        if((throttleValue > (togglingPt + TPS_TOLERENCE_IN_PERCENTAGE)) || (throttleValue < (togglingPt - TPS_TOLERENCE_IN_PERCENTAGE)))
        {
            //trim Tps value
            if (throttleValue > END_BAND)
            {
                gTps = MOTOR_MAX_CURRENT;       //Max current is applied
            }
            else
            {
                //get the Tps value
                gTps = ((throttleValue - startValue)/(WIDTH_PER_DIVISION));
                gTps = (gTps * (MOTOR_MAX_CURRENT/NO_OF_DIVISION))+(BASE_CURRENT_A);

                //limit the tps value under the motor max current
                if(gTps > MOTOR_MAX_CURRENT)gTps = MOTOR_MAX_CURRENT;
            }

            //set current throttle value for processing toggling
            togglingPt = throttleValue;
        }
    }
    else
    {
        //when throttle is in zero position
        togglingPt = _IQ(0.0);
        gTps = _IQ(0.0);
    }

    return (gTps);
}//end of computeTps()


//! \brief         Get the status of throttle wiring connection
//! \return        tpsError     The state of the throttle wiring connection
bool getThrottleCalibration(void)
{
    bool tpsError = false;          //No error

    //gtest= gAdcData.potentiometer;

    return (tpsError);
}//end of getThrottleCalibration()


//! \brief          Get the status of battery
//                  if battery status shows greater than 60 or less than 50
//! \return         VoltageErrorr     The state of the battery Voltage
bool getBatteryStatus(void)
{
    bool voltageError;

    if ((gMotorVars.VdcBus_kV > OVER_VOLTAGE_Kv) || (gMotorVars.VdcBus_kV < UNDER_VOLTAGE_Kv))  //Battery greater than 60 or less than 50
    {
        voltageError = true;    //Error

    }
    else
    {
        voltageError = false;   //No error

    }

    return(voltageError);
}//end of getBatteryStatus()


//! \brief          Get the temperature of the motor Using the Rs Online Feature
//                  temperature calculation : (35)+ (RsOnLine_Ohm * (16.3) - 1.0) * (254.4)
//                  if motor temp is greater than max temp then there is an error
//                  else no error
//! \return         temperatureError     The state of the motor temperature
bool getMotorTemperature(void)
{
    bool temperatureError;

    //temperature calculation
    gMotorTemperature_C = (ROOM_TEMP_C)+\
            (gMotorVars.RsOnLine_Ohm * (INV_RS_AT_ROOM_TEMP_INV_OHMS)-1.0)*\
            (INV_COPPER_TEMP_COEF_C);

    if (gMotorTemperature_C > MOTOR_MAX_TEMP_C) //Temperature is greater than 100.0
    {
        temperatureError = true;    //Error
    }
    else
    {
        temperatureError = false;   //No error
    }

    return(temperatureError);
}//end of getMotorTemperature()


//! \brief      Disable limp mode, DRV is in active state
void disableLpmDrv8301(void)
{
    HAL_setGpioHigh(halHandle,(GPIO_Number_e)GPIO_Number_51);

    return;
}//end of disableLpmDrv8301()


//! \brief      Enable limp mode (sleep mode) to save the battery
void enableLpmDrv8301(void)
{
    HAL_setGpioLow(halHandle,(GPIO_Number_e)GPIO_Number_51);

    return;
}//end of enableLpmDrv8301()


#ifdef STARTUP

//! \brief     Starting the motor with three conditions
//!                  1) low speed - set peak current
//!                  2) back to low speed - set low speed current
//!                  3) Crossing the timercnt - set TPS value

// EnablePeakCurrent - global
// _IQabs - Makes negative numbers as positive, defined in IQmathLib.h
// gHall_krpm - revolution per minute, global
// gTimerCnt - reads the timer count, global
// MAX_TIMER_CNT_SEC(delay) - maximum timer count (already defined), global
// USER_MOTOR_MAX_CURRENT - max current the motor should exhibit, current is already defined in user.h
// gHall_speed_FastToBldc_low_pu - when motor enters from FOC(Fast) to BLDC(Sensor) mode, global
// MOTOR_LOWSPEED_CURRENT - current at low speed, global, speed already defined
// _IQ(computeTps(&gAdcData)) - tps value is applied once this function is called

void new_startup(void)
{
    if (EnablePeakCurrent == true)
    {
        if ((_IQabs(gHall_krpm) <=_IQ(0.004)) && (gTimerCnt < MAX_TIMER_CNT_SEC))  //Speed is less than 4rpm and timer gTimerCnt is less than 20 Sec
        {
            gMotorVars.IqRef_A = _IQ(USER_MOTOR_MAX_CURRENT); //set motor max current
        }

        else if ((_IQabs(gHall_krpm) <= gHall_speed_FastToBldc_low_pu) && (gflag == true))  //enter FAST to BLDC condition
        {
            gMotorVars.IqRef_A = _IQ(MOTOR_LOWSPEED_CURRENT); //set constant current
            //gflag = false;
        }
        else if ((gTimerCnt >= MAX_TIMER_CNT_SEC) && (gflag == false))  //Timer is greater than 20 sec and flag is false
        {
            gMotorVars.IqRef_A = _IQ(computeTps(&gAdcData));    //Set tps value
            HAL_setGpioHigh(halHandle,(GPIO_Number_e)GPIO_Number_22); //Turn off STATUS LED
        }
    }
    else
    {
        gMotorVars.IqRef_A = _IQ(computeTps(&gAdcData)); //if speed is above the BLDC to fast condition, Tps value will be activated
        gflag = true;
    }
}//end of new_startup()
#endif


//! \brief      Set the Peak current flag based on the speed of the motor {true: enable peak current; false: disable peak current}
//! \param[in]  Void
//! \return     Void

// EnablePeakCurrentMode, EnablePeakCurrent - global
// IQabs - Makes negative numbers as positive, defined in IQmathLib.h
// speed_est_pu - speed estimation per unit, global
// gHall_speed_FastToBldc_low_pu - when motor enters from FOC(Fast) to BLDC(Sensor) mode, global
// gHall_speed_fdb_pu - speed feedback, global
// gHall_speed_BldcToFast_high_pu - when motor enters from BLDC(Sensor) to FOC(Fast) mode, global

//  Check the following conditions
// if EnablePeakCurrentMode is true and if the speed is less than low_pu
// then enable the PeakCurrent if it is already disabled
// if speed is greater than high_pu
// then disable PeakCurrent if it is already enabled

void PeakTorqueMode(void)
{
    //check enable startup
    if(EnablePeakCurrentMode == true)
    {
        //if speed less than FastToBldc
        if(_IQabs(speed_est_pu)  < gHall_speed_FastToBldc_low_pu)//50
        {
            if(EnablePeakCurrent == false)     //check PeakCurrent
            {
                EnablePeakCurrent = true;      //disable PeakCurrent
            }
        }
        //if speed greater than BldcToFast
        else if(_IQabs(gHall_speed_fdb_pu) > gHall_speed_BldcToFast_high_pu)//100
        {
            if(EnablePeakCurrent  == true)     // check PeakCurrent
            {
                EnablePeakCurrent  = false;    // disable PeakCurrent
            }
        }
    }
    return;
}//end of PeakTorqueMode()



//********************************************END SAKI-1******************************************

// Flag_Run_Identify - Identifies the motor, defined in main.h
// Flag_RunState - Run the motor, defined in main.h
// Flag_enableFlyingStart - User may be moving the vehicle without starting or they may be driving from hill top
//                          with very low speed and when they suddenly increase the speed, the vehicle should be able to
//                          move smoothly without a jerk or giving back and forth movement
//                        - defined in main.h

// Once the motor is identified and if the motor is idle (stop to start condition)
// flying start flag is enabled and reset
// Enable runstate flag (motor starts running)

// if motor is already in run condition (Run to Stop condition)
// disable the flying start
// disable runstate flag

void motor_RunCtrl(void)
{
    if( gMotorVars.Flag_Run_Identify == true)       // Stop to Start
    {

        if(gMotorVars.Flag_RunState == false)
        {
            FS_setFlag_enableFs(fsHandle, gMotorVars.Flag_enableFlyingStart);

            FS_reset(fsHandle);

            gMotorVars.Flag_RunState = true;
        }

    }
    else if(gMotorVars.Flag_RunState == true)  // Run to Stop
    {

        FS_setFlag_enableFs(fsHandle, false);

        gMotorVars.Flag_RunState = false;

        // disable the PWM
        HAL_disablePwm(halHandle);

        //gHall_Flag_EnableBldc = true;
        gHall_Flag_State_Change = false;
        gHall_speed_fdb_pu = _IQ(0.0);

        PID_setUi(pidHandle[0],_IQ(0.0));
        PID_setUi(pidHandle[1],_IQ(0.0));
        PID_setUi(pidHandle[2],_IQ(0.0));
        PID_setUi(pidHandle[3],_IQ(0.0));
    }

    return;
}


//! \brief      Convert per unit to krpm
//! \param[in]  Speed_pu    Speed in per unit
//! \return     Speed_kRPM  Speed in krpm

// USER_IQ_FULL_SCALE_FREQ_Hz is defined in user.h
//! \brief          Defines the full scale frequency for IQ variable, Hz
// _iq pu_to_khz_sf = 338.5/1000.0 = 0.3385
// USER_MOTOR_NUM_POLE_PAIRS is defined in user.h
// _iq khz_to_krpm_sf = 60.0/26 = 2.307
// _iq Mechanical_Freq_kHz = _IQmpy(speed, 0.3385)
// _iq Speed_kRPM = _IQmpy(Mechanical_Freq_kHz, 2.307)

static inline _iq perUnit_to_Krpm(_iq Speed_pu)
{
    _iq pu_to_khz_sf = _IQ(USER_IQ_FULL_SCALE_FREQ_Hz/1000.0);
    _iq khz_to_krpm_sf = _IQ(60.0/USER_MOTOR_NUM_POLE_PAIRS);
    _iq Mechanical_Freq_kHz = _IQmpy(Speed_pu,pu_to_khz_sf);
    _iq Speed_kRPM = _IQmpy(Mechanical_Freq_kHz,khz_to_krpm_sf);

    return(Speed_kRPM);
}//end of perUnit_to_Krpm()


#ifdef ECANA2_0B
//Mailbox 0 is used...
//* This TRANSMITS data to another CAN module using MAILBOX0
void CAN_SendMsg(void)
{
    halHandle->ecanMboxesHandle->MBOX0.MDL.all = 0x55555555; //data
    halHandle->ecanMboxesHandle->MBOX0.MDH.all = 0x55555555; //data

    CAN_enableTx(halHandle->ecanRegsHandle);   // enable the Tx
    CAN_ackTx(halHandle->ecanRegsHandle);      // read the ack
    CAN_clearAckTx(halHandle->ecanRegsHandle); // clear the ack for next Tx

    return;
}
#endif


#ifdef SCIA
void scia_msg(char * chmsg)
{
    uint16_t i;
    i = 0;

    while(*(chmsg+i) != '\0')
    {
        SCI_putDataBlocking(halHandle->sciAHandle, *(chmsg+i));

        i++;
    }
    return;
    //i = 0;
}
#endif




//@} //defgroup
// end of file


