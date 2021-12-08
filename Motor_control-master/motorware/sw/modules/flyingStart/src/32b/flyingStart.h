/* --COPYRIGHT--,BSD
 * Copyright (c) 2012, Texas Instruments Incorporated
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
#ifndef _FS_H_
#define _FS_H_

//! \file   modules/FS/src/32b/FS.h
//! \brief  Contains public interface to various functions related
//!         to the FS object
//!
//! (C) Copyright 2011, Texas Instruments, Inc.


// **************************************************************************
// the includes

#include "sw/modules/iqmath/src/32b/IQmathLib.h"
#include "sw/modules/types/src/types.h"

#include "sw/modules/ctrl/src/32b/ctrl.h"

//!
//!
//! \defgroup FS FS
//!
//@{


#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines


//! \brief Defines the Flying Start (FS) minimum switch speed
//!
#define FS_SPEED_MIN                   _IQ(0.0030)


//! \brief Defines the Flying Start (FS) minimum switch speed
//!
#define FS_MAX_CHECK_TIME              (6000)

// **************************************************************************
// the typedefs


//! \brief Defines the Flying Start (FS) data
//!
typedef struct _FS_Obj_
{
    _iq speed_ref_pu;						//!<
    _iq speed_set_pu;						//!<
    _iq speed_est_pu;						//!<

    _iq speed_est_avg;					    //!<

    _iq speed_fs_min;						//!<

    _iq run_direction;					    //!<

    _iq hall_Krpm;                      //SAKI-1
    _iq foc_Krpm;                       //SAKI-1

    uint32_t  cnt_checktime;          	//!< the Flying Start (FS) counter
    uint32_t  max_checktime;           	//!< the Flying Start (FS) hold time

    bool flag_enableSpeedCtrl;			//!< a flag to enable speed control Flying Start (FS)
    bool flag_enableFs;         			//!< a flag to enable Flying Start (FS)
} FS_Obj;


//! \brief Defines the FS handle
//!
typedef struct _FS_Obj_ *FS_Handle;


// **************************************************************************
// the globals

static inline _iq pu_to_Krpm(_iq Speed_pu); //SAKI-1
// **************************************************************************
// the function prototypes


//! \brief     Initializes the Flying Start (FS) object
//! \param[in] pMemory   A pointer to the memory for the Flying Start (FS) object
//! \param[in] numBytes  The number of bytes allocated for the Flying Start (FS) object, bytes
//! \return    The Flying Start (FS) object handle
extern FS_Handle FS_init(void *pMemory, const size_t numBytes);

//! \brief     Sets the enable flag of the Flying Start (FS) (FS) object
//! \param[in] fsHandle  The Flying Start (FS) handle
//! \param[in] state     The Flying Start (FS) enable state
static inline void FS_setSpeedFsMin(FS_Handle fsHandle, const _iq speedFsMin)
{
    FS_Obj *fs = (FS_Obj *)fsHandle;

    fs->speed_fs_min = speedFsMin;

    return;
} // end of FS_setSpeedFsMin() function


static inline FS_setSpeedFsMin_krpm(FS_Handle fsHandle, CTRL_Handle handle,const _iq spd_ref_krpm)
{
    FS_Obj *fs = (FS_Obj *)fsHandle;
    CTRL_Obj *obj = (CTRL_Obj *)handle;

    _iq krpm_to_pu_sf = EST_get_krpm_to_pu_sf(obj->estHandle);

    _iq spd_ref_pu = _IQmpy(spd_ref_krpm,krpm_to_pu_sf);

    fs->speed_fs_min = spd_ref_pu;

    return;
} // end of CTRL_setSpd_ref_krpm() function

//! \brief     Sets the enable flag of the Flying Start (FS) (FS) object
//! \param[in] fsHandle  The Flying Start (FS) handle
//! \param[in] state     The Flying Start (FS) enable state
static inline void FS_setMaxCheckTime(FS_Handle fsHandle, const uint32_t checktime)
{
    FS_Obj *fs = (FS_Obj *)fsHandle;

    fs->max_checktime = checktime;

    return;
} // end of FS_setMaxCheckTime() function

//! \brief     Sets the counter of the Flying Start (FS) (FS) object
//! \param[in] fsHandle  The Flying Start (FS) handle
static inline void FS_setCntCheckTime(FS_Handle fsHandle, const uint32_t checktime)
{
    FS_Obj *fs = (FS_Obj *)fsHandle;

    fs->cnt_checktime = checktime;

    return;
} // end of FS_setCntCheckTime() function

//! \brief     Clears the counter of the Flying Start (FS) (FS) object
//! \param[in] fsHandle  The Flying Start (FS) handle
static inline void FS_clearCntCheckTime(FS_Handle fsHandle)
{
    FS_Obj *fs = (FS_Obj *)fsHandle;

    fs->cnt_checktime = 0;

    return;
} // end of FS_clearCntCheckTime() function


//! \brief     Sets the enable flag of the Flying Start (FS) (FS) object
//! \param[in] fsHandle  The Flying Start (FS) handle
//! \param[in] state     The Flying Start (FS) enable state
static inline void FS_setFlag_enableFs(FS_Handle fsHandle, const bool state)
{
    FS_Obj *fs = (FS_Obj *)fsHandle;

    fs->flag_enableFs = state;

    return;
} // end of FS_setFlag_enableFS() function


//! \brief     Gets the enable flag of the Flying Start (FS) (FS) object
//! \param[in] fsHandle  The Flying Start (FS) handle
//! \return    The Flying Start (FS) enable state
static inline bool FS_getFlag_enableFS(FS_Handle fsHandle)
{
    FS_Obj *fs = (FS_Obj *)fsHandle;

    return(fs->flag_enableFs);
} // end of FS_getFlag_enableFS() function


//! \brief     Gets the enable flag of the Flying Start (FS) (FS) object
//! \param[in] fsHandle  The Flying Start (FS) handle
//! \return    The Flying Start (FS) enable state
static inline bool FS_getFlag_SpeedCtrl(FS_Handle fsHandle)
{
    FS_Obj *fs = (FS_Obj *)fsHandle;

    return(fs->flag_enableSpeedCtrl);
} // end of FS_getFlag_SpeedCtrl() function


//! \brief     Runs the FS controller
//! \param[in] fsHandle    The FS controller handle
//! \param[in]  handle     The controller (CTRL) handle
static inline void FS_run(CTRL_Handle handle, FS_Handle fsHandle)
{
  CTRL_Obj *ctrl = (CTRL_Obj *)handle;

  FS_Obj *fs = (FS_Obj *)fsHandle;

  if(fs->flag_enableFs == true)
  {
	  CTRL_setId_ref_pu(handle, _IQ(0.0));
	  CTRL_setIq_ref_pu(handle, _IQ(0.0));

	  fs->speed_ref_pu = CTRL_getSpd_ref_pu(handle);
	  fs->speed_est_pu = EST_getFm_pu(ctrl->estHandle);
	  fs->speed_est_avg = _IQmpy(fs->speed_est_avg, _IQ(0.8)) + _IQmpy(fs->speed_est_pu, _IQ(0.20));

	  if(fs->cnt_checktime >= fs->max_checktime)
	  {
		  if(_IQabs(fs->speed_est_avg) < fs->speed_fs_min)
		  {
			  fs->speed_set_pu = _IQ(0.0);
		  }
		  else
		  {
			  fs->speed_set_pu = fs->speed_est_avg;
		  }

		  fs->flag_enableFs = false;
		  fs->flag_enableSpeedCtrl = true;

		  TRAJ_setIntValue(ctrl->trajHandle_spd, fs->speed_set_pu);
		  PID_setUi(ctrl->pidHandle_spd, _IQ(0.0));

		  CTRL_setSpd_out_pu(handle, _IQ(0.0));
	  }
	  else
		  fs->cnt_checktime++;
  }
  else
  {
	  fs->flag_enableSpeedCtrl = true;
  }

  return;
} // end of FS_run() function


//! \brief     Runs the FS controller
//! \param[in] fsHandle    The FS controller handle
static inline void FS_reset(FS_Handle fsHandle)
{
    FS_Obj *fs = (FS_Obj *)fsHandle;

    if(fs->flag_enableFs == true)
    {
        fs->flag_enableSpeedCtrl = false;
    }
    else
    {
        fs->flag_enableSpeedCtrl = true;
    }

    fs->speed_est_avg = _IQ(0.0);
    fs->cnt_checktime = 0;
}

/*
//SAKI-1
static inline void FS_run(MATH_vec2 *gIdq_pu, FS_Handle fsHandle, _iq speed_est_pu, _iq gHall_speed_fdb_pu)
{
    FS_Obj *fs = (FS_Obj *)fsHandle;

    if(fs->flag_enableFs == true)
    {
        gIdq_pu->value[0] = _IQ(0.0);
        gIdq_pu->value[1] = _IQ(0.0);

        fs->hall_Krpm = pu_to_Krpm(gHall_speed_fdb_pu); // SAKI-1
        fs->foc_Krpm = pu_to_Krpm(speed_est_pu);  //SAKI-1


        fs->speed_est_pu = speed_est_pu;

        fs->speed_est_avg = _IQmpy(fs->speed_est_avg, _IQ(0.8)) + _IQmpy(fs->speed_est_pu, _IQ(0.20));

        if(fs->cnt_checktime >= fs->max_checktime)
        {
            if(_IQabs(fs->speed_est_avg) < fs->speed_fs_min)
            {
                fs->speed_set_pu = _IQ(0.0);
            }
            else
            {
                fs->speed_set_pu = fs->speed_est_avg;
            }

            fs->flag_enableFs = false;
            fs->flag_enableSpeedCtrl = true;

        }
        else
            fs->cnt_checktime++;
    }
    else
    {
        fs->flag_enableSpeedCtrl = true;
    }

    return;
} // end of FS_run() function   */

//SAKI-1 convert per unit to Krpm
static inline _iq pu_to_Krpm(_iq Speed_pu)
{
    //Convert pu to Krpm
    _iq pu_to_khz_sf = _IQ(USER_IQ_FULL_SCALE_FREQ_Hz/1000.0);
    _iq khz_to_krpm_sf = _IQ(60.0/USER_MOTOR_NUM_POLE_PAIRS);
    _iq Mechanical_Freq_kHz = _IQmpy(Speed_pu,pu_to_khz_sf);
    _iq Speed_kRPM = _IQmpy(Mechanical_Freq_kHz,khz_to_krpm_sf);

    return(Speed_kRPM);
}

#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif // end of _FS_H_ definition

