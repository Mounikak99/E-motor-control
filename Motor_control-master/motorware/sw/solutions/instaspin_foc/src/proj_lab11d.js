expRemoveAll();
expAdd ("gSystemVars.Flag_enableSystem", getDecimal());
expAdd ("gSystemVars.Flag_enableSynControl", getDecimal());
expAdd ("gSystemVars.Flag_enableRun", getDecimal());
expAdd ("gSystemVars.SpeedSet_krpm", getQValue(24));
expAdd ("gSystemVars.MaxAccelSet_krpmps", getQValue(24));

expAdd ("gMotorVars[0].Speed_krpm", getQValue(24));
expAdd ("gMotorVars[1].Speed_krpm", getQValue(24));
expAdd ("gMotorVars[0].VdcBus_kV", getQValue(24));
expAdd ("gMotorVars[1].VdcBus_kV", getQValue(24));

expAdd ("gMotorVars[0].Flag_enableSys", getDecimal());
expAdd ("gMotorVars[0].Flag_Run_Identify", getDecimal());
expAdd ("gMotorVars[0].Flag_enableForceAngle", getDecimal());
expAdd ("gMotorVars[0].Flag_enableUserParams", getDecimal());
expAdd ("gMotorVars[0].Flag_enableOffsetcalc", getDecimal());
expAdd ("gMotorVars[0].Flag_enableRsRecalc", getDecimal());
expAdd ("gMotorVars[0].EstState");
expAdd ("gMotorVars[0].SpeedRef_krpm", getQValue(24));
expAdd ("gMotorVars[0].MaxAccel_krpmps", getQValue(24));
expAdd ("gMotorVars[0].Speed_krpm", getQValue(24));
expAdd ("gMotorVars[0].Rr_Ohm");
expAdd ("gMotorVars[0].Rs_Ohm");
expAdd ("gMotorVars[0].Lsd_H");
expAdd ("gMotorVars[0].Lsq_H");
expAdd ("gMotorVars[0].Flux_VpHz");
expAdd ("gMotorVars[0].Flux_Wb", getQValue(24));
expAdd ("gMotorVars[0].Vd", getQValue(24));
expAdd ("gMotorVars[0].Vq", getQValue(24));
expAdd ("gMotorVars[0].Vs", getQValue(24));
expAdd ("gMotorVars[0].Id_A", getQValue(24));
expAdd ("gMotorVars[0].Iq_A", getQValue(24));
expAdd ("gMotorVars[0].Is_A", getQValue(24));
expAdd ("gMotorVars[0].VdcBus_kV", getQValue(24));
expAdd ("gOffsets_I_pu[0].value", getQValue(24));
expAdd ("gOffsets_V_pu[0].value", getQValue(24));
expAdd ("pid[0]", getQValue(24));
expAdd ("gPwmData[0]", getQValue(24));
expAdd ("gDrvSpi8301Vars[0]");
expAdd ("gMotorVars[0]");
expAdd ("halMtr[0]");

expAdd ("gMotorVars[1].Flag_enableSys", getDecimal());
expAdd ("gMotorVars[1].Flag_Run_Identify", getDecimal());
expAdd ("gMotorVars[1].Flag_enableForceAngle", getDecimal());
expAdd ("gMotorVars[1].Flag_enableUserParams", getDecimal());
expAdd ("gMotorVars[1].Flag_enableOffsetcalc", getDecimal());
expAdd ("gMotorVars[1].Flag_enableRsRecalc", getDecimal());
expAdd ("gMotorVars[1].EstState");
expAdd ("gMotorVars[1].SpeedRef_krpm", getQValue(24));
expAdd ("gMotorVars[1].MaxAccel_krpmps", getQValue(24));
expAdd ("gMotorVars[1].Speed_krpm", getQValue(24));
expAdd ("gMotorVars[1].Rr_Ohm");
expAdd ("gMotorVars[1].Rs_Ohm");
expAdd ("gMotorVars[1].Lsd_H");
expAdd ("gMotorVars[1].Lsq_H");
expAdd ("gMotorVars[1].Flux_VpHz");
expAdd ("gMotorVars[1].Flux_Wb", getQValue(24));
expAdd ("gMotorVars[1].Vd", getQValue(24));
expAdd ("gMotorVars[1].Vq", getQValue(24));
expAdd ("gMotorVars[1].Vs", getQValue(24));
expAdd ("gMotorVars[1].Id_A", getQValue(24));
expAdd ("gMotorVars[1].Iq_A", getQValue(24));
expAdd ("gMotorVars[1].Is_A", getQValue(24));
expAdd ("gMotorVars[1].VdcBus_kV", getQValue(24));
expAdd ("gOffsets_I_pu[1].value", getQValue(24));
expAdd ("gOffsets_V_pu[1].value", getQValue(24));
expAdd ("pid[1]", getQValue(24));
expAdd ("gPwmData[1]", getQValue(24));
expAdd ("cpu_time[0]");
expAdd ("cpu_time[1]");
expAdd ("gDrvSpi8301Vars[1]");
expAdd ("gMotorVars[1]");
expAdd ("halMtr[1]");