################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Add inputs and outputs from these tool invocations to the build variables 
CMD_SRCS += \
E:/motor\ control/Motor_control-master/motorware/sw/ide/ccs/cmd/f2806x/f28069F_ram_lnk.cmd 

LIB_SRCS += \
E:/motor\ control/Motor_control-master/motorware/sw/modules/fast/lib/32b/f28x/f2806x/2806xRevB_FastSpinROMSymbols.lib \
E:/motor\ control/Motor_control-master/motorware/sw/modules/iqmath/lib/f28x/32b/IQmath.lib 

ASM_SRCS += \
E:/motor\ control/Motor_control-master/motorware/sw/drivers/cpu/src/32b/f28x/f2806x/CodeStartBranch.asm \
E:/motor\ control/Motor_control-master/motorware/sw/modules/usDelay/src/32b/f28x/usDelay.asm 

C_SRCS += \
E:/motor\ control/Motor_control-master/motorware/sw/drivers/adc/src/32b/f28x/f2806x/adc.c \
E:/motor\ control/Motor_control-master/motorware/sw/modules/clarke/src/32b/clarke.c \
E:/motor\ control/Motor_control-master/motorware/sw/drivers/clk/src/32b/f28x/f2806x/clk.c \
E:/motor\ control/Motor_control-master/motorware/sw/drivers/cpu/src/32b/f28x/f2806x/cpu.c \
E:/motor\ control/Motor_control-master/motorware/sw/modules/ctrl/src/32b/ctrl.c \
E:/motor\ control/Motor_control-master/motorware/sw/drivers/drvic/drv8301/src/32b/f28x/f2806x/drv8301.c \
E:/motor\ control/Motor_control-master/motorware/sw/modules/filter/src/32b/filter_fo.c \
E:/motor\ control/Motor_control-master/motorware/sw/drivers/flash/src/32b/f28x/f2806x/flash.c \
E:/motor\ control/Motor_control-master/motorware/sw/drivers/gpio/src/32b/f28x/f2806x/gpio.c \
E:/motor\ control/Motor_control-master/motorware/sw/modules/hal/boards/drv8301kit_revD/f28x/f2806x/src/hal.c \
E:/motor\ control/Motor_control-master/motorware/sw/modules/ipark/src/32b/ipark.c \
E:/motor\ control/Motor_control-master/motorware/sw/modules/offset/src/32b/offset.c \
E:/motor\ control/Motor_control-master/motorware/sw/drivers/osc/src/32b/f28x/f2806x/osc.c \
E:/motor\ control/Motor_control-master/motorware/sw/modules/park/src/32b/park.c \
E:/motor\ control/Motor_control-master/motorware/sw/modules/pid/src/32b/pid.c \
E:/motor\ control/Motor_control-master/motorware/sw/drivers/pie/src/32b/f28x/f2806x/pie.c \
E:/motor\ control/Motor_control-master/motorware/sw/drivers/pll/src/32b/f28x/f2806x/pll.c \
E:/motor\ control/Motor_control-master/motorware/sw/solutions/instaspin_foc/src/proj_lab05a.c \
E:/motor\ control/Motor_control-master/motorware/sw/drivers/pwm/src/32b/f28x/f2806x/pwm.c \
E:/motor\ control/Motor_control-master/motorware/sw/drivers/pwr/src/32b/f28x/f2806x/pwr.c \
E:/motor\ control/Motor_control-master/motorware/sw/drivers/spi/src/32b/f28x/f2806x/spi.c \
E:/motor\ control/Motor_control-master/motorware/sw/modules/svgen/src/32b/svgen.c \
E:/motor\ control/Motor_control-master/motorware/sw/drivers/timer/src/32b/f28x/f2806x/timer.c \
E:/motor\ control/Motor_control-master/motorware/sw/modules/traj/src/32b/traj.c \
E:/motor\ control/Motor_control-master/motorware/sw/modules/user/src/32b/user.c \
E:/motor\ control/Motor_control-master/motorware/sw/drivers/wdog/src/32b/f28x/f2806x/wdog.c 

C_DEPS += \
./adc.d \
./clarke.d \
./clk.d \
./cpu.d \
./ctrl.d \
./drv8301.d \
./filter_fo.d \
./flash.d \
./gpio.d \
./hal.d \
./ipark.d \
./offset.d \
./osc.d \
./park.d \
./pid.d \
./pie.d \
./pll.d \
./proj_lab05a.d \
./pwm.d \
./pwr.d \
./spi.d \
./svgen.d \
./timer.d \
./traj.d \
./user.d \
./wdog.d 

OBJS += \
./CodeStartBranch.obj \
./adc.obj \
./clarke.obj \
./clk.obj \
./cpu.obj \
./ctrl.obj \
./drv8301.obj \
./filter_fo.obj \
./flash.obj \
./gpio.obj \
./hal.obj \
./ipark.obj \
./offset.obj \
./osc.obj \
./park.obj \
./pid.obj \
./pie.obj \
./pll.obj \
./proj_lab05a.obj \
./pwm.obj \
./pwr.obj \
./spi.obj \
./svgen.obj \
./timer.obj \
./traj.obj \
./usDelay.obj \
./user.obj \
./wdog.obj 

ASM_DEPS += \
./CodeStartBranch.d \
./usDelay.d 

OBJS__QUOTED += \
"CodeStartBranch.obj" \
"adc.obj" \
"clarke.obj" \
"clk.obj" \
"cpu.obj" \
"ctrl.obj" \
"drv8301.obj" \
"filter_fo.obj" \
"flash.obj" \
"gpio.obj" \
"hal.obj" \
"ipark.obj" \
"offset.obj" \
"osc.obj" \
"park.obj" \
"pid.obj" \
"pie.obj" \
"pll.obj" \
"proj_lab05a.obj" \
"pwm.obj" \
"pwr.obj" \
"spi.obj" \
"svgen.obj" \
"timer.obj" \
"traj.obj" \
"usDelay.obj" \
"user.obj" \
"wdog.obj" 

C_DEPS__QUOTED += \
"adc.d" \
"clarke.d" \
"clk.d" \
"cpu.d" \
"ctrl.d" \
"drv8301.d" \
"filter_fo.d" \
"flash.d" \
"gpio.d" \
"hal.d" \
"ipark.d" \
"offset.d" \
"osc.d" \
"park.d" \
"pid.d" \
"pie.d" \
"pll.d" \
"proj_lab05a.d" \
"pwm.d" \
"pwr.d" \
"spi.d" \
"svgen.d" \
"timer.d" \
"traj.d" \
"user.d" \
"wdog.d" 

ASM_DEPS__QUOTED += \
"CodeStartBranch.d" \
"usDelay.d" 

ASM_SRCS__QUOTED += \
"E:/motor control/Motor_control-master/motorware/sw/drivers/cpu/src/32b/f28x/f2806x/CodeStartBranch.asm" \
"E:/motor control/Motor_control-master/motorware/sw/modules/usDelay/src/32b/f28x/usDelay.asm" 

C_SRCS__QUOTED += \
"E:/motor control/Motor_control-master/motorware/sw/drivers/adc/src/32b/f28x/f2806x/adc.c" \
"E:/motor control/Motor_control-master/motorware/sw/modules/clarke/src/32b/clarke.c" \
"E:/motor control/Motor_control-master/motorware/sw/drivers/clk/src/32b/f28x/f2806x/clk.c" \
"E:/motor control/Motor_control-master/motorware/sw/drivers/cpu/src/32b/f28x/f2806x/cpu.c" \
"E:/motor control/Motor_control-master/motorware/sw/modules/ctrl/src/32b/ctrl.c" \
"E:/motor control/Motor_control-master/motorware/sw/drivers/drvic/drv8301/src/32b/f28x/f2806x/drv8301.c" \
"E:/motor control/Motor_control-master/motorware/sw/modules/filter/src/32b/filter_fo.c" \
"E:/motor control/Motor_control-master/motorware/sw/drivers/flash/src/32b/f28x/f2806x/flash.c" \
"E:/motor control/Motor_control-master/motorware/sw/drivers/gpio/src/32b/f28x/f2806x/gpio.c" \
"E:/motor control/Motor_control-master/motorware/sw/modules/hal/boards/drv8301kit_revD/f28x/f2806x/src/hal.c" \
"E:/motor control/Motor_control-master/motorware/sw/modules/ipark/src/32b/ipark.c" \
"E:/motor control/Motor_control-master/motorware/sw/modules/offset/src/32b/offset.c" \
"E:/motor control/Motor_control-master/motorware/sw/drivers/osc/src/32b/f28x/f2806x/osc.c" \
"E:/motor control/Motor_control-master/motorware/sw/modules/park/src/32b/park.c" \
"E:/motor control/Motor_control-master/motorware/sw/modules/pid/src/32b/pid.c" \
"E:/motor control/Motor_control-master/motorware/sw/drivers/pie/src/32b/f28x/f2806x/pie.c" \
"E:/motor control/Motor_control-master/motorware/sw/drivers/pll/src/32b/f28x/f2806x/pll.c" \
"E:/motor control/Motor_control-master/motorware/sw/solutions/instaspin_foc/src/proj_lab05a.c" \
"E:/motor control/Motor_control-master/motorware/sw/drivers/pwm/src/32b/f28x/f2806x/pwm.c" \
"E:/motor control/Motor_control-master/motorware/sw/drivers/pwr/src/32b/f28x/f2806x/pwr.c" \
"E:/motor control/Motor_control-master/motorware/sw/drivers/spi/src/32b/f28x/f2806x/spi.c" \
"E:/motor control/Motor_control-master/motorware/sw/modules/svgen/src/32b/svgen.c" \
"E:/motor control/Motor_control-master/motorware/sw/drivers/timer/src/32b/f28x/f2806x/timer.c" \
"E:/motor control/Motor_control-master/motorware/sw/modules/traj/src/32b/traj.c" \
"E:/motor control/Motor_control-master/motorware/sw/modules/user/src/32b/user.c" \
"E:/motor control/Motor_control-master/motorware/sw/drivers/wdog/src/32b/f28x/f2806x/wdog.c" 


