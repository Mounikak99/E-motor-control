################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Add inputs and outputs from these tool invocations to the build variables 
CMD_SRCS += \
G:/motor_control/motorware_1_01_00_18/sw/ide/ccs/cmd/f2806x/f28069M_ram_lnk.cmd 

LIB_SRCS += \
G:/motor_control/motorware_1_01_00_18/sw/modules/fast/lib/32b/f28x/f2806x/2806xRevB_FastSpinROMSymbols.lib \
G:/motor_control/motorware_1_01_00_18/sw/modules/iqmath/lib/f28x/32b/IQmath.lib \
G:/motor_control/motorware_1_01_00_18/sw/modules/spintac/lib/32b/SpinTAC.lib 

ASM_SRCS += \
G:/motor_control/motorware_1_01_00_18/sw/drivers/cpu/src/32b/f28x/f2806x/CodeStartBranch.asm \
G:/motor_control/motorware_1_01_00_18/sw/modules/usDelay/src/32b/f28x/usDelay.asm 

C_SRCS += \
G:/motor_control/motorware_1_01_00_18/sw/drivers/adc/src/32b/f28x/f2806x/adc.c \
G:/motor_control/motorware_1_01_00_18/sw/modules/clarke/src/32b/clarke.c \
G:/motor_control/motorware_1_01_00_18/sw/drivers/clk/src/32b/f28x/f2806x/clk.c \
G:/motor_control/motorware_1_01_00_18/sw/drivers/cpu/src/32b/f28x/f2806x/cpu.c \
G:/motor_control/motorware_1_01_00_18/sw/modules/ctrl/src/32b/ctrl.c \
G:/motor_control/motorware_1_01_00_18/sw/drivers/drvic/drv8301/src/32b/f28x/f2806x/drv8301.c \
G:/motor_control/motorware_1_01_00_18/sw/modules/filter/src/32b/filter_fo.c \
G:/motor_control/motorware_1_01_00_18/sw/drivers/flash/src/32b/f28x/f2806x/flash.c \
G:/motor_control/motorware_1_01_00_18/sw/drivers/gpio/src/32b/f28x/f2806x/gpio.c \
G:/motor_control/motorware_1_01_00_18/sw/modules/hal/boards/drv8301kit_revD/f28x/f2806x/src/hal.c \
G:/motor_control/motorware_1_01_00_18/sw/modules/ipark/src/32b/ipark.c \
G:/motor_control/motorware_1_01_00_18/sw/modules/offset/src/32b/offset.c \
G:/motor_control/motorware_1_01_00_18/sw/drivers/osc/src/32b/f28x/f2806x/osc.c \
G:/motor_control/motorware_1_01_00_18/sw/modules/park/src/32b/park.c \
G:/motor_control/motorware_1_01_00_18/sw/modules/pid/src/32b/pid.c \
G:/motor_control/motorware_1_01_00_18/sw/drivers/pie/src/32b/f28x/f2806x/pie.c \
G:/motor_control/motorware_1_01_00_18/sw/drivers/pll/src/32b/f28x/f2806x/pll.c \
G:/motor_control/motorware_1_01_00_18/sw/solutions/instaspin_motion/src/proj_lab06a.c \
G:/motor_control/motorware_1_01_00_18/sw/drivers/pwm/src/32b/f28x/f2806x/pwm.c \
G:/motor_control/motorware_1_01_00_18/sw/drivers/pwr/src/32b/f28x/f2806x/pwr.c \
G:/motor_control/motorware_1_01_00_18/sw/drivers/spi/src/32b/f28x/f2806x/spi.c \
G:/motor_control/motorware_1_01_00_18/sw/modules/svgen/src/32b/svgen.c \
G:/motor_control/motorware_1_01_00_18/sw/drivers/timer/src/32b/f28x/f2806x/timer.c \
G:/motor_control/motorware_1_01_00_18/sw/modules/traj/src/32b/traj.c \
G:/motor_control/motorware_1_01_00_18/sw/modules/user/src/32b/user.c \
G:/motor_control/motorware_1_01_00_18/sw/drivers/wdog/src/32b/f28x/f2806x/wdog.c 

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
./proj_lab06a.d \
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
./proj_lab06a.obj \
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
"proj_lab06a.obj" \
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
"proj_lab06a.d" \
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
"G:/motor_control/motorware_1_01_00_18/sw/drivers/cpu/src/32b/f28x/f2806x/CodeStartBranch.asm" \
"G:/motor_control/motorware_1_01_00_18/sw/modules/usDelay/src/32b/f28x/usDelay.asm" 

C_SRCS__QUOTED += \
"G:/motor_control/motorware_1_01_00_18/sw/drivers/adc/src/32b/f28x/f2806x/adc.c" \
"G:/motor_control/motorware_1_01_00_18/sw/modules/clarke/src/32b/clarke.c" \
"G:/motor_control/motorware_1_01_00_18/sw/drivers/clk/src/32b/f28x/f2806x/clk.c" \
"G:/motor_control/motorware_1_01_00_18/sw/drivers/cpu/src/32b/f28x/f2806x/cpu.c" \
"G:/motor_control/motorware_1_01_00_18/sw/modules/ctrl/src/32b/ctrl.c" \
"G:/motor_control/motorware_1_01_00_18/sw/drivers/drvic/drv8301/src/32b/f28x/f2806x/drv8301.c" \
"G:/motor_control/motorware_1_01_00_18/sw/modules/filter/src/32b/filter_fo.c" \
"G:/motor_control/motorware_1_01_00_18/sw/drivers/flash/src/32b/f28x/f2806x/flash.c" \
"G:/motor_control/motorware_1_01_00_18/sw/drivers/gpio/src/32b/f28x/f2806x/gpio.c" \
"G:/motor_control/motorware_1_01_00_18/sw/modules/hal/boards/drv8301kit_revD/f28x/f2806x/src/hal.c" \
"G:/motor_control/motorware_1_01_00_18/sw/modules/ipark/src/32b/ipark.c" \
"G:/motor_control/motorware_1_01_00_18/sw/modules/offset/src/32b/offset.c" \
"G:/motor_control/motorware_1_01_00_18/sw/drivers/osc/src/32b/f28x/f2806x/osc.c" \
"G:/motor_control/motorware_1_01_00_18/sw/modules/park/src/32b/park.c" \
"G:/motor_control/motorware_1_01_00_18/sw/modules/pid/src/32b/pid.c" \
"G:/motor_control/motorware_1_01_00_18/sw/drivers/pie/src/32b/f28x/f2806x/pie.c" \
"G:/motor_control/motorware_1_01_00_18/sw/drivers/pll/src/32b/f28x/f2806x/pll.c" \
"G:/motor_control/motorware_1_01_00_18/sw/solutions/instaspin_motion/src/proj_lab06a.c" \
"G:/motor_control/motorware_1_01_00_18/sw/drivers/pwm/src/32b/f28x/f2806x/pwm.c" \
"G:/motor_control/motorware_1_01_00_18/sw/drivers/pwr/src/32b/f28x/f2806x/pwr.c" \
"G:/motor_control/motorware_1_01_00_18/sw/drivers/spi/src/32b/f28x/f2806x/spi.c" \
"G:/motor_control/motorware_1_01_00_18/sw/modules/svgen/src/32b/svgen.c" \
"G:/motor_control/motorware_1_01_00_18/sw/drivers/timer/src/32b/f28x/f2806x/timer.c" \
"G:/motor_control/motorware_1_01_00_18/sw/modules/traj/src/32b/traj.c" \
"G:/motor_control/motorware_1_01_00_18/sw/modules/user/src/32b/user.c" \
"G:/motor_control/motorware_1_01_00_18/sw/drivers/wdog/src/32b/f28x/f2806x/wdog.c" 


