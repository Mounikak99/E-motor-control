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
E:/motor\ control/Motor_control-master/motorware/sw/modules/usDelay/src/32b/f28x/usDelay.asm 

C_SRCS += \
E:/motor\ control/Motor_control-master/motorware/sw/drivers/adc/src/32b/f28x/f2806x/adc.c \
E:/motor\ control/Motor_control-master/motorware/sw/drivers/clk/src/32b/f28x/f2806x/clk.c \
E:/motor\ control/Motor_control-master/motorware/sw/drivers/cpu/src/32b/f28x/f2806x/cpu.c \
E:/motor\ control/Motor_control-master/motorware/sw/drivers/drvic/drv8301/src/32b/f28x/f2806x/drv8301.c \
E:/motor\ control/Motor_control-master/motorware/sw/drivers/flash/src/32b/f28x/f2806x/flash.c \
E:/motor\ control/Motor_control-master/motorware/sw/drivers/gpio/src/32b/f28x/f2806x/gpio.c \
E:/motor\ control/Motor_control-master/motorware/sw/modules/hal/boards/drv8301kit_revD/f28x/f2806x/src/hal.c \
E:/motor\ control/Motor_control-master/motorware/sw/drivers/osc/src/32b/f28x/f2806x/osc.c \
E:/motor\ control/Motor_control-master/motorware/sw/drivers/pie/src/32b/f28x/f2806x/pie.c \
E:/motor\ control/Motor_control-master/motorware/sw/drivers/pll/src/32b/f28x/f2806x/pll.c \
E:/motor\ control/Motor_control-master/motorware/sw/solutions/instaspin_foc/src/proj_lab01.c \
E:/motor\ control/Motor_control-master/motorware/sw/drivers/pwm/src/32b/f28x/f2806x/pwm.c \
E:/motor\ control/Motor_control-master/motorware/sw/drivers/pwr/src/32b/f28x/f2806x/pwr.c \
E:/motor\ control/Motor_control-master/motorware/sw/drivers/spi/src/32b/f28x/f2806x/spi.c \
E:/motor\ control/Motor_control-master/motorware/sw/drivers/timer/src/32b/f28x/f2806x/timer.c \
E:/motor\ control/Motor_control-master/motorware/sw/modules/user/src/32b/user.c \
E:/motor\ control/Motor_control-master/motorware/sw/drivers/wdog/src/32b/f28x/f2806x/wdog.c 

C_DEPS += \
./adc.d \
./clk.d \
./cpu.d \
./drv8301.d \
./flash.d \
./gpio.d \
./hal.d \
./osc.d \
./pie.d \
./pll.d \
./proj_lab01.d \
./pwm.d \
./pwr.d \
./spi.d \
./timer.d \
./user.d \
./wdog.d 

OBJS += \
./adc.obj \
./clk.obj \
./cpu.obj \
./drv8301.obj \
./flash.obj \
./gpio.obj \
./hal.obj \
./osc.obj \
./pie.obj \
./pll.obj \
./proj_lab01.obj \
./pwm.obj \
./pwr.obj \
./spi.obj \
./timer.obj \
./usDelay.obj \
./user.obj \
./wdog.obj 

ASM_DEPS += \
./usDelay.d 

OBJS__QUOTED += \
"adc.obj" \
"clk.obj" \
"cpu.obj" \
"drv8301.obj" \
"flash.obj" \
"gpio.obj" \
"hal.obj" \
"osc.obj" \
"pie.obj" \
"pll.obj" \
"proj_lab01.obj" \
"pwm.obj" \
"pwr.obj" \
"spi.obj" \
"timer.obj" \
"usDelay.obj" \
"user.obj" \
"wdog.obj" 

C_DEPS__QUOTED += \
"adc.d" \
"clk.d" \
"cpu.d" \
"drv8301.d" \
"flash.d" \
"gpio.d" \
"hal.d" \
"osc.d" \
"pie.d" \
"pll.d" \
"proj_lab01.d" \
"pwm.d" \
"pwr.d" \
"spi.d" \
"timer.d" \
"user.d" \
"wdog.d" 

ASM_DEPS__QUOTED += \
"usDelay.d" 

C_SRCS__QUOTED += \
"E:/motor control/Motor_control-master/motorware/sw/drivers/adc/src/32b/f28x/f2806x/adc.c" \
"E:/motor control/Motor_control-master/motorware/sw/drivers/clk/src/32b/f28x/f2806x/clk.c" \
"E:/motor control/Motor_control-master/motorware/sw/drivers/cpu/src/32b/f28x/f2806x/cpu.c" \
"E:/motor control/Motor_control-master/motorware/sw/drivers/drvic/drv8301/src/32b/f28x/f2806x/drv8301.c" \
"E:/motor control/Motor_control-master/motorware/sw/drivers/flash/src/32b/f28x/f2806x/flash.c" \
"E:/motor control/Motor_control-master/motorware/sw/drivers/gpio/src/32b/f28x/f2806x/gpio.c" \
"E:/motor control/Motor_control-master/motorware/sw/modules/hal/boards/drv8301kit_revD/f28x/f2806x/src/hal.c" \
"E:/motor control/Motor_control-master/motorware/sw/drivers/osc/src/32b/f28x/f2806x/osc.c" \
"E:/motor control/Motor_control-master/motorware/sw/drivers/pie/src/32b/f28x/f2806x/pie.c" \
"E:/motor control/Motor_control-master/motorware/sw/drivers/pll/src/32b/f28x/f2806x/pll.c" \
"E:/motor control/Motor_control-master/motorware/sw/solutions/instaspin_foc/src/proj_lab01.c" \
"E:/motor control/Motor_control-master/motorware/sw/drivers/pwm/src/32b/f28x/f2806x/pwm.c" \
"E:/motor control/Motor_control-master/motorware/sw/drivers/pwr/src/32b/f28x/f2806x/pwr.c" \
"E:/motor control/Motor_control-master/motorware/sw/drivers/spi/src/32b/f28x/f2806x/spi.c" \
"E:/motor control/Motor_control-master/motorware/sw/drivers/timer/src/32b/f28x/f2806x/timer.c" \
"E:/motor control/Motor_control-master/motorware/sw/modules/user/src/32b/user.c" \
"E:/motor control/Motor_control-master/motorware/sw/drivers/wdog/src/32b/f28x/f2806x/wdog.c" 

ASM_SRCS__QUOTED += \
"E:/motor control/Motor_control-master/motorware/sw/modules/usDelay/src/32b/f28x/usDelay.asm" 


