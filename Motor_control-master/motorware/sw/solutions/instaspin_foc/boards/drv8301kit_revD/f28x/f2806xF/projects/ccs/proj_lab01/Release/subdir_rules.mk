################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
adc.obj: E:/motor\ control/Motor_control-master/motorware/sw/drivers/adc/src/32b/f28x/f2806x/adc.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs920/ccs/tools/compiler/ti-cgt-c2000_18.12.3.LTS/bin/cl2000" -v28 -ml -mt -O2 --include_path="C:/ti/ccs920/ccs/tools/compiler/ti-cgt-c2000_18.12.3.LTS/include" --include_path="E:/motor control/Motor_control-master/motorware/sw/modules/hal/boards/drv8301kit_revD/f28x/f2806x/src/" --include_path="E:/motor control/Motor_control-master/motorware/sw/solutions/instaspin_foc/boards/drv8301kit_revD/f28x/f2806xF/src" --include_path="E:/motor control/Motor_control-master/motorware" -g --define=FAST_ROM_V1p6 --diag_warning=225 --display_error_number --preproc_with_compile --preproc_dependency="adc.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

clk.obj: E:/motor\ control/Motor_control-master/motorware/sw/drivers/clk/src/32b/f28x/f2806x/clk.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs920/ccs/tools/compiler/ti-cgt-c2000_18.12.3.LTS/bin/cl2000" -v28 -ml -mt -O2 --include_path="C:/ti/ccs920/ccs/tools/compiler/ti-cgt-c2000_18.12.3.LTS/include" --include_path="E:/motor control/Motor_control-master/motorware/sw/modules/hal/boards/drv8301kit_revD/f28x/f2806x/src/" --include_path="E:/motor control/Motor_control-master/motorware/sw/solutions/instaspin_foc/boards/drv8301kit_revD/f28x/f2806xF/src" --include_path="E:/motor control/Motor_control-master/motorware" -g --define=FAST_ROM_V1p6 --diag_warning=225 --display_error_number --preproc_with_compile --preproc_dependency="clk.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

cpu.obj: E:/motor\ control/Motor_control-master/motorware/sw/drivers/cpu/src/32b/f28x/f2806x/cpu.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs920/ccs/tools/compiler/ti-cgt-c2000_18.12.3.LTS/bin/cl2000" -v28 -ml -mt -O2 --include_path="C:/ti/ccs920/ccs/tools/compiler/ti-cgt-c2000_18.12.3.LTS/include" --include_path="E:/motor control/Motor_control-master/motorware/sw/modules/hal/boards/drv8301kit_revD/f28x/f2806x/src/" --include_path="E:/motor control/Motor_control-master/motorware/sw/solutions/instaspin_foc/boards/drv8301kit_revD/f28x/f2806xF/src" --include_path="E:/motor control/Motor_control-master/motorware" -g --define=FAST_ROM_V1p6 --diag_warning=225 --display_error_number --preproc_with_compile --preproc_dependency="cpu.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

drv8301.obj: E:/motor\ control/Motor_control-master/motorware/sw/drivers/drvic/drv8301/src/32b/f28x/f2806x/drv8301.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs920/ccs/tools/compiler/ti-cgt-c2000_18.12.3.LTS/bin/cl2000" -v28 -ml -mt -O2 --include_path="C:/ti/ccs920/ccs/tools/compiler/ti-cgt-c2000_18.12.3.LTS/include" --include_path="E:/motor control/Motor_control-master/motorware/sw/modules/hal/boards/drv8301kit_revD/f28x/f2806x/src/" --include_path="E:/motor control/Motor_control-master/motorware/sw/solutions/instaspin_foc/boards/drv8301kit_revD/f28x/f2806xF/src" --include_path="E:/motor control/Motor_control-master/motorware" -g --define=FAST_ROM_V1p6 --diag_warning=225 --display_error_number --preproc_with_compile --preproc_dependency="drv8301.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

flash.obj: E:/motor\ control/Motor_control-master/motorware/sw/drivers/flash/src/32b/f28x/f2806x/flash.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs920/ccs/tools/compiler/ti-cgt-c2000_18.12.3.LTS/bin/cl2000" -v28 -ml -mt -O2 --include_path="C:/ti/ccs920/ccs/tools/compiler/ti-cgt-c2000_18.12.3.LTS/include" --include_path="E:/motor control/Motor_control-master/motorware/sw/modules/hal/boards/drv8301kit_revD/f28x/f2806x/src/" --include_path="E:/motor control/Motor_control-master/motorware/sw/solutions/instaspin_foc/boards/drv8301kit_revD/f28x/f2806xF/src" --include_path="E:/motor control/Motor_control-master/motorware" -g --define=FAST_ROM_V1p6 --diag_warning=225 --display_error_number --preproc_with_compile --preproc_dependency="flash.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

gpio.obj: E:/motor\ control/Motor_control-master/motorware/sw/drivers/gpio/src/32b/f28x/f2806x/gpio.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs920/ccs/tools/compiler/ti-cgt-c2000_18.12.3.LTS/bin/cl2000" -v28 -ml -mt -O2 --include_path="C:/ti/ccs920/ccs/tools/compiler/ti-cgt-c2000_18.12.3.LTS/include" --include_path="E:/motor control/Motor_control-master/motorware/sw/modules/hal/boards/drv8301kit_revD/f28x/f2806x/src/" --include_path="E:/motor control/Motor_control-master/motorware/sw/solutions/instaspin_foc/boards/drv8301kit_revD/f28x/f2806xF/src" --include_path="E:/motor control/Motor_control-master/motorware" -g --define=FAST_ROM_V1p6 --diag_warning=225 --display_error_number --preproc_with_compile --preproc_dependency="gpio.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

hal.obj: E:/motor\ control/Motor_control-master/motorware/sw/modules/hal/boards/drv8301kit_revD/f28x/f2806x/src/hal.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs920/ccs/tools/compiler/ti-cgt-c2000_18.12.3.LTS/bin/cl2000" -v28 -ml -mt -O2 --include_path="C:/ti/ccs920/ccs/tools/compiler/ti-cgt-c2000_18.12.3.LTS/include" --include_path="E:/motor control/Motor_control-master/motorware/sw/modules/hal/boards/drv8301kit_revD/f28x/f2806x/src/" --include_path="E:/motor control/Motor_control-master/motorware/sw/solutions/instaspin_foc/boards/drv8301kit_revD/f28x/f2806xF/src" --include_path="E:/motor control/Motor_control-master/motorware" -g --define=FAST_ROM_V1p6 --diag_warning=225 --display_error_number --preproc_with_compile --preproc_dependency="hal.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

osc.obj: E:/motor\ control/Motor_control-master/motorware/sw/drivers/osc/src/32b/f28x/f2806x/osc.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs920/ccs/tools/compiler/ti-cgt-c2000_18.12.3.LTS/bin/cl2000" -v28 -ml -mt -O2 --include_path="C:/ti/ccs920/ccs/tools/compiler/ti-cgt-c2000_18.12.3.LTS/include" --include_path="E:/motor control/Motor_control-master/motorware/sw/modules/hal/boards/drv8301kit_revD/f28x/f2806x/src/" --include_path="E:/motor control/Motor_control-master/motorware/sw/solutions/instaspin_foc/boards/drv8301kit_revD/f28x/f2806xF/src" --include_path="E:/motor control/Motor_control-master/motorware" -g --define=FAST_ROM_V1p6 --diag_warning=225 --display_error_number --preproc_with_compile --preproc_dependency="osc.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

pie.obj: E:/motor\ control/Motor_control-master/motorware/sw/drivers/pie/src/32b/f28x/f2806x/pie.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs920/ccs/tools/compiler/ti-cgt-c2000_18.12.3.LTS/bin/cl2000" -v28 -ml -mt -O2 --include_path="C:/ti/ccs920/ccs/tools/compiler/ti-cgt-c2000_18.12.3.LTS/include" --include_path="E:/motor control/Motor_control-master/motorware/sw/modules/hal/boards/drv8301kit_revD/f28x/f2806x/src/" --include_path="E:/motor control/Motor_control-master/motorware/sw/solutions/instaspin_foc/boards/drv8301kit_revD/f28x/f2806xF/src" --include_path="E:/motor control/Motor_control-master/motorware" -g --define=FAST_ROM_V1p6 --diag_warning=225 --display_error_number --preproc_with_compile --preproc_dependency="pie.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

pll.obj: E:/motor\ control/Motor_control-master/motorware/sw/drivers/pll/src/32b/f28x/f2806x/pll.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs920/ccs/tools/compiler/ti-cgt-c2000_18.12.3.LTS/bin/cl2000" -v28 -ml -mt -O2 --include_path="C:/ti/ccs920/ccs/tools/compiler/ti-cgt-c2000_18.12.3.LTS/include" --include_path="E:/motor control/Motor_control-master/motorware/sw/modules/hal/boards/drv8301kit_revD/f28x/f2806x/src/" --include_path="E:/motor control/Motor_control-master/motorware/sw/solutions/instaspin_foc/boards/drv8301kit_revD/f28x/f2806xF/src" --include_path="E:/motor control/Motor_control-master/motorware" -g --define=FAST_ROM_V1p6 --diag_warning=225 --display_error_number --preproc_with_compile --preproc_dependency="pll.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

proj_lab01.obj: E:/motor\ control/Motor_control-master/motorware/sw/solutions/instaspin_foc/src/proj_lab01.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs920/ccs/tools/compiler/ti-cgt-c2000_18.12.3.LTS/bin/cl2000" -v28 -ml -mt -O2 --include_path="C:/ti/ccs920/ccs/tools/compiler/ti-cgt-c2000_18.12.3.LTS/include" --include_path="E:/motor control/Motor_control-master/motorware/sw/modules/hal/boards/drv8301kit_revD/f28x/f2806x/src/" --include_path="E:/motor control/Motor_control-master/motorware/sw/solutions/instaspin_foc/boards/drv8301kit_revD/f28x/f2806xF/src" --include_path="E:/motor control/Motor_control-master/motorware" -g --define=FAST_ROM_V1p6 --diag_warning=225 --display_error_number --preproc_with_compile --preproc_dependency="proj_lab01.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

pwm.obj: E:/motor\ control/Motor_control-master/motorware/sw/drivers/pwm/src/32b/f28x/f2806x/pwm.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs920/ccs/tools/compiler/ti-cgt-c2000_18.12.3.LTS/bin/cl2000" -v28 -ml -mt -O2 --include_path="C:/ti/ccs920/ccs/tools/compiler/ti-cgt-c2000_18.12.3.LTS/include" --include_path="E:/motor control/Motor_control-master/motorware/sw/modules/hal/boards/drv8301kit_revD/f28x/f2806x/src/" --include_path="E:/motor control/Motor_control-master/motorware/sw/solutions/instaspin_foc/boards/drv8301kit_revD/f28x/f2806xF/src" --include_path="E:/motor control/Motor_control-master/motorware" -g --define=FAST_ROM_V1p6 --diag_warning=225 --display_error_number --preproc_with_compile --preproc_dependency="pwm.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

pwr.obj: E:/motor\ control/Motor_control-master/motorware/sw/drivers/pwr/src/32b/f28x/f2806x/pwr.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs920/ccs/tools/compiler/ti-cgt-c2000_18.12.3.LTS/bin/cl2000" -v28 -ml -mt -O2 --include_path="C:/ti/ccs920/ccs/tools/compiler/ti-cgt-c2000_18.12.3.LTS/include" --include_path="E:/motor control/Motor_control-master/motorware/sw/modules/hal/boards/drv8301kit_revD/f28x/f2806x/src/" --include_path="E:/motor control/Motor_control-master/motorware/sw/solutions/instaspin_foc/boards/drv8301kit_revD/f28x/f2806xF/src" --include_path="E:/motor control/Motor_control-master/motorware" -g --define=FAST_ROM_V1p6 --diag_warning=225 --display_error_number --preproc_with_compile --preproc_dependency="pwr.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

spi.obj: E:/motor\ control/Motor_control-master/motorware/sw/drivers/spi/src/32b/f28x/f2806x/spi.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs920/ccs/tools/compiler/ti-cgt-c2000_18.12.3.LTS/bin/cl2000" -v28 -ml -mt -O2 --include_path="C:/ti/ccs920/ccs/tools/compiler/ti-cgt-c2000_18.12.3.LTS/include" --include_path="E:/motor control/Motor_control-master/motorware/sw/modules/hal/boards/drv8301kit_revD/f28x/f2806x/src/" --include_path="E:/motor control/Motor_control-master/motorware/sw/solutions/instaspin_foc/boards/drv8301kit_revD/f28x/f2806xF/src" --include_path="E:/motor control/Motor_control-master/motorware" -g --define=FAST_ROM_V1p6 --diag_warning=225 --display_error_number --preproc_with_compile --preproc_dependency="spi.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

timer.obj: E:/motor\ control/Motor_control-master/motorware/sw/drivers/timer/src/32b/f28x/f2806x/timer.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs920/ccs/tools/compiler/ti-cgt-c2000_18.12.3.LTS/bin/cl2000" -v28 -ml -mt -O2 --include_path="C:/ti/ccs920/ccs/tools/compiler/ti-cgt-c2000_18.12.3.LTS/include" --include_path="E:/motor control/Motor_control-master/motorware/sw/modules/hal/boards/drv8301kit_revD/f28x/f2806x/src/" --include_path="E:/motor control/Motor_control-master/motorware/sw/solutions/instaspin_foc/boards/drv8301kit_revD/f28x/f2806xF/src" --include_path="E:/motor control/Motor_control-master/motorware" -g --define=FAST_ROM_V1p6 --diag_warning=225 --display_error_number --preproc_with_compile --preproc_dependency="timer.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

usDelay.obj: E:/motor\ control/Motor_control-master/motorware/sw/modules/usDelay/src/32b/f28x/usDelay.asm $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs920/ccs/tools/compiler/ti-cgt-c2000_18.12.3.LTS/bin/cl2000" -v28 -ml -mt -O2 --include_path="C:/ti/ccs920/ccs/tools/compiler/ti-cgt-c2000_18.12.3.LTS/include" --include_path="E:/motor control/Motor_control-master/motorware/sw/modules/hal/boards/drv8301kit_revD/f28x/f2806x/src/" --include_path="E:/motor control/Motor_control-master/motorware/sw/solutions/instaspin_foc/boards/drv8301kit_revD/f28x/f2806xF/src" --include_path="E:/motor control/Motor_control-master/motorware" -g --define=FAST_ROM_V1p6 --diag_warning=225 --display_error_number --preproc_with_compile --preproc_dependency="usDelay.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

user.obj: E:/motor\ control/Motor_control-master/motorware/sw/modules/user/src/32b/user.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs920/ccs/tools/compiler/ti-cgt-c2000_18.12.3.LTS/bin/cl2000" -v28 -ml -mt -O2 --include_path="C:/ti/ccs920/ccs/tools/compiler/ti-cgt-c2000_18.12.3.LTS/include" --include_path="E:/motor control/Motor_control-master/motorware/sw/modules/hal/boards/drv8301kit_revD/f28x/f2806x/src/" --include_path="E:/motor control/Motor_control-master/motorware/sw/solutions/instaspin_foc/boards/drv8301kit_revD/f28x/f2806xF/src" --include_path="E:/motor control/Motor_control-master/motorware" -g --define=FAST_ROM_V1p6 --diag_warning=225 --display_error_number --preproc_with_compile --preproc_dependency="user.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

wdog.obj: E:/motor\ control/Motor_control-master/motorware/sw/drivers/wdog/src/32b/f28x/f2806x/wdog.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs920/ccs/tools/compiler/ti-cgt-c2000_18.12.3.LTS/bin/cl2000" -v28 -ml -mt -O2 --include_path="C:/ti/ccs920/ccs/tools/compiler/ti-cgt-c2000_18.12.3.LTS/include" --include_path="E:/motor control/Motor_control-master/motorware/sw/modules/hal/boards/drv8301kit_revD/f28x/f2806x/src/" --include_path="E:/motor control/Motor_control-master/motorware/sw/solutions/instaspin_foc/boards/drv8301kit_revD/f28x/f2806xF/src" --include_path="E:/motor control/Motor_control-master/motorware" -g --define=FAST_ROM_V1p6 --diag_warning=225 --display_error_number --preproc_with_compile --preproc_dependency="wdog.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


