################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
Driver/%.o: ../Driver/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"D:/ti/CCS/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -I"D:/car_example/timx_timer_mode_periodic_sleep" -I"D:/car_example/timx_timer_mode_periodic_sleep/Debug" -I"C:/TI/mspm0_sdk_2_09_00_01/source/third_party/CMSIS/Core/Include" -I"C:/TI/mspm0_sdk_2_09_00_01/source" -gdwarf-3 -MMD -MP -MF"Driver/$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


