################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
source/Delta_S_Line.obj: ../source/Delta_S_Line.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"D:/CCS7.2/ccsv7/tools/compiler/ti-cgt-c2000_16.9.3.LTS/bin/cl2000" -v28 -ml -mt --float_support=fpu32 -g --include_path="E:/workspace_v7/kong/math_include" --include_path="E:/workspace_v7/kong/Include" --diag_warning=225 --display_error_number --diag_wrap=off --preproc_with_compile --preproc_dependency="source/Delta_S_Line.d" --obj_directory="source" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

source/SCARA_S_Line.obj: ../source/SCARA_S_Line.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"D:/CCS7.2/ccsv7/tools/compiler/ti-cgt-c2000_16.9.3.LTS/bin/cl2000" -v28 -ml -mt --float_support=fpu32 -g --include_path="E:/workspace_v7/kong/math_include" --include_path="E:/workspace_v7/kong/Include" --diag_warning=225 --display_error_number --diag_wrap=off --preproc_with_compile --preproc_dependency="source/SCARA_S_Line.d" --obj_directory="source" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

source/main.obj: ../source/main.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"D:/CCS7.2/ccsv7/tools/compiler/ti-cgt-c2000_16.9.3.LTS/bin/cl2000" -v28 -ml -mt --float_support=fpu32 -g --include_path="E:/workspace_v7/kong/math_include" --include_path="E:/workspace_v7/kong/Include" --diag_warning=225 --display_error_number --diag_wrap=off --preproc_with_compile --preproc_dependency="source/main.d" --obj_directory="source" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


