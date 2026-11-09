cmake_minimum_required(VERSION 3.22)
project(imu_ekf_test C)
set(CMAKE_C_STANDARD 11)
set(ROOT "${CMAKE_CURRENT_LIST_DIR}/../..")
set(MATRIX "${ROOT}/Drivers/CMSIS/DSP/Source/MatrixFunctions")
add_executable(imu_ekf_test
  test_imu_ekf.c
  ${ROOT}/Skywalker/Algorithm/Src/alg_imu_ekf.c
  ${MATRIX}/arm_mat_init_f32.c
  ${MATRIX}/arm_mat_trans_f32.c
  ${MATRIX}/arm_mat_mult_f32.c
  ${MATRIX}/arm_mat_add_f32.c
  ${MATRIX}/arm_mat_sub_f32.c
  ${MATRIX}/arm_mat_inverse_f32.c)
target_include_directories(imu_ekf_test PRIVATE
  ${ROOT}/Skywalker/Algorithm/Inc
  ${ROOT}/Drivers/CMSIS/DSP/Include
  ${ROOT}/Drivers/CMSIS/DSP/PrivateInclude)
# CMSIS-DSP提供的本机GCC兼容路径，避免引入ARM内核汇编。
target_compile_definitions(imu_ekf_test PRIVATE __GNUC_PYTHON__ ARM_MATH_MATRIX_CHECK)
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(imu_ekf_test PRIVATE -Wall -Wextra)
  target_link_libraries(imu_ekf_test PRIVATE m)
endif()
enable_testing()
add_test(NAME imu_ekf_attitude COMMAND imu_ekf_test)
