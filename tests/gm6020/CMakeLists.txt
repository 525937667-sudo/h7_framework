cmake_minimum_required(VERSION 3.22)
project(gm6020_test C)
set(CMAKE_C_STANDARD 11)
set(ROOT "${CMAKE_CURRENT_LIST_DIR}/../..")
add_executable(gm6020_test
  test_gm6020.c
  ${ROOT}/Skywalker/Application/Src/app_gm6020_test.c
  ${ROOT}/Skywalker/Device/Src/dev_dji_motor.c
  ${ROOT}/Skywalker/Algorithm/Src/alg_conversion.c
  ${ROOT}/Skywalker/Algorithm/Src/alg_motor_tune.c
  ${ROOT}/Skywalker/Algorithm/Src/alg_autotune.c
  ${ROOT}/Skywalker/Algorithm/Src/alg_sysid.c
  ${ROOT}/Skywalker/Algorithm/Src/alg_pid.c
  ${ROOT}/Skywalker/Algorithm/Src/alg_pid_tune.c
  ${ROOT}/Skywalker/Algorithm/Src/alg_dob.c)
target_include_directories(gm6020_test PRIVATE
  stubs
  ${ROOT}/Skywalker/Application/Inc
  ${ROOT}/Skywalker/Device/Inc
  ${ROOT}/Skywalker/Algorithm/Inc
  ${ROOT}/Skywalker/BSP/Inc)
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(gm6020_test PRIVATE -Wall -Wextra)
  target_link_libraries(gm6020_test PRIVATE m)
endif()
enable_testing()
add_test(NAME gm6020_integration COMMAND gm6020_test)

# Optional replay of the user's saved traces; not a test-data dependency in CI.
add_executable(replay_sysid replay_sysid.c ${ROOT}/Skywalker/Algorithm/Src/alg_sysid.c)
target_include_directories(replay_sysid PRIVATE ${ROOT}/Skywalker/Algorithm/Inc)
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(replay_sysid PRIVATE -Wall -Wextra)
  target_link_libraries(replay_sysid PRIVATE m)
endif()
