cmake_minimum_required(VERSION 3.22.1)
project(gz_gimbal_controller VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc video tracking)

add_library(gz-gimbal-controller SHARED
  src/gimbal_controller.cc
  src/kinematics.cc
  src/tracker.cc
  src/control.cc
  src/mini_json.cc)

target_include_directories(gz-gimbal-controller PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS})

target_link_libraries(gz-gimbal-controller PRIVATE
  gz-sim8::core
  gz-transport13::core
  gz-msgs10::core
  gz-plugin2::core
  ${OpenCV_LIBRARIES})

# 输出为 libgz-gimbal-controller.so，SDF 中以 filename="gz-gimbal-controller" 引用
set_target_properties(gz-gimbal-controller PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

install(TARGETS gz-gimbal-controller
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

include(CTest)
if(BUILD_TESTING)
  add_executable(gimbal-control-test
    tests/control_test.cc
    src/mini_json.cc
    src/control.cc
    src/tracker.cc
    src/kinematics.cc)
  target_include_directories(gimbal-control-test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(gimbal-control-test PRIVATE ${OpenCV_LIBRARIES})
  add_test(NAME gimbal-control COMMAND gimbal-control-test)
  add_test(NAME gimbal-control-watchdog COMMAND gimbal-control-test --regressions)
  set_tests_properties(gimbal-control-watchdog PROPERTIES LABELS regression)
endif()
