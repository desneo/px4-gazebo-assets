cmake_minimum_required(VERSION 3.22.1)
project(gz_jielian_a2416a_controller LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-plugin2 REQUIRED)

add_library(gz-jielian-a2416a-controller SHARED
  src/strapdown_controller.cc
  src/tracking.cc)
target_include_directories(gz-jielian-a2416a-controller PRIVATE src)
target_link_libraries(gz-jielian-a2416a-controller PRIVATE
  gz-sim8::core
  gz-transport13::core
  gz-msgs10::core
  gz-plugin2::core)
set_target_properties(gz-jielian-a2416a-controller PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

include(GNUInstallDirs)
install(TARGETS gz-jielian-a2416a-controller
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

include(CTest)
if(BUILD_TESTING)
  add_executable(jielian-tracking-state-test
    tests/tracking_state_test.cc
    src/tracking.cc)
  target_include_directories(jielian-tracking-state-test PRIVATE src)
  add_test(NAME jielian-tracking-state COMMAND jielian-tracking-state-test)
endif()
