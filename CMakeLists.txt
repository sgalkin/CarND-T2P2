project(UnscentedKF)
cmake_minimum_required (VERSION 3.5)

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS 4.9)
    message(FATAL_ERROR "GCC version must be at least 4.9!")
  endif()
endif()

add_definitions(-std=c++11)

set(CMAKE_CXX_FLAGS "-Wall -Wextra -O3 -g")

set(main
  src/main.cpp)

set(test
  test/main.cpp)

set(sources
  src/application.cpp
  src/protocol.cpp
  src/model.cpp
  src/laser.cpp
  src/radar.cpp
  src/application.h
  src/filter.h
  src/protocol.h
  src/model.h
  src/laser.h
  src/radar.h
  src/tools.h
  src/kalman_filter.h
  src/sigma_points.h
  )

set(tests
  test/test_filter.cpp
  test/test_tools.cpp
  test/test_rubric.cpp
  test/test_measurement.cpp
  test/test_model.cpp
  test/test_radar.cpp
  test/test_laser.cpp
  test/test_sigma_points.cpp
  test/test_kalman_filter.cpp
  )

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  execute_process(COMMAND brew --prefix openssl
    OUTPUT_VARIABLE OPENSSL_ROOT_DIR)
  string(STRIP ${OPENSSL_ROOT_DIR} OPENSSL_ROOT_DIR)
endif()

find_package(ZLIB REQUIRED)
find_package(OpenSSL REQUIRED)

include_directories(${OPENSSL_INCLUDE_DIR})

find_library(UWS_LIB uWS)
if(NOT UWS_LIB)
  message(FATAL_ERROR "uWS library not found")
endif()

find_library(UV_LIB uv)
if(NOT UV_LIB)
  message(FATAL_ERROR "uv library not found")
endif()

set(TEST_NAME "Test")
add_executable(${CMAKE_PROJECT_NAME} ${sources} ${main})
add_executable(${TEST_NAME} ${sources} ${tests} ${test})

enable_testing()
add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})

target_link_libraries(${CMAKE_PROJECT_NAME} 
  ${ZLIB_LIBRARIES} 
  ${OPENSSL_LIBRARIES} 
  ${UV_LIB}
  ${UWS_LIB})
