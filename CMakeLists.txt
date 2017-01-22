cmake_minimum_required(VERSION 2.8)

project("CBPFlameFeedback")

add_executable(cbpflamefeedback src/cbpflamefeedback.cpp)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -pthread")
else()
  message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

set(CONFIG_DIR ${PROJECT_SOURCE_DIR}/libraries/config)
set(CONFIG_DIR_BIN ${PROJECT_BINARY_DIR}/config)

add_subdirectory (${CONFIG_DIR} ${CONFIG_DIR_BIN})

include_directories(${PROJECT_SOURCE_DIR}/include ${CONFIG_DIR})

target_link_libraries(cbpflamefeedback config -lrt -lwiringPi)
