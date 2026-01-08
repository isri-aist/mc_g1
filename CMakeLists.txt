cmake_minimum_required(VERSION 3.10)

project(mc_g1)

set(CMAKE_CXX_STANDARD 17)

find_package(mc_rtc REQUIRED)
find_package(g1_description REQUIRED)

if(${g1_description_FOUND})
  if("${g1_description_INSTALL_PREFIX}" STREQUAL "")
    if("${g1_description_SOURCE_PREFIX}" STREQUAL "")
      message(
        FATAL_ERROR
          "Your g1_description package does not define where to find the data")
    endif()
    set(G1_DESCRIPTION_PATH "${g1_description_SOURCE_PREFIX}")
  else()
    set(G1_DESCRIPTION_PATH "${g1_description_INSTALL_PREFIX}")
  endif()
endif()

message("-- G1_DESCRIPTION_PATH: ${G1_DESCRIPTION_PATH}")
configure_file(src/config.h.in "${CMAKE_CURRENT_BINARY_DIR}/include/config.h")

macro(add_module NAME)
  add_robot(${NAME} src/${NAME}.cpp src/${NAME}.h)
  target_include_directories(${NAME}
                             PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/include")
endmacro()

add_module(g1)

option(DISABLE_TESTS "Disable unit tests" OFF)
if(NOT ${DISABLE_TESTS})
  enable_testing()
  add_subdirectory(tests)
endif()
