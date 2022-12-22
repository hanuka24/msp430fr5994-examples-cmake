cmake_minimum_required(VERSION 3.6)

include(ExternalProject)

project(Example C)

message(STATUS "CMAKE_C_COMPILER='${CMAKE_C_COMPILER}'")
message(STATUS "CMAKE_CXX_COMPILER='${CMAKE_CXX_COMPILER}'")

# The default C Flags
set(CMAKE_C_FLAGS
"${CMAKE_C_FLAGS} \
-O0 \
-std=c99 \
-Wall \
-g \
-gdwarf-3 \
-fstrict-aliasing \
-Wstrict-aliasing \
-Wno-int-to-pointer-cast \
"
)

# Compile the source
add_subdirectory(Tools)
add_subdirectory(Blinky)
add_subdirectory(LPM35)
add_subdirectory(Timer)
add_subdirectory(Test)
add_subdirectory(Test_LPM35)
add_subdirectory(Test_Si7021)
add_subdirectory(RFM95)