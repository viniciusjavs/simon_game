cmake_minimum_required(VERSION 3.8.2)
project(hello_world)

#Toolchain file
include(tm4c123g.cmake)

#Tivaware files
set(TIVAWARE_PATH "/home/vinicius/Documentos/genius/tivaware")
include_directories(${TIVAWARE_PATH}/inc)

#Source files
file(GLOB SOURCES "src/*.c" "src/*.cpp")
add_executable(${CMAKE_PROJECT_NAME}.axf ${SOURCES})
