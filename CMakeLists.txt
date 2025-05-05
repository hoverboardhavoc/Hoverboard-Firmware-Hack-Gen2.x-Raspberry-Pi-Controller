cmake_minimum_required(VERSION 3.13)
project(hoverboard_control C)

# Use C11
set(CMAKE_C_STANDARD 11)

find_package(PkgConfig REQUIRED)
pkg_check_modules(SDL2 REQUIRED sdl2)

# Where to find your public headers
include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories(${SDL2_INCLUDE_DIRS})
link_directories(${SDL2_LIBRARY_DIRS})

# Point at the real source file in src/
add_executable(hoverboard_control
    ${CMAKE_SOURCE_DIR}/src/main.c
    ${CMAKE_SOURCE_DIR}/src/hoverserial.c
)

target_link_libraries(hoverboard_control ${SDL2_LIBRARIES})
