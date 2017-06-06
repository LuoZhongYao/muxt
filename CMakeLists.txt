cmake_minimum_required (VERSION 2.8)

project (muxtd)

set(CMAKE_C_FLAGS_DEBUG "-g" )
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -s")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99 -g")

add_definitions(-D_XOPEN_SOURCE=500)

add_subdirectory(src)
