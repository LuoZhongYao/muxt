SET(CMAKE_SYSTEM_NAME QNX)
#this one not so much
SET(CMAKE_SYSTEM_VERSION 6.6)

# specify the cross compiler
SET(CMAKE_C_COMPILER   arm-unknown-nto-qnx6.6.0eabi-gcc)
SET(CMAKE_CXX_COMPILER arm-unknown-nto-qnx6.6.0eabi-g++)
SET(CMAKE_SYSTEM_PROCESSOR ARMVLE-V7A)
add_definitions(-D_QNX_SOURCE)

# where is the target environment 
SET(CMAKE_FIND_ROOT_PATH  ${QNX_HOST})

# search for programs in the build host directories
SET(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM ONLY)
# for libraries and headers in the target directories
SET(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
SET(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
