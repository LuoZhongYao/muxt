# this one is important
#SET(QNX TRUE)
SET(CMAKE_SYSTEM_NAME QNX)
#this one not so much
SET(CMAKE_SYSTEM_VERSION 6.6)

# specify the cross compiler
SET(QNXSDK /home/z/tools/qnx)
SET(QNX_TARGET ${QNXSDK}/target/qnx6)
SET(QNX_HOST   ${QNXSDK}/host/linux/x86)
SET(CMAKE_C_COMPILER   ${QNX_HOST}/usr/bin/arm-unknown-nto-qnx6.6.0eabi-gcc)
SET(CMAKE_CXX_COMPILER ${QNX_HOST}/usr/bin/arm-unknown-nto-qnx6.6.0eabi-g++)
SET(CMAKE_SYSTEM_PROCESSOR ARMVLE-V7A)
add_definitions(-D_QNX_SOURCE)

# where is the target environment 
SET(CMAKE_FIND_ROOT_PATH  ${QNXSDK})

# search for programs in the build host directories
SET(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM ONLY)
# for libraries and headers in the target directories
SET(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
SET(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
