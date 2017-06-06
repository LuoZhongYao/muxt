if(NOT "${CMAKE_SYSTEM_NAME}" MATCHES "QNX")
    set(oslib util)
endif()

add_definitions(-DLC_INCLUDE="lc-addrlabels.h" -D_DEFAULT_SOURCE -D_BSD_SOURCE)

add_executable(muxt
    ftq.c
    slip.c
    serial.c
    channel.c
    )
add_executable(msz msz.c)

target_link_libraries(muxt ${oslib})
target_compile_definitions(muxt PRIVATE -DMUXT)

add_executable(muxtd
    ftq.c
    slip.c
    serial.c
    channel.c
    )
target_link_libraries(muxtd ${oslib})
target_compile_definitions(muxtd PRIVATE -DMUXTD)

install_targets(/bin muxt muxtd msz)
