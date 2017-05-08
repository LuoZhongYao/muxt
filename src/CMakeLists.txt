add_executable(muxtd
    slip.c
    muxt.c
    )

if(NOT "${CMAKE_SYSTEM_NAME}" MATCHES "QNX")
    target_link_libraries(muxtd util)
endif()

target_compile_definitions(muxtd
    PRIVATE -DMUXTD
    )

add_executable(muxt
    slip.c
    muxt.c
    )

target_compile_definitions(muxt
    PRIVATE -DMUXT
    )

