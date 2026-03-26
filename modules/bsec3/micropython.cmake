# BSEC3 MicroPython module (BSEC v3.x, multi-instance API, Cortex-M33F)

add_library(usermod_bsec3 INTERFACE)

target_sources(usermod_bsec3 INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/bsec_wrapper.c
    ${CMAKE_CURRENT_LIST_DIR}/bsec3_bme68x.c
    ${CMAKE_CURRENT_LIST_DIR}/bme68x.c
)

target_include_directories(usermod_bsec3 INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)

add_library(algobsec3 STATIC IMPORTED)
set_target_properties(algobsec3 PROPERTIES
    IMPORTED_LOCATION ${CMAKE_CURRENT_LIST_DIR}/lib/libalgobsec.a
)

target_link_libraries(usermod INTERFACE usermod_bsec3 algobsec3)
