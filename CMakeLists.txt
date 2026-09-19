add_library(cutter_comp INTERFACE)

target_sources(cutter_comp INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/cutter_comp.c
    ${CMAKE_CURRENT_LIST_DIR}/cutter_comp_grblhal.c
)
target_include_directories(cutter_comp INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)
