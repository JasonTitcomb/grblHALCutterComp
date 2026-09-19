cmake_minimum_required(VERSION 3.15)
project(cutter_comp LANGUAGES C)

add_library(cutter_comp INTERFACE)
target_sources(cutter_comp INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/cutter_comp.c
    ${CMAKE_CURRENT_SOURCE_DIR}/cutter_comp_grblhal.c
)
target_include_directories(cutter_comp INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)
