cmake_minimum_required(VERSION 2.8)
project(WMJVision)
set(CMAKE_CXX_STANDARD 11)

add_definitions(-DRUNE_CFG="${CMAKE_CURRENT_SOURCE_DIR}/ArmorTrigger.yaml")
add_definitions(-DBINO_CFG="${CMAKE_CURRENT_SOURCE_DIR}/../Bino/Binocular.yaml")

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
get_property(include_dir DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
set(WMJ_VISION_RUNE_INCLUDE ${include_dir} PARENT_SCOPE)

add_library(ArmorTrigger SHARED ./src/ArmorTrigger.cpp)
target_link_libraries(ArmorTrigger ${OpenCV_LIBS} Binocular)
exec_program(ln ARGS -s ${CMAKE_CURRENT_LIST_DIR}/ArmorTrigger.yaml ${PROJECT_BINARY_DIRR} OUTPUT_VARIABLE LN_OUT RETURN_VALUE LN_RET)
