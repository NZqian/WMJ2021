find_package(OpenCV)
include_directories(${CMAKE_CURRENT_LIST_DIR}/include)
include_directories(${CMAKE_CURRENT_LIST_DIR}/../Transport/include)
include_directories(${CMAKE_CURRENT_LIST_DIR}/../Transport/SerialPort/include)
get_property(include_dir DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
set(CONTROL_INCLUDE ${include_dir} PARENT_SCOPE)

add_definitions(-DCONTROL_CFG="${CMAKE_CURRENT_LIST_DIR}/Control.yaml")
aux_source_directory(./src RMCONTROL)
add_library(RMControl SHARED ${RMCONTROL})
target_link_libraries(RMControl SerialPort CanPort)
exec_program(ln ARGS -s ${CMAKE_CURRENT_LIST_DIR}/Control.yaml ${PROJECT_BINARY_DIRR} OUTPUT_VARIABLE LN_OUT RETURN_VALUE LN_RET)


