include_directories(${CMAKE_CURRENT_LIST_DIR}/include)
get_property(include_dir DIRECTORY ${CMAKE_CURRENT_LIST_DIR} PROPERTY INCLUDE_DIRECTORIES)
set(PID_INCLUDE ${include_dir} PARENT_SCOPE)

add_definitions(-DPID_CFG="${CMAKE_CURRENT_LIST_DIR}/PID.yaml")
aux_source_directory(./src PID)
add_library(SpeedResolver SHARED ${PID})
target_link_libraries(SpeedResolver Base)
exec_program(ln ARGS -s ${CMAKE_CURRENT_LIST_DIR}/PID.yaml ${PROJECT_BINARY_DIRR} OUTPUT_VARIABLE LN_OUT RETURN_VALUE LN_RET)
