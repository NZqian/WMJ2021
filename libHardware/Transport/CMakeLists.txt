include_directories(
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${SERIAL_INCLUDE}
    ${CAN_INCLUDE}
)
add_subdirectory(SerialPort)
add_subdirectory(CanPort)

get_property(include_dir DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
set(TRANSPORT_INCLUDE 
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${SERIAL_INCLUDE}
    ${CAN_INCLUDE} 
    PARENT_SCOPE)

exec_program(
    ln ARGS
    -s ${CMAKE_CURRENT_LIST_DIR}/Port.yaml ${PROJECT_BINARY_DIRR}
    OUTPUT_VARIABLE LN_OUT
    RETURN_VALUE LN_RET)
