find_package (OpenCV REQUIRED)
set(USBCAPTURE_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/include PARENT_SCOPE)
add_definitions(-DUSBCAPTURE_CFG="${CMAKE_CURRENT_SOURCE_DIR}/UsbCapture.yaml")
include_directories(include_dir DIRECTORY ${CMAKE_CURRENT_LIST_DIR} PROPERTY INCLUDE_DIRECTORIES)

aux_source_directory(./src USBCAPTURE_SRC)
add_library(UsbCapture SHARED ${USBCAPTURE_SRC})
target_link_libraries(UsbCapture ${OpenCV_LIBS} gxiapi)
exec_program(ln ARGS -s ${CMAKE_CURRENT_LIST_DIR}/UsbCapture.yaml ${PROJECT_BINARY_DIR} OUTPUT_VARIABLE LN_OUT RETURN_VALUE LN_RET)
