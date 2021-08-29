add_subdirectory(UsbCapture)
include_directories(${USBCAPTURE_INCLUDE})
add_subdirectory(Transport)
include_directories(${TRANSPORT_INCLUDE})
add_subdirectory(Control)
include_directories(${CONTROL_INCLUDE})


get_property(include_dir DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
set(HARDWARE_INCLUDE ${include_dir} PARENT_SCOPE)
