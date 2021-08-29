include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
add_definitions(-DARMOR_CFG="${CMAKE_CURRENT_LIST_DIR}/ArmorDetector.yaml")
add_definitions(-DSVM_XML="${CMAKE_CURRENT_LIST_DIR}/HOG_SVM.xml")
get_property(include_dir DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
set(WMJ_VISION_ARMOR_INCLUDE ${include_dir} PARENT_SCOPE)

find_package(OpenCV REQUIRED)
include_directories(include ${OpenCV_INCLUDE_DIRS})

add_library(ArmorDebug SHARED ./src/ArmorDebug.cpp)
target_link_libraries(ArmorDebug ${OpenCV_LIBS} Base Binocular)

add_library(ArmorSingle SHARED ./src/ArmorSingle.cpp)
target_link_libraries(ArmorSingle ${OpenCV_LIBS} Base Binocular ArmorDebug)

add_library(ArmorDetector SHARED ./src/ArmorDetector.cpp)
target_link_libraries(ArmorDetector ${OpenCV_LIBS} Base Binocular ArmorSingle)


exec_program(ln ARGS -s ${CMAKE_CURRENT_LIST_DIR}/ArmorDetector.yaml ${PROJECT_BINARY_DIRR} OUTPUT_VARIABLE LN_OUT RETURN_VALUE LN_RET)
