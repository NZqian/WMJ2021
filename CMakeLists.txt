cmake_minimum_required(VERSION 3.1)
if(EXISTS /usr/bin/clang AND EXISTS /usr/bin/clang++)
   set(CMAKE_C_COMPILER /usr/bin/clang)
   set(CMAKE_CXX_COMPILER /usr/bin/clang++)
endif()
project(WMJVision)

set(CMAKE_CXX_STANDARD 11)
set(PROJECT_ROOT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
set(TEST_PATH ${CMAKE_CURRENT_LIST_DIR}/test)
message(STATUS "##########${CMAKE_SYSTEM_NAME}#########")
if(CMAKE_SYSTEM_NAME MATCHES "Linux")

elseif(CMAKE_SYSTEM_NAME MATCHES "Darwin")
    set(OpenCV_DIR /usr/local/opt/opencv@3/share/OpenCV)
endif()

add_definitions(-DROOT_PATH="${CMAKE_CURRENT_LIST_DIR}")
add_definitions(-DFSM_CFG="${CMAKE_CURRENT_LIST_DIR}/libFSM/StateMachine.yaml")
add_definitions(-DAUTOEXPOSURE_CFG="${CMAKE_CURRENT_LIST_DIR}/libFSM/AutoExposure.yaml")
add_definitions(-D${CMAKE_SYSTEM_NAME})

exec_program(
    rm ARGS
    ${PROJECT_BINARY_DIR}/*.yaml
    OUTPUT_VARIABLE RM_OUT
    RETURN_VALUE RM_RET)
exec_program(
    rm ARGS
    ${CMAKE_CURRENT_LIST_DIR}/*.yaml
    OUTPUT_VARIABLE RM_OUT
    RETURN_VALUE RM_RET)
exec_program(
    ln ARGS
    -s ${CMAKE_CURRENT_LIST_DIR}/libFSM/AutoExposure.yaml ${PROJECT_BINARY_DIR}
    OUTPUT_VARIABLE LN_OUT
    RETURN_VALUE LN_RET)
exec_program(
    ln ARGS
    -s ${CMAKE_CURRENT_LIST_DIR}/shell/* ${PROJECT_BINARY_DIR}
    OUTPUT_VARIABLE LN_OUT
    RETURN_VALUE LN_RET)
if(CMAKE_SYSTEM_NAME MATCHES "Darwin")
    include_directories("/usr/local/include")
    link_directories(/usr/local/opt/opencv@3/lib)
    link_directories(/usr/local/lib)
    link_libraries(${OpenCV_LIBS})
endif()
exec_program(
    mkdir ARGS
    -p ${CMAKE_CURRENT_LIST_DIR}/video
    OUTPUT_VARIABLE MKDIR_OUT
    RETURN_VALUE MKDIR_RET)

# Compile Depend modules########
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/libBase)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/libControl)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/libHardware)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/libVision)

include_directories(${BASE_INCLUDE})
include_directories(${CONTROL_INCLUDE})
include_directories(${HARDWARE_INCLUDE})
include_directories(${VISION_INCLUDE})

# Set Variable########
set(VISION_LIB ArmorDetector ArmorTrigger Binocular)
set(HARDWARE_LIB RMControl UsbCapture)
set(CONTROL_LIB SpeedResolver AngleSolver MotionPredict StaticAimer AngleAimer)
set(WMJ_LIBRARIES Base ${VISION_LIB} ${HARDWARE_LIB} ${CONTROL_LIB})

aux_source_directory(${CMAKE_CURRENT_LIST_DIR}/libFSM/src/ STATEMACHINE)
add_library(StateMachine SHARED ${STATEMACHINE})
target_link_libraries(StateMachine ${WMJ_LIBRARIES} ${Boost_LIBRARIES})

add_executable(Main ${PROJECT_ROOT_DIRECTORY}/Main.cpp)
target_link_libraries(Main StateMachine)

#find_package(PythonLibs 3.6)
#if(EXISTS ${PYTHON_LIBRARIES})
#add_executable(PlotCSV  ${CMAKE_CURRENT_LIST_DIR}/test/PlotCSV.cpp)
#target_include_directories(PlotCSV PRIVATE ${PYTHON_INCLUDE_DIR})
#target_link_libraries (PlotCSV ${OpenCV_LIBS} ${PYTHON_LIBRARIES} ${WMJ_LIBRARIES})
#endif()


# test###########
macro(add_test_file TEST_FILE)
    add_executable(${TEST_FILE} ${TEST_PATH}/${TEST_FILE}.cpp)
    target_link_libraries(${TEST_FILE} ${WMJ_LIBRARIES})
endmacro()

# Calib
add_test_file(TakePhoto)
add_test_file(CalibMono)
add_test_file(CalibBino)
add_test_file(Undistort)

# libHardware
add_test_file(CameraTest)
add_test_file(AngleTest)
add_test_file(SetExposureTest)

# libVision
add_test_file(DoubleArmorTest)
add_test_file(ArmorTest)
add_test_file(RuneVisionTest)
add_test_file(RuneTest)
add_test_file(TopTest)

# others
add_test_file(Recorder)
add_test_file(clear)
