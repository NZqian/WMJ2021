include_directories(${CMAKE_CURRENT_LIST_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/libBase/include)
include_directories(${PROJECT_SOURCE_DIR}/libHardWare/Transport/include)

if(CMAKE_SYSTEM_NAME MATCHES "Linux")
    find_package(Boost REQUIRED thread system)
elseif(CMAKE_SYSTEM_NAME MATCHES "Darwin")
    set(Boost_INCLUDE_DIR /usr/local/opt/boost@1.60/include)
    link_directories(/usr/local/opt/boost@1.60/lib)
    set(Boost_LIB_DIR /usr/local/opt/boost@1.60/lib)
    set(Boost_LIBRARIES
        ${Boost_LIB_DIR}/libboost_system-mt.dylib
        ${Boost_LIB_DIR}/libboost_filesystem-mt.dylib
        ${Boost_LIB_DIR}/libboost_thread-mt.dylib)
endif()

add_definitions(-DPORT_CFG="${CMAKE_CURRENT_LIST_DIR}/../Port.yaml")
aux_source_directory(./src SERIAL_SRC)
add_library(SerialPort SHARED ${SERIAL_SRC})
target_link_libraries(SerialPort Base ${Boost_LIBRARIES} ${OpenCV_LIBS})

set(SERIAL_INCLUDE
    ${CMAKE_CURRENT_LIST_DIR}/include
    PARENT_SCOPE)
