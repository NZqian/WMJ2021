include_directories(${CMAKE_CURRENT_LIST_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/libBase/include)
include_directories(${PROJECT_SOURCE_DIR}/libHardWare/Transport/include)
include_directories(./include)

aux_source_directory(./src CAN_SRC)
add_library(CanPort SHARED ${CAN_SRC})
target_link_libraries(CanPort pthread)

set(CAN_INCLUDE
    ${CMAKE_CURRENT_LIST_DIR}
    PARENT_SCOPE)
