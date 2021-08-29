set(CMAKE_CXX_STANDARD 11)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
get_property(include_dir DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
set(WMJ_CONTROL_AIMER_INCLUDE ${include_dir} PARENT_SCOPE)

add_library(Aimer SHARED ./src/Aimer.cpp ./src/Ransac.cpp)
target_link_libraries(Aimer AngleSolver Base)
add_library(StaticAimer SHARED ./src/StaticAimer.cpp)
target_link_libraries(StaticAimer Aimer AngleSolver)
add_library(AngleAimer SHARED ./src/AngleAimer.cpp)
target_link_libraries(AngleAimer Aimer)

add_definitions(-DTOP_CFG="${CMAKE_CURRENT_LIST_DIR}/TopAimer.yaml")
exec_program(ln ARGS -s ${CMAKE_CURRENT_LIST_DIR}/TopAimer.yaml ${PROJECT_BINARY_DIRR} OUTPUT_VARIABLE LN_OUT RETURN_VALUE LN_RET)