include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
get_property(include_dir DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
set(POSE_INCLUDE ${include_dir} PARENT_SCOPE)

add_definitions(-DANGLE_CFG="${CMAKE_CURRENT_SOURCE_DIR}/AngleSolver.yaml")
add_library(AngleSolver SHARED ./src/AngleSolver.cpp)
target_link_libraries(AngleSolver ${OpenCV_LIBS})
exec_program(ln ARGS -s ${CMAKE_CURRENT_LIST_DIR}/AngleSolver.yaml ${PROJECT_BINARY_DIRR} OUTPUT_VARIABLE LN_OUT RETURN_VALUE LN_RET)
