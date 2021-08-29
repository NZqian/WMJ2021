include_directories(${CMAKE_CURRENT_LISt_DIR}/include)
get_property(include_dir DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
set(WMJ_VISION_BINO_INCLUDE ${include_dir} PARENT_SCOPE)

add_definitions(-DBINO_CFG="${CMAKE_CURRENT_LIST_DIR}/Binocular.yaml")
add_library(Binocular SHARED ./src/Binocular.cpp)
target_link_libraries(Binocular ${OpenCV_LIBS})
