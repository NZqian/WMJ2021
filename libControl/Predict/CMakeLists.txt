include_directories(${CMAKE_CURRENT_LIST_DIR}/include)
get_property(include_dir DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
set(PREDICT_INCLUDE ${include_dir} PARENT_SCOPE)

add_definitions(-DPREDICT_CFG="${CMAKE_CURRENT_LIST_DIR}/Predict.yaml")
aux_source_directory(./src PREDICT_SRC)
add_library(MotionPredict SHARED ${PREDICT_SRC})
target_link_libraries(MotionPredict ${OpenCV_LIBS})
exec_program(ln ARGS -s ${CMAKE_CURRENT_LIST_DIR}/Predict.yaml ${PROJECT_BINARY_DIRR} OUTPUT_VARIABLE LN_OUT RETURN_VALUE LN_RET)
