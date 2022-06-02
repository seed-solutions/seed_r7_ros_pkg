include_directories(${catkin_INCLUDE_DIRS})

set(STROKE_CONVERTER ${PROJECT_NAME}_stroke_converter)

add_library(${STROKE_CONVERTER} src/stroke_converter.cpp)
target_link_libraries(${STROKE_CONVERTER} ${catkin_LIBRARIES})
target_compile_options(${STROKE_CONVERTER} PUBLIC -O0 -g -Wall)