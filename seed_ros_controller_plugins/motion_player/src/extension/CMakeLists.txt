project (${PROJECT_NAME}_extension)

ADD_FILES("${PROJECT_SOURCE_DIR}" "hpp" HEADER_FILES)
ADD_FILES("${PROJECT_SOURCE_DIR}" "cpp" SOURCE_FILES)
ADD_FILE_DIRECTORIES("${PROJECT_SOURCE_DIR}" "hpp" HEADER_DIRS)

set(LIBS ${LIBS} common)

include_directories(include ${HEADER_DIRS})
add_library(${PROJECT_NAME} STATIC ${HEADER_FILES} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} PRIVATE ${LIBS})
target_compile_options(${PROJECT_NAME} PUBLIC -O0 -g -Wall)
