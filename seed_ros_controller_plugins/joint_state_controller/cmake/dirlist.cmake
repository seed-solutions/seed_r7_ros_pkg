MACRO(FILE_DIRECTORIES root_dir extension return_list)
    FILE(GLOB_RECURSE new_list ${root_dir}/*.${extension})
    SET(dir_list "")
    FOREACH(file_path ${new_list})
        GET_FILENAME_COMPONENT(dir_path ${file_path} PATH)
        SET(dir_list ${dir_list} ${dir_path})
    ENDFOREACH()
    LIST(REMOVE_DUPLICATES dir_list)
    SET(${return_list} ${dir_list})
ENDMACRO()

MACRO(ADD_CMAKE_DIRECTORIES root_dir)
    FILE(GLOB new_list ${root_dir}/*/CMakeLists.txt)
    FOREACH(file_path ${new_list})
        GET_FILENAME_COMPONENT(dir_path ${file_path} PATH)
        message("====== add subdirectory: " ${dir_path} " ======")
        add_subdirectory(${dir_path})
    ENDFOREACH()
ENDMACRO()

FUNCTION(ADD_FILES root_dir extension return_list)
    FILE(GLOB_RECURSE return_list_tmp "${root_dir}/*.${extension}")
    SET(return_list_tmp ${${return_list}} ${return_list_tmp})
    if(return_list_tmp)
        LIST(REMOVE_DUPLICATES return_list_tmp)
     endif()
    SET(${return_list} ${return_list_tmp} PARENT_SCOPE)
ENDFUNCTION()

FUNCTION(ADD_DIRECTORY root_dir result)
    get_filename_component(result_tmp ${root_dir} ABSOLUTE)
    SET(result_tmp ${${result}} ${result_tmp})
    LIST(REMOVE_DUPLICATES result_tmp)
    SET(${result} ${result_tmp} PARENT_SCOPE)
ENDFUNCTION()

FUNCTION(REMOVE_DIRECTORY dir_tgt result)
    FOREACH(dir_tmp ${${result}})
        IF(${dir_tmp} MATCHES ${dir_tgt})
            list(REMOVE_ITEM ${result} ${dir_tmp})
        ENDIF()
    ENDFOREACH()
    SET(${result} ${${result}} PARENT_SCOPE)
ENDFUNCTION()

FUNCTION(ADD_FILE_DIRECTORIES root_dir extension return_list)
    FILE_DIRECTORIES(${root_dir} ${extension} return_list_tmp)
    SET(return_list_tmp ${${return_list}} ${return_list_tmp})
    LIST(REMOVE_DUPLICATES return_list_tmp)
    SET(${return_list} ${return_list_tmp} PARENT_SCOPE)
ENDFUNCTION()

# USE_QT(Widgets Multimedia SerialPort)みたいな感じで使う
MACRO(USE_QT)
    set(CMAKE_AUTOUIC ON)
    set(CMAKE_AUTOMOC ON)
    set(CMAKE_AUTORCC ON)
    
    #共通で利用するリソースファイル
    ADD_FILES("${CMAKE_SOURCE_DIR}/resources/ui" "qrc" HEADER_FILES)

    find_package(Qt5 COMPONENTS Core ${ARGV} REQUIRED)
    foreach(pkg ${ARGV})
        string(CONCAT LIB "Qt5::" ${pkg})
        set(LIBS ${LIBS} ${LIB})
    endforeach()
    if(LIBS)
        LIST(REMOVE_DUPLICATES LIBS)
    endif()
ENDMACRO()

MACRO(USE_CV)
    find_package(OpenCV REQUIRED)

    set(LIBS ${LIBS} ${OpenCV_LIBRARIES})
    if(LIBS)
        LIST(REMOVE_DUPLICATES LIBS)
    endif()

ENDMACRO()

MACRO(USE_LIBRARY name version)
    find_package(${name} ${version} REQUIRED)
    
    set(HEADER_DIRS ${HEADER_DIRS} ${${name}_INCLUDE_DIRS})
    if(HEADER_DIRS)
        LIST(REMOVE_DUPLICATES HEADER_DIRS)
    endif()

    set(LIBS ${LIBS} ${${name}_LIBRARIES})
    if(LIBS)
        LIST(REMOVE_DUPLICATES LIBS)
    endif()

ENDMACRO()

MACRO(USE_ROS)
#    set(CATKIN_DEVEL_PREFIX ${CMAKE_BINARY_DIR}/devel)
    set(CATKIN_ENABLE_TESTING OFF)
    find_package(catkin REQUIRED COMPONENTS ${ARGV})

    set(HEADER_DIRS ${HEADER_DIRS} ${catkin_INCLUDE_DIRS})
    if(HEADER_DIRS)
        LIST(REMOVE_DUPLICATES HEADER_DIRS)
     endif()
    
    set(LIBS ${LIBS} ${catkin_LIBRARIES})
    if(LIBS)
        LIST(REMOVE_DUPLICATES LIBS)
     endif()

ENDMACRO()


MACRO(ADD_ROS_SERVICES)
    #SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY_TMP ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
    add_service_files(FILES ${ARGV})
    generate_messages(DEPENDENCIES)
    #SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY_TMP})
    SET(HEADER_DIRS ${HEADER_DIRS} "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}")
    if(HEADER_DIRS)
        LIST(REMOVE_DUPLICATES HEADER_DIRS)
    endif()
ENDMACRO()

#バージョン取得
MACRO(INIT_PROJECT)
    execute_process(COMMAND git -C ${CMAKE_CURRENT_SOURCE_DIR} rev-parse --short HEAD OUTPUT_VARIABLE REVISION)
    string(REPLACE "\n" "" REVISION ${REVISION} "")

    execute_process(COMMAND git -C ${CMAKE_CURRENT_SOURCE_DIR} tag --points-at OUTPUT_VARIABLE VERSION)
    string(REPLACE "\n" "" VERSION ${VERSION} "")
ENDMACRO()
