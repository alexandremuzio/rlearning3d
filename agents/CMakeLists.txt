cmake_minimum_required(VERSION 3.0)
project(learning)

set(LIB_DIR ../../../../../itandroids-lib/source)
set(SOURCE_DIR ../../../../source)


include_directories(${SOURCE_DIR})
file(GLOB SRCS *.cpp)
foreach (file_path ${SRCS})
    get_filename_component(dir_path ${file_path} PATH)
    get_filename_component(file_name_we ${file_path} NAME_WE)

    set(TARGET_NAME ${file_name_we}_learning)

    add_executable(${TARGET_NAME} ${file_path})
    target_link_libraries(${TARGET_NAME} itandroids-lib-library core test_source ${Boost_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY})
endforeach ()
