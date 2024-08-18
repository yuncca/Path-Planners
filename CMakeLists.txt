cmake_minimum_required(VERSION 3.5)

project(pathplan)
set(CMAKE_CXX_STANDARD 17)

include_directories(${CMAKE_SOURCE_DIR}/common)

add_subdirectory(common)

add_subdirectory(planners) 



aux_source_directory(test 
    SRC_FILES
)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)


add_executable(pathplan ${SRC_FILES})

target_link_libraries(pathplan PRIVATE 
    planners 
    common 
)

add_library(pathplan_lib ${SRC_FILES})

target_link_libraries(pathplan PRIVATE 
    planners 
    common
)