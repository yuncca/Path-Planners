get_filename_component(CURRENT_FOLDER ${CMAKE_CURRENT_SOURCE_DIR} NAME)

set(TargetName ${CURRENT_FOLDER})

file(GLOB SOURCE_FILES
  "${CMAKE_CURRENT_SOURCE_DIR}/*.cc"
  "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
)

list(FILTER SOURCE_FILES EXCLUDE REGEX ".*_test\\.cc$")
list(FILTER SOURCE_FILES EXCLUDE REGEX ".*_test\\.cpp$")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../bin)
# set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../lib)
# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../lib)

add_executable(${TargetName}
  ${SOURCE_FILES}
)

add_library(l${TargetName} SHARED
  ${SOURCE_FILES}
)

target_link_libraries(${TargetName}
  common
)

file(GLOB HEADER_FILES 
  "${CMAKE_CURRENT_SOURCE_DIR}/*.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp"
)

foreach(v ${HEADER_FILES})
    string(REGEX MATCH "${TargetName}/.*" relative_path ${v})
    string(FIND ${relative_path} "/" test REVERSE)
    string(SUBSTRING ${relative_path} 0 ${test} path)
    install(FILES ${v} DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/../../include/${path})
endforeach()

install(TARGETS l${TargetName}
  DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/../../lib
)
