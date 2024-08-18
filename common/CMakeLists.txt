file(GLOB SOURCE_FILES
  "${CMAKE_CURRENT_SOURCE_DIR}/*.cc"
)

list(FILTER SOURCE_FILES EXCLUDE REGEX ".*_test\\.cc$")

add_library(common SHARED
  ${SOURCE_FILES}
)

target_link_libraries(common
)

file(GLOB HEADER_FILES
  "${CMAKE_CURRENT_SOURCE_DIR}/*.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp"
)

include_directories(${HEADER_FILES})

foreach(v ${HEADER_FILES})
    string(REGEX MATCH "common/.*" relative_path ${v})
    string(FIND ${relative_path} "/" test REVERSE)
    string(SUBSTRING ${relative_path} 0 ${test} path)
    install(FILES ${v} DESTINATION include/common/${path})
    
endforeach()

target_include_directories(common PUBLIC 
  ${HEADER_FILES}
)

install(TARGETS common
  DESTINATION lib
)
