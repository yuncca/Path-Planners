file(GLOB SOURCE_FILES
  "${CMAKE_CURRENT_SOURCE_DIR}/*.cc"
  "${CMAKE_CURRENT_SOURCE_DIR}/**/*.cc"
)

list(FILTER SOURCE_FILES EXCLUDE REGEX ".*_test\\.cc$")

add_library(planners SHARED
  ${SOURCE_FILES}
)

include_directories(common)

target_link_libraries(planners PUBLIC
  common
)

file(GLOB HEADER_FILES
  "${CMAKE_CURRENT_SOURCE_DIR}/*.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/**/*.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/**/*.hpp"
)

foreach(v ${HEADER_FILES})
    string(REGEX MATCH "planners/.*" relative_path ${v})
    string(FIND ${relative_path} "/" test REVERSE)
    string(SUBSTRING ${relative_path} 0 ${test} path)
    # install(FILES ${v} DESTINATION include/planning/${path})
endforeach()

# install(TARGETS planners
#   DESTINATION lib
# )
