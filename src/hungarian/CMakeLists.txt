set(SUBSYS_NAME hungarian)

set(HUNGARIAN_SOURCE_DIR ${PROJECT_SOURCE_DIR}/third-party/hungarian-v2.0)

include_directories(${CMAKE_SOURCE_DIR}/include/${SUBSYS_NAME})
include_directories(${HUNGARIAN_SOURCE_DIR})

add_library(hungarian
  hungarian.cpp
  ${HUNGARIAN_SOURCE_DIR}/Assignment.cpp
  ${HUNGARIAN_SOURCE_DIR}/BipartiteGraph.cpp
  ${HUNGARIAN_SOURCE_DIR}/Hungarian.cpp
  ${HUNGARIAN_SOURCE_DIR}/PlotGraph.cpp
  ${HUNGARIAN_SOURCE_DIR}/plot/Cgnuplot.cpp
  ${HUNGARIAN_SOURCE_DIR}/plot/graph_hungarian.cpp
)

set_target_properties(hungarian
  PROPERTIES COMPILE_FLAGS -std=c++98
)

SET_SOURCE_FILES_PROPERTIES(
  hungarian.cpp
  PROPERTIES COMPILE_FLAGS -std=gnu++11
)
