add_library(tools
  yaml_parser.cc
)

add_library(maliput_viz::tools ALIAS tools)
set_target_properties(tools
  PROPERTIES
    OUTPUT_NAME tools
)

target_link_libraries(tools
  maliput::common
  yaml-cpp
)

target_include_directories(tools
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
)

install(
  TARGETS tools
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
