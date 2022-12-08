add_library(flags
  gflags.cc
)

add_library(maliput_viz::flags ALIAS flags)
set_target_properties(flags
  PROPERTIES
    OUTPUT_NAME maliput_viz_flags
)

target_link_libraries(flags
  gflags
  maliput::common
)

target_include_directories(flags
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
)

install(
  TARGETS flags
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
