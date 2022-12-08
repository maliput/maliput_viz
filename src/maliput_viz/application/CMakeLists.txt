add_executable(maliput_viz
  maliput_viz.cc
)

target_link_libraries(maliput_viz
  ignition-common3::ignition-common3
  ignition-gui3::ignition-gui3
  maliput_viz::flags
  ${Qt5Core_LIBRARIES}
  ${Qt5Widgets_LIBRARIES}
)

target_include_directories(maliput_viz
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
)

install(
  TARGETS maliput_viz
  EXPORT ${PROJECT_NAME}-targets
  DESTINATION bin
)
