add_executable(maliput_viz
  maliput_viz.cc
)

target_link_libraries(maliput_viz
  ignition-common3::ignition-common3
  ignition-gui3::ignition-gui3
  ${Qt5Core_LIBRARIES}
  ${Qt5Widgets_LIBRARIES}
)

install(
  TARGETS maliput_viz
  EXPORT ${PROJECT_NAME}-targets
  DESTINATION bin
)
