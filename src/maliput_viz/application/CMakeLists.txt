add_executable(maliput_viz
  maliput_viz.cc
)

target_link_libraries(maliput_viz
  ignition-common3::ignition-common3
  ignition-gui3::ignition-gui3
  ${Qt5Core_LIBRARIES}
  ${Qt5Widgets_LIBRARIES}
)

target_compile_definitions(maliput_viz
  PRIVATE
    MALIPUT_VIZ_PLUGIN_PATH="${CMAKE_INSTALL_PREFIX}/lib/gui_plugins"
    MALIPUT_VIZ_INITIAL_CONFIG_PATH="${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_DATAROOTDIR}/maliput_viz/layouts"
)

install(
  TARGETS maliput_viz
  EXPORT ${PROJECT_NAME}-targets
  DESTINATION bin
)
