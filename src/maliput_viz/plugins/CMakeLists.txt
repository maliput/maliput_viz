include_directories(
  ${Qt5Core_INCLUDE_DIRS}
)

# ----------------------------------------
# maliput_viewer_model library.
add_library(maliput_viewer_model
  maliput_viewer_model.cc
  maliput_mesh_converter.cc
)
add_library(maliput_viz::maliput_viewer_model ALIAS maliput_viewer_model)
set_target_properties(maliput_viewer_model
  PROPERTIES
    OUTPUT_NAME maliput_viz_maliput_viewer_model
)

target_link_libraries(maliput_viewer_model
  ${drake_LIBRARIES}
  ignition-common3::ignition-common3
  ignition-rendering3::ignition-rendering3
  maliput::api
  maliput::utility
)

install(
  TARGETS maliput_viewer_model
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

# ----------------------------------------
# arrow_mesh library.
add_library(arrow_mesh
  arrow_mesh.cc
)
add_library(maliput_viz::arrow_mesh ALIAS arrow_mesh)
set_target_properties(arrow_mesh
  PROPERTIES
    OUTPUT_NAME maliput_viz_arrow_mesh
)

target_link_libraries(arrow_mesh
  ${drake_LIBRARIES}
  ignition-common3::ignition-common3
  ignition-math6::ignition-math6
  ignition-rendering3::ignition-rendering3
)

install(
  TARGETS arrow_mesh
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

# ----------------------------------------
# selector library.
add_library(selector
  selector.cc
)
add_library(maliput_viz::selector ALIAS selector)
set_target_properties(selector
  PROPERTIES
    OUTPUT_NAME maliput_viz_selector
)

target_link_libraries(selector
  ignition-common3::ignition-common3
  ignition-math6::ignition-math6
  ignition-rendering3::ignition-rendering3
  maliput::api
)

install(
  TARGETS selector
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

# ----------------------------------------
# traffic light manager library.
add_library(traffic_light_manager
  traffic_light_manager.cc
)
add_library(maliput_viz::traffic_light_manager ALIAS traffic_light_manager)
set_target_properties(traffic_light_manager
  PROPERTIES
    OUTPUT_NAME maliput_viz_traffic_light_manager
)

target_link_libraries(traffic_light_manager
  ignition-common3::ignition-common3
  ignition-math6::ignition-math6
  ignition-rendering3::ignition-rendering3
  maliput::api
)

install(
  TARGETS traffic_light_manager
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

#-------------------------------------------------------------------------------
# maliput_backend_selection (Qt Object)
QT5_WRAP_CPP(maliput_backend_selection_headers_MOC maliput_backend_selection.hh)

add_library(maliput_backend_selection
  ${CMAKE_CURRENT_SOURCE_DIR}/maliput_backend_selection.cc
  ${maliput_backend_selection_headers_MOC}
  ${maliput_backend_selection_RCC}
)
add_library(maliput_viz::maliput_backend_selection ALIAS maliput_backend_selection)
set_target_properties(maliput_backend_selection
  PROPERTIES
    OUTPUT_NAME maliput_backend_selection
)
target_link_libraries(maliput_backend_selection
    maliput_viz::maliput_viewer_model
    maliput::plugin
    ignition-gui3::ignition-gui3
    ignition-common3::ignition-common3
    ignition-transport8::ignition-transport8
    ignition-rendering3::ignition-rendering3
    ${Qt5Core_LIBRARIES}
    ${Qt5Widgets_LIBRARIES}
)

install(
  TARGETS maliput_backend_selection
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

#-------------------------------------------------------------------------------
# MaliputViewerPlugin (ign-gui 3)
QT5_WRAP_CPP(MaliputViewerPlugin_headers_MOC maliput_viewer_plugin.hh)
QT5_ADD_RESOURCES(MaliputViewerPlugin_RCC maliput_viewer_plugin.qrc)

add_library(MaliputViewerPlugin
  ${CMAKE_CURRENT_SOURCE_DIR}/maliput_viewer_plugin.cc
  ${maliput_backend_selection_headers_MOC}
  ${MaliputViewerPlugin_headers_MOC}
  ${MaliputViewerPlugin_RCC}
)
add_library(maliput_viz::MaliputViewerPlugin ALIAS MaliputViewerPlugin)
set_target_properties(MaliputViewerPlugin
  PROPERTIES
    OUTPUT_NAME MaliputViewerPlugin
)

target_link_libraries(MaliputViewerPlugin
    maliput_viz::arrow_mesh
    maliput_viz::flags
    maliput_viz::maliput_backend_selection
    maliput_viz::maliput_viewer_model
    maliput_viz::selector
    maliput_viz::tools
    maliput_viz::traffic_light_manager
    ignition-gui3::ignition-gui3
    ignition-common3::ignition-common3
    ignition-transport8::ignition-transport8
    ignition-rendering3::ignition-rendering3
    ${Qt5Core_LIBRARIES}
    ${Qt5Widgets_LIBRARIES}
    ignition-plugin1::register
)

install(
  TARGETS MaliputViewerPlugin
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib/gui_plugins
  ARCHIVE DESTINATION lib/gui_plugins
)
