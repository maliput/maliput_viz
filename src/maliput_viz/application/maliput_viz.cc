// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2021-2022, Toyota Research Institute. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <iostream>
#include <string>

#include <ignition/common/Console.hh>
#include <ignition/common/Filesystem.hh>

#ifndef Q_MOC_RUN
#include <ignition/gui/Application.hh>
#include <ignition/gui/MainWindow.hh>
#include <ignition/gui/qt.h>
#endif

namespace maliput {
namespace viz {
namespace {

/// Constants.
constexpr char kVersionStr[] = "Maliput Viz 0.1.0";
constexpr char kDefaultLayout[] = "maliput_viz_layout.config";

/// Environment variable to look for user custom plugins.
constexpr char kVisualizerPluginPath[] = "VISUALIZER_PLUGIN_PATH";

/////////////////////////////////////////////////
/// \brief Get the path of the default configuration file for Maliput Viz.
/// \return The default configuration path.
std::string defaultConfigPath() {
  std::string homePath;
  ignition::common::env("HOME", homePath);
  return ignition::common::joinPaths(homePath, ".maliput_viz", "maliput_viz.config");
}

/////////////////////////////////////////////////
int Main(int argc, char** argv) {
  static const std::string initialConfigFile =
      ignition::common::joinPaths(MALIPUT_VIZ_INITIAL_CONFIG_PATH, kDefaultLayout);

  ignition::common::Console::SetVerbosity(3);
  ignmsg << kVersionStr << std::endl;

  // Initialize app
  ignition::gui::Application app(argc, argv);

  // Set the default location for saving user settings.
  app.SetDefaultConfigPath(defaultConfigPath());

  // Look for custom plugins.
  app.SetPluginPathEnv(kVisualizerPluginPath);

  // Then look for plugins on compile-time defined path.
  // Plugins installed by delphyne_gui end up here
  app.AddPluginPath(MALIPUT_VIZ_PLUGIN_PATH);

  const bool layout_loaded = app.LoadConfig(initialConfigFile);
  // If no layout has been loaded, exit the application.
  if (!layout_loaded) {
    ignerr << "Unable to load the configuration file, exiting." << std::endl;
    return 1;
  }

  // Create main window
  auto win = app.findChild<ignition::gui::MainWindow*>()->QuickWindow();
  win->setProperty("title", kVersionStr);

  // Run window
  app.exec();

  return 0;
}

}  // namespace
}  // namespace viz
}  // namespace maliput

int main(int argc, char** argv) { return maliput::viz::Main(argc, argv); }
