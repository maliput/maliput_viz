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

#include "maliput_viz/flags/gflags.h"

namespace maliput {
namespace viz {
namespace {

/// Constants.
constexpr char kVersionStr[] = "Maliput Viz 0.1.0";
constexpr char kMaliputVizLayoutPath[] = "layouts/maliput_viz_layout.config";
constexpr char kMaliputVizResourceRootEnv[] = "MALIPUT_VIZ_RESOURCE_ROOT";
constexpr char kMaliputVizGuiPluginsEnv[] = "MALIPUT_VIZ_GUI_PLUGINS";

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

std::string GetMaliputVizConfigFile() {
  const std::list<std::string> paths = ignition::common::SystemPaths::PathsFromEnv(kMaliputVizResourceRootEnv);
  if (paths.size() == 0) {
    ignerr << kMaliputVizResourceRootEnv << " env var isn't set" << std::endl;
    return "";
  }
  if (paths.size() > 1) {
    ignwarn << "There are more than one path in " << kMaliputVizResourceRootEnv << " env var. " << std::endl;
  }
  return ignition::common::joinPaths(paths.front(), kMaliputVizLayoutPath);
}

/////////////////////////////////////////////////
int Main(int argc, char** argv) {
  flags::SetUsageMessage();
  flags::ParseCommandLineFlags(argc, argv, true);

  ignition::common::Console::SetVerbosity(flags::GetVerbosity());
  ignmsg << kVersionStr << std::endl;

  // Initialize app
  ignition::gui::Application app(argc, argv);

  // Set the default location for saving user settings.
  app.SetDefaultConfigPath(defaultConfigPath());

  // Look for custom plugins.
  app.SetPluginPathEnv(kVisualizerPluginPath);

  // Add paths for plugin discovery. The MaliputViewerPlugin is expected to be in.
  const std::list<std::string> paths = ignition::common::SystemPaths::PathsFromEnv(kMaliputVizGuiPluginsEnv);
  for (const auto& path : paths) {
    app.AddPluginPath(path);
  }

  const bool layout_loaded = app.LoadConfig(GetMaliputVizConfigFile());
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
