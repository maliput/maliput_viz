// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet.
// All rights reserved.
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
#include "maliput_viz/flags/gflags.h"

#include <string>

#include <gflags/gflags.h>
#include <maliput/common/filesystem.h>
#include <maliput/common/logger.h>

// Gflags
DEFINE_int32(verbosity, 3, "Verbosity is set within a range between [0-4]");
DEFINE_string(yaml_file_path, "", "Path to the configuration YAML file");

// Validators
static bool ValidateVerbosity(const char* flagname, int32_t value) { return value >= 0 && value <= 4; }
static bool ValidateConfigFilePath(const char* flagname, const std::string& value) {
  if (!value.empty()) {
    maliput::common::Path path{value};
    if (!path.exists() || !path.is_file()) {
      maliput::log()->error("Please verify that '{}' is a valid file path.", value);
      return false;
    }
  }
  return true;
}

DEFINE_validator(verbosity, &ValidateVerbosity);
DEFINE_validator(yaml_file_path, &ValidateConfigFilePath);

namespace maliput {
namespace viz {
namespace flags {
namespace {

constexpr const char* kUsageMessage =
    R"R(Application for maliput road network visualization.

  Usage: maliput_viz [--<flag_1>] [--<flag_2>] .. [--<flag_n>]

  * Summary:
     This application offers a visualizer for the maliput road networks.
     It provides a GUI for interactively select a maliput backend and load the road network
     passing the correspondent parameters.
     In addition, a YAML configuration file could be passed via command line for setting up the visualizer beforehand.

  * Examples of use:
     1. Running the visualizer:
        $ maliput_viz
     2. Running the visualizer with a higher verbosity level:
        $ maliput_viz --verbosity=4
     3. Running the visualizer with a configuration file:
        $ maliput_viz --yaml_file_path=<path-to-yaml-file>

  * YAML file configuration:
     The --yaml_file_path flag expects a valid path to a YAML file.
     The YAML file should contain the configuration for loading the road network.
     The structure of the YAML file should be:
      |  maliput_viz:
      |    maliput_backend: <backend_name>
      |    parameters:
      |       <key_1>: <value_1>
      |       <key_2>: <value_2>
      |       ...
      |       <key_N>: <value_N>
)R";

}  // namespace

void ParseCommandLineFlags(int argc, char** argv, bool remove_flags) {
  gflags::ParseCommandLineFlags(&argc, &argv, remove_flags);
}

void SetUsageMessage() { gflags::SetUsageMessage(static_cast<std::string>(kUsageMessage)); }

int GetVerbosity() { return FLAGS_verbosity; }

std::string GetYamlFilePath() { return FLAGS_yaml_file_path; }

}  // namespace flags
}  // namespace viz
}  // namespace maliput
