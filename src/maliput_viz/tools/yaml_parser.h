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

#pragma once

#include <map>
#include <string>

namespace maliput {
namespace viz {
namespace tools {

/// Parser for a YAML file to be used by the maliput_viz application.
/// The YAML file must have the following structure:
/// @code {.yaml}
/// maliput_viz:"
///   maliput_backend: <backend_name> "
///   parameters: "
///      <key_1>: <value_1> "
///      <key_2>: <value_2> "
///      // ...
///      <key_N>: <value_N> "
/// @endcode
class YamlConfigFileParser {
 public:
  /// @brief  Constructor
  /// @param file_path Path to a YAML file.
  YamlConfigFileParser(const std::string& file_path);
  ~YamlConfigFileParser() = default;

  /// @returns The parsed backend name.
  std::string GetBackendName() const;
  /// @returns The parsed parameters
  std::map<std::string, std::string> GetBackendParameters() const;

 private:
  static constexpr char kMaliputVizKey[] = "maliput_viz";
  static constexpr char kMaliputBackendKey[] = "maliput_backend";
  static constexpr char kParametersKey[] = "parameters";
  std::string backend_name_;
  std::map<std::string, std::string> parameters_;
};

}  // namespace tools
}  // namespace viz
}  // namespace maliput
