// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2019-2022, Toyota Research Institute. All rights reserved.
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

#include <string>
#include <unordered_map>
#include <vector>

#include <ignition/math/Vector3.hh>
#include <ignition/rendering/Scene.hh>
#include <maliput/api/rules/traffic_sign.h>

namespace maliput {
namespace viz {

/// \brief Class that creates and owns rendering visuals for traffic signs.
///
/// Each sign is represented as a single colored box matching the sign's bounding box.
/// The manager also supports proximity-based hit testing so that a scene click can be
/// resolved to the nearest traffic sign.
class TrafficSignManager final {
 public:
  explicit TrafficSignManager(ignition::rendering::ScenePtr _scene);
  /// \brief Destructor. Visual destruction is handled by the scene.
  ~TrafficSignManager() = default;

  /// \brief Creates rendering visuals for all provided traffic signs.
  /// \param[in] _signs Vector of traffic sign pointers to visualize.
  void CreateTrafficSigns(const std::vector<const maliput::api::rules::TrafficSign*>& _signs);

  /// \brief Removes and destroys all visuals owned by this manager.
  void Clear();

  /// \brief Returns the traffic sign closest to @p _point whose bounding sphere
  ///        contains @p _point (within @p _tolerance), or nullptr if none match.
  /// \param[in] _point World-space position to test.
  /// \param[in] _tolerance Extra distance added to each sign's bounding sphere radius.
  /// \returns Pointer to the matching TrafficSign, or nullptr.
  const maliput::api::rules::TrafficSign* GetTrafficSignAtPoint(const ignition::math::Vector3d& _point,
                                                                double _tolerance = 0.0) const;

 private:
  /// \brief Material name used for all sign visuals.
  static const std::string kSignMaterialName;

  /// \brief Holds per-sign rendering data used for hit testing.
  struct SignVisual {
    /// Ignition rendering visual for the bounding box.
    ignition::rendering::VisualPtr visual;
    /// World-space centroid of the sign's bounding box.
    ignition::math::Vector3d worldCenter;
    /// Half-diagonal of the bounding box used for bounding-sphere hit tests.
    double boundingSphereRadius{0.0};
    /// Non-owning pointer back to the source sign (owned by the TrafficSignBook).
    const maliput::api::rules::TrafficSign* sign{nullptr};
  };

  /// \brief Creates a single sign visual and registers it.
  /// \param[in] _sign Non-null pointer to a TrafficSign.
  void CreateSingleTrafficSign(const maliput::api::rules::TrafficSign* _sign);

  /// \brief Pointer to the ignition rendering scene.
  ignition::rendering::ScenePtr scene;
  /// \brief Shared material applied to every sign visual.
  ignition::rendering::MaterialPtr signMaterial;
  /// \brief Map from TrafficSign::Id to its rendering data.
  std::unordered_map<maliput::api::rules::TrafficSign::Id, SignVisual> signs_;
};

}  // namespace viz
}  // namespace maliput
