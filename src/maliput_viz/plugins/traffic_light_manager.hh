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

#ifndef TRAFFIC_LIGHT_MANAGER_HH
#define TRAFFIC_LIGHT_MANAGER_HH

#include <unordered_map>
#include <vector>

#include <ignition/math/Vector3.hh>
#include <ignition/rendering/Scene.hh>
#include <maliput/api/lane.h>
#include <maliput/api/lane_data.h>
#include <maliput/api/rules/phase.h>
#include <maliput/api/rules/traffic_lights.h>

namespace maliput {
namespace viz {

/// \brief Class that creates a representation for traffic light and controls them.
class TrafficLightManager final {
 public:
  TrafficLightManager(ignition::rendering::ScenePtr _scene);
  /// \brief Destructor. All mesh destruction will be in charge of the scene's destructor.
  ~TrafficLightManager() = default;

  /// \brief Renders all bulbs and box covering them passed by parameter with their corresponding colors
  /// \param[in] _trafficLights Vector containing all the required traffic lights to render.
  void CreateTrafficLights(const std::vector<const maliput::api::rules::TrafficLight*>& _trafficLights);

  /// \brief Destroy all meshes created by the manager explictly.
  void Clear();

  /// \brief Change the material of the bulbs when a blink ocurrs.
  void Tick();

  /// \brief Set the state of all the bulbs.
  /// \param[in] _bulbStates Unordered map containing the new state of each bulb.
  void SetBulbStates(const maliput::api::rules::BulbStates& _bulbStates);

 private:
  /// \brief Name of the material used for the green bulb turned off.
  static const std::string kGreenMaterialName;
  /// \brief Name of the material used for the green bulb turned on.
  static const std::string kGreenBrightMaterialName;
  /// \brief Name of the material used for the red bulb turned off.
  static const std::string kRedMaterialName;
  /// \brief Name of the material used for the red bulb turned on.
  static const std::string kRedBrightMaterialName;
  /// \brief Name of the material used for the yellow bulb turned off.
  static const std::string kYellowMaterialName;
  /// \brief Name of the material used for the yellow bulb turned on.
  static const std::string kYellowBrightMaterialName;
  /// \brief Relative path to the arrow obj file.
  static const std::string kArrowBulbOBJFilePath;
  /// \brief Name of the radius 1 sphere used for the round bulbs.
  static const std::string kBulbSphereName;

  /// \brief Structure that contains the mesh related with the bulb group and connects it to a set of bulbs
  struct BulbMeshes {
    ignition::rendering::VisualPtr visual;
    std::unordered_map<maliput::api::rules::Bulb::Id, ignition::rendering::VisualPtr> bulbs;
  };

  /// \brief Structure that contains the mesh related with the traffic light (unused at the moment) and connects
  /// it to a set of bulb groups.
  struct TrafficLightMesh {
    ignition::rendering::VisualPtr visual;
    std::unordered_map<maliput::api::rules::BulbGroup::Id, BulbMeshes> bulbGroups;
  };

  ignition::rendering::MaterialPtr& GetRedMaterial() { return bulbMaterials[maliput::api::rules::BulbColor::kRed]; }
  ignition::rendering::MaterialPtr& GetGreenMaterial() { return bulbMaterials[maliput::api::rules::BulbColor::kGreen]; }
  ignition::rendering::MaterialPtr& GetYellowMaterial() {
    return bulbMaterials[maliput::api::rules::BulbColor::kYellow];
  }

  ignition::rendering::MaterialPtr& GetBrightRedMaterial() {
    return brightBulbMaterials[maliput::api::rules::BulbColor::kRed];
  }
  ignition::rendering::MaterialPtr& GetBrightGreenMaterial() {
    return brightBulbMaterials[maliput::api::rules::BulbColor::kGreen];
  }
  ignition::rendering::MaterialPtr& GetBrightYellowMaterial() {
    return brightBulbMaterials[maliput::api::rules::BulbColor::kYellow];
  }

  /// \brief Gets the bulb mesh related with a given id.
  /// \param[in] _uniqueBulbId Unique id of a bulb.
  /// \returns A pointer that contains the bulb's mesh or nullptr if not found.
  ignition::rendering::VisualPtr GetBulbMesh(const maliput::api::rules::UniqueBulbId& _uniqueBulbId) const;

  /// \brief Gets the color of the bulb based on its material name.
  /// \param[in] bulb Pointer containing the bulb's mesh.
  /// \returns The color of a given bulb.
  maliput::api::rules::BulbColor GetBulbColor(const ignition::rendering::VisualPtr& _bulb) const;

  /// \brief Initializes the bulb materials (red, yellow and green along with their brighter versions). This is called
  /// just once per instance.
  void InitializeBulbMaterials();

  /// \brief Creates a sphere of radius 1 in ignition common's mesh manager to use it as a bulb.
  /// This is called just once.
  void CreateRoundBulbMeshInManager();
  /// \brief Loads the arrow obj living in resources directory within the current dir in ignition common's mesh manager.
  /// This is called just once.
  void CreateArrowBulbMeshInManager();

  /// \brief Sets the proper material for a bulb for a new state.
  /// \param[in] _uniqueBulbId Unique id of a bulb.
  /// \param[in] _bulb Pointer containing the mesh of a given bulb.
  /// \param[in] _color Color used by the bulb.
  /// \param[in] _newBulbState The new state the bulb will be.
  void SetBulbMaterial(const maliput::api::rules::UniqueBulbId& _uniqueBulbId, ignition::rendering::VisualPtr& _bulb,
                       maliput::api::rules::BulbColor _color, maliput::api::rules::BulbState _newBulbState);

  /// \brief Removes a given bulb from a list of blinking bulbs to not tick them anymore.
  /// \param[in] _uniqueBulbId Bulb's unique id.
  void RemoveBlinkingLight(const maliput::api::rules::UniqueBulbId& _uniqueBulbId);

  /// \brief Create the mesh for a given traffic light.
  /// \param[in] _trafficLight Traffic light that will be rendered.
  void CreateSingleTrafficLight(const maliput::api::rules::TrafficLight* _trafficLight);

  /// \brief Create the mesh for a given bulb group.
  /// \param[in] _bulbGroup Bulb group information to render the mesh.
  /// \param[in] _trafficLightWorldPosition World position of the traffic light where this group lives in.
  /// \param[in] _trafficLightWorldRotation World rotation of the traffic light where this group lives in.
  /// \param[out] _trafficLightMesh Structure that holds the visual of the traffic light and the unordered map of bulb
  /// groups to be part of.
  void CreateBulbGroup(const maliput::api::rules::BulbGroup* _bulbGroup,
                       const maliput::api::InertialPosition& _trafficLightWorldPosition,
                       const maliput::api::Rotation& _trafficLightWorldRotation, TrafficLightMesh* _trafficLightMesh);

  /// \brief Create the mesh for a given bulb that is within a bulb group.
  /// \param[in] _uniqueBulbId Unique id that belongs to the bulb to be rendered.
  /// \param[in] _singleBulb Bulb information to render the mesh.
  /// \param[in] _bulbGroupWorldPosition World position of the bulb group where this bulb lives in.
  /// \param[in] _bulbGroupWorldRotation World rotation of the bulb group where this bulb lives in.
  /// \param[out] _bulbGroup Group from where this bulb will live in.
  maliput::api::rules::Bulb::BoundingBox CreateSingleBulb(const maliput::api::rules::UniqueBulbId& _uniqueBulbId,
                                                          const maliput::api::rules::Bulb* _singleBulb,
                                                          const maliput::api::InertialPosition& _bulbGroupWorldPosition,
                                                          const maliput::api::Rotation& _bulbGroupWorldRotation,
                                                          BulbMeshes* _bulbGroup);

  /// \brief Unordered map containing the green, yellow and red material for the turned off bulb state.
  std::unordered_map<maliput::api::rules::BulbColor, ignition::rendering::MaterialPtr> bulbMaterials;
  /// \brief Unordered map containing the green, yellow and red material for the turned on bulb state.
  std::unordered_map<maliput::api::rules::BulbColor, ignition::rendering::MaterialPtr> brightBulbMaterials;
  /// \brief Unordered map containing all the traffic light visual hierarchy.
  std::unordered_map<maliput::api::rules::TrafficLight::Id, TrafficLightMesh> trafficLights;
  /// \brief Unordered map containing all the bulbs that are in the blinking state to facilitate the change
  /// of materials.
  std::unordered_map<maliput::api::rules::UniqueBulbId, ignition::rendering::VisualPtr> blinkingBulbs;
  /// \brief Full path of the arrow obj file so it can be used to create instances of it.
  std::string arrowName;
  /// \brief Pointer to the scene where the traffic lights will be rendered.
  ignition::rendering::ScenePtr scene;
  /// \brief Min point of the sphere's bounding box used for the bulbs.
  ignition::math::Vector3d sphereBulbAABBMax;
  /// \brief Max point of the sphere's bounding box used for the bulbs.
  ignition::math::Vector3d sphereBulbAABBMin;
  /// \brief Min point of the arrow's bounding box used for the bulbs.
  ignition::math::Vector3d arrowBulbAABBMax;
  /// \brief Max point of the arrow's bounding box used for the bulbs.
  ignition::math::Vector3d arrowBulbAABBMin;
  /// \brief Min point of the bulb group's bounding box.
  ignition::math::Vector3d unitBoxAABBMin;
  /// \brief Max point of the bulb group's bounding box.
  ignition::math::Vector3d unitBoxAABBMax;
  /// \brief Used to change the blinking bulb's state.
  bool blinkTrafficLight{false};
};
}  // namespace viz
}  // namespace maliput

#endif  // TRAFFIC_LIGHT_MANAGER_HH
