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
#include "traffic_light_manager.hh"

#include <ignition/common/Console.hh>
#include <ignition/common/MeshManager.hh>
#include <ignition/common/SystemPaths.hh>
#include <ignition/math/Color.hh>
#include <ignition/math/Helpers.hh>
#include <maliput/api/lane.h>
#include <maliput/api/lane_data.h>
#include <maliput/api/rules/phase.h>
#include <maliput/api/rules/traffic_lights.h>
#include <maliput/common/maliput_throw.h>

namespace maliput {
namespace viz {

const std::string TrafficLightManager::kGreenMaterialName{"GreenBulb"};
const std::string TrafficLightManager::kGreenBrightMaterialName{"GreenBulbBright"};
const std::string TrafficLightManager::kRedMaterialName{"RedBulb"};
const std::string TrafficLightManager::kRedBrightMaterialName{"RedBulbBright"};
const std::string TrafficLightManager::kYellowMaterialName{"YellowBulb"};
const std::string TrafficLightManager::kYellowBrightMaterialName{"YellowBulbBright"};
const std::string TrafficLightManager::kBulbSphereName{"BulbSphere"};
const std::string TrafficLightManager::kArrowBulbOBJFilePath{"resources/arrow_bulb.obj"};

TrafficLightManager::TrafficLightManager(ignition::rendering::ScenePtr _scene) : scene(_scene) {
  InitializeBulbMaterials();
  ignition::common::MeshManager* meshManager = ignition::common::MeshManager::Instance();
  MALIPUT_THROW_UNLESS(meshManager);
  const ignition::common::Mesh* unit_box_mesh = meshManager->MeshByName("unit_box");
  MALIPUT_THROW_UNLESS(unit_box_mesh);
  unitBoxAABBMin = unit_box_mesh->Min();
  unitBoxAABBMax = unit_box_mesh->Max();
  CreateRoundBulbMeshInManager();
  CreateArrowBulbMeshInManager();
}

void TrafficLightManager::CreateTrafficLights(
    const std::vector<const maliput::api::rules::TrafficLight*>& _trafficLights) {
  trafficLights.reserve(_trafficLights.size());
  for (const maliput::api::rules::TrafficLight* trafficLight : _trafficLights) {
    CreateSingleTrafficLight(trafficLight);
  }
}

void TrafficLightManager::Clear() {
  blinkingBulbs.clear();
  for (const auto& trafficLightMesh : trafficLights) {
    for (const auto& bulbGroup : trafficLightMesh.second.bulbGroups) {
      for (const auto& bulbMesh : bulbGroup.second.bulbs) {
        scene->RootVisual()->RemoveChild(bulbMesh.second);
      }
      scene->RootVisual()->RemoveChild(bulbGroup.second.visual);
    }
  }
  trafficLights.clear();
}

void TrafficLightManager::Tick() {
  for (const auto& bulb : blinkingBulbs) {
    const maliput::api::rules::BulbColor bulb_color = GetBulbColor(bulb.second);
    if (blinkTrafficLight) {
      bulb.second->SetMaterial(brightBulbMaterials[bulb_color], false);
    } else {
      bulb.second->SetMaterial(bulbMaterials[bulb_color], false);
    }
  }
  blinkTrafficLight = !blinkTrafficLight;
}

void TrafficLightManager::SetBulbStates(const maliput::api::rules::BulbStates& _bulbStates) {
  for (const auto& state : _bulbStates) {
    const maliput::api::rules::UniqueBulbId& unique_bulb_id = state.first;
    ignition::rendering::VisualPtr bulb_mesh = GetBulbMesh(unique_bulb_id);
    if (bulb_mesh) {
      const maliput::api::rules::BulbState& new_bulb_state = state.second;
      SetBulbMaterial(state.first, bulb_mesh, GetBulbColor(bulb_mesh), new_bulb_state);
    }
  }
}

ignition::rendering::VisualPtr TrafficLightManager::GetBulbMesh(
    const maliput::api::rules::UniqueBulbId& _uniqueBulbId) const {
  auto trafficLight = trafficLights.find(_uniqueBulbId.traffic_light_id());
  if (trafficLight != trafficLights.end()) {
    auto bulbGroup = trafficLight->second.bulbGroups.find(_uniqueBulbId.bulb_group_id());
    if (bulbGroup != trafficLight->second.bulbGroups.end()) {
      auto bulb = bulbGroup->second.bulbs.find(_uniqueBulbId.bulb_id());
      if (bulb != bulbGroup->second.bulbs.end()) {
        return bulb->second;
      }
    }
  }
  return ignition::rendering::VisualPtr();
}

maliput::api::rules::BulbColor TrafficLightManager::GetBulbColor(const ignition::rendering::VisualPtr& _bulb) const {
  maliput::api::rules::BulbColor color;
  const std::string& materialName = _bulb->Material()->Name();
  if (materialName.find(kGreenMaterialName) != std::string::npos) {
    color = maliput::api::rules::BulbColor::kGreen;
  } else if (materialName.find(kYellowMaterialName) != std::string::npos) {
    color = maliput::api::rules::BulbColor::kYellow;
  } else {
    color = maliput::api::rules::BulbColor::kRed;
  }
  return color;
}

void TrafficLightManager::InitializeBulbMaterials() {
  bulbMaterials[maliput::api::rules::BulbColor::kRed] = scene->CreateMaterial(kRedMaterialName);
  bulbMaterials[maliput::api::rules::BulbColor::kYellow] = scene->CreateMaterial(kYellowMaterialName);
  bulbMaterials[maliput::api::rules::BulbColor::kGreen] = scene->CreateMaterial(kGreenMaterialName);

  brightBulbMaterials[maliput::api::rules::BulbColor::kRed] = scene->CreateMaterial(kRedBrightMaterialName);
  brightBulbMaterials[maliput::api::rules::BulbColor::kYellow] = scene->CreateMaterial(kYellowBrightMaterialName);
  brightBulbMaterials[maliput::api::rules::BulbColor::kGreen] = scene->CreateMaterial(kGreenBrightMaterialName);

  ignition::rendering::MaterialPtr& redMaterial = GetRedMaterial();
  ignition::rendering::MaterialPtr& greenMaterial = GetGreenMaterial();
  ignition::rendering::MaterialPtr& yellowMaterial = GetYellowMaterial();

  ignition::rendering::MaterialPtr& redBrightMaterial = GetBrightRedMaterial();
  ignition::rendering::MaterialPtr& greenBrightMaterial = GetBrightGreenMaterial();
  ignition::rendering::MaterialPtr& yellowBrightMaterial = GetBrightYellowMaterial();

  redMaterial->SetDiffuse(60.0, 0.0, 0.0, 1.0);
  redMaterial->SetAmbient(60.0, 0.0, 0.0, 1.0);
  redBrightMaterial->SetDiffuse(255.0, 0.0, 0.0, 1.0);
  redBrightMaterial->SetAmbient(255.0, 0.0, 0.0, 1.0);

  greenMaterial->SetDiffuse(0.0, 60.0, 0.0, 1.0);
  greenMaterial->SetAmbient(0.0, 60.0, 0.0, 1.0);
  greenBrightMaterial->SetDiffuse(0.0, 255.0, 0.0, 1.0);
  greenBrightMaterial->SetAmbient(0.0, 255.0, 0.0, 1.0);

  yellowMaterial->SetDiffuse(60.0, 60.0, 0.0, 1.0);
  yellowMaterial->SetAmbient(60.0, 60.0, 0.0, 1.0);
  yellowBrightMaterial->SetDiffuse(255.0, 255.0, 0.0, 1.0);
  yellowBrightMaterial->SetAmbient(255.0, 255.0, 0.0, 1.0);
}

void TrafficLightManager::CreateRoundBulbMeshInManager() {
  ignition::common::MeshManager* meshManager = ignition::common::MeshManager::Instance();
  MALIPUT_THROW_UNLESS(meshManager);
  meshManager->CreateSphere(kBulbSphereName, 1.0f, 32, 32);
  const ignition::common::Mesh* bulbSphere = meshManager->MeshByName(kBulbSphereName);
  MALIPUT_THROW_UNLESS(bulbSphere);
  sphereBulbAABBMax = bulbSphere->Max();
  sphereBulbAABBMin = bulbSphere->Min();
}

void TrafficLightManager::CreateArrowBulbMeshInManager() {
  const std::list<std::string> paths = ignition::common::SystemPaths::PathsFromEnv("MALIPUT_VIZ_RESOURCE_ROOT");
  MALIPUT_VALIDATE(!paths.empty(),
                   "MALIPUT_VIZ_RESOURCE_ROOT environment "
                   "variable is not set");
  const std::vector<std::string> resource_paths(paths.begin(), paths.end());
  arrowName = ignition::common::SystemPaths::LocateLocalFile(kArrowBulbOBJFilePath, resource_paths);
  MALIPUT_THROW_UNLESS(!arrowName.empty());

  ignition::common::MeshManager* meshManager = ignition::common::MeshManager::Instance();
  MALIPUT_THROW_UNLESS(meshManager);
  const ignition::common::Mesh* arrow = meshManager->Load(arrowName);
  MALIPUT_THROW_UNLESS(arrow);
  arrowBulbAABBMax = arrow->Max();
  arrowBulbAABBMin = arrow->Min();
}

void TrafficLightManager::SetBulbMaterial(const maliput::api::rules::UniqueBulbId& _uniqueBulbId,
                                          ignition::rendering::VisualPtr& _bulb, maliput::api::rules::BulbColor _color,
                                          maliput::api::rules::BulbState _newBulbState) {
  switch (_newBulbState) {
    case maliput::api::rules::BulbState::kOff:
      _bulb->SetMaterial(bulbMaterials[_color], false);
      RemoveBlinkingLight(_uniqueBulbId);
      break;
    case maliput::api::rules::BulbState::kOn:
      _bulb->SetMaterial(brightBulbMaterials[_color], false);
      RemoveBlinkingLight(_uniqueBulbId);
      break;
    case maliput::api::rules::BulbState::kBlinking:
      blinkingBulbs[_uniqueBulbId] = _bulb;
      break;
    default:
      MALIPUT_VALIDATE(false, "Bulb state not supported");
      break;
  }
}

void TrafficLightManager::RemoveBlinkingLight(const maliput::api::rules::UniqueBulbId& _uniqueBulbId) {
  if (blinkingBulbs.find(_uniqueBulbId) != blinkingBulbs.end()) {
    blinkingBulbs.erase(_uniqueBulbId);
  }
}

void TrafficLightManager::CreateSingleTrafficLight(const maliput::api::rules::TrafficLight* _trafficLight) {
  const maliput::api::InertialPosition& traffic_light_world_position = _trafficLight->position_road_network();
  const maliput::api::Rotation& traffic_light_world_rotation = _trafficLight->orientation_road_network();

  TrafficLightManager::TrafficLightMesh traffic_light_mesh;
  traffic_light_mesh.bulbGroups.reserve(_trafficLight->bulb_groups().size());

  for (const maliput::api::rules::BulbGroup* bulb_group : _trafficLight->bulb_groups()) {
    CreateBulbGroup(bulb_group, traffic_light_world_position, traffic_light_world_rotation, &traffic_light_mesh);
  }

  trafficLights[_trafficLight->id()] = std::move(traffic_light_mesh);
}

void TrafficLightManager::CreateBulbGroup(const maliput::api::rules::BulbGroup* _bulbGroup,
                                          const maliput::api::InertialPosition& _trafficLightWorldPosition,
                                          const maliput::api::Rotation& _trafficLightWorldRotation,
                                          TrafficLightManager::TrafficLightMesh* _trafficLightMesh) {
  const maliput::api::InertialPosition bulb_group_world_position =
      _trafficLightWorldPosition + _bulbGroup->position_traffic_light();
  const maliput::api::Rotation bulb_group_world_rotation = maliput::api::Rotation::FromQuat(
      _trafficLightWorldRotation.quat() * (_bulbGroup->orientation_traffic_light().quat()));

  TrafficLightManager::BulbMeshes bulb_meshes;
  bulb_meshes.bulbs.reserve(_bulbGroup->bulbs().size());
  bulb_meshes.visual = scene->CreateVisual();
  bulb_meshes.visual->AddGeometry(scene->CreateBox());
  scene->RootVisual()->AddChild(bulb_meshes.visual);

  ignition::math::Vector3d box_aabb_min =
      unitBoxAABBMin + ignition::math::Vector3d(bulb_group_world_position.x(), bulb_group_world_position.y(),
                                                bulb_group_world_position.z());
  ignition::math::Vector3d box_aabb_max =
      unitBoxAABBMax + ignition::math::Vector3d(bulb_group_world_position.x(), bulb_group_world_position.y(),
                                                bulb_group_world_position.z());

  ignition::math::Vector3d bulb_group_aabb_max(std::numeric_limits<double>::lowest(),
                                               std::numeric_limits<double>::lowest(),
                                               std::numeric_limits<double>::lowest());
  ignition::math::Vector3d bulb_group_aabb_min(std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
                                               std::numeric_limits<double>::max());

  for (const maliput::api::rules::Bulb* bulb : _bulbGroup->bulbs()) {
    const maliput::api::rules::UniqueBulbId unique_bulb_id = bulb->unique_id();
    const maliput::api::rules::Bulb::BoundingBox bulb_bb_world_pos =
        CreateSingleBulb(unique_bulb_id, bulb, bulb_group_world_position, bulb_group_world_rotation, &bulb_meshes);
    bulb_group_aabb_max = ignition::math::Vector3d(std::max(bulb_group_aabb_max.X(), bulb_bb_world_pos.p_BMax.x()),
                                                   std::max(bulb_group_aabb_max.Y(), bulb_bb_world_pos.p_BMax.y()),
                                                   std::max(bulb_group_aabb_max.Z(), bulb_bb_world_pos.p_BMax.z()));
    bulb_group_aabb_min = ignition::math::Vector3d(std::min(bulb_group_aabb_min.X(), bulb_bb_world_pos.p_BMin.x()),
                                                   std::min(bulb_group_aabb_min.Y(), bulb_bb_world_pos.p_BMin.y()),
                                                   std::min(bulb_group_aabb_min.Z(), bulb_bb_world_pos.p_BMin.z()));
  }

  const ignition::math::Vector3d mid_point = (bulb_group_aabb_max + bulb_group_aabb_min) / 2.0;

  bulb_meshes.visual->SetWorldScale(2.0 * std::abs(bulb_group_aabb_max.X() - mid_point.X()),
                                    2.0 * std::abs(bulb_group_aabb_max.Y() - mid_point.Y()),
                                    2.0 * std::abs(bulb_group_aabb_max.Z() - mid_point.Z()));
  bulb_meshes.visual->SetWorldRotation(bulb_group_world_rotation.roll(), bulb_group_world_rotation.pitch(),
                                       bulb_group_world_rotation.yaw());
  bulb_meshes.visual->SetWorldPosition(mid_point);
  bulb_meshes.visual->SetVisible(false);
  _trafficLightMesh->bulbGroups[_bulbGroup->id()] = std::move(bulb_meshes);
}

maliput::api::rules::Bulb::BoundingBox TrafficLightManager::CreateSingleBulb(
    const maliput::api::rules::UniqueBulbId& _uniqueBulbId, const maliput::api::rules::Bulb* _single_bulb,
    const maliput::api::InertialPosition& _bulbGroupWorldPosition,
    const maliput::api::Rotation& _bulbGroupWorldRotation, BulbMeshes* _bulbGroup) {
  const maliput::api::rules::Bulb::BoundingBox& bb = _single_bulb->bounding_box();
  // Bulb's bounding box is in terms of 1 meter per unit coordinate. We consider that this bounding box is
  // symmetric and will be used as a scale vector to set the proper size of the bulb in the visualizer.
  MALIPUT_THROW_UNLESS(bb.p_BMax == (-1.0 * bb.p_BMin));

  // The visual used has a bounding box size of 1x1x1 meter.
  // Considering that the bulb's bounding box is symmetric and expressed in function of this size, it can be used
  // for a scale operation.
  const maliput::math::Vector3& min_scale{
      bb.p_BMin.x(),
      bb.p_BMin.y(),
      bb.p_BMin.z(),
  };
  const maliput::math::Vector3& max_scale{
      bb.p_BMax.x(),
      bb.p_BMax.y(),
      bb.p_BMax.z(),
  };

  ignition::math::Vector3d world_bounding_box_max;
  ignition::math::Vector3d world_bounding_box_min;

  maliput::api::Rotation bulb_rotation =
      maliput::api::Rotation::FromQuat(_bulbGroupWorldRotation.quat() * _single_bulb->orientation_bulb_group().quat());
  ignition::rendering::VisualPtr visual = scene->CreateVisual();
  if (_single_bulb->type() == maliput::api::rules::BulbType::kRound) {
    world_bounding_box_max = sphereBulbAABBMax;
    world_bounding_box_min = sphereBulbAABBMin;
    visual->AddGeometry(scene->CreateMesh(kBulbSphereName));
  } else {
    world_bounding_box_max = arrowBulbAABBMax;
    world_bounding_box_min = arrowBulbAABBMin;
    bulb_rotation = maliput::api::Rotation::FromQuat(
        bulb_rotation.quat() *
        maliput::api::Rotation::FromRpy({_single_bulb->arrow_orientation_rad().value(), 0.0, 0.0}).quat());
    visual->AddGeometry(scene->CreateMesh(arrowName));
  }

  maliput::api::rules::Bulb::BoundingBox bulb_world_bounding_box;
  bulb_world_bounding_box.p_BMin.x() = world_bounding_box_min.X() * std::abs(min_scale.x());
  bulb_world_bounding_box.p_BMin.y() = world_bounding_box_min.Y() * std::abs(min_scale.y());
  bulb_world_bounding_box.p_BMin.z() = world_bounding_box_min.Z() * std::abs(min_scale.z());
  bulb_world_bounding_box.p_BMax.x() = world_bounding_box_max.X() * std::abs(max_scale.x());
  bulb_world_bounding_box.p_BMax.y() = world_bounding_box_max.Y() * std::abs(max_scale.y());
  bulb_world_bounding_box.p_BMax.z() = world_bounding_box_max.Z() * std::abs(max_scale.z());

  visual->SetWorldScale(max_scale.x(), max_scale.y(), max_scale.z());
  const maliput::api::InertialPosition bulb_world_position =
      _bulbGroupWorldPosition + _single_bulb->position_bulb_group();
  visual->SetWorldRotation(bulb_rotation.roll(), bulb_rotation.pitch(), bulb_rotation.yaw());
  visual->SetWorldPosition(bulb_world_position.x(), bulb_world_position.y(), bulb_world_position.z());
  SetBulbMaterial(_uniqueBulbId, visual, _single_bulb->color(), _single_bulb->GetDefaultState());
  scene->RootVisual()->AddChild(visual);
  _bulbGroup->bulbs[_single_bulb->id()] = visual;

  bulb_world_bounding_box.p_BMax += bulb_world_position.xyz();
  bulb_world_bounding_box.p_BMin += bulb_world_position.xyz();

  return bulb_world_bounding_box;
}

}  // namespace viz
}  // namespace maliput
