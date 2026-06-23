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
#include "traffic_sign_manager.hh"

#include <cmath>
#include <limits>

#include <ignition/common/Console.hh>
#include <ignition/math/Quaternion.hh>
#include <maliput/common/maliput_throw.h>

namespace maliput {
namespace viz {

const std::string TrafficSignManager::kSignMaterialName{"TrafficSignBox"};

TrafficSignManager::TrafficSignManager(ignition::rendering::ScenePtr _scene) : scene(_scene) {
  MALIPUT_THROW_UNLESS(scene != nullptr);
  signMaterial = scene->CreateMaterial(kSignMaterialName);
  MALIPUT_THROW_UNLESS(signMaterial != nullptr);
  // Cyan-ish color with transparency so the bounding box is distinguishable but not occluding.
  signMaterial->SetDiffuse(0.0, 200.0, 200.0, 1.0);
  signMaterial->SetAmbient(0.0, 200.0, 200.0, 1.0);
  signMaterial->SetTransparency(0.5);
}

void TrafficSignManager::CreateTrafficSigns(const std::vector<const maliput::api::rules::TrafficSign*>& _signs) {
  signs_.reserve(_signs.size());
  for (const maliput::api::rules::TrafficSign* sign : _signs) {
    CreateSingleTrafficSign(sign);
  }
}

void TrafficSignManager::Clear() {
  for (const auto& entry : signs_) {
    scene->RootVisual()->RemoveChild(entry.second.visual);
  }
  signs_.clear();
}

const maliput::api::rules::TrafficSign* TrafficSignManager::GetTrafficSignAtPoint(
    const ignition::math::Vector3d& _point, double _tolerance) const {
  const maliput::api::rules::TrafficSign* closest{nullptr};
  double closestDist{std::numeric_limits<double>::max()};

  for (const auto& entry : signs_) {
    const double dist = (entry.second.worldCenter - _point).Length();
    if (dist <= entry.second.boundingSphereRadius + _tolerance && dist < closestDist) {
      closestDist = dist;
      closest = entry.second.sign;
    }
  }
  return closest;
}

void TrafficSignManager::CreateSingleTrafficSign(const maliput::api::rules::TrafficSign* _sign) {
  MALIPUT_THROW_UNLESS(_sign != nullptr);

  const maliput::api::InertialPosition& world_pos = _sign->position_road_network();
  const maliput::api::Rotation& world_rot = _sign->orientation_road_network();
  const maliput::math::BoundingBox& bb = _sign->bounding_box();

  // The bounding box position is expressed in the sign's local frame.
  // Rotate it to world frame using the sign's orientation quaternion.
  const auto& maliput_q = world_rot.quat();
  const ignition::math::Quaterniond ign_q(maliput_q.w(), maliput_q.x(), maliput_q.y(), maliput_q.z());
  const maliput::math::Vector3& bb_local_pos = bb.position();
  const ignition::math::Vector3d world_center =
      ignition::math::Vector3d(world_pos.x(), world_pos.y(), world_pos.z()) +
      ign_q * ignition::math::Vector3d(bb_local_pos.x(), bb_local_pos.y(), bb_local_pos.z());

  const maliput::math::Vector3& size = bb.box_size();
  const double half_diagonal = 0.5 * std::sqrt(size.x() * size.x() + size.y() * size.y() + size.z() * size.z());

  ignition::rendering::VisualPtr visual = scene->CreateVisual();
  if (!visual) {
    ignerr << "TrafficSignManager: failed to create visual for sign " << _sign->id().string() << std::endl;
    return;
  }
  visual->AddGeometry(scene->CreateBox());
  visual->SetWorldScale(size.x(), size.y(), size.z());
  visual->SetWorldRotation(world_rot.roll(), world_rot.pitch(), world_rot.yaw());
  visual->SetWorldPosition(world_center);
  visual->SetMaterial(signMaterial, false);
  scene->RootVisual()->AddChild(visual);

  signs_[_sign->id()] = SignVisual{visual, world_center, half_diagonal, _sign};
}

}  // namespace viz
}  // namespace maliput
