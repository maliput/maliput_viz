// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2018-2022, Toyota Research Institute. All rights reserved.
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

#include "arrow_mesh.hh"

#include <ignition/common/MeshManager.hh>
#include <ignition/math/Helpers.hh>
#include <ignition/rendering/Material.hh>
#include <ignition/rendering/Mesh.hh>
#include <ignition/rendering/Scene.hh>

namespace maliput {
namespace viz {

ArrowMesh::ArrowMesh(ignition::rendering::ScenePtr& _scene, double _zOffset, double _scaleFactor)
    : zOffset(_zOffset),
      scaleFactor(_scaleFactor),
      distanceToMove(zOffset / 4.0),
      step(0.005),
      totalTicks(distanceToMove / step),
      currentTick(0),
      currentDirection(-1) {
  ignition::rendering::MaterialPtr material = _scene->CreateMaterial();
  material->SetDiffuse(255.0, 0.0, 0.0, 1.0);
  material->SetAmbient(255.0, 0.0, 0.0, 1.0);
  this->arrow = _scene->CreateVisual();
  this->arrow->AddGeometry(_scene->CreateCone());
  this->arrow->SetMaterial(material);
  this->arrow->SetVisible(false);
  this->arrow->SetWorldPosition(0., 0., 0.);
  this->arrow->SetWorldRotation(0, IGN_PI, 0.);
  _scene->RootVisual()->AddChild(this->arrow);
  ignition::common::MeshManager* meshManager = ignition::common::MeshManager::Instance();
  const ignition::common::Mesh* unitConeMesh = meshManager->MeshByName("unit_cone");
  minArrowBoundingBox = unitConeMesh->Min();
}

void ArrowMesh::SelectAt(double _distanceFromCamera, const ignition::math::Vector3d& _worldPosition) {
  const double scaleIncrement = 1.0 + scaleFactor * _distanceFromCamera;
  const double newMinArrowBBZAxis = scaleIncrement * minArrowBoundingBox.Z();
  this->arrow->SetWorldScale(scaleIncrement, scaleIncrement, scaleIncrement);
  this->arrow->SetWorldPosition(_worldPosition.X(), _worldPosition.Y(),
                                _worldPosition.Z() + std::abs(newMinArrowBBZAxis) + zOffset);
  currentDirection = -1;
  currentTick = 0;
}

void ArrowMesh::SetVisibility(bool _visible) { this->arrow->SetVisible(_visible); }

void ArrowMesh::Update() {
  ignition::math::Vector3d worldPosition = this->arrow->WorldPosition();
  this->arrow->SetWorldPosition(worldPosition.X(), worldPosition.Y(), worldPosition.Z() + currentDirection * step);
  if (currentTick++ == totalTicks) {
    currentDirection = currentDirection * -1;
    currentTick = 0;
  }
}

}  // namespace viz
}  // namespace maliput
