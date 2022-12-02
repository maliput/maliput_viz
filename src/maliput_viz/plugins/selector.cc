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

#include "selector.hh"

#include <ignition/common/Console.hh>
#include <ignition/math/Helpers.hh>
#include <ignition/math/Matrix4.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/rendering/Material.hh>
#include <ignition/rendering/Mesh.hh>
#include <ignition/rendering/Scene.hh>
#include <maliput/api/lane.h>
#include <maliput/api/lane_data.h>
#include <maliput/common/maliput_throw.h>

namespace maliput {
namespace viz {

Selector::Selector(ignition::rendering::ScenePtr& _scene, double _scaleX, double _scaleY, double _scaleZ, int _poolSize,
                   int _numLanes, double _minTolerance)
    : scene(_scene),
      numLanes(_numLanes),
      cubesPerLane(_poolSize),
      minTolerance(_minTolerance < _scaleX * 2.0 ? _scaleX : _minTolerance) {
  MALIPUT_THROW_UNLESS(_poolSize > 3);
  MALIPUT_THROW_UNLESS(_numLanes > 0);
  MALIPUT_THROW_UNLESS(_scene != nullptr);
  MALIPUT_THROW_UNLESS(_minTolerance >= 0);
  MALIPUT_THROW_UNLESS(_scaleX > 0);
  MALIPUT_THROW_UNLESS(_scaleY > 0);
  MALIPUT_THROW_UNLESS(_scaleZ > 0);
  ignition::rendering::MaterialPtr material = _scene->CreateMaterial();
  material->SetDiffuse(255.0, 0.0, 0.0, 1.0);
  material->SetAmbient(255.0, 0.0, 0.0, 1.0);
  populationMap.resize(_numLanes);
  dimensionScale = ignition::math::Vector3d(_scaleX, _scaleY, _scaleZ);
  cubeMaterial = material;

  // Create and init space for _numLanes selected lanes, create more later if more lanes are selected
  this->CreateCubes(_scene, _scaleX, _scaleY, _scaleZ, cubeMaterial, _numLanes * _poolSize);
}

int Selector::GetLaneIndex(int _slot) { return cubesPerLane * _slot; }

int Selector::GetRemainingCubes() { return cubesPerLane - 4; }

int Selector::GetCubeIndex(int _laneIndex) { return 4 + _laneIndex; }

void Selector::SelectLane(const maliput::api::Lane* _lane) {
  MALIPUT_THROW_UNLESS(_lane != nullptr);
  const std::string laneId = _lane->id().string();
  const std::string start_bp_id = _lane->GetBranchPoint(maliput::api::LaneEnd::kStart)->id().string();
  const std::string end_bp_id = _lane->GetBranchPoint(maliput::api::LaneEnd::kFinish)->id().string();

  // Remove markers from previously selected lane
  if (lanesSelected[laneId]) {
    const int slot = lanesSelected[laneId] - 1;
    const int laneIndex = slot * cubesPerLane;
    branchPointsSelected[start_bp_id]--;
    branchPointsSelected[end_bp_id]--;
    // Turn off visibility of a previously selected lane
    SetVisibilityOfCubesStartingFromTo(laneIndex, laneIndex + cubesPerLane, false);
    populationMap[slot] = false;
    lanesSelected[laneId] = 0;
    return;
  }

  // Increment value of branch point selection
  branchPointsSelected[start_bp_id]++;
  branchPointsSelected[end_bp_id]++;

  const double max_s = _lane->length();

  const maliput::api::RBounds initialRBounds = _lane->lane_bounds(0.);
  const maliput::api::RBounds endRBounds = _lane->lane_bounds(max_s);

  const maliput::api::InertialPosition initialRMinInertialPos =
      _lane->ToInertialPosition(maliput::api::LanePosition(0., initialRBounds.min(), 0.));
  const maliput::api::InertialPosition initialRMaxInertialPos =
      _lane->ToInertialPosition(maliput::api::LanePosition(0., initialRBounds.max(), 0.));

  const maliput::api::InertialPosition endRMinInertialPos =
      _lane->ToInertialPosition(maliput::api::LanePosition(max_s, endRBounds.min(), 0.));
  const maliput::api::InertialPosition endRMaxInertialPos =
      _lane->ToInertialPosition(maliput::api::LanePosition(max_s, endRBounds.max(), 0.));

  // Find first empty slot of unused markers
  int slot = FindFirstEmpty();

  // If all preinitialized markers are in use, create more
  if (slot == -1) {
    this->CreateCubes(scene, dimensionScale.X(), dimensionScale.Y(), dimensionScale.Z(), cubeMaterial,
                      numLanes * cubesPerLane);
    for (size_t i = 0; i < numLanes; ++i) {
      populationMap.push_back(false);
    }
    slot = FindFirstEmpty();
  }

  // Add one for easy distinction between non-existing keys
  lanesSelected[laneId] = slot + 1;

  const unsigned int laneIndex = GetLaneIndex(slot);
  cubes[laneIndex]->SetWorldPosition(initialRMinInertialPos.x(), initialRMinInertialPos.y(),
                                     initialRMinInertialPos.z());
  cubes[laneIndex]->SetVisible(true);
  cubes[laneIndex + 1]->SetWorldPosition(initialRMaxInertialPos.x(), initialRMaxInertialPos.y(),
                                         initialRMaxInertialPos.z());
  cubes[laneIndex + 1]->SetVisible(true);

  cubes[laneIndex + 2]->SetWorldPosition(endRMinInertialPos.x(), endRMinInertialPos.y(), endRMinInertialPos.z());
  cubes[laneIndex + 2]->SetVisible(true);
  cubes[laneIndex + 3]->SetWorldPosition(endRMaxInertialPos.x(), endRMaxInertialPos.y(), endRMaxInertialPos.z());
  cubes[laneIndex + 3]->SetVisible(true);

  int cubesUsed = GetCubeIndex(laneIndex);
  int remainingCubes = GetRemainingCubes();
  MoveCubeAtMidPointInR(initialRMinInertialPos, initialRMaxInertialPos, &cubesUsed, &remainingCubes);

  MoveCubeAtMidPointInR(endRMinInertialPos, endRMaxInertialPos, &cubesUsed, &remainingCubes);

  int cubesLeftSide = std::ceil(remainingCubes / 2);
  int cubesRightSide = remainingCubes - cubesLeftSide;
  double oldTolerance = minTolerance;

  // If we have less cubes to cover the lane, increase the tolerance.
  minTolerance = GetNewToleranceToPopulateLane(max_s, cubesLeftSide);
  cubesLeftSide = static_cast<int>(max_s / minTolerance);
  MoveCubeAtMidPointInS(_lane, 0., max_s, true, &cubesUsed, &cubesLeftSide);

  minTolerance = oldTolerance;
  minTolerance = GetNewToleranceToPopulateLane(max_s, cubesRightSide);
  cubesRightSide = static_cast<int>(max_s / minTolerance);
  MoveCubeAtMidPointInS(_lane, 0., max_s, false, &cubesUsed, &cubesRightSide);
  minTolerance = oldTolerance;
}

int Selector::FindFirstEmpty() {
  for (size_t i = 0; i < populationMap.size(); ++i) {
    if (!populationMap[i]) {
      populationMap[i] = true;
      return i;
    }
  }
  return -1;
}

void Selector::SetVisibility(bool _visible) { SetVisibilityOfCubesStartingFromTo(0, cubes.size(), _visible); }

void Selector::ResetPopulationMap() {
  for (size_t i = 0; i < populationMap.size(); ++i) {
    populationMap[i] = false;
  }
}

std::vector<std::string> Selector::GetSelectedBranchPoints() {
  std::vector<std::string> selectedBranchPoints;

  for (const auto& i : branchPointsSelected) {
    if (i.second) {
      selectedBranchPoints.push_back(i.first);
    }
  }

  return selectedBranchPoints;
}

std::vector<std::string> Selector::GetSelectedLanes() {
  std::vector<std::string> selectedLanes;

  for (const auto& i : lanesSelected) {
    if (i.second) {
      selectedLanes.push_back(i.first);
    }
  }

  return selectedLanes;
}

void Selector::ClearSelectedBranchPoints() { branchPointsSelected.clear(); }

void Selector::ClearSelectedLanes() { lanesSelected.clear(); }

void Selector::DeselectAll() {
  SetVisibility(false);
  ResetPopulationMap();
  ClearSelectedLanes();
  ClearSelectedBranchPoints();
}

bool Selector::IsSelected(const std::string& _id) { return lanesSelected[_id] || branchPointsSelected[_id]; }

bool Selector::IsSelected(const maliput::api::Lane* _lane) { return this->IsSelected(_lane->id().string()); }

void Selector::CreateCubes(ignition::rendering::ScenePtr& _scene, double _scaleX, double _scaleY, double _scaleZ,
                           ignition::rendering::MaterialPtr& _material, unsigned int _numCubes) {
  for (size_t i = 0; i < _numCubes; ++i) {
    ignition::rendering::VisualPtr cube = _scene->CreateVisual();
    cube->AddGeometry(_scene->CreateBox());
    cube->SetMaterial(_material, false);
    cube->SetVisible(false);
    cube->SetWorldScale(_scaleX, _scaleY, _scaleZ);
    cube->SetWorldPosition(0., 0., 0.);
    _scene->RootVisual()->AddChild(cube);
    cubes.push_back(cube);
  }
}

double Selector::GetNewToleranceToPopulateLane(double _laneLength, int _cubesUsedForSide) {
  const double newToleranceForLaneSide = _laneLength / _cubesUsedForSide;
  return newToleranceForLaneSide < minTolerance ? minTolerance : newToleranceForLaneSide;
}

void Selector::MoveCubeAtMidPointInR(const maliput::api::InertialPosition& _minRInertialPos,
                                     const maliput::api::InertialPosition& _maxRInertialPos, int* _cubesUsed,
                                     int* _maxAmountOfCubesToUse) {
  maliput::api::InertialPosition midPoint =
      maliput::api::InertialPosition::FromXyz((_maxRInertialPos.xyz() + _minRInertialPos.xyz()) / 2);
  if ((_maxRInertialPos - midPoint).length() > minTolerance && *_maxAmountOfCubesToUse != 0) {
    cubes[*_cubesUsed]->SetWorldPosition(midPoint.x(), midPoint.y(), midPoint.z());
    cubes[*_cubesUsed]->SetVisible(true);
    ++(*_cubesUsed);
    --(*_maxAmountOfCubesToUse);
    MoveCubeAtMidPointInR(_minRInertialPos, midPoint, _cubesUsed, _maxAmountOfCubesToUse);
    MoveCubeAtMidPointInR(midPoint, _maxRInertialPos, _cubesUsed, _maxAmountOfCubesToUse);
  }
}

void Selector::MoveCubeAtMidPointInS(const maliput::api::Lane* _lane, double min_s, double max_s, bool _left_side,
                                     int* _cubesUsed, int* _maxAmountOfCubesToUse) {
  const double mid_s = (max_s + min_s) / 2.0;
  const maliput::api::RBounds minRBounds = _lane->lane_bounds(min_s);
  const maliput::api::RBounds midRBounds = _lane->lane_bounds(mid_s);
  const maliput::api::RBounds maxRBounds = _lane->lane_bounds(max_s);
  const double r_min_bound = _left_side ? minRBounds.min() : minRBounds.max();
  const double r_mid_bound = _left_side ? midRBounds.min() : midRBounds.max();
  const double r_max_bound = _left_side ? maxRBounds.min() : maxRBounds.max();

  const maliput::api::InertialPosition minPoint =
      _lane->ToInertialPosition(maliput::api::LanePosition(min_s, r_min_bound, 0.));
  const maliput::api::InertialPosition midPoint =
      _lane->ToInertialPosition(maliput::api::LanePosition(mid_s, r_mid_bound, 0.));
  const maliput::api::InertialPosition maxPoint =
      _lane->ToInertialPosition(maliput::api::LanePosition(max_s, r_max_bound, 0.));

  if (!DoPointsViolateTolerance(midPoint, maxPoint) && !DoPointsViolateTolerance(midPoint, minPoint) &&
      *_maxAmountOfCubesToUse != 0) {
    ignition::math::Vector3d extremeMidPointMathVector(maxPoint.x(), maxPoint.y(), maxPoint.z());
    cubes[*_cubesUsed]->SetWorldPosition(midPoint.x(), midPoint.y(), midPoint.z());
    cubes[*_cubesUsed]->SetWorldRotation(
        ignition::math::Matrix4d::LookAt(cubes[*_cubesUsed]->WorldPosition(), extremeMidPointMathVector).Pose().Rot());
    cubes[*_cubesUsed]->SetVisible(true);
    ++(*_cubesUsed);
    --(*_maxAmountOfCubesToUse);
    MoveCubeAtMidPointInS(_lane, mid_s, max_s, _left_side, _cubesUsed, _maxAmountOfCubesToUse);
    MoveCubeAtMidPointInS(_lane, min_s, mid_s, _left_side, _cubesUsed, _maxAmountOfCubesToUse);
  }
}

void Selector::SetVisibilityOfCubesStartingFromTo(int _startFrom, int _to, bool _visible) {
  for (int i = _startFrom; i < _to; ++i) {
    cubes[i]->SetVisible(_visible);
  }
}

bool Selector::DoPointsViolateTolerance(const maliput::api::InertialPosition& _first_point,
                                        const maliput::api::InertialPosition& _second_point) {
  return (_first_point - _second_point).length() < minTolerance;
}

}  // namespace viz
}  // namespace maliput
