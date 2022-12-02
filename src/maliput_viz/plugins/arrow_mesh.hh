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
#pragma once

#include <ignition/math/Vector3.hh>
#include <ignition/rendering/Scene.hh>

namespace maliput {
namespace viz {

/// \brief Renders a cone which will act as a pointing arrow when the user clicks over a lane
class ArrowMesh {
 public:
  /// \brief Creates a cone and adds it as a child to the RootVisual of the scene, which will move
  /// upwards and downwards for a fixed amount of ticks.
  /// \param[in] _scene Scene pointer to create the cone.
  /// \param[in] _zOffset Units above the pointed object that the cone tip should be.
  /// \param[in] _scaleFactor Factor to increase/decrease the scale of the arrow based on the distance from the camera
  /// to the clicked position.
  ArrowMesh(ignition::rendering::ScenePtr& _scene, double _zOffset = 2.0, double _scaleFactor = 0.025);
  /// \brief Destructor.
  ~ArrowMesh() = default;

  /// \brief Moves the arrow to a given world position and resets the downwards movement.
  /// \param[in] _distanceFromCamera How far the camera is from the clicked point.
  /// \param[in] _worldPosition World position of the cursor ray cast click.
  void SelectAt(double _distanceFromCamera, const ignition::math::Vector3d& _worldPosition);

  /// \brief Toggles the visibility of the arrow.
  /// \param[in] _visible Boolean that determines if the arrow should be visible or not.
  void SetVisibility(bool _visible);

  /// \brief Updates the position of the arrow moving slightly the z axis.
  void Update();

 private:
  /// \brief Bounding box of the cone in construction time.
  ignition::math::Vector3d minArrowBoundingBox;

  /// \brief Visual pointer of the created cone.
  ignition::rendering::VisualPtr arrow;

  /// \brief Units that the tip of the cone should be from the clicked position.
  const double zOffset;
  /// \brief Factor to increase/decrease the size of the cone.
  const double scaleFactor;
  /// \brief Distance to move downwards/upwards.
  const double distanceToMove;
  /// \brief How much units should the arrow move downwards/upwards per tick.
  const double step;
  /// \brief How much ticks required to move the arrow from down to up or up to down.
  const int totalTicks;
  /// \brief How many ticks passed since it started moving.
  int currentTick;
  /// \brief Tells the arrow if it should move upwards (1) or downwards (-1).
  int currentDirection;
};

}  // namespace viz
}  // namespace maliput
