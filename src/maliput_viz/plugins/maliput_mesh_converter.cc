// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2017-2022, Toyota Research Institute. All rights reserved.
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
//
// Note: code in this file was forked and changed from this file:
// https://github.com/RobotLocomotion/drake/blob/master/automotive/maliput/utility/generate_obj.cc
// and this commit: 82bf3c8a02678f553c746bdbbe0f8e5a345841b7

#include "maliput_mesh_converter.hh"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <functional>
#include <initializer_list>
#include <tuple>
#include <unordered_map>
#include <vector>

#include <ignition/common/SubMesh.hh>
#include <ignition/math/Vector3.hh>
#include <maliput/api/branch_point.h>
#include <maliput/api/junction.h>
#include <maliput/api/lane.h>
#include <maliput/api/lane_data.h>
#include <maliput/api/road_geometry.h>
#include <maliput/api/segment.h>
#include <maliput/common/maliput_throw.h>
#include <maliput/utility/mesh.h>

using namespace maliput::utility::mesh;

namespace maliput {
namespace viz {
namespace mesh {
namespace {

// Sorts a vector of tuples composed of 3D points and an index in a polar and
// counterclockwise way.
//
// Points in `unordered_vector` are supposed to be on the same plane. Based on
// that, the mid point of all of them is computed and then used as a center of
// coordinates. Angles are computed using that center and the compared to sort
// the vector.
// @returns A std::vector<std::tuple<ignition::math::Vector3d, int>> with the
// same items of `unordered_vector` but sorted in a polar and counterclockwise
// order.
// @throws std::runtime_error When `unordered_vector`' size is zero.
std::vector<std::tuple<ignition::math::Vector3d, int>> PolarSort(
    const std::vector<std::tuple<ignition::math::Vector3d, int>>& unordered_vector) {
  MALIPUT_THROW_UNLESS(unordered_vector.size() > 0);

  std::vector<std::tuple<ignition::math::Vector3d, int>> ordered_vector = unordered_vector;

  // Computes the center.
  std::vector<ignition::math::Vector3d> points;
  ignition::math::Vector3d center;
  for (const std::tuple<ignition::math::Vector3d, int>& vertex_index : unordered_vector) {
    center += std::get<0>(vertex_index);
  }
  center /= static_cast<double>(points.size());

  // TODO(#4): This polar sort should be done with respect to the
  //           plane of all the points in the face instead of the
  //           plane z=0. We need the normal of the plane to do
  //           so.
  std::sort(
      ordered_vector.begin(), ordered_vector.end(),
      [center](const std::tuple<ignition::math::Vector3d, int>& a, const std::tuple<ignition::math::Vector3d, int>& b) {
        double a_y = std::atan2(std::get<0>(a).Y() - center.Y(), std::get<0>(a).X() - center.X());
        double b_y = std::atan2(std::get<0>(b).Y() - center.Y(), std::get<0>(b).X() - center.X());
        if (a_y < 0) a_y += 2 * M_PI;
        if (b_y < 0) b_y += 2 * M_PI;
        return a_y < b_y;
      });
  return ordered_vector;
}

// \brief Generates a unique name for a mesh.
// \param baseName The base name. It will be appended with "_ID".
// \return A string formatted like: @p baseName + "_" + ID , where ID is a
// increasing unsigned integer.
const std::string GenerateUniqueMeshName(const std::string& baseName) {
  static uint counter = 0;
  return baseName + "_" + std::to_string(counter++);
}

}  // namespace

std::unique_ptr<ignition::common::Mesh> Convert(const std::string& name, const GeoMesh& geo_mesh) {
  // Checks before actually creating the mesh.
  if (geo_mesh.num_vertices() < 3 && geo_mesh.faces().size() < 1) {
    return nullptr;
  }

  auto mesh = std::make_unique<ignition::common::Mesh>();
  // ignition::rendering and ignition::common do not support unloading meshes. So, as for now, we need to
  // create meshes with unique names (by adding an increasing ID to name) so it can be referenced and
  // not mixed with another mesh. See: https://github.com/gazebosim/gz-rendering/issues/27
  mesh->SetName(GenerateUniqueMeshName(name));

  auto sub_mesh = std::make_unique<ignition::common::SubMesh>();
  sub_mesh->SetPrimitiveType(ignition::common::SubMesh::TRIANGLES);

  auto geo_position_to_ign_vector = [](const maliput::api::InertialPosition& v) {
    return ignition::math::Vector3d(v.x(), v.y(), v.z());
  };

  // Adds vertices, fake normals and fake texture coordinates. Each vertex needs
  // a normal and a texture coordinate. Fake texture coordinates are added given
  // that Drake's meshes do not have any information about that and normals are
  // filled with stubs to match the same number of vertices. Later, when
  // iterating through faces, normals will be filled with the correct value.
  // Note that geo_mesh may not have the same number of vertices as normals
  // given that it keeps no duplicates.
  for (int i = 0; i < geo_mesh.num_vertices(); ++i) {
    sub_mesh->AddVertex(geo_position_to_ign_vector(geo_mesh.get_vertex(i).v()));
    sub_mesh->AddNormal({0, 0, 1});
    sub_mesh->AddTexCoord({0, 0});
  }

  // Sets the indices based on how the faces were built.
  for (const IndexFace& index_face : geo_mesh.faces()) {
    // Assuming that IndexFace will not have more
    // than 4 vertices. The class supports more, however
    // proper triangulation code needs to be done so as
    // to support it.
    MALIPUT_THROW_UNLESS(index_face.vertices().size() == 3 || index_face.vertices().size() == 4);

    std::vector<std::tuple<ignition::math::Vector3d, int>> ordered_vertices_indices;
    for (const IndexFace::Vertex& ifv : index_face.vertices()) {
      // Sets the correct normal.
      const maliput::api::InertialPosition normal = geo_mesh.get_normal(ifv.normal_index).n();
      sub_mesh->SetNormal(ifv.vertex_index, geo_position_to_ign_vector(normal));
      // Adds the vertices to the vector so we can later order them.
      ordered_vertices_indices.push_back(
          std::make_tuple<ignition::math::Vector3d, int>(sub_mesh->Vertex(ifv.vertex_index), int(ifv.vertex_index)));
    }
    ordered_vertices_indices = PolarSort(ordered_vertices_indices);

    sub_mesh->AddIndex(std::get<1>(ordered_vertices_indices[0]));
    sub_mesh->AddIndex(std::get<1>(ordered_vertices_indices[1]));
    sub_mesh->AddIndex(std::get<1>(ordered_vertices_indices[2]));
    // Includes the remaining triangle.
    if (index_face.vertices().size() == 4) {
      sub_mesh->AddIndex(std::get<1>(ordered_vertices_indices[2]));
      sub_mesh->AddIndex(std::get<1>(ordered_vertices_indices[3]));
      sub_mesh->AddIndex(std::get<1>(ordered_vertices_indices[0]));
    }
  }

  mesh->AddSubMesh(std::move(sub_mesh));
  return mesh;
}

}  // namespace mesh
}  // namespace viz
}  // namespace maliput
