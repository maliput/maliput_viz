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

#ifndef MALIPUT_MESH_CONVERTER_HH
#define MALIPUT_MESH_CONVERTER_HH

#include <map>
#include <string>
#include <vector>

#include <ignition/common/Mesh.hh>
#include <maliput/utility/mesh.h>

namespace maliput {
namespace viz {
namespace mesh {

/// Converts a GeoMesh into an ignition::common::Mesh.
///
/// The ignition::common::Mesh will contain only one ignition::common::SubMesh,
/// and current implementation only supports up to four vertices per GeoFace.
/// Note that generated meshes will point to both sides, consequently for each
/// group of three vertices that represent a triangle in the mesh, there are six
/// indices entries.
///
/// @param name The name of the mesh.
/// @param geo_mesh The GeoMesh to convert into a ignition::common::Mesh. It
/// must have at least once GeoFace with at least three vertices on it.
/// @return A std::unique_ptr<ignition::common::Mesh> with the equivalent mesh
/// construction. The pointer will be nullptr when the mesh has not the correct
/// vertices requirements.
std::unique_ptr<ignition::common::Mesh> Convert(const std::string& name,
                                                const maliput::utility::mesh::GeoMesh& geo_mesh);

}  // namespace mesh
}  // namespace viz
}  // namespace maliput

#endif
