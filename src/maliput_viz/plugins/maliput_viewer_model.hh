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

#include <map>
#include <memory>
#include <ostream>
#include <sstream>
#include <string>

#include <ignition/common/Mesh.hh>
#include <ignition/math/Vector3.hh>
#include <maliput/api/lane.h>
#include <maliput/api/regions.h>
#include <maliput/api/road_geometry.h>
#include <maliput/api/road_network.h>
#include <maliput/api/rules/phase.h>
#include <maliput/api/rules/right_of_way_rule.h>
#include <maliput/api/rules/traffic_lights.h>
#include <maliput/common/maliput_throw.h>
#include <maliput/utility/generate_obj.h>
#include <maliput/utility/mesh.h>

// Returns a vector of all possible direction usage values. Item order
// matches maliput::api::rules::DirectionUsageRule::Type enumeration.
const std::vector<std::string> DirectionUsageRuleNames();

// Serializes `road_position` into `out`.
std::ostream& operator<<(std::ostream& out, const maliput::api::RoadPosition& road_position);

// Serializes `road_position_result` into `out`.
std::ostream& operator<<(std::ostream& out, const maliput::api::RoadPositionResult& road_position_result);

// Serializes `state_type` into `out`.
std::ostream& operator<<(std::ostream& out, const maliput::api::rules::RightOfWayRule::State::Type& state_type);

// Serializes `state` into `out`.
std::ostream& operator<<(std::ostream& out, const maliput::api::rules::RightOfWayRule::State& state);

// Serializes `s_range` into `out`.
std::ostream& operator<<(std::ostream& out, const maliput::api::SRange& s_range);

// Serializes `lane_s_range` into `out`.
std::ostream& operator<<(std::ostream& out, const maliput::api::LaneSRange& lane_s_range);

// Serializes `lane_s_route` into `out`.
std::ostream& operator<<(std::ostream& out, const maliput::api::LaneSRoute& lane_s_route);

// Serializes `zone_type` into `out`.
std::ostream& operator<<(std::ostream& out, const maliput::api::rules::RightOfWayRule::ZoneType& zone_type);

namespace maliput {
namespace viz {

/// Query and logs results to RoadGeometry or RoadRulebook minimizing the
/// overhead of getting the right calls / asserting conditions.
class RoadNetworkQuery {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(RoadNetworkQuery)

  /// Constructs a RoadNetworkQuery.
  ///
  /// @param out A pointer to an output stream where results will be logged.
  ///            It must not be nullptr.
  /// @param rn A pointer to a RoadNetwork. It must not be nullptr.
  /// @throws std::runtime_error When `out` or `rn` are nullptr.
  RoadNetworkQuery(std::ostream* out, maliput::api::RoadNetwork* rn) : out_(out), rn_(rn) {
    MALIPUT_THROW_UNLESS(out_ != nullptr);
    MALIPUT_THROW_UNLESS(rn_ != nullptr);
  }

  /// Redirects `inertial_position` and `radius` to RoadGeometry::FindRoadPosition().
  void FindRoadPositions(const maliput::api::InertialPosition& inertial_position, double radius);

  /// Redirects `lane_position` to `lane_id`'s Lane::ToInertialPosition().
  void ToInertialPosition(const maliput::api::LaneId& lane_id, const maliput::api::LanePosition& lane_position);

  /// Redirects `inertial_position` to `lane_id`'s Lane::ToSegmentPosition().
  void ToSegmentPosition(const maliput::api::LaneId& lane_id, const maliput::api::InertialPosition& inertial_position);

  /// Redirects `inertial_position` to RoadGeometry::ToRoadPosition().
  void ToRoadPosition(const maliput::api::InertialPosition& inertial_position);

  /// Looks for all the maximum speed limits allowed at `lane_id`.
  void GetMaxSpeedLimit(const maliput::api::LaneId& lane_id);

  /// Looks for all the direction usages at `lane_id`.
  void GetDirectionUsage(const maliput::api::LaneId& lane_id);

  /// Gets all right-of-way rules for the given `lane_s_range`.
  void GetRightOfWay(const maliput::api::LaneSRange& lane_s_range);

  /// Gets all the rule states for the given `state`.
  void GetState(const maliput::api::rules::Rule::State& state);

  /// Gets all the range value rules for the given `lane_id`.
  void GetRangeValue(const maliput::api::LaneId& lane_id);

  /// Gets all discrete value rules for the given `lane_id`.
  void GetDiscreteValue(const maliput::api::LaneId& lane_id);

  /// Gets all right-of-way rules' states for a given phase in a given phase
  /// ring.
  void GetPhaseRightOfWay(const maliput::api::rules::PhaseRing::Id& phase_ring_id,
                          const maliput::api::rules::Phase::Id& phase_id);

 private:
  // Finds QueryResults of Rules for `lane_id`.
  maliput::api::rules::RoadRulebook::QueryResults FindRulesFor(const maliput::api::LaneId& lane_id);

  std::ostream* out_{};
  maliput::api::RoadNetwork* rn_{};
};

/// \brief Type of label based on road entities.
enum class MaliputLabelType {
  kLane,         ///< A lane label.
  kBranchPoint,  ///< A branch point label.
};

/// \brief Holds the mesh, material and visualization status.
class MaliputMesh {
 public:
  /// \brief Holds the visualization status.
  bool visible{false};

  /// \brief Holds the mesh status.
  bool enabled{false};

  /// \brief Holds a pointer to the mesh.
  std::unique_ptr<ignition::common::Mesh> mesh{};

  /// \brief Holds a pointer to the material information.
  std::unique_ptr<maliput::utility::Material> material{};
};

/// \brief Holds the information to build a label.
class MaliputLabel {
 public:
  /// \brief Holds the visualization status.
  bool visible{true};

  /// \brief Holds the mesh status.
  bool enabled{true};

  /// \brief Text to show by the label.
  std::string text{};

  /// \brief Position of the label.
  ignition::math::Vector3d position{};

  /// \brief The type of label
  MaliputLabelType labelType{};
};

/// \brief Converts @p _type into a MaliputLabelType.
/// \details @p _type must be either "lane_text_label" or
/// "branchpoint_text_label".
/// \throws std::runtime_error When @p _type is not "lane_text_label" nor
/// "branchpoint_text_label".
/// \return A MaliputLabelType that is kLane when @p _type is "lane_text_label"
/// and kBranchPoint when @p _type is "branchpoint_text_label".
MaliputLabelType FromString(const std::string& _type);

/// \brief Model of the plugin.
///
/// Holds the information, as a map of meshes and materials.
class MaliputViewerModel {
 public:
  /// \brief Constructor.
  /// @param _roadNetwork A maliput::api::RoadNetwork.
  MaliputViewerModel(std::unique_ptr<maliput::api::RoadNetwork> _roadNetwork);

  /// \brief Destructor.
  ~MaliputViewerModel() = default;

  /// \brief Getter of the map of meshes.
  /// \return The map of meshes.
  const std::map<std::string, std::unique_ptr<MaliputMesh>>& Meshes() const;

  /// \brief Getter of the map of labels.
  /// \return The map of labels.
  const std::map<std::string, MaliputLabel>& Labels() const;

  /// \brief Modifies the visualization state of @p key mesh.
  /// \param[in] _key The name of the mesh.
  /// \param[in] _isVisible The new visualization status of the mesh.
  void SetLayerState(const std::string& _key, bool _isVisible);

  /// \brief Modifies the visualization state of @p _type type text labels.
  /// \param[in] _key The id of the lane or branch point.
  /// \param[in] _isVisible The new visualization status of the text label.
  void SetTextLabelState(const std::string& _key, bool _isVisible);

  /// \brief Modifies the visualization state of @p _type type text labels.
  /// \param[in] _type The desired label type to target.
  /// \param[in] _isVisible The new visualization status of the text label.
  void SetTextLabelState(MaliputLabelType _type, bool _isVisible);

  /// \brief Get the lane associated with a point in world space if exists.
  /// \param[in] _position World position of point that intersected with a plane
  /// \return Lane associated with that position or nullptr if not found.
  const maliput::api::Lane* GetLaneFromWorldPosition(const ignition::math::Vector3d& _position);

  /// \brief Get the maliput::api::RoadPositionResult associated with a point in
  ///        world space if exists.
  /// \pre The RoadGeometry (in case of multilane) or the RoadNetwork must be initialized.
  /// \param[in] _position World position of point that intersected with a plane
  /// \return maliput::api::RoadPositionResult associated with that position.
  const maliput::api::RoadPositionResult GetRoadPositionResult(const ignition::math::Vector3d& _position);

  /// \brief Get the lane associated with an id.
  /// \param[in] _id Id of the lane.
  /// \return Lane associated with given id or nullptr if id is invalid.
  const maliput::api::Lane* GetLaneFromId(const std::string& _id);

  /// \brief Get all the traffic lights from the underlying traffic light book that lives in the  road network if any.
  /// \returns Vector containing all the traffic lights that the underlying road network contains.
  std::vector<const maliput::api::rules::TrafficLight*> GetTrafficLights() const;

  /// \brief Get N lanes from the underlying road geometry.
  /// \param[in] _n Amount of lanes desired to get from the underlying
  /// road geometry.
  /// \tparam ContainerType A container class that must implement size(), push_back() and reserve() methods.
  /// See std::vector for further reference on each one of these methods. Its elements' type must be a string class
  /// that must be constructible with a single const char* argument and must support concatenation via operator+.
  /// \return Container that contains N lane ids.
  template <typename ContainerType>
  ContainerType GetNLanes(size_t _n) const;

  /// \brief Get all the lanes that the road geometry posses.
  /// \tparam ContainerType A container class that must implement size(), push_back() and reserve() methods.
  /// See std::vector for further reference on each one of these methods. Its elements' type must be a string class
  /// that must be constructible with a single const char* argument and must support concatenation via operator+.
  /// \return All lane ids from the underlying road geometry
  template <typename ContainerType>
  ContainerType GetAllLaneIds() const;

  /// \brief Get all the rules for a given lane.
  /// \param[in] _phaseRingId Id of the desired phase ring to get the rules from.
  /// \param[in] _phaseId Id of the desired phase to get the rules from.
  /// \param[in] _laneId Id of the desired lane to get the rules from.
  /// \tparam StringType A string class that must be constructible with a single const char* argument and
  /// must support concatenation via operator+.
  /// \return rules separated by brackets.
  /// Ex: [Right of way Rule]\n.
  template <typename StringType>
  StringType GetRulesOfLane(const std::string& _phaseRingId, const std::string& _phaseId,
                            const std::string& _laneId) const;

  template <typename StringType>
  std::unordered_map<std::string, std::vector<StringType>> GetPhaseRings() const;

  /// \brief Get the state of all the bulbs for a given phase ring id and phase id.
  /// \param[in] _phaseRingId Id of the desired phase ring to get the bulb states from.
  /// \param[in] _phaseId Id of the desired phase to get the bulb states from.
  /// \returns The state of all the bulbs in the underlying road network.
  maliput::api::rules::BulbStates GetBulbStates(const std::string& _phaseRingId, const std::string& _phaseId) const;

 private:
  /// \brief Converts @p _geoMeshes into a
  ///        std::map<std::string, std::unique_ptr<ignition::common::Mesh>>
  ///        filling the instance variable meshes.
  /// \param[in] _geoMeshes An named collection of GeoMesh objects to convert.
  void ConvertMeshes(
      const std::map<std::string, std::pair<maliput::utility::mesh::GeoMesh, maliput::utility::Material>>& _geoMeshes);

  /// \brief Converts @p _geoMeshes into a
  ///        std::map<std::string, std::unique_ptr<ignition::common::Mesh>>
  ///        filling the instance variable meshes.
  /// \param[in] _geoMeshes A named collection of GeoMesh objects to convert.
  void ConvertRoadGeometryMeshes(const maliput::utility::RoadGeometryMesh& _geoMeshes);

  /// \brief Populates this->labels map with this->roadGeometry lane and branch
  ///        point IDs.
  void GenerateLabels();

  /// \brief Get the right of way rules for a given LaneSRange.
  /// \param[in] _laneSRange Object that contains a lane id and a range in the s
  /// coordinate.
  /// \tparam StringType A string class that must be constructible with a single const char* argument and
  /// must support concatenation via operator+.
  /// \return Right of way rules as a StringType representation.
  template <typename StringType>
  StringType GetRightOfWayRules(const maliput::api::LaneSRange& _laneSRange) const;

  /// \brief Get the range value rules for a given lane id.
  /// \param[in] _laneId Id of the lane to get the direction usage rules from.
  /// \tparam StringType A string class that must be constructible with a single const char* argument and
  /// must support concatenation via operator+.
  /// \return Range value rules as a StringType representation.
  template <typename StringType>
  StringType GetRangeValueRules(const maliput::api::LaneId& _laneId) const;

  /// \brief Get the discrete value rules for a given lane id.
  /// \param[in] _laneId Id of the lane to get the direction usage rules from.
  /// \tparam StringType A string class that must be constructible with a single const char* argument and
  /// must support concatenation via operator+.
  /// \return Discrete value rules as a StringType representation.
  template <typename StringType>
  StringType GetDiscreteValueRules(const maliput::api::LaneId& _laneId) const;

  /// \brief Get the max speed rules for a given lane id.
  /// \param[in] _laneId Id of the lane to get the max speed limit rules from.
  /// \tparam StringType A string class that must be constructible with a single const char* argument and
  /// must support concatenation via operator+.
  /// \return Max speed limit rules as a StringType representation.
  template <typename StringType>
  StringType GetMaxSpeedLimitRules(const maliput::api::LaneId& _laneId) const;

  /// \brief Get the direction usage rules for a given lane id.
  /// \param[in] _laneId Id of the lane to get the direction usage rules from.
  /// \tparam StringType A string class that must be constructible with a single const char* argument and
  /// must support concatenation via operator+.
  /// \return Direction usage rules as a StringType representation.
  template <typename StringType>
  StringType GetDirectionUsageRules(const maliput::api::LaneId& _laneId) const;

  template <typename StringType>
  StringType GetPhaseRightOfWayRules(const maliput::api::rules::PhaseRing::Id& _phaseRingId,
                                     const maliput::api::rules::Phase::Id& _phaseId) const;

  // To support both malidrive and multilane files, we have both. roadNetwork
  // has a pointer to a RoadGeometry.

  /// \brief Maliput RoadNetwork pointer.
  std::unique_ptr<maliput::api::RoadNetwork> roadNetwork;

  /// \brief Map of meshes to hold all the ignition meshes.
  std::map<std::string, std::unique_ptr<MaliputMesh>> maliputMeshes;

  /// \brief Map of labels.
  std::map<std::string, MaliputLabel> labels;
};

template <typename ContainerType>
ContainerType MaliputViewerModel::GetNLanes(size_t _n) const {
  const maliput::api::RoadGeometry* rg = this->roadNetwork->road_geometry();
  const std::unordered_map<maliput::api::LaneId, const maliput::api::Lane*>& all_lanes = rg->ById().GetLanes();
  ContainerType lanes;
  lanes.reserve(_n);
  for (const auto& lane : all_lanes) {
    if (static_cast<size_t>(lanes.size()) == _n) {
      break;
    }
    lanes.push_back(lane.first.string().c_str());
  }
  return lanes;
}

template <typename ContainerType>
ContainerType MaliputViewerModel::GetAllLaneIds() const {
  const maliput::api::RoadGeometry* rg = this->roadNetwork->road_geometry();
  const std::unordered_map<maliput::api::LaneId, const maliput::api::Lane*>& all_lanes = rg->ById().GetLanes();
  ContainerType lanes;
  lanes.reserve(all_lanes.size());
  for (const auto& lane : all_lanes) {
    lanes.push_back(lane.first.string().c_str());
  }
  return lanes;
}

template <typename StringType>
std::unordered_map<std::string, std::vector<StringType>> MaliputViewerModel::GetPhaseRings() const {
  if (this->roadNetwork == nullptr) {
    return std::unordered_map<std::string, std::vector<StringType>>();
  }
  std::unordered_map<std::string, std::vector<StringType>> phase_rings;
  const maliput::api::rules::PhaseRingBook* phase_ring_book = this->roadNetwork->phase_ring_book();
  std::vector<maliput::api::rules::PhaseRing::Id> phase_ring_ids = phase_ring_book->GetPhaseRings();
  phase_rings.reserve(phase_ring_ids.size());
  for (const auto& phase_ring_id : phase_ring_ids) {
    std::optional<maliput::api::rules::PhaseRing> phase_ring = phase_ring_book->GetPhaseRing(phase_ring_id);
    MALIPUT_THROW_UNLESS(phase_ring.has_value());
    const std::unordered_map<maliput::api::rules::Phase::Id, maliput::api::rules::Phase>& phases =
        phase_ring.value().phases();
    std::vector<StringType> phase_ids;
    phase_ids.reserve(phases.size());
    std::transform(phases.begin(), phases.end(), std::back_inserter(phase_ids),
                   [](const std::pair<maliput::api::rules::Phase::Id, maliput::api::rules::Phase>& key_value) {
                     return StringType(key_value.first.string().c_str());
                   });
    phase_rings[phase_ring_id.string()] = std::move(phase_ids);
  }
  return phase_rings;
}

template <typename StringType>
StringType MaliputViewerModel::GetRulesOfLane(const std::string& _phaseRingId, const std::string& _phaseId,
                                              const std::string& _laneId) const {
  if (this->roadNetwork == nullptr) {
    return StringType("There are no rules for this road");
  }
  maliput::api::LaneId id(_laneId);
  const maliput::api::RoadGeometry::IdIndex& roadIndex = this->roadNetwork->road_geometry()->ById();
  maliput::api::LaneSRange laneSRange(id, maliput::api::SRange(0., roadIndex.GetLane(id)->length()));
  StringType rules = "[Right of way rules]\n" + GetRightOfWayRules<StringType>(laneSRange) + "\n" +
                     "[Max speed limit rules]\n" + GetMaxSpeedLimitRules<StringType>(id) + "\n" +
                     "[Direction usage rules]\n" + GetDirectionUsageRules<StringType>(id) + "\n" +
                     "[Range Value rules]\n" + GetRangeValueRules<StringType>(id) + "\n" + "[Discrete Value rules]\n" +
                     GetDiscreteValueRules<StringType>(id) + "\n";

  if (!_phaseRingId.empty() && !_phaseId.empty()) {
    rules += "[Right of way rules by phase ring id and phase id]\n" +
             GetPhaseRightOfWayRules<StringType>(maliput::api::rules::PhaseRing::Id(_phaseRingId),
                                                 maliput::api::rules::Phase::Id(_phaseId)) +
             "\n";
  }

  return rules;
}

template <typename StringType>
StringType MaliputViewerModel::GetDiscreteValueRules(const maliput::api::LaneId& _laneId) const {
  std::ostringstream discreteValueRules;
  RoadNetworkQuery query(&discreteValueRules, roadNetwork.get());
  query.GetDiscreteValue(_laneId);
  return StringType(discreteValueRules.str().c_str());
}

template <typename StringType>
StringType MaliputViewerModel::GetRangeValueRules(const maliput::api::LaneId& _laneId) const {
  std::ostringstream rangeValueRules;
  RoadNetworkQuery query(&rangeValueRules, roadNetwork.get());
  query.GetRangeValue(_laneId);
  return StringType(rangeValueRules.str().c_str());
}

template <typename StringType>
StringType MaliputViewerModel::GetRightOfWayRules(const maliput::api::LaneSRange& _laneSRange) const {
  std::ostringstream rightOfWayRules;
  RoadNetworkQuery query(&rightOfWayRules, roadNetwork.get());
  query.GetRightOfWay(_laneSRange);
  return StringType(rightOfWayRules.str().c_str());
}

template <typename StringType>
StringType MaliputViewerModel::GetMaxSpeedLimitRules(const maliput::api::LaneId& _laneId) const {
  std::ostringstream speedLimitRules;
  RoadNetworkQuery query(&speedLimitRules, roadNetwork.get());
  query.GetMaxSpeedLimit(_laneId);
  return StringType(speedLimitRules.str().c_str());
}

template <typename StringType>
StringType MaliputViewerModel::GetDirectionUsageRules(const maliput::api::LaneId& _laneId) const {
  std::ostringstream directionUsageRules;
  RoadNetworkQuery query(&directionUsageRules, roadNetwork.get());
  query.GetDirectionUsage(_laneId);
  return StringType(directionUsageRules.str().c_str());
}

template <typename StringType>
StringType MaliputViewerModel::GetPhaseRightOfWayRules(const maliput::api::rules::PhaseRing::Id& _phaseRingId,
                                                       const maliput::api::rules::Phase::Id& _phaseId) const {
  std::ostringstream rightOfWayRulesPhaseRing;
  RoadNetworkQuery query(&rightOfWayRulesPhaseRing, roadNetwork.get());
  query.GetPhaseRightOfWay(_phaseRingId, _phaseId);
  return StringType(rightOfWayRulesPhaseRing.str().c_str());
}

}  // namespace viz
}  // namespace maliput
