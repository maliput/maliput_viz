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

#include "maliput_viewer_model.hh"

#include <iostream>
#include <map>
#include <string>

#include <ignition/common/Console.hh>
#include <ignition/rendering/RayQuery.hh>
#include <maliput/api/lane_data.h>
#include <maliput/api/rules/phase.h>
#include <maliput/api/rules/traffic_light_book.h>
#include <maliput/utility/generate_obj.h>
#include <maliput/utility/mesh.h>

#include "maliput_mesh_converter.hh"

using namespace maliput;
using namespace viz;

// Returns a vector of all possible direction usage values. Item order
// matches maliput::api::rules::DirectionUsageRule::Type enumeration.
const std::vector<std::string> DirectionUsageRuleNames() {
  return {"WithS", "AgainstS", "Bidirectional", "BidirectionalTurnOnly", "NoUse", "Parking"};
}

// Serializes `road_position` into `out`.
std::ostream& operator<<(std::ostream& out, const maliput::api::RoadPosition& road_position) {
  return out << "(lane: " << road_position.lane->id().string() << ", lane_pos: " << road_position.pos << ")";
}

// Serializes `road_position_result` into `out`.
std::ostream& operator<<(std::ostream& out, const maliput::api::RoadPositionResult& road_position_result) {
  return out << "(road_pos:" << road_position_result.road_position
             << ", nearest_pos: " << road_position_result.nearest_position
             << ", distance: " << road_position_result.distance << ")";
}

// Serializes `state_type` into `out`.
std::ostream& operator<<(std::ostream& out, const maliput::api::rules::RightOfWayRule::State::Type& state_type) {
  switch (state_type) {
    case maliput::api::rules::RightOfWayRule::State::Type::kGo:
      out << "go";
      break;
    case maliput::api::rules::RightOfWayRule::State::Type::kStop:
      out << "stop";
      break;
    case maliput::api::rules::RightOfWayRule::State::Type::kStopThenGo:
      out << "stop then go";
      break;
    default:
      out << "unknown";
      break;
  }
  return out;
}

// Serializes `state` into `out`.
std::ostream& operator<<(std::ostream& out, const maliput::api::rules::RightOfWayRule::State& state) {
  out << "State(id: " << state.id().string() << ", type: '" << state.type() << "'"
      << ", yield group: [";
  for (const auto& right_of_way_rule_id : state.yield_to()) {
    out << right_of_way_rule_id.string() << ", ";
  }
  out << "])";
  return out;
}

// Serializes `s_range` into `out`.
std::ostream& operator<<(std::ostream& out, const maliput::api::SRange& s_range) {
  return out << "[" << s_range.s0() << ", " << s_range.s1() << "]";
}

// Serializes `lane_s_range` into `out`.
std::ostream& operator<<(std::ostream& out, const maliput::api::LaneSRange& lane_s_range) {
  return out << "Range(lane_id: " << lane_s_range.lane_id().string() << ", s_range:" << lane_s_range.s_range() << ")";
}

// Serializes `lane_s_route` into `out`.
std::ostream& operator<<(std::ostream& out, const maliput::api::LaneSRoute& lane_s_route) {
  out << "Route(ranges: [";
  for (const auto& range : lane_s_route.ranges()) {
    out << range << ", ";
  }
  return out << "])";
}

// Serializes `zone_type` into `out`.
std::ostream& operator<<(std::ostream& out, const maliput::api::rules::RightOfWayRule::ZoneType& zone_type) {
  switch (zone_type) {
    case maliput::api::rules::RightOfWayRule::ZoneType::kStopExcluded:
      out << "stop excluded";
      break;
    case maliput::api::rules::RightOfWayRule::ZoneType::kStopAllowed:
      out << "stop allowed";
      break;
    default:
      out << "unknown";
      break;
  }
  return out;
}

/////////////////////////////////////////////////
void RoadNetworkQuery::FindRoadPositions(const maliput::api::InertialPosition& inertial_position, double radius) {
  const std::vector<maliput::api::RoadPositionResult> results =
      rn_->road_geometry()->FindRoadPositions(inertial_position, radius);

  (*out_) << "FindRoadPositions(inertial_position:" << inertial_position << ", radius: " << radius << ")" << std::endl;
  for (const maliput::api::RoadPositionResult& result : results) {
    (*out_) << "              : Result: " << result << std::endl;
  }
}

/////////////////////////////////////////////////
void RoadNetworkQuery::ToInertialPosition(const maliput::api::LaneId& lane_id,
                                          const maliput::api::LanePosition& lane_position) {
  const maliput::api::Lane* lane = rn_->road_geometry()->ById().GetLane(lane_id);

  if (lane == nullptr) {
    (*out_) << "              : Result: Could not find lane. " << std::endl;
    return;
  }

  const maliput::api::InertialPosition inertial_position = lane->ToInertialPosition(lane_position);

  (*out_) << "(" << lane_id.string() << ")->ToInertialPosition(lane_position: " << lane_position << ")" << std::endl;
  (*out_) << "              : Result: inertial_position:" << inertial_position << std::endl;

  const maliput::api::RoadPositionResult result = rn_->road_geometry()->ToRoadPosition(inertial_position);

  (*out_) << "              : Result round_trip inertial_position" << result.nearest_position
          << ", with distance: " << result.distance << std::endl;
  (*out_) << "              : RoadPosition: " << result.road_position << std::endl;
}

/////////////////////////////////////////////////
void RoadNetworkQuery::ToSegmentPosition(const maliput::api::LaneId& lane_id,
                                         const maliput::api::InertialPosition& inertial_position) {
  const maliput::api::Lane* lane = rn_->road_geometry()->ById().GetLane(lane_id);
  if (lane == nullptr) {
    (*out_) << "              : Result: Could not find lane. " << std::endl;
    return;
  }

  const maliput::api::LanePositionResult result = lane->ToSegmentPosition(inertial_position);

  (*out_) << "(" << lane_id.string() << ")->ToSegmentPosition(inertial_position: " << inertial_position << ")"
          << std::endl;
  (*out_) << "              : Result: lane_pos:" << result.lane_position << ", nearest_pos: " << result.nearest_position
          << ", with distance: " << result.distance << std::endl;
}

/////////////////////////////////////////////////
void RoadNetworkQuery::ToRoadPosition(const maliput::api::InertialPosition& inertial_position) {
  const maliput::api::RoadPositionResult result = rn_->road_geometry()->ToRoadPosition(inertial_position, std::nullopt);

  (*out_) << "ToRoadPosition(inertial_position: " << inertial_position << ")" << std::endl;
  (*out_) << "              : Result: nearest_pos:" << result.nearest_position << " with distance: " << result.distance
          << std::endl;
  (*out_) << "                RoadPosition: " << result.road_position << std::endl;
}

/////////////////////////////////////////////////
void RoadNetworkQuery::GetState(const maliput::api::rules::Rule::State& state) {
  const int severity = state.severity;
  const auto& related_rules = state.related_rules;

  (*out_) << ", severity" << severity;
  for (const auto& related_rule : related_rules) {
    std::string rule_name = related_rule.first;
    (*out_) << ", related rule name: " << rule_name;
    (*out_) << ", ids [";
    const auto& ids = related_rule.second;
    for (const auto& id : ids) {
      (*out_) << id.string() << ", ";
    }
    (*out_) << "]";
  }

  const auto& related_unique_ids = state.related_unique_ids;
  for (const auto& related_unique_id : related_unique_ids) {
    std::string related_unique_id_name = related_unique_id.first;
    (*out_) << ", related unique id name: " << related_unique_id_name;
    (*out_) << ", unique ids [";
    const auto& unique_ids = related_unique_id.second;
    for (const auto& unique_id : unique_ids) {
      (*out_) << unique_id.string() << ", ";
    }
    (*out_) << "]";
  }
}

/////////////////////////////////////////////////
void RoadNetworkQuery::GetDiscreteValue(const maliput::api::LaneId& lane_id) {
  const maliput::api::rules::RoadRulebook::QueryResults query_result = FindRulesFor(lane_id);

  const int n_rules = static_cast<int>(query_result.discrete_value_rules.size());
  if (n_rules > 0) {
    for (const auto& discrete_value_rule : query_result.discrete_value_rules) {
      const auto& values = discrete_value_rule.second.states();
      for (const auto& value : values) {
        const std::string val = value.value;
        (*out_) << "              : Result: Rule (id: " << discrete_value_rule.first.string();
        (*out_) << ", value: " << val;
        this->GetState(value);
        (*out_) << ")" << std::endl;
      }
    }
  } else {
    (*out_) << "              : Result: There are no discrete value rules "
            << "found for this lane" << std::endl;
  }
  (*out_) << std::endl;
}

/////////////////////////////////////////////////
void RoadNetworkQuery::GetRangeValue(const maliput::api::LaneId& lane_id) {
  const maliput::api::rules::RoadRulebook::QueryResults query_result = FindRulesFor(lane_id);

  const int n_rules = static_cast<int>(query_result.range_value_rules.size());
  if (n_rules > 0) {
    for (const auto& range_value_rule : query_result.range_value_rules) {
      const auto& ranges = range_value_rule.second.states();
      for (const auto& range : ranges) {
        const std::string description = range.description;
        const double min = range.min;
        const double max = range.max;
        (*out_) << "              : Result: Rule (id: " << range_value_rule.first.string();
        (*out_) << ", description: " << description << ", min: " << min << ", max: " << max;
        this->GetState(range);
        (*out_) << ")" << std::endl;
      }
    }
  } else {
    (*out_) << "              : Result: There are no discrete value rules "
            << "found for this lane" << std::endl;
  }
  (*out_) << std::endl;
}

/////////////////////////////////////////////////
void RoadNetworkQuery::GetMaxSpeedLimit(const maliput::api::LaneId& lane_id) {
  const maliput::api::rules::RoadRulebook::QueryResults query_result = FindRulesFor(lane_id);

  const int n_speed_limits = static_cast<int>(query_result.speed_limit.size());
  if (n_speed_limits > 0) {
    double max_speed = query_result.speed_limit.begin()->second.max();
    maliput::api::rules::SpeedLimitRule::Id max_speed_id = query_result.speed_limit.begin()->first;
    for (const auto& speed_val : query_result.speed_limit) {
      const double max_speed_cur = speed_val.second.max();
      if (max_speed_cur < max_speed) {
        max_speed = max_speed_cur;
        max_speed_id = speed_val.first;
      }
    }

    (*out_) << "Speed limit (" << max_speed_id.string() << "): " << max_speed << " m/s" << std::endl;
  } else {
    (*out_) << "There is no speed limit found for this lane" << std::endl;
  }
}

/////////////////////////////////////////////////
void RoadNetworkQuery::GetDirectionUsage(const maliput::api::LaneId& lane_id) {
  const maliput::api::rules::RoadRulebook::QueryResults query_result = FindRulesFor(lane_id);

  const int n_rules = static_cast<int>(query_result.direction_usage.size());
  const std::vector<std::string> direction_usage_names = DirectionUsageRuleNames();

  if (n_rules > 0) {
    for (const auto& direction_rule : query_result.direction_usage) {
      const auto& states = direction_rule.second.states();
      for (const auto& state : states) {
        const int state_type = int(state.second.type());
        if (state_type < 0 || state_type >= int(direction_usage_names.size())) {
          (*out_) << "              : Result: Invalid direction usage rule. " << std::endl;
          return;
        }

        (*out_) << "              : Result: Rule (" << direction_rule.first.string()
                << "): " << direction_usage_names[state_type] << std::endl;
      }
    }
  } else {
    (*out_) << "              : Result: There are no direction usage rules "
            << "found for this lane" << std::endl;
  }
}

/////////////////////////////////////////////////
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
void RoadNetworkQuery::GetRightOfWay(const maliput::api::LaneSRange& lane_s_range) {
  const maliput::api::rules::RoadRulebook::QueryResults results = rn_->rulebook()->FindRules({lane_s_range}, 0.);
  maliput::api::rules::RightOfWayRuleStateProvider* right_of_way_rule_state_provider =
      rn_->right_of_way_rule_state_provider();
  (*out_) << "Right of way for " << lane_s_range << ":" << std::endl;
  for (const auto& rule : results.right_of_way) {
    (*out_) << "    Rule(id: " << rule.first.string() << ", zone: " << rule.second.zone() << ", zone-type: '"
            << rule.second.zone_type() << "'";
    if (!rule.second.is_static()) {
      (*out_) << ", states: [";
      for (const auto& entry : rule.second.states()) {
        (*out_) << entry.second << ", ";
      }
      (*out_) << "]";
      const auto rule_state_result = right_of_way_rule_state_provider->GetState(rule.first);
      if (rule_state_result.has_value()) {
        auto it = rule.second.states().find(rule_state_result->state);
        MALIPUT_THROW_UNLESS(it != rule.second.states().end());
        (*out_) << ", current_state: " << it->second;
      }
    } else {
      (*out_) << ", current_state: " << rule.second.static_state();
    }
    (*out_) << ", static: " << (rule.second.is_static() ? "yes" : "no") << ")" << std::endl << std::endl;
  }
}
#pragma GCC diagnostic pop

/////////////////////////////////////////////////
void RoadNetworkQuery::GetPhaseRightOfWay(const maliput::api::rules::PhaseRing::Id& phase_ring_id,
                                          const maliput::api::rules::Phase::Id& phase_id) {
  const maliput::api::rules::PhaseRingBook* phase_ring_book = rn_->phase_ring_book();
  if (phase_ring_book == nullptr) {
    (*out_) << "Road network has no phase ring book" << std::endl;
    return;
  }

  const maliput::api::rules::RoadRulebook* road_rule_book = rn_->rulebook();
  if (road_rule_book == nullptr) {
    (*out_) << "Road network has no road rule book" << std::endl;
    return;
  }

  std::optional<maliput::api::rules::PhaseRing> phase_ring = phase_ring_book->GetPhaseRing(phase_ring_id);
  if (!phase_ring.has_value()) {
    (*out_) << "'" << phase_ring_id.string() << "' is not a known phase ring" << std::endl;
    return;
  }

  auto it = phase_ring->phases().find(phase_id);
  if (it == phase_ring->phases().end()) {
    (*out_) << "'" << phase_id.string() << "' is not a phase in phase ring '" << phase_ring_id.string() << "'"
            << std::endl;
    return;
  }
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  const maliput::api::rules::Phase& phase = it->second;
  (*out_) << "Right of way for " << phase_id.string() << ":" << std::endl;
  for (const auto& rule_id_to_rule_state_id : phase.rule_states()) {
    const maliput::api::rules::RightOfWayRule rule = road_rule_book->GetRule(rule_id_to_rule_state_id.first);
    const maliput::api::rules::RightOfWayRule::State& rule_state = rule.states().at(rule_id_to_rule_state_id.second);
    (*out_) << "    Rule(id: " << rule.id().string() << ", zone: " << rule.zone() << ", zone-type: '"
            << rule.zone_type() << "'"
            << ", current_state: " << rule_state << ", static: " << (rule.is_static() ? "yes" : "no") << ")"
            << std::endl
            << std::endl;
  }
}
#pragma GCC diagnostic pop

/////////////////////////////////////////////////
maliput::api::rules::RoadRulebook::QueryResults RoadNetworkQuery::FindRulesFor(const maliput::api::LaneId& lane_id) {
  const maliput::api::Lane* lane = rn_->road_geometry()->ById().GetLane(lane_id);
  if (lane == nullptr) {
    std::cerr << " Could not find lane. " << std::endl;
    return maliput::api::rules::RoadRulebook::QueryResults();
  }

  const maliput::api::SRange s_range(0., lane->length());
  const maliput::api::LaneSRange lane_s_range(lane->id(), s_range);
  const std::vector<maliput::api::LaneSRange> lane_s_ranges(1, lane_s_range);

  return rn_->rulebook()->FindRules(lane_s_ranges, 0.);
}

MaliputViewerModel::MaliputViewerModel(std::unique_ptr<maliput::api::RoadNetwork> _roadNetwork)
    : roadNetwork(std::move(_roadNetwork)) {
  MALIPUT_THROW_UNLESS(roadNetwork != nullptr);
  const maliput::api::RoadGeometry* rg = roadNetwork->road_geometry();
  ignmsg << "Loading RoadGeometry meshes of " << rg->id().string() << std::endl;
  maliput::utility::ObjFeatures features;
  features.off_grid_mesh_generation = true;
  const maliput::utility::RoadGeometryMesh geoMeshes = BuildRoadGeometryMesh(rg, features);
  ignmsg << "Meshes loaded." << std::endl;
  this->ConvertRoadGeometryMeshes(geoMeshes);
  ignmsg << "Meshes converted to ignition type." << std::endl;
  this->GenerateLabels();
  ignmsg << "Labels generated." << std::endl;
}

/////////////////////////////////////////////////
const std::map<std::string, std::unique_ptr<MaliputMesh>>& MaliputViewerModel::Meshes() const {
  return this->maliputMeshes;
}

/////////////////////////////////////////////////
const std::map<std::string, MaliputLabel>& MaliputViewerModel::Labels() const { return this->labels; }

/////////////////////////////////////////////////
void MaliputViewerModel::ConvertMeshes(
    const std::map<std::string, std::pair<maliput::utility::mesh::GeoMesh, maliput::utility::Material>>& _geoMeshes) {
  for (const auto& it : _geoMeshes) {
    auto maliputMesh = std::make_unique<MaliputMesh>();
    // Converts from drake to ignition mesh and sets the state.
    maliputMesh->mesh = maliput::viz::mesh::Convert(it.first, it.second.first);
    if (maliputMesh->mesh == nullptr) {
      ignmsg << "Skipping mesh [" << it.first << "] because it is empty.\n";
      maliputMesh->enabled = false;
      maliputMesh->visible = false;
    } else {
      ignmsg << "Enabling mesh [" << it.first << "].\n";
      maliputMesh->enabled = true;
      maliputMesh->visible = true;
    }
    // Retrieves the material
    maliputMesh->material = std::make_unique<maliput::utility::Material>(it.second.second);

    this->maliputMeshes[it.first] = std::move(maliputMesh);
  }
}

void MaliputViewerModel::ConvertRoadGeometryMeshes(const maliput::utility::RoadGeometryMesh& _geoMeshes) {
  ConvertMeshes(_geoMeshes.asphalt_mesh);
  ConvertMeshes(_geoMeshes.grayed_asphalt_mesh);
  ConvertMeshes(_geoMeshes.hbounds_mesh);
  ConvertMeshes(_geoMeshes.lane_lane_mesh);
  ConvertMeshes(_geoMeshes.lane_marker_mesh);
  ConvertMeshes(_geoMeshes.lane_grayed_lane_mesh);
  ConvertMeshes(_geoMeshes.lane_grayed_marker_mesh);
  ConvertMeshes(_geoMeshes.branch_point_mesh);
}

///////////////////////////////////////////////////////
namespace {

// \brief An offset to be applied over lane's height to avoid meshes
// intersection.
static const double kLaneHeightOffset{7.};

// \brief An offset to be applied over branch point's height to avoid meshes
// intersection.
static const double kBranchPointHeightOffset{3.};

// \brief Returns the world position of @p laneEnd.
// \param laneEnd The LaneEnd to get the position from.
// \return An ignition::math::Vector3d with the world position of
// @p laneEnd.lane at @p laneEnd.end extent.
ignition::math::Vector3d LaneEndWorldPosition(const maliput::api::LaneEnd& laneEnd) {
  const double s_position = laneEnd.end == maliput::api::LaneEnd::Which::kStart ? 0. : laneEnd.lane->length();
  const maliput::api::InertialPosition position = laneEnd.lane->ToInertialPosition({s_position, 0., 0.});
  return {position.x(), position.y(), position.z() + kBranchPointHeightOffset};
}

// \brief Builds a MaliputLabel from the branch point @p bp.
// \details Sets label's position to one of the lane ends that @p bp has.
// Label's text is @p bp's ID.
// \param bp A BranchPoint to build a label from.
// \return A MaliputLabel with @p bp's information.
MaliputLabel LabelFor(const maliput::api::BranchPoint& bp) {
  MaliputLabel label;
  label.text = bp.id().string();
  label.labelType = MaliputLabelType::kBranchPoint;
  if (bp.GetASide() && bp.GetASide()->size() != 0) {
    label.position = LaneEndWorldPosition(bp.GetASide()->get(0));
  } else if (bp.GetBSide() && bp.GetBSide()->size() != 0) {
    label.position = LaneEndWorldPosition(bp.GetBSide()->get(0));
  } else {
    ignerr << "Maliput's BranchPoint [" << bp.id().string() << "] has two empty sides." << std::endl;
  }
  return label;
}

// \brief Builds a MaliputLabel from the lane @p lane.
// \details Sets label's position to half the @p lane's length world position.
// Label's text is @p lane's ID.
// \param lane A Lane to build a label from.
// \return A MaliputLabel with @p lane's information.
MaliputLabel LabelFor(const maliput::api::Lane& lane) {
  MaliputLabel label;
  label.text = lane.id().string();
  const maliput::api::InertialPosition position = lane.ToInertialPosition({lane.length() / 2., 0., 0.});
  label.position.Set(position.x(), position.y(), position.z() + kLaneHeightOffset);
  label.labelType = MaliputLabelType::kLane;
  return label;
}
}  // namespace

///////////////////////////////////////////////////////
void MaliputViewerModel::GenerateLabels() {
  // Traverses branch points to generate labels for them.
  const maliput::api::RoadGeometry* rg = roadNetwork->road_geometry();
  for (int i = 0; i < rg->num_branch_points(); ++i) {
    const maliput::api::BranchPoint* bp = rg->branch_point(i);
    this->labels["branch_point_" + bp->id().string()] = LabelFor(*bp);
  }

  // Traverses lanes to generate labels for them.
  const auto lanes = rg->ById().GetLanes();
  for (const auto& lane : lanes) {
    this->labels["lane_" + lane.first.string()] = LabelFor(*lane.second);
  }
}

///////////////////////////////////////////////////////
void MaliputViewerModel::SetLayerState(const std::string& _key, bool _isVisible) {
  if (this->maliputMeshes.find(_key) == this->maliputMeshes.end()) {
    igndbg << _key + " does not exist in maliputMeshes." << std::endl;
    return;
  }
  this->maliputMeshes[_key]->visible = _isVisible;
}

///////////////////////////////////////////////////////
void MaliputViewerModel::SetTextLabelState(MaliputLabelType _type, bool _isVisible) {
  for (auto& i : this->labels) {
    if (_type == i.second.labelType) {
      i.second.visible = _isVisible;
    }
  }
}

///////////////////////////////////////////////////////
void MaliputViewerModel::SetTextLabelState(const std::string& _key, bool _isVisible) {
  if (this->labels.find(_key) == this->labels.end()) {
    igndbg << _key + " does not exist in labels." << std::endl;
    return;
  }
  this->labels[_key].visible = _isVisible;
}

///////////////////////////////////////////////////////
const maliput::api::Lane* MaliputViewerModel::GetLaneFromWorldPosition(const ignition::math::Vector3d& _position) {
  const maliput::api::RoadGeometry* rg = this->roadNetwork->road_geometry();
  MALIPUT_THROW_UNLESS(rg != nullptr);
  const maliput::api::InertialPosition inertial_pos(_position.X(), _position.Y(), _position.Z());
  return rg->ToRoadPosition(inertial_pos).road_position.lane;
}

///////////////////////////////////////////////////////
const maliput::api::RoadPositionResult MaliputViewerModel::GetRoadPositionResult(
    const ignition::math::Vector3d& _position) {
  const maliput::api::RoadGeometry* rg = this->roadNetwork->road_geometry();
  MALIPUT_THROW_UNLESS(rg != nullptr);
  const maliput::api::InertialPosition inertial_pos(_position.X(), _position.Y(), _position.Z());
  return rg->ToRoadPosition(inertial_pos);
}

///////////////////////////////////////////////////////
const maliput::api::Lane* MaliputViewerModel::GetLaneFromId(const std::string& _id) {
  const maliput::api::RoadGeometry* rg = this->roadNetwork->road_geometry();
  MALIPUT_THROW_UNLESS(rg != nullptr);
  return rg->ById().GetLane(maliput::api::LaneId(_id));
}

///////////////////////////////////////////////////////
std::vector<const maliput::api::rules::TrafficLight*> MaliputViewerModel::GetTrafficLights() const {
  return this->roadNetwork != nullptr ? this->roadNetwork->traffic_light_book()->TrafficLights()
                                      : std::vector<const maliput::api::rules::TrafficLight*>();
}

///////////////////////////////////////////////////////
maliput::api::rules::BulbStates MaliputViewerModel::GetBulbStates(const std::string& _phaseRingId,
                                                                  const std::string& _phaseId) const {
  if (this->roadNetwork && !_phaseRingId.empty() && !_phaseId.empty()) {
    const maliput::api::rules::PhaseRingBook* phase_ring_book = this->roadNetwork->phase_ring_book();
    const std::optional<maliput::api::rules::PhaseRing> phase_ring =
        phase_ring_book->GetPhaseRing(maliput::api::rules::PhaseRing::Id(_phaseRingId));
    MALIPUT_THROW_UNLESS(phase_ring.has_value());
    const std::unordered_map<maliput::api::rules::Phase::Id, maliput::api::rules::Phase>& phases = phase_ring->phases();
    const auto phase = phases.find(maliput::api::rules::Phase::Id(_phaseId));
    MALIPUT_THROW_UNLESS(phase != phases.end());
    const std::optional<maliput::api::rules::BulbStates>& bulb_states = phase->second.bulb_states();
    if (bulb_states.has_value()) {
      return *bulb_states;
    }
  }
  return maliput::api::rules::BulbStates();
}

///////////////////////////////////////////////////////
MaliputLabelType maliput::viz::FromString(const std::string& _type) {
  if (_type == "lane_text_label") {
    return MaliputLabelType::kLane;
  } else if (_type == "branchpoint_text_label") {
    return MaliputLabelType::kBranchPoint;
  }
  throw std::runtime_error(std::string("_type = [") + _type + std::string(" ] is not \"lane_text_label\" nor ") +
                           std::string("\"branchpoint_text_label\"."));
}
