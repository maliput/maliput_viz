// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2021-2022, Toyota Research Institute. All rights reserved.
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

#include <atomic>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>

#include <QStandardItem>
#include <ignition/common/MouseEvent.hh>
#include <ignition/gui/Plugin.hh>
#include <ignition/rendering/RayQuery.hh>
#include <ignition/rendering/RenderEngine.hh>
#include <ignition/rendering/RenderTypes.hh>
#include <ignition/rendering/Scene.hh>
#include <maliput/api/road_geometry.h>

#include "arrow_mesh.hh"
#include "maliput_backend_selection.hh"
#include "selector.hh"
#include "traffic_light_manager.hh"

namespace maliput {
namespace viz {

/// Model for describing a tree view for the phase rings and their phases.
/// Treeview example:
///
/// PhaseRing#1
///          |_____ PhaseA#1
///          |_____ PhaseA#2
/// PhaseRing#2
///          |_____ PhaseB#1
///          |_____ PhaseB#2
///
///
class PhaseTreeModel : public QStandardItemModel {
  Q_OBJECT
 public:
  PhaseTreeModel(QObject* parent);

  /// Adds a new row to the treeview to group all the phases of a new phase ring.
  /// \param[in] _phaseRingName Unique name of the new phase ring.
  void AddPhaseRing(const std::string& _phaseRingName);

  /// Adds a new row under the @p _phaseRingName row to indicate a new phase.
  /// \param[in] _phaseName Unique name in the phase ring for the new phase.
  /// \param[in] _phaseRingName Name of the existing phase ring where the new phase should be added.
  /// \note The phase ring should be previously added using #AddPhaseRing method.
  void AddPhaseToPhaseRing(const std::string& _phaseName, const std::string& _phaseRingName);

  /// \param[in] _phaseRingItem QStandardItem pointer.
  /// \returns true if the item corresponds to a valid previously added phase ring.
  bool IsPhaseRingItem(const QStandardItem* _phaseRingItem) const;

  /// \param[in] _phaseItem Phase item.
  /// \param[in] _phaseRingItem Phase ring item.
  /// \returns true when the phase item corresponds to a valid previously added phase under the provided phase ring.
  bool IsPhaseItem(const QStandardItem* _phaseItem, const QStandardItem* _phaseRingItem) const;

  /// Clear the model.
  void Clear();

 private:
  /// Holds a phase ring item for the tree view.
  struct PhaseRing {
    /// Item pointing to the phase ring.
    QStandardItem* phaseRingItem{nullptr};

    /// Holds a map for the phases items.
    /// The key is the id of the phase.
    std::map<std::string, QStandardItem*> phaseIdAndItem;
  };
  /// Holds a map for the phase rings items.
  /// The key is the id of the phase ring.
  std::map<std::string, PhaseRing> phaseRings;
};

/// \brief Loads a road geometry out of a xodr file or a yaml file.
///        Meshes are created and displayed in the scene provided by Scene3D plugin.
///
class MaliputViewerPlugin : public ignition::gui::Plugin {
  Q_OBJECT

  /// Property used to load the default state of layers visualization in its correspondant UI's checkboxes.
  Q_PROPERTY(QList<bool> layerCheckboxes READ LayerCheckboxes NOTIFY LayerCheckboxesChanged)

  /// Property used to load the default state of labels visualization in its correspondant UI's checkboxes.
  Q_PROPERTY(QList<bool> labelCheckboxes READ LabelCheckboxes NOTIFY LabelCheckboxesChanged)

  /// Property used to load the lanes id in the correspondant UI's table.
  Q_PROPERTY(QStringList listLanes READ ListLanes NOTIFY ListLanesChanged)

  /// Property used to load the rules in the correspondant UI's area.
  Q_PROPERTY(QString rulesList READ RulesList NOTIFY RulesListChanged)

  /// Property used to load the info about the surface clicked in the correspondant UI's area.
  Q_PROPERTY(QString laneInfo READ LaneInfo NOTIFY LaneInfoChanged)

 public:
  /// \brief Default constructor.
  MaliputViewerPlugin();

  /// Called by Ignition GUI when plugin is instantiated.
  /// \param[in] _pluginElem XML configuration for this plugin.
  void LoadConfig(const tinyxml2::XMLElement* _pluginElem) override;

  /// Called when a new RoadNetwork is loaded to load the ids of the lanes into the table.
  Q_INVOKABLE QStringList ListLanes() const;

  /// Called when a new lane is selected to load the lane's rules into the UI.
  Q_INVOKABLE QString RulesList() const;

  /// Called when a lane is clicked. Update info related to the clicked surface.
  Q_INVOKABLE QString LaneInfo() const;

  /// Called when a new RoadNetwork is loaded to default the checkboxes' state
  /// in the layers selection panel for the meshes.
  Q_INVOKABLE QList<bool> LayerCheckboxes() const;

  /// Called when a new RoadNetwork is loaded to default the checkboxes' state
  /// in the label selection panel.
  Q_INVOKABLE QList<bool> LabelCheckboxes() const;

 signals:
  /// \brief Signal emitted to update the id of the lanes in the table.
  void ListLanesChanged();

  /// \brief Signal emitted to update the rules of the selected lane.
  void RulesListChanged();

  /// \brief Signal emitted to update the info about the clicked lane.
  void LaneInfoChanged();

  /// \brief Signal emitted to reset the checkboxes' state for the layers visualization
  ///        when a new RoadNetwork is loaded.
  void LayerCheckboxesChanged();

  /// \brief Signal emitted to reset the checkboxes' state for the label visualization
  ///        when a new RoadNetwork is loaded.
  void LabelCheckboxesChanged();

  /// \brief Signal emitted to indicate which lane id in the table should be highlighted.
  void tableLaneIdSelection(int _index);

 protected:
  /// \brief Filters ignition::gui::events::LeftClickToScene to get the clicks events.
  ///        Filters ignition::gui::events::Render events to update the meshes and labels of the roads and the animation
  ///        of the arrow mesh.
  /// \details To make this method be called by Qt Event System, install the event filter in target object.
  ///          \see QObject::installEventFilter() method.
  bool eventFilter(QObject* _obj, QEvent* _event) override;

  /// @brief Timer event callback which handles the logic to get the scene.
  void timerEvent(QTimerEvent* _event) override;

 protected slots:
  /// \brief Clears the visualizer and updates the GUI with meshes and labels using the new RoadNetwork.
  void OnNewRoadNetwork();

  /// \brief Change the visibility of the layers.
  /// \param[in] _layer The layer to change its visibility.
  /// \param[in] _state The state of the visibility checkbox.
  void OnNewMeshLayerSelection(const QString& _layer, bool _state);

  /// \brief Change the visibility of the labels.
  /// \param[in] _label Name of the label.
  /// \param[in] _state The state of the visibility checkbox.
  void OnNewTextLabelSelection(const QString& _label, bool _state);

  /// \brief Manages the selection of lanes id from the table.
  /// \param[in] _index Correspondant to the position of the row in the table.
  void OnTableLaneIdSelection(int _index);

  /// \brief Manages the selection of phase from the phase ring tree view.
  /// \param[in] _index Correspondant to the position of the row in the tree view.
  void OnPhaseSelection(const QModelIndex& _index);

 private:
  /// @brief The scene name.
  static constexpr char const* kSceneName = "scene";

  /// @brief The rendering engine name.
  static constexpr char const* kEngineName = "ogre";

  /// @brief The period in milliseconds of the timer to try to get the scene.
  static constexpr int kTimerPeriodInMs{500};

  /// \brief Holds a maliput::api::RoadPositionResult which results from the
  ///        mapped scene click position into the Inertial frame and the
  ///        distance to the camera.
  /// \details Note that this wrapper may hold a std::nullopt and the dirty flag
  ///          is true. It helps to change the selection state of the lanes.
  class RoadPositionResultValue {
   public:
    /// \brief Default constructor.
    RoadPositionResultValue() = default;

    /// \brief Constructs a RoadPositionResultValue from @p _other
    ///        maliput::api::RoadPositionResult and @p _distance to the camera
    ///        position in the scene.
    /// \param _other A maliput::api::RoadPositionResult.
    /// \param _distance The distance between the camera and the intersected
    ///        point in the scene.
    explicit RoadPositionResultValue(const maliput::api::RoadPositionResult& _other, double _distance)
        : value(_other), distance(_distance), dirty(true) {}

    /// \brief Sets the dirty flag.
    /// \param _isDirty Whether the value is dirty or not.
    void SetDirty(bool _isDirty) { dirty = _isDirty; }

    /// \return The distance between the clicked scene position and the camera.
    double Distance() const { return distance; }

    /// \return True when the value is dirty.
    bool IsDirty() const { return dirty; }

    /// \return The stored optional with maliput::api::RoadPositionResult.
    const std::optional<maliput::api::RoadPositionResult>& Value() const { return value; }

   private:
    std::optional<maliput::api::RoadPositionResult> value{std::nullopt};
    double distance{0.};
    bool dirty{false};
  };

  /// \brief Fills a material for a lane label.
  /// \param[in] _material Material to be filled.
  static void CreateLaneLabelMaterial(ignition::rendering::MaterialPtr& _material);

  /// \brief Fills a material for a branch point label.
  /// \param[in] _material Material to be filled.
  static void CreateBranchPointLabelMaterial(ignition::rendering::MaterialPtr& _material);

  /// \brief Fills @p _ignitionMaterial with @p _maliputMaterial properties.
  /// \param[in] _maliputMaterial Material properties.
  /// \param[in] _ignitionMaterial A valid ignition::rendering::MaterialPtr.
  /// \return True when @p _maliputMaterial is valid and @p _ignitionMaterial
  /// can be filled.
  static bool FillMaterial(const maliput::utility::Material* _maliputMaterial,
                           ignition::rendering::MaterialPtr& _ignitionMaterial);

  /// \brief Holds flags used to render meshes.
  struct RenderMeshesOption {
    /// \brief Set both #executeMeshRendering and #executeLabelRendering to true;
    void RenderAll() {
      executeMeshRendering = true;
      executeLabelRendering = true;
    }
    /// Indicates whether meshes must be rendered.
    bool executeMeshRendering{false};
    /// Indicates whether labels must be rendered.
    bool executeLabelRendering{false};
  } renderMeshesOption;

  /// \brief Builds visuals for each mesh inside @p _maliputMeshes that is
  /// enabled.
  /// \param[in] _maliputMeshes A map of meshes to render.
  void RenderRoadMeshes(const std::map<std::string, std::unique_ptr<MaliputMesh>>& _maliputMeshes);

  /// \brief Builds visuals for each label inside @p _labels that is enabled.
  /// \param[in] _labels A map of labels to render.
  void RenderLabels(const std::map<std::string, MaliputLabel>& _labels);

  /// \brief Clears all the references to text labels, meshes and the scene.
  void Clear();

  /// \brief Set up the scene modifying the light and instantiating the ArrowMesh, Selector and TrafficLightManager.
  void SetUpScene();

  /// \brief Handles the left click mouse event and populates roadPositionResultValue.
  /// \param[in] _sceneInertialPosition The scene (Inertial) frame position to
  ///            map into the RoadGeometry.
  /// \param[in] _distance The distance between the camera and @p _sceneInertialPosition.
  void MouseClickHandler(const ignition::math::Vector3d& _sceneInertialPosition, double _distance);

  /// \brief Handles the left click mouse event and populates roadPositionResultValue.
  void UpdateLaneSelectionOnLeftClick();

  /// \brief Filters by title all the children of the parent plugin.
  /// \param _pluginTitle Title of the ignition::gui::plugin.
  /// \return A pointer to the plugin if found, nullptr otherwise.
  ignition::gui::Plugin* FilterPluginsByTitle(const std::string& _pluginTitle);

  /// \brief Updates all stored visual defaults for the meshes and labels.
  /// \param[in] _key The key indicating which default to update.
  /// \param[in] _newValue The new value to set the default value.
  void UpdateObjectVisualDefaults(const std::string& _key, bool _newValue);

  /// \brief Updates a lane so that if it is selected then all meshes are on, but if it is
  /// not selected, all meshes are set to the default values.
  /// \param[in] _id The id of the lane.
  void UpdateLane(const std::string& _id);

  /// \brief Updates a branch point so that if it is selected then all meshes are on, but if it is
  /// not selected, all meshes are set to the default values.
  /// \param[in] _id The id of the branch point.
  void UpdateBranchPoint(const std::string& _id);

  /// \brief Updates all of the selected regions to the default mesh values.
  void UpdateSelectedLanesWithDefault();

  /// \brief Updates the list that holds the lanes id of the road network.
  void UpdateLaneList();

  /// \brief Updates the list that holds the lane's rules of the selected lane.
  void UpdateRulesList(const std::string& _laneId);

  /// \brief Updates the text related to the clicked surface if any.
  /// \param[in] _roadPositionResult Result of the mapped RoadPosition after a click in the scene.
  void UpdateLaneInfoArea(const maliput::api::RoadPositionResult& _roadPositionResult);

  /// Keys used by the checkbox logic to visualize different layers and by
  /// the default map #objectVisualDefaults.
  /// @{
  /// \brief Key used to detect an asphalt checkbox event.
  const std::string kAsphalt{"asphalt"};
  /// \brief Keyword used by the checkboxes to indicate the new default for all
  /// of the provided mesh.
  const std::string kAll{"all"};
  /// \brief Key for the marker mesh in the default map.
  const std::string kMarker{"marker"};
  /// \brief Key for the lane mesh in the default map.
  const std::string kLane{"lane"};
  /// \brief Key for the branch point mesh in the default map.
  const std::string kBranchPoint{"branch_point"};
  /// \brief Key used to identify labels from meshes.
  const std::string kLabels{"_labels"};
  /// \brief Key for the branch point label mesh in the default map.
  const std::string kBranchPointLabels{kBranchPoint + kLabels};
  /// \brief Key for the lane label mesh in the default map.
  const std::string kLaneLabels{kLane + kLabels};
  /// @}

  /// \brief Holds the map file path.
  std::string mapFile{""};

  /// \brief Holds the rule registry file path.
  std::string ruleRegistryFile{""};

  /// \brief Holds the road rulebook file path.
  std::string roadRulebookFile{""};

  /// \brief Holds the traffic light book file path.
  std::string trafficLightBookFile{""};

  /// \brief Holds the phase ring book file path.
  std::string phaseRingBookFile{""};

  /// \brief Holds the phase intersection book file path.
  std::string intersectionBookFile{""};

  /// \brief Holds the lanes id that are shown in the table.
  ///        The order in this collection will affect the order
  ///        that the lanes id are displayed in the table.
  QStringList listLanes{};

  /// \brief Holds the rules of the last selected lane that are displayed in the UI.
  QString rulesList{};

  /// \brief Holds the info about the clicked surface that is displayed in the UI.
  QString laneInfo{};

  /// @brief Triggers an event every `kTimerPeriodInMs` to try to get the scene.
  QBasicTimer timer;

  /// \brief Holds the model of the tree view table where the phases are listed.
  PhaseTreeModel phaseTreeModel{this};

  /// \brief Holds the current selected phase.
  std::pair<std::string, std::string> currentPhase{"" /* phaseId */, "", /* phaseRingId*/};

  /// \brief Root visual where all the child mesh visuals are added.
  ignition::rendering::VisualPtr rootVisual;

  /// \brief Map of mesh visual pointers.
  std::unordered_map<std::string, ignition::rendering::VisualPtr> meshes;

  /// \brief Map of text labels visual pointers.
  std::unordered_map<std::string, ignition::rendering::VisualPtr> textLabels;

  /// \brief A map that contains the default of the checkbox for meshes and labels.
  std::unordered_map<std::string, bool> objectVisualDefaults;

  /// \brief Holds a pointer to the scene.
  ignition::rendering::ScenePtr scene{nullptr};

  /// \brief Holds a pointer to the camera.
  ignition::rendering::CameraPtr camera{};

  /// \brief Manages the backend selection and road network loading.
  MaliputBackendSelection maliputBackendSelection{this};

  /// \brief Arrow that points the location clicked in the visualizer.
  std::unique_ptr<ArrowMesh> arrow;

  /// \brief Selector used for selecting clicked lanes in the visualizer.
  std::unique_ptr<Selector> selector;

  /// \brief Manager of traffic lights visualization.
  std::unique_ptr<TrafficLightManager> trafficLightManager;

  /// \brief Holds the RoadPositionResult upon a left click in the scene.
  RoadPositionResultValue roadPositionResultValue;

  /// \brief Indicates a RoadNetwork is already loaded.
  std::atomic<bool> isRoadNetworkLoaded{false};

  /// \brief Indicates a new RoadNetwork has been loaded.
  std::atomic<bool> newRoadNetwork{false};

  /// \brief Indicates the scene needs to be set-up.
  std::atomic<bool> setUpScene{false};
};

}  // namespace viz
}  // namespace maliput
