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
#include "maliput_viewer_plugin.hh"

#include <algorithm>
#include <vector>

#include <ignition/common/Console.hh>
#include <ignition/gui/Application.hh>
#include <ignition/gui/Conversions.hh>
#include <ignition/gui/GuiEvents.hh>
#include <ignition/gui/MainWindow.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/rendering/RenderEngine.hh>
#include <ignition/rendering/RenderingIface.hh>
#include <ignition/rendering/Text.hh>
#include <ignition/rendering/Visual.hh>
#include <maliput/common/maliput_throw.h>

#include "maliput_viz/flags/gflags.h"
#include "maliput_viz/tools/yaml_parser.h"

namespace maliput {
namespace viz {
namespace {

// \returns True when @p keyword is found in @p word.
bool FoundKeyword(const std::string& word, const std::string& keyword) {
  return word.find(keyword) != std::string::npos;
}

// \brief Obtains the id of a branchpoint or lane from the string used as it's
//        key value in the mesh map.
//        e.g: If @p keyword is "lane_1_0_1", it returns "1_0_1";
// \param[in] keyword The keyword that the lane or branchpoint is associated with in the
//                    mesh map.
// \returns The lane or branchpoint id.
std::string GetID(const std::string& keyword) {
  const std::size_t firstNum = keyword.find_first_of("0123456789");
  return keyword.substr(firstNum, keyword.length() - firstNum + 1);
  ;
}

// \brief Combines @p _type and @p _id to create a key with the same structure than
//        used in MaliputViewerModel for the mesh map.
//        e.g. : if @p _type is "lane" and @p _id is "1_0_1". It returns "lane_" + "1_0_1"
std::string FromIdToMapKey(const std::string& _type, const std::string& _id) { return _type + "_" + _id; }

}  // namespace

PhaseTreeModel::PhaseTreeModel(QObject* parent) : QStandardItemModel(parent) { setColumnCount(1); }

bool PhaseTreeModel::IsPhaseRingItem(const QStandardItem* _phaseRingItem) const {
  const auto phaseRing = phaseRings.find(_phaseRingItem->text().toStdString());
  return phaseRing != phaseRings.end();
}

bool PhaseTreeModel::IsPhaseItem(const QStandardItem* _phaseItem, const QStandardItem* _phaseRingItem) const {
  const std::string phaseRingItem{_phaseRingItem->text().toStdString()};
  const auto phaseRing = phaseRings.find(phaseRingItem);
  if (phaseRing == phaseRings.end()) {
    ignerr << "PhaseRing: " << phaseRingItem << " is missing." << std::endl;
    return false;
  }
  const std::map<std::string, QStandardItem*>& phasesMap = phaseRings.at(phaseRingItem).phaseIdAndItem;
  const auto phase = phasesMap.find(_phaseItem->text().toStdString());
  return phase != phasesMap.end();
}

void PhaseTreeModel::Clear() {
  removeRows(0, rowCount());
  phaseRings.clear();
}

void PhaseTreeModel::AddPhaseRing(const std::string& _phaseRingName) {
  if (phaseRings.find(_phaseRingName) != phaseRings.end()) {
    ignerr << "PhaseRing: " << _phaseRingName << " is repeated." << std::endl;
    return;
  }
  QStandardItem* newPhaseRing = new QStandardItem;
  newPhaseRing->setText(QString::fromStdString(_phaseRingName));
  invisibleRootItem()->appendRow(newPhaseRing);
  phaseRings.emplace(_phaseRingName, PhaseRing{newPhaseRing, {}});
}

void PhaseTreeModel::AddPhaseToPhaseRing(const std::string& _phaseName, const std::string& _phaseRingName) {
  auto phaseRing = phaseRings.find(_phaseRingName);
  if (phaseRing == phaseRings.end()) {
    ignerr << "Phase: " << _phaseName << "can't be added. PhaseRing: " << _phaseRingName << " is missing." << std::endl;
    return;
  }
  QStandardItem* newPhase = new QStandardItem;
  newPhase->setText(QString::fromStdString(_phaseName));
  if (phaseRing->second.phaseIdAndItem.find(_phaseName) != phaseRing->second.phaseIdAndItem.end()) {
    ignerr << "Phase: " << _phaseName << " is repeated in PhaseRing: " << _phaseRingName << std::endl;
    return;
  }
  phaseRing->second.phaseRingItem->appendRow(newPhase);
  phaseRing->second.phaseIdAndItem.emplace(_phaseName, newPhase);
}

MaliputViewerPlugin::MaliputViewerPlugin() : Plugin() {
  ignition::gui::App()->Engine()->rootContext()->setContextProperty(QString::fromStdString("PhaseTreeModel"),
                                                                    &phaseTreeModel);
  ignition::gui::App()->Engine()->rootContext()->setContextProperty(QString::fromStdString("MaliputBackendSelection"),
                                                                    &maliputBackendSelection);
}

QStringList MaliputViewerPlugin::ListLanes() const { return listLanes; }

QString MaliputViewerPlugin::RulesList() const { return rulesList; }

QString MaliputViewerPlugin::LaneInfo() const { return laneInfo; }

QList<bool> MaliputViewerPlugin::LayerCheckboxes() const {
  // Returns the checkboxes' state by default.
  return {true /* asphalt */,      true /* lane */,
          true /* marker */,       true /* h_bounds */,
          true /* branchpoint */,  false /* grayed_asphalt */,
          false /* grayed_lane */, false /* grayed_marker */};
}

QList<bool> MaliputViewerPlugin::LabelCheckboxes() const {
  // Returns the checkboxes' state by default.
  return {true /* lane */, true /* branch_point */};
}

void MaliputViewerPlugin::OnNewRoadNetwork() {
  Clear();
  UpdateLaneList();
  emit LayerCheckboxesChanged();
  emit LabelCheckboxesChanged();
  renderMeshesOption.RenderAll();

  // Get phases and update the list.
  const auto phaseRings = maliputBackendSelection.GetMaliputModel()->GetPhaseRings<std::string>();
  for (const auto& phaseRing : phaseRings) {
    phaseTreeModel.AddPhaseRing(phaseRing.first);
    for (const auto& phase : phaseRing.second) {
      phaseTreeModel.AddPhaseToPhaseRing(phase, phaseRing.first);
    }
  }

  isRoadNetworkLoaded = true;
  newRoadNetwork = true;
}

void MaliputViewerPlugin::UpdateLaneList() {
  std::vector<std::string> laneIds =
      maliputBackendSelection.GetMaliputModel()->GetAllLaneIds<std::vector<std::string>>();
  std::sort(laneIds.begin(), laneIds.end());
  listLanes.clear();
  std::for_each(laneIds.cbegin(), laneIds.cend(),
                [&](const std::string& _id) { listLanes.append(QString::fromStdString(_id)); });
  emit ListLanesChanged();
}

void MaliputViewerPlugin::UpdateObjectVisualDefaults(const std::string& _key, bool _newValue) {
  // Store the value the mesh should return to if not selected
  if (FoundKeyword(_key, kMarker)) {
    objectVisualDefaults[kMarker] = _newValue;
  } else if (FoundKeyword(_key, kLaneLabels)) {
    objectVisualDefaults[kLaneLabels] = _newValue;
  } else if (FoundKeyword(_key, kBranchPointLabels)) {
    objectVisualDefaults[kBranchPointLabels] = _newValue;
  } else if (FoundKeyword(_key, kLane)) {
    objectVisualDefaults[kLane] = _newValue;
  } else if (FoundKeyword(_key, kBranchPoint)) {
    objectVisualDefaults[kBranchPoint] = _newValue;
  }
}

void MaliputViewerPlugin::OnNewMeshLayerSelection(const QString& _layer, bool _state) {
  const std::string layer{_layer.toStdString()};
  const std::size_t all_keyword = layer.find(kAll);
  // If the keyword "all" is found, enable all of the parsed type.
  if (FoundKeyword(layer, kAll)) {
    const std::string keyword = layer.substr(0, all_keyword);
    UpdateObjectVisualDefaults(layer, _state);
    for (auto const& it : maliputBackendSelection.GetMaliputModel()->Meshes()) {
      if (FoundKeyword(it.first, keyword)) {
        const std::string id = GetID(it.first);
        // If the region is not selected, update with the default setting
        if (!selector->IsSelected(id)) {
          // Updates the model.
          maliputBackendSelection.GetMaliputModel()->SetLayerState(it.first, _state);
        }
      }
    }
  } else {
    maliputBackendSelection.GetMaliputModel()->SetLayerState(layer, _state);
    // If the asphalt is turned off, deselect all lanes.
    if (FoundKeyword(layer, kAsphalt) && !_state) {
      selector->DeselectAll();
      arrow->SetVisibility(false);
    }
  }
  renderMeshesOption.executeMeshRendering = true;
}

void MaliputViewerPlugin::OnNewTextLabelSelection(const QString& _label, bool _state) {
  const std::string label{_label.toStdString()};
  if (label == kLaneLabels || label == kBranchPointLabels) {
    UpdateObjectVisualDefaults(label, _state);
    const std::string keyword = label.substr(0, label.find(kLabels));
    for (auto const& it : maliputBackendSelection.GetMaliputModel()->Labels()) {
      if (FoundKeyword(it.first, keyword)) {
        if (!selector->IsSelected(it.second.text)) {
          maliputBackendSelection.GetMaliputModel()->SetTextLabelState(it.first, _state);
        }
      }
    }
  } else {
    maliputBackendSelection.GetMaliputModel()->SetTextLabelState(label, _state);
  }

  renderMeshesOption.executeLabelRendering = true;
}
void MaliputViewerPlugin::OnTableLaneIdSelection(int _index) {
  // Because the table is filled in the same order that the table's index increases,
  // we can easily get the lane id out of the index of the table.
  const QString& laneId = listLanes[_index];
  const maliput::api::Lane* lane = maliputBackendSelection.GetMaliputModel()->GetLaneFromId(laneId.toStdString());
  if (!lane) {
    ignerr << "There is no loaded lane that matches with this id: " << laneId.toStdString() << std::endl;
    return;
  }
  const std::string lane_id = lane->id().string();
  ignmsg << "Selected lane ID: " << lane_id << std::endl;
  selector->SelectLane(lane);

  // Update visualization to default if it is deselected
  UpdateLane(lane_id);
  UpdateRulesList(lane_id);

  const std::string start_bp_id = lane->GetBranchPoint(maliput::api::LaneEnd::kStart)->id().string();
  const std::string end_bp_id = lane->GetBranchPoint(maliput::api::LaneEnd::kFinish)->id().string();
  UpdateBranchPoint(start_bp_id);
  UpdateBranchPoint(end_bp_id);
}

void MaliputViewerPlugin::OnPhaseSelection(const QModelIndex& _index) {
  const QStandardItem* item = phaseTreeModel.itemFromIndex(_index);
  if (phaseTreeModel.IsPhaseRingItem(item)) {
    // Phase changes when phases are selected.
    return;
  } else {
    // Item should be a phase item.
    const QStandardItem* phaseRingItem = item->parent();
    if (!phaseTreeModel.IsPhaseRingItem(phaseRingItem)) {
      ignerr << "Phase cannot be selected, PhaseRing tree isn't coherent" << std::endl;
      return;
    }
    if (!phaseTreeModel.IsPhaseItem(item, phaseRingItem)) {
      ignerr << "Phase cannot be selected in PhaseRing " << phaseRingItem->text().toStdString() << std::endl;
      return;
    }
    currentPhase.first = item->text().toStdString();
    currentPhase.second = phaseRingItem->text().toStdString();
    trafficLightManager->SetBulbStates(maliputBackendSelection.GetMaliputModel()->GetBulbStates(
        currentPhase.second /* phaseRingId */, currentPhase.first /* phaseId */));
  }
}

void MaliputViewerPlugin::RenderRoadMeshes(const std::map<std::string, std::unique_ptr<MaliputMesh>>& _maliputMeshes) {
  for (const auto& id_mesh : _maliputMeshes) {
    // Checks if the mesh to be rendered already exists or not.
    const auto meshExists = meshes.find(id_mesh.first);

    if (!id_mesh.second->enabled) {
      igndbg << "Road mesh " << id_mesh.first << " is disabled." << std::endl;
      // If the mesh already exists, set visibility to false.
      if (meshExists != this->meshes.end()) {
        meshes[id_mesh.first]->SetVisible(false);
      }
      continue;
    }
    // If the mesh doesn't exist, it creates new one.
    if (meshExists == meshes.end()) {
      ignmsg << "Rendering road mesh: " << id_mesh.first << std::endl;
      ignition::rendering::VisualPtr visual;
      // Creates a material for the visual.
      ignition::rendering::MaterialPtr material = scene->CreateMaterial();
      if (!material) {
        ignerr << "Failed to create material.\n";
        continue;
      }
      visual = scene->CreateVisual();
      if (!visual) {
        ignerr << "Failed to create visual.\n";
        continue;
      }
      // Adds the visual to the map for later reference.
      meshes[id_mesh.first] = visual;
      // Sets the pose of the mesh.
      visual->SetLocalPose(ignition::math::Pose3d(0, 0, 0, 1, 0, 0, 0));
      // Loads the mesh into the visual.
      if (id_mesh.second->mesh.get() == nullptr) {
        ignerr << id_mesh.first << "'s mesh pointer is nullptr" << std::endl;
        continue;
      }
      ignition::rendering::MeshDescriptor descriptor(id_mesh.second->mesh.get());
      descriptor.Load();
      ignition::rendering::MeshPtr meshGeom = scene->CreateMesh(descriptor);
      visual->AddGeometry(meshGeom);
      // Adds the mesh to the parent root visual.
      rootVisual->AddChild(visual);

      // Applies the correct material to the mesh.
      if (!FillMaterial(id_mesh.second->material.get(), material)) {
        ignerr << "Failed to fill " << id_mesh.first << " material information.\n";
        continue;
      }
      visual->SetMaterial(material);
    }
    meshes.at(id_mesh.first)->SetVisible(id_mesh.second->visible);
  }
}

void MaliputViewerPlugin::RenderLabels(const std::map<std::string, MaliputLabel>& _labels) {
  for (const auto& id_label : _labels) {
    // Checks if the text labels to be rendered already exists or not.
    const auto labelExists = textLabels.find(id_label.first);
    if (!id_label.second.enabled) {
      igndbg << "Label mesh " << id_label.first << " is disabled." << std::endl;
      // If the text label already exists, set visibility to false.
      if (labelExists != textLabels.end()) {
        textLabels[id_label.first]->SetVisible(false);
      }
      continue;
    }
    // If the text label doesn't exist, it creates new one.
    if (labelExists == textLabels.end()) {
      ignmsg << "Rendering label mesh: " << id_label.first << std::endl;
      ignition::rendering::VisualPtr visual;
      // Creates a material for the visual.
      ignition::rendering::MaterialPtr material = scene->CreateMaterial();
      if (!material) {
        ignerr << "Failed to create material.\n";
        continue;
      }
      visual = scene->CreateVisual();
      if (!visual) {
        ignerr << "Failed to create visual.\n";
        continue;
      }
      // Adds the visual to the map for later reference.
      textLabels[id_label.first] = visual;
      visual->SetLocalPose(ignition::math::Pose3d(id_label.second.position, ignition::math::Quaterniond()));
      // Creates the text geometry.
      ignition::rendering::TextPtr textGeometry = scene->CreateText();
      textGeometry->SetFontName("Liberation Sans");
      textGeometry->SetTextString(id_label.second.text);
      textGeometry->SetShowOnTop(true);
      textGeometry->SetTextAlignment(ignition::rendering::TextHorizontalAlign::CENTER,
                                     ignition::rendering::TextVerticalAlign::CENTER);
      visual->AddGeometry(textGeometry);
      // Adds the mesh to the parent root visual.
      rootVisual->AddChild(visual);
      // Assigns a material for the visual.
      if (id_label.second.labelType == MaliputLabelType::kLane) {
        CreateLaneLabelMaterial(material);
      } else if (id_label.second.labelType == MaliputLabelType::kBranchPoint) {
        CreateBranchPointLabelMaterial(material);
      } else {
        ignerr << "Unsupported label type for: " << id_label.first << std::endl;
      }
      // Applies the correct material to the mesh.
      visual->SetMaterial(material);
    }
    textLabels.at(id_label.first)->SetVisible(id_label.second.visible);
  }
}

void MaliputViewerPlugin::Clear() {
  if (isRoadNetworkLoaded) {
    selector->DeselectAll();
    arrow->SetVisibility(false);
    // Clears the text labels.
    for (auto it : textLabels) {
      this->rootVisual->RemoveChild(it.second);
    }
    // Clears the meshes.
    for (auto it : meshes) {
      this->rootVisual->RemoveChild(it.second);
    }
    textLabels.clear();
    meshes.clear();
    // Reset phase table.
    phaseTreeModel.Clear();
    // Resert traffic light manager.
    trafficLightManager->Clear();
    isRoadNetworkLoaded = false;
  }
  newRoadNetwork = false;
  // Reset default values for meshes' visualization.
  for (const std::string& key : {kLane, kMarker, kBranchPoint, kBranchPointLabels, kLaneLabels}) {
    objectVisualDefaults[key] = true;
  }
  currentPhase = {"" /* phaseId */, "", /* phaseRingId*/};
}

void MaliputViewerPlugin::CreateLaneLabelMaterial(ignition::rendering::MaterialPtr& _material) {
  _material->SetDiffuse(0.8, 0.8, 0.0);
  _material->SetAmbient(1.0, 1.0, 0.0);
  _material->SetSpecular(1.0, 1.0, 0.5);
  _material->SetShininess(10.);
  _material->SetTransparency(0.5);
}

void MaliputViewerPlugin::CreateBranchPointLabelMaterial(ignition::rendering::MaterialPtr& _material) {
  _material->SetDiffuse(0.0, 0.7, 0.0);
  _material->SetAmbient(1.0, 1.0, 0.0);
  _material->SetSpecular(1.0, 1.0, 0.5);
  _material->SetShininess(10.);
  _material->SetTransparency(0.5);
}

bool MaliputViewerPlugin::FillMaterial(const maliput::utility::Material* _maliputMaterial,
                                       ignition::rendering::MaterialPtr& _ignitionMaterial) {
  if (!_maliputMaterial) {
    return false;
  }

  _ignitionMaterial->SetDiffuse(_maliputMaterial->diffuse.x(), _maliputMaterial->diffuse.y(),
                                _maliputMaterial->diffuse.z());
  _ignitionMaterial->SetAmbient(_maliputMaterial->ambient.x(), _maliputMaterial->ambient.y(),
                                _maliputMaterial->ambient.z());
  _ignitionMaterial->SetSpecular(_maliputMaterial->specular.x(), _maliputMaterial->specular.y(),
                                 _maliputMaterial->specular.z());
  _ignitionMaterial->SetShininess(_maliputMaterial->shininess);
  _ignitionMaterial->SetTransparency(_maliputMaterial->transparency);

  return true;
}

void MaliputViewerPlugin::LoadConfig(const tinyxml2::XMLElement* _pluginElem) {
  title = "Maliput Viewer Plugin";

  if (!_pluginElem) {
    ignerr << "Error reading plugin XML element " << std::endl;
    return;
  }

  // Install event filter.
  ignition::gui::App()->findChild<ignition::gui::MainWindow*>()->installEventFilter(this);

  // Set timer to get the scene.
  timer.start(kTimerPeriodInMs, this);
}

void MaliputViewerPlugin::timerEvent(QTimerEvent* _event) {
  if (_event->timerId() != timer.timerId()) {
    return;
  }

  // Get the render engine.
  // Note: we don't support other engines than Ogre.
  auto engine = ignition::rendering::engine(kEngineName);
  scene = engine->SceneByName(kSceneName);
  if (!scene) {
    ignwarn << "Scene \"" << kSceneName << "\" not found yet. Trying again in "
            << " Trying again in " << kTimerPeriodInMs << "ms" << std::endl;
    return;
  }
  // Get root visual.
  rootVisual = scene->RootVisual();
  if (!rootVisual) {
    ignwarn << "Failed to find the root visual, trying again" << std::endl;
    return;
  }
  if (rootVisual->ChildCount() == 0) {
    ignwarn << "Failed to find the camera, trying again" << std::endl;
    return;
  }
  // Get camera.
  camera = std::dynamic_pointer_cast<ignition::rendering::Camera>(rootVisual->ChildByIndex(0));
  if (!camera) {
    ignwarn << "Failed to find the camera, trying again" << std::endl;
    return;
  }
  setUpScene = true;
  ignmsg << "MaliputViewerPlugin has been initialized." << std::endl;
  timer.stop();

  // Verify if the visualizer should initiate a new road network from CLI.
  const std::string yaml_file_path = flags::GetYamlFilePath();
  if (!yaml_file_path.empty()) {
    const tools::MaliputVizConfig config{tools::LoadYamlConfigFile(yaml_file_path)};
    maliputBackendSelection.LoadBackendByDemand(config.backend_name, config.backend_parameters);
    OnNewRoadNetwork();
  }
}

ignition::gui::Plugin* MaliputViewerPlugin::FilterPluginsByTitle(const std::string& _pluginTitle) {
  QList<ignition::gui::Plugin*> plugins =
      ignition::gui::App()->findChild<ignition::gui::MainWindow*>()->findChildren<ignition::gui::Plugin*>();
  auto plugin = std::find_if(std::begin(plugins), std::end(plugins), [&_pluginTitle](ignition::gui::Plugin* _plugin) {
    return _plugin->Title() == _pluginTitle;
  });
  return plugin == plugins.end() ? nullptr : *plugin;
}

void MaliputViewerPlugin::SetUpScene() {
  // Adds Directional light to improve visualization.
  const auto ambientLight = scene->AmbientLight();
  auto directionalLight = scene->CreateDirectionalLight();
  if (!directionalLight) {
    ignerr << "Failed to create a directional light" << std::endl;
  } else {
    directionalLight->SetDirection(-0.5, -0.5, -1);
    directionalLight->SetDiffuseColor(ambientLight);
    directionalLight->SetSpecularColor(ambientLight);
    rootVisual->AddChild(directionalLight);
  }
  // Create an ArrowMesh.
  const double zArrowOffset{0.5};
  arrow = std::make_unique<ArrowMesh>(this->scene, zArrowOffset);
  // Create a Selector.
  const double selScaleX{0.3};
  const double selScaleY{0.5};
  const double selScaleZ{0.1};
  const double selMinTolerance{0.6};
  const int selPoolSize{50};
  const int selNumLanes{15};
  selector = std::make_unique<Selector>(this->scene, selScaleX, selScaleY, selScaleZ, selPoolSize, selNumLanes,
                                        selMinTolerance);
  // Create traffic light manager.
  trafficLightManager = std::make_unique<TrafficLightManager>(this->scene);
}

bool MaliputViewerPlugin::eventFilter(QObject* _obj, QEvent* _event) {
  if (this->scene != nullptr && isRoadNetworkLoaded) {
    if (_event->type() == ignition::gui::events::LeftClickToScene::kType) {
      auto leftClickToScene = static_cast<ignition::gui::events::LeftClickToScene*>(_event);
      // TODO(https://github.com/ignitionrobotics/ign-gui/issues/209): use distance to camera once
      //                                                               it is available.
      this->MouseClickHandler(leftClickToScene->Point(),
                              this->camera->WorldPosition().Distance(leftClickToScene->Point()));
    }
    // Hooking to the Render event to safely make rendering calls.
    // See https://github.com/ignitionrobotics/ign-gui/blob/ign-gui3/include/ignition/gui/GuiEvents.hh#L36-L37
    if (_event->type() == ignition::gui::events::Render::kType) {
      if (setUpScene) {
        SetUpScene();
        setUpScene = false;
      }
      if (this->newRoadNetwork) {
        // New road network loaded.
        this->RenderRoadMeshes(maliputBackendSelection.GetMaliputModel()->Meshes());
        this->RenderLabels(maliputBackendSelection.GetMaliputModel()->Labels());
        this->trafficLightManager->CreateTrafficLights(maliputBackendSelection.GetMaliputModel()->GetTrafficLights());
        this->renderMeshesOption.executeLabelRendering = false;
        this->renderMeshesOption.executeMeshRendering = false;
        this->newRoadNetwork = false;
      } else {
        // Update scene.
        // Lane selection.
        if (roadPositionResultValue.IsDirty()) {
          this->UpdateLaneSelectionOnLeftClick();
          roadPositionResultValue.SetDirty(false);
        }
        // Update arrow movement.
        arrow->Update();
        // Tick traffic light manager.
        trafficLightManager->Tick();
        // Update road meshes if necessary.
        if (renderMeshesOption.executeMeshRendering) {
          this->RenderRoadMeshes(maliputBackendSelection.GetMaliputModel()->Meshes());
          renderMeshesOption.executeMeshRendering = false;
        }
        // Update label meshes if necessary.
        if (renderMeshesOption.executeLabelRendering) {
          this->RenderLabels(maliputBackendSelection.GetMaliputModel()->Labels());
          renderMeshesOption.executeLabelRendering = false;
        }
      }
    }
  }
  // Standard event processing
  return QObject::eventFilter(_obj, _event);
}

void MaliputViewerPlugin::MouseClickHandler(const ignition::math::Vector3d& _sceneInertialPosition, double _distance) {
  const maliput::api::RoadPositionResult newRoadPositionResult =
      maliputBackendSelection.GetMaliputModel()->GetRoadPositionResult(_sceneInertialPosition);
  // There is no intersection vs There is an intersection and should update the the properties of the lane, etc.
  roadPositionResultValue = newRoadPositionResult.distance > 1e-6
                                ? RoadPositionResultValue()
                                : RoadPositionResultValue(newRoadPositionResult, _distance);
  roadPositionResultValue.SetDirty(true);
}

void MaliputViewerPlugin::UpdateLaneSelectionOnLeftClick() {
  if (roadPositionResultValue.Value().has_value()) {
    if (roadPositionResultValue.Value()->road_position.lane) {
      const maliput::api::Lane* lane = roadPositionResultValue.Value()->road_position.lane;
      const std::string lane_id = lane->id().string();
      ignmsg << "Clicked lane ID: " << lane_id << std::endl;
      selector->SelectLane(lane);
      // Update visualization to default if it is deselected
      UpdateLane(lane_id);
      UpdateRulesList(lane_id);
      UpdateLaneInfoArea(roadPositionResultValue.Value().value());

      const std::string start_bp_id = lane->GetBranchPoint(maliput::api::LaneEnd::kStart)->id().string();
      const std::string end_bp_id = lane->GetBranchPoint(maliput::api::LaneEnd::kFinish)->id().string();
      UpdateBranchPoint(start_bp_id);
      UpdateBranchPoint(end_bp_id);

      const ignition::math::Vector3d sceneClickPosition(roadPositionResultValue.Value()->nearest_position.x(),
                                                        roadPositionResultValue.Value()->nearest_position.y(),
                                                        roadPositionResultValue.Value()->nearest_position.z());
      arrow->SelectAt(roadPositionResultValue.Distance(), sceneClickPosition);
      arrow->SetVisibility(true);

      // Update selected table's lane id.
      for (int i = 0; i < listLanes.length(); ++i) {
        if (listLanes[i].toStdString() == lane_id) {
          tableLaneIdSelection(i);
        }
      }
    }
  } else {
    // Nothing was clicked. Remove lane selection and arrow.
    UpdateSelectedLanesWithDefault();
    arrow->SetVisibility(false);
    selector->DeselectAll();
  }
}

void MaliputViewerPlugin::UpdateLaneInfoArea(const maliput::api::RoadPositionResult& _roadPositionResult) {
  const maliput::api::Lane* lane = _roadPositionResult.road_position.lane;
  const maliput::api::LanePosition& lanePos = _roadPositionResult.road_position.pos;
  const maliput::api::HBounds hBounds = lane->elevation_bounds(lanePos.s(), lanePos.r());
  const maliput::api::RBounds startLaneBounds = lane->lane_bounds(0.);
  const maliput::api::RBounds midLaneBounds = lane->lane_bounds(lane->length() / 2.);
  const maliput::api::RBounds endLaneBounds = lane->lane_bounds(lane->length());
  const maliput::api::RBounds laneBounds = lane->lane_bounds(lanePos.s());
  const maliput::api::Rotation rotation = lane->GetOrientation(lanePos);
  // Update message to be displayed in the info area.
  std::stringstream ss;
  ss << "----  LANE ID: " << lane->id() << "  -----";
  ss << "\nLength ------------> " << lane->length() << "m";
  ss << "\nLanePosition ------> " << lanePos;
  ss << "\nInertialPosition --> " << _roadPositionResult.nearest_position;
  ss << "\nRotation --> " << rotation;
  ss << "\nRBounds ------> (min: " << laneBounds.min() << ", max: " << laneBounds.max() << ")";
  ss << "\nHBounds ------> (min: " << hBounds.min() << ", max: " << hBounds.max() << ")";
  ss << "\nSegmentId: ------------> " << lane->segment()->id().string();
  ss << "\nJunctionId: ------------> " << lane->segment()->junction()->id().string();
  ss << "\n----  LANE BOUNDARIES (INERTIAL FRAME)  ----";
  ss << "\n(s, r, h) ------> (x, y, z)";
  ss << "\n(0, 0, 0) ----------> " << lane->ToInertialPosition({0., 0., 0.});
  ss << "\n(0, r_min, 0) ------> " << lane->ToInertialPosition({0., startLaneBounds.min(), 0.});
  ss << "\n(0, r_max, 0) ------> " << lane->ToInertialPosition({0., startLaneBounds.max(), 0.});
  ss << "\n(s_max / 2, 0, 0) ----------> " << lane->ToInertialPosition({lane->length() / 2., 0., 0.});
  ss << "\n(s_max / 2, r_min, 0) ------> " << lane->ToInertialPosition({lane->length() / 2., midLaneBounds.min(), 0.});
  ss << "\n(s_max / 2, r_max, 0) ------> " << lane->ToInertialPosition({lane->length() / 2., midLaneBounds.max(), 0.});
  ss << "\n(s_max, 0, 0) ----------> " << lane->ToInertialPosition({lane->length(), 0., 0.});
  ss << "\n(s_max, r_min, 0) ------> " << lane->ToInertialPosition({lane->length(), endLaneBounds.min(), 0.});
  ss << "\n(s_max, r_max, 0) ------> " << lane->ToInertialPosition({lane->length(), endLaneBounds.max(), 0.});

  laneInfo = QString::fromStdString(ss.str());
  emit LaneInfoChanged();
}

void MaliputViewerPlugin::UpdateLane(const std::string& _id) {
  const bool isLaneVisualized = selector->IsSelected(_id) || objectVisualDefaults[kLane];
  const bool isMarkerVisualized = selector->IsSelected(_id) || objectVisualDefaults[kMarker];
  const bool isLaneLabelVisualized = selector->IsSelected(_id) || objectVisualDefaults[kLaneLabels];

  const std::string laneKey = FromIdToMapKey(kLane, _id);
  const std::string markerKey = FromIdToMapKey(kMarker, _id);

  OnNewMeshLayerSelection(QString::fromStdString(laneKey), isLaneVisualized);
  OnNewMeshLayerSelection(QString::fromStdString(markerKey), isMarkerVisualized);
  OnNewTextLabelSelection(QString::fromStdString(laneKey), isLaneLabelVisualized);
}

void MaliputViewerPlugin::UpdateBranchPoint(const std::string& _id) {
  const bool isBPMeshVisualized = selector->IsSelected(_id) || objectVisualDefaults[kBranchPoint];
  const bool isBPLabelVisualized = selector->IsSelected(_id) || objectVisualDefaults[kBranchPointLabels];
  const std::string key = FromIdToMapKey(kBranchPoint, _id);

  OnNewMeshLayerSelection(QString::fromStdString(key), isBPMeshVisualized);
  OnNewTextLabelSelection(QString::fromStdString(key), isBPLabelVisualized);
}

void MaliputViewerPlugin::UpdateSelectedLanesWithDefault() {
  const std::vector<std::string> selectedLanes = selector->GetSelectedLanes();
  const std::vector<std::string> selectedBranchPoints = selector->GetSelectedBranchPoints();
  for (const auto& id : selectedLanes) {
    maliputBackendSelection.GetMaliputModel()->SetLayerState(FromIdToMapKey(kLane, id), objectVisualDefaults[kLane]);
    maliputBackendSelection.GetMaliputModel()->SetLayerState(FromIdToMapKey(kMarker, id),
                                                             objectVisualDefaults[kMarker]);
    maliputBackendSelection.GetMaliputModel()->SetTextLabelState(FromIdToMapKey(kLane, id),
                                                                 objectVisualDefaults[kLaneLabels]);
  }
  for (const auto& id : selectedBranchPoints) {
    maliputBackendSelection.GetMaliputModel()->SetLayerState(FromIdToMapKey(kBranchPoint, id),
                                                             objectVisualDefaults[kBranchPoint]);
    maliputBackendSelection.GetMaliputModel()->SetTextLabelState(FromIdToMapKey(kBranchPoint, id),
                                                                 objectVisualDefaults[kBranchPointLabels]);
  }

  renderMeshesOption.RenderAll();
}

void MaliputViewerPlugin::UpdateRulesList(const std::string& _laneId) {
  rulesList = maliputBackendSelection.GetMaliputModel()->GetRulesOfLane<QString>(
      currentPhase.second /* phaseRingId */, currentPhase.first /* phaseId */, _laneId);
  emit RulesListChanged();
}

}  // namespace viz
}  // namespace maliput

// Register this plugin
IGNITION_ADD_PLUGIN(maliput::viz::MaliputViewerPlugin, ignition::gui::Plugin)
