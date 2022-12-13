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
#include "maliput_backend_selection.hh"

#include <sstream>

#include <ignition/common/Console.hh>
#include <ignition/gui/Application.hh>

namespace maliput {
namespace viz {
namespace {

std::unique_ptr<maliput::plugin::RoadNetworkLoader> CreateRoadNetworkLoader(
    const std::string& road_network_loader_id, const maliput::plugin::MaliputPluginManager* maliput_plugin_manager) {
  const maliput::plugin::MaliputPlugin* maliput_plugin =
      maliput_plugin_manager->GetPlugin(maliput::plugin::MaliputPlugin::Id(road_network_loader_id));
  if (!maliput_plugin) {
    ignerr << road_network_loader_id + " plugin can't be obtained." << std::endl;
    MALIPUT_THROW_MESSAGE(road_network_loader_id + " plugin can't be obtained.");
  }
  if (maliput_plugin->GetType() != maliput::plugin::MaliputPluginType::kRoadNetworkLoader) {
    ignerr << road_network_loader_id + " plugin should be a RoadNetworkLoader plugin type." << std::endl;
    MALIPUT_THROW_MESSAGE(road_network_loader_id + " plugin should be a RoadNetworkLoader plugin type.");
  }
  maliput::plugin::RoadNetworkLoaderPtr rn_loader_ptr =
      maliput_plugin->ExecuteSymbol<maliput::plugin::RoadNetworkLoaderPtr>(
          maliput::plugin::RoadNetworkLoader::GetEntryPoint());
  // Use smart pointers to gracefully manage heap allocation.
  return std::unique_ptr<maliput::plugin::RoadNetworkLoader>{
      reinterpret_cast<maliput::plugin::RoadNetworkLoader*>(rn_loader_ptr)};
}

}  // namespace

ParameterTableModel::ParameterTableModel(QObject* parent) : QAbstractTableModel(parent) {}

int ParameterTableModel::rowCount(const QModelIndex& parent) const { return parameterList.size(); }

int ParameterTableModel::columnCount(const QModelIndex& parent) const { return roleNames().size(); }

QVariant ParameterTableModel::data(const QModelIndex& index, int role) const {
  QString data;
  const int row = index.row();
  return GetData(row, role);
}

QHash<int, QByteArray> ParameterTableModel::roleNames() const {
  return {
      {kParameterRole, "parameter"},
      {kValueRole, "value"},
  };
}

QString ParameterTableModel::GetData(int row, int role) const {
  return role == kParameterRole ? parameterList[row].parameterName : parameterList[row].parameterValue;
}

std::map<std::string, std::string> ParameterTableModel::GetMapFromParameters() const {
  std::map<std::string, std::string> map;
  std::transform(parameterList.begin(), parameterList.end(), std::inserter(map, map.end()), [](const Parameter& p) {
    return std::make_pair(p.parameterName.toStdString(), p.parameterValue.toStdString());
  });
  return map;
}

std::optional<int> ParameterTableModel::getIndex(const QString& _parameterName, const QList<Parameter>& _list) {
  for (int i = 0; i < _list.size(); i++) {
    if (_list[i].parameterName == _parameterName) {
      return i;
    }
  }
  return std::nullopt;
}

void ParameterTableModel::AddParameter(const QString& _parameterName, const QString& _parameterValue) {
  // If the parameter already exists, update the value
  if (std::optional<int> idx = getIndex(_parameterName, parameterList); idx != std::nullopt) {
    parameterList[idx.value()].parameterValue = _parameterValue;
    emit dataChanged(index(idx.value(), 0), index(idx.value(), 1));
    return;
  }
  beginInsertRows(QModelIndex(), parameterList.size(), parameterList.size());
  parameterList.append({_parameterName, _parameterValue});
  endInsertRows();
}

void ParameterTableModel::ClearParameters() {
  beginResetModel();
  parameterList.clear();
  endResetModel();
}

void ParameterTableModel::DeleteParameter(const QString& _parameterName) {
  if (std::optional<int> idx = getIndex(_parameterName, parameterList); idx != std::nullopt) {
    beginRemoveRows(QModelIndex(), idx.value(), idx.value());
    parameterList.removeAt(idx.value());
    endRemoveRows();
  }
}

MaliputBackendSelection::MaliputBackendSelection(QObject* parent) : QObject(parent) {
  parameterTableModel = std::make_unique<ParameterTableModel>();
  backendListModel = std::make_unique<QStringListModel>();
  ignition::gui::App()->Engine()->rootContext()->setContextProperty("parameterTableModel", parameterTableModel.get());
  ignition::gui::App()->Engine()->rootContext()->setContextProperty("backendListModel", backendListModel.get());

  // Obtains the list of available maliput backends.
  maliputManager = std::make_unique<maliput::plugin::MaliputPluginManager>();
  const auto plugin_list = maliputManager->ListPlugins();
  QList<QString> backend_list;
  std::for_each(plugin_list.begin(), plugin_list.end(), [&backend_list](const auto& plugin) {
    if (plugin.second == maliput::plugin::MaliputPluginType::kRoadNetworkLoader) {
      backend_list.append(QString::fromStdString(plugin.first.string()));
    }
  });
  backendListModel->setStringList(backend_list);
}

void MaliputBackendSelection::OnBackendSelected(const QString& _backendName) {
  // Clears the table model.
  parameterTableModel->ClearParameters();

  // Obtains the RoadNetworkLoader for the selected backend.
  roadNetworkLoader = CreateRoadNetworkLoader(_backendName.toStdString(), maliputManager.get());
  if (!roadNetworkLoader) {
    ignerr << "Failed to create road network loader for backend: " << _backendName.toStdString() << std::endl;
    return;
  }

  // Obtains the list of parameters for the selected backend.
  const auto default_parameters = roadNetworkLoader->GetDefaultParameters();
  for (const auto& parameter : default_parameters) {
    parameterTableModel->AddParameter(QString::fromStdString(parameter.first),
                                      QString::fromStdString(parameter.second));
  }
}

void MaliputBackendSelection::OnLoadButtonPressed() {
  if (!roadNetworkLoader) {
    ignerr << "Make sure that a maliput backend is selected." << std::endl;
    return;
  }

  // Initializes the maliput viewer model.
  maliputViewerModel =
      std::make_unique<MaliputViewerModel>(roadNetworkLoader->operator()(parameterTableModel->GetMapFromParameters()));
}

void MaliputBackendSelection::LoadBackendByDemand(const std::string& _backendName,
                                                  const std::map<std::string, std::string>& _parameters) {
  OnBackendSelected(QString::fromStdString(_backendName));
  maliputViewerModel = std::make_unique<MaliputViewerModel>(roadNetworkLoader->operator()(_parameters));
  parameterTableModel->ClearParameters();
  for (const auto& parameter : _parameters) {
    parameterTableModel->AddParameter(QString::fromStdString(parameter.first),
                                      QString::fromStdString(parameter.second));
  }
}

}  // namespace viz
}  // namespace maliput
