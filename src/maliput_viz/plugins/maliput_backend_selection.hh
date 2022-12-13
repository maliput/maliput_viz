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

#include <map>
#include <string>
#include <vector>

#include <ignition/gui/qt.h>
#include <maliput/plugin/maliput_plugin_manager.h>
#include <maliput/plugin/road_network_loader.h>

#include "maliput_viewer_model.hh"

namespace maliput {
namespace viz {

/// Represents a key-value parameter for a Maliput backend.
struct Parameter {
  QString parameterName;
  QString parameterValue;
};

/// A table model for a table of Maliput's RoadNetworkLoader's parameters.
class ParameterTableModel : public QAbstractTableModel {
  Q_OBJECT
 public:
  static constexpr int kParameterRole{101};
  static constexpr int kValueRole{102};

  /// Constructor
  /// @param parent Parent object,
  ParameterTableModel(QObject* parent = nullptr);

  /// Get the parameter list in a std map.
  std::map<std::string, std::string> GetMapFromParameters() const;

  /// Adds a parameter to the table.
  /// @param _parameterName Parameter to add.
  /// @param _parameterValue Value of the parameter.
  Q_INVOKABLE void AddParameter(const QString& _parameterName, const QString& _parameterValue);

  /// Removes a parameter from the table.
  /// @param _parameterName Parameter to remove.
  Q_INVOKABLE void DeleteParameter(const QString& _parameterName);

  /// Removes all parameters from the table.
  Q_INVOKABLE void ClearParameters();

  /// Get data from a row.
  Q_INVOKABLE QString GetData(int row, int role) const;

 protected:
  /// Documentation inherited
  Q_INVOKABLE int rowCount(const QModelIndex& parent = QModelIndex()) const override;
  /// Documentation inherited
  Q_INVOKABLE int columnCount(const QModelIndex& parent = QModelIndex()) const override;
  /// Documentation inherited
  Q_INVOKABLE QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const override;
  /// Documentation inherited
  Q_INVOKABLE QHash<int, QByteArray> roleNames() const override;

 private:
  /// Get the index of a parameter in the table.
  /// @param _parameterName Name of the parameter.
  /// @param _list List of parameters.
  /// @return Index of the parameter in the list. std::nullopt if not found.
  static std::optional<int> getIndex(const QString& _parameterName, const QList<Parameter>& _list);

  /// List of parameters.
  QList<Parameter> parameterList;
};

/// Show stats of all topics.
class MaliputBackendSelection : public QObject {
  Q_OBJECT

 public:
  /// Constructor.
  MaliputBackendSelection(QObject* parent);

  /// Returns a pointer to the MaliputViewerModel.
  MaliputViewerModel* GetMaliputModel() { return maliputViewerModel.get(); }

  /// Load a backend with the given parameters. This is expected to be called by a parent class, skipping the use of the
  /// gui.
  void LoadBackendByDemand(const std::string& _backendName, const std::map<std::string, std::string>& _parameters);

 protected slots:
  /// Called when the user press the Load RoadNetwork button.
  void OnLoadButtonPressed();

  /// Called when the user selects a backend.
  /// @param _backendName Name of the backend.
  void OnBackendSelected(const QString& _backendName);

 private:
  std::unique_ptr<ParameterTableModel> parameterTableModel;
  std::unique_ptr<QStringListModel> backendListModel;

  std::unique_ptr<maliput::plugin::MaliputPluginManager> maliputManager;
  std::unique_ptr<MaliputViewerModel> maliputViewerModel;
  std::unique_ptr<maliput::plugin::RoadNetworkLoader> roadNetworkLoader;
};

}  // namespace viz
}  // namespace maliput
