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

import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Controls.Material 2.1
import QtQuick.Dialogs 1.0
import QtQuick.Layouts 1.3

Rectangle {
  id: maliputViewerPlugin
  Layout.minimumWidth: 450
  Layout.preferredWidth: 500
  Layout.minimumHeight: 1250
  anchors.fill: parent

  // Files Selection Panel
  Loader {
    id: maliputLoader
    width: parent.width
    source: "MaliputBackendConfigurationArea.qml"
  }
  ToolSeparator {
    id: fileSectionSeparator
    orientation: Qt.Horizontal
    anchors.top: maliputLoader.bottom
    anchors.topMargin: 15
    anchors.left: parent.left
    anchors.leftMargin: 10
    anchors.right: parent.right
    anchors.rightMargin: 10
  }
  // Layers Selection Panel
  Loader {
    id: layersLoader
    width: parent.width
    source: "LayersSelectionArea.qml"
    anchors.top: fileSectionSeparator.bottom
    anchors.left: parent.left
    anchors.right: parent.right
  }
  ToolSeparator {
    id: layersVisualizationSeparator
    orientation: Qt.Horizontal
    anchors.top: layersLoader.bottom
    anchors.topMargin: 15
    anchors.left: parent.left
    anchors.leftMargin: 10
    anchors.right: parent.right
    anchors.rightMargin: 10
  }
  // Label Selection Panel
  Loader {
    id: labelsLoader
    width: parent.width
    source: "LabelsSelectionArea.qml"
    anchors.top: layersVisualizationSeparator.bottom
    anchors.left: parent.left
    anchors.right: parent.right
  }
  ToolSeparator {
    id: labelsLoaderSeparator
    orientation: Qt.Horizontal
    anchors.top: labelsLoader.bottom
    anchors.topMargin: 15
    anchors.left: parent.left
    anchors.leftMargin: 10
    anchors.right: parent.right
    anchors.rightMargin: 10
  }
  // Mouse Info Selection Panel
  Loader {
    id: mouseInfo
    width: parent.width
    source: "InfoArea.qml"
    anchors.top: labelsLoaderSeparator.bottom
    anchors.left: parent.left
    anchors.right: parent.right
  }
  ToolSeparator {
    id: mouseInfoSeparator
    orientation: Qt.Horizontal
    anchors.top: mouseInfo.bottom
    anchors.topMargin: 15
    anchors.left: parent.left
    anchors.leftMargin: 10
    anchors.right: parent.right
    anchors.rightMargin: 10
  }
  // Phase selection Panel
  Loader {
    id: phaseSelectionLoader
    width: parent.width
    source: "PhaseSelectionArea.qml"
    anchors.top: mouseInfoSeparator.bottom
    anchors.left: parent.left
    anchors.right: parent.right
  }
  ToolSeparator {
    id: phaseSelectionLoaderSeparator
    orientation: Qt.Horizontal
    anchors.top: phaseSelectionLoader.bottom
    anchors.topMargin: 0
    anchors.left: parent.left
    anchors.leftMargin: 10
    anchors.right: parent.right
    anchors.rightMargin: 10
  }
  // Lanes List Panel
  Loader {
    id: lanesListLoader
    width: parent.width
    source: "LanesListArea.qml"
    anchors.top: phaseSelectionLoaderSeparator.bottom
    anchors.left: parent.left
    anchors.right: parent.right
  }
  ToolSeparator {
    id: lanesListLoaderSeparator
    orientation: Qt.Horizontal
    anchors.top: lanesListLoader.bottom
    anchors.topMargin: 0
    anchors.left: parent.left
    anchors.leftMargin: 10
    anchors.right: parent.right
    anchors.rightMargin: 10
  }
  // Rules List Panel
  Loader {
    id: rulesListLoader
    width: parent.width
    source: "RulesListArea.qml"
    anchors.top: lanesListLoaderSeparator.bottom
    anchors.left: parent.left
    anchors.right: parent.right
  }
}
