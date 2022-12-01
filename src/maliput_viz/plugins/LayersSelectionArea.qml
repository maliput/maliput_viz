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
import QtQuick.Controls 1.4
import QtQuick.Controls.Styles 1.4
import QtQuick.Controls.Material 2.1
import QtQuick.Dialogs 1.0
import QtQuick.Layouts 1.3

// Panel that contains checkboxs to enable or disable layers.
GridLayout {
  id: layersVisualizationPanel
  columns: 3
  anchors.left: parent.left
  anchors.leftMargin: 10
  anchors.right: parent.right
  anchors.rightMargin: 10
  anchors.top: parent.top
  Layout.fillWidth: true

  /**
  * Title text
  */
  Text {
    id: titleText
    Layout.columnSpan: 3
    Layout.alignment: Qt.AlignVTop | Qt.AlignHCenter
    font.pixelSize: 14
    text: "MESH LAYERS"
  }

  /**
  * Ashpalt checkbox
  */
  CheckBox {
    id: layerAsphalt
    text: qsTr("Asphalt")
    checked: true
    onClicked : {
      MaliputViewerPlugin.OnNewMeshLayerSelection("asphalt", checked);
    }
    style: CheckBoxStyle {
      label: Text {
        renderType: Text.NativeRendering
        verticalAlignment: Text.AlignVCenter
        horizontalAlignment: Text.AlignHCenter
        font.family: "Helvetica"
        font.pixelSize: 12
        color: "black"
        text: layerAsphalt.text
      }
    }
  }

  /**
  * Lane checkbox
  */
  CheckBox {
    id: layerLane
    text: qsTr("Lane")
    checked: true
    onClicked : {
      MaliputViewerPlugin.OnNewMeshLayerSelection("lane_all", checked);
    }
    style: CheckBoxStyle {
      label: Text {
        renderType: Text.NativeRendering
        verticalAlignment: Text.AlignVCenter
        horizontalAlignment: Text.AlignHCenter
        font.family: "Helvetica"
        font.pixelSize: 12
        color: "black"
        text: layerLane.text
      }
    }
  }

  /**
  * Marker checkbox
  */
  CheckBox {
    id: layerMarker
    text: qsTr("Marker")
    checked: true
    onClicked : {
      MaliputViewerPlugin.OnNewMeshLayerSelection("marker_all", checked);
    }
    style: CheckBoxStyle {
      label: Text {
        renderType: Text.NativeRendering
        verticalAlignment: Text.AlignVCenter
        horizontalAlignment: Text.AlignHCenter
        font.family: "Helvetica"
        font.pixelSize: 12
        color: "black"
        text: layerMarker.text
      }
    }
  }

  /**
  * HBounds checkbox
  */
  CheckBox {
    id: layerHBounds
    text: qsTr("HBounds")
    checked: true
    onClicked : {
      MaliputViewerPlugin.OnNewMeshLayerSelection("h_bounds", checked);
    }
    style: CheckBoxStyle {
      label: Text {
        renderType: Text.NativeRendering
        verticalAlignment: Text.AlignVCenter
        horizontalAlignment: Text.AlignHCenter
        font.family: "Helvetica"
        font.pixelSize: 12
        color: "black"
        text: layerHBounds.text
      }
    }
  }

  /**
  * BranchPoint checkbox
  */
  CheckBox {
    id: layerBranchPoint
    text: qsTr("Branch Point")
    checked: true
    onClicked : {
      MaliputViewerPlugin.OnNewMeshLayerSelection("branch_point_all", checked);
    }
    style: CheckBoxStyle {
      label: Text {
        renderType: Text.NativeRendering
        verticalAlignment: Text.AlignVCenter
        horizontalAlignment: Text.AlignHCenter
        font.family: "Helvetica"
        font.pixelSize: 12
        color: "black"
        text: layerBranchPoint.text
      }
    }
  }

  /**
  * GrayedAsphalt checkbox
  */
  CheckBox {
    id: layerGrayedAsphalt
    text: qsTr("Grayed asphalt")
    checked: false
    onClicked : {
      MaliputViewerPlugin.OnNewMeshLayerSelection("grayed_asphalt", checked);
    }
    style: CheckBoxStyle {
      label: Text {
        renderType: Text.NativeRendering
        verticalAlignment: Text.AlignVCenter
        horizontalAlignment: Text.AlignHCenter
        font.family: "Helvetica"
        font.pixelSize: 12
        color: "black"
        text: layerGrayedAsphalt.text
      }
    }
  }

  /**
  * GrayedLane checkbox
  */
  CheckBox {
    id: layerGrayedLane
    text: qsTr("Grayed lane")
    checked: false
    onClicked : {
      MaliputViewerPlugin.OnNewMeshLayerSelection("grayed_lane_all", checked);
    }
    style: CheckBoxStyle {
      label: Text {
        renderType: Text.NativeRendering
        verticalAlignment: Text.AlignVCenter
        horizontalAlignment: Text.AlignHCenter
        font.family: "Helvetica"
        font.pixelSize: 12
        color: "black"
        text: layerGrayedLane.text
      }
    }
  }

  /**
  * GrayedMarker checkbox
  */
  CheckBox {
    id: layerGrayedMarker
    text: qsTr("Grayed marker")
    checked: false
    onClicked : {
      MaliputViewerPlugin.OnNewMeshLayerSelection("grayed_marker_all", checked);
    }
    style: CheckBoxStyle {
      label: Text {
        renderType: Text.NativeRendering
        verticalAlignment: Text.AlignVCenter
        horizontalAlignment: Text.AlignHCenter
        font.family: "Helvetica"
        font.pixelSize: 12
        color: "black"
        text: layerGrayedMarker.text
      }
    }
  }

  /*
  * Update checkboxes' state.
  */
  Connections {
    target: MaliputViewerPlugin
    onLayerCheckboxesChanged: {
      layerAsphalt.checked = MaliputViewerPlugin.layerCheckboxes[0]
      layerLane.checked = MaliputViewerPlugin.layerCheckboxes[1]
      layerMarker.checked = MaliputViewerPlugin.layerCheckboxes[2]
      layerHBounds.checked = MaliputViewerPlugin.layerCheckboxes[3]
      layerBranchPoint.checked = MaliputViewerPlugin.layerCheckboxes[4]
      layerGrayedAsphalt.checked = MaliputViewerPlugin.layerCheckboxes[5]
      layerGrayedLane.checked = MaliputViewerPlugin.layerCheckboxes[6]
      layerGrayedMarker.checked = MaliputViewerPlugin.layerCheckboxes[7]
    }
  }
}
