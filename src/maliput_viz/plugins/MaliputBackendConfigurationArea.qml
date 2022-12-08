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
import QtQuick.Controls 1.4
import QtQuick.Controls.Material 2.1
import QtQuick.Dialogs 1.0
import QtQuick.Layouts 1.3
import QtQuick.Controls.Styles 1.4

GridLayout {
  id: maliputBackendConfigurationArea
  columns: 1
  anchors.left: parent.left
  anchors.right: parent.right
  anchors.leftMargin: 10
  anchors.rightMargin: 10
  anchors.top: parent.top
  anchors.topMargin: 10
  Layout.fillWidth: true

  /**
  * Title text
  */
  Text {
    id: titleText
    Layout.columnSpan: 1
    Layout.alignment: Qt.AlignVTop | Qt.AlignHCenter
    font.family: "Helvetica"
    font.pixelSize: 14
    text: "ROAD NETWORK LOADER"
  }

  // Grid for the backend selection
  GridLayout {
    id: gridBackendSelection
    columns: 2
    Layout.fillWidth: true

    // Maliput backend text
    Text {
      id: maliputBackendText
      Layout.columnSpan: 1
      Layout.alignment: Qt.AlignVTop | Qt.AlignHCenter
      font.pixelSize: 14
      text: "Maliput Backend Selection"
    }

    // ComboBox for Maliput backend selection
    ComboBox {
        id: comboBox
        model: backendListModel
        textRole: "display" // QStringListModel has many roles. This is the one we want to display
        anchors.leftMargin: 0
        anchors.rightMargin: 0
        Layout.fillWidth: true
        onActivated: { // After Qt 5.15 textActivated signal is added.
          console.log("Accepted: ", comboBox.currentText);
          MaliputBackendSelection.OnBackendSelected(comboBox.currentText)
        }
    }

  } // End gridBackendSelection

  // Table of parameters.
  TableView {
    id: parametersTable
    Layout.fillWidth: true
    selectionMode: 1 /* Single Selection */
    model: parameterTableModel
    TableViewColumn {
        role: "parameter"
        title: "Parameter"
        width: parametersTable.width * 0.6
    }
    TableViewColumn {
        role: "value"
        title: "Value"
        width: parametersTable.width * 0.4
    }
    onClicked: {
      console.log("Clicked on row: ", row);
      addKeyTextField.text = parameterTableModel.GetData(row, 101 /* parameter */);
      addValueTextField.text = parameterTableModel.GetData(row, 102 /* value */);
    }
    itemDelegate: Item {
      Text {
          anchors.verticalCenter: parent.verticalCenter
          color: styleData.textColor
          elide: styleData.elideMode
          font.family: "Helvetica"
          font.pixelSize: 12
          text: styleData.value
      }
    }
  }

  // Grid for parameter addition
  GridLayout {

    id: gridParameterAddition

    columns: 4
    Layout.fillWidth: true

    // Button for adding a parameter
    Button {
      id: addButton
      text: "Add"
      checkable: false
      Layout.columnSpan: 1
      Layout.fillWidth: true
      onClicked: {
        parameterTableModel.AddParameter(addKeyTextField.text, addValueTextField.text);
      }
    }

    // Key text for parameter addition
    TextField {
      id: addKeyTextField
      Layout.fillWidth: true
      readOnly: false
      placeholderText: qsTr("key to add (e.g. 'bar')")
      font.pixelSize: 12
      font.family: "Helvetica"
    }

    // Value text for parameter addition
    TextField {
      id: addValueTextField
      Layout.fillWidth: true
      readOnly: false
      placeholderText: qsTr("value to add (e.g. 'foo')")
      font.pixelSize: 12
      font.family: "Helvetica"
    }

    // Button for opening the file dialog
    Button {
      id: fileDialogButton
      text: "..."
      checkable: false
      Layout.preferredWidth: 15
      onClicked: {
        filePathDialog.open()
      }
      Material.background: Material.primary
      style: ButtonStyle {
        label: Text {
          renderType: Text.NativeRendering
          verticalAlignment: Text.AlignVCenter
          horizontalAlignment: Text.AlignHCenter
          font.family: "Helvetica"
          font.pointSize: 10
          color: "black"
          text: fileDialogButton.text
        }
      }
    }

    // File dialog for selecting a filepath if necessary.
    FileDialog {
      id: filePathDialog
      title: "Add a file path to add"
      nameFilters: [ "All files (*)" ]
      Layout.fillWidth: true
      selectExisting : true
      selectFolder : false
      selectMultiple : false
      sidebarVisible : true
      onAccepted: {
        var path = filePathDialog.fileUrl.toString();
        // remove prefixed "file:///"
        path = path.replace(/^(file:\/{2})/,"");
        // unescape html codes like '%23' for '#'
        addValueTextField.text = decodeURIComponent(path);
        console.log("Selection: You chose: " + addValueTextField.text)
      }
      onRejected: {
        console.log("Selection: Canceled")
      }
      visible: false
    }

  } // End gridParameterAddition

  // Grid for parameter remotion
  GridLayout {

    id: gridParameterRemotion

    columns: 3
    Layout.fillWidth: true

    // Button for removing a parameter
    Button {
      id: delButton
      text: "Delete"
      checkable: false
      Layout.columnSpan: 1
      Layout.fillWidth: true
      onClicked: {
        parameterTableModel.DeleteParameter(removeKeyTextField.text);
      }
    }

    // Key text for parameter addition
    TextField {
      id: removeKeyTextField
      Layout.fillWidth: true
      readOnly: false
      placeholderText: qsTr("key to delete (e.g. 'bar')")
      font.pixelSize: 12
      font.family: "Helvetica"
    }

    // Button for clearing all the parameters
    Button {
      id: delAllButton
      text: "Delete All"
      checkable: false
      Layout.columnSpan: 1
      Layout.fillWidth: true
      onClicked: {
        parameterTableModel.ClearParameters();
      }
    }
  }

  // Button for loading the road network
  Button {
    id: loadButton
    text: "LOAD ROAD NETWORK"
    checkable: false
    Layout.fillWidth: true
    onClicked: {
      console.log("Load button pressed");
      MaliputBackendSelection.OnLoadButtonPressed()
      MaliputViewerPlugin.OnNewRoadNetwork()
    }
    Material.background: Material.primary
  }
}
