import logging
import os
from typing import Annotated, Optional


import vtk

import slicer
from slicer.i18n import tr as _
from slicer.i18n import translate
from slicer.ScriptedLoadableModule import *
from slicer.util import VTKObservationMixin
from slicer.parameterNodeWrapper import (
    parameterNodeWrapper,
    WithinRange,
)

from slicer import vtkMRMLScalarVolumeNode

import qt
import Scripts.Logics.install_dependency as ID
ID.install_dependencies()
from Scripts.Logics.communication_serial import SerialCommunication
from Scripts.Logics.communicaton_TCPIP import TCPIPCommunication
from Scripts.Logics.communication_UDP import UDPCommunication

#
# SlicerComm
#


class SlicerComm(ScriptedLoadableModule):

    def __init__(self, parent):
        ScriptedLoadableModule.__init__(self, parent)
        self.parent.title = _("SlicerComm")  
        self.parent.categories = ["SlicerCR"]
        self.parent.dependencies = []  # TODO: add here list of module names that this module requires
        self.parent.contributors = ["Letian Ai (BM2)"] 
        # TODO: update with short description of the module and a link to online module documentation
        # _() function marks text as translatable to other languages
        self.parent.helpText = _(""" Tool to communicate with external devices via serial ports, TCP/IP, or UDP.""")
        # TODO: replace with organization, grant and thanks
        self.parent.acknowledgementText = _(""" This file was originally developed by Letian Ai """)



#
# SlicerCommWidget
#


class SlicerCommWidget(ScriptedLoadableModuleWidget, VTKObservationMixin):

    def __init__(self, parent=None) -> None:
        """Called when the user opens the module the first time and the widget is initialized."""
        ScriptedLoadableModuleWidget.__init__(self, parent)
        VTKObservationMixin.__init__(self)  # needed for parameter node observation
        self.logic = None
        self.communication_count = 0
        self.communication_panels = []

    def setup(self) -> None:
        """Called when the user opens the module the first time and the widget is initialized."""
        ScriptedLoadableModuleWidget.setup(self)
        uiWidget = slicer.util.loadUI(self.resourcePath("UI/SlicerComm.ui"))
        self.layout.addWidget(uiWidget)
        self.ui = slicer.util.childWidgetVariables(uiWidget)
        uiWidget.setMRMLScene(slicer.mrmlScene)
        self.logic = SlicerCommLogic()

        # Add new communication panel
        self.ui.AddNewCommunicationPushBtn.clicked.connect(self.onAddNewCommunication)
        self.spacer_index = None
        self.main_layout = self.ui.verticalLayout
        for i in range(self.main_layout.count()):
            item = self.main_layout.itemAt(i)
            if item.spacerItem():
                self.spacer_index = i
                break
            

    def onAddNewCommunication(self):
        """Add a new communication panel"""
        self.communication_count += 1
        
        communication_panel = self.createCommunicationPanelFromTemplate(self.communication_count)
        if communication_panel:
            self.communication_panels.append(communication_panel)
            insert_index = self.spacer_index if self.spacer_index is not None else self.main_layout.count()
            self.main_layout.insertWidget(insert_index-1, communication_panel)
            if self.spacer_index is not None:
                self.spacer_index += 1

    def createCommunicationPanelFromTemplate(self, communication_number):
        """Create a new communication panel from template"""
        module_dir = os.path.dirname(os.path.abspath(__file__))
        if self.ui.ProtocalSelection.currentText == "Serial":
            template_path = os.path.join(module_dir,"Resources", "UI", "Template_SerialCommunicationPanel.ui")  
        elif self.ui.ProtocalSelection.currentText == "TCP/IP":
            template_path = os.path.join(module_dir,"Resources", "UI", "Template_TCPIPCommunicationPanel.ui")  
        elif self.ui.ProtocalSelection.currentText == "UDP":
            template_path = os.path.join(module_dir,"Resources", "UI", "Template_UDPCommunicationPanel.ui")  
        else:
            print("Invalid communication protocol")
        try:
            # Load the UI file
            loader = qt.QUiLoader()
            ui_file = qt.QFile(template_path)
            ui_file.open(qt.QFile.ReadOnly)
            robot_panel = loader.load(ui_file)
            ui_file.close()
            
            if not robot_panel:
                print("Failed to load template UI")
                return None
                
            # Customize the loaded panel for this specific robot
            type_string = self.ui.ProtocalSelection.currentText
            self.customizeCommunicationPanel(robot_panel, communication_number, type_string)
            
            return robot_panel
            
        except Exception as e:
            print(f"Error loading template UI: {str(e)}")
            return None
    
    def customizeCommunicationPanel(self, communication_panel, communication_number, type_string):
        """Customize the loaded template for specific communication instance"""
        if communication_panel: # ctkCollapsibleButton
           communication_panel.setText(f"{type_string} Communication {communication_number}")
        
        # Find and connect the Connect button
        connect_button = communication_panel.findChild(qt.QPushButton, "ConnectPushBtn")
        if connect_button:
            connect_button.clicked.connect(lambda: self.onConnectCommunication(communication_number, communication_panel, type_string))
        
        # Find and connect the Remove button
        remove_button = communication_panel.findChild(qt.QPushButton, "RemovePushBtn")
        if remove_button:
            remove_button.clicked.connect(lambda: self.onRemoveCommunication(communication_panel, communication_number, type_string))
        
        
    def onConnectCommunication(self, communication_number, communication_panel, type_string):

        print(f"Connecting {type_string} communication {communication_number}")
        if type_string == "Serial":
            port_name = self.__GetPortName(communication_panel)
            baud_rate = self.__GetBaudRate(communication_panel)
            print(f"Port name: {port_name}, Baud rate: {baud_rate}")
            # self.logic.createCommunication(communication_number, protocol='Serial', port=port_name, baudrate=baud_rate)
        
        pass

    def onRemoveCommunication(self, communication_panel, communication_number, type_string):
        """Remove a communication panel"""
        reply = qt.QMessageBox.question(None, "Confirm Removal", 
                                      f"Are you sure you want to remove {type_string} Communication {communication_number}?",
                                      qt.QMessageBox.Yes | qt.QMessageBox.No)
        
        if reply == qt.QMessageBox.Yes:
            
            self.logic.RemoveCommunication(communication_number)
            self.main_layout.removeWidget(communication_panel)
            if communication_panel in self.communication_panels:
                self.communication_panels.remove(communication_panel)
            communication_panel.hide()
            qt.QTimer.singleShot(0, communication_panel.deleteLater)
            if self.spacer_index is not None:
                self.spacer_index -= 1
            
            print(f"Removed Communication {communication_number}")
    
    def removeAllCommunicationPanels(self):
        for communication_panel in self.communication_panels:
            self.main_layout.removeWidget(communication_panel)
            communication_panel.hide()
            qt.QTimer.singleShot(0, communication_panel.deleteLater)
            if self.spacer_index is not None:
                self.spacer_index -= 1
        self.communication_panels = []
        self.communication_count = 0

    def onReload(self):
        self.removeAllCommunicationPanels()
        self.communication_count = 0
    
    def cleanup(self) -> None:
        """Called when the application closes and the module widget is destroyed."""
        self.removeObservers()

    def enter(self) -> None:
        """Called each time the user opens this module."""
        pass

    def exit(self) -> None:
        """Called each time the user opens a different module."""
        pass

    def onSceneStartClose(self, caller, event) -> None:
        """Called just before the scene is closed."""
        pass

    def onSceneEndClose(self, caller, event) -> None:
        """Called just after the scene is closed."""
        pass

    def __GetPortName(self, communication_panel):
        port_name_input = communication_panel.findChild(qt.QLineEdit, "PortNameInput")
        port = port_name_input.text if hasattr(port_name_input, 'text') else None
        return port

    def __GetBaudRate(self, communication_panel):
        baud_rate_input = communication_panel.findChild(qt.QLineEdit, "BaudRateInput")
        baud_rate = baud_rate_input.text if hasattr(baud_rate_input, 'text') else None
        return baud_rate
    
#
# SlicerCommLogic
#

class SlicerCommLogic(ScriptedLoadableModuleLogic):
    """This class should implement all the actual
    computation done by your module.  The interface
    should be such that other python code can import
    this class and make use of the functionality without
    requiring an instance of the Widget.
    Uses ScriptedLoadableModuleLogic base class, available at:
    https://github.com/Slicer/Slicer/blob/main/Base/Python/slicer/ScriptedLoadableModule.py
    """

    def __init__(self) -> None:
        """Called when the logic class is instantiated. Can be used for initializing member variables."""
        ScriptedLoadableModuleLogic.__init__(self)
        self.communication_panels = []
        self.communication_count = 1
        self.communications = {}

    def RemoveCommunication(self, communication_number):
        pass

    def createCommunication(self, communication_id, protocol='Serial', port=None, baudrate=9600, ip=None):
        """Create and configure a communication instance"""
       
        if protocol == 'Serial':
            communication = SerialCommunication(communication_id)
            if communication.connectSerialPort(port, baudrate):
                self.communications[communication_id] = communication
                return communication
            return None
        elif protocol == 'TCP/IP':
            communication = TCPIPCommunication(communication_id)
            if ip is None:
                if communication.startServer(port):
                    self.communications[communication_id] = communication
                    return communication
            elif communication.connect(ip, port):
                self.communications[communication_id] = communication
                return communication
            return None
        elif protocol == 'UDP':
            communication = UDPCommunication(communication_id)
            if communication.connect(ip, port):
                self.communications[communication_id] = communication
                return communication
            return None
        else:
            raise ValueError(f"Invalid protocol: {protocol}")

    def getParameterNode(self,communication_id):
        communication = self.communications[communication_id]
        return communication.getParameterNode()

#
# SlicerCommTest
#


class SlicerCommTest(ScriptedLoadableModuleTest):
    """
    This is the test case for your scripted module.
    Uses ScriptedLoadableModuleTest base class, available at:
    https://github.com/Slicer/Slicer/blob/main/Base/Python/slicer/ScriptedLoadableModule.py
    """

    def setUp(self):
        """Do whatever is needed to reset the state - typically a scene clear will be enough."""
        slicer.mrmlScene.Clear()

    def runTest(self):
        """Run as few or as many tests as needed here."""
        self.setUp()
        self.test_SlicerComm1()

    def test_SlicerComm1(self):
       pass
