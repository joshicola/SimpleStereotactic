"""
Module that governs all of the functionality of the SimpleStereotactic Slicer Extenstion
"""
import ast
import logging
import math
import os

import ctk
import numpy as np
import qt
import slicer
import vtk
from FiducialsToModelRegistration import FiducialsToModelRegistrationLogic
from slicer.ScriptedLoadableModule import (
    ScriptedLoadableModule,
    ScriptedLoadableModuleLogic,
    ScriptedLoadableModuleTest,
    ScriptedLoadableModuleWidget,
)


class kopf_frame(ScriptedLoadableModule):
    """
    Class for constructing the extension display.

    Uses ScriptedLoadableModule base class, available at:
    https://github.com/Slicer/Slicer/blob/master/Base/Python/slicer/ScriptedLoadableModule.py
    """

    def __init__(self, parent):
        super(kopf_frame, self).__init__(parent)
        self.parent.title = "Kopf Frame"
        self.parent.categories = ["SimpleStereotactic"]
        self.parent.dependencies = []
        self.parent.contributors = ["Joshua Jacobs, PhD"]
        self.parent.helpText = (
            "This is a simple stereotactic navigation tool for compass coordinates."
        )
        self.parent.acknowledgementText = (
            "This file was originally developed by Joshua Jacobs."
        )


class kopf_frameWidget(ScriptedLoadableModuleWidget):
    """
    Class for constructing the Widget controls and loading necessary files.

    Uses ScriptedLoadableModuleWidget base class, available at:
    https://github.com/Slicer/Slicer/blob/master/Base/Python/slicer/ScriptedLoadableModule.py
    """

    def __init__(self, parent):
        super(kopf_frameWidget, self).__init__(parent)
        # Setup neccessary elements
        self.curr_electrode = None
        self.switching_electrodes = False  # if we are "Switching Electrodes"
        # Ensures consistent distance between entry_point and target_point with
        # the use of the widgets
        self.point_moving = False
        # For adjusting red, green, yellow views when moving widgets
        self.slice_nodes = slicer.util.getNodesByClass("vtkMRMLSliceNode")
        # Targets
        self.targs = None
        # Initialize registration controls
        self.input_fiducial_selector = None
        self.input_volume_selector = None
        self.apply_reg_button = None
        # Initialize atlas registration controls
        self.atlas_fiducials = None
        self.scan_fiducials = None
        self.atlas_volume_selector = slicer.qMRMLNodeComboBox()
        self.atlas_register_button = None
        # Initialize targeting controls
        self.input_selector = None
        self.arc_angle_widget = None
        self.collar_angle_widget = None
        self.x_origin_widget = None
        self.y_origin_widget = None
        self.z_origin_widget = None
        self.new_electrode_button = None

    def setup(self):
        """
        Setup the user interface.
        """
        super(kopf_frameWidget, self).setup()

        # Instantiate and connect widgets ...
        # Load and Hide the following objects from Editors
        self.load_frame_mrb()
        self.hide_special_components()

        self.setup_registration_area()
        self.setup_atlas_registration_area()
        self.setup_targeting_area()
        # Create a Default Electrode
        self.new_electrode()
        # Refresh Apply button state
        self.on_electrode_select()

        # Add vertical spacer
        self.layout.addStretch(1)

        # Connect the movement of the x,y,z target to the movement of the
        # "Targets" Fiducial Node
        # Other events for FiducialNode listed in
        # (https://www.slicer.org/doc/html/classvtkMRMLMarkupsNode.html)
        self.targs.AddObserver(self.targs.PointEndInteractionEvent, self.point_moved)

    def load_frame_mrb(self):
        """
        Load the components for the simple frame from the embedded MRB file.
        """
        slicer.mrmlScene.Clear(0)
        mrml_path = slicer.modules.kopf_frame.path
        mrml_path = (
            mrml_path[: mrml_path.rfind("/")] + "/Resources/Models/kopf_frame.mrb"
        )
        slicer.util.loadScene(mrml_path)
        self.targs = slicer.util.getNode("Targets")

    def hide_special_components(self):
        """
        Hide components from editors and drop-down lists.
        """
        hidden_nodes = slicer.util.getNodes(pattern="kopf_*")

        for node in hidden_nodes.values():
            node.SetHideFromEditors(True)

    def setup_registration_area(self):
        """
        Setup image registration area and connect functions.
        """

        registration_collapsible_button = ctk.ctkCollapsibleButton()
        registration_collapsible_button.text = "Register Image to Frame"
        registration_collapsible_button.setDisabled(False)
        registration_collapsible_button.collapsed = False
        self.layout.addWidget(registration_collapsible_button)

        # Layout within the dummy collapsible button
        registration_form_layout = qt.QFormLayout(registration_collapsible_button)

        #
        # input fiducial list selector
        #

        fiducial_warning_label = qt.QLabel(
            "Note: Parent transforms of fiducials are not used. "
            "Fiducials should be defined in the coordinate system "
            "that is being registered."
        )
        fiducial_warning_label.setWordWrap(True)
        registration_form_layout.addRow(fiducial_warning_label)

        self.input_fiducial_selector = slicer.qMRMLNodeComboBox()
        self.input_fiducial_selector.nodeTypes = (("vtkMRMLMarkupsFiducialNode"), "")
        self.input_fiducial_selector.selectNodeUponCreation = False
        self.input_fiducial_selector.addEnabled = False
        self.input_fiducial_selector.removeEnabled = False
        self.input_fiducial_selector.noneEnabled = True
        self.input_fiducial_selector.showHidden = False
        self.input_fiducial_selector.showChildNodeTypes = False
        self.input_fiducial_selector.setMRMLScene(slicer.mrmlScene)
        self.input_fiducial_selector.setToolTip(
            "Pick the input fiducial list for the algorithm."
        )
        registration_form_layout.addRow(
            "Input fiducials: ", self.input_fiducial_selector
        )

        #
        # input volume selector
        #
        self.input_volume_selector = slicer.qMRMLNodeComboBox()
        self.input_volume_selector.nodeTypes = (("vtkMRMLScalarVolumeNode"), "")
        self.input_volume_selector.selectNodeUponCreation = False
        self.input_volume_selector.addEnabled = False
        self.input_volume_selector.removeEnabled = False
        self.input_volume_selector.noneEnabled = True
        self.input_volume_selector.showHidden = False
        self.input_volume_selector.showChildNodeTypes = False
        self.input_volume_selector.setMRMLScene(slicer.mrmlScene)
        self.input_volume_selector.setToolTip(
            "Pick the input volume for the algorithm."
        )
        registration_form_layout.addRow("Input Volume: ", self.input_volume_selector)

        # Register Button
        self.apply_reg_button = qt.QPushButton("Apply")
        self.apply_reg_button.toolTip = "Run the algorithm."
        self.apply_reg_button.enabled = False
        registration_form_layout.addRow(self.apply_reg_button)

        # Connect Procedures
        # Image to Frame Registration
        self.apply_reg_button.connect("clicked(bool)", self.onapply_reg_button)
        self.input_volume_selector.connect(
            "currentNodeChanged(vtkMRMLNode*)", self.on_registration_select
        )
        self.input_fiducial_selector.connect(
            "currentNodeChanged(vtkMRMLNode*)", self.on_registration_select
        )

    def setup_atlas_registration_area(self):
        """
        Setup atlas registration area and connect functions.
        """

        atlas_registration_collapsible_button = ctk.ctkCollapsibleButton()
        atlas_registration_collapsible_button.text = "Register Atlas to Frame"
        atlas_registration_collapsible_button.setDisabled(False)
        self.layout.addWidget(atlas_registration_collapsible_button)

        # Layout within the dummy collapsible button
        atlas_registration_form_layout = qt.QFormLayout(
            atlas_registration_collapsible_button
        )

        self.atlas_fiducials = slicer.qMRMLNodeComboBox()
        self.atlas_fiducials.nodeTypes = ["vtkMRMLMarkupsFiducialNode"]
        self.atlas_fiducials.selectNodeUponCreation = True
        self.atlas_fiducials.addEnabled = True
        self.atlas_fiducials.removeEnabled = True
        self.atlas_fiducials.noneEnabled = True
        self.atlas_fiducials.showHidden = True
        self.atlas_fiducials.showChildNodeTypes = True
        self.atlas_fiducials.setMRMLScene(slicer.mrmlScene)
        self.atlas_fiducials.setToolTip("Pick the input to the algorithm.")

        atlas_registration_form_layout.addRow(
            "Atlas AC/PC/Sup1/Sup2 Fiducials", self.atlas_fiducials
        )

        self.scan_fiducials = slicer.qMRMLNodeComboBox()
        self.scan_fiducials.nodeTypes = ["vtkMRMLMarkupsFiducialNode"]
        self.scan_fiducials.selectNodeUponCreation = True
        self.scan_fiducials.addEnabled = True
        self.scan_fiducials.removeEnabled = True
        self.scan_fiducials.noneEnabled = True
        self.scan_fiducials.showHidden = True
        self.scan_fiducials.showChildNodeTypes = True
        self.scan_fiducials.setMRMLScene(slicer.mrmlScene)
        self.scan_fiducials.setToolTip("Pick the input to the algorithm.")

        atlas_registration_form_layout.addRow(
            "Scan AC/PC/Sup1/Sup2 Fiducials", self.scan_fiducials
        )

        # atlas volume selector
        #

        self.atlas_volume_selector.nodeTypes = (("vtkMRMLLabelMapVolumeNode"), "")
        self.atlas_volume_selector.selectNodeUponCreation = False
        self.atlas_volume_selector.addEnabled = False
        self.atlas_volume_selector.removeEnabled = False
        self.atlas_volume_selector.noneEnabled = True
        self.atlas_volume_selector.showHidden = False
        self.atlas_volume_selector.showChildNodeTypes = False
        self.atlas_volume_selector.setMRMLScene(slicer.mrmlScene)
        self.atlas_volume_selector.setToolTip(
            "Pick the input volume for the algorithm."
        )
        atlas_registration_form_layout.addRow(
            "Atlas Volume: ", self.atlas_volume_selector
        )

        #
        # Register Button
        #
        self.atlas_register_button = qt.QPushButton("Register Atlas to Frame")
        self.atlas_register_button.toolTip = "Run the algorithm."
        self.atlas_register_button.enabled = False
        atlas_registration_form_layout.addRow(self.atlas_register_button)

        # Connect Procedures
        self.atlas_fiducials.connect(
            "currentNodeChanged(vtkMRMLNode*)", self.on_atlas_registration_select
        )
        self.scan_fiducials.connect(
            "currentNodeChanged(vtkMRMLNode*)", self.on_atlas_registration_select
        )
        self.atlas_volume_selector.connect(
            "currentNodeChanged(vtkMRMLNode*)", self.on_atlas_registration_select
        )
        # Atlas to Frame Registration
        self.atlas_register_button.connect("clicked(bool)", self.register_ac_pc)

    def default_angle_component(
        self, tool_tip_text, def_val=90, min_val=0, max_val=180
    ):
        """
        Create and initialize angle widget

        Returns:
            ctk.ctkSliderWidget(): Initalized angle widget
        """
        angle_widget = ctk.ctkSliderWidget()
        angle_widget.singleStep = 0.1
        angle_widget.minimum = min_val
        angle_widget.maximum = max_val
        angle_widget.value = def_val
        angle_widget.setToolTip(tool_tip_text)

        return angle_widget

    def default_origin_component(
        self, tool_tip_text, def_val=100, min_val=50, max_val=150
    ):
        """
        Create and initialize origin widget.

        Returns:
            ctk.ctkDoubleSpinBox(): Initialized origin widget.
        """
        origin_widget = ctk.ctkDoubleSpinBox()
        origin_widget.singleStep = 0.1
        origin_widget.minimum = min_val
        origin_widget.maximum = max_val
        origin_widget.value = def_val
        origin_widget.setToolTip(tool_tip_text)
        return origin_widget

    def setup_targeting_area(self):
        """
        Setup target and trajectory area and connect functions.
        """
        parameters_collapsible_button = ctk.ctkCollapsibleButton()
        parameters_collapsible_button.text = "Target and Trajectory"
        self.layout.addWidget(parameters_collapsible_button)

        # Layout within the dummy collapsible button
        parameters_form_layout = qt.QFormLayout(parameters_collapsible_button)

        # input model selector
        self.input_selector = slicer.qMRMLNodeComboBox()
        self.input_selector.nodeTypes = ["vtkMRMLModelNode"]
        self.input_selector.selectNodeUponCreation = False
        self.input_selector.addEnabled = False
        self.input_selector.removeEnabled = False
        self.input_selector.noneEnabled = False
        self.input_selector.showHidden = False
        self.input_selector.showChildNodeTypes = False
        self.input_selector.setMRMLScene(slicer.mrmlScene)
        self.input_selector.setToolTip("Pick the Electrode to Target")
        parameters_form_layout.addRow("Target Electrode: ", self.input_selector)

        # Arc Angle value
        self.arc_angle_widget = self.default_angle_component(
            "Set Arc angle of approach.", 0, -180, 180
        )
        parameters_form_layout.addRow("Arc", self.arc_angle_widget)

        # Collar Angle value
        self.collar_angle_widget = self.default_angle_component(
            "Set Collar angle of approach", 90, 0, 180
        )
        parameters_form_layout.addRow("Collar", self.collar_angle_widget)

        # X-origin
        self.x_origin_widget = self.default_origin_component("X", 0, -40, 40)
        parameters_form_layout.addRow("X0", self.x_origin_widget)

        # Y-origin
        self.y_origin_widget = self.default_origin_component("Y", 0, -40, 40)
        parameters_form_layout.addRow("Y0", self.y_origin_widget)

        # Z-origin
        self.z_origin_widget = self.default_origin_component("Z", 0, -40, 40)
        parameters_form_layout.addRow("Z0", self.z_origin_widget)

        # Apply Button
        self.new_electrode_button = qt.QPushButton("New Electrode")
        self.new_electrode_button.toolTip = "Generate New Electrode"
        self.new_electrode_button.enabled = False
        parameters_form_layout.addRow(self.new_electrode_button)

        # Connect Procedures
        # Target and Trajectory
        self.new_electrode_button.connect("clicked(bool)", self.new_electrode)
        self.input_selector.connect(
            "currentNodeChanged(vtkMRMLNode*)", self.on_electrode_select
        )

        self.arc_angle_widget.connect("valueChanged(double)", self.on_changes)
        self.collar_angle_widget.connect("valueChanged(double)", self.on_changes)

        self.x_origin_widget.connect("valueChanged(double)", self.on_changes)
        self.y_origin_widget.connect("valueChanged(double)", self.on_changes)
        self.z_origin_widget.connect("valueChanged(double)", self.on_changes)

    def cleanup(self):
        """
        Cleanup extra variables.
        """
        pass

    def on_registration_select(self):
        """
        Enable registration button on select condition.
        """
        self.apply_reg_button.enabled = (
            self.input_volume_selector.currentNode()
            and self.input_fiducial_selector.currentNode()
        )

    def onapply_reg_button(self):
        """
        Perform fiducial registration to the wireframe.
        """
        # This is where we ensure appropriate requirements of the registration technique
        logic = FiducialsToModelRegistrationLogic()

        input_fiducials = self.input_fiducial_selector.currentNode()
        input_model = slicer.util.getNode("Frame_Template_wf")
        input_volume = self.input_volume_selector.currentNode()

        transform_id = input_volume.GetTransformNodeID()

        if transform_id == input_fiducials.GetTransformNodeID() and transform_id:
            # they both have the same transform
            output_transform = slicer.util.getNode(transform_id)
        elif not transform_id:
            # Neither of them have a transform
            output_transform = slicer.mrmlScene.AddNewNodeByClass(
                "vtkMRMLLinearTransformNode", "transRegToFrame"
            )
            input_fiducials.SetAndObserveTransformNodeID(output_transform.GetID())
            input_volume.SetAndObserveTransformNodeID(output_transform.GetID())
        else:
            # Harden Transform, Make it identity, put Volume and Fiducials in
            output_transform = slicer.util.GetNode(transform_id)
            input_volume.HardenTransform()
            input_fiducials.HardenTransform()
            mat = vtk.vtkMatrix4x4()
            mat.Identity()
            output_transform.SetMatrixTransformToParent(mat)
            input_fiducials.SetAndObserveTransformNodeID(output_transform.GetID())
            input_volume.SetAndObserverTransformNodeID(output_transform.GetID())

        logic.run(input_fiducials, input_model, output_transform, 0, 100)

    def on_atlas_registration_select(self):
        """
        Enable atlas registration button on select condition.
        """
        self.atlas_register_button.enabled = (
            self.atlas_volume_selector.currentNode()
            and self.atlas_fiducials.currentNode()
            and self.scan_fiducials.currentNode()
        )

    def register_ac_pc(self):
        """
        Register atlas to image using fiducials representing AC/PC.

        Anterior Commissure and Posterior Commissure
        """
        atlas_fiducials = self.atlas_fiducials.currentNode()
        scan_fiducials = self.scan_fiducials.currentNode()
        atlas_volume = self.atlas_volume_selector.currentNode()

        transform_id = atlas_volume.GetTransformNodeID()

        if transform_id == atlas_fiducials.GetTransformNodeID() and transform_id:
            # they both have the same transform
            output_transform = slicer.util.getNode(transform_id)
        elif not transform_id:
            # Neither of them have a transform
            output_transform = slicer.mrmlScene.AddNewNodeByClass(
                "vtkMRMLLinearTransformNode", "transAtlasToScan"
            )
            atlas_fiducials.SetAndObserveTransformNodeID(output_transform.GetID())
            atlas_volume.SetAndObserveTransformNodeID(output_transform.GetID())
        else:
            # Harden Transform, Make it identity, put Volume and Fiducials in
            output_transform = slicer.util.getNode(transform_id)
            atlas_volume.HardenTransform()
            atlas_fiducials.HardenTransform()
            mat = vtk.vtkMatrix4x4()
            mat.Identity()
            output_transform.SetMatrixTransformToParent(mat)
            atlas_fiducials.SetAndObserveTransformNodeID(output_transform.GetID())
            atlas_volume.SetAndObserverTransformNodeID(output_transform.GetID())

        atlas_transform = slicer.util.getNode("transAtlasToScan")

        atlas_points = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])

        atlas_fiducials.GetNthFiducialPosition(0, atlas_points[0])
        atlas_fiducials.GetNthFiducialPosition(1, atlas_points[1])

        scan_points = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])

        scan_fiducials.GetNthFiducialPosition(0, scan_points[0])
        scan_fiducials.GetNthFiducialPosition(1, scan_points[1])

        # Get Vectors of each position
        vector_1 = atlas_points[1] - atlas_points[0]
        vector_2 = scan_points[1] - scan_points[0]
        # Cross product to define perpendicular vector to rotate around
        vector_3 = np.cross(vector_1, vector_2)
        scale = np.linalg.norm(vector_2) / np.linalg.norm(vector_1)
        # The angle to rotate around
        theta = (
            np.arccos(
                np.dot(vector_1, vector_2)
                / (np.linalg.norm(vector_1) * np.linalg.norm(vector_2))
            )
            * 360.0
            / (2 * np.pi)
        )

        transform_scaling = vtk.vtkTransform()
        transform_scaling.PostMultiply()
        transform_scaling.Translate(
            -atlas_points[0, 0], -atlas_points[0, 1], -atlas_points[0, 2]
        )  # Translate the points to origin
        transform_scaling.Scale(scale, scale, scale)  # Scale
        transform_scaling.RotateWXYZ(
            theta, vector_3
        )  # Rotate system around perpendicular vector
        transform_scaling.Translate(
            scan_points[0, 0], scan_points[0, 1], scan_points[0, 2]
        )  # Translate to second point set
        atlas_transform.SetAndObserveMatrixTransformToParent(
            transform_scaling.GetMatrix()
        )

    def point_moved(self, caller, event):
        """
        Updates electrode attributes to match positions of entry and target points.

        Jumps slices to coordinates of point moved.

        Args:
            caller (vtkSlicerMarkupsModuleMRMLPython.vtkMRMLMarkupsFiducialNode):
                The target/entry fiducial that called
            event ([type]): Not used, but required in the function description.
        """
        self.point_moving = True
        moving_point_indx = int(caller.GetAttribute("Markups.MovingMarkupIndex"))
        moving_point_indx = int(moving_point_indx)
        caller.SetNthFiducialVisibility(moving_point_indx, False)
        coord = [0.0, 0.0, 0.0]
        caller.GetNthFiducialPosition(moving_point_indx, coord)
        self.update_coords_from_fiducials()
        caller.SetNthFiducialVisibility(moving_point_indx, True)
        slicer.modules.markups.logic().JumpSlicesToLocation(
            coord[0], coord[1], coord[2], True
        )
        self.point_moving = False

    def update_coords_from_fiducials(self):
        """
        Updates form widgets to values calculated from target and entry points.
        """
        entry_point = [0.0, 0.0, 0.0]
        target_point = [0.0, 0.0, 0.0]
        self.targs.GetNthFiducialPosition(1, entry_point)
        self.targs.GetNthFiducialPosition(0, target_point)

        self.x_origin_widget.setValue(-1 * target_point[0])
        self.y_origin_widget.setValue(target_point[1])
        self.z_origin_widget.setValue(-1 * target_point[2])

        # Adjust target for widget constraints applied in lines above:
        target_point = [
            -1 * self.x_origin_widget.value,
            self.y_origin_widget.value,
            -1 * self.z_origin_widget.value,
        ]

        [arc, collar] = self.targets_to_frame(target_point, entry_point)

        self.arc_angle_widget.setValue(arc)
        self.collar_angle_widget.setValue(collar)

    def targets_to_frame(self, target_point, entry_point):
        # TODO: Export to logic Candidate
        """
        Convert target point and point of entry to arc and collar angles.

        To convert a formal description of a cylinder, the target point, and the point
        of entry into coordinates for a cylindrically shaped stereotactic head frame.

        Creates the arc and collar angles neccessary for a trajectory to go through
        points target_point and entry_point.

        Args:
            target_point (numpy.array): The Target Point
            entry_point (numpy.array): Point of entry into the subject

        Returns:
            tuple: arc degrees, collar degrees
        """

        # unit vector from target to entry point
        target_point = np.array(target_point)
        entry_point = np.array(entry_point)
        r_vector = np.array(entry_point - target_point)
        dist = np.linalg.norm(r_vector)  # The distance between the points.
        phi = np.array([0.0, 0.0])  # arc and collar

        phi[0] = np.arccos(r_vector[0] / dist)

        if r_vector[1] != 0:
            phi[1] = np.arctan(r_vector[2] / r_vector[1])
        else:
            phi[1] = np.pi / 2.0
        if phi[1] < 0:
            phi[1] = np.pi + phi[1]

        return [math.degrees(phi[0]), math.degrees(phi[1])]

    def on_electrode_select(self):
        """
        Update form properties to selected electrode attributes.
        """
        self.new_electrode_button.enabled = self.input_selector.currentNode()
        # Get Current Electrode
        selected_electrode = self.input_selector.currentNode()
        if (
            (selected_electrode != self.curr_electrode)
            and selected_electrode
            and self.curr_electrode
            and slicer.util.getFirstNodeByName(
                "Transform_" + self.curr_electrode.GetName()
            )
        ):
            # Switching Electrodes set to true so not to prematurely update
            # Electrode Properties.
            # Yes, this could be applied more "intelligently"
            self.switching_electrodes = True
            # Harden transform
            # logic=slicer.vtkSlicerTransformLogic()
            # logic.hardenTransform(self.curr_electrode)
            electrode_transform = slicer.util.getNode(
                "Transform_" + self.curr_electrode.GetName()
            )
            self.curr_electrode.SetAndObserveTransformNodeID(
                electrode_transform.GetID()
            )
            self.curr_electrode = selected_electrode

            x_0 = float(selected_electrode.GetAttribute("X"))
            y_0 = float(selected_electrode.GetAttribute("Y"))
            z_0 = float(selected_electrode.GetAttribute("Z"))
            self.x_origin_widget.setValue(x_0)
            self.y_origin_widget.setValue(y_0)
            self.z_origin_widget.setValue(z_0)
            self.targs.SetNthFiducialPosition(0, -1 * x_0, y_0, -1 * z_0)
            self.arc_angle_widget.setValue(
                float(selected_electrode.GetAttribute("Arc"))
            )
            self.collar_angle_widget.setValue(
                float(selected_electrode.GetAttribute("Collar"))
            )
            selected_electrode.SetAndObserveTransformNodeID(
                slicer.util.getNode("Arc").GetID()
            )
            entry_point = ast.literal_eval(selected_electrode.GetAttribute("Entry"))
            self.targs.SetNthFiducialPosition(
                1, entry_point[0], entry_point[1], entry_point[2]
            )

            self.update_nodes()

            self.switching_electrodes = False
            self.save_electrode_properties(self.curr_electrode)

    def on_changes(self):
        """
        On changes to form coordinates widget save properties in electrode attributes.
        """
        self.curr_electrode = self.input_selector.currentNode()

        if self.new_electrode_button.enabled:
            self.update_nodes()

        self.save_electrode_properties(self.curr_electrode)

    def update_nodes(self):
        """
        Update all relavant transform and fiducial nodes to correspond to form entries.
        """
        arc_angle = -1 * (self.arc_angle_widget.value)
        collar_angle = self.collar_angle_widget.value - 90
        x_0 = self.x_origin_widget.value
        y_0 = self.y_origin_widget.value
        z_0 = self.z_origin_widget.value

        # The Frame is set such that Frame coordinate 100,100,100 is set to the origin
        # of the Slicer Space (0,0,0)
        # Future iterations need to align Frame Coordinate to Slicer Space and adjust
        # the appropriate Frame elements accordingly.

        # Set X Translation
        target_x_transform = slicer.util.getNode("Target_X")
        trans = vtk.vtkTransform()
        trans.Translate(1 * (x_0), 0, 0)  # y_0,-1*z_0)
        target_x_transform.SetMatrixTransformToParent(trans.GetMatrix())

        # Set Y Translation
        target_y_transform = slicer.util.getNode("Target_Y")
        trans = vtk.vtkTransform()
        trans.Translate(0, y_0, 0)  # y_0,-1*z_0)
        target_y_transform.SetMatrixTransformToParent(trans.GetMatrix())

        # Set Z Translation
        target_z_transform = slicer.util.getNode("Target_Z")
        trans = vtk.vtkTransform()
        trans.Translate(0, 0, 1 * (z_0))  # y_0,-1*z_0)
        target_z_transform.SetMatrixTransformToParent(trans.GetMatrix())

        # Set Collar Rotation
        collar_transform = slicer.util.getNode("Collar")
        trans.Identity()
        trans.RotateX(collar_angle)
        collar_transform.SetMatrixTransformToParent(trans.GetMatrix())

        # Set Arc Rotation
        arc_transform = slicer.util.getNode("Arc")
        trans.Identity()
        trans.RotateZ(arc_angle)
        arc_transform.SetMatrixTransformToParent(trans.GetMatrix())

        self.targs.SetNthFiducialPosition(0, -1 * x_0, y_0, -1 * z_0)
        entry_point = np.array([0.0, 0.0, 0.0])
        target_point = np.array([0.0, 0.0, 0.0])

        self.targs.GetNthFiducialPosition(1, entry_point)
        self.targs.GetNthFiducialPosition(0, target_point)
        if not self.point_moving:
            point_difference = entry_point - target_point
            dist = np.linalg.norm(point_difference)

            # With the target point as a constant, set the entry point
            # when moving the controls.
            point_difference[0] = dist * np.cos(math.radians(90 + arc_angle))
            dist_p = dist * np.sin(math.radians(90 + arc_angle))

            point_difference[1] = dist_p * np.cos(math.radians(90 + collar_angle))
            point_difference[2] = dist_p * np.sin(math.radians(90 + collar_angle))

            entry_point = target_point + point_difference * np.array([-1, 1, 1])
            self.targs.SetNthFiducialPosition(
                1, entry_point[0], entry_point[1], entry_point[2]
            )
            # When using x,y,z,collar,arc widgets adjust view in triptych windows
            for slice_node in self.slice_nodes:
                slice_node.SetActiveSlice(1)
                slice_node.SetActiveSlice(0)

    def save_electrode_properties(self, electrode):
        """
        Save electrode properties to model attributes.

        Args:
            electrode (vtkMRMLModelNode): Electrode to save properties of.
        """
        if not self.switching_electrodes:
            # Save its full transform
            electrode_transform = slicer.vtkMRMLLinearTransformNode()
            trans = vtk.vtkMatrix4x4()

            # create an inverse
            electrode_transform.GetMatrixTransformFromNode(
                slicer.util.getNode("Arc"), trans
            )
            electrode_transform = slicer.util.getNode(
                "Transform_" + electrode.GetName()
            )
            electrode_transform.SetMatrixTransformToParent(trans)

            # save specific values
            electrode.SetAttribute("X", str(self.x_origin_widget.value))
            electrode.SetAttribute("Y", str(self.y_origin_widget.value))
            electrode.SetAttribute("Z", str(self.z_origin_widget.value))
            electrode.SetAttribute("Arc", str(self.arc_angle_widget.value))
            electrode.SetAttribute("Collar", str(self.collar_angle_widget.value))

            entry_point = np.array([0.0, 0.0, 0.0])
            self.targs.GetNthFiducialPosition(1, entry_point)
            electrode.SetAttribute("Entry", str(list(entry_point)))

    def new_electrode(self):
        """
        Create new electrode model and paired,hidden transform.
        """
        if self.curr_electrode:
            # Get Current Electrode
            self.curr_electrode = self.input_selector.currentNode()
            self.save_electrode_properties(self.curr_electrode)

            # Harden transform
            # logic=slicer.vtkSlicerTransformLogic()
            # logic.hardenTransform(self.curr_electrode)
            electrode_transform = slicer.util.getNode(
                "Transform_" + self.curr_electrode.GetName()
            )
            self.curr_electrode.SetAndObserveTransformNodeID(
                electrode_transform.GetID()
            )

        # Set up new electrode
        origin = [0, 0, 0]
        terminus = [0, 0, 200]

        # This section takes care of the "Electrode"
        line = vtk.vtkLineSource()
        line.SetPoint1(origin)
        line.SetPoint2(terminus)
        line.Update()

        tube_filter = vtk.vtkTubeFilter()
        tube_filter.SetCapping(True)
        tube_filter.SetRadius(1.1)
        tube_filter.SetNumberOfSides(20)

        tube_filter.SetInputDataObject(line.GetOutput())
        tube_filter.Update()

        electrode = tube_filter.GetOutput()

        new_electrode = slicer.vtkMRMLModelNode()
        new_electrode.SetAndObservePolyData(electrode)

        slicer.mrmlScene.AddNode(new_electrode)

        electrode_display_node = slicer.vtkMRMLModelDisplayNode()
        slicer.mrmlScene.AddNode(electrode_display_node)

        electrode_display_node.SetName("NeedleModelDisplay")
        electrode_display_node.SetColor(0.0, 1.0, 1.0)

        new_electrode.SetAndObserveDisplayNodeID(electrode_display_node.GetID())
        electrode_display_node.SetAmbient(0.2)
        electrode_display_node.SetSliceIntersectionVisibility(True)

        new_electrode.SetAndObserveTransformNodeID(slicer.util.getNode("Arc").GetID())
        self.curr_electrode = new_electrode
        electrode_transform = slicer.vtkMRMLLinearTransformNode()
        electrode_transform.SetName("Transform_" + self.curr_electrode.GetName())
        electrode_transform.SetHideFromEditors(True)
        slicer.mrmlScene.AddNode(electrode_transform)
        self.save_electrode_properties(new_electrode)
        self.input_selector.setCurrentNodeID(new_electrode.GetID())

        # The "Logic" has yet to be constructed...along with the unit tests
        # for the kopf_frame


# TODO: How much of the computation could I port into this Logic Module?
class kopf_frameLogic(ScriptedLoadableModuleLogic):
    """This class should implement all the actual
    computation done by your module.  The interface
    should be such that other python code can import
    this class and make use of the functionality without
    requiring an instance of the Widget.
    Uses ScriptedLoadableModuleLogic base class, available at:
    https://github.com/Slicer/Slicer/blob/master/Base/Python/slicer/ScriptedLoadableModule.py
    """

    def has_image_data(self, volume_node):
        """This is an example logic method that
        returns true if the passed in volume
        node has valid image data
        """
        if not volume_node:
            logging.debug("has_image_data failed: no volume node")
            return False
        if volume_node.GetImageData() is None:
            logging.debug("has_image_data failed: no image data in volume node")
            return False
        return True

    def isValidInputOutputData(self, input_volumeNode, output_volume_node):
        """Validates if the output is not the same as input"""
        if not input_volumeNode:
            logging.debug("isValidInputOutputData failed: no input volume node defined")
            return False
        if not output_volume_node:
            logging.debug(
                "isValidInputOutputData failed: no output volume node defined"
            )
            return False
        if input_volumeNode.GetID() == output_volume_node.GetID():
            logging.debug(
                "isValidInputOutputData failed: input and output volume is the same. "
                "Create a new volume for output to avoid this error."
            )
            return False
        return True

    def takeScreenshot(self, name, description, shot_type=-1):
        # show the message even if not taking a screen shot
        slicer.util.delayDisplay(
            "Take screenshot: "
            + description
            + ".\nResult is available in the Annotations module.",
            3000,
        )

        layout_manager = slicer.app.layoutManager()
        # switch on the shot_type to get the requested window
        widget = 0
        if shot_type == slicer.qMRMLScreenShotDialog.FullLayout:
            # full layout
            widget = layout_manager.viewport()
        elif shot_type == slicer.qMRMLScreenShotDialog.ThreeD:
            # just the 3D window
            widget = layout_manager.threeDWidget(0).threeDView()
        elif shot_type == slicer.qMRMLScreenShotDialog.Red:
            # red slice window
            widget = layout_manager.sliceWidget("Red")
        elif shot_type == slicer.qMRMLScreenShotDialog.Yellow:
            # yellow slice window
            widget = layout_manager.sliceWidget("Yellow")
        elif shot_type == slicer.qMRMLScreenShotDialog.Green:
            # green slice window
            widget = layout_manager.sliceWidget("Green")
        else:
            # default to using the full window
            widget = slicer.util.mainWindow()
            # reset the shot_type so that the node is set correctly
            shot_type = slicer.qMRMLScreenShotDialog.FullLayout

        # grab and convert to vtk image data
        qpix_map = qt.QPixmap().grabWidget(widget)
        qimage = qpix_map.toImage()
        image_data = vtk.vtkImageData()
        slicer.qMRMLUtils().qImageToVtkImageData(qimage, image_data)

        annotation_logic = slicer.modules.annotations.logic()
        annotation_logic.CreateSnapShot(name, description, shot_type, 1, image_data)

    def run(self, input_volume, output_volume, image_threshold, enable_screenshots=0):
        """
        Run the actual algorithm
        """

        if not self.isValidInputOutputData(input_volume, output_volume):
            slicer.util.errorDisplay(
                "Input volume is the same as output volume. "
                "Choose a different output volume."
            )
            return False

        logging.info("Processing started")

        # Compute the thresholded output volume using the Threshold Scalar Volume
        # CLI module
        cli_params = {
            "InputVolume": input_volume.GetID(),
            "OutputVolume": output_volume.GetID(),
            "ThresholdValue": image_threshold,
            "ThresholdType": "Above",
        }
        cli_node = slicer.cli.run(
            slicer.modules.thresholdscalarvolume,
            None,
            cli_params,
            wait_for_completion=True,
        )

        # Capture screenshot
        if enable_screenshots:
            self.takeScreenshot("kopf_frameTest-Start", "MyScreenshot", -1)

        logging.info("Processing completed")

        return True


# TODO: Run some tests on what data we have.
class kopf_frameTest(ScriptedLoadableModuleTest):
    """
    This is the test case for your scripted module.
    Uses ScriptedLoadableModuleTest base class, available at:
    https://github.com/Slicer/Slicer/blob/master/Base/Python/slicer/ScriptedLoadableModule.py
    """

    def setup(self):
        """
        Do whatever is needed to reset the state - typically a scene clear
        will be enough.
        """
        slicer.mrmlScene.Clear(0)

    def run_test(self):
        """Run as few or as many tests as needed here."""
        self.setup()
        self.test_simple_frame_1()

    def test_simple_frame_1(self):
        """Ideally you should have several levels of tests.  At the lowest level
        tests should exercise the functionality of the logic with different inputs
        (both valid and invalid).  At higher levels your tests should emulate the
        way the user would interact with your code and confirm that it still works
        the way you intended.
        One of the most important features of the tests is that it should alert other
        developers when their changes will have an impact on the behavior of your
        module.  For example, if a developer removes a feature that you depend on,
        your test should break so they know that the feature is needed.
        """

        self.delayDisplay("Starting the test")
        #
        # first, get some data
        #
        import urllib

        downloads = (
            (
                "http://slicer.kitware.com/midas3/download?items=5767",
                "FA.nrrd",
                slicer.util.loadVolume,
            ),
        )

        for url, name, loader in downloads:
            file_path = slicer.app.temporaryPath + "/" + name
            if not os.path.exists(file_path) or os.stat(file_path).st_size == 0:
                logging.info("Requesting download %s from %s...\n" % (name, url))
                urllib.urlretrieve(url, file_path)
            if loader:
                logging.info("Loading %s..." % (name,))
                loader(file_path)
        self.delayDisplay("Finished with download and loading")

        volume_node = slicer.util.getNode(pattern="FA")
        logic = kopf_frameLogic()
        self.assertIsNotNone(logic.has_image_data(volume_node))
        self.delayDisplay("Test passed!")
