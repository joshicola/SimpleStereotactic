import os, numpy as np, math
import unittest
import vtk, qt, ctk, slicer
from slicer.ScriptedLoadableModule import *
import logging
from platform import node
from FiducialsToModelRegistration import FiducialsToModelRegistrationLogic

#
# PigFrame
#

class PigFrame(ScriptedLoadableModule):
  """Uses ScriptedLoadableModule base class, available at:
  https://github.com/Slicer/Slicer/blob/master/Base/Python/slicer/ScriptedLoadableModule.py
  """

  def __init__(self, parent):
    ScriptedLoadableModule.__init__(self, parent)
    self.parent.title = "PigFrame" # TODO make this more human readable by adding spaces
    self.parent.categories = ["SimpleStereotactic"]
    self.parent.dependencies = []
    self.parent.contributors = ["Joshua Jacobs PhD (Mayo Clinic)"]
    self.parent.helpText = """
    This is an example of scripted loadable module bundled in an extension.
    It performs a simple thresholding on the input volume and optionally captures a screenshot.
    """
    self.parent.acknowledgementText = """
    This file was originally developed by Joshua Jacobs."""

#
# PigFrameWidget
#

class PigFrameWidget(ScriptedLoadableModuleWidget):
  """Uses ScriptedLoadableModuleWidget base class, available at:
  https://github.com/Slicer/Slicer/blob/master/Base/Python/slicer/ScriptedLoadableModule.py
  """

  def setup(self):
    ScriptedLoadableModuleWidget.setup(self)
    
    #Setup neccessary elements
    self.currElectrode=None
    self.switchingElectrodes=False #if we are "Switching Electrodes"
    slicer.mrmlScene.Clear(0)
    
    #Load and Hide the following objects from Editors
    #ToDo: Load from Extension-Relative Path
    mrmlPath=slicer.modules.pigframe.path
    mrmlPath=mrmlPath[:mrmlPath.rfind('/')]+'/Resources/Models/NewPigFrameRed.mrml'
    slicer.util.loadScene(mrmlPath)

    Nodes=slicer.util.getNodes(pattern='*NEL2016*')
    node=slicer.util.getNode('Target')
    Nodes[node.GetName]=node
    node=slicer.util.getNode('Frame_Template')
    Nodes[node.GetName]=node
    node=slicer.util.getNode('Frame_Template_wf')
    Nodes[node.GetName]=node
    for node in Nodes.values():
        node.SetHideFromEditors(True)

    self.SliceNodes=slicer.util.getNodesByClass('vtkMRMLSliceNode')

    self.PointMoving=False
    # Instantiate and connect widgets ...

    #
    # Acquire Images Area
    #
    AcquireImageCollapsibleButton = ctk.ctkCollapsibleButton()
    AcquireImageCollapsibleButton.text = "Acquire Images"
    AcquireImageCollapsibleButton.setDisabled(True)
    AcquireImageCollapsibleButton.collapsed=True

    self.layout.addWidget(AcquireImageCollapsibleButton)
    
    # Layout within the dummy collapsible button
    AcquireImageFormLayout = qt.QFormLayout(AcquireImageCollapsibleButton)

    self.ChooseImgCbo=slicer.qMRMLNodeComboBox()
    self.ChooseImgCbo.nodeTypes=["vtkMRMLVolumeNode"]
    self.ChooseImgCbo.selectNodeUponCreation = True
    self.ChooseImgCbo.addEnabled = False
    self.ChooseImgCbo.removeEnabled = False
    self.ChooseImgCbo.noneEnabled = True
    self.ChooseImgCbo.showHidden = True
    self.ChooseImgCbo.showChildNodeTypes = True
    self.ChooseImgCbo.setMRMLScene( slicer.mrmlScene )
    self.ChooseImgCbo.setToolTip( "Pick the input to the algorithm.")

    AcquireImageFormLayout.addRow("Choose Image",self.ChooseImgCbo)



    #
    # Registration Area
    #
    registrationCollapsibleButton = ctk.ctkCollapsibleButton()
    registrationCollapsibleButton.text = "Register Image to Frame"
    registrationCollapsibleButton.setDisabled(False)
    registrationCollapsibleButton.collapsed=False
    self.layout.addWidget(registrationCollapsibleButton)
    
    # Layout within the dummy collapsible button
    registrationFormLayout = qt.QFormLayout(registrationCollapsibleButton)

    #
    # input fiducial list selector
    #

    fiducialWarningLabel = qt.QLabel( "Note: Parent transforms of fiducials are not used. Fiducials should be defined in the coordinate system that is being registered." )
    fiducialWarningLabel.setWordWrap( True )
    registrationFormLayout.addRow(fiducialWarningLabel)

    self.inputFiducialSelector = slicer.qMRMLNodeComboBox()
    self.inputFiducialSelector.nodeTypes = ( ("vtkMRMLMarkupsFiducialNode"), "" )
    self.inputFiducialSelector.selectNodeUponCreation = False
    self.inputFiducialSelector.addEnabled = False
    self.inputFiducialSelector.removeEnabled = False
    self.inputFiducialSelector.noneEnabled = True
    self.inputFiducialSelector.showHidden = False
    self.inputFiducialSelector.showChildNodeTypes = False
    self.inputFiducialSelector.setMRMLScene( slicer.mrmlScene )
    self.inputFiducialSelector.setToolTip( "Pick the input fiducial list for the algorithm." )
    registrationFormLayout.addRow("Input fiducials: ", self.inputFiducialSelector)

    #
    # input volume selector
    #
    self.inputVolumeSelector = slicer.qMRMLNodeComboBox()
    self.inputVolumeSelector.nodeTypes = (("vtkMRMLScalarVolumeNode"), "")
    self.inputVolumeSelector.selectNodeUponCreation = False
    self.inputVolumeSelector.addEnabled = False
    self.inputVolumeSelector.removeEnabled = False
    self.inputVolumeSelector.noneEnabled = True
    self.inputVolumeSelector.showHidden = False
    self.inputVolumeSelector.showChildNodeTypes = False
    self.inputVolumeSelector.setMRMLScene(slicer.mrmlScene)
    self.inputVolumeSelector.setToolTip("Pick the input model for the algorithm.")
    registrationFormLayout.addRow("Input model: ", self.inputVolumeSelector)
    #
    # Register Button
    #
    self.applyRegButton = qt.QPushButton("Apply")
    self.applyRegButton.toolTip = "Run the algorithm."
    self.applyRegButton.enabled = False
    registrationFormLayout.addRow(self.applyRegButton)

###############################################################################
    #Atlas Registration Area
    atlasRegistrationCollapsibleButton = ctk.ctkCollapsibleButton()
    atlasRegistrationCollapsibleButton.text = "Register Atlas to Frame"
    atlasRegistrationCollapsibleButton.setDisabled(False)
    self.layout.addWidget(atlasRegistrationCollapsibleButton)
    
    # Layout within the dummy collapsible button
    atlasregistrationFormLayout = qt.QFormLayout(atlasRegistrationCollapsibleButton)
    
    self.AtlasFids=slicer.qMRMLNodeComboBox()
    self.AtlasFids.nodeTypes=["vtkMRMLMarkupsFiducialNode"]
    self.AtlasFids.selectNodeUponCreation = True
    self.AtlasFids.addEnabled = True
    self.AtlasFids.removeEnabled = True
    self.AtlasFids.noneEnabled = True
    self.AtlasFids.showHidden = True
    self.AtlasFids.showChildNodeTypes = True
    self.AtlasFids.setMRMLScene( slicer.mrmlScene )
    self.AtlasFids.setToolTip( "Pick the input to the algorithm." )    
    
    atlasregistrationFormLayout.addRow("Atlas AC/PC/Sup1/Sup2 Fiducials",self.AtlasFids)
    
    self.ScanFids=slicer.qMRMLNodeComboBox()
    self.ScanFids.nodeTypes=["vtkMRMLMarkupsFiducialNode"]
    self.ScanFids.selectNodeUponCreation = True
    self.ScanFids.addEnabled = True
    self.ScanFids.removeEnabled = True
    self.ScanFids.noneEnabled = True
    self.ScanFids.showHidden = True
    self.ScanFids.showChildNodeTypes = True
    self.ScanFids.setMRMLScene( slicer.mrmlScene )
    self.ScanFids.setToolTip( "Pick the input to the algorithm." )
    
    atlasregistrationFormLayout.addRow("Scan AC/PC/Sup1/Sup2 Fiducials",self.ScanFids)
    
    #
    # Register Button
    #
    self.AtlasRegisterButton = qt.QPushButton("Register Atlas to Frame")
    self.AtlasRegisterButton.toolTip = "Run the algorithm."
    self.AtlasRegisterButton.enabled = True
    atlasregistrationFormLayout.addRow(self.AtlasRegisterButton)
    

#####################################
    #
    # Target and Trajectory Area
    #
    parametersCollapsibleButton = ctk.ctkCollapsibleButton()
    parametersCollapsibleButton.text = "Target and Trajectory"
    self.layout.addWidget(parametersCollapsibleButton)

    # Layout within the dummy collapsible button
    parametersFormLayout = qt.QFormLayout(parametersCollapsibleButton)

    #
    # input model selector
    #
    self.inputSelector = slicer.qMRMLNodeComboBox()
    self.inputSelector.nodeTypes = ["vtkMRMLModelNode"]
    self.inputSelector.selectNodeUponCreation = False
    self.inputSelector.addEnabled = False
    self.inputSelector.removeEnabled = False
    self.inputSelector.noneEnabled = False
    self.inputSelector.showHidden = False
    self.inputSelector.showChildNodeTypes = False
    self.inputSelector.setMRMLScene( slicer.mrmlScene )
    self.inputSelector.setToolTip( "Pick the Electrode to Target" )
    parametersFormLayout.addRow("Target Electrode: ", self.inputSelector)
    
    #Dictionary for inverse matrices
    self.ElectrInv={}

    #
    # Arc Angle value
    #
    self.arcAngleWidget = ctk.ctkSliderWidget()
    self.arcAngleWidget.singleStep = 0.1
    self.arcAngleWidget.minimum = 0
    self.arcAngleWidget.maximum = 180
    self.arcAngleWidget.value = 90
    self.arcAngleWidget.setToolTip("Set Arc angle of approach.")
    parametersFormLayout.addRow("Arc", self.arcAngleWidget)
    
    #
    # Collar Angle value
    #
    self.collarAngleWidget = ctk.ctkSliderWidget()
    self.collarAngleWidget.singleStep = 0.1
    self.collarAngleWidget.minimum = 0
    self.collarAngleWidget.maximum = 180
    self.collarAngleWidget.value = 90
    self.collarAngleWidget.setToolTip("Set Collar angle of approach.")
    parametersFormLayout.addRow("Collar", self.collarAngleWidget)    
    

    #
    # X-Origin
    #
    self.xOriginWidget = ctk.ctkDoubleSpinBox()
    self.xOriginWidget.singleStep = 0.1
    self.xOriginWidget.minimum = 50
    self.xOriginWidget.maximum = 150
    self.xOriginWidget.value = 100
    self.xOriginWidget.setToolTip("Set threshold value for computing the output image. Voxels that have intensities lower than this value will set to zero.")
    parametersFormLayout.addRow("X0", self.xOriginWidget)    

    #
    # Y-Origin
    #
    self.yOriginWidget = ctk.ctkDoubleSpinBox()
    self.yOriginWidget.singleStep = 0.1
    self.yOriginWidget.minimum = 50
    self.yOriginWidget.maximum = 150
    self.yOriginWidget.value = 100
    self.yOriginWidget.setToolTip("Set threshold value for computing the output image. Voxels that have intensities lower than this value will set to zero.")
    parametersFormLayout.addRow("Y0", self.yOriginWidget)    

    #
    # Z-Origin
    #
    self.zOriginWidget = ctk.ctkDoubleSpinBox()
    self.zOriginWidget.singleStep = 0.1
    self.zOriginWidget.minimum = 50
    self.zOriginWidget.maximum = 150
    self.zOriginWidget.value = 100
    self.zOriginWidget.setToolTip("Set threshold value for computing the output image. Voxels that have intensities lower than this value will set to zero.")
    parametersFormLayout.addRow("Z0", self.zOriginWidget)    
    
    #
    # Apply Button
    #
    self.applyButton = qt.QPushButton("New Electrode")
    self.applyButton.toolTip = "Generate New Electrode"
    self.applyButton.enabled = False
    parametersFormLayout.addRow(self.applyButton)
########################
    #Procedure connections
    #Image to Frame Registration
    self.applyRegButton.connect('clicked(bool)', self.onApplyRegButton)
    self.inputVolumeSelector.connect("currentNodeChanged(vtkMRMLNode*)",self.onRegSelect)
    self.inputFiducialSelector.connect("currentNodeChanged(vtkMRMLNode*)", self.onRegSelect)

    #Atlas to Frame Registration
    self.AtlasRegisterButton.connect('clicked(bool)', self.RegisterACPC)

    #Target and Trajectory
    self.applyButton.connect('clicked(bool)', self.newElectrode)
    self.inputSelector.connect("currentNodeChanged(vtkMRMLNode*)", self.onSelect)

    self.arcAngleWidget.connect("valueChanged(double)", self.onChanges)
    self.collarAngleWidget.connect("valueChanged(double)", self.onChanges)
   
    self.xOriginWidget.connect("valueChanged(double)", self.onChanges)
    self.yOriginWidget.connect("valueChanged(double)", self.onChanges)
    self.zOriginWidget.connect("valueChanged(double)", self.onChanges)
    
#     # Set the original Electrode
#     self.currElectrode=slicer.util.getNode('Model')
#     TRX=slicer.vtkMRMLLinearTransformNode()
#     TRX.SetName('Transform_' + self.currElectrode.GetName())
#     TRX.SetHideFromEditors(False)
#     slicer.mrmlScene.AddNode(TRX)
#     self.inputSelector.setCurrentNodeID(self.currElectrode.GetID())
#     self.saveElectrodeProperties(self.currElectrode)

    

    # Refresh Apply button state
    self.onSelect()

    
    # Add vertical spacer
    self.layout.addStretch(1)
    
    #Create a Default Electrode
    self.newElectrode()
    
    #Connect the movement of the x,y,z target to the movement of the
    #"Targets" Fiducial Node
    #Other events for FiducialNode listed in (https://www.slicer.org/doc/html/classvtkMRMLMarkupsNode.html)
    #ToDo: Have a "target" fiducial for each electrode that can be placed independently
    self.targs=slicer.util.getNode('Targets')
    self.targs.RemoveAllObservers()
    self.targs.AddObserver(self.targs.PointEndInteractionEvent,self.pointMoved)
    
    #path=slicer.util.getModule('PigFrame').path
    #print path
    ##path=path[:path.rfind('/')]+'Resources/Models/NewPigFrameRed.mrml'
    #print path
    #slicer.util.loadScene(path)


  def cleanup(self):
    pass

  def onRegSelect(self):
    self.applyRegButton.enabled = self.inputVolumeSelector.currentNode() and self.inputFiducialSelector.currentNode()

  def onApplyRegButton(self):
    #This is where we ensure appropriate requirements of the registration technique
    logic = FiducialsToModelRegistrationLogic()

    inputFiducials = self.inputFiducialSelector.currentNode()
    inputModel = slicer.util.getNode('Frame_Template_wf')
    inputVolume = self.inputVolumeSelector.currentNode()

    transID=inputVolume.GetTransformNodeID()

    if transID==inputFiducials.GetTransformNodeID() and transID != None:
        #they both have the same transform
        outputTransform = slicer.util.getNode(transID)
    elif transID==None:
        #Neither of them have a transform
        outputTransform=slicer.mrmlScene.AddNewNodeByClass('vtkMRMLLinearTransformNode','transRegToFrame')
        inputFiducials.SetAndObserveTransformNodeID(outputTransform.GetID())
        inputVolume.SetAndObserveTransformNodeID(outputTransform.GetID())
    else:
        #Harden Transform, Make it identity, put Volume and Fiducials in
        outputTransform=slicer.util.GetNode(transID)
        inputVolume.HardenTransform()
        inputFiducials.HardenTransform()
        mat=vtk.vtkMatrix4x4()
        mat.Identity()
        outputTransform.SetMatrixTransformToParent(mat)
        inputFiducials.SetAndObserveTransformNodeID(outputTransform.GetID())
        inputVolume.SetAndObserverTransformNodeID(outputTransform.GetID())

    logic.run(inputFiducials, inputModel, outputTransform, 0, 100) #Rigid, 100 iterations

    #self.outputLine.setText( logic.ComputeMeanDistance(inputFiducials, inputModel, outputTransform) )


  def RegisterACPC(self):
    Pts1=self.AtlasFids.currentNode()
    Pts2=self.ScanFids.currentNode()
    TRX=slicer.util.getNode('Atlas_Trans')
    slicer.util.getNode()

    #Pts1 = slicer.util.getNode(AtlasID)
    #Pts2 = slicer.util.getNode(ScanID)

    pts1 = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])

    Pts1.GetNthFiducialPosition(0, pts1[0])
    Pts1.GetNthFiducialPosition(1, pts1[1])

    pts2 = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])

    Pts2.GetNthFiducialPosition(0, pts2[0])
    Pts2.GetNthFiducialPosition(1, pts2[1])

    # Get Vectors of each position
    Vec1 = pts1[1] - pts1[0]
    Vec2 = pts2[1] - pts2[0]
    # Cross product to define perpendicular vector to rotate around
    Vec3 = np.cross(Vec1, Vec2)
    Sign = 1  # What is this going to be? Do I need this?
    scale = np.linalg.norm(Vec2) / np.linalg.norm(Vec1)
    # The angle to rotate around
    theta = Sign * np.arccos(np.dot(Vec1, Vec2) / (np.linalg.norm(Vec1) * np.linalg.norm(Vec2))) * 360.0 / (2 * np.pi)

    transScale = vtk.vtkTransform()
    transScale.PostMultiply()
    transScale.Translate(-pts1[0, 0], -pts1[0, 1], -pts1[0, 2])  # Translate the points to origin
    transScale.Scale(scale, scale, scale)  # Scale
    transScale.RotateWXYZ(theta, Vec3)  # Rotate system around perpendicular vector
    transScale.Translate(pts2[0, 0], pts2[0, 1], pts2[0, 2])  # Translate to second point set
    TRX.SetAndObserveMatrixTransformToParent(transScale.GetMatrix())

  def pointMoved(self,caller, event):
    #Moves the currently selected Electrode with respects to the position of first Fiducial in the list
    #ToDo: Use a fiducial for each electrode.
    self.PointMoving=True
    movingPointIDX=int(caller.GetAttribute('Markups.MovingMarkupIndex'))
    movingPointIDX=int(movingPointIDX)
    caller.SetNthFiducialVisibility(movingPointIDX,False)
    coord = [0.0,0.0,0.0]
    caller.GetNthFiducialPosition(movingPointIDX,coord)
    self.updateCoordsFromFids(movingPointIDX)
    caller.SetNthFiducialVisibility(movingPointIDX,True)
    slicer.modules.markups.logic().JumpSlicesToLocation(coord[0],coord[1],coord[2],True)
    self.PointMoving = False

  def updateCoordsFromFids(self,movingPointIDX):
    Entry = [0.0, 0.0, 0.0]
    Target = [0.0, 0.0, 0.0]
    self.targs.GetNthFiducialPosition(1, Entry)
    self.targs.GetNthFiducialPosition(0, Target)
    [arc, collar] = self.Targets_to_Frame(Target, Entry)

    self.xOriginWidget.setValue(-1*Target[0])
    self.yOriginWidget.setValue(Target[1])
    self.zOriginWidget.setValue(-1*Target[2])

    self.arcAngleWidget.setValue(arc) 
    self.collarAngleWidget.setValue(collar)

  def Targets_to_Frame(self, Xt, Xe):
    ################################################
    # Purpose: To convert a formal description of a cylinder, the target point, and the point of entry into coordinates for a cylindrically shaped stereotactic head frame.
    # Inputs:
    #      Xt: the target point
    #      Xe: point of entry into the patient
    # Outputs:
    #     arc: The arc angle neccessary for a trajectory to go through point Xt and Xe
    #  collar: The collar angle neccessary for a trajectory to go through point Xt and Xe
    ################################################

    # unit vector from target to entry point
    Xt = np.array(Xt)
    Xe = np.array(Xe)
    Xr = np.array(Xe-Xt)
    dist = np.linalg.norm(Xr) #The distance between the points.
    phi=np.array([0.0,0.0])#arc and collar

    phi[0]=np.arccos(Xr[0]/dist)

    if Xr[1]!=0:
        phi[1]=np.arctan(Xr[2]/Xr[1])
    else:
        phi[1]=np.pi/2.0
    if phi[1]<0:
        phi[1]=np.pi+phi[1]


    return [math.degrees(phi[0]),math.degrees(phi[1])]

    
  def onSelect(self):

    self.applyButton.enabled = self.inputSelector.currentNode()
    #Get Current Electrode
    selElectrode=self.inputSelector.currentNode()
    if (selElectrode != self.currElectrode)&(selElectrode!=None)&(self.currElectrode!=None):
        #Switching Electrodes set to true so not to prematurely update Electrode Properties.
        #Yes, this could be applied more "intelligently"
        self.switchingElectrodes=True
        #Harden transform 
        #logic=slicer.vtkSlicerTransformLogic()
        #logic.hardenTransform(self.currElectrode)
        TRX=slicer.util.getNode('Transform_'+self.currElectrode.GetName())
        self.currElectrode.SetAndObserveTransformNodeID(TRX.GetID())
        self.currElectrode=selElectrode
        #selElectrode.ApplyTransformMatrix(self.ElectrInv[selElectrode.GetName()])
        x0=float(selElectrode.GetAttribute('X'))
        y0=float(selElectrode.GetAttribute('Y'))
        z0=float(selElectrode.GetAttribute('Z'))
        self.xOriginWidget.setValue(x0)
        self.yOriginWidget.setValue(y0)
        self.zOriginWidget.setValue(z0)
        self.targs.SetNthFiducialPosition(0,-1*x0,y0,-1*z0)
        self.arcAngleWidget.setValue(float(selElectrode.GetAttribute('Arc')))
        self.collarAngleWidget.setValue(float(selElectrode.GetAttribute('Collar')))
        selElectrode.SetAndObserveTransformNodeID(slicer.util.getNode('Arc').GetID())
        self.pointMoved(self.targs, self.targs.PointEndInteractionEvent)
        self.onApplyButton()
        self.switchingElectrodes=False
        self.saveElectrodeProperties(self.currElectrode)


  def onChanges(self):
    self.currElectrode=self.inputSelector.currentNode()
    self.saveElectrodeProperties(self.currElectrode)
      
    if self.applyButton.enabled:
        self.onApplyButton()
        
  def onApplyButton(self):
    logic = PigFrameLogic()
    arcAngle = -1*(self.arcAngleWidget.value-90)
    collarAngle = self.collarAngleWidget.value-90
    x0=self.xOriginWidget.value
    y0=self.yOriginWidget.value
    z0=self.zOriginWidget.value
    
    
    #The Frame is set such that Frame coordinate 100,100,100 is set to the origin of the Slicer Space (0,0,0)
    #Future iterations need to align Frame Coordinate to Slicer Space and adjust the appropriate Frame elements accordingly.
    #Set X Translation
    TRX_Target_X=slicer.util.getNode('Target_X')
    trans=vtk.vtkTransform()
    trans.Translate(-1*(x0),0,0) #y0,-1*z0)
    TRX_Target_X.SetMatrixTransformToParent(trans.GetMatrix())

    #Set Y Translation
    TRX_Target_X=slicer.util.getNode('Target_Y')
    trans=vtk.vtkTransform()
    trans.Translate(0,y0,0) #y0,-1*z0)
    TRX_Target_X.SetMatrixTransformToParent(trans.GetMatrix())

    #Set Z Translation
    TRX_Target_X=slicer.util.getNode('Target_Z')
    trans=vtk.vtkTransform()
    trans.Translate(0,0,-1*(z0)) #y0,-1*z0)
    TRX_Target_X.SetMatrixTransformToParent(trans.GetMatrix())

    TRX_Collar=slicer.util.getNode('Collar')
    trans.Identity()
    trans.RotateX(collarAngle)
    #print trans
    TRX_Collar.SetMatrixTransformToParent(trans.GetMatrix())

    TRX_Arc=slicer.util.getNode('Arc')
    trans.Identity()
    trans.RotateY(arcAngle)
    #print trans
    TRX_Arc.SetMatrixTransformToParent(trans.GetMatrix())

    self.targs.SetNthFiducialPosition(0,-1*x0,y0,-1*z0)
    Entry = np.array([0.0, 0.0, 0.0])
    Target = np.array([0.0, 0.0, 0.0])

    self.targs.GetNthFiducialPosition(1, Entry)
    self.targs.GetNthFiducialPosition(0, Target)
    if not self.PointMoving:
        Diff = Entry - Target
        dist=np.linalg.norm(Diff)

        Diff[0]=dist*np.cos(math.radians(90+arcAngle))
        distP=dist*np.sin(math.radians(90+arcAngle))

        Diff[1]=distP*np.cos(math.radians(90+collarAngle))
        Diff[2] = distP * np.sin(math.radians(90+collarAngle))

        Entry=Target+Diff*np.array([-1,1,1])
        self.targs.SetNthFiducialPosition(1,Entry[0],Entry[1],Entry[2])
        for SliceNode in self.SliceNodes:
            SliceNode.SetActiveSlice(1)
            SliceNode.SetActiveSlice(0)



  def saveElectrodeProperties(self,electrode):
    if not self.switchingElectrodes:
          #Save its full transform
        TRX=slicer.vtkMRMLLinearTransformNode()
        trans=vtk.vtkMatrix4x4()
        #inv=vtk.vtkMatrix4x4()
        #for trName in ['Arc','Target_X','Collar','Target_Z','Target_Y']:
        #    TRX.GetMatrixTransformFromNode(slicer.util.getNode(trName),trans)
        #    inv.Multiply4x4(inv,trans,inv)
        #create an inverse
        TRX.GetMatrixTransformFromNode(slicer.util.getNode('Arc'),trans)
        TRX=slicer.util.getNode('Transform_'+electrode.GetName())
        TRX.SetMatrixTransformToParent(trans)
        #inv.Invert()
        #Store it in a dictionary for retrieval
        #self.ElectrInv[electrode.GetName()]=inv
        #save specific values
        electrode.SetAttribute('X',str(self.xOriginWidget.value))
        electrode.SetAttribute('Y',str(self.yOriginWidget.value))
        electrode.SetAttribute('Z',str(self.zOriginWidget.value))
        electrode.SetAttribute('Arc',str(self.arcAngleWidget.value))
        electrode.SetAttribute('Collar',str(self.collarAngleWidget.value))
    
    
    #Harden transform 
    #logic=slicer.vtkSlicerTransformLogic()
    #logic.hardenTransform(self.currElectrode)    
    
  def newElectrode(self):
    if not self.currElectrode==None:
        #Get Current Electrode
        self.currElectrode=self.inputSelector.currentNode()
        self.saveElectrodeProperties(self.currElectrode)
    
        #Harden transform 
        #logic=slicer.vtkSlicerTransformLogic()
        #logic.hardenTransform(self.currElectrode)
        TRX=slicer.util.getNode('Transform_'+self.currElectrode.GetName())
        self.currElectrode.SetAndObserveTransformNodeID(TRX.GetID())
    
    #Set up new electrode  
    Origin=[0,0,0]
    Terminus=[0,0,200]

    #This section takes care of the "Electrode"
    line = vtk.vtkLineSource()
    line.SetPoint1(Origin)
    line.SetPoint2(Terminus)
    line.Update()
    
    tube_filter = vtk.vtkTubeFilter()
    tube_filter.SetCapping(True)
    tube_filter.SetRadius(1.1)
    tube_filter.SetNumberOfSides(20)
    
    tube_filter.SetInputDataObject(line.GetOutput())
    tube_filter.Update()
    
    electrode=tube_filter.GetOutput() 
    
    newElectrode =slicer.vtkMRMLModelNode()    
    newElectrode.SetAndObservePolyData(electrode)
    
    slicer.mrmlScene.AddNode(newElectrode)
    
    EDisplayNode=slicer.vtkMRMLModelDisplayNode()
    slicer.mrmlScene.AddNode(EDisplayNode)
    
    EDisplayNode.SetName( "NeedleModelDisplay" );
    EDisplayNode.SetColor( 0.0, 1.0, 1.0 );
      
    newElectrode.SetAndObserveDisplayNodeID( EDisplayNode.GetID() );
    EDisplayNode.SetAmbient( 0.2 );
    EDisplayNode.SetSliceIntersectionVisibility(True)
    
    newElectrode.SetAndObserveTransformNodeID(slicer.util.getNode('Arc').GetID())
    self.currElectrode=newElectrode
    TRX=slicer.vtkMRMLLinearTransformNode()
    TRX.SetName('Transform_' + self.currElectrode.GetName())
    TRX.SetHideFromEditors(True)
    slicer.mrmlScene.AddNode(TRX)
    self.saveElectrodeProperties(newElectrode)
    self.inputSelector.setCurrentNodeID(newElectrode.GetID())
    
    
    
    #The "Logic" has yet to be constructed...along with the unit tests for the PigFrame
    
    #logic.run(self.inputSelector.currentNode(), arcAngle,collarAngle)
#
# PigFrameLogic
#

class PigFrameLogic(ScriptedLoadableModuleLogic):
  """This class should implement all the actual
  computation done by your module.  The interface
  should be such that other python code can import
  this class and make use of the functionality without
  requiring an instance of the Widget.
  Uses ScriptedLoadableModuleLogic base class, available at:
  https://github.com/Slicer/Slicer/blob/master/Base/Python/slicer/ScriptedLoadableModule.py
  """

  def hasImageData(self,volumeNode):
    """This is an example logic method that
    returns true if the passed in volume
    node has valid image data
    """
    if not volumeNode:
      logging.debug('hasImageData failed: no volume node')
      return False
    if volumeNode.GetImageData() is None:
      logging.debug('hasImageData failed: no image data in volume node')
      return False
    return True

  def isValidInputOutputData(self, inputVolumeNode, outputVolumeNode):
    """Validates if the output is not the same as input
    """
    if not inputVolumeNode:
      logging.debug('isValidInputOutputData failed: no input volume node defined')
      return False
    if not outputVolumeNode:
      logging.debug('isValidInputOutputData failed: no output volume node defined')
      return False
    if inputVolumeNode.GetID()==outputVolumeNode.GetID():
      logging.debug('isValidInputOutputData failed: input and output volume is the same. Create a new volume for output to avoid this error.')
      return False
    return True

  def takeScreenshot(self,name,description,type=-1):
    # show the message even if not taking a screen shot
    slicer.util.delayDisplay('Take screenshot: '+description+'.\nResult is available in the Annotations module.', 3000)

    lm = slicer.app.layoutManager()
    # switch on the type to get the requested window
    widget = 0
    if type == slicer.qMRMLScreenShotDialog.FullLayout:
      # full layout
      widget = lm.viewport()
    elif type == slicer.qMRMLScreenShotDialog.ThreeD:
      # just the 3D window
      widget = lm.threeDWidget(0).threeDView()
    elif type == slicer.qMRMLScreenShotDialog.Red:
      # red slice window
      widget = lm.sliceWidget("Red")
    elif type == slicer.qMRMLScreenShotDialog.Yellow:
      # yellow slice window
      widget = lm.sliceWidget("Yellow")
    elif type == slicer.qMRMLScreenShotDialog.Green:
      # green slice window
      widget = lm.sliceWidget("Green")
    else:
      # default to using the full window
      widget = slicer.util.mainWindow()
      # reset the type so that the node is set correctly
      type = slicer.qMRMLScreenShotDialog.FullLayout

    # grab and convert to vtk image data
    qpixMap = qt.QPixmap().grabWidget(widget)
    qimage = qpixMap.toImage()
    imageData = vtk.vtkImageData()
    slicer.qMRMLUtils().qImageToVtkImageData(qimage,imageData)

    annotationLogic = slicer.modules.annotations.logic()
    annotationLogic.CreateSnapShot(name, description, type, 1, imageData)

  def run(self, inputVolume, outputVolume, imageThreshold, enableScreenshots=0):
    """
    Run the actual algorithm
    """

    if not self.isValidInputOutputData(inputVolume, outputVolume):
      slicer.util.errorDisplay('Input volume is the same as output volume. Choose a different output volume.')
      return False

    logging.info('Processing started')

    # Compute the thresholded output volume using the Threshold Scalar Volume CLI module
    cliParams = {'InputVolume': inputVolume.GetID(), 'OutputVolume': outputVolume.GetID(), 'ThresholdValue' : imageThreshold, 'ThresholdType' : 'Above'}
    cliNode = slicer.cli.run(slicer.modules.thresholdscalarvolume, None, cliParams, wait_for_completion=True)

    # Capture screenshot
    if enableScreenshots:
      self.takeScreenshot('PigFrameTest-Start','MyScreenshot',-1)

    logging.info('Processing completed')

    return True


class PigFrameTest(ScriptedLoadableModuleTest):
  """
  This is the test case for your scripted module.
  Uses ScriptedLoadableModuleTest base class, available at:
  https://github.com/Slicer/Slicer/blob/master/Base/Python/slicer/ScriptedLoadableModule.py
  """

  def setUp(self):
    """ Do whatever is needed to reset the state - typically a scene clear will be enough.
    """
    slicer.mrmlScene.Clear(0)

  def runTest(self):
    """Run as few or as many tests as needed here.
    """
    self.setUp()
    self.test_PigFrame1()

  def test_PigFrame1(self):
    """ Ideally you should have several levels of tests.  At the lowest level
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
        ('http://slicer.kitware.com/midas3/download?items=5767', 'FA.nrrd', slicer.util.loadVolume),
        )

    for url,name,loader in downloads:
      filePath = slicer.app.temporaryPath + '/' + name
      if not os.path.exists(filePath) or os.stat(filePath).st_size == 0:
        logging.info('Requesting download %s from %s...\n' % (name, url))
        urllib.urlretrieve(url, filePath)
      if loader:
        logging.info('Loading %s...' % (name,))
        loader(filePath)
    self.delayDisplay('Finished with download and loading')

    volumeNode = slicer.util.getNode(pattern="FA")
    logic = PigFrameLogic()
    self.assertIsNotNone( logic.hasImageData(volumeNode) )
    self.delayDisplay('Test passed!')
