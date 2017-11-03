import vtk
import numpy as np
import meshOperations
import renderer
import math
import decimator
import raycasting
import vtkpointcloud
import smoothing

# Create polydata to slice the grid with. In this case, use a cone. This could
# be any polydata including a stl file.
reader = vtk.vtkOBJReader()

reader.SetFileName("upperJawMesh.obj")

reader.Update()

reader2 = vtk.vtkOBJReader()

reader2.SetFileName("suggest_alveolarRidgeLine_upper_finalVisualization.obj")

reader2.Update()



polydata = reader.GetOutput()

#Preprocessing : rotate data and find center of gravity
meshOp = meshOperations.MeshOperations(poly_data=polydata)
PCADict = meshOp.computePCA()
polydata = meshOp.rotate([1.0,0.0,0.0],PCADict['eigenvectors'][0])

meshOp.changePolyData(polydata)

PCADict2 = meshOp.computePCA()
polydata = meshOp.rotate([0.0,1.0,0.0],PCADict2['eigenvectors'][1])

translation = [-polydata.GetBounds()[0],-polydata.GetBounds()[2],-polydata.GetBounds()[4]]
meshOp.changePolyData(polydata)
polydata = meshOp.translate(translation[0],translation[1],translation[2])


suggest = reader2.GetOutput() 

meshOp2 = meshOperations.MeshOperations(poly_data=suggest)
suggest = meshOp2.rotate([1.0,0.0,0.0],PCADict['eigenvectors'][0])

meshOp2.changePolyData(suggest)
suggest = meshOp2.rotate([0.0,1.0,0.0],PCADict2['eigenvectors'][1])

meshOp2.changePolyData(suggest)
suggest = meshOp2.translate(translation[0],translation[1],translation[2])

# meshOp.changePolyData(polydata)
# gravity = meshOp.computeCenterOfMass()

bounds = polydata.GetBounds()

# meshOp.changePolyData(polydata)
# gravity = meshOp.computeCenterOfMass()





#Open found points
reader3 = vtk.vtkPolyDataReader()
reader3.SetFileName("./FirstTestUpper.vtk")
reader3.Update()

firstTest = reader3.GetOutput()





# Smooth and weight smoothed with firstTest, then smooth again and find points on mesh
smth = smoothing.Smoothing(poly_data=firstTest)
firstTest2 = smth.weightedCombination(500,0.1) #smoothing + weighting smoothing with firstTest
smth.changePolyData(firstTest2)
firstTest2 = smth.polyDataSmoothed(50,0.1) #smooth again (but less) => points are NOT on the mesh but have right curve
rayC = raycasting.RayCasting(poly_data=polydata) #find points on mesh
sommets = rayC.findPoints2(firstTest2,True,0.001)

pointCloud = vtkpointcloud.VtkPointCloud(zMin=bounds[4],zMax=bounds[5])
for i in range (0,sommets.GetNumberOfPoints()):
    pointCloud.addPoint(sommets.GetPoint(i))  

#Visualization stuff
inputMapper = vtk.vtkDataSetMapper()
planeMapper = vtk.vtkDataSetMapper()
if vtk.VTK_MAJOR_VERSION <= 5:
    inputMapper.SetInput(polydata)
else:
    inputMapper.SetInputData(polydata)
 
imgMapper = vtk.vtkDataSetMapper()
imgMapper2 = vtk.vtkDataSetMapper()

if vtk.VTK_MAJOR_VERSION <= 5:
    imgMapper.SetInput(pointCloud.vtkPolyData)
    imgMapper2.SetInput(suggest)
else:
    imgMapper.SetInputData(pointCloud.vtkPolyData)
    imgMapper2.SetInputData(suggest)

inputActor = vtk.vtkActor()
inputActor.SetMapper(inputMapper)
selectedActor = vtk.vtkActor()
selectedActor.SetMapper(imgMapper)
selectedActor2 = vtk.vtkActor()
selectedActor2.SetMapper(imgMapper2)
selectedActor2.GetProperty().SetColor(1.0,0.0,0.0)
inputActor.GetProperty().SetOpacity(0.2)


# There will be one render window
renderWindow = vtk.vtkRenderWindow()
renderWindow.SetSize(900, 300)

# # And one interactor
interactor = vtk.vtkRenderWindowInteractor()
interactor.SetRenderWindow(renderWindow)

# Define viewport ranges
# (xmin, ymin, xmax, ymax)
leftViewport = [0.0, 0.0, 0.5, 1.0]
centerViewport = [0.5, 0.0, 1.0, 1.0]

# Setup the renderers
leftRenderer = vtk.vtkRenderer()
renderWindow.AddRenderer(leftRenderer)
leftRenderer.SetViewport(leftViewport)
leftRenderer.SetBackground(.6, .5, .4)


centerRenderer = vtk.vtkRenderer()
renderWindow.AddRenderer(centerRenderer)
centerRenderer.SetViewport(centerViewport)
centerRenderer.SetBackground(.8, .8, .8)

leftRenderer.AddActor(inputActor)
leftRenderer.AddActor(selectedActor)
leftRenderer.AddActor(selectedActor2)
centerRenderer.AddActor(inputActor)
centerRenderer.AddActor(selectedActor2)

leftRenderer.ResetCamera()

renderWindow.Render()
interactor.Start()