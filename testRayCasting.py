import vtk
import numpy as np
import meshOperations
import renderer
import math
import decimator
import raycasting
import levelset
import vtkpointcloud

# Create polydata to slice the grid with. In this case, use a cone. This could
# be any polydata including a stl file.
reader = vtk.vtkOBJReader()

reader.SetFileName("lowerJawMesh.obj")

reader.Update()

polydata = reader.GetOutput()

#Preprocessing : rotate data and find center of gravity
meshOp = meshOperations.MeshOperations(poly_data=polydata)
PCADict = meshOp.computePCA()
reorientedPolyData = meshOp.rotate([1.0,0.0,0.0],PCADict['eigenvectors'][0])
meshOp2 = meshOperations.MeshOperations(poly_data=reorientedPolyData)
PCADict2 = meshOp2.computePCA()
reorientedPolyData = meshOp2.rotate([0.0,1.0,0.0],PCADict2['eigenvectors'][1])
print reorientedPolyData.GetBounds()
meshOps = meshOperations.MeshOperations(poly_data=reorientedPolyData)
polydata = meshOps.translate(-reorientedPolyData.GetBounds()[0],-reorientedPolyData.GetBounds()[2],-reorientedPolyData.GetBounds()[4])

meshOps = meshOperations.MeshOperations(poly_data=polydata)
gravity = meshOps.computeCenterOfMass()

bounds = polydata.GetBounds()
polydata = meshOps.cropMesh(bounds[0],bounds[1],bounds[2],bounds[3],bounds[4],gravity[2])

meshOps.changePolyData(polydata)
gravity = meshOps.computeCenterOfMass()


#Ray casting part
rayC = raycasting.RayCasting(poly_data=polydata)
center = [gravity[0],gravity[1],bounds[5]]
points = rayC.findPoints(center,False)

#Add found points into a pointcloud => vtkpolydata
pointCloud = vtkpointcloud.VtkPointCloud(zMin=bounds[4],zMax=bounds[5])
for i in range (0,points.GetNumberOfPoints()):
	pointCloud.addPoint(points.GetPoint(i))


#Visualization stuff
inputMapper = vtk.vtkDataSetMapper()
planeMapper = vtk.vtkDataSetMapper()
if vtk.VTK_MAJOR_VERSION <= 5:
    inputMapper.SetInput(polydata)
else:
    inputMapper.SetInputData(polydata)
 
imgMapper = vtk.vtkDataSetMapper()

if vtk.VTK_MAJOR_VERSION <= 5:
    imgMapper.SetInput(pointCloud.vtkPolyData)
else:
    imgMapper.SetInputData(pointCloud.vtkPolyData)

inputActor = vtk.vtkActor()
inputActor.SetMapper(inputMapper)
selectedActor = vtk.vtkActor()
selectedActor.SetMapper(imgMapper)
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
centerRenderer.AddActor(selectedActor)

leftRenderer.ResetCamera()

renderWindow.Render()
interactor.Start()