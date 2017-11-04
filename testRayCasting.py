import vtk
import meshOperations
import raycasting



##############################################################
################## Part 1 : Preprocessing ####################
##############################################################

### Load data, find principal components and reorient the data according to these principal components axes
### The first component will be the x axis and corresponds to the left-right axis
### The second component will be the y axis and corresponds to the anteroposterior axis
### The last component will be the z axis and corresponds to the craniocauda axis
### Then crop the mesh to get rid of the tongue/palatin, to ease the computation

# Load data
meshOp = meshOperations.MeshOperations()
polydata = meshOp.read("lowerJawMesh.obj")

# Reorient data
PCADict = meshOp.compute_pca(polydata)
reorientedPolyData = meshOp.rotate(polydata,[0.0,1.0,0.0],PCADict['eigenvectors'][1])
PCADict2 = meshOp.compute_pca(reorientedPolyData)
reorientedPolyData = meshOp.rotate(reorientedPolyData,[0.0,0.0,1.0],PCADict2['eigenvectors'][2])
bounds = reorientedPolyData.GetBounds()
polydata = meshOp.translate(reorientedPolyData,-bounds[0],-bounds[2],-bounds[4])

gravity = meshOp.compute_center_of_mass(polydata)

# Crop upper part (the center of gravity should be under the teeth hopefully)
bounds = polydata.GetBounds()
polydata = meshOp.crop_mesh(polydata,bounds[0], bounds[1], bounds[2], bounds[3], gravity[2], bounds[5])

polydata = meshOp.translate_to_origin(polydata)

# Compute center of gravity
gravity = meshOp.compute_center_of_mass(polydata)
bounds = polydata.GetBounds()




##############################################################
################### Part 2 : "Ray casting" ###################
##############################################################

### This part throws rays (planes to be exact) from the the center of gravity every degree around the z axis
### The planes will intersect with the polydata on some points, and for each plane the point with the highest z is computed
### All points with highest z components are returned by findPoints

center = [gravity[0],gravity[1],bounds[5]]
rayC = raycasting.RayCasting(poly_data=polydata)
points = rayC.findPoints(center,True)




##############################################################
################### Part 3 : Postprocessing ##################
##############################################################

# Put found points into a vtkpolydata structure
skeleton_points_final = vtk.vtkPoints() # here we add the points that we find
intersection_points = vtk.vtkPoints()
intersection_cells = vtk.vtkIdList()
whole_line_final = vtk.vtkCellArray()
poly_line_final = vtk.vtkPolyLine()
poly_line_final.GetPointIds().SetNumberOfIds(points.GetNumberOfPoints())
counter = 0
for i in range (0,points.GetNumberOfPoints()):
    pid = skeleton_points_final.InsertNextPoint(points.GetPoint(i))
    poly_line_final.GetPointIds().SetId(counter, pid)

    counter +=1

poly_line_final.GetPointIds().SetNumberOfIds(counter-1)
whole_line_final.InsertNextCell(poly_line_final)

skeleton_final = vtk.vtkPolyData()
skeleton_final.SetPoints(skeleton_points_final)
skeleton_final.SetLines(whole_line_final)


# writer = vtk.vtkPolyDataWriter()
# writer.SetFileName("./rayCastLower360.vtk")
# writer.SetInputData(skeleton_final)
# writer.Write()




##############################################################
################### Part 4 : Visualization ###################
##############################################################

inputMapper = vtk.vtkDataSetMapper()
planeMapper = vtk.vtkDataSetMapper()
if vtk.VTK_MAJOR_VERSION <= 5:
    inputMapper.SetInput(polydata)
else:
    inputMapper.SetInputData(polydata)
 
imgMapper = vtk.vtkDataSetMapper()

if vtk.VTK_MAJOR_VERSION <= 5:
    imgMapper.SetInput(skeleton_final)
else:
    imgMapper.SetInputData(skeleton_final)

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