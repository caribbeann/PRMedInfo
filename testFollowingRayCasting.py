import vtk
import meshOperations
import smoothing


##############################################################
################## Part 1 : Preprocessing ####################
##############################################################

### Load data, find principal components and reorient the data according to these principal components axes
### The first component will be the x axis and corresponds to the left-right axis
### The second component will be the y axis and corresponds to the anteroposterior axis
### The last component will be the z axis and corresponds to the craniocauda axis
### Then crop the mesh to get rid of the tongue/palatin, to ease the computation

# Load initial mesh
meshOp = meshOperations.MeshOperations()
polydata = meshOp.read("lowerJawMesh.obj")

# Load suggested alveolar line for comparison purposes
suggest = meshOp.read("suggest_alveolarRidgeLine_lower_finalVisualization.obj")

# Load computed alveolar line
reader3 = vtk.vtkPolyDataReader()
reader3.SetFileName("./rayCastLower360.vtk")
reader3.Update()
rays = reader3.GetOutput()


# Reorient data
PCADict = meshOp.compute_pca(polydata)
polydata = meshOp.rotate(polydata,[0.0,1.0,0.0],PCADict['eigenvectors'][1])

PCADict2 = meshOp.compute_pca(polydata)
polydata = meshOp.rotate(polydata,[0.0,0.0,1.0],PCADict2['eigenvectors'][2])

translation = [-polydata.GetBounds()[0],-polydata.GetBounds()[2],-polydata.GetBounds()[4]]
polydata = meshOp.translate(polydata,translation[0],translation[1],translation[2])


# Reorient suggest to match the reoriented data
suggest = meshOp.rotate(suggest,[0.0,1.0,0.0],PCADict['eigenvectors'][1])

suggest = meshOp.rotate(suggest,[0.0,0.0,1.0],PCADict2['eigenvectors'][2])

suggest = meshOp.translate(suggest,translation[0],translation[1],translation[2])


# Compute center of mass
bounds = polydata.GetBounds()

x,y,z = meshOp.compute_center_of_mass(polydata)
polydata = meshOp.translate_to_origin(polydata)
suggest = meshOp.translate(suggest,-x,-y,-z)




##############################################################
################## Part 2 : Smoothing ########################
##############################################################

### The computed polyline is very noisy, there is a need to smooth it
### Here we bring closer the points which are too far from each other, in order to have a smoother line
### Then, we compute the distance between each smoothed point and the corresponding first computed polyline point
### The new point's location will be a weighted position between the smooth and noisy initial point
### These new positions might be a bit noisy, they will be smoothed again

# Smooth and weight smoothed with firstTest, then smooth again and find points on mesh
smth = smoothing.Smoothing()
rays = smth.weightedCombination(rays,200,0.1) #smoothing + weighting smoothing with firstTest
alv_line_points = smth.smooth(rays,50,0.1) #smooth again (but less) => points are NOT on the mesh but have right curve




##############################################################
################## Part 3 : Postprocessing ###################
##############################################################

### The smoothed line points are currently not on the mesh (as they are interpolations), we need to project them on the mesh

# Find intersection of vertical lines with the original mesh
original_locator = vtk.vtkOBBTree()
original_locator.SetDataSet(polydata)
original_locator.BuildLocator()

bounds = polydata.GetBounds()

skeleton_points_final = vtk.vtkPoints() # here we add the points that we find
intersection_points = vtk.vtkPoints()
intersection_cells = vtk.vtkIdList()
whole_line_final = vtk.vtkCellArray()
poly_line_final = vtk.vtkPolyLine()
poly_line_final.GetPointIds().SetNumberOfIds(len(alv_line_points))
counter = 0
for alv_line_point in alv_line_points:
    nr = original_locator.IntersectWithLine([alv_line_point[0],alv_line_point[1],bounds[4]],
                                         [alv_line_point[0], alv_line_point[1], bounds[5]],
                                            intersection_points,
                                        intersection_cells)

    if intersection_points.GetNumberOfPoints()>=1:

        p_int = intersection_points.GetPoint(intersection_points.GetNumberOfPoints()-1)

        pid = skeleton_points_final.InsertNextPoint(p_int)
        poly_line_final.GetPointIds().SetId(counter, pid)

        counter +=1

    else :
    	print intersection_points.GetNumberOfPoints()    
  

poly_line_final.GetPointIds().SetNumberOfIds(counter-1)
whole_line_final.InsertNextCell(poly_line_final)

# Build polydata
skeleton_final = vtk.vtkPolyData()
skeleton_final.SetPoints(skeleton_points_final)
skeleton_final.SetLines(whole_line_final)



##############################################################
#################### Part 4 : Visualization ##################
##############################################################

### Visualize the jaw mesh, as well as the computed alveolar line (white) and the suggested alveolar line (red)

inputMapper = vtk.vtkDataSetMapper()
planeMapper = vtk.vtkDataSetMapper()
if vtk.VTK_MAJOR_VERSION <= 5:
    inputMapper.SetInput(polydata)
else:
    inputMapper.SetInputData(polydata)
 
imgMapper = vtk.vtkDataSetMapper()
imgMapper2 = vtk.vtkDataSetMapper()

if vtk.VTK_MAJOR_VERSION <= 5:
    imgMapper.SetInput(skeleton_final)
    imgMapper2.SetInput(suggest)
else:
    imgMapper.SetInputData(skeleton_final)
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