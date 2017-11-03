import vtk
import numpy as np
import meshOperations
import renderer
import raycasting
import smoothing

# Create polydata to slice the grid with. In this case, use a cone. This could
# be any polydata including a stl file.
reader = vtk.vtkOBJReader()

reader.SetFileName("lowerJawMesh.obj")

reader.Update()

reader2 = vtk.vtkOBJReader()

reader2.SetFileName("suggest_alveolarRidgeLine_lower_finalVisualization.obj")

reader2.Update()



polydata = reader.GetOutput()

#Preprocessing : rotate data and find center of gravity
meshOp = meshOperations.MeshOperations(poly_data=polydata)
PCADict = meshOp.computePCA()
polydata = meshOp.rotate([0.0,1.0,0.0],PCADict['eigenvectors'][1])

meshOp.changePolyData(polydata)

PCADict2 = meshOp.computePCA()
polydata = meshOp.rotate([0.0,0.0,1.0],PCADict2['eigenvectors'][2])

translation = [-polydata.GetBounds()[0],-polydata.GetBounds()[2],-polydata.GetBounds()[4]]
meshOp.changePolyData(polydata)
polydata = meshOp.translate(translation[0],translation[1],translation[2])


suggest = reader2.GetOutput() 

meshOp2 = meshOperations.MeshOperations(poly_data=suggest)
suggest = meshOp2.rotate([0.0,1.0,0.0],PCADict['eigenvectors'][1])

meshOp2.changePolyData(suggest)
suggest = meshOp2.rotate([0.0,0.0,1.0],PCADict2['eigenvectors'][2])

meshOp2.changePolyData(suggest)
suggest = meshOp2.translate(translation[0],translation[1],translation[2])

bounds = polydata.GetBounds()

meshOp.changePolyData(polydata)
x,y,z = meshOp.computeCenterOfMass()
polydata = meshOp.move_to_origin()
meshOp.changePolyData(suggest)
suggest = meshOp.translate(-x,-y,-z)




#Open found points
reader3 = vtk.vtkPolyDataReader()
reader3.SetFileName("./rayCastLower360.vtk")
reader3.Update()

rays = reader3.GetOutput()


rend=renderer.Renderer(poly_data=rays, wire_frame=False)
rend.render()


# Smooth and weight smoothed with firstTest, then smooth again and find points on mesh
smth = smoothing.Smoothing(poly_data=rays)
rays = smth.weightedCombination(200,0.1) #smoothing + weighting smoothing with firstTest
smth.changePolyData(rays)
alv_line_points = smth.smooth(50,0.1) #smooth again (but less) => points are NOT on the mesh but have right curve


# find intersection of vertical lines with the original aligned mesh

original_locator = vtk.vtkOBBTree()
original_locator.SetDataSet(polydata)
original_locator.BuildLocator()

cells_original = vtk.vtkIdList()

line_length = (polydata.GetBounds()[5]-polydata.GetBounds()[4]) + 1
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

skeleton_final = vtk.vtkPolyData()
skeleton_final.SetPoints(skeleton_points_final)
skeleton_final.SetLines(whole_line_final)



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