import renderer
import meshOperations
import decimator
import vtk
import vtkpointcloud
from vtk.util.numpy_support import vtk_to_numpy
import numpy as np
import math


# my_renderer = renderer.Renderer(file_name=r".\image data\lowerJawMesh.obj", wire_frame=False)

# my_renderer.render()


reader = vtk.vtkOBJReader()

reader.SetFileName("upperJawMesh.obj")

reader.Update()

polydata = reader.GetOutput()

reader2 = vtk.vtkOBJReader()

reader2.SetFileName("suggest_alveolarRidgeLine_upper_finalVisualization.obj")

reader2.Update()

suggest = reader2.GetOutput()


########################
##### 1. ALIGN MESH ####
########################

meshOp = meshOperations.MeshOperations(poly_data=polydata)

# move center of gravity of the object to origin - do this every time something changes
x, y, z = meshOp.computeCenterOfMass()
moved_poly_data = meshOp.move_to_origin()
meshOp.changePolyData(moved_poly_data)


#my_renderer = renderer.Renderer(poly_data=moved_poly_data, wire_frame=False)
#my_renderer.render()

# compute PCA 3 times to align the object main axis with the coordinate system

PCADict2 = meshOp.computePCA()
reorientedPolyData = meshOp.rotate([0.0,1.0,0.0],PCADict2['eigenvectors'][1])
meshOp.changePolyData(reorientedPolyData)
x1, y1, z1 = meshOp.computeCenterOfMass()
moved_poly_data = meshOp.move_to_origin()
meshOp.changePolyData(moved_poly_data)


PCADict3 = meshOp.computePCA()
reorientedPolyData = meshOp.rotate([0.0,0.0,1.0],PCADict3['eigenvectors'][2])
meshOp.changePolyData(reorientedPolyData)
x2, y2, z2 = meshOp.computeCenterOfMass()
moved_poly_data = meshOp.move_to_origin()
meshOp.changePolyData(moved_poly_data)


#move also suggestion
meshOp.changePolyData(suggest)
suggest = meshOp.translate(-x,-y,-z)
suggest = meshOp.rotate([0.0,1.0,0.0],PCADict2['eigenvectors'][1])

meshOp.changePolyData(suggest)
suggest = meshOp.translate(-x1,-y1,-z1)
meshOp.changePolyData(suggest)
suggest = meshOp.rotate([0.0,0.0,1.0],PCADict3['eigenvectors'][2])
meshOp.changePolyData(suggest)
suggest = meshOp.translate(-x2,-y2,-z2)

# my_renderer = renderer.Renderer(poly_data=suggest, wire_frame=False)
# my_renderer.render()

# find out where are the teeth and align

reorientedPolyData = moved_poly_data
meshOp.changePolyData(reorientedPolyData)


# extent = reorientedPolyData.GetBounds()[1] - reorientedPolyData.GetBounds()[0]

# third1_poly = meshOp.cropMesh(reorientedPolyData.GetBounds()[0], reorientedPolyData.GetBounds()[0] + extent/3,
#                              reorientedPolyData.GetBounds()[2], reorientedPolyData.GetBounds()[3],
#                              reorientedPolyData.GetBounds()[4], reorientedPolyData.GetBounds()[5])

# third2_poly = meshOp.cropMesh(reorientedPolyData.GetBounds()[0] + extent/3,reorientedPolyData.GetBounds()[0] + extent*2/3,
#                              reorientedPolyData.GetBounds()[2], reorientedPolyData.GetBounds()[3],
#                              reorientedPolyData.GetBounds()[4], reorientedPolyData.GetBounds()[5])

# third3_poly = meshOp.cropMesh(reorientedPolyData.GetBounds()[0] + extent*2/3, reorientedPolyData.GetBounds()[0] + extent,
#                              reorientedPolyData.GetBounds()[2], reorientedPolyData.GetBounds()[3],
#                              reorientedPolyData.GetBounds()[4], reorientedPolyData.GetBounds()[5])


# if int(third1_poly.GetBounds()[5]) == int(third2_poly.GetBounds()[5]) == int(third3_poly.GetBounds()[5]):
#     #  then we have a lawer jaw, so rotate 180 deg around y axis to align
#     transform = vtk.vtkTransform()
#     transform.RotateWXYZ(180, 0,1,0)
#     transformFilter=vtk.vtkTransformPolyDataFilter()
#     transformFilter.SetInputData(reorientedPolyData)
#     transformFilter.SetTransform(transform)
#     transformFilter.Update()
#     reorientedPolyData = transformFilter.GetOutput()
#     meshOp.changePolyData(reorientedPolyData)
#     reorientedPolyData = meshOp.move_to_origin()

#my_renderer = renderer.Renderer(poly_data=reorientedPolyData, wire_frame=False)
#my_renderer.render()


# move object to origin of Z
meshOp.changePolyData(reorientedPolyData)
reorientedPolyData = meshOp.translate(0,0,-reorientedPolyData.GetBounds()[4])

#my_renderer = renderer.Renderer(poly_data=reorientedPolyData, wire_frame=False)
#my_renderer.render()





#####################################
#### 2. SIMPLIFY (decimate) MESH ####
#####################################

dec = decimator.Decimator(level=0.0, poly_data=reorientedPolyData)
dec.decimate()
dec_poly_data = dec.get_decimated_poly()

######################
#### 3. CROP MESH ####
######################

meshOp.changePolyData(dec_poly_data)

con_filter = vtk.vtkPolyDataConnectivityFilter()

last_biggest_percentage = 0
last_crop_poly = None
for i in np.arange(0.60, 0.95, 0.0125):
    cropped_poly_data = meshOp.cropMesh(dec_poly_data.GetBounds()[0], dec_poly_data.GetBounds()[1],
                                          dec_poly_data.GetBounds()[2], dec_poly_data.GetBounds()[3],
                                        i * dec_poly_data.GetBounds()[5], dec_poly_data.GetBounds()[5])
    con_filter.SetInputData(cropped_poly_data)
    con_filter.SetExtractionModeToAllRegions()
    con_filter.Update()
    if con_filter.GetNumberOfExtractedRegions()==1:
        if i>last_biggest_percentage:
            last_biggest_percentage = i
            last_crop_poly = cropped_poly_data





#####################################
#### 4. SIMPLIFY (decimate) MESH (again) ####
#####################################

dec = decimator.Decimator(level=0.0, poly_data=last_crop_poly)
dec.decimate()
dec_poly_data = dec.get_decimated_poly()


#############################
#### 5. SKELETONIZE/THIN ####
#############################


# get outer edges

featureEdges = vtk.vtkFeatureEdges()

featureEdges.SetInputData(dec_poly_data)
featureEdges.BoundaryEdgesOn()
featureEdges.FeatureEdgesOff()
featureEdges.ManifoldEdgesOff()
featureEdges.NonManifoldEdgesOff()
featureEdges.Update()
edge_poly = featureEdges.GetOutput()

# my_renderer = renderer.Renderer(poly_data=edge_poly, wire_frame=False)
# my_renderer.render()



# move edges polydata to the XY plane and in Y positive region

def align_mesh(operation, mesh, tran1,tran2,tran3,tran4,tran5):
    """
    Aligns mesh with given translations
    :param operation:
    :param mesh:
    :return:
    # """  
    operation.changePolyData(mesh)
    mesh = operation.transl(tran1)
    operation.changePolyData(mesh)
    mesh = operation.transl(tran2)
    operation.changePolyData(mesh)
    mesh = operation.transl(tran3) #move in the XY plane to meet the line
    operation.changePolyData(mesh)
    mesh = operation.transl(tran4)
    operation.changePolyData(mesh)
    mesh = operation.transl(tran5)
    operation.changePolyData(mesh)

    return mesh


def align_edge_mesh(operation, mesh):
    """
    Aligns mesh with minimum z bound to the XY plane and minimum y bound to the XZ plane. The Y axis will
    , at the and, pass through the center of mass of the mesh passed
    :param operation:
    :param mesh:
    :return:
    """   
    operation.changePolyData(mesh)
    tran1 = [0,-mesh.GetBounds()[2], -mesh.GetBounds()[4]]
    mesh = operation.translate(0,-mesh.GetBounds()[2], -mesh.GetBounds()[4])
    operation.changePolyData(mesh)
    x,y,z = operation.computeCenterOfMass()
    tran2 = [-x,-y,-z]
    mesh = operation.translate(-x,-y,-z)
    operation.changePolyData(mesh)
    tran3 = [0,0, -mesh.GetBounds()[4]]
    mesh = operation.translate(0,0, -mesh.GetBounds()[4]) #move in the XY plane to meet the line
    operation.changePolyData(mesh)
    tran4 = [0,0,0]
    mesh = operation.translate(0,0,0)
    operation.changePolyData(mesh)
    tran5 = [0,-mesh.GetBounds()[2], 0]
    mesh = operation.translate(0,-mesh.GetBounds()[2], 0)
    operation.changePolyData(mesh)

    return (mesh,tran1,tran2,tran3,tran4,tran5)  

edge_poly,tran1,tran2,tran3,tran4,tran5 = align_edge_mesh(meshOp, edge_poly)

original_aligned_dec_poly = align_mesh(meshOp, dec_poly_data,tran1,tran2,tran3,tran4,tran5)
original_aligned_poly = align_mesh(meshOp, reorientedPolyData,tran1,tran2,tran3,tran4,tran5)
# meshOps = meshOperations.MeshOperations(suggest)
#suggest = meshOps.translate(tran1[0],tran1[1],tran1[2])
suggest = align_mesh(meshOp, suggest,tran1,tran2,tran3,tran4,tran5)


# get intersection of rays sent from the center of mass outwards
edge_locator = vtk.vtkCellLocator()
edge_locator.SetDataSet(edge_poly)
edge_locator.BuildLocator()

cells_edges = vtk.vtkIdList()


line_length = abs(edge_poly.GetBounds()[3]) + abs(edge_poly.GetBounds()[1])
start_point = [0,0,0]
end_points = []
for angle in np.linspace(math.pi/6, math.pi*5/6, 120): # from 30deg to 150deg in 1 deg steps
    end_points.append([start_point[0]+line_length*math.cos(angle),start_point[1]+ line_length*math.sin(angle),
                       start_point[2]])

end_points = np.array(end_points)

skeleton_points = vtk.vtkPoints() # here we add the points that we find
whole_line = vtk.vtkCellArray()
poly_line = vtk.vtkPolyLine()

alv_line_points = []
poly_line.GetPointIds().SetNumberOfIds(end_points.shape[0])
counter = 0
for i in range(end_points.shape[0]):

    nr = edge_locator.FindCellsAlongLine(start_point,end_points[i],0.001, cells_edges)
    pid1 = vtk.vtkIdList() # contains the point ids of the first intersection
    pid2 = vtk.vtkIdList()
    if cells_edges.GetNumberOfIds()== 2:
        # get only the first two intersection points
        id1 = cells_edges.GetId(0)
        id2 = cells_edges.GetId(1)
        edge_poly.GetCellPoints(id1, pid1)
        edge_poly.GetCellPoints(id2, pid2)

        p_int_1 = np.add(np.array(edge_poly.GetPoint(pid1.GetId(0))),np.array(edge_poly.GetPoint(pid1.GetId(1)))) /2
        p_int_2 = np.add(np.array(edge_poly.GetPoint(pid2.GetId(0))),np.array(edge_poly.GetPoint(pid2.GetId(1)))) /2

        p_alv_line = np.add(p_int_1, p_int_2)/2
        alv_line_points.append(p_alv_line)
        skeleton_points.InsertNextPoint(p_alv_line)

        poly_line.GetPointIds().SetId(counter, counter)
        counter += 1 
poly_line.GetPointIds().SetNumberOfIds(counter-1)
whole_line.InsertNextCell(poly_line)
skeleton = vtk.vtkPolyData()
skeleton.SetPoints(skeleton_points)
skeleton.SetLines(whole_line)

# find intersection of vertical lines with the original aligned mesh

original_locator = vtk.vtkOBBTree()
original_locator.SetDataSet(original_aligned_poly)
original_locator.BuildLocator()

cells_original = vtk.vtkIdList()

line_length = abs(original_aligned_poly.GetBounds()[5]) + 1

skeleton_points_final = vtk.vtkPoints() # here we add the points that we find
intersection_points = vtk.vtkPoints()
intersection_cells = vtk.vtkIdList()
whole_line_final = vtk.vtkCellArray()
poly_line_final = vtk.vtkPolyLine()
poly_line_final.GetPointIds().SetNumberOfIds(len(alv_line_points))
counter = 0
for alv_line_point in alv_line_points:
    nr = original_locator.IntersectWithLine(alv_line_point,
                                         [alv_line_point[0], alv_line_point[1], alv_line_point[2]+line_length],
                                            intersection_points,
                                        intersection_cells)

    if intersection_points.GetNumberOfPoints()>=1:

        p_int = intersection_points.GetPoint(intersection_points.GetNumberOfPoints()-1)

        pid = skeleton_points_final.InsertNextPoint(p_int)
        poly_line_final.GetPointIds().SetId(counter, pid)

        counter +=1
  

poly_line_final.GetPointIds().SetNumberOfIds(counter-1)
whole_line_final.InsertNextCell(poly_line_final)

skeleton_final = vtk.vtkPolyData()
skeleton_final.SetPoints(skeleton_points_final)
skeleton_final.SetLines(whole_line_final)


###############################
### 6. visualize everything ###
###############################

# rend=renderer.Renderer(poly_data=skeleton_final, wire_frame=False)
# rend.render()


#print intersect_plane_with_edges(poly, [100,100,0])

mapper = vtk.vtkPolyDataMapper()
mapper.SetInputData(original_aligned_poly)


actor = vtk.vtkActor()
actor.SetMapper(mapper)

mapper2 = vtk.vtkPolyDataMapper()
mapper2.SetInputData(skeleton_final)

actor2 = vtk.vtkActor()
actor2.SetMapper(mapper2)
actor2.GetProperty().SetColor(0.5, 0.5, 1.0)
actor2.GetProperty().SetLineWidth(5)
renderWindow = vtk.vtkRenderWindow()
rend = vtk.vtkRenderer()



renderWindow.AddRenderer(rend)

rend.AddActor(actor)
rend.AddActor(actor2)
rend.ResetCamera()
renderWindowInteractor = vtk.vtkRenderWindowInteractor()
renderWindowInteractor.SetRenderWindow(renderWindow)
renderWindow.Render()
renderWindowInteractor.Start()

#Visualization stuff
inputMapper = vtk.vtkDataSetMapper()
planeMapper = vtk.vtkDataSetMapper()
if vtk.VTK_MAJOR_VERSION <= 5:
    inputMapper.SetInput(original_aligned_poly)
else:
    inputMapper.SetInputData(original_aligned_poly)
 
imgMapper = vtk.vtkDataSetMapper()
imgMapper2 = vtk.vtkDataSetMapper()

if vtk.VTK_MAJOR_VERSION <= 5:
    imgMapper.SetInput(skeleton_final)
    imgMapper2.SetInput(suggest)
else:
    imgMapper.SetInputData(skeleton_final)
    imgMapper2.SetInputData(suggest)

my_renderer = renderer.Renderer(poly_data=suggest, wire_frame=False)
my_renderer.render()    

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


# Setup the renderers
leftRenderer = vtk.vtkRenderer()
renderWindow.AddRenderer(leftRenderer)
leftRenderer.SetBackground(.6, .5, .4)



leftRenderer.AddActor(inputActor)
leftRenderer.AddActor(selectedActor)
leftRenderer.AddActor(selectedActor2)

transform = vtk.vtkTransform()
transform.Translate(1.0, 0.0, 0.0)
transform.Scale(50,50,50)
 
axes = vtk.vtkAxesActor()
axes.SetUserTransform(transform)
leftRenderer.AddActor(axes)


leftRenderer.ResetCamera()

renderWindow.Render()
interactor.Start()