import renderer
import meshOperations
import decimator
import vtk
import numpy as np
import math

########################
#### 0. READ MESHES ####
########################
meshOp = meshOperations.MeshOperations()

original_polydata = meshOp.read(r"./image_data/lowerJawMesh.obj")


########################
##### 1. ALIGN MESH ####
########################

reoriented_polydata = meshOp.align_to_axes(original_polydata)

reoriented_polydata = meshOp.move_to_XY_plane(reoriented_polydata)

#####################################
#### 2. SIMPLIFY (decimate) MESH ####
#####################################

dec = decimator.Decimator(level=0.0)
dec_poly_data = dec.decimate(reoriented_polydata)

######################
#### 3. CROP MESH ####
######################


cropped_poly = meshOp.crop_mesh_maximum(dec_poly_data, dec_poly_data.GetBounds()[0],
                                        dec_poly_data.GetBounds()[1],
                                        dec_poly_data.GetBounds()[2], dec_poly_data.GetBounds()[3],
                                        dec_poly_data.GetBounds()[5], dec_poly_data.GetBounds()[5], 0.65, 0.90, 0.0125)



#############################################
#### 4. SIMPLIFY (decimate) MESH (again) ####
#############################################

dec = decimator.Decimator(level=0.0)
dec_poly_data = dec.decimate(cropped_poly)


#############################
#### 5. SKELETONIZE/THIN ####
#############################

# get outer edges

edge_poly = meshOp.get_outer_edges(dec_poly_data)
edge_poly,trans = meshOp.translate_edge_to_XY_plane_center_of_mass_Y_centered(edge_poly)
original_aligned_poly = meshOp.translate_tuple(reoriented_polydata, trans)


skeleton, skeleton_points = meshOp.get_edge_skeleton(edge_poly)

# find intersection of vertical lines with the original aligned mesh

skeleton_final = meshOp.place_skeleton_on_original_mesh(original_aligned_poly, skeleton_points)


###############################
### 6. visualize everything ###
###############################

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

if vtk.VTK_MAJOR_VERSION <= 5:
    imgMapper.SetInput(skeleton_final)

else:
    imgMapper.SetInputData(skeleton_final)


inputActor = vtk.vtkActor()
inputActor.SetMapper(inputMapper)
selectedActor = vtk.vtkActor()
selectedActor.SetMapper(imgMapper)
selectedActor2 = vtk.vtkActor()
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