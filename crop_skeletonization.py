import renderer
import meshOperations
import decimator
import vtk
import numpy as np


# my_renderer = renderer.Renderer(file_name=r".\image data\lowerJawMesh.obj", wire_frame=False)

# my_renderer.render()


reader = vtk.vtkOBJReader()

reader.SetFileName(r".\image data\lowerJawMesh.obj")

reader.Update()

polydata = reader.GetOutput()




########################
##### 1. ALIGN MESH ####
########################

meshOp = meshOperations.MeshOperations(poly_data=polydata)

# move center of gravity of the object to origin - do this every time something changes

moved_poly_data = meshOp.move_to_origin()
meshOp.changePolyData(moved_poly_data)


#my_renderer = renderer.Renderer(poly_data=moved_poly_data, wire_frame=False)
#my_renderer.render()

# compute PCA 3 times to align the object main axis with the coordinate system
# PCADict = meshOp.computePCA()
# reorientedPolyData = meshOp.rotate([1.0,0.0,0.0],PCADict['eigenvectors'][0])
# meshOp.changePolyData(reorientedPolyData)
# moved_poly_data = meshOp.move_to_origin()
# meshOp.changePolyData(moved_poly_data)
#
PCADict2 = meshOp.computePCA()
reorientedPolyData = meshOp.rotate([0.0,1.0,0.0],PCADict2['eigenvectors'][1])
meshOp.changePolyData(reorientedPolyData)
moved_poly_data = meshOp.move_to_origin()
meshOp.changePolyData(moved_poly_data)


PCADict3 = meshOp.computePCA()
reorientedPolyData = meshOp.rotate([0.0,0.0,1.0],PCADict3['eigenvectors'][2])
meshOp.changePolyData(reorientedPolyData)
moved_poly_data = meshOp.move_to_origin()
meshOp.changePolyData(moved_poly_data)


# find out where are the teeth and align

reorientedPolyData = moved_poly_data
meshOp.changePolyData(reorientedPolyData)

#my_renderer = renderer.Renderer(poly_data=reorientedPolyData, wire_frame=False)
#my_renderer.render()

extent = reorientedPolyData.GetBounds()[1] - reorientedPolyData.GetBounds()[0]

third1_poly = meshOp.cropMesh(reorientedPolyData.GetBounds()[0], reorientedPolyData.GetBounds()[0] + extent/3,
                             reorientedPolyData.GetBounds()[2], reorientedPolyData.GetBounds()[3],
                             reorientedPolyData.GetBounds()[4], reorientedPolyData.GetBounds()[5])

third2_poly = meshOp.cropMesh(reorientedPolyData.GetBounds()[0] + extent/3,reorientedPolyData.GetBounds()[0] + extent*2/3,
                             reorientedPolyData.GetBounds()[2], reorientedPolyData.GetBounds()[3],
                             reorientedPolyData.GetBounds()[4], reorientedPolyData.GetBounds()[5])

third3_poly = meshOp.cropMesh(reorientedPolyData.GetBounds()[0] + extent*2/3, reorientedPolyData.GetBounds()[0] + extent,
                             reorientedPolyData.GetBounds()[2], reorientedPolyData.GetBounds()[3],
                             reorientedPolyData.GetBounds()[4], reorientedPolyData.GetBounds()[5])


if int(third1_poly.GetBounds()[5]) == int(third2_poly.GetBounds()[5]) == int(third3_poly.GetBounds()[5]):
    #  then we have a lawer jaw, so rotate 180 deg around y axis to align
    transform = vtk.vtkTransform()
    transform.RotateWXYZ(180, 0,1,0)
    transformFilter=vtk.vtkTransformPolyDataFilter()
    transformFilter.SetInputData(reorientedPolyData)
    transformFilter.SetTransform(transform)
    transformFilter.Update()
    reorientedPolyData = transformFilter.GetOutput()
    meshOp.changePolyData(reorientedPolyData)
    reorientedPolyData = meshOp.move_to_origin()

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

decimator = decimator.Decimator(level=0.0, poly_data=reorientedPolyData)
decimator.decimate()
dec_poly_data = decimator.get_decimated_poly()

######################
#### 3. CROP MESH ####
######################

meshOp.changePolyData(dec_poly_data)

con_filter = vtk.vtkPolyDataConnectivityFilter()

last_biggest_percentage = 0
last_crop_poly = None
for i in np.arange(0.50, 0.9, 0.025):
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


my_renderer = renderer.Renderer(poly_data=last_crop_poly, wire_frame=False)
my_renderer.render()


#############################
#### 4. SKELETONIZE/THIN ####
#############################