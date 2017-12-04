import vtk
import meshOperations
import numpy as np
import renderer
import raycasting
import smoothing
import scipy.interpolate as si


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
polydata = meshOp.align_to_axes(polydata)
polydata = meshOp.translate_to_origin(polydata)
print polydata.GetNumberOfPoints()
gravity = meshOp.compute_center_of_mass(polydata)
bounds = polydata.GetBounds()

#check if y correct
front = meshOp.crop_mesh(polydata, bounds[0], bounds[1], gravity[1], bounds[3], bounds[4], bounds[5])
back = meshOp.crop_mesh(polydata, bounds[0], bounds[1], bounds[2], gravity[1], bounds[4], bounds[5])
frBounds = front.GetBounds()
bkBounds = back.GetBounds()


print meshOp.compute_center_of_mass(polydata)


if frBounds[1]-frBounds[0]>bkBounds[1]-bkBounds[0]:
    print "flip y"
    polydata = meshOp.rotate_angle(polydata,[0,0,1],180)


# scale = vtk.vtkTransform()
# scale.Scale(0.9,0.9,0.9)
# scale.Update()
#
# scaleTransform = vtk.vtkTransformFilter()
# scaleTransform.SetInputConnection(featureEdges.GetOutputPort())
# scaleTransform.SetTransform(scale)
# scaleTransform.Update()

# extrude = vtk.vtkLinearExtrusionFilter()
# extrude.SetInputData(featureEdges.GetOutput())
# extrude.SetScaleFactor(-5)
# extrude.SetExtrusionTypeToNormalExtrusion()
# extrude.Update()
#
# transl = vtk.vtkTransform()
# transl.Translate(0.0,0.0,-5)
# transl.Update()
#
# translTransform = vtk.vtkTransformFilter()
# translTransform.SetInputConnection(extrude.GetOutputPort())
# translTransform.SetTransform(transl)
# translTransform.Update()
#
# extrude2 = vtk.vtkLinearExtrusionFilter()
# extrude2.SetInputData(translTransform.GetOutput())
# extrude2.SetScaleFactor(1)
# extrude2.SetExtrusionTypeToVectorExtrusion()
# extrude2.SetVector(0, 0, 50)
# extrude2.Update()
#
# rend = renderer.Renderer()
# rend.add_actor(polydata, color=[1,1,1], wireframe= False)
# rend.add_actor(extrude2.GetOutput(), color=[1,0,1], wireframe= False)
# rend.render()


dec = vtk.vtkDecimatePro()
dec.SetInputData(polydata)
dec.SetTargetReduction(0.001)
dec.PreserveTopologyOff()
dec.SplittingOn()
dec.BoundaryVertexDeletionOn()
dec.Update()



#smooth = vtk.vtkSmoothPolyDataFilter()
#smooth.SetInputConnection(dec.GetOutputPort())
#smooth.SetNumberOfIterations(1000)
#smooth.Update()

# compute curvature value
curv = vtk.vtkCurvatures()
curv.SetInputData(dec.GetOutput())
curv.SetCurvatureTypeToMean()
curv.Update()

# get rid of too high and too low curvature (the interest points are in range [0,1])
thres = vtk.vtkThresholdPoints()
thres.SetInputConnection(curv.GetOutputPort())
#thres.ThresholdByUpper(valMean-stdev)
thres.ThresholdBetween(0,1)
thres.Update()


# there are still too many points (with flat points as well), need to get rid of them by using mean and std
scalarValues = thres.GetOutput().GetPointData().GetScalars()
vals = np.zeros(scalarValues.GetNumberOfTuples())
for i in range (0, scalarValues.GetNumberOfTuples()):
    vals[i] = scalarValues.GetTuple(i)[0]

valMean =  np.mean(vals)
stdev = np.std(vals)

print curv.GetOutput().GetNumberOfPoints()
print thres.GetOutput().GetNumberOfPoints()

thres2 = vtk.vtkThresholdPoints()
thres2.SetInputConnection(thres.GetOutputPort())
thres2.ThresholdBetween(valMean-2*stdev,valMean+2*stdev)
thres2.Update()


print thres2.GetOutput().GetNumberOfPoints()

# thres3 = vtk.vtkThresholdPoints()
# thres3.SetInputConnection(thres2.GetOutputPort())
# thres3.ThresholdBetween(valMean-2*stdev,valMean+2*stdev)#/2)
# thres3.Update()


# the result is a cloud of points, need to build a polydata, but only when points are close enough to one other
randomDelaunay = vtk.vtkDelaunay2D()
randomDelaunay.SetInputConnection(thres2.GetOutputPort())
# if there are not many points, increase a bit the radius
if thres2.GetOutput().GetNumberOfPoints() < 10000:
    randomDelaunay.SetAlpha(0.3) #radius max of a point to connect with another one
else:
    randomDelaunay.SetAlpha(0.2)
randomDelaunay.Update()


# get biggest component
con_filter = vtk.vtkPolyDataConnectivityFilter()
con_filter.SetExtractionModeToLargestRegion()
con_filter.SetInputConnection(randomDelaunay.GetOutputPort())
con_filter.Update()
newpolydata = vtk.vtkPolyData()
newpolydata.DeepCopy(con_filter.GetOutput())

newBounds = newpolydata.GetBounds()

# if the teeth are disconnected, only a part has been found, need to add other big regions as well
if newpolydata.GetNumberOfCells()<3000 or newBounds[1]-newBounds[0]<0.6*(bounds[1]-bounds[0]):
    con_filter = vtk.vtkPolyDataConnectivityFilter()
    con_filter.SetExtractionModeToAllRegions()
    con_filter.ColorRegionsOn()
    con_filter.SetInputConnection(randomDelaunay.GetOutputPort())
    con_filter.ScalarConnectivityOff()
    con_filter.Update()

    nbRegions = con_filter.GetNumberOfExtractedRegions()
    con_filter.InitializeSpecifiedRegionList ()

    previousCellNb = 0

    for i in range (0, nbRegions):
        con_filter.SetExtractionModeToSpecifiedRegions()
        con_filter.AddSpecifiedRegion(i) # add a region to the selection
        con_filter.Update()
        nbCells = con_filter.GetOutput().GetNumberOfCells()
        if nbCells-previousCellNb<200: # get only big regions
            con_filter.DeleteSpecifiedRegion(i) # do not keep the region (delete from selection)
        previousCellNb = nbCells #contains all cells except the ones from deleted regions
    newpolydata.DeepCopy(con_filter.GetOutput())



newGravity = meshOp.compute_center_of_mass(newpolydata)
print newGravity

# the upper part of teeth has been extracted, as the mesh is centered on the center of mass of initial polydata
# if the center of mass of the upper part is above, the mesh is correctly oriented, otherwise, need to rotate around y
# to have z facing the good direction
if newGravity[2]<0:
    print "flip z"
    newpolydata = meshOp.rotate_angle(newpolydata,[0,1,0],180)
    polydata = meshOp.rotate_angle(polydata, [0, 1, 0], 180)


print "compute diff"


featureEdges = vtk.vtkFeatureEdges()
featureEdges.SetInputData(polydata)  # get basis
featureEdges.BoundaryEdgesOn()
featureEdges.FeatureEdgesOff()
featureEdges.ManifoldEdgesOff()
featureEdges.NonManifoldEdgesOff()
featureEdges.Update()

scale = vtk.vtkTransform()
scale.Scale(1,1,0)
scale.Update()

scaleTransform = vtk.vtkTransformFilter()
scaleTransform.SetInputConnection(featureEdges.GetOutputPort())
scaleTransform.SetTransform(scale)
scaleTransform.Update()


tri = vtk.vtkTriangleFilter()
tri.SetInputData(scaleTransform.GetOutput())
tri.Update()
edge_poly = tri.GetOutput()

print edge_poly.GetBounds()

# tri = vtk.vtkDelaunay3D()
# tri.SetInputData(cube.GetOutput())
# tri.BoundingTriangulationOff()


# extrusion = extrude2.GetOutput()
#
# randomDelaunay2 = vtk.vtkDelaunay2D()
# randomDelaunay2.SetInputConnection(extrude2.GetOutputPort())
# randomDelaunay2.SetAlpha(0.05)
# randomDelaunay2.Update()
#
# # vtkImplicitDataSet needs some scalars to interpolate to find inside/outside
# curv2 = vtk.vtkCurvatures()
# curv2.SetInputData(randomDelaunay2.GetOutput())
# curv2.SetCurvatureTypeToMean()
# curv2.Update()
#
#
# rend = renderer.Renderer()
# rend.add_actor(randomDelaunay2.GetOutput(), color=[1,1,1], wireframe= False)
# rend.add_actor(polydata, color=[1,1,1], wireframe= False)
# rend.render()
#
# dataset = vtk.vtkImplicitDataSet()
# dataset.SetDataSet(curv2.GetOutput())
#dataset.Update()
#
# clipper = vtk.vtkClipPolyData()
# clipper.SetInputData(newpolydata)
# clipper.SetClipFunction(dataset)
# clipper.GenerateClipScalarsOn()
# clipper.GenerateClippedOutputOn()
# #clipper.SetValue(0.5)
# clipper.Update()

# difference = vtk.vtkBooleanOperationPolyDataFilter()
# difference.SetOperationToDifference()
# difference.SetInputData(0,newpolydata)
# difference.SetInputData(1,randomDelaunay2.GetOutput())
# difference.SetTolerance(0.1)
# difference.Update()
#
# print "diff"
#
# newnewpoly = difference.GetOutput()
# newnewpoly.DeleteCells()
# newnewpoly.RemoveDeletedCells()
# newnewpoly.BuildCells()
#
rend = renderer.Renderer()
rend.add_actor(edge_poly, color=[1,1,1], wireframe= False)
rend.add_actor(newpolydata, color=[1,0,1], wireframe= False)
rend.render()



# rend = renderer.Renderer()
# rend.add_actor(polydata, color=[1,1,1], wireframe= False)
# rend.add_actor(newpolydata, color=[1,0,1], wireframe= False)
# rend.render()

bounds = polydata.GetBounds()

##############################################################
################### Part 2 : "Ray casting" ###################
##############################################################

### This part throws rays (planes to be exact) from the the center of gravity every degree around the z axis
### The planes will intersect with the polydata on some points, and for each plane the point with the highest z is computed
### All points with highest z components are returned by findPoints

center = [newGravity[0], newGravity[1], bounds[5]]
rayC = raycasting.RayCasting(poly_data=newpolydata)
points = rayC.findPointsAvoidContours(center, edge_poly,200)

##############################################################
################### Part 3 : Postprocessing ##################
##############################################################

# Put found points into a vtkpolydata structure
skeleton_points_final = vtk.vtkPoints()  # here we add the points that we find
intersection_points = vtk.vtkPoints()
intersection_cells = vtk.vtkIdList()
whole_line_final = vtk.vtkCellArray()
poly_line_final = vtk.vtkPolyLine()
poly_line_final.GetPointIds().SetNumberOfIds(points.GetNumberOfPoints())
counter = 0
for i in range(0, points.GetNumberOfPoints()):
    pid = skeleton_points_final.InsertNextPoint(points.GetPoint(i))
    poly_line_final.GetPointIds().SetId(counter, pid)

    counter += 1

poly_line_final.GetPointIds().SetNumberOfIds(counter - 1)
whole_line_final.InsertNextCell(poly_line_final)

rays = vtk.vtkPolyData()
rays.SetPoints(skeleton_points_final)
rays.SetLines(whole_line_final)

writer = vtk.vtkPolyDataWriter()
writer.SetFileName("./rayCastLower3602.vtk")
writer.SetInputData(rays)
writer.Write()

##############################################################
################## Part 2 : Smoothing ########################
##############################################################

### The computed polyline is very noisy, there is a need to smooth it
### Here we bring closer the points which are too far from each other, in order to have a smoother line
### Then, we compute the distance between each smoothed point and the corresponding first computed polyline point
### The new point's location will be a weighted position between the smooth and noisy initial point
### These new positions might be a bit noisy, they will be smoothed again

#
# line = np.zeros(shape=(2,points.GetNumberOfPoints()))
#
# for i in range (0,points.GetNumberOfPoints()):
#     pt = points.GetPoint(i)
#     line[0,i] = pt[0]
#     line[1,i] = pt[1]
#
#
# tck,u = si.splprep(line,k=3, s=10000, nest=3)#, nest=15)
# new_points = si.splev(u, tck)
# #new_points = tck[1]
#
#
# # Put found points into a vtkpolydata structure
# skeleton_points_final = vtk.vtkPoints() # here we add the points that we find
# intersection_points = vtk.vtkPoints()
# intersection_cells = vtk.vtkIdList()
# whole_line_final = vtk.vtkCellArray()
# poly_line_final = vtk.vtkPolyLine()
# poly_line_final.GetPointIds().SetNumberOfIds(points.GetNumberOfPoints())
# counter = 0
# for i in range (0,points.GetNumberOfPoints()):
#     pt = [new_points[0][i],new_points[1][i],gravity[2]]
#     pid = skeleton_points_final.InsertNextPoint(pt)
#     poly_line_final.GetPointIds().SetId(counter, pid)
#
#     counter +=1
#
# poly_line_final.GetPointIds().SetNumberOfIds(counter-1)
# whole_line_final.InsertNextCell(poly_line_final)
#
# # Build polydata
# skeleton_final = vtk.vtkPolyData()
# skeleton_final.SetPoints(skeleton_points_final)
# skeleton_final.SetLines(whole_line_final)




# spline = vtk.vtkSplineGraphEdges()
# spline.SetInputData(graph)
# spline.Update()
# back = vtk.vtkGraphToPolyData()
# back.SetInputConnection(spline.GetOutputPort())
# back.Update()

rend = renderer.Renderer()
rend.add_actor(polydata, color=[1,1,1], wireframe= False)
rend.add_actor(rays, color=[1,0,1], wireframe= False)
rend.render()


##############################################################
################## Part 3 : Postprocessing ###################
##############################################################

### The smoothed line points are currently not on the mesh (as they are interpolations), we need to project them on the mesh

# Smooth and weight smoothed with firstTest, then smooth again and find points on mesh
smth = smoothing.Smoothing()
rays = smth.weightedCombination(rays, 200, 0.1)  # smoothing + weighting smoothing with firstTest
alv_line_points = smth.smooth(rays, 50, 0.1)  # smooth again (but less) => points are NOT on the mesh but have right curve


# Find intersection of vertical lines with the original mesh
original_locator = vtk.vtkOBBTree()
original_locator.SetDataSet(polydata)
original_locator.BuildLocator()

bounds = polydata.GetBounds()

skeleton_points_final = vtk.vtkPoints()  # here we add the points that we find
intersection_points = vtk.vtkPoints()
intersection_cells = vtk.vtkIdList()
whole_line_final = vtk.vtkCellArray()
poly_line_final = vtk.vtkPolyLine()
poly_line_final.GetPointIds().SetNumberOfIds(len(alv_line_points))
counter = 0
for alv_line_point in alv_line_points:
    nr = original_locator.IntersectWithLine([alv_line_point[0], alv_line_point[1], bounds[4]],
                                            [alv_line_point[0], alv_line_point[1], bounds[5]],
                                            intersection_points,
                                            intersection_cells)

    if intersection_points.GetNumberOfPoints() >= 1:

        p_int = intersection_points.GetPoint(intersection_points.GetNumberOfPoints() - 1)

        pid = skeleton_points_final.InsertNextPoint(p_int)
        poly_line_final.GetPointIds().SetId(counter, pid)

        counter += 1

    else:
        print intersection_points.GetNumberOfPoints()

poly_line_final.GetPointIds().SetNumberOfIds(counter - 1)
whole_line_final.InsertNextCell(poly_line_final)

# Build polydata
skeleton_final = vtk.vtkPolyData()
skeleton_final.SetPoints(skeleton_points_final)
skeleton_final.SetLines(whole_line_final)

rend = renderer.Renderer()
rend.add_actor(polydata, color=[1,1,1], wireframe= False)
rend.add_actor(skeleton_final, color=[1,0,1], wireframe= False)
rend.render()


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