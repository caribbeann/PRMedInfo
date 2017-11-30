import vtk
import meshOperations
import scipy.interpolate as si
import numpy as np


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

# Compute center of gravity
gravity = meshOp.compute_center_of_mass(polydata)
bounds = polydata.GetBounds()

featureEdges = vtk.vtkFeatureEdges()

featureEdges.SetInputData(polydata) # get basis
featureEdges.BoundaryEdgesOn()
featureEdges.FeatureEdgesOff()
featureEdges.ManifoldEdgesOff()
featureEdges.NonManifoldEdgesOff()
featureEdges.Update()
edge_poly = featureEdges.GetOutput()	

ptIds = vtk.vtkIdList()
for i in range (0, edge_poly.GetNumberOfPoints()):
    ptIds.InsertNextId(polydata.FindPoint(edge_poly.GetPoint(i)))

while ptIds.GetNumberOfIds()>0:
    # print "id"
    # print ptIds.GetId(0)
    pt = polydata.GetPoint(ptIds.GetId(0))
    #print ptIds.GetNumberOfIds()
    #if pt[2]<zmax and pt[2]>zmin: #delete
    neighborsCellIds = vtk.vtkIdList()
    polydata.GetPointCells(ptIds.GetId(0),neighborsCellIds)
    # print pt
    # print neighborsCellIds.GetNumberOfIds()
    for i in range (0,neighborsCellIds.GetNumberOfIds()):
        cellId = neighborsCellIds.GetId(i)
        #print cellId
        newPtIds = vtk.vtkIdList()
        if polydata.GetCell(cellId).GetCellType() != vtk.VTK_EMPTY_CELL:
            polydata.GetCellPoints(cellId,newPtIds)
            for j in range (0,newPtIds.GetNumberOfIds()):
                newPtId = newPtIds.GetId(j)
                newPt = polydata.GetPoint(newPtId)
                if pt[2]-newPt[2]<-0.3:
                    ptIds.InsertNextId(newPtId)
        polydata.DeleteCell(cellId)
    #print ptIds.GetId(0)
    polydata.DeletePoint(ptIds.GetId(0))
    ptIds.DeleteId(ptIds.GetId(0))
polydata.RemoveDeletedCells()
polydata.BuildCells()

#get rid of horizontal edges

# featureEdges = vtk.vtkFeatureEdges()

# featureEdges.SetInputData(polydata) # get basis
# featureEdges.BoundaryEdgesOn()
# featureEdges.FeatureEdgesOff()
# featureEdges.ManifoldEdgesOff()
# featureEdges.NonManifoldEdgesOff()
# featureEdges.Update()
# edge_poly = featureEdges.GetOutput()	


# ptIds = vtk.vtkIdList()
# for i in range (0, edge_poly.GetNumberOfPoints()):
# 	ptIds.InsertNextId(polydata.FindPoint(edge_poly.GetPoint(i)))

# while ptIds.GetNumberOfIds()>0:
# 	# print "id"
# 	# print ptIds.GetId(0)
# 	pt = polydata.GetPoint(ptIds.GetId(0))
# 	#print ptIds.GetNumberOfIds()
# 	#if pt[2]<zmax and pt[2]>zmin: #delete
# 	neighborsCellIds = vtk.vtkIdList()
# 	polydata.GetPointCells(ptIds.GetId(0),neighborsCellIds)
# 	# print pt
# 	# print neighborsCellIds.GetNumberOfIds()
# 	for i in range (0,neighborsCellIds.GetNumberOfIds()):
# 		cellId = neighborsCellIds.GetId(i)
# 		#print cellId
# 		newPtIds = vtk.vtkIdList()
# 		if polydata.GetCell(cellId).GetCellType() != vtk.VTK_EMPTY_CELL:
# 			polydata.GetCellPoints(cellId,newPtIds)
# 			for j in range (0,newPtIds.GetNumberOfIds()):
# 				newPtId = newPtIds.GetId(j)
# 				newPt = polydata.GetPoint(newPtId)
# 				if pt[2]-newPt[2]>-0.06 and pt[2]-newPt[2]<0.06:
# 					ptIds.InsertNextId(newPtId)
# 		polydata.DeleteCell(cellId)
# 	#print ptIds.GetId(0)
# 	polydata.DeletePoint(ptIds.GetId(0))
# 	ptIds.DeleteId(ptIds.GetId(0))
# polydata.RemoveDeletedCells()
# polydata.BuildCells()





edge_locator = vtk.vtkCellLocator()
edge_locator.SetDataSet(polydata)
edge_locator.BuildLocator()

centerTongue = [0.0,0.0,0.0]

intersection_points = vtk.vtkPoints()
intersection_cells = vtk.vtkIdList()

pcoord = [0.0,0.0,0.0]
junk= [0.0,0.0,0.0]
t = vtk.mutable(0)
subId = vtk.mutable(0)
centerCellId = vtk.mutable(0)
cell = vtk.vtkGenericCell()
nr = edge_locator.IntersectWithLine([gravity[0],gravity[1],bounds[4]+1],
                                         [gravity[0], gravity[1], bounds[5]-1],0.001,t,pcoord,junk, subId,centerCellId,cell)


con_filter = vtk.vtkPolyDataConnectivityFilter()
con_filter.SetExtractionModeToLargestRegion()
con_filter.SetInputData(polydata)
con_filter.Update()
polydata = con_filter.GetOutput()


neighborsCellIds = vtk.vtkIdList()
neighborsCellIds.InsertNextId(centerCellId)

ptIds = vtk.vtkIdList()
ptIds.InsertNextId(polydata.FindPoint(pcoord))


while ptIds.GetNumberOfIds()>0:
    # print "id"
    # print ptIds.GetId(0)
    pt = polydata.GetPoint(ptIds.GetId(0))
    #print ptIds.GetNumberOfIds()
    #if pt[2]<zmax and pt[2]>zmin: #delete
    neighborsCellIds = vtk.vtkIdList()
    polydata.GetPointCells(ptIds.GetId(0),neighborsCellIds)
    # print pt
    # print neighborsCellIds.GetNumberOfIds()
    for i in range (0,neighborsCellIds.GetNumberOfIds()):
        cellId = neighborsCellIds.GetId(i)
        #print cellId
        newPtIds = vtk.vtkIdList()
        if polydata.GetCell(cellId).GetCellType() != vtk.VTK_EMPTY_CELL:
            polydata.GetCellPoints(cellId,newPtIds)
            for j in range (0,newPtIds.GetNumberOfIds()):
                newPtId = newPtIds.GetId(j)
                newPt = polydata.GetPoint(newPtId)
                #if pt[2]-newPt[2]<0.08 and pt[2]-newPt[2]>-0.08:
                if pt[2]-newPt[2]>-0.055 or (pt[2]-newPt[2]>-0.01 and pt[2]-newPt[2]<0.01):# and pt[2]-newPt[2]>-0.08:
                    ptIds.InsertNextId(newPtId)
        polydata.DeleteCell(cellId)
    #print ptIds.GetId(0)
    polydata.DeletePoint(ptIds.GetId(0))
    ptIds.DeleteId(ptIds.GetId(0))
polydata.RemoveDeletedCells()
polydata.BuildCells()



con_filter = vtk.vtkPolyDataConnectivityFilter()
con_filter.SetExtractionModeToLargestRegion()
con_filter.SetInputData(polydata)
con_filter.Update()
polydata = con_filter.GetOutput()



#get rid of horizontal edges

featureEdges = vtk.vtkFeatureEdges()

featureEdges.SetInputData(polydata) # get basis
featureEdges.BoundaryEdgesOn()
featureEdges.FeatureEdgesOff()
featureEdges.ManifoldEdgesOff()
featureEdges.NonManifoldEdgesOff()
featureEdges.Update()
edge_poly = featureEdges.GetOutput()	


ptIds = vtk.vtkIdList()
for i in range (0, edge_poly.GetNumberOfPoints()):
    ptIds.InsertNextId(polydata.FindPoint(edge_poly.GetPoint(i)))

while ptIds.GetNumberOfIds()>0:
    # print "id"
    # print ptIds.GetId(0)
    pt = polydata.GetPoint(ptIds.GetId(0))
    #print ptIds.GetNumberOfIds()
    #if pt[2]<zmax and pt[2]>zmin: #delete
    neighborsCellIds = vtk.vtkIdList()
    polydata.GetPointCells(ptIds.GetId(0),neighborsCellIds)
    # print pt
    # print neighborsCellIds.GetNumberOfIds()
    for i in range (0,neighborsCellIds.GetNumberOfIds()):
        cellId = neighborsCellIds.GetId(i)
        #print cellId
        newPtIds = vtk.vtkIdList()
        if polydata.GetCell(cellId).GetCellType() != vtk.VTK_EMPTY_CELL:
            polydata.GetCellPoints(cellId,newPtIds)
            for j in range (0,newPtIds.GetNumberOfIds()):
                newPtId = newPtIds.GetId(j)
                newPt = polydata.GetPoint(newPtId)
                if pt[2]-newPt[2]>-0.05 and pt[2]-newPt[2]<0.05:
                    ptIds.InsertNextId(newPtId)
        polydata.DeleteCell(cellId)
    #print ptIds.GetId(0)
    polydata.DeletePoint(ptIds.GetId(0))
    ptIds.DeleteId(ptIds.GetId(0))
polydata.RemoveDeletedCells()
polydata.BuildCells()


con_filter = vtk.vtkPolyDataConnectivityFilter()
con_filter.SetExtractionModeToLargestRegion()
con_filter.SetInputData(polydata)
con_filter.Update()
polydata = con_filter.GetOutput()

import renderer
rend = renderer.Renderer()
rend.add_actor(polydata, color=[1,1,1], wireframe= False)
rend.render()


featureEdges = vtk.vtkFeatureEdges()

featureEdges.SetInputData(polydata) # get basis
featureEdges.BoundaryEdgesOn()
featureEdges.FeatureEdgesOff()
featureEdges.ManifoldEdgesOff()
featureEdges.NonManifoldEdgesOff()
featureEdges.Update()
edge_poly = featureEdges.GetOutput()



ptIds = vtk.vtkIdList()
ptIds.InsertNextId(edge_poly.FindPoint(edge_poly.GetPoint(0)))

skeleton_points_final = vtk.vtkPoints() # here we add the points that we find
edgePoints = vtk.vtkPoints()
pointsIds = vtk.vtkIdList()
whole_line_final = vtk.vtkCellArray()
poly_line_final = vtk.vtkPolyLine()
poly_line_final.GetPointIds().SetNumberOfIds(edge_poly.GetNumberOfPoints())




line = np.zeros(shape=(2,edge_poly.GetNumberOfPoints()))

counter = 0
# print edge_poly.GetNumberOfPoints()
# cellId = neighborsCellIds.GetId(0)
# newPtIds = vtk.vtkIdList()
# if edge_poly.GetCell(cellId).GetCellType() != vtk.VTK_EMPTY_CELL:
#     edge_poly.GetCellPoints(cellId, newPtIds)
#     for j in range(0, newPtIds.GetNumberOfIds()):
#         newPtId = newPtIds.GetId(j)
#         newPt = edge_poly.GetPoint(newPtId)
#         if ptIds.InsertUniqueId(newPtId) > counter:
#             counter = counter + 1

pb = False
lastIntersection = -1
nbPts = 0
lastId = 0
while nbPts < edge_poly.GetNumberOfPoints()-1:
    print ptIds
    id = nbPts
    if nbPts>=ptIds.GetNumberOfIds():
        id = lastIntersection
        pt = edge_poly.GetPoint(id)
        nbPts = nbPts-1
        pb= True
        print pb
        print lastIntersection
        if lastIntersection == 0:
            print "fini !"
            break
    else:
        pt = edge_poly.GetPoint(ptIds.GetId(nbPts))
        pid = skeleton_points_final.InsertNextPoint(pt)
        poly_line_final.GetPointIds().SetId(nbPts, pid)
        id = ptIds.GetId(id)
        line[0,nbPts] = pt[0]
        line[1,nbPts] = pt[1]

    print pt
    print id
    print nbPts
    neighborsCellIds = vtk.vtkIdList()
    edge_poly.GetPointCells(id,neighborsCellIds)

    addedNewPt = False
    j = 0
    print neighborsCellIds.GetNumberOfIds()
    #while addedNewPt == False and j<neighborsCellIds.GetNumberOfIds():

    while not addedNewPt and j<neighborsCellIds.GetNumberOfIds():# for j in range (0, neighborsCellIds.GetNumberOfIds()):
        cellId = neighborsCellIds.GetId(j)
        print edge_poly.GetCell(cellId)
        newPtIds = vtk.vtkIdList()
        if edge_poly.GetCell(cellId).GetCellType() != vtk.VTK_EMPTY_CELL:
            edge_poly.GetCellPoints(cellId, newPtIds)
            for k in range(0, newPtIds.GetNumberOfIds()):
                newPtId = newPtIds.GetId(k)
                if ptIds.InsertUniqueId(newPtId) > counter:
                    counter = counter + 1
                    addedNewPt = True
                    lastCellId = cellId
                    break
        j = j + 1
    if not addedNewPt :
        j = 0
        while not addedNewPt and j < neighborsCellIds.GetNumberOfIds():  # for j in range (0, neighborsCellIds.GetNumberOfIds()):
            cellId = neighborsCellIds.GetId(j)
            if cellId!=lastCellId:
                print edge_poly.GetCell(cellId)
                newPtIds = vtk.vtkIdList()
                if edge_poly.GetCell(cellId).GetCellType() != vtk.VTK_EMPTY_CELL:
                    edge_poly.GetCellPoints(cellId, newPtIds)
                    for k in range(0, newPtIds.GetNumberOfIds()):
                        newPtId = newPtIds.GetId(k)
                        if newPtId!=id:
                            lastIntersection = newPtId
                            addedNewPt = True
                            lastCellId = cellId
                            break
            j = j + 1
    nbPts = nbPts + 1

print counter

#tck,u = si.splprep(line,k=3)#, s=10000, nest=10)#, nest=15)
#print tck
#new_points = si.splev(u, tck)
#new_points = tck[1]

# Put found points into a vtkpolydata structure
skeleton_points_final = vtk.vtkPoints() # here we add the points that we find
intersection_points = vtk.vtkPoints()
intersection_cells = vtk.vtkIdList()
whole_line_final = vtk.vtkCellArray()
poly_line_final = vtk.vtkPolyLine()
poly_line_final.GetPointIds().SetNumberOfIds(nbPts)
counter = 0
for i in range (0,nbPts):
    pt = [line[0,i],line[1,i],gravity[2]]
    pid = skeleton_points_final.InsertNextPoint(pt)
    poly_line_final.GetPointIds().SetId(counter, pid)

    counter +=1

poly_line_final.GetPointIds().SetNumberOfIds(counter-1)
whole_line_final.InsertNextCell(poly_line_final)

# Build polydata
skeleton_final = vtk.vtkPolyData()
skeleton_final.SetPoints(skeleton_points_final)
skeleton_final.SetLines(whole_line_final)




# spline = vtk.vtkSplineGraphEdges()
# spline.SetInputData(graph)
# spline.Update()
# back = vtk.vtkGraphToPolyData()
# back.SetInputConnection(spline.GetOutputPort())
# back.Update()

rend = renderer.Renderer()
rend.add_actor(polydata, color=[1,1,1], wireframe= False)
rend.add_actor(skeleton_final, color=[1,0,1], wireframe= False)
rend.render()