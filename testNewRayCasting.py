import vtk
import meshOperations
import numpy as np
import renderer
import raycasting
import smoothing
import os


base_path = r'./image_data//'

for i, case in enumerate(os.listdir(base_path)):
    print case
    if i+1 >= 1 and i>6:# and i>15:# and i>6:# and i>2:# and i>9:# and i>17:# and i>1:# and i>9:
        for side in ["lower"]:#,"lower",]:

            ##############################################################
            ################## Part 1 : Preprocessing ####################
            ##############################################################

            ### Load data, find principal components and reorient the data according to these principal components axes
            ### The first component will be the x axis and corresponds to the left-right axis
            ### The second component will be the y axis and corresponds to the anteroposterior axis
            ### The last component will be the z axis and corresponds to the craniocauda axis
            ### Then crop the mesh to get rid of the tongue/palatin, to ease the computation

            meshOp = meshOperations.MeshOperations()

            polydata = meshOp.read(base_path + case+ "//{}JawMesh.obj".format(side))
            suggest_line = meshOp.read(base_path + case + "//suggest_alveolarRidgeLine_{}_finalVisualization.obj".format(side))


            # Reorient data
            if side=="lower":
                polydata,transform,gingiva = meshOp.align_to_axes(polydata,False,0.7)
            else:
                polydata, transform, gingiva = meshOp.align_to_axes(polydata, True, 0.7)
            suggest_line, _ = meshOp.transform(suggest_line, transform)



            featureEdges = vtk.vtkFeatureEdges()
            featureEdges.SetInputData(gingiva)  # get basis
            featureEdges.BoundaryEdgesOff()
            featureEdges.FeatureEdgesOff()
            featureEdges.ManifoldEdgesOn()
            featureEdges.NonManifoldEdgesOn()
            featureEdges.Update()


            randomDelaunay = vtk.vtkDelaunay2D()
            randomDelaunay.SetInputConnection(featureEdges.GetOutputPort())
            randomDelaunay.SetAlpha(1)
            randomDelaunay.Update()


            ##############################################################
            ################### Part 2 : "Ray casting" ###################
            ##############################################################

            ### This part throws rays (planes to be exact) from the the center of gravity every degree around the z axis
            ### The planes will intersect with the polydata on some points, and for each plane the point with the highest z is computed
            ### All points with highest z components are returned by findPoints

            dec = vtk.vtkDecimatePro()
            dec.SetInputData(polydata)
            dec.SetTargetReduction(0.001)
            dec.PreserveTopologyOff()
            dec.SplittingOn()
            dec.BoundaryVertexDeletionOn()
            dec.Update()

            ging = meshOp.extractGradient(dec.GetOutput())

            randomPt = ging.GetPoint(0)
            bounds = polydata.GetBounds()

            original_locator = vtk.vtkOBBTree()
            original_locator.SetDataSet(polydata)
            original_locator.BuildLocator()
            intersection_points = vtk.vtkPoints()
            intersection_cells = vtk.vtkIdList()

            original_locator.IntersectWithLine([randomPt[0], randomPt[1], bounds[4]],
                                               [randomPt[0], randomPt[1], bounds[5]],
                                               intersection_points,
                                               intersection_cells)
            if intersection_points.GetNumberOfPoints()>=1:
                print randomPt[2]-intersection_points.GetPoint(0)[2]
                ging,_=meshOp.translate(ging,0,0,intersection_points.GetPoint(0)[2]-randomPt[2])
            else:
                print "oupsi"

            if side=="lower":
                bounds = gingiva.GetBounds()
                gingivaGravity = meshOp.compute_center_of_mass(gingiva)
                center = [gingivaGravity[0], gingivaGravity[1], bounds[5]]


                rayC = raycasting.RayCasting(poly_data=randomDelaunay.GetOutput())
                newX,newY = rayC.findPointsInterp(center)#,ging)
                #print points
            else:
                gingiva = meshOp.extract(polydata)


                bounds = gingiva.GetBounds()
                gingivaGravity = meshOp.compute_center_of_mass(gingiva)
                center = [gingivaGravity[0], gingivaGravity[1], bounds[5]]
                rayC = raycasting.RayCasting(poly_data=gingiva)
                points = rayC.findPointsOld(center)


            ##############################################################
            ################## Part 3 : Postprocessing ###################
            ##############################################################

            ### The smoothed line points are currently not on the mesh (as they are interpolations), we need to project them on the mesh

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
            poly_line_final.GetPointIds().SetNumberOfIds(len(newX))#len(alv_line_points))
            counter = 0
            for i in range (0, len(newX)):#alv_line_point in alv_line_points:
                original_locator.IntersectWithLine([newX[i], newY[i], bounds[4]],
                                                        [newX[i], newY[i], bounds[5]],
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



            ##############################################################
            ################### Part 4 : Visualization ###################
            ##############################################################


            rend = renderer.Renderer()
            rend.add_actor(polydata, color=[1,1,1], wireframe= False)
            rend.add_actor(skeleton_final, color=[1,0,1], wireframe= False)
            rend.add_actor(suggest_line, color=[0,0,1], wireframe= False)
            rend.render()

            appendFilter = vtk.vtkAppendPolyData()
            appendFilter.AddInputData(polydata)
            appendFilter.AddInputData(skeleton_final)
            appendFilter.AddInputData(suggest_line)

            appendFilter.Update()

            writer = vtk.vtkPolyDataWriter()
            writer.SetFileName(base_path + case + "//rayCast_{}_finalVisualization2.vtk".format(side))
            writer.SetInputData(appendFilter.GetOutput())
            writer.Write()