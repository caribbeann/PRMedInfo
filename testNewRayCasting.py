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
    if i+1 >= 1:# and i>17:# and i>1:# and i>9:
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

            rend = renderer.Renderer()
            rend.add_actor(randomDelaunay.GetOutput(), color=[1, 1, 1], wireframe=False)
            rend.render()

            if side=="lower":
                bounds = gingiva.GetBounds()
                gingivaGravity = meshOp.compute_center_of_mass(gingiva)
                center = [gingivaGravity[0], gingivaGravity[1], bounds[5]]


                rayC = raycasting.RayCasting(poly_data=randomDelaunay.GetOutput())
                points = rayC.findPoints(center,ging)
            else:
                gingiva = meshOp.extract(polydata)

                #ging = meshOp.remove_tongue(gingiva, 0.02)
                #test, transform, gingiva = meshOp.align_to_axes(ging, False, 0.7)

                # rend = renderer.Renderer()
                # rend.add_actor(test, color=[0, 0, 1], wireframe=False)
                # rend.render()

                bounds = gingiva.GetBounds()
                gingivaGravity = meshOp.compute_center_of_mass(gingiva)
                center = [gingivaGravity[0], gingivaGravity[1], bounds[5]]
                rayC = raycasting.RayCasting(poly_data=gingiva)
                points = rayC.findPointsOld(center)



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
            writer.SetFileName(base_path + case + "//rayCast_{}_finalVisualization.vtk".format(side))
            writer.SetInputData(rays)
            writer.Write()


            rend = renderer.Renderer()
            rend.add_actor(polydata, color=[1, 1, 1], wireframe=False)
            rend.add_actor(gingiva, color=[0, 0, 1], wireframe=False)
            rend.add_actor(rays, color=[1, 0, 1], wireframe=False)
            #rend.add_actor(ging, color=[0, 1, 0], wireframe=False)
            rend.render()

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
            rays = smth.weightedCombination(rays, 200,0.1)  # smoothing + weighting smoothing with firstTest
            alv_line_points = smth.smooth(rays, 50, 0.1)  # smooth again (but less) => points are NOT on the mesh but have right curve


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
            poly_line_final.GetPointIds().SetNumberOfIds(len(alv_line_points))
            counter = 0
            for alv_line_point in alv_line_points:
                original_locator.IntersectWithLine([alv_line_point[0], alv_line_point[1], bounds[4]],
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


            ## Deal when missing part : approximate

            gravity = meshOp.compute_center_of_mass(skeleton_final)
            bounds = skeleton_final.GetBounds()

            left = meshOp.crop_mesh(skeleton_final, bounds[0], 0, bounds[2], bounds[3], bounds[4],
                                   bounds[5])
            right = meshOp.crop_mesh(skeleton_final, 0, bounds[1], bounds[2], bounds[3], bounds[4],
                                  bounds[5])
            rightBounds = right.GetBounds()
            leftBounds = left.GetBounds()
            rightRange = rightBounds[3] - rightBounds[2]
            leftRange = leftBounds[3] - leftBounds[2]
            print leftRange, rightRange, leftBounds, rightBounds

            if leftRange<0.8*rightRange or rightRange<0.8*leftRange:
                # line = np.zeros(shape=(2,len(alv_line_points)))
                # idxLeftRight = -1
                #
                # for i in range (0,len(alv_line_points)):
                #     pt = alv_line_points[i]
                #     line[0,i] = pt[0]
                #     line[1,i] = pt[1]
                #     if pt[0]>0 and idxLeftRight==-1:
                #         idxLeftRight = i
                #         print idxLeftRight

                distances = np.zeros(shape=(10,3))

                if rightRange<0.8*leftRange:
                    for i in range (0,10):
                        distances[i]=alv_line_points[len(alv_line_points)-1-i]- alv_line_points[len(alv_line_points)-2-i]
                    distances = np.mean(distances,0)
                    number =abs(int(0.75*(leftRange-rightRange)/distances[1]))
                    print number
                    xnew = np.zeros(number)
                    ynew = np.zeros(number)
                    #fitting_parameters, covariance = curve_fit = np.zeros(20)
                    xnew[0] = alv_line_points[len(alv_line_points)-1][0]+distances[0]
                    ynew[0]=alv_line_points[len(alv_line_points)-1][1]+distances[1]
                    for i in range (1,number):
                        xnew[i] = xnew[i-1]+distances[0]-2*distances[0]*distances[0]
                        ynew[i] = ynew[i-1]+distances[1]#+2*distances[1]*distances[1]
                        z = alv_line_points[len(alv_line_points)-1][2]
                    print ynew[number-1]
                else:
                    for i in range (0,10):
                        distances[i]=alv_line_points[i]- alv_line_points[i+1]
                    distances = np.mean(distances,0)
                    number =abs(int(0.75*(rightRange-leftRange)/distances[1]))
                    xnew = np.zeros(number)
                    ynew = np.zeros(number)
                    #fitting_parameters, covariance = curve_fit = np.zeros(20)
                    xnew[0] = alv_line_points[0][0]+distances[0]
                    ynew[0]=alv_line_points[0][1]+distances[1]
                    for i in range (1,number):
                        xnew[i] = xnew[i-1]+distances[0]-2*distances[0]*distances[0]
                        ynew[i] = ynew[i-1]+distances[1]#+2*distances[1]*distances[1]
                        z=alv_line_points[0][2]
                    #fitting_parameters, covariance = curve_fit(f, line[1, 0:idxLeftRight],line[0, 0:idxLeftRight])

                #ynew = np.linspace(max(leftBounds[2], rightBounds[2]), min(leftBounds[2], rightBounds[2]), num=number)

                poly_line_final2 = vtk.vtkPolyLine()
                poly_line_final2.GetPointIds().SetNumberOfIds(len(alv_line_points) + number)

                #TODO


                for i in range (0, len(alv_line_points)):
                    poly_line_final2.GetPointIds().SetId(i,poly_line_final.GetPointIds().GetId(i))
                counter = len(alv_line_points)
                for i in range (0,number):
                    pt =[xnew[i],ynew[i],z]
                    pid = skeleton_points_final.InsertNextPoint(pt)
                    poly_line_final2.GetPointIds().SetId(counter, pid)
                    counter = counter + 1

                poly_line_final2.GetPointIds().SetNumberOfIds(counter-1)
                whole_line_final = vtk.vtkCellArray()
                whole_line_final.InsertNextCell(poly_line_final2)

                # Build polydata
                skeleton_final2 = vtk.vtkPolyData()
                skeleton_final2.SetPoints(skeleton_points_final)
                skeleton_final2.SetLines(whole_line_final)

                # import scipy.interpolate as si
                #
                # if rightRange<0.8*leftRange:
                #     print "right",rightBounds[0]-5,rightBounds[0]+5
                #     f = si.interp1d(line[1,idxLeftRight:],line[0,idxLeftRight:],kind='linear',fill_value=(rightBounds[1]-2,rightBounds[1]+2),bounds_error=False)
                # else:
                #     print "left",leftBounds[1]-5,leftBounds[1]+5
                #     f = si.interp1d(line[1,0:idxLeftRight],line[0,0:idxLeftRight],kind='linear',fill_value=(leftBounds[0]-2,leftBounds[0]+2),bounds_error=False)
                #
                # xnew = f(ynew)

                # Smooth and weight smoothed with firstTest, then smooth again and find points on mesh
                rend = renderer.Renderer()
                rend.add_actor(polydata, color=[1, 1, 1], wireframe=False)
                rend.add_actor(skeleton_final2, color=[0, 1, 0], wireframe=False)
                rend.render()

                smth = smoothing.Smoothing()
                rays = smth.weightedCombination(skeleton_final2, 200,
                                                0.1)  # 200, 0.1)  # smoothing + weighting smoothing with firstTest
                alv_line_points = smth.smooth(skeleton_final2, 50,
                                              0.1)  # smooth again (but less) => points are NOT on the mesh but have right curve

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
                poly_line_final.GetPointIds().SetNumberOfIds(len(alv_line_points))
                counter = 0
                for alv_line_point in alv_line_points:
                    original_locator.IntersectWithLine([alv_line_point[0], alv_line_point[1], bounds[4]],
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