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
    if i+1 >= 1:
        for side in ["upper", "lower"]:

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
            polydata,trans,gingiva = meshOp.align_to_axes(polydata,False)
            gingivaGravity = meshOp.compute_center_of_mass(gingiva)
            bounds = gingiva.GetBounds()


            ##############################################################
            ################### Part 2 : "Ray casting" ###################
            ##############################################################

            ### This part throws rays (planes to be exact) from the the center of gravity every degree around the z axis
            ### The planes will intersect with the polydata on some points, and for each plane the point with the highest z is computed
            ### All points with highest z components are returned by findPoints

            center = [gingivaGravity[0], gingivaGravity[1], bounds[5]]
            rayC = raycasting.RayCasting(poly_data=gingiva)
            points = rayC.findPoints(center,True)

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

            # Smooth and weight smoothed with firstTest, then smooth again and find points on mesh
            smth = smoothing.Smoothing()
            rays = smth.weightedCombination(rays, 200, 0.1)  # smoothing + weighting smoothing with firstTest
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