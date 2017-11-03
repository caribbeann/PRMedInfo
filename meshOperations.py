import vtk
import math
import numpy as np

class MeshOperations:

    def read(self, original_mesh):
        """
        Used to read an .obj object by parsing a path
        :param original_mesh: path to the filename
        :return: vtkPolyData of the read object
        """
        reader = vtk.vtkOBJReader()

        reader.SetFileName(original_mesh)

        reader.Update()

        polydata = reader.GetOutput()

        return polydata

    def align_to_axes(self, polydata_input):
        PCADict0 = self.computePCA(polydata_input)
        reorientedPolyData = self.rotate(polydata_input, [0.0, 1.0, 0.0], PCADict0['eigenvectors'][0])

        PCADict1 = self.computePCA(reorientedPolyData)
        reorientedPolyData = self.rotate(reorientedPolyData, [0.0, 1.0, 0.0], PCADict1['eigenvectors'][1])

        PCADict2 = self.computePCA(reorientedPolyData)
        reorientedPolyData = self.rotate(reorientedPolyData, [0.0, 0.0, 1.0], PCADict2['eigenvectors'][2])

        # move also suggestion
        #suggest_line = self.rotate(reference_line, [1.0, 0.0, 0.0], PCADict0['eigenvectors'][0])
        #suggest_line = self.rotate(reference_line, [0.0, 1.0, 0.0], PCADict1['eigenvectors'][1])
        #suggest_line = self.rotate(reference_line, [0.0, 0.0, 1.0], PCADict2['eigenvectors'][2])

        return reorientedPolyData

    def translate_edge_to_XY_plane_center_of_mass_Y_centered(self, input_poly):
        tran1 = [0, -input_poly.GetBounds()[2], -input_poly.GetBounds()[4]]
        mesh = self.translate(input_poly, 0, -input_poly.GetBounds()[2], -input_poly.GetBounds()[4])

        x, y, z = self.computeCenterOfMass(mesh)
        tran2 = [-x, -y, -z]
        mesh = self.translate(mesh, -x, -y, -z)

        tran3 = [0, 0, -mesh.GetBounds()[4]]
        mesh = self.translate(mesh, 0, 0, -mesh.GetBounds()[4])  # move in the XY plane to meet the line

        tran4 = [0, 0, 0]
        mesh = self.translate(mesh, 0, 0, 0)

        tran5 = [0, -mesh.GetBounds()[2], 0]
        mesh = self.translate(mesh, 0, -mesh.GetBounds()[2], 0)

        trans = tran1+tran2+tran3+tran4+tran5

        return mesh, trans

    def move_to_XY_plane(self, polydata_input):
        x, y, _ = self.computeCenterOfMass(polydata_input)
        z = polydata_input.GetBounds()[4]
        reoriented_polydata = self.translate(polydata_input, -x, -y, -z)

        #suggest_line = self.translate(reference_line, -x, -y, -z)  # translate it together with the original line

        return reoriented_polydata

    def translate(self, poly_data_input, x,y,z):
        transform = vtk.vtkTransform()
        transform.Translate(x,y,z)
        transformFilter=vtk.vtkTransformPolyDataFilter()
        transformFilter.SetTransform(transform)
        if vtk.VTK_MAJOR_VERSION <= 5:
            transformFilter.SetInput(poly_data_input)
        else:
            transformFilter.SetInputData(poly_data_input)
        transformFilter.Update()
        return transformFilter.GetOutput()

    def translate_tuple(self, poly_data_input, tuple):
        return self.translate(poly_data_input, tuple[0],tuple[1],tuple[2])

    def rotateAngle(self,poly_data_input,rotAxis,angle):
        transform = vtk.vtkTransform()
        transform.RotateWXYZ(angle,rotAxis)

        transformFilter=vtk.vtkTransformPolyDataFilter()
        transformFilter.SetTransform(transform)
        if vtk.VTK_MAJOR_VERSION <= 5:
            transformFilter.SetInput(poly_data_input)
        else:
            transformFilter.SetInputData(poly_data_input)
        transformFilter.Update()

        return transformFilter.GetOutput()

    def rotate(self, poly_data_input, oldVect, newVect):
        vmath = vtk.vtkMath()
        rotAxis = [0.0,0.0,0.0]
        vmath.Cross(newVect,oldVect,rotAxis)
        theta = vmath.AngleBetweenVectors(oldVect,newVect)
        #print vmath.DegreesFromRadians(theta)

        transform = vtk.vtkTransform()
        transform.RotateWXYZ(vmath.DegreesFromRadians(theta),rotAxis)

        transformFilter=vtk.vtkTransformPolyDataFilter()
        transformFilter.SetTransform(transform)
        if vtk.VTK_MAJOR_VERSION <= 5:
            transformFilter.SetInput(poly_data_input)
        else:
            transformFilter.SetInputData(poly_data_input)
        transformFilter.Update()

        return transformFilter.GetOutput()

    def scale(self,poly_data_input, x,y,z):
        transform = vtk.vtkTransform()
        transform.Scale(x, y, z)
        transformFilter = vtk.vtkTransformPolyDataFilter()
        transformFilter.SetTransform(transform)
        if vtk.VTK_MAJOR_VERSION <= 5:
            transformFilter.SetInput(poly_data_input)
        else:
            transformFilter.SetInputData(poly_data_input)
        transformFilter.Update()
        return transformFilter.GetOutput()

    def move_to_origin(self, poly_data_input):
        x, y, z = self.computeCenterOfMass(poly_data_input)
        moved_poly_data = self.translate(poly_data_input, -x, -y, -z)
        return moved_poly_data

    def computePCA(self, poly_data_input):
        xArray = vtk.vtkDoubleArray()
        xArray.SetNumberOfComponents(1)
        xArray.SetName('x')
        yArray = vtk.vtkDoubleArray()
        yArray.SetNumberOfComponents(1)
        yArray.SetName('y')
        zArray = vtk.vtkDoubleArray()
        zArray.SetNumberOfComponents(1)
        zArray.SetName('z')

        for i in range (0,poly_data_input.GetNumberOfPoints()):
            pt = poly_data_input.GetPoint(i)
            xArray.InsertNextValue(pt[0])
            yArray.InsertNextValue(pt[1])
            zArray.InsertNextValue(pt[2])

        table = vtk.vtkTable()
        table.AddColumn(xArray)
        table.AddColumn(yArray)
        table.AddColumn(zArray)    

        pcaStats = vtk.vtkPCAStatistics()

        if vtk.VTK_MAJOR_VERSION <= 5:
            pcaStats.SetInput(table)

        else:
            pcaStats.SetInputData(table)

        pcaStats.SetColumnStatus("x",1)
        pcaStats.SetColumnStatus("y",1)
        pcaStats.SetColumnStatus("z",1)

        pcaStats.RequestSelectedColumns()
        pcaStats.SetDeriveOption(True)
        pcaStats.Update()

        eigenvalues = vtk.vtkDoubleArray()
        pcaStats.GetEigenvalues(eigenvalues)
        eigenvector0 = vtk.vtkDoubleArray()
        pcaStats.GetEigenvector(0,eigenvector0)
        eigenvector1 = vtk.vtkDoubleArray()
        pcaStats.GetEigenvector(1,eigenvector1)
        eigenvector2 = vtk.vtkDoubleArray()
        pcaStats.GetEigenvector(2,eigenvector2)

        eigv0 = [0.0,0.0,0.0]
        eigv1 = [0.0,0.0,0.0]
        eigv2 = [0.0,0.0,0.0]

        for i in range (0,3):
            eigv0[i] = eigenvector0.GetValue(i)
            eigv1[i] = eigenvector1.GetValue(i)
            eigv2[i] = eigenvector2.GetValue(i)


        return {'eigenvalues':eigenvalues,'eigenvectors':[eigv0,eigv1,eigv2]}


    def computeCenterOfMass(self, poly_data_input):
        centerOfMassFilter = vtk.vtkCenterOfMass()
        if vtk.VTK_MAJOR_VERSION<=5:
            centerOfMassFilter.SetInput(poly_data_input)
        else:
            centerOfMassFilter.SetInputData(poly_data_input)
        centerOfMassFilter.SetUseScalarsAsWeights(False)
        centerOfMassFilter.Update()

        return centerOfMassFilter.GetCenter()#(center)

    # crop the mesh, keep what is inside the box
    def cropMesh(self,poly_data_input, xmin,xmax,ymin,ymax,zmin,zmax):



        box = vtk.vtkBox()
        box.SetBounds(xmin,xmax,ymin,ymax,zmin,zmax)


        pdNormals = vtk.vtkPolyDataNormals()
        pdNormals.SetInputData(poly_data_input)

        clipper = vtk.vtkClipPolyData()
        clipper.SetInputConnection(pdNormals.GetOutputPort())
        clipper.SetClipFunction(box)
        clipper.GenerateClippedOutputOn()
        clipper.InsideOutOff()
        clipper.Update()
        return clipper.GetClippedOutput()


    def crop_mesh_maximum(self, poly_input, xmin,xmax,ymin,ymax,zmin,zmax, min_percent, max_percent, percent_step):
        con_filter = vtk.vtkPolyDataConnectivityFilter()

        last_biggest_percentage = 0
        last_crop_poly = poly_input
        for i in np.arange(min_percent, max_percent, percent_step):
            cropped_poly_data = self.cropMesh(poly_input,
                                            xmin, xmax,
                                            ymin, ymax,
                                            i * zmin, zmax)
            con_filter.SetInputData(cropped_poly_data)
            con_filter.SetExtractionModeToAllRegions()
            con_filter.Update()
            if con_filter.GetNumberOfExtractedRegions() == 1:
                if i > last_biggest_percentage:
                    last_biggest_percentage = i
                    last_crop_poly = cropped_poly_data

        return last_crop_poly
    def meshTo3DImg(self, poly_data_input):
        whiteImage = vtk.vtkImageData()
        bounds = poly_data_input.GetBounds()
        spacing = [0.5,0.5,0.5]
        whiteImage.SetSpacing(spacing)
        dim = [0,0,0]
        for i in range (0,3):
            dim[i] = int(math.ceil(bounds[i*2+1]-bounds[i*2])/spacing[i])
            print dim[i]
        whiteImage.SetDimensions(dim)
        whiteImage.SetExtent(0,dim[0]-1,0,dim[1]-1,0,dim[2]-1)
        origin = [bounds[0]+spacing[0]/2,bounds[2]+spacing[1]/2,bounds[4]+spacing[2]/2]
        whiteImage.SetOrigin(origin)

        if vtk.VTK_MAJOR_VERSION <= 5:
            whiteImage.SetScalarTypeToUnsignedChar()
            whiteImage.AllocateScalars()
        else:
            whiteImage.AllocateScalars(vtk.VTK_UNSIGNED_CHAR,1)

        inval = 255
        outval = 0
        for i in range (0, whiteImage.GetNumberOfPoints()):
            whiteImage.GetPointData().GetScalars().SetTuple1(i,inval)

        pol2stenc = vtk.vtkPolyDataToImageStencil()
        if vtk.VTK_MAJOR_VERSION <= 5:
            pol2stenc.SetInput(poly_data_input)
        else:
            pol2stenc.SetInputData(poly_data_input)

        pol2stenc.SetOutputOrigin(origin)
        pol2stenc.SetOutputSpacing(spacing)
        pol2stenc.SetOutputWholeExtent(whiteImage.GetExtent())
        pol2stenc.Update()

        imgstenc = vtk.vtkImageStencil()
        if vtk.VTK_MAJOR_VERSION <= 5:
            imgstenc.SetInput(whiteImage)
            imgstenc.SetStencil(pol2stenc.GetOutput())
        else:
            imgstenc.SetInputData(whiteImage)
            imgstenc.SetStencilConnection(pol2stenc.GetOutputPort())

        imgstenc.ReverseStencilOff()
        imgstenc.SetBackgroundValue(outval)
        imgstenc.Update()
        return imgstenc.GetOutput()

    def nbPointsInsideBox(self,poly_data_input,xmin,xmax,ymin,ymax,zmin,zmax):
        box = vtk.vtkCubeSource()
        box.SetBounds(xmin,xmax,ymin,ymax,zmin,zmax)
        box.Update()

        selectEnclosedPoints = vtk.vtkSelectEnclosedPoints()
        if vtk.VTK_MAJOR_VERSION <= 5:
            selectEnclosedPoints.SetInput(poly_data_input)
            selectEnclosedPoints.SetSurface(box.GetOutput())
        else:
            selectEnclosedPoints.SetInputData(poly_data_input)
            selectEnclosedPoints.SetSurfaceData(box.GetOutput())

        selectEnclosedPoints.Update()

        nbOfPointsInside = 0
        for i in range (0, poly_data_input.GetNumberOfPoints()):
            if selectEnclosedPoints.IsInside(i):
                nbOfPointsInside = nbOfPointsInside + 1

        return nbOfPointsInside 

 

    def extractEdges(self, poly_data_input):
        vextractEdges = vtk.vtkFeatureEdges()
        vextractEdges.FeatureEdgesOn()
        vextractEdges.BoundaryEdgesOff()
        vextractEdges.ColoringOn()
        vextractEdges.SetInputData(poly_data_input)
        vextractEdges.Update()    
        return vextractEdges.GetOutput()

    def get_outer_edges(self, input_poly):
        featureEdges = vtk.vtkFeatureEdges()

        featureEdges.SetInputData(input_poly)
        featureEdges.BoundaryEdgesOn()
        featureEdges.FeatureEdgesOff()
        featureEdges.ManifoldEdgesOff()
        featureEdges.NonManifoldEdgesOff()
        featureEdges.Update()
        edge_poly = featureEdges.GetOutput()

        return edge_poly

    def get_edge_skeleton(self, input_poly):
        # get intersection of rays sent from the center of mass outwards
        edge_locator = vtk.vtkCellLocator()
        edge_locator.SetDataSet(input_poly)
        edge_locator.BuildLocator()

        cells_edges = vtk.vtkIdList()

        line_length = abs(input_poly.GetBounds()[3]) + abs(input_poly.GetBounds()[1])
        start_point = [0, 0, 0]
        end_points = []
        for angle in np.linspace(math.pi / 6, math.pi * 5 / 6, 120):  # from 30deg to 150deg in 1 deg steps
            end_points.append(
                [start_point[0] + line_length * math.cos(angle), start_point[1] + line_length * math.sin(angle),
                 start_point[2]])

        end_points = np.array(end_points)

        skeleton_points = vtk.vtkPoints()  # here we add the points that we find
        whole_line = vtk.vtkCellArray()
        poly_line = vtk.vtkPolyLine()

        alv_line_points = []
        poly_line.GetPointIds().SetNumberOfIds(end_points.shape[0])
        counter = 0
        for i in range(end_points.shape[0]):

            nr = edge_locator.FindCellsAlongLine(start_point, end_points[i], 0.001, cells_edges)
            pid1 = vtk.vtkIdList()  # contains the point ids of the first intersection
            pid2 = vtk.vtkIdList()
            if cells_edges.GetNumberOfIds() == 2:
                # get only the first two intersection points
                id1 = cells_edges.GetId(0)
                id2 = cells_edges.GetId(1)
                input_poly.GetCellPoints(id1, pid1)
                input_poly.GetCellPoints(id2, pid2)

                p_int_1 = np.add(np.array(input_poly.GetPoint(pid1.GetId(0))),
                                 np.array(input_poly.GetPoint(pid1.GetId(1)))) / 2
                p_int_2 = np.add(np.array(input_poly.GetPoint(pid2.GetId(0))),
                                 np.array(input_poly.GetPoint(pid2.GetId(1)))) / 2

                p_alv_line = np.add(p_int_1, p_int_2) / 2
                alv_line_points.append(p_alv_line)
                skeleton_points.InsertNextPoint(p_alv_line)

                poly_line.GetPointIds().SetId(counter, counter)
                counter += 1
        poly_line.GetPointIds().SetNumberOfIds(counter - 1)
        whole_line.InsertNextCell(poly_line)
        skeleton = vtk.vtkPolyData()
        skeleton.SetPoints(skeleton_points)
        skeleton.SetLines(whole_line)
        return skeleton, alv_line_points

    def place_skeleton_on_original_mesh(self, original_poly, skeleton_points):

        original_locator = vtk.vtkOBBTree()
        original_locator.SetDataSet(original_poly)
        original_locator.BuildLocator()

        cells_original = vtk.vtkIdList()

        line_length = abs(original_poly.GetBounds()[5]) + 1

        skeleton_points_final = vtk.vtkPoints()  # here we add the points that we find
        intersection_points = vtk.vtkPoints()
        intersection_cells = vtk.vtkIdList()
        whole_line_final = vtk.vtkCellArray()
        poly_line_final = vtk.vtkPolyLine()
        poly_line_final.GetPointIds().SetNumberOfIds(len(skeleton_points))
        counter = 0
        for alv_line_point in skeleton_points:
            nr = original_locator.IntersectWithLine(alv_line_point,
                                                    [alv_line_point[0], alv_line_point[1],
                                                     alv_line_point[2] + line_length],
                                                    intersection_points,
                                                    intersection_cells)

            if intersection_points.GetNumberOfPoints() >= 1:
                p_int = intersection_points.GetPoint(intersection_points.GetNumberOfPoints() - 1)

                pid = skeleton_points_final.InsertNextPoint(p_int)
                poly_line_final.GetPointIds().SetId(counter, pid)

                counter += 1

        poly_line_final.GetPointIds().SetNumberOfIds(counter - 1)
        whole_line_final.InsertNextCell(poly_line_final)

        skeleton_final = vtk.vtkPolyData()
        skeleton_final.SetPoints(skeleton_points_final)
        skeleton_final.SetLines(whole_line_final)

        return skeleton_final