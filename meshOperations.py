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
        """
        Align the axes of the scene with the main axes of the object by PCA
        :param polydata_input: mesh to be aligned
        :return: aligned vtkPolyData
        """
        # pca_dict0 = self.compute_pca(polydata_input)
        # reoriented_poly_data = self.rotate(polydata_input, [0.0, 1.0, 0.0], pca_dict0['eigenvectors'][0])

        tmp_transform = vtk.vtkTransform()
        tmp_transform.PostMultiply()

        pca_dict1 = self.compute_pca(polydata_input)
        reoriented_poly_data, trans = self.rotate(polydata_input, [0.0, 1.0, 0.0], pca_dict1['eigenvectors'][1])
        tmp_transform.Concatenate(trans)
        reoriented_poly_data, trans = self.translate_to_origin(reoriented_poly_data)
        tmp_transform.Concatenate(trans)

        p_c_a_dict2 = self.compute_pca(reoriented_poly_data)
        reoriented_poly_data, trans = self.rotate(reoriented_poly_data, [0.0, 0.0, 1.0], p_c_a_dict2['eigenvectors'][2])
        tmp_transform.Concatenate(trans)
        reoriented_poly_data, trans = self.translate_to_origin(reoriented_poly_data)
        tmp_transform.Concatenate(trans)

        print(tmp_transform.GetNumberOfConcatenatedTransforms())

        return reoriented_poly_data, tmp_transform

    def translate_to_xy_y_centered(self, input_poly):
        """
        Translates a mesh in the XY plane (Zmin=0) in the Y positive half and Y axis passing through the center of mass
        :param input_poly: vtkPolyData to be translated
        :return: translated vtkPolyData and the transformation vector used to translate
        """
        tran1 = [0, -input_poly.GetBounds()[2], -input_poly.GetBounds()[4]]
        mesh, trans = self.translate(input_poly, 0, -input_poly.GetBounds()[2], -input_poly.GetBounds()[4])

        x, y, z = self.compute_center_of_mass(mesh)
        tran2 = [-x, -y, -z]
        mesh, trans = self.translate(mesh, -x, -y, -z)

        tran3 = [0, 0, -mesh.GetBounds()[4]]
        mesh, trans = self.translate(mesh, 0, 0, -mesh.GetBounds()[4])

        tran4 = [0, 0, 0]
        mesh, trans = self.translate(mesh, 0, 0, 0)

        tran5 = [0, -mesh.GetBounds()[2], 0]
        mesh, trans = self.translate(mesh, 0, -mesh.GetBounds()[2], 0)

        trans = tran1+tran2+tran3+tran4+tran5

        return mesh, trans

    def translate_to_xy_plane(self, polydata_input):
        """
        Translates a mesh to the XY plane (Zmin = 0)
        :param polydata_input: vtkPolyData to be moved
        :return: translated vtkPolyData
        """
        x, y, _ = self.compute_center_of_mass(polydata_input)
        z = polydata_input.GetBounds()[4]
        reoriented_polydata, trans = self.translate(polydata_input, -x, -y, -z)
        return reoriented_polydata, trans

    def translate(self, poly_data_input, x, y, z):
        """
        Translates a mesh using the given transformation vector
        :param poly_data_input: vtkPolyData to be translated
        :param x: x change
        :param y: y change
        :param z: z change
        :return: translated vtkPolyData and vtkTransform used
        """
        transform = vtk.vtkTransform()
        transform.Translate(x, y, z)
        transform_filter = vtk.vtkTransformPolyDataFilter()
        transform_filter.SetTransform(transform)
        if vtk.VTK_MAJOR_VERSION <= 5:
            transform_filter.SetInput(poly_data_input)
        else:
            transform_filter.SetInputData(poly_data_input)
        transform_filter.Update()

        return transform_filter.GetOutput(), transform

    def transform(self, poly_data_input, transform):
        """
        Transforms (rotates, scales or translates a mesh with a given transform)
        :param poly_data_input: vtkPolyData to be transformed
        :param transform: vtkTransform to be applied
        :return: transformed vtkPolyData and vtkTransform used
        """
        local_transform = vtk.vtkTransform()

        if transform.GetOrientationWXYZ()[0] != 0:
            tmp = transform.GetOrientationWXYZ()
            local_transform.RotateWXYZ(tmp[0], tmp[1:])
        if transform.GetScale() != [1.0, 1.0, 1.0]:
            local_transform.Scale(transform.GetScale())
        if transform.GetPosition() != [0.0, 0.0, 0.0]:
            local_transform.Translate(transform.GetPosition())

        transform_filter = vtk.vtkTransformPolyDataFilter()
        transform_filter.SetTransform(transform)
        if vtk.VTK_MAJOR_VERSION <= 5:
            transform_filter.SetInput(poly_data_input)
        else:
            transform_filter.SetInputData(poly_data_input)
        transform_filter.Update()

        return transform_filter.GetOutput(), local_transform



    def translate_tuple(self, poly_data_input, t):
        """
        Translates a given mesh by the given vector
        :param poly_data_input: vtkPolyData to be translated
        :param t: vector tuple containing the transformation
        :return: translated vtkPolyData
        """
        return self.translate(poly_data_input, t[0], t[1], t[2])

    def rotate_angle(self, poly_data_input, rot_axis, angle):
        """
        Rotate a poly data around a specific axis by a specific angle
        :param poly_data_input: vtkPolyData to be rotated
        :param rot_axis: rotation axis as a vector
        :param angle: rotation angle
        :return: rotated vtkPolyData
        """
        transform = vtk.vtkTransform()
        transform.RotateWXYZ(angle, rot_axis)

        transform_filter = vtk.vtkTransformPolyDataFilter()
        transform_filter.SetTransform(transform)
        if vtk.VTK_MAJOR_VERSION <= 5:
            transform_filter.SetInput(poly_data_input)
        else:
            transform_filter.SetInputData(poly_data_input)
        transform_filter.Update()

        return transform_filter.GetOutput()

    def rotate(self, poly_data_input, old_vect, new_vect):
        """
        Rotates a mesh's axis to match the orientation of another vector
        :param poly_data_input: vtkPolyData to be rotated
        :param old_vect: original mesh axis
        :param new_vect: new position of the mesh axis
        :return: rotated vtkPolyData
        """
        vmath = vtk.vtkMath()
        rot_axis = [0.0, 0.0, 0.0]
        vmath.Cross(new_vect, old_vect, rot_axis)
        theta = vmath.AngleBetweenVectors(old_vect, new_vect)

        transform = vtk.vtkTransform()
        transform.RotateWXYZ(vmath.DegreesFromRadians(theta), rot_axis)

        transform_filter = vtk.vtkTransformPolyDataFilter()
        transform_filter.SetTransform(transform)
        if vtk.VTK_MAJOR_VERSION <= 5:
            transform_filter.SetInput(poly_data_input)
        else:
            transform_filter.SetInputData(poly_data_input)
        transform_filter.Update()

        return transform_filter.GetOutput(), transform

    def scale(self, poly_data_input, x, y, z):
        """
        Scale a mesh with given transformation vector
        :param poly_data_input: vtkPolyData to be scaled
        :param x: x change
        :param y: y change
        :param z: z change
        :return:
        """
        transform = vtk.vtkTransform()
        transform.Scale(x, y, z)
        transform_filter = vtk.vtkTransformPolyDataFilter()
        transform_filter.SetTransform(transform)
        if vtk.VTK_MAJOR_VERSION <= 5:
            transform_filter.SetInput(poly_data_input)
        else:
            transform_filter.SetInputData(poly_data_input)
        transform_filter.Update()
        return transform_filter.GetOutput()

    def translate_to_origin(self, poly_data_input):
        """
        Moves a mesh with its center of mass to the origin of the coordinate system
        :param poly_data_input: vtkPolyData to be moved
        :return: translated vtkPolyData and vtkTransform used
        """
        x, y, z = self.compute_center_of_mass(poly_data_input)
        moved_poly_data, transform = self.translate(poly_data_input, -x, -y, -z)
        return moved_poly_data, transform

    def compute_pca(self, poly_data_input):
        """
        Computes Principal Component Analysis of a mesh
        :param poly_data_input: compute PCA of this vtkPolyData
        :return: eigenvalues, eigenvectors
        """
        x_array = vtk.vtkDoubleArray()
        x_array.SetNumberOfComponents(1)
        x_array.SetName('x')
        y_array = vtk.vtkDoubleArray()
        y_array.SetNumberOfComponents(1)
        y_array.SetName('y')
        z_array = vtk.vtkDoubleArray()
        z_array.SetNumberOfComponents(1)
        z_array.SetName('z')

        for i in range(0, poly_data_input.GetNumberOfPoints()):
            pt = poly_data_input.GetPoint(i)
            x_array.InsertNextValue(pt[0])
            y_array.InsertNextValue(pt[1])
            z_array.InsertNextValue(pt[2])

        table = vtk.vtkTable()
        table.AddColumn(x_array)
        table.AddColumn(y_array)
        table.AddColumn(z_array)

        pca_stats = vtk.vtkPCAStatistics()

        if vtk.VTK_MAJOR_VERSION <= 5:
            pca_stats.SetInput(table)

        else:
            pca_stats.SetInputData(table)

        pca_stats.SetColumnStatus("x", 1)
        pca_stats.SetColumnStatus("y", 1)
        pca_stats.SetColumnStatus("z", 1)

        pca_stats.RequestSelectedColumns()
        pca_stats.SetDeriveOption(True)
        pca_stats.Update()

        eigenvalues = vtk.vtkDoubleArray()
        pca_stats.GetEigenvalues(eigenvalues)
        eigenvector0 = vtk.vtkDoubleArray()
        pca_stats.GetEigenvector(0, eigenvector0)
        eigenvector1 = vtk.vtkDoubleArray()
        pca_stats.GetEigenvector(1, eigenvector1)
        eigenvector2 = vtk.vtkDoubleArray()
        pca_stats.GetEigenvector(2, eigenvector2)

        eigv0 = [0.0, 0.0, 0.0]
        eigv1 = [0.0, 0.0, 0.0]
        eigv2 = [0.0, 0.0, 0.0]

        for i in range(0, 3):
            eigv0[i] = eigenvector0.GetValue(i)
            eigv1[i] = eigenvector1.GetValue(i)
            eigv2[i] = eigenvector2.GetValue(i)

        return {'eigenvalues': eigenvalues, 'eigenvectors': [eigv0, eigv1, eigv2]}

    def compute_center_of_mass(self, poly_data_input):
        """
        Computes the coordinates of the center of mass of a mesh
        :param poly_data_input: input vtkPolyData
        :return: center of mass coordinates
        """
        center_of_mass_filter = vtk.vtkCenterOfMass()
        if vtk.VTK_MAJOR_VERSION <= 5:
            center_of_mass_filter.SetInput(poly_data_input)
        else:
            center_of_mass_filter.SetInputData(poly_data_input)
        center_of_mass_filter.SetUseScalarsAsWeights(False)
        center_of_mass_filter.Update()

        return center_of_mass_filter.GetCenter()

    def crop_mesh(self, poly_data_input, xmin, xmax, ymin, ymax, zmin, zmax):
        """
        Crop part of the mesh specified
        :param poly_data_input: vtkPolyData to be cropped
        :param xmin:
        :param xmax:
        :param ymin:
        :param ymax:
        :param zmin:
        :param zmax:
        :return: cropped vtkPolyData
        """
        box = vtk.vtkBox()
        box.SetBounds(xmin, xmax, ymin, ymax, zmin, zmax)

        pd_normals = vtk.vtkPolyDataNormals()
        pd_normals.SetInputData(poly_data_input)

        clipper = vtk.vtkClipPolyData()
        clipper.SetInputConnection(pd_normals.GetOutputPort())
        clipper.SetClipFunction(box)
        clipper.GenerateClippedOutputOn()
        clipper.InsideOutOff()
        clipper.Update()
        return clipper.GetClippedOutput()

    def crop_mesh_maximum(self, poly_input, xmin, xmax, ymin, ymax, zmin, zmax, min_percent, max_percent, percent_step):
        """
        Crop a given mesh but with constraints on the maximum crop. The percentages represent parts of total Z height
        of the mesh
        :param poly_input:
        :param xmin:
        :param xmax:
        :param ymin:
        :param ymax:
        :param zmin:
        :param zmax:
        :param min_percent: below this amount, the mesh will not be kept
        :param max_percent: above this amount, cropping is not allowed anymore
        :param percent_step:
        :return:
        """
        con_filter = vtk.vtkPolyDataConnectivityFilter()

        last_biggest_percentage = 0
        last_crop_poly = poly_input
        for i in np.arange(min_percent, max_percent, percent_step):
            cropped_poly_data = self.crop_mesh(poly_input,
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

    def mesh_to_3d_img(self, poly_data_input):
        """
        transform a mesh to a 3D image
        :param poly_data_input: vtkPolyData to be transformed
        :return: vtkImageStencil of the mesh
        """
        white_image = vtk.vtkImageData()
        bounds = poly_data_input.GetBounds()
        spacing = [0.5, 0.5, 0.5]
        white_image.SetSpacing(spacing)
        dim = [0, 0, 0]
        for i in range(0, 3):
            dim[i] = int(math.ceil(bounds[i*2+1]-bounds[i*2])/spacing[i])
            print dim[i]
        white_image.SetDimensions(dim)
        white_image.SetExtent(0, dim[0]-1, 0, dim[1]-1, 0, dim[2]-1)
        origin = [bounds[0]+spacing[0]/2, bounds[2]+spacing[1]/2, bounds[4]+spacing[2]/2]
        white_image.SetOrigin(origin)

        if vtk.VTK_MAJOR_VERSION <= 5:
            white_image.SetScalarTypeToUnsignedChar()
            white_image.AllocateScalars()
        else:
            white_image.AllocateScalars(vtk.VTK_UNSIGNED_CHAR, 1)

        inval = 255
        outval = 0
        for i in range(0, white_image.GetNumberOfPoints()):
            white_image.GetPointData().GetScalars().SetTuple1(i, inval)

        pol2stenc = vtk.vtkPolyDataToImageStencil()
        if vtk.VTK_MAJOR_VERSION <= 5:
            pol2stenc.SetInput(poly_data_input)
        else:
            pol2stenc.SetInputData(poly_data_input)

        pol2stenc.SetOutputOrigin(origin)
        pol2stenc.SetOutputSpacing(spacing)
        pol2stenc.SetOutputWholeExtent(white_image.GetExtent())
        pol2stenc.Update()

        imgstenc = vtk.vtkImageStencil()
        if vtk.VTK_MAJOR_VERSION <= 5:
            imgstenc.SetInput(white_image)
            imgstenc.SetStencil(pol2stenc.GetOutput())
        else:
            imgstenc.SetInputData(white_image)
            imgstenc.SetStencilConnection(pol2stenc.GetOutputPort())

        imgstenc.ReverseStencilOff()
        imgstenc.SetBackgroundValue(outval)
        imgstenc.Update()
        return imgstenc.GetOutput()

    def nr_points_inside_box(self, poly_data_input, xmin, xmax, ymin, ymax, zmin, zmax):
        """
        Returns the number of points a mesh has inside a box with specified bounds
        :param poly_data_input: vtkPolyData to be analyzed
        :param xmin: bounds of the box
        :param xmax:
        :param ymin:
        :param ymax:
        :param zmin:
        :param zmax:
        :return: number of points inside the specified box
        """
        box = vtk.vtkCubeSource()
        box.SetBounds(xmin, xmax, ymin, ymax, zmin, zmax)
        box.Update()

        select_enclosed_points = vtk.vtkSelectEnclosedPoints()
        if vtk.VTK_MAJOR_VERSION <= 5:
            select_enclosed_points.SetInput(poly_data_input)
            select_enclosed_points.SetSurface(box.GetOutput())
        else:
            select_enclosed_points.SetInputData(poly_data_input)
            select_enclosed_points.SetSurfaceData(box.GetOutput())

        select_enclosed_points.Update()

        nb_of_points_inside = 0
        for i in range(0, poly_data_input.GetNumberOfPoints()):
            if select_enclosed_points.IsInside(i):
                nb_of_points_inside = nb_of_points_inside + 1

        return nb_of_points_inside

    def extract_edges(self, poly_data_input):
        """
        Extracts the edges of a mesh
        :param poly_data_input: vtkPolyData to be analyzed
        :return: vtkPolyData containing only edges
        """
        vextract_edges = vtk.vtkFeatureEdges()
        vextract_edges.FeatureEdgesOn()
        vextract_edges.BoundaryEdgesOff()
        vextract_edges.ColoringOn()
        vextract_edges.SetInputData(poly_data_input)
        vextract_edges.Update()
        return vextract_edges.GetOutput()

    def get_outer_edges(self, input_poly):
        """
        Extract only the outer edges of a mesh
        :param input_poly: vtkPolyData to be analyzed
        :return: vtkPolyData containing only edges
        """
        feature_edges = vtk.vtkFeatureEdges()

        feature_edges.SetInputData(input_poly)
        feature_edges.BoundaryEdgesOn()
        feature_edges.FeatureEdgesOff()
        feature_edges.ManifoldEdgesOff()
        feature_edges.NonManifoldEdgesOff()
        feature_edges.Update()
        edge_poly = feature_edges.GetOutput()

        return edge_poly

    def get_edge_skeleton(self, input_poly):
        """
        Computes the skeleton the mesh
        :param input_poly: outer edge vtkPolyData of the maxilar shape
        :return: vtkPolyData of the alveolar line,
                vtkPoints containing all the points on the line
        """
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

            edge_locator.FindCellsAlongLine(start_point, end_points[i], 0.001, cells_edges)
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

    def numpyArrayToPolyData(self,npArray):
        """
        Turns a numpy array of points into a vtkPolydata
        :param npArray: a np array containing points (3D)
        :return: a vtkpolydata containing the points
        """
        vtkpoints = vtk.vtkPoints()  # here we add the points
        whole_line_final = vtk.vtkCellArray()
        poly_line_final = vtk.vtkPolyLine()
        poly_line_final.GetPointIds().SetNumberOfIds(len(npArray))
        counter = 0
        for pt in npArray:
            vtkPt = [pt[0],pt[1],pt[2]]
            pid = vtkpoints.InsertNextPoint(vtkPt)
            poly_line_final.GetPointIds().SetId(counter, pid)

            counter += 1

        poly_line_final.GetPointIds().SetNumberOfIds(counter - 1)
        whole_line_final.InsertNextCell(poly_line_final)

        polydata_final = vtk.vtkPolyData()
        polydata_final.SetPoints(vtkpoints)
        polydata_final.SetLines(whole_line_final)

        return polydata_final

    def place_skeleton_on_original_mesh(self, original_poly, skeleton_points):
        """
        Places the computed alveolar line points on the mesh by intersecting vertical lines with the original 3D mold
        to find intersection points and therefore to find elevations
        :param original_poly: vtkPolyData of the original 3D mold
        :param skeleton_points: points of the alveolar line to be raised at the level of the original 3D mold
        :return: vtkPolyData of the final skeleton
        """

        original_locator = vtk.vtkOBBTree()
        original_locator.SetDataSet(original_poly)
        original_locator.BuildLocator()

        line_length = abs(original_poly.GetBounds()[5]) + 1

        skeleton_points_final = vtk.vtkPoints()  # here we add the points that we find
        intersection_points = vtk.vtkPoints()
        intersection_cells = vtk.vtkIdList()
        whole_line_final = vtk.vtkCellArray()
        poly_line_final = vtk.vtkPolyLine()
        poly_line_final.GetPointIds().SetNumberOfIds(len(skeleton_points))
        counter = 0
        for alv_line_point in skeleton_points:
            original_locator.IntersectWithLine(alv_line_point, [alv_line_point[0], alv_line_point[1],
                                                                alv_line_point[2] + line_length], intersection_points,
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
