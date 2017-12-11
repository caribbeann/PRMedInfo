import vtk
import math
import numpy as np
from bresenham import bresenham
from sklearn.cluster import KMeans

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

    def align_to_z_axis(self, polydata_input):
        """
        Align the mesh so that the Z axis points along the smalles dimension of the mesh
        :param polydata_input: vtkPolyData to be aligned
        :return: transformed vtkPolyData and used transformation
        """

        tmp_transform = vtk.vtkTransform()
        tmp_transform.PostMultiply()

        pca_dict1 = self.compute_pca(polydata_input)
        reoriented_poly_data, trans = self.rotate(polydata_input, [0.0, 0.0, 1.0], pca_dict1['eigenvectors'][2])
        tmp_transform.Concatenate(trans)
        reoriented_poly_data, trans = self.translate_to_origin(reoriented_poly_data)
        tmp_transform.Concatenate(trans)

        # check if the mesh is still on its head and rotate if that is the case
        featureEdges = vtk.vtkFeatureEdges()

        featureEdges.SetInputData(reoriented_poly_data)
        featureEdges.BoundaryEdgesOn()
        featureEdges.FeatureEdgesOff()
        featureEdges.ManifoldEdgesOff()
        featureEdges.NonManifoldEdgesOff()
        featureEdges.Update()
        edge_poly = featureEdges.GetOutput()
        gr_edge = self.compute_center_of_mass(edge_poly)

        if gr_edge[2] > 0:  # the basis should be negative (under center of gravity which has (0,0,0) coordinates)
            reoriented_poly_data, trans = self.rotate_angle(reoriented_poly_data, [0, 1, 0], 180)
            tmp_transform.Concatenate(trans)

        return reoriented_poly_data, tmp_transform

    def align_to_axes(self, polydata_input, skip_gingiva = True, threshold=1):
        """
        Align the axes of the scene with the main axes of the object by PCA, for lower mesh also detects upper part of gingiva
        :param polydata_input: mesh to be aligned
        :return: aligned vtkPolyData, corresponding transformation, upper part of gingiva
        """

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


        # Reorient data
        gravity = self.compute_center_of_mass(reoriented_poly_data)
        bounds = reoriented_poly_data.GetBounds()

        # check if y correct
        front = self.crop_mesh(reoriented_poly_data, bounds[0], bounds[1], gravity[1], bounds[3], bounds[4], bounds[5])
        back = self.crop_mesh(reoriented_poly_data, bounds[0], bounds[1], bounds[2], gravity[1], bounds[4], bounds[5])
        frBounds = front.GetBounds()
        bkBounds = back.GetBounds()

        # check if y in right direction
        if frBounds[1] - frBounds[0] > bkBounds[1] - bkBounds[0]:
            print "flip y"
            reoriented_poly_data,trans = self.rotate_angle(reoriented_poly_data, [0, 0, 1], 180)
            tmp_transform.Concatenate(trans)
            bounds = reoriented_poly_data.GetBounds()

        # decimate mesh to help computation
        dec = vtk.vtkDecimatePro()
        dec.SetInputData(reoriented_poly_data)
        dec.SetTargetReduction(0.001)
        dec.PreserveTopologyOff()
        dec.SplittingOn()
        dec.BoundaryVertexDeletionOn()
        dec.Update()


        # compute curvature value
        curv = vtk.vtkCurvatures()
        curv.SetInputData(dec.GetOutput())
        curv.SetCurvatureTypeToMean()
        curv.Update()

        import renderer
        # rend = renderer.Renderer()
        # rend.add_actor(curv.GetOutput(), color=[1, 1, 1], wireframe=False)
        # rend.render()

        # get rid of too high and too low curvature (the interest points are in range [0,1])
        thres = vtk.vtkThresholdPoints()
        thres.SetInputConnection(curv.GetOutputPort())
        thres.ThresholdBetween(0, threshold)
        thres.Update()

        #
        # import renderer
        # rend = renderer.Renderer()
        # rend.add_actor(thres.GetOutput(), color=[1, 1, 1], wireframe=False)
        # rend.render()

        # there are still too many points (with flat points as well), need to get rid of them by using mean and std
        scalarValues = thres.GetOutput().GetPointData().GetScalars()
        vals = np.zeros(scalarValues.GetNumberOfTuples())
        for i in range(0, scalarValues.GetNumberOfTuples()):
            vals[i] = scalarValues.GetTuple(i)[0]

        valMean = np.mean(vals)
        stdev = np.std(vals)

        print valMean
        print stdev


        thres2 = vtk.vtkThresholdPoints()
        thres2.SetInputConnection(thres.GetOutputPort())
        thres2.ThresholdBetween(valMean - 2 * stdev, valMean + 2* stdev)
        thres2.Update()

        # import renderer
        # rend = renderer.Renderer()
        # rend.add_actor(thres2.GetOutput(), color=[1, 1, 1], wireframe=False)
        # rend.render()


        # the result is a cloud of points, need to build a polydata, but only when points are close enough to one other
        randomDelaunay = vtk.vtkDelaunay2D()
        randomDelaunay.SetInputConnection(thres2.GetOutputPort())

        print thres2.GetOutput().GetNumberOfPoints()

        # if there are not many points, increase a bit the radius
        if thres2.GetOutput().GetNumberOfPoints() < 15000:
            randomDelaunay.SetAlpha(0.3)  # radius max of a point to connect with another one
        else:
            randomDelaunay.SetAlpha(0.2)
        randomDelaunay.Update()


        # rend = renderer.Renderer()
        # rend.add_actor(randomDelaunay.GetOutput(), color=[1, 1, 1], wireframe=False)
        # rend.render()

        # get biggest component
        con_filter = vtk.vtkPolyDataConnectivityFilter()
        con_filter.SetExtractionModeToLargestRegion()
        con_filter.SetInputConnection(randomDelaunay.GetOutputPort())
        con_filter.Update()
        upperGingiva = vtk.vtkPolyData()
        upperGingiva.DeepCopy(con_filter.GetOutput())

        newBounds = upperGingiva.GetBounds()

        # rend = renderer.Renderer()
        # rend.add_actor(con_filter.GetOutput(), color=[1, 1, 1], wireframe=False)
        # rend.render()

        print upperGingiva.GetNumberOfCells()
        print newBounds
        print bounds

        # if the teeth are disconnected, only a part has been found, need to add other big regions as well
        if not skip_gingiva and (newBounds[1] - newBounds[0] < 0.7 * (bounds[1] - bounds[0])
                                 or newBounds[3] - newBounds[2] < 0.7 * (bounds[3] - bounds[2]) ):
            con_filter = vtk.vtkPolyDataConnectivityFilter()
            con_filter.SetExtractionModeToAllRegions()
            con_filter.ColorRegionsOn()
            con_filter.SetInputConnection(randomDelaunay.GetOutputPort())
            con_filter.ScalarConnectivityOff()
            con_filter.Update()

            nbRegions = con_filter.GetNumberOfExtractedRegions()
            con_filter.InitializeSpecifiedRegionList()

            previousCellNb = 0

            for i in range(0, nbRegions):
                con_filter.SetExtractionModeToSpecifiedRegions()
                con_filter.AddSpecifiedRegion(i)  # add a region to the selection
                con_filter.Update()
                nbCells = con_filter.GetOutput().GetNumberOfCells()
                if nbCells - previousCellNb < 200:  # get only big regions
                    con_filter.DeleteSpecifiedRegion(i)  # do not keep the region (delete from selection)
                previousCellNb = nbCells  # contains all cells except the ones from deleted regions
            upperGingiva.DeepCopy(con_filter.GetOutput())

        newGravity = self.compute_center_of_mass(upperGingiva)
        print newGravity


        # the upper part of teeth has been extracted, as the mesh is centered on the center of mass of initial polydata
        # if the center of mass of the upper part is above, the mesh is correctly oriented, otherwise, need to rotate around y
        # to have z facing the good direction
        if newGravity[2] < 0:
            print "flip z"
            upperGingiva,_ = self.rotate_angle(upperGingiva, [0, 1, 0], 180)
            reoriented_poly_data,trans = self.rotate_angle(reoriented_poly_data, [0, 1, 0], 180)
            tmp_transform.Concatenate(trans)


        return reoriented_poly_data, tmp_transform, upperGingiva

    def is_mesh_with_basis_on_head(self, input_poly_data):
        """
        Checks if a mesh that has a basis (i guess these are the gips meshes) is rotated.
        i.e. if the Z axis points toward the brain and not toward the floor as we want it to
        :param input_poly_data: vtkPolyData to be analyzed
        :return: True if needs rotation
                False if its pointing correctly
        """
        featureEdges = vtk.vtkFeatureEdges()

        featureEdges.SetInputData(input_poly_data)
        featureEdges.BoundaryEdgesOn()
        featureEdges.FeatureEdgesOff()
        featureEdges.ManifoldEdgesOff()
        featureEdges.NonManifoldEdgesOff()
        featureEdges.Update()
        edge_poly = featureEdges.GetOutput()
        gr_edge = self.compute_center_of_mass(edge_poly)
        gr_input = self.compute_center_of_mass(input_poly_data)
        if gr_edge[2] > gr_input[2]:  # the basis should be negative (under center of gravity which has (0,0,0) coordinates)
            return True
        return False

    def flatten(self, input_poly):

        transform = vtk.vtkTransform()
        transform.Scale(1,1,0)

        filter = vtk.vtkTransformPolyDataFilter()
        filter.SetTransform(transform)
        filter.SetInputData(input_poly)

        filter.Update()

        return filter.GetOutput(), transform

    def translate_to_xy_x_centered(self, input_poly):
        """
        Translates a mesh in the XY plane (Zmin=0) in the Y positive half and Y axis passing through the center of mass
        :param input_poly: vtkPolyData to be translated
        :return: translated vtkPolyData and the transformation vector used to translate
        """
        tran1 = [-input_poly.GetBounds()[0], 0, -input_poly.GetBounds()[4]]
        mesh, trans = self.translate(input_poly, 0, -input_poly.GetBounds()[2], -input_poly.GetBounds()[4])

        x, y, z = self.compute_center_of_mass(mesh)
        tran2 = [-x, -y, -z]
        mesh, trans = self.translate(mesh, -x, -y, -z)

        tran3 = [0, 0, -mesh.GetBounds()[4]]
        mesh, trans = self.translate(mesh, 0, 0, -mesh.GetBounds()[4])

        tran4 = [0, 0, 0]
        mesh, trans = self.translate(mesh, 0, 0, 0)

        tran5 = [-mesh.GetBounds()[0], 0, 0]
        mesh, trans = self.translate(mesh, -mesh.GetBounds()[0], 0, 0)

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
        :return: rotated vtkPolyData and vtkTransformed used
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

        return transform_filter.GetOutput(), transform

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

    def check_if_enough_is_left(self, original_mesh, cropped_mesh, percentage):
        """
        Checks if after cropping of a mesh, the cropping has not caused the mesh to reduce a lot in size due to for example
        local maxima on the original mesh that still act as a single component when checked.
        With this method we check if a certain percentage of the X and Y dimensions of the mesh are still there
        :param original_mesh:
        :param cropped_mesh:
        :param percentage:
        :return:
        """
        bounds_o = original_mesh.GetBounds()
        bounds_c = cropped_mesh.GetBounds()

        if (abs(bounds_c[0]) + abs(bounds_c[1])) >= percentage *(abs(bounds_o[0]) + abs(bounds_o[1])) and \
                        (abs(bounds_c[2]) + abs(bounds_c[3])) >= percentage * (abs(bounds_o[2]) + abs(bounds_o[3])):
            return True
        return False

    def crop_mesh_maximum(self, poly_input, xmin, xmax, ymin, ymax, zmin, zmax, min_percent, max_percent, percent_step, keep_percentage):
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
        :param keep_percentage: how much of the X and Y dimensions of the original mesh to keep
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
            if con_filter.GetNumberOfExtractedRegions() == 1 and \
                    self.check_if_enough_is_left(poly_input, cropped_poly_data, keep_percentage):
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
            #print dim[i]
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

    def smoothen(self, input_poly, iterations, relaxation_factor) :
        """
        Smoothens a given polydata as a possible preprocessing
        :param input_poly: vtkPolyData to smoothen
        :return: smoothened vtkPolyData
        """
        sf = vtk.vtkSmoothPolyDataFilter()
        sf.SetInputData(input_poly)
        sf.SetNumberOfIterations(iterations)
        sf.SetRelaxationFactor(relaxation_factor)
        sf.FeatureEdgeSmoothingOn()
        sf.BoundarySmoothingOn()
        sf.Update()

        return sf.GetOutput()

    def compute_elevation(self, input_poly_data):
        """
        Compute elevation field of a mesh.
        Method assumes that the mesh was previously aligned with its lowest dimension (3rd eigenvector) along Z axis
        :param input_poly_data:
        :return: vtkPolyData containing Data about the elevation of a mesh
        """
        bounds = input_poly_data.GetBounds()
        s_elev = vtk.vtkElevationFilter()
        s_elev.SetInputData(input_poly_data)
        s_elev.SetHighPoint(0, 0, bounds[5])
        s_elev.SetLowPoint(0, 0, bounds[4])
        s_elev.Update()

        return s_elev.GetOutput()

    def extract_largest_region(self, input_poly_data):
        """
        Returns only the largest component of a disconnected mesh.
        :param input_poly_data:
        :return: vtkPolyData of the largest component
        """
        con_filter = vtk.vtkPolyDataConnectivityFilter()
        con_filter.SetExtractionModeToLargestRegion()
        con_filter.SetInputData(input_poly_data)
        con_filter.Update()

        return con_filter.GetOutput()

    def compute_elevation_gradient(self, input_poly_data):
        """
        Computes the gradient field of elevation of a mesh.
        Method assumes that the mesh was previously aligned with its lowest dimension (3rd eigenvector) along Z axis.
        :param input_poly_data:
        :return: vtkPolyData containing data about gradients
        """
        elev = self.compute_elevation(input_poly_data)

        gradients = vtk.vtkGradientFilter()
        gradients.SetInputData(elev)
        gradients.SetInputArrayToProcess(0, 0, 0, vtk.vtkDataObject.FIELD_ASSOCIATION_POINTS, "Elevation")
        gradients.Update()

        return gradients.GetOutput()

    def extract(self, input_poly, threshold=0.02):
        """
        Removes the border (border region) of the passed vtkPolyData.
        To do this, a gradient field of elevation is built.
        Then, a gradient magnitude is built in each point of the mesh
        The removal starts at the point on the mesh corresponding to the center of mass.
        The neighboring points are then recursively removed if the gradient magnitude at that position is below a
            threshold.
        Method assumes that the mesh was previously aligned with its lowest dimension (3rd eigenvector) along Z axis.
        :param input_poly:
        :param threshold: remove
        :return: same vtkPolyData that was passed, but with tongue (center region removed)
        """

        bounds = input_poly.GetBounds()

        polydata = input_poly

        con_filter = vtk.vtkPolyDataConnectivityFilter()
        con_filter.SetExtractionModeToLargestRegion()
        con_filter.SetInputData(polydata)
        con_filter.Update()
        polydata = con_filter.GetOutput()
        polydata, _ = self.translate_to_xy_plane(polydata)

        # compute gradient
        gradient = self.compute_elevation_gradient(polydata)

        # gradientVals = np.zeros(polydata.GetNumberOfPoints())
        # for i in range (0, polydata.GetNumberOfPoints()):
        #     gt = gradient.GetPointData().GetArray("Gradients").GetTuple(i)
        #     m = math.sqrt(gt[0] ** 2 + gt[1] ** 2 + gt[2] ** 2)
        #     gradientVals[i] = m
        #
        #
        # print gradient.GetPoint(0)
        # print polydata.GetPoint(0)
        # print np.mean(gradientVals)
        # minVal = np.min(gradientVals)
        # stdev = np.std(gradientVals)
        # print stdev
        #
        # threshold = minVal+2*stdev
        #
        # skeleton_points_final = vtk.vtkPoints()  # here we add the points that we find
        # for i in range(0, polydata.GetNumberOfPoints()):
        #     if gradientVals[i]<threshold:
        #         skeleton_points_final.InsertNextPoint(gradient.GetPoint(i))
        #
        #
        # skeleton_final = vtk.vtkPolyData()
        # skeleton_final.SetPoints(skeleton_points_final)
        #
        # randomDelaunay = vtk.vtkDelaunay2D()
        # randomDelaunay.SetInputData(skeleton_final)
        # randomDelaunay.SetAlpha(0.8)
        # randomDelaunay.Update()
        # #skeleton_final.SetLines(whole_line_final)
        # # import renderer
        # # rend = renderer.Renderer()
        # # rend.add_actor(randomDelaunay.GetOutput(), color=[1, 1, 1], wireframe=False)
        # # rend.render()

        elevVals = np.zeros(polydata.GetNumberOfPoints())
        for i in range(0, polydata.GetNumberOfPoints()):
            gt = gradient.GetPointData().GetArray("Elevation").GetTuple(i)
            m = gt[0]#math.sqrt(gt[0] ** 2 + gt[1] ** 2 + gt[2] ** 2)
            elevVals[i] = m

        print gradient.GetPoint(0)
        print polydata.GetPoint(0)
        print np.mean(elevVals)
        minVal = np.min(elevVals)
        print np.max(elevVals)
        stdev = np.std(elevVals)
        print stdev

        threshold = np.mean(elevVals) + 0.75*stdev#minVal + 2 * stdev

        skeleton_points_final = vtk.vtkPoints()  # here we add the points that we find
        for i in range(0, polydata.GetNumberOfPoints()):
            if elevVals[i] > threshold:
                skeleton_points_final.InsertNextPoint(gradient.GetPoint(i))

        skeleton_final = vtk.vtkPolyData()
        skeleton_final.SetPoints(skeleton_points_final)

        randomDelaunay = vtk.vtkDelaunay2D()
        randomDelaunay.SetInputData(skeleton_final)
        randomDelaunay.SetAlpha(0.8)
        randomDelaunay.Update()


        return randomDelaunay.GetOutput()

    def remove_border(self, input_poly, threshold=0.02):
        """
        Removes the border (border region) of the passed vtkPolyData.
        To do this, a gradient field of elevation is built.
        Then, a gradient magnitude is built in each point of the mesh
        The removal starts at the point on the mesh corresponding to the center of mass.
        The neighboring points are then recursively removed if the gradient magnitude at that position is below a
            threshold.
        Method assumes that the mesh was previously aligned with its lowest dimension (3rd eigenvector) along Z axis.
        :param input_poly:
        :param threshold: remove
        :return: same vtkPolyData that was passed, but with tongue (center region removed)
        """

        bounds = input_poly.GetBounds()
        gravity = self.compute_center_of_mass(input_poly)

        polydata = input_poly
        edge_locator = vtk.vtkCellLocator()
        edge_locator.SetDataSet(polydata)
        edge_locator.BuildLocator()

        featureEdges = vtk.vtkFeatureEdges()

        featureEdges.SetInputData(polydata)  # get basis
        featureEdges.BoundaryEdgesOn()
        featureEdges.FeatureEdgesOff()
        featureEdges.ManifoldEdgesOff()
        featureEdges.NonManifoldEdgesOff()
        featureEdges.Update()
        edge_poly = featureEdges.GetOutput()

        ptIds = vtk.vtkIdList()
        for i in range(0, edge_poly.GetNumberOfPoints()):
            ptIds.InsertNextId(polydata.FindPoint(edge_poly.GetPoint(i)))
        #
        # con_filter = vtk.vtkPolyDataConnectivityFilter()
        # con_filter.SetExtractionModeToLargestRegion()
        # con_filter.SetInputData(polydata)
        # con_filter.Update()
        # polydata = con_filter.GetOutput()
        # polydata, _ = self.translate_to_xy_plane(polydata)

        # compute gradient
        gradient = self.compute_elevation_gradient(polydata)

        while ptIds.GetNumberOfIds() > 0:

            # get neighboring cells
            neighborsCellIds = vtk.vtkIdList()
            polydata.GetPointCells(ptIds.GetId(0), neighborsCellIds)

            # from each cell, get the points that compose it
            for i in range(0, neighborsCellIds.GetNumberOfIds()):
                cellId = neighborsCellIds.GetId(i)

                newPtIds = vtk.vtkIdList()
                if polydata.GetCell(cellId).GetCellType() != vtk.VTK_EMPTY_CELL:  # if cell was not marked as deleted
                    # get the points of these cell
                    polydata.GetCellPoints(cellId, newPtIds)
                    for j in range(0, newPtIds.GetNumberOfIds()):
                        # insert these points to the set if they meet the requirements
                        newPtId = newPtIds.GetId(j)
                        gt = gradient.GetPointData().GetArray("Gradients").GetTuple(newPtId)
                        m = math.sqrt(gt[0] ** 2 + gt[1] ** 2 + gt[2] ** 2)
                        if m < threshold:
                            ptIds.InsertNextId(newPtId)
                polydata.DeleteCell(cellId)  # mark it so we know it was already traversed

            polydata.DeletePoint(ptIds.GetId(0))
            ptIds.DeleteId(ptIds.GetId(0))

        polydata.RemoveDeletedCells()
        polydata.BuildCells()


        #polydata = self.extract_largest_region(polydata)

        return polydata

    def adaptative_remove_tongue(self, input_poly, threshold=0.02):
        """
        Removes the tongue (central region) of the passed vtkPolyData.
        To do this, a gradient field of elevation is built.
        Then, a gradient magnitude is built in each point of the mesh
        The removal starts at the point on the mesh corresponding to the center of mass.
        The neighboring points are then recursively removed if the gradient magnitude at that position is below a
            threshold.
        Method assumes that the mesh was previously aligned with its lowest dimension (3rd eigenvector) along Z axis.
        :param input_poly:
        :param threshold: remove
        :return: same vtkPolyData that was passed, but with tongue (center region removed)
        """

        bounds = input_poly.GetBounds()
        gravity = self.compute_center_of_mass(input_poly)

        polydata = input_poly
        edge_locator = vtk.vtkCellLocator()
        edge_locator.SetDataSet(polydata)
        edge_locator.BuildLocator()

        pcoord = [0.0, 0.0, 0.0]  # will contain the coords of the intersection of a vertical line with the mesh through the c.o.m.
        junk = [0.0, 0.0, 0.0]
        t = vtk.mutable(0)
        subId = vtk.mutable(0)
        centerCellId = vtk.mutable(0)
        cell = vtk.vtkGenericCell()
        nr = edge_locator.IntersectWithLine([gravity[0], gravity[1], bounds[4] + 1],
                                            [gravity[0], gravity[1], bounds[5] - 1], 0.001, t, pcoord, junk, subId,
                                            centerCellId, cell)

        con_filter = vtk.vtkPolyDataConnectivityFilter()
        con_filter.SetExtractionModeToLargestRegion()
        con_filter.SetInputData(polydata)
        con_filter.Update()
        polydata = con_filter.GetOutput()
        #polydata, _ = self.translate_to_xy_plane(polydata)

        # compute gradient
        gradient = self.compute_elevation_gradient(polydata)


        gradientVals = np.zeros(polydata.GetNumberOfPoints())
        for i in range(0, polydata.GetNumberOfPoints()):
            gt = gradient.GetPointData().GetArray("Elevation").GetTuple(i)
            #m = math.sqrt(gt[0] ** 2 + gt[1] ** 2 + gt[2] ** 2)
            gradientVals[i] = gt[0]#m

        meanVal = np.mean(gradientVals)
        stdev = np.std(gradientVals)
        print np.max(gradientVals)
        print meanVal
        print np.min(gradientVals)
        print stdev


        neighborsCellIds = vtk.vtkIdList()
        neighborsCellIds.InsertNextId(centerCellId)

        # at the beginning this contains only the c.o.m
        ptIds = vtk.vtkIdList()
        ptIds.InsertNextId(polydata.FindPoint(pcoord))

        while ptIds.GetNumberOfIds() > 0:

            # get neighboring cells
            neighborsCellIds = vtk.vtkIdList()
            polydata.GetPointCells(ptIds.GetId(0), neighborsCellIds)

            # from each cell, get the points that compose it
            for i in range(0, neighborsCellIds.GetNumberOfIds()):
                cellId = neighborsCellIds.GetId(i)

                newPtIds = vtk.vtkIdList()
                if polydata.GetCell(cellId).GetCellType() != vtk.VTK_EMPTY_CELL:  # if cell was not marked as deleted
                    # get the points of these cell
                    polydata.GetCellPoints(cellId, newPtIds)
                    for j in range(0, newPtIds.GetNumberOfIds()):
                        # insert these points to the set if they meet the requirements
                        newPtId = newPtIds.GetId(j)
                        gt = gradient.GetPointData().GetArray("Elevation").GetTuple(newPtId)
                        m = gt[0]#math.sqrt(gt[0] ** 2 + gt[1] ** 2 + gt[2] ** 2)
                        if m < meanVal-stdev:
                            ptIds.InsertNextId(newPtId)
                polydata.DeleteCell(cellId)  # mark it so we know it was already traversed

            polydata.DeletePoint(ptIds.GetId(0))
            ptIds.DeleteId(ptIds.GetId(0))

        polydata.RemoveDeletedCells()
        polydata.BuildCells()


        polydata = self.extract_largest_region(polydata)

        return polydata


    def remove_tongue(self, input_poly, threshold=0.02):
        """
        Removes the tongue (central region) of the passed vtkPolyData.
        To do this, a gradient field of elevation is built.
        Then, a gradient magnitude is built in each point of the mesh
        The removal starts at the point on the mesh corresponding to the center of mass.
        The neighboring points are then recursively removed if the gradient magnitude at that position is below a
            threshold.
        Method assumes that the mesh was previously aligned with its lowest dimension (3rd eigenvector) along Z axis.
        :param input_poly:
        :param threshold: remove
        :return: same vtkPolyData that was passed, but with tongue (center region removed)
        """

        bounds = input_poly.GetBounds()
        gravity = self.compute_center_of_mass(input_poly)

        polydata = input_poly
        edge_locator = vtk.vtkCellLocator()
        edge_locator.SetDataSet(polydata)
        edge_locator.BuildLocator()

        pcoord = [0.0, 0.0, 0.0]  # will contain the coords of the intersection of a vertical line with the mesh through the c.o.m.
        junk = [0.0, 0.0, 0.0]
        t = vtk.mutable(0)
        subId = vtk.mutable(0)
        centerCellId = vtk.mutable(0)
        cell = vtk.vtkGenericCell()
        nr = edge_locator.IntersectWithLine([gravity[0], gravity[1], bounds[4] + 1],
                                            [gravity[0], gravity[1], bounds[5] - 1], 0.001, t, pcoord, junk, subId,
                                            centerCellId, cell)

        con_filter = vtk.vtkPolyDataConnectivityFilter()
        con_filter.SetExtractionModeToLargestRegion()
        con_filter.SetInputData(polydata)
        con_filter.Update()
        polydata = con_filter.GetOutput()
        #polydata, _ = self.translate_to_xy_plane(polydata)

        # compute gradient
        gradient = self.compute_elevation_gradient(polydata)

        neighborsCellIds = vtk.vtkIdList()
        neighborsCellIds.InsertNextId(centerCellId)

        # at the beginning this contains only the c.o.m
        ptIds = vtk.vtkIdList()
        ptIds.InsertNextId(polydata.FindPoint(pcoord))

        while ptIds.GetNumberOfIds() > 0:

            # get neighboring cells
            neighborsCellIds = vtk.vtkIdList()
            polydata.GetPointCells(ptIds.GetId(0), neighborsCellIds)

            # from each cell, get the points that compose it
            for i in range(0, neighborsCellIds.GetNumberOfIds()):
                cellId = neighborsCellIds.GetId(i)

                newPtIds = vtk.vtkIdList()
                if polydata.GetCell(cellId).GetCellType() != vtk.VTK_EMPTY_CELL:  # if cell was not marked as deleted
                    # get the points of these cell
                    polydata.GetCellPoints(cellId, newPtIds)
                    for j in range(0, newPtIds.GetNumberOfIds()):
                        # insert these points to the set if they meet the requirements
                        newPtId = newPtIds.GetId(j)
                        gt = gradient.GetPointData().GetArray("Gradients").GetTuple(newPtId)
                        m = math.sqrt(gt[0] ** 2 + gt[1] ** 2 + gt[2] ** 2)
                        if m < threshold:
                            ptIds.InsertNextId(newPtId)
                polydata.DeleteCell(cellId)  # mark it so we know it was already traversed

            polydata.DeletePoint(ptIds.GetId(0))
            ptIds.DeleteId(ptIds.GetId(0))

        polydata.RemoveDeletedCells()
        polydata.BuildCells()


        polydata = self.extract_largest_region(polydata)

        return polydata

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

        line_length = 2*abs(input_poly.GetBounds()[3]) + 2*abs(input_poly.GetBounds()[1])
        start_point = [0, 0, 0]
        end_points = []
        for angle in np.linspace(0, 2* math.pi, 72):  # every 5 degrees in full circle
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
            pid = vtk.vtkIdList()  # contains the point ids of the first cell intersection
            p_intersections = []
            pid_list = []
            if cells_edges.GetNumberOfIds() >= 2:

                for i in range(cells_edges.GetNumberOfIds()):

                    id = cells_edges.GetId(i)
                    input_poly.GetCellPoints(id, pid)
                    pid_list.append(pid)

                    p_int = np.add(np.array(input_poly.GetPoint(pid.GetId(0))),
                                     np.array(input_poly.GetPoint(pid.GetId(1)))) / 2
                    p_intersections.append(p_int)
                parr = np.array(p_intersections)
                kmeans = KMeans(n_clusters=2, random_state=0).fit(parr)

                if len(kmeans.labels_) >= 2:

                    mean_cluster1 = np.sum(parr[kmeans.labels_== 0], axis=0)/parr[kmeans.labels_== 0].shape[0]
                    mean_cluster2 = np.sum(parr[kmeans.labels_== 1], axis=0)/parr[kmeans.labels_== 1].shape[0]

                    p_alv_line = (mean_cluster1+mean_cluster2)/2
                    alv_line_points.append(p_alv_line)
                    skeleton_points.InsertNextPoint(list(p_alv_line))

                    poly_line.GetPointIds().SetId(counter, counter)
                    counter += 1
        poly_line.GetPointIds().SetNumberOfIds(counter - 1)
        whole_line.InsertNextCell(poly_line)
        skeleton = vtk.vtkPolyData()
        skeleton.SetPoints(skeleton_points)
        skeleton.SetLines(whole_line)
        return skeleton, alv_line_points

    def numpy_array_to_poly_data(self, npArray):
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

    def place_skeleton_on_original_mesh(self, original_poly, skeleton_points, line_accuracy, percentage_of_radius, percentage_of_radius_outwards):
        """
        Places the computed alveolar line points on the mesh by intersecting vertical lines with the original 3D mold
        to find intersection points and therefore to find elevations
        :param original_poly: vtkPolyData of the original 3D mold
        :param skeleton_points: points of the alveolar line to be raised at the level of the original 3D mold
        :return: vtkPolyData of the final skeleton
        """

        def find_higher_elevation(elevation_poly, original_elevation, original_pt, point_list, section, l_length):

            locator = vtk.vtkOBBTree()
            locator.SetDataSet(elevation_poly)
            locator.BuildLocator()
            int_points = vtk.vtkPoints()
            int_cells = vtk.vtkIdList()
            max_elevation = original_elevation
            max_pt = original_pt
            c_p_ids = vtk.vtkIdList()
            for k in range(int(point_list.shape[0]*section)):

                sp = [point_list[k][0],point_list[k][1], 0]
                ep = [sp[0], sp[1], sp[2] + l_length]
                locator.IntersectWithLine(sp, ep, int_points, int_cells)

                if int_points.GetNumberOfPoints()>=1:
                    elevation_poly.GetCellPoints(int_cells.GetId(int_cells.GetNumberOfIds() - 1), c_p_ids)
                    e0 = elevation_poly.GetPointData().GetArray("Elevation").GetValue(c_p_ids.GetId(0))
                    e1 = elevation_poly.GetPointData().GetArray("Elevation").GetValue(c_p_ids.GetId(1))
                    e2 = elevation_poly.GetPointData().GetArray("Elevation").GetValue(c_p_ids.GetId(2))

                    elevation = max(e0, e1, e2)

                    if elevation > max_elevation:
                        max_elevation = elevation
                        max_pt = int_points.GetPoint(int_points.GetNumberOfPoints() - 1)


            return max_pt

        def find_points(xs, xe, ys, ye, nr):
            x = np.linspace(xs, xe, nr)
            y = np.linspace(ys, ye, nr)
            xv, yv = np.meshgrid(x, y)

            points = np.zeros((nr,2))

            for l in range(nr):
                points[l,0] = xv[l, l]
                points[l,1] = yv[l, l]
            return points

        def extend_line(points, amount, accuracy):
            difx = points[0][0] - points[points.shape[0] - 1][0]
            dify = points[0][1] - points[points.shape[0] - 1][1]
            end_point_other_direction = [points[0][0] + difx, points[0][1] + dify]

            new_points = find_points(points[0][0], end_point_other_direction[0], points[0][1], end_point_other_direction[1], accuracy)

            new_points = new_points[:int(amount*new_points.shape[0]), :]

            points_final = np.concatenate([points, new_points], axis=0)

            return points_final


        elev_poly = self.compute_elevation(original_poly)

        original_locator = vtk.vtkOBBTree()
        original_locator.SetDataSet(elev_poly)
        original_locator.BuildLocator()

        line_length = abs(elev_poly.GetBounds()[5]) + 1

        skeleton_points_final = vtk.vtkPoints()  # here we add the points that we find
        intersection_points = vtk.vtkPoints()
        intersection_cells = vtk.vtkIdList()
        whole_line_final = vtk.vtkCellArray()
        poly_line_final = vtk.vtkPolyLine()
        poly_line_final.GetPointIds().SetNumberOfIds(len(skeleton_points))
        counter = 0
        cell_point_ids = vtk.vtkIdList()

        for alv_line_point in skeleton_points:
            original_locator.IntersectWithLine(alv_line_point, [alv_line_point[0], alv_line_point[1],
                                                                alv_line_point[2] + line_length], intersection_points,
                                               intersection_cells)

            if intersection_points.GetNumberOfPoints() >= 1:
                p_int = intersection_points.GetPoint(intersection_points.GetNumberOfPoints() - 1) # get the last point (it coincides with the surface point)
                elev_poly.GetCellPoints(intersection_cells.GetId(intersection_cells.GetNumberOfIds() - 1), cell_point_ids)

                # find out the approx (mean) elevation for the intersection point(cell)
                elev0 = elev_poly.GetPointData().GetArray("Elevation").GetValue(cell_point_ids.GetId(0))
                elev1 = elev_poly.GetPointData().GetArray("Elevation").GetValue(cell_point_ids.GetId(1))
                elev2 = elev_poly.GetPointData().GetArray("Elevation").GetValue(cell_point_ids.GetId(2))

                elev = max(elev0, elev1, elev2)
                # check the profile "perpendicular to the alveolar line for maxima

                com = self.compute_center_of_mass(elev_poly) # center of mass

                # point_list_com = list(bresenham(int(alv_line_point[0]), int(alv_line_point[1]), int(com[0]), int(com[1])))
                point_list_com = find_points(alv_line_point[0], com[0], alv_line_point[1], com[1], line_accuracy)
                # point_list_com = np.array(point_list_com)

                point_list_com = extend_line(point_list_com, percentage_of_radius_outwards, line_accuracy)
                p_int_new = find_higher_elevation(elev_poly,
                                                  elev,
                                                  p_int, point_list_com, percentage_of_radius, line_length)

                pid = skeleton_points_final.InsertNextPoint(p_int_new)
                poly_line_final.GetPointIds().SetId(counter, pid)

                counter += 1

        poly_line_final.GetPointIds().SetNumberOfIds(counter - 1)
        whole_line_final.InsertNextCell(poly_line_final)

        skeleton_final = vtk.vtkPolyData()
        skeleton_final.SetPoints(skeleton_points_final)
        skeleton_final.SetLines(whole_line_final)

        return skeleton_final
