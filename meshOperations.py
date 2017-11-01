import vtk
import math

class MeshOperations:
    def __init__(self, *args, **kwargs):
        self.poly_data = kwargs.get("poly_data")

    def translate(self,x,y,z):
        transform = vtk.vtkTransform()
        transform.Translate(x,y,z)
        transformFilter=vtk.vtkTransformPolyDataFilter()
        transformFilter.SetTransform(transform)
        if vtk.VTK_MAJOR_VERSION <= 5:
            transformFilter.SetInput(self.poly_data)
        else:
            transformFilter.SetInputData(self.poly_data)
        transformFilter.Update()
        return transformFilter.GetOutput()

    def rotate(self, oldVect, newVect):
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
            transformFilter.SetInput(self.poly_data)
        else:
            transformFilter.SetInputData(self.poly_data)
        transformFilter.Update()

        return transformFilter.GetOutput()

    def scale(self, x,y,z):
        transform = vtk.vtkTransform()
        transform.Scale(x, y, z)
        transformFilter = vtk.vtkTransformPolyDataFilter()
        transformFilter.SetTransform(transform)
        if vtk.VTK_MAJOR_VERSION <= 5:
            transformFilter.SetInput(self.poly_data)
        else:
            transformFilter.SetInputData(self.poly_data)
        transformFilter.Update()
        return transformFilter.GetOutput()

    def move_to_origin(self):
        x, y, z = self.computeCenterOfMass()
        moved_poly_data = self.translate(-x, -y, -z)
        return moved_poly_data

    def computePCA(self):
        xArray = vtk.vtkDoubleArray()
        xArray.SetNumberOfComponents(1)
        xArray.SetName('x')
        yArray = vtk.vtkDoubleArray()
        yArray.SetNumberOfComponents(1)
        yArray.SetName('y')
        zArray = vtk.vtkDoubleArray()
        zArray.SetNumberOfComponents(1)
        zArray.SetName('z')

        for i in range (0,self.poly_data.GetNumberOfPoints()):
            pt = self.poly_data.GetPoint(i)
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


    def computeCenterOfMass(self):
        centerOfMassFilter = vtk.vtkCenterOfMass()
        if vtk.VTK_MAJOR_VERSION<=5:
            centerOfMassFilter.SetInput(self.poly_data)
        else:
            centerOfMassFilter.SetInputData(self.poly_data)
        centerOfMassFilter.SetUseScalarsAsWeights(False)
        centerOfMassFilter.Update()

        return centerOfMassFilter.GetCenter()#(center)

    # crop the mesh, keep what is inside the box
    def cropMesh(self,xmin,xmax,ymin,ymax,zmin,zmax):
        cube = vtk.vtkCubeSource()
        cube.SetBounds(xmin,xmax,ymin,ymax,zmin,zmax)
        cube.Update()

        tri2 = vtk.vtkTriangleFilter()
        tri2.SetInputData(cube.GetOutput())
        tri2.Update()

        tri = vtk.vtkTriangleFilter()
        tri.SetInputData(self.poly_data)
        tri.Update()

        inter = vtk.vtkIntersectionPolyDataFilter()
        inter.SetInputData(0,tri.GetOutput())
        inter.SetInputData(1,tri2.GetOutput())
        inter.Update()

        return inter.GetOutput()


        # box = vtk.vtkBox()
        # box.SetBounds(xmin,xmax,ymin,ymax,zmin,zmax)


        # pdNormals = vtk.vtkPolyDataNormals()
        # pdNormals.SetInputData(self.poly_data)

        # clipper = vtk.vtkClipPolyData()
        # clipper.SetInputConnection(pdNormals.GetOutputPort())
        # clipper.SetClipFunction(box)
        # clipper.GenerateClippedOutputOn()
        # clipper.InsideOutOff()
        # clipper.Update()
        # return clipper.GetClippedOutput()

    def meshTo3DImg(self):
        whiteImage = vtk.vtkImageData()
        bounds = self.poly_data.GetBounds()
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
            pol2stenc.SetInput(self.poly_data)
        else:
            pol2stenc.SetInputData(self.poly_data)

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

    def nbPointsInsideBox(self,xmin,xmax,ymin,ymax,zmin,zmax):
        box = vtk.vtkCubeSource()
        box.SetBounds(xmin,xmax,ymin,ymax,zmin,zmax)
        box.Update()

        selectEnclosedPoints = vtk.vtkSelectEnclosedPoints()
        if vtk.VTK_MAJOR_VERSION <= 5:
            selectEnclosedPoints.SetInput(self.poly_data)
            selectEnclosedPoints.SetSurface(box.GetOutput())
        else:
            selectEnclosedPoints.SetInputData(self.poly_data)
            selectEnclosedPoints.SetSurfaceData(box.GetOutput())

        selectEnclosedPoints.Update()

        nbOfPointsInside = 0
        for i in range (0, self.poly_data.GetNumberOfPoints()):
            if selectEnclosedPoints.IsInside(i):
                nbOfPointsInside = nbOfPointsInside + 1

        return nbOfPointsInside 

 

    def extractEdges(self):
        vextractEdges = vtk.vtkFeatureEdges()
        vextractEdges.FeatureEdgesOn()
        vextractEdges.BoundaryEdgesOff()
        vextractEdges.ColoringOn()
        vextractEdges.SetInputData(self.poly_data)
        vextractEdges.Update()    
        return vextractEdges.GetOutput()


    def changePolyData(self, polydata):
        self.poly_data = polydata    