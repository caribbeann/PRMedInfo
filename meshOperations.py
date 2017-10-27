import vtk

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
        center = [0.0,0.0,0.0]
        return centerOfMassFilter.GetCenter()#(center)

    def cropMesh(self,xmin,xmax,ymin,ymax,zmin,zmax):
        box = vtk.vtkBox()
        box.SetBounds(xmin,xmax,ymin,ymax,zmin,zmax)


        pdNormals = vtk.vtkPolyDataNormals()
        pdNormals.SetInputData(self.poly_data)

        clipper = vtk.vtkClipPolyData()
        clipper.SetInputConnection(pdNormals.GetOutputPort())
        clipper.SetClipFunction(box)
        clipper.GenerateClippedOutputOn()
        clipper.InsideOutOff()
        clipper.Update()
        return clipper.GetClippedOutput()