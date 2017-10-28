import vtk
import math

class RayCasting:
    def __init__(self, *args, **kwargs):
        self.poly_data = kwargs.get("poly_data")

    def findPoints(self,center,isYMax):
        vmaths = vtk.vtkMath()
        points = vtk.vtkPoints()
        bounds = self.poly_data.GetBounds()
        firstCorner = [bounds[0],bounds[3],center[2]]
        thetaInit = vmaths.AngleBetweenVectors([0.0,1.0,0.0],[bounds[0]-center[0],bounds[3]-center[1],0.0])
        angleStep = vmaths.RadiansFromDegrees(1.0)
        for i in range (0,240):
            pt = self._rayCast(center,[math.sin(thetaInit+angleStep*i),math.cos(thetaInit+angleStep*i),center[2]],isYMax)
            if pt != -1:
                points.InsertNextValue(pt)
                print pt
        return points

    def _rayCast(self,center,point1,isYMax):
        # perform ray casting
        plane = vtk.vtkPlaneSource()
        plane.SetCenter(center)
        plane.SetPoint1(point1)
        p = [center[0],center[1],center[2]+10]
        plane.SetPoint2(p)
        selectEnclosedPoints = vtk.vtkSelectEnclosedPoints()
        if vtk.VTK_MAJOR_VERSION <= 5:
            selectEnclosedPoints.SetInput(self.poly_data)
            selectEnclosedPoints.SetSurface(plane.GetOutput())
        else:
            selectEnclosedPoints.SetInputData(self.poly_data)
            selectEnclosedPoints.SetSurfaceData(plane.GetOutput())

        selectEnclosedPoints.Update()

        vmaths = vtk.vtkMath()

        v1 = [center[0]-point1[0],center[1]-point1[1]]
        points = vtk.vtkPoints()
        for i in range (0, self.poly_data.GetNumberOfPoints()):
            if selectEnclosedPoints.IsInside(i):
                pt = self.poly_data.GetPoint(i)
                v2 = [center[0]-pt[0],center[1]-pt[1]]
                if vmaths.Dot2D(v1,v2)>0: #for some cases there can be 2 surfaces hits, take only the one in the right direction
                    points.InsertNextValue(self.poly_data.GetPoint(i))
        if points.GetNumberOfPoints()==0:
            return -1
        # find the min or max height points among the found points
        idMinMax = 0
        if isYMax==True:
            valMax = points.GetPoint(0)
            for i in range (0, points.GetNumberOfPoints()):
                p = points.GetPoint(i)
                if p[2]>valMax:
                    valMax = p[2]
                    idMinMax = i
        else :
            valMin = points.GetPoint(0) 
            for i in range (0, points.GetNumberOfPoints()):
                p = points.GetPoint(i)
                if p[2]<valMin:
                    valMin = p[2]
                    idMinMax = i
        return points.GetPoint(idMinMax)
