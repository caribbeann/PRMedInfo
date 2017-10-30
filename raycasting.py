import vtk
import math

class RayCasting:
    def __init__(self, *args, **kwargs):
        self.poly_data = kwargs.get("poly_data")
        tri = vtk.vtkTriangleFilter()
        tri.SetInputData(self.poly_data)
        tri.Update()
        self.tripoly = tri.GetOutput()

    def findPoints(self,center,maxZ):
        vmaths = vtk.vtkMath()
        points = vtk.vtkPoints()
        bounds = self.poly_data.GetBounds()
        firstCorner = [bounds[0],bounds[3],center[2]]
        thetaInit = vmaths.AngleBetweenVectors([0.0,1.0,0.0],[bounds[0]-center[0],bounds[3]-center[1],0.0])
        angleStep = vmaths.RadiansFromDegrees(1.0)
        length = bounds[3]-bounds[2]-(bounds[3]-center[1])/2
        for i in range (0,300):
            if i!=127 and i!=128:
                pt = self._rayCast(center,[center[0]+length*math.sin(thetaInit+angleStep*i),center[1]+length*math.cos(thetaInit+angleStep*i),center[2]],maxZ)
                if pt != -1:
                    points.InsertNextPoint(pt)
                    print (i,pt)
                else:
                    print ":(" 
            else:
                print i
        return points


    def _rayCast(self,center,point1,maxZ):
        # perform ray casting
        plane = vtk.vtkPlaneSource()
        plane.SetCenter(center)
        plane.SetPoint1(point1)
        p = [center[0],center[1],self.poly_data.GetBounds()[4]]
        plane.SetPoint2(p)
        plane.Update()

        tri2 = vtk.vtkTriangleFilter()
        tri2.SetInputData(plane.GetOutput())
        tri2.Update()

        inter = vtk.vtkIntersectionPolyDataFilter()
        inter.SetInputData(0,self.tripoly)
        inter.SetInputData(1,tri2.GetOutput())
        inter.Update()

        vmaths = vtk.vtkMath()

        points = inter.GetOutput()

        if points.GetNumberOfPoints()==0:
            return -1
        # find the min or max height points among the found points
        idMinMax = 0
        if maxZ==True: #maximize Z
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
