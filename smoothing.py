import vtk
import numpy as np

class Smoothing:
    def __init__(self, *args, **kwargs):
        self.poly_data = kwargs.get("poly_data")
        tri = vtk.vtkTriangleFilter()
        tri.SetInputData(self.poly_data)
        tri.Update()
        self.tripoly = tri.GetOutput()

    def distPt(self,pt1,pt2):
        pt12 = pt1-pt2
        return np.dot(pt12,pt12)

    def putPointsCloser(self,pts,threshold):
        (nbPts,x)=pts.shape
        for i in range (0,nbPts-1):
            pt1 = pts[i]
            pt2 = pts[i+1]
            val = self.distPt(pt1,pt2)
            if val > threshold:
                pt12 = (pt2-pt1)/4
                newPt1 = pt1+pt12
                newPt2 = pt2-pt12
                pts[i]=newPt1
                pts[i+1]=newPt2

    def smooth(self,nbIter,threshold):
        npPoints = np.zeros(shape=(self.poly_data.GetNumberOfPoints(),3))
        npPoints[0]=self.poly_data.GetPoint(0)
        for i in range (0,self.poly_data.GetNumberOfPoints()-1):
            npPoints[i+1]=self.poly_data.GetPoint(i+1)
        for j in range (0,nbIter):
            self.putPointsCloser(npPoints,threshold)#[idBiggestComponent:idEndDisc],np.mean(vals))
        return npPoints 