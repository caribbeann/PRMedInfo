import vtk
import numpy as np
import vtkpointcloud

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
        dists = np.zeros(nbPts)
        for i in range (0,nbPts-1):
            pt1 = pts[i]
            pt2 = pts[i+1]
            val = self.distPt(pt1,pt2)
            dists[i]=val
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

    def polyDataSmoothed(self,nbIter,threshold):
        bounds = self.poly_data.GetBounds()
        pointCloud = vtkpointcloud.VtkPointCloud(zMin=bounds[4],zMax=bounds[5])
        npPoints = self.smooth(nbIter,threshold)
        for i in range (0,self.poly_data.GetNumberOfPoints()):
            pt = [npPoints[i,0],npPoints[i,1],npPoints[i,2]]
            pointCloud.addPoint(pt)
        return pointCloud.vtkPolyData

    def weightedCombination(self,nbIter,threshold):
        vmaths = vtk.vtkMath()
        bounds = self.poly_data.GetBounds()
        pointCloud = vtkpointcloud.VtkPointCloud(zMin=bounds[4],zMax=bounds[5])
        npPoints = self.smooth(nbIter,threshold)
        dists = np.zeros(self.poly_data.GetNumberOfPoints())
        for i in range (0,self.poly_data.GetNumberOfPoints()):
            pt = [npPoints[i,0],npPoints[i,1],npPoints[i,2]]
            firstPt = self.poly_data.GetPoint(i)
            dists[i] = vmaths.Distance2BetweenPoints(pt,firstPt)
            if dists[i]>1:
                moy = [pt[0]+(firstPt[0]-pt[0])/dists[i], pt[1]+(firstPt[1]-pt[1])/dists[i] ,pt[2]]
                pointCloud.addPoint(moy)
            else:   
                moy = [pt[0]+(firstPt[0]-pt[0])/2, pt[1]+(firstPt[1]-pt[1])/2 ,pt[2]] 
                pointCloud.addPoint(moy)
        return pointCloud.vtkPolyData     

    def meanWeightedCombination(self,nbIter,threshold):
        vmaths = vtk.vtkMath()
        bounds = self.poly_data.GetBounds()
        pointCloud = vtkpointcloud.VtkPointCloud(zMin=bounds[4],zMax=bounds[5])
        npPoints = self.smooth(nbIter,threshold)
        dists = np.zeros(self.poly_data.GetNumberOfPoints())
        for i in range (0,firstTest.GetNumberOfPoints()):
            pt = [npPoints[i,0],npPoints[i,1],npPoints[i,2]]
            firstPt = self.poly_data.GetPoint(i)
            dists[i] = vmaths.Distance2BetweenPoints(pt,firstPt)
            if dists[i]<2:
                moy = [pt[0]+(firstPt[0]-pt[0])/2, pt[1]+(firstPt[1]-pt[1])/2 ,pt[2]]
                pointCloud.addPoint(moy)
            else:   
                pointCloud.addPoint(pt)
        return pointCloud.vtkPolyData     

    def changePolyData(self,polydata):
        self.poly_data = polydata            

