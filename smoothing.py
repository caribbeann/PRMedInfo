import numpy as np
import meshOperations

class Smoothing:
    def distPt(self,pt1,pt2):
        """
        Compute the squared distance between points pt1 and pt2
        :param pt1: first point (numpy array)
        :param pt2: second point (numpy array)
        :return: the distance between the points
        """
        pt12 = pt1-pt2
        return np.dot(pt12,pt12)

    def putPointsCloser(self,pts,threshold):
        """
        Brings two following points closer if their distance is above the given threshold
        It brings both points toward one another, such as the new distance will be half the old one
        :param pts: numpy array of points to bring closer
        :param threshold: the threshold above which the points are considered as "far" from one another
        :return: a numpy array with all points such as the distance of points further than threshold are closer
        """
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

    def smooth(self,polydata,nbIter,threshold):
        """
        Performs smoothing of a polydata by bringing points that are further than the given distance
        threshold toward one another.
        :param polydata: polydata to smooth
        :param nbIter: number of smoothing iterations (number of times the points will be brought closer)
        :param threshold: distance threshold above which two following points are considered "far"
        :return: a numpy array containing the smoothed points
        """
        npPoints = np.zeros(shape=(polydata.GetNumberOfPoints(),3))
        npPoints[0]=polydata.GetPoint(0)
        for i in range (0,polydata.GetNumberOfPoints()-1):
            npPoints[i+1]=polydata.GetPoint(i+1)
        for j in range (0,nbIter):
            self.putPointsCloser(npPoints,threshold)
        return npPoints 

    def polyDataSmoothed(self,polydata,nbIter,threshold):
        """
        Performs smoothing of a polydata by bringing points that are further than the given distance
        threshold toward one another and returns a polydata
        :param polydata: polydata to smooth
        :param nbIter:number of smoothing iterations (number of times the points will be brought closer)
        :param threshold: distance threshold above which two following points are considered "far"
        :return: a new polydata containing the smoothed points
        """
        npPoints = self.smooth(polydata,nbIter,threshold)
        meshOp = meshOperations.MeshOperations()
        return meshOp.numpyArrayToPolyData(npPoints)

    def weightedCombination(self,polydata,nbIter,threshold):
        """
        Computes a smoothed version of the points with the given number of iteration and threshold
        Then computes the distance between the new smoothed point and its corresponding old point
        If this distance is above 1, computes a new point by weighting with the distance
        Then, the further the points are, the closer the new point will be to the new point
        Indeed, if they are far from each other, it means there was a strong discontinuity nearby
        :param polydata: the polydata to smooth
        :param nbIter: number of iterations for smoothing
        :param threshold: thresholding distance for smoothing (bringing points closer if they are far)
        :return: a polydata containing a weighted combination between the initial polydata and its smooth version
        """
        npPoints = self.smooth(polydata,nbIter,threshold)
        moys = np.zeros(shape=(polydata.GetNumberOfPoints(),3))
        for i in range (0,polydata.GetNumberOfPoints()):
            firstPt = np.asarray(polydata.GetPoint(i))
            dist = self.distPt(npPoints[i],firstPt)
            if dist>1:
                moy = npPoints[i]+(firstPt-npPoints[i])/dist
            else:   
                moy = npPoints[i]+(firstPt-npPoints[i])/2
            moys[i]=moy
        meshOp = meshOperations.MeshOperations()
        return meshOp.numpyArrayToPolyData(moys)

    def meanWeightedCombination(self,polydata,nbIter,threshold):
        """
        Computes a smoothed version of the points with the given number of iteration and threshold
        Then computes the distance between the new smoothed point and its corresponding old point
        If this distance is above 2, computes the mean of the two points, otherwise take smooth point
        :param polydata: the polydata to smooth
        :param nbIter: number of iterations for smoothing
        :param threshold: thresholding distance for smoothing (bringing points closer if they are far)
        :return: a polydata containing a mean weighted combination between the initial polydata and its smooth version
        """
        npPoints = self.smooth(polydata,nbIter,threshold)
        moys = np.zeros(shape=(polydata.GetNumberOfPoints()))
        for i in range (0,polydata.GetNumberOfPoints()):
            pt = [npPoints[i,0],npPoints[i,1],npPoints[i,2]]
            firstPt = polydata.GetPoint(i)
            dist = self.distPt(npPoints[i],firstPt)
            if dist<2:
                moy = pt+(firstPt-pt)/2
                moys[i] = moy
            else:   
                moys[i]=pt
        meshOp = meshOperations.MeshOperations()
        return meshOp.numpyArrayToPolyData(moys)

