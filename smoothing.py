import numpy as np
import meshOperations
import vtk
import scipy.interpolate as si

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
        for i in range (0,polydata.GetNumberOfPoints()):
            npPoints[i]=polydata.GetPoint(i)
        for j in range (0,nbIter):
            self.putPointsCloser(npPoints,threshold)
        return npPoints

    def interpolate(self,polydata):
        points = vtk.vtkPoints()
        indexInit = -1
        i = 0
        #print 'interpolate'
        while indexInit == -1 and i<polydata.GetNumberOfPoints():
            pt = polydata.GetPoint(i)
            if pt[0]!=0.0 and pt[1]!=0.0 and pt[2]!=0.0:
                indexInit = i
                points.InsertNextPoint(pt)
            i = i + 1
        #print indexInit
        # line = np.zeros(shape=(2,points.GetNumberOfPoints()))
        #
        # for i in range (0,points.GetNumberOfPoints()):
        #     pt = points.GetPoint(i)
        #     line[0,i] = pt[0]
        #     line[1,i] = pt[1]
        #
        #
        # tck,u = si.splprep(line,k=3, s=10000, nest=3)#, nest=15)
        # new_points = si.splev(u, tck)
        lastIndex = indexInit
        while i < polydata.GetNumberOfPoints():
            pt = polydata.GetPoint(i)
            if pt[0] != 0.0 and pt[1] != 0.0 and pt[2] != 0.0:
                if lastIndex < i-1: #some points lie between, interpolate them
                    print (lastIndex,i, polydata.GetPoint(lastIndex), pt)
                    lastPoint = polydata.GetPoint(lastIndex)
                    currentMinusLast = [pt[0]-lastPoint[0],pt[1]-lastPoint[1]]
                    middleInterpolated = [(lastPoint[0] + pt[0]- currentMinusLast[1])/4,
                                          (lastPoint[1]+ pt[1]+currentMinusLast[0])/4]
                    x = [lastPoint[0],middleInterpolated[0],pt[0]]
                    y = [lastPoint[1], middleInterpolated[1], pt[1]]
                    print (x,y)
                    tck = si.splrep(x, y,k=2)
                    xnew = np.linspace(lastPoint[0], pt[0], num=i +1 - lastIndex, endpoint=True)
                    ynew = interpolate.splev(xnew, tck)
                    for j in range (1, i - lastIndex):
                        points.InsertNextPoint([xnew[j],ynew[j],pt[2]])
                points.InsertNextPoint(pt)
                lastIndex = i
            # else:
            #     print i
            i = i + 1
        return points



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
        return meshOp.numpy_array_to_poly_data(npPoints)

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
        points = self.interpolate(polydata)
        npPoints = self.smooth(points,nbIter,threshold)
        moys = np.zeros(shape=(points.GetNumberOfPoints(),3))
        for i in range (0,points.GetNumberOfPoints()):
            firstPt = np.asarray(points.GetPoint(i))
            dist = self.distPt(npPoints[i],firstPt)
            if dist>1:
                moy = npPoints[i]+(firstPt-npPoints[i])/dist
            else:   
                moy = npPoints[i]+(firstPt-npPoints[i])/2
            moys[i]=moy
        meshOp = meshOperations.MeshOperations()
        return meshOp.numpy_array_to_poly_data(moys)

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
        return meshOp.numpy_array_to_poly_data(moys)

