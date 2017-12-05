import vtk
import math
import renderer
from sklearn.cluster import DBSCAN
import numpy as np
import renderer
from scipy.sparse.csgraph import shortest_path

class RayCasting:
    def __init__(self, *args, **kwargs):
        self.poly_data = kwargs.get("poly_data")
        tri = vtk.vtkTriangleFilter()
        tri.SetInputData(self.poly_data)
        tri.Update()
        self.tripoly = tri.GetOutput()
        self.bounds = self.poly_data.GetBounds()

    def distPt(self,pt1,pt2):
        """
        Compute the squared distance between points pt1 and pt2
        :param pt1: first point (numpy array)
        :param pt2: second point (numpy array)
        :return: the distance between the points
        """
        pt12 = pt1-pt2
        return np.dot(pt12,pt12)

    def findPoints(self,center):#, thresholdNb):
        vmaths = vtk.vtkMath()
        pts = [None]*360#np.zeros(shape=(360, 4))
        bounds = self.poly_data.GetBounds()
        angleStep = vmaths.RadiansFromDegrees(-1.0)
        length = bounds[3]/2-bounds[2]
        nbPts = 0
        nbRaysOk = 0
        for i in range (0,360):
            print i
            pt = self._rayCast(center,[-length*math.sin(angleStep*i),-length*math.cos(angleStep*i),center[2]])
            if pt != -1:
                pts[i] = pt
                nbPts = nbPts + len(pt)
                nbRaysOk = nbRaysOk + 1
            else :
                pts[i]=-1

        # compute distances
        dists = np.zeros(shape=(nbPts, nbPts))
        indexInit = -1
        i = 0
        ptIds = np.zeros(shape=(nbPts,2),dtype=np.uint64) #idmap
        current = 0
        #find first point(s)
        while indexInit == -1 and i<360:
            pt = pts[i]
            if pt!=-1:
                indexInit = i
                currentIdx = len(pt)
                lastPtsDistIdx = np.arange(currentIdx)
                lastRay = indexInit
                for j in range(0, len(pts[i])):
                    ptIds[current, 0] = i
                    ptIds[current, 1] = j
                    current = current + 1
            i = i + 1
        stop = False
        for i in range(indexInit+1, 360):
            if pts[i] != -1 and not stop:
                currentPts = pts[i]
                lastPts = pts[lastRay]
                #compute distances and store them in distance matrix
                for j in range (0, len(currentPts)):
                    for k in range (0,lastPtsDistIdx.size):
                        currentDist = self.distPt(np.asarray(currentPts[j]),np.asarray(lastPts[k]))
                        dists[lastPtsDistIdx[k], current] = currentDist
                        if currentDist>10 and i-indexInit>265:
                            stop = True
                            print 'stop'

                    ptIds[current, 0] = i
                    ptIds[current, 1] = j
                    current = current + 1

                lastPtsDistIdx = np.arange(current-len(currentPts),current)
                lastRay = i
        current = current-1
        #compute shortest path
        dist_matrix, predecessors = shortest_path(dists,method="D",directed=True,return_predecessors=True)
        minDist = dist_matrix[0,current]
        optima = (0,current)
        for i in range (0,len(pts[indexInit])):
            for j in range(current-len(pts[lastRay]),current):
                curDist = dist_matrix[i,j]
                if curDist<minDist:
                    optima = (i,j)

        #print predecessors[optima[0],:]

        points = vtk.vtkPoints()
        current = optima[1]
        i = 0
        while current!=optima[0]:
            pt = pts[ptIds[current, 0]][ptIds[current, 1]]
            points.InsertNextPoint([pt[0],pt[1],pt[2]])
            current = predecessors[optima[0],current]
            i = i + 1
        print lastRay-indexInit
        return points

    def findPoints2(self,line,maxZ,windowSize):
        vmaths = vtk.vtkMath()
        points = vtk.vtkPoints()
        bounds = self.poly_data.GetBounds()
        
        for i in range (0,line.GetNumberOfPoints()-1):
            currentPt = line.GetPoint(i)
            nextPt = line.GetPoint(i+1)
            currentLine = [currentPt[0]-nextPt[0],currentPt[1]-nextPt[1]]
            normal = [currentLine[1],-currentLine[0],0]
            norm = vmaths.Norm(normal)
            normal[0]=normal[0]/norm
            normal[1]=normal[1]/norm
            center = [currentPt[0],currentPt[1],bounds[5]]
            pt1 = [currentPt[0]+windowSize*normal[0],currentPt[1]+windowSize*normal[1],bounds[5]]
            pt = self._rayCast(center,pt1,maxZ)
            if pt != -1:
                points.InsertNextPoint(pt)
                print i
            else:
                print ":'("    
        return points

    def findPointsOld(self, center):
            vmaths = vtk.vtkMath()
            points = vtk.vtkPoints()
            bounds = self.poly_data.GetBounds()
            firstCorner = [bounds[0], bounds[2], center[2]]
            # thetaInit = vmaths.AngleBetweenVectors([0.0,1.0,0.0],[bounds[0],bounds[2],0.0])
            angleStep = vmaths.RadiansFromDegrees(-1.0)
            length = bounds[3] / 2 - bounds[2]
            for i in range(0, 360):
                print i
                pt = self._rayCast(center,
                                   [-length * math.sin(angleStep * i), -length * math.cos(angleStep * i), center[2]])
                if pt != -1:
                    points.InsertNextPoint(pt)
                    # else :
                    #    points.InsertNextPoint([0.0,0.0,0.0])
            return points

    # def goUpward(self,line):
    #     points = vtk.vtkPoints()
    #     for i in range (0,line.GetNumberOfPoints()):
    #         currentPt = line.GetPoint(i)
    #         pt1 = [currentPt[0],currentPt[1],self.bounds[4]]
    #         lineSrc = vtk.vtkLineSource()
    #         lineSrc.SetPoint1(pt1)
    #         pt2 = [currentPt[0],currentPt[1],self.bounds[5]]
    #         lineSrc.SetPoint2(pt2) 
    #         lineSrc.Update()

    #         tri2 = vtk.vtkTriangleFilter()
    #         tri2.SetInputConnection(lineSrc.GetOutputPort())
    #         tri2.Update()

    #         inter = vtk.vtkIntersectionPolyDataFilter()
    #         inter.SetInputData(0,self.tripoly)
    #         inter.SetInputData(1,tri2.GetOutput())
    #         inter.Update()

    #         if inter.GetOutput().GetNumberOfPoints()!=0:
    #             points.InsertNextPoint(inter.GetOutput().GetPoint(0))
    #         else:
    #             print ":'("
    #     return points            

    def _constrainedRayCast(self,center,point1):
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
        inter.SetSplitFirstOutput(0)
        inter.SetSplitSecondOutput(0)
        inter.Update()

        points = inter.GetOutput()

        print points.GetNumberOfPoints()

        if points.GetNumberOfPoints()==0:
            return -1

        idMax = 0
        if maxZ == True:  # maximize Z
            valMax = points.GetPoint(0)[2]
            for i in range(0, points.GetNumberOfPoints()):
                p = points.GetPoint(i)
                if p[2] > valMax:
                    valMax = p[2]
                    idMax = i

        return points.GetPoint(idMax)


    def _rayCast(self,center,point1):
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
        inter.SetSplitFirstOutput(0)
        inter.SetSplitSecondOutput(0)
        inter.Update()

        points = inter.GetOutput()

        #print points.GetNumberOfPoints()

        if points.GetNumberOfPoints()==0:
            return -1

        pts = np.zeros(shape=(points.GetNumberOfPoints(), 3))
        for i in range(0, points.GetNumberOfPoints()):
            pts[i,:] = np.asarray(points.GetPoint(i))
        #print pts
        # Compute DBSCAN
        db = DBSCAN(eps=1).fit(pts)
        core_samples_mask = np.zeros_like(db.labels_, dtype=bool)
        core_samples_mask[db.core_sample_indices_] = True
        labels = db.labels_

        #print labels
        labels = labels+1
        #print labels

        y = np.bincount(labels)
        ii = np.nonzero(y)[0]
        ans = [None]*len(ii)
        #print y
        #print ii
        ansIdx = 0
        for i in ii:

            newPoints = np.zeros(shape=(points.GetNumberOfPoints(),3))
            idx = 0
            #get points
            for j in range (0,points.GetNumberOfPoints()):
                if labels[j]==i:
                    newPoints[idx,:] = pts[j,:]
                    idx = idx+1

            #print newPoints
            #print db.labels_

            # Number of clusters in labels, ignoring noise if present.
            n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)

            #print('Estimated number of clusters: %d' % n_clusters_)

            # rend = renderer.Renderer()
            # rend.add_actor(self.poly_data, color=[1, 1, 1], wireframe=False)
            # rend.add_actor(plane.GetOutput(), color=[0, 0, 1], wireframe=False)
            # rend.render()


            # find the min or max height points among the found points
            idMax = 0
            valMax = newPoints[0,2]
            for j in range (0, idx):#points.GetNumberOfPoints()):
                #p = points.GetPoint(i)
                p = newPoints[j,:]
                if p[2]>valMax:
                    valMax = p[2]
                    idMax = j
            #print ans
            #print newPoints[idMax,:]
            ans[ansIdx]=newPoints[idMax,:]
            ansIdx = ansIdx+1
        return ans#newPoints[idMax,:]#,newPoints[idMax,1],newPoints[idMax,2]]#points.GetPoint(idMax)
