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

    def findPoints(self,center):
        vmaths = vtk.vtkMath()
        pts = [None]*360
        bounds = self.poly_data.GetBounds()
        angleStep = vmaths.RadiansFromDegrees(-1.0)
        length = bounds[3]-bounds[2]
        nbPts = 0
        nbRaysOk = 0
        lastRay = 0
        # perform ray casting and store it in pts
        for i in range (0,360):
            print i
            pt = self._rayCast(center,
                               [-length * math.sin(angleStep * i), -length * math.cos(angleStep * i), center[2]])
            if pt != -1:
                pts[i] = pt
                nbPts = nbPts + len(pt)
                nbRaysOk = nbRaysOk + 1
                lastRay = i
            else :
                pts[i]=-1 #no found point

        # compute distances
        dists = np.zeros(shape=(nbPts, nbPts))
        indexInit = -1
        i = 0
        ptIds = np.zeros(shape=(nbPts,2),dtype=np.uint32) # stores ray nb and pt index in ray, index = id in dists
        current = 0
        idmap = [None]*360 # idmap : stores the id (in dist) for each point (i corresponds to ray, j to pt index in ray)

        #find first point(s)
        while indexInit == -1 and i<360:
            pt = pts[i]
            if pt!=-1:  #found first point, go out of loop and stores characteristics of point
                indexInit = i
                previousPtsDistIdx = np.arange(len(pt))
                prepreviousPtsDistIdx = previousPtsDistIdx
                previousRay = indexInit
                prepreviousRay = previousRay
                currentIIndices = [0]*len(pt)
                for j in range(0, len(pt)):
                    ptIds[current, 0] = i
                    ptIds[current, 1] = j
                    currentIIndices[j] = current
                    current = current + 1
                idmap[i] = currentIIndices
            i = i + 1
        stop = False
        gaps = vtk.vtkIdList()
        for i in range(indexInit+1, lastRay):
            if pts[i] != -1 and not stop:
                currentPts = pts[i]
                previousPts = pts[previousRay]
                prepreviousPts = pts[prepreviousRay]
                minDist = 100
                maxDist = 0
                currentIIndices = [0]*len(currentPts)
                #compute distances and store them in distance matrix
                for j in range (0, len(currentPts)):
                    for k in range (0,previousPtsDistIdx.size):
                        currentDist = self.distPt(np.asarray(currentPts[j]),np.asarray(previousPts[k]))
                        dists[previousPtsDistIdx[k], current] = currentDist
                        if currentDist<minDist:
                            minDist = currentDist
                        if currentDist>maxDist:
                            maxDist=currentDist
                    #compute distances with points before previous points
                    for k in range (0,prepreviousPtsDistIdx.size):
                        currentDist = self.distPt(np.asarray(currentPts[j]),np.asarray(prepreviousPts[k]))
                        dists[prepreviousPtsDistIdx[k], current] = currentDist

                    currentIIndices[j] = current
                    ptIds[current, 0] = i
                    ptIds[current, 1] = j
                    current = current + 1
                if maxDist>1000 and i-indexInit>265:
                    stop = True
                    print 'stop'
                    lastRay = i #this was the last ray
                elif minDist>0.15:
                    for j in range (0, previousPtsDistIdx.size):
                        gaps.InsertNextId(i)
                    #print "gap ",i

                prepreviousPtsDistIdx = previousPtsDistIdx
                previousPtsDistIdx = np.arange(current-len(currentPts),current)
                prepreviousRay = previousRay
                previousRay = i
                idmap[i] = currentIIndices
        current = current-1


        #handle gaps : if there is a gap, add links from point to until points from ray 7 rays after the current point's ray
        for i in range (0, gaps.GetNumberOfIds()):
            gapRay = gaps.GetId(i)
            previousPts = pts[gapRay]
            for j in range (gapRay+1,min(gapRay+8,lastRay)):
                currentPts = pts[j]
                if currentPts!=-1:
                    for k in range (0,len(currentPts)):
                        for l in range (0, len(previousPts)):
                            currentDist = self.distPt(np.asarray(currentPts[k]), np.asarray(previousPts[l]))
                            dists[idmap[gapRay][l], idmap[j][k]] = currentDist #TODO


        #compute shortest path
        dist_matrix, predecessors = shortest_path(dists,method="D",directed=True,return_predecessors=True)

        #find optima (which of the potential several init and end points should be taken :
        # the ones that minimize the total distance
        minDist = dist_matrix[0,current]
        optima = (0,current)
        for i in range (0,len(pts[indexInit])):
            for j in range(current-len(pts[lastRay]),current):
                curDist = dist_matrix[i,j]
                if curDist<minDist:
                    optima = (i,j)

        points = vtk.vtkPoints()
        current = optima[1]
        i = 0
        # loop in the predecessor list to create the points
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
            maxDist = 0
            idMax = 0
            firstCorner = [bounds[0], bounds[2], center[2]]
            # thetaInit = vmaths.AngleBetweenVectors([0.0,1.0,0.0],[bounds[0],bounds[2],0.0])
            angleStep = vmaths.RadiansFromDegrees(-1.0)
            length = bounds[3] / 2 - bounds[2]
            lastPt = -1
            idx = 0
            for i in range(0, 360):
                print i
                pt = self._rayCastOld(center,
                                   [-length * math.sin(angleStep * i), -length * math.cos(angleStep * i), center[2]])
                if pt != -1:
                    points.InsertNextPoint(pt)
                    idx = idx+1
                    if lastPt!=-1:
                        currentDist = self.distPt(np.asarray(pt),np.asarray(lastPt))
                        if currentDist>maxDist:
                            maxDist = currentDist
                            idMax = idx
                    lastPt = pt
            if self.distPt(np.asarray(lastPt),np.asarray(points.GetPoint(0)))>maxDist:
                idMax=0

            #reorder
            finalPoints = vtk.vtkPoints()
            for i in range(0, points.GetNumberOfPoints()):
                finalPoints.InsertNextPoint(points.GetPoint((idMax+i)%points.GetNumberOfPoints()))
            return finalPoints

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

    def _rayCastOld(self,center,point1):
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

        if points.GetNumberOfPoints()==0:
            return -1

        idMax = 0
        valMax = points.GetPoint(0)[2]
        for i in range(0, points.GetNumberOfPoints()):
            p = points.GetPoint(i)
            if p[2] > valMax:
                valMax = p[2]
                idMax = i

        return points.GetPoint(idMax)


    def _rayCast(self,center,point1):
        '''
        perform ray casting
        throw a plane through mesh passing through given points (center, point1)
        then compute the intersection between mesh and plane
        cluster the intersected points and compute the points with highest z value for each cluster
        :param center: point through which the plane must go
        :param point1: point through which the plane must go
        :return: the max height points for each cluster
        '''
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


        if points.GetNumberOfPoints()==0:
            return -1
        # if shouldPrint:
        #     rend = renderer.Renderer()
        #     rend.add_actor(self.poly_data, color=[1, 1, 1], wireframe=False)
        #     rend.add_actor(plane.GetOutput(), color=[0, 0, 1], wireframe=False)
        #     rend.render()

        pts = np.zeros(shape=(points.GetNumberOfPoints(), 3))
        for i in range(0, points.GetNumberOfPoints()):
            pts[i,:] = np.asarray(points.GetPoint(i))

        # Compute DBSCAN to cluster the points along rays into clusters
        db = DBSCAN(eps=1).fit(pts)
        core_samples_mask = np.zeros_like(db.labels_, dtype=bool)
        core_samples_mask[db.core_sample_indices_] = True
        labels = db.labels_

        # labels with noise had -1 value, add 1 and ignore 0
        labels = labels+1

        y = np.bincount(labels)
        ii = np.nonzero(y)[0] #ignore noise
        ans = [None]*len(ii)
        ansIdx = 0
        for i in ii: # for each non noisy cluster, compute max
            newPoints = np.zeros(shape=(points.GetNumberOfPoints(),3))
            idx = 0

            #get points in current cluster
            for j in range (0,points.GetNumberOfPoints()):
                if labels[j]==i:
                    newPoints[idx,:] = pts[j,:]
                    idx = idx+1


            # find the max height points among the found points
            idMax = 0
            valMax = newPoints[0,2]
            for j in range (0, idx):
                p = newPoints[j,:]
                if p[2]>valMax:
                    valMax = p[2]
                    idMax = j
            ans[ansIdx]=newPoints[idMax,:]
            ansIdx = ansIdx+1
        return ans