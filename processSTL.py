# -*- coding: utf-8 -*-
"""
Created on Thu Jul  9 07:44:39 2020

@author: Doug
"""

import numpy as np
import itertools
import math

import importSTL


class triangle():
 
    def __init__(self, ID, normalVector, P1, P2, P3):
        self.ID = ID
        #REVERSE THE NORMAL TO HAVE INWARDS NORMALS!
        self.normalVector = -1*np.array(normalVector)
        self.P1 = np.array(P1)
        self.P2 = np.array(P2)
        self.P3 = np.array(P3)
        self.vec_12 = np.subtract(P2, P1)
        self.vec_23 = np.subtract(P3, P2)
        self.vec_31 = np.subtract(P1, P3)
        
        self.centroid = np.mean( [P1, P2, P3], axis = 0)
        self.area = np.linalg.norm(np.cross(self.vec_12, -1*self.vec_31)/2.0)
        
        self.adj_12 = ''
        self.adj_23 = ''
        self.adj_31 = ''
        
        #velocity vector of cell is described in gloal body fixed 
        #coordinate system
        self.velocity = np.zeros((3,1),dtype=float)
        self.pressure = 0.0
        self.density = 0.0
        self.temperature = 0.0
        
        self.flowIncoming = []

    def adjCells(self):
        return [self.adj_12, self.adj_23, self.adj_31]
        
    
    def isLeadingEdge(self):
        """
        ???
        """
        
        
        
        
        #test whether one edge of the cell is on the edge of the craft
        #then test whether this edge is directly in line with oncoming flow
        #or whether the edge is behind another part of the vehicle meaning that
        #it does not directly see the oncoming flow
        
        #how to test whether it is an edge??
        #something to do with adjacent cell normal vectors. 
        #If they have an angle larger than pi/2??
        #again can create a local coordinate system and rotation matrix to find 
        #the atan2 rtesult to estimate whthere its a compression or wtv
        #best to do this for angles between all cells for flow turning
        #and then if a great angle is found, 
        
        
        
        
        return
    
    
    def updateVelocity(self):
        #based of adjacent cells with incoming flows
        #will create a linear interpolation between incoming values based off 
        #centroid distances
        
        
        return
    
    def updatePressure(self):
        return

    def forceTriangle(self):
        return self.pressure*self.area
    
    def checkFlowsIncoming(self):
        #will check all the adjacent cells and find what flows are incoming
        #will return a list of the object handles which have incoming 
        #velocities
        for obj in self.adjCells():
            self.flowIncoming(self, obj)
        return
    
    def flowIncoming(self, objectHandle):
        #based off another triangle object, 
        #uses it's properties to determine whether the velocity from adjacent
        #triangle is incoming or not
        
        #if adacent object has no velocity data, ignore it.
        if np.sum(objectHandle.velocity) == 0:
            return
      
        #find which side it is adjacent on and use the vectors from centroid
        #to the outer nodes to create roation matrix
        if self.adj_12 == objectHandle.ID:
            A = (self.P1 - self.centroid)
            B = (self.P2 - self.centroid)         
        elif self.adj_23 == objectHandle.ID:
            A = (self.P2 - self.centroid)
            B = (self.P3 - self.centroid)        
        elif self.adj_31 == objectHandle.ID:
            A = (self.P3 - self.centroid)
            B = (self.P1 - self.centroid)
        
        #Order normal vectors for new coordinates in plane of triangle object
        #x prime is the normal vector 
        x_prime = (B - self.centroid)/np.linalg.norm(B - self.centroid)
        #z_prime
        z_prime = self.normalVector
        #y_vector - defined as the cross product of the other two
        yp = np.cross(z_prime, x_prime)
        y_prime = yp/np.linalg.norm(yp)
        #setup the rotation matrix based of the vectors
        R = np.hstack( ( x_prime.transpose(), y_prime.transpose(), 
                         z_prime.transpose() ) )
        #convert A vector and velocity vector to object coordinate system       
        A = R*B
        V = R*objectHandle.velocity
        thetaBA = math.atan2(A[1], A[0])
        thetaBV = math.atan2(V[1], V[0])
        if (thetaBA > 0) and (thetaBV > 0) and (thetaBV < thetaBA):
            #both positive rotations and velocity is in between
            if objectHandle not in self.flowIncoming:
                self.flowIncoming.append(objectHandle)
        elif (thetaBA < 0) and (thetaBV < 0) and (thetaBV > thetaBA):
            #both negative and velocity is closer to x axis
            if objectHandle not in self.flowIncoming:
                self.flowIncoming.append(objectHandle)    
        #add a process to remove a flow in the case a REcalculation is performed
        #and the velocity vectors changes such that object is now longer incoming
        else:
            if objectHandle in self.flowIncoming:
                self.flowIncoming.pop(self.flowIncoming.index(objectHandle))
        return  
    

def processSTL(filename):
    '''
    iterates through stl file data and generates triangle class objects
    
    '''
    trianglesData = importSTL.ASCIIorBinary(filename)
    
    triangleObjects = [triangle(i, trianglesData[i][0], trianglesData[i][1], 
            trianglesData[i][2], trianglesData[i][3]) 
            for i in range(len(trianglesData))]

    #not a class method, but a method that operates on all of the objects in 
    #that class. actually unsure here...
    findAdjacentTriangles(triangleObjects)
    
    return triangleObjects


def findAdjacentTriangles(triangleObjects):
    '''
    Highly optimised, very good! :)
        
    Tests every STL triangle against every other STL triangle. 
    Tests every combination of pairs of points against eachother 
        from the pair of triangle
    Establishes whether the two triangles share a side
    The combinatorics method works as you are matching the x,y,z arrays of points
    
    There may be ways to optimise this process, but the function works in 
    its current form
    '''
    i = 0
    n = len(triangleObjects)
#    print('preparing STL Data')
    for t in triangleObjects:
#        print(round(i/n*100, 1), ' % complete')
        for ot in triangleObjects:
            #check all the data isn't complete for triangle
            if (t.adj_12=='')  or (t.adj_23=='') or (t.adj_31==''):
                #check it itsn't the same triangle
                if ot.ID != t.ID:
                    combinations_t = list(itertools.combinations([t.P1, t.P2, t.P3], 2))
                    combinations_ot = list(itertools.combinations([ot.P1, ot.P2, ot.P3], 2))
                    combinations_Points = list(itertools.product(combinations_t, combinations_ot))
                    for cbm in combinations_Points:
                        #test combinations of points being equal P1,1 == P2,1 & P1,2 == P2,2 etc
                        test11 = np.array_equal(cbm[0][0], cbm[1][0])
                        test12 = np.array_equal(cbm[0][0], cbm[1][1])   
                        test21 = np.array_equal(cbm[0][1], cbm[1][0])
                        test22 = np.array_equal(cbm[0][1], cbm[1][1])
                        if (test11 and test22) or (test12 and test21):
                            #shares two points -> WOW
                            #now need to assigned which two points it was!
                            test12 = np.array_equal(cbm[0][0], t.P1) and np.array_equal(cbm[0][1], t.P2)
                            test23 = np.array_equal(cbm[0][0], t.P2) and np.array_equal(cbm[0][1], t.P3)
                            test31 = np.array_equal(cbm[0][0], t.P1) and np.array_equal(cbm[0][1], t.P3)
                            if test12:
                                t.adj_12 = ot.ID
                            elif test23:
                                t.adj_23 = ot.ID
                            elif test31:
                                t.adj_31 = ot.ID
        i += 1 #used for a % indicator to user when printing
    return ' WOW '

def findFlowTurningAngles(triangleObjects):
    """
    Iterates through all objects
    
    Finds adjacent triangles, tests the flow turning angle by checking normal 
    vector angles
    
    
    """


if __name__ == "__main__":
    
    #adjacent triangle finding method Test
    triangles = processSTL("ModelTest.STL")
    print(triangles[0].normalVector)
    print(triangles[0].P1)
    print(triangles[0].P2)
    print(triangles[0].P3)
    print(triangles[0].area)
    print(triangles[0].centroid)
    #should technically reuse the adj results and assert that they are equal
    #this quick test showed me that it functioned as intended though
    print(triangles[0].adj_12)
    print(triangles[0].adj_23)
    print(triangles[0].adj_31)
    print(triangles[99].adj_12)
    print(triangles[99].adj_23)
    print(triangles[99].adj_31)
    print(triangles[54].adj_12)
    print(triangles[54].adj_23)
    print(triangles[54].adj_31)
    print(triangles[3].adj_12)
    print(triangles[3].adj_23)
    print(triangles[3].adj_31)
    
    
    
    #How to test the velocity incoming??
    #give a velocity in the plane of the 0th triangle object and see if it 
    #gets registered in one of the adjacent triangles
    triangles[0].velocity = np.array()
    A = triangles[99].adjacentCells()
    
    
    
    
    
    
    
    
    
    
  #        #convert to the plane of the triangle by subtracting all
#        #parts normal to the plane
#        V = v - np.dot(v, self.normalector)/(self.normalVector**2)*
#                self.normalVector    
        #now the dot product between v.B and v.A  and A.B 
        #will all lie in the same plane  
    
    
    
    
    
    
    
    
    
    
    
    #            #check all the data isn't complete for triangle
##            if ot.adj_12  or (ot.adj_23==None) or (ot.adj_31==None):
#                #check it itsn't the same triangle
#                if ot.ID != t.ID:
#                    i1 = 0
#                    points = [t.P1, t.P2, t.P3]
#                    for p1 in points:
#                        i2 = 0
#                        for p2 in points:
#                            if np.array_equal(p1,p2) == False:
#                                j1 = 0
#                                for op1 in [ot.P1, ot.P2, ot.P3]: 
#                                    j2 = 0
#                                    for op2 in [ot.P1, ot.P2, ot.P3]: 
#                                        if np.array_equal(op1, op2) == False:
#                                            if np.array_equal(p1, op1) and np.array_equal(p2, op2):
#                                                if ( np.array_equal(p1, t.P1) and np.array_equal(p2, t.P2) ) or ( np.array_equal(p1, t.P1) and np.array_equal(p2, t.P2) ):
#                                                    t.adj_12 = ot.ID
#                                                elif ( np.array_equal(p1, t.P2) and np.array_equal(p2, t.P3) or ( (p1 == t.P3) and (p2 == t.P2) ):
#                                                    t.adj_23 = ot.ID
#                                                elif ( (p1 == t.P3) and (p2 == t.P1) ) or ( (p1 == t.P3) and (p2 == t.P1) ):
#                                                    t.adj_31 = ot.ID
#                                                
#                                                if ( (op1 == ot.P1) and (op2 == ot.P2) ) or ( (op1 == ot.P1) and (op2 == ot.P2) ):
#                                                    ot.adj_12 = t.ID
#                                                elif ( (op1 == ot.P2) and (op2 == ot.P3) ) or ( (op1 == ot.P3) and (op2 == ot.P2) ):
#                                                    ot.adj_23 = t.ID
#                                                elif ( (op1 == ot.P3) and (op2 == ot.P1) ) or ( (op1 == ot.P3) and (op2 == ot.P1) ):
#                                                    ot.adj_31 = t.ID
#                                        j2 += 1
#                                    j1 += 1
#                            i2 += 1
#                        i1 += 1
    
    
#    for index, row in triangles.iterrows():
        
        
#    # Plot of Structrue etc
#    fig = plt.figure()
#    ax = fig.gca(projection="3d")
#    for i in len()
#    #Verticies
#    x1 = triangles[1][0]
#    x2 = triangle[2][0]
#    x3 = triangle[3][0]
#    y1 = triangle[1][1]
#    y2 = triangle[2][1]
#    y3 = triangle[3][1]
#    z1 = triangle[1][2]
#    z2 = triangle[2][2]
#    z3 = triangle[3][2]
#    ax.plot( [x1,x2], [y1,y2], [z1,z2], color = 'k' )
#    ax.plot( [x2,x3], [y2,y3], [z2,z3], color = 'k')
#    ax.plot( [x3,x1], [y3,y1], [z3,z1], color = 'k')



#    dataArray = []
#    
#    for i in range(0, len(triangles)):
#        normalVector = np.array( triangles[i][0] )
#        P1 = np.array( triangles[i][1] )
#        P2 = np.array( triangles[i][2] )
#        P3 = np.array( triangles[i][3] )
#        area = areaTriangle(P1, P2, P3)
#        centroid = centroidTriangle(P1, P2, P3)
#        dataArray.append( [i, normalVector, P1, P2, P3, area, centroid, 
#                           np.empty((3,0)) ] )
        
#    for i in range(0, len(triangles)):
#        normalVector_x = np.array( triangles[i][0] )
#        normalVector_y
#        normalVector_z
#        
#        P1_x =
#        P1_y =
#        P1_z =
#        
#        P2_x =
#        P2_y =
#        P2_z =
#        
#        P3_x =
#        P3_y =
#        P3_z =
#        
#        P1 = np.array( triangles[i][1] )
#        P2 = np.array( triangles[i][2] )
#        P3 = np.array( triangles[i][3] )
#        area = areaTriangle(P1, P2, P3)
#        centroid = centroidTriangle(P1, P2, P3)
#        dataArray.append( [i, normalVector, P1, P2, P3, area, centroid, 
#                           np.empty((3,0)) ] )
#        
#    print(dataArray)


#    return stlData      
#    
#def areaTriangle(P1, P2, P3):
#    v12 = np.subtract(P2, P1)
#    v13 = np.subtract(P3, P1)
#    return np.cross(v12, v13)/2.0
#
#def centroidTriangle(P1, P2, P3):
#    return np.mean( [P1, P2, P3], axis = 0)
#
#def findRelatedTriangles(triangleID):
#    '''
#    attempts to find all triangles that the triangle shares sides with
#    '''
#    ID1 = 0
#    ID2 = 0
#    ID3 = 0
#    return ID1, ID2, ID3  
    
    
    