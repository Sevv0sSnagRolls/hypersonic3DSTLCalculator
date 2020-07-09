# -*- coding: utf-8 -*-
"""
Created on Thu Jul  9 07:44:39 2020

@author: Doug
"""

import numpy as np
import pandas as pd
import importSTL



class triangle():
    
    def __init__(self, ID, normalVector, P1, P2, P3, area, centroid):
        self.ID = ID
        self.normalVector = normalVector
        self.P1 = P1
        self.P2 = P2
        self.P3 = P3
        self.centroid = self.centroidTriangle(P1, P2, P3)
        self.area = self.areaTriangle(P1, P2, P3)

    def areaTriangle(self, P1, P2, P3):
        v12 = np.subtract(P2, P1)
        v13 = np.subtract(P3, P1)
        return np.cross(v12, v13)/2.0

    def centroidTriangle(self, P1, P2, P3):
        return np.mean( [P1, P2, P3], axis = 0)
    
    def isLeadingEdge(self):
        return
    
    def velocityTriangle(self):
        return
    
    def pressureTriangle(self):
        return

    def forceTriangle(self):
        return self.pressure*self.area
    
    def findRelatedTriangles(self):
        '''
        attempts to find all triangles that the triangle shares sides with
        '''
        ID1 = 0
        ID2 = 0
        ID3 = 0
        return ID1, ID2, ID3

def processSTL(filename):
    '''
    iterates through stl file data and generates triangle class objects
    
    
    '''
    
    #get the data from either Binary or ASCII STL file
    triangles = importSTL.ASCIIorBinary(filename)
    
    

    
    
    
if __name__ == "__main__":
    triangles = processSTL("ModelTest.STL")
    import matplotlib.pyplot as plt
    print(triangles)
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
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
    
    
    