# -*- coding: utf-8 -*-
"""
Created on Thu Jul  9 07:29:46 2020

@author: Doug
"""


def ASCIIorBinary(filename):
    '''
    decides which reader to use and passes the result of the reader back to the
    main stl data processing function
    '''
    assert type(filename) == str, "The output filename must be a string."
    assert filename[-4:].lower() == ".stl", "Check your filename: {}".format(filename)
    
    with open(filename) as f:
        if f.readline()[0:5] == 'solid':
            return readASCIISTL(filename)
        else:
            return readBinarySTL(filename)
    return -1

def readASCIISTL(filename):
    with open(filename) as f:
        file = f.readlines()
        triangles = []
        i = 1 #start from the second line and exit before the final line
        while i < len(file) - 2:
            normalVector = [float(word) for word in file[i].split()[2::]]
            vertex1 = [float(word) for word in file[i+2].split()[1::]]
            vertex2 = [float(word) for word in file[i+3].split()[1::]]
            vertex3 = [float(word) for word in file[i+4].split()[1::]]
            triangles.append( [normalVector, vertex1, vertex2, vertex3] )
            i += 7
    return triangles


def readBinarySTL():
    
    pass
















#if __name__ == "__main__":
#    triangles = readASCIIStl("Cube 12.STL.txt")
#    
#    # Plot of Structrue etc
#    fig = pyplot.figure()
#    ax = fig.gca(projection="3d")
#    for triangle in triangles:
#        
#        #Verticies
#        x1 = triangle[1][0]
#        x2 = triangle[2][0]
#        x3 = triangle[3][0]
#        y1 = triangle[1][1]
#        y2 = triangle[2][1]
#        y3 = triangle[3][1]
#        z1 = triangle[1][2]
#        z2 = triangle[2][2]
#        z3 = triangle[3][2]
#        ax.plot( [x1,x2], [y1,y2], [z1,z2], color = 'k' )
#        ax.plot( [x2,x3], [y2,y3], [z2,z3], color = 'k')
#        ax.plot( [x3,x1], [y3,y1], [z3,z1], color = 'k')
#    
#        #centroids
#        xc = (x1+x2+x3)/3
#        yc = (y1+y2+y3)/3
#        zc = (z1+z2+z3)/3
#        ax.scatter3D(xc,yc,zc, color = 'r')
#        
#        #areas
#        v1 = [x2-x1, y2-y1, z2-z1]
#        v2 = [x3-x1, y3-y1, z3-z1]
#        a = np.cross(v1,v2)
#        
#        #pressureCalc - Newtonian Theory