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
    """
    To fill out... should give the same format as the ASCII version
    """
    pass



#if __name__ == "__main__":
     """
     Code was tested with the Ward Smart Paper
     """






















