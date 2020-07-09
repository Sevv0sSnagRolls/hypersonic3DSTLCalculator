# -*- coding: utf-8 -*-
"""
Created on Thu Jul  9 15:45:18 2020

@author: Doug
"""

import numpy as np

def solver(triangleObjects):
    '''
    similar to a search algorithm uses a frontier concet to explore to adjacent
    triangles found by the processSTL algorithms
    '''
    frontier = frontTriangles(triangleObjects)
    while(frontier):
        frontier.pop(0)
        
    

    















    def frontTriangles(triangleObjects):
    origin = np.array([0,0,0])
    originTriganles = []
    for obj in triangleObjects:
        if np.array_equal(obj.P1, origin) or np.array_equal(obj.P2, origin) 
            or np.array_equal(obj.P3, origin):
                originTriganles.append(obj)
    return originTriganles













