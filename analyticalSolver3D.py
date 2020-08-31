# -*- coding: utf-8 -*-
"""
Created on Thu Jul  9 15:45:18 2020

@author: Doug
"""

import numpy as np
 
def solverControl(triangleObjects):
    #run the solver - calculate properties in each cell
    success = solver(triangleObjects)
    if success != True:
        return -1

    #now calculate the total properties from list of object names
    for i in range(len(triangleObjects)): 
        forces[i] = triangleObjects[i].force()

    force = np.sumElementWise(forces)
    return force


def solver(triangleObjects):
    '''
    similar to a search algorithm uses a frontier concet to explore to adjacent
    triangles found by the processSTL algorithms
    '''
    frontier = frontTriangles(triangleObjects)
    explored = []
    while(frontier):
        currentCell = frontier.pop(0)
        
        for adjCell in currentCell.adjCells():
            if adjCell.velocity:
                currentCell.v_in
        
        #check is cell has a leading Edge etc - needs to calculate if node is 
        if currentCell.isLeadingEdge():
            if currentCell.isInUpstreamShockWakes(shockCells):
                #use post shock properties as Vinf, double or triple processed in order...
                
            else:
                #use Vinf as the input
                
        elif currentCell.isStagnationPoint():
            
                
        #calculate turning angle of each velocity vector in.
        for velocity in currentCell.velocitiesIn():
            angle = 
            direction = compression or expansion?
            if angle >= angleMin:
                shockCalculator()
            else:
                linearInterpolateInputs()
                
        #Finish calculating the other proeprties of the cell etc
        currentCell.calculateProperties()
            
        #find adjacent triangle and add it to frontier
        for obj in currentCell.adjTriangles():
            
    return True #no errors


def frontTriangles(triangleObjects):
    origin = np.array([0.0,0.0,0.0])
    originTriganles = []
    for obj in triangleObjects:
        case1 = np.array_equal(obj.P1, origin) 
        case2 = np.array_equal(obj.P2, origin)
        case3 = np.array_equal(obj.P3, origin) 
        if case1 | case2 | case3:
                originTriganles.append(obj)
    return originTriganles













