# -*- coding: utf-8 -*-
"""
Created on Mon Jul  6 09:48:26 2020

@author: Doug
"""





def directionVector(v1, v2):
    '''
    Take two normal vectors from the upstream cell and the current cell
    
    Assumes the vectors are inward normal vectors (as used by the force 
    calculator)
    
    if the vectors both have positive x direction, the flow is compressed,
    
    if vectors both have negative x direction
    '''
    
    if v1[0]*v2[0] > 0:
        #both +ve x
        if (v2[1] - v1[1]) < 0.0:
            #negtive change so CW
            return 'CW'
        elif (v2[1] - v1[1]) > 0.0:
            #positive is CCW
            return 'CCW'
    elif v1[0]*v2[0] < 0:
        #negative so > 90 deg
        return 'bugger it'
    
    
    
    
    
    
if __name__ == "__main__":
    #some test conditions, right to test conditions
    
    