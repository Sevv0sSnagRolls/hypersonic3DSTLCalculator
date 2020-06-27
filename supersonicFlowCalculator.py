# -*- coding: utf-8 -*-
"""
Created on Sat Jun 27 20:54:16 2020

@author: Doug
"""

import numpy as np
import scipy.optimize

def weakShock(beta, theta, M1, gamma=1.4):
    '''
    Theta Beta M Equation
    
    :param theta [RADIANS]: 
        Input the flow turning angle theta
        if theta is > max weak shock angle? need to contrain this function
        
    :param M1: 
        Input Mach number upstream of cell that needs a calulation
        
    :param gamma:
    '''
    if abs(theta) > np.pi/4:
        #arbitary cut off but 45 degrees is probably too much to apply 
        #this relation
        return -1
    A = (M1**2)*((np.sin(beta))**2) - 1
    B = (M1**2)*( gamma + np.cos(2*beta) )  +  2
    return 2.0/(np.tan(beta))*A/B - np.tan(theta)

def weakShockDownstreamValues(beta, theta, M1, gamma, P1):
    '''
    Computes Downstream M and P values based of beta from Weak Shock()
    '''
    A = 1 + ( (M1**2)*( (np.sin(beta))**2 )*((gamma - 1.0)/2.0) )
    B = gamma*(M1**2)*( (np.sin(beta))**2 ) - ((gamma - 1.0)/2.0)
    M2 = 1/(np.sin(beta - theta))*(A/B)**0.5
    C = 2*gamma*( (M1**2)*( (np.sin(beta))**2 ) - 1 )/(gamma + 1)
    P2 = P1*(1 + C)
    return M2, P2
    
def vPrandtlMeyer(M, gamma):
    '''
    Prandlty Meyer v(M) expression
    '''
    A = (gamma + 1)/(gamma - 1)
    B = (M**2) - 1
    return (A**0.5)*np.arctan( ( (1/A)*B )**0.5 ) - np.arctan( B**0.5 )

def prandtlMeyer(M2, theta, M1, gamma):
    '''
    Root finder format of Prandlt Meyer Expansion solving for downstream 
    mach number
    '''
    return vPrandtlMeyer(M2, gamma) - vPrandtlMeyer(M1, gamma) - theta

def expansionFanDownstreamPressure(M1, M2, P1, gamma):
    A = 1 + (M1**2)*(gamma - 1)/2
    B = 1 + (M2**2)*(gamma - 1)/2
    C = (gamma)/(gamma - 1)
    return P1*( (A/B)**C )

def flowShockCalculator(theta, thetaDir, topOrBot, M1, gamma, P1, 
                        thetaMin=0.5*np.pi/180, thetaMax=45*np.pi/180):
    '''
    Intended to be a backbone of the full analytical solver.
    Takes a flow turning input from the previous cell to the current cell and 
    includes the direction of this turn. Direction tells flow function whether 
    flow is compressed or expanded between cells
    
    
    :param theta [DEG]: 
        Input the flow turning angle theta
    
    :param thetaDir [1,-1]: 
        Ideally will be converted to a 1, -1 expression which tells Calc
        whether its compressed or expanded from previous cell
        1 == Compressed
        -1 == Expanded
        
    :param M1: 
        Input Mach number upstream of cell that needs a calulation
    
    :param P1: 
        Input pressure upstream of cell that needs a calulation
        
    :param gamma:
    
    :param thetaMin [RADIANS]: 
        this is very important, smooths out bumps in stl file by including a 
        minimum angle before flow is recognised as being compressed
        
    :param thetaMax [RADIANS]: 
        constrains what the calculator can do, no support for > 45 degree 
        sections. solver assumes the craft is somewhat aerodynamic.
    
    '''
    
    #convert to Radians - Maybe this should occur somewhere else....
    theta *= np.pi/180
    
    if topOrBot == 'Top':
        if thetaDir == 'CCW':
            #then use weak shock stuff
            initialGuess = theta*2
            beta = scipy.optimize.newton(weakShock, initialGuess, args=(theta, M1, gamma))
            M2, P2 = weakShockDownstreamValues(beta, theta, M1, gamma, P1)
        elif thetaDir == 'CW':
            initialGuess = M1*0.5
            M2 = scipy.optimize.newton(prandtlMeyer, initialGuess, args=(theta, M1, gamma))
            P2 = expansionFanDownstreamPressure(M1, M2, P1, gamma)
        else:
            print('ERROR!')
    
    elif topOrBot == 'Bot':
        if thetaDir == 'CW':
            #then use weak shock stuff
            initialGuess = theta*2
            beta = scipy.optimize.newton(weakShock, initialGuess, args=(theta, M1, gamma))
            M2, P2 = weakShockDownstreamValues(beta, theta, M1, gamma, P1)
        elif thetaDir == 'CCW':
            initialGuess = M1*0.5
            M2 = scipy.optimize.newton(prandtlMeyer, initialGuess, args=(theta, M1, gamma))
            P2 = expansionFanDownstreamPressure(M1, M2, P1, gamma)
        else:
            print('ERROR!')
            
    return M2, P2





















 