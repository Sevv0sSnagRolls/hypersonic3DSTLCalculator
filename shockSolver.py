# -*- coding: utf-8 -*-
"""
Created on Sat Jun 20 14:28:36 2020

@author: Doug
"""


import scipy.optimize
import numpy as np
from DiamondWingProfileGenerator import wingProfileGenerator
import matplotlib.pyplot as plt

def weakShock(beta, theta, M1, gamma):
    A = (M1**2)*((np.sin(beta))**2) - 1
    B = (M1**2)*( gamma + np.cos(2*beta) )  +  2
    return 2.0/(np.tan(beta))*A/B - np.tan(theta)

def weakShockDownstreamValues(beta, theta, M1, gamma, P1):
    A = 1 + ( (M1**2)*( (np.sin(beta))**2 )*((gamma - 1.0)/2.0) )
    B = gamma*(M1**2)*( (np.sin(beta))**2 ) - ((gamma - 1.0)/2.0)
    M2 = 1/(np.sin(beta - theta))*(A/B)**0.5
    
    C = 2*gamma*( (M1**2)*( (np.sin(beta))**2 ) - 1 )/(gamma + 1)
    P2 = P1*(1 + C)
    
    return M2, P2
    
def vPrandtlMeyer(M, gamma):
    A = (gamma + 1)/(gamma - 1)
    B = (M**2) - 1
    return (A**0.5)*np.arctan( ( (1/A)*B )**0.5 ) - np.arctan( B**0.5 )

def prandtlMeyer(M2, theta, M1, gamma):
    return vPrandtlMeyer(M2, gamma) - vPrandtlMeyer(M1, gamma) - theta

def expansionFanDownstreamPressure(M1, M2, P1, gamma):
    A = 1 + (M1**2)*(gamma - 1)/2
    B = 1 + (M2**2)*(gamma - 1)/2
    C = (gamma)/(gamma - 1)
    return P1*( (A/B)**C )

def flowCalculator(theta, thetaDir, topOrBot, M1, gamma, P1, thetaMin=0.5*np.pi/180, thetaMax=30*np.pi/180):
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


def directionVector(v1, v2):
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
        
def objectForcesCalculator(Vinf, Pinf, gamma, topPanels, botPanels, topNormals, botNormals, span):    
    VinfUnitVec = Vinf/np.linalg.norm(Vinf)
    MTop = [Minf, 0, 0, 0]
    pTop = [Pinf, 0, 0, 0]
    forcesTop = np.array([0.0,0.0,0.0], dtype=float)  
    topPanelsUnitVectors =[]
    for i in range(0, len(topPanels)):
        topPanelsUnitVectors.append(topPanels[i]/np.linalg.norm(topPanels[i]))
        if i == 0:
            theta = np.arccos( np.dot(VinfUnitVec, topPanelsUnitVectors[i]) )*180/np.pi
            thetaDir = directionVector(VinfUnitVec, topPanelsUnitVectors[i])
            MTop[i+1], pTop[i+1] = flowCalculator(theta, thetaDir, 'Top', MTop[i], gamma, pTop[i])
        else:
            theta = np.arccos( np.dot(topPanelsUnitVectors[i-1], topPanelsUnitVectors[i]) )*180/np.pi
            thetaDir = directionVector(topPanelsUnitVectors[i-1], topPanelsUnitVectors[i])
            MTop[i+1], pTop[i+1] = flowCalculator(theta, thetaDir, 'Top', MTop[i], gamma, pTop[i])
        
        length = np.linalg.norm(topPanels[i])
        area = length*span
        forcesTop[0] += pTop[i]*area*topNormals[i][0] 
        forcesTop[1] += pTop[i]*area*topNormals[i][1] 
        print('i: ', i, ' theta: ', theta, MTop[i+1], pTop[i+1] )
    
    MBot = [Minf, 0, 0, 0]
    pBot = [Pinf, 0, 0, 0]
    forcesBot = np.array([0.0,0.0,0.0], dtype=float)  
    botPanelsUnitVectors =[]
    for i in range(0, len(botPanels)):
        botPanelsUnitVectors.append(botPanels[i]/np.linalg.norm(botPanels[i]))
        if i == 0:
            theta = np.arccos( np.dot(VinfUnitVec, botPanelsUnitVectors[i]) )*180/np.pi
            thetaDir = directionVector(VinfUnitVec, botPanelsUnitVectors[i])
            MBot[i+1], pBot[i+1] = flowCalculator(theta, thetaDir, 'Bot', Minf, gamma, Pinf)
        else:
            theta = np.arccos( np.dot(botPanelsUnitVectors[i-1], botPanelsUnitVectors[i]) )*180/np.pi
            thetaDir = directionVector(botPanelsUnitVectors[i-1], botPanelsUnitVectors[i])
            MBot[i+1], pBot[i+1] = flowCalculator(theta, thetaDir, 'Bot', MBot[i], gamma, pBot[i])
        
        length = np.linalg.norm(topPanels[i])
        area = length*span
        forcesBot[0] += pBot[i]*area*botNormals[i][0] 
        forcesBot[1] += pBot[i]*area*botNormals[i][1] 
        print('i: ', i, ' theta: ', theta, MBot[i+1], pBot[i+1] )
        
    return forcesTop, forcesBot


def altitudeModel(altitude):
    if altitude > 25000:
        "i have no clue if stupid thing even works here"        
        T = -131.21 + 0.00299*altitude
        P = 2.488*( (T +273.1)/216.6 )**-11.388
    elif altitude < 25000 and altitude > 11000:
        T = -56.46
        P = 22.65*np.exp(1.73 - 0.000157*altitude)    
    else:
        T = 15.05 - 0.00648*altitude
        P = 101.29*( (T+273.1)/288.08 )**5.256
    return T, P


def solveDeflectionRange(elevonMinDeflection, elevonMaxDeflection, stepSize):
    Cd = []
    Cl = []
    deflections = range(elevonMinDeflection, elevonMaxDeflection, stepSize)
    for e in deflections:
        topPanels, bottomPanels, topNormals, bottomNormals = wingProfileGenerator(c, kt, ktx, ktTopT, kCe, e, True, z=0)       
        forcesTop, forcesBot = objectForcesCalculator(Vinf, Pinf, gamma, topPanels, bottomPanels, topNormals, bottomNormals, elevonSpan)
        Fx = forcesTop[0] + forcesBot[0]
        Fy = forcesTop[1] + forcesBot[1]
        Cd.append(Fx)
        Cl.append(Fy)
    fig2, (ax2, ax3) = plt.subplots(nrows=2, ncols=1)
    ax2.plot(deflections, Cl)
    ax2.set_title('Lift')
    ax2.set_xlabel('Elevon Defelction Deg')
    ax2.set_ylabel('Lift [N]')
    ax3.plot(deflections, Cd)
    ax3.set_title('Drag')
    ax3.set_xlabel('Elevon Defelction Deg')
    ax3.set_ylabel('Drag [N]')
    
    return Cl, Cd

if __name__ == "__main__":
    c = 9.2
    kt = 4 #4%
    ktx = 50 #50%
    ktTopT = 50 #50%
    kCe = 15 #15%
    elevonDeflection = 15 #degrees 
    elevonSpan = 1.2 
    gamma = 1.4
    Minf = 8.0
    gammainf = 1.4
    altitude = 20000 #m
    AOA = 2.0 #deg
    SS = 0 #deg -> Not sure about sideslip
    Tinf, Pinf = altitudeModel(altitude)
    ainf = (1.4*286* (Tinf+273.15) )**0.5
    VinfMag = Minf*ainf
    Vinf = VinfMag*np.array([np.cos(AOA*np.pi/180), np.sin(AOA*np.pi/180), 0.0])
    
    plt.cla()
    
    Cl, Cd = solveDeflectionRange(-2, 15, 1)
    
#    topPanels, bottomPanels, topNormals, bottomNormals = wingProfileGenerator(c, kt, ktx, ktTopT, kCe, elevonDeflection, True, z=0)       
#    forcesTop, forcesBot = objectForcesCalculator(Vinf, Pinf, gamma, topPanels, bottomPanels, topNormals, bottomNormals, elevonSpan)
#    
#    Fx = forcesTop[0] + forcesBot[0]
#    Fy = forcesTop[1] + forcesBot[1]
#    
#    print(Fx, Fy)





















#
#
#def wingProfileGenerator(chord, percentageChordMaxThickness, percentageChordMaxThicknessLocation, percentageTopT, percentageChordElevon, elevonDeflection, z=0):
#    """
#    when there is an offset - the length along primary axis is < chord length, use angle to account for this
#        theta = sin^-1(offset/chord)
#        
#    Create a diamond style wing structure of six segments based on inputs
#    
#                     B(chord*chordMaxT, maxThickness/2*percentageTopT/100)
#             ------------------------
#         - -----------------------------------  C(chord(1 - percentageChordElevon/100)*cos(theta), )
#    A----------------------------------------------------------------------------------------------D'(chord, profileEndOffset)
#       -----------------------------------------E(chord(1 - percentageChordElevon/100), )--------
#                -----F(chord*chordMaxT, maxThickness/2 * (1 - percentageTopT/100) )
#
#
#    """
##    if percentageTopT > 99 or percentageTopT < 1:
##        return "shakes head"
##    elif percentageChordElevon > 99:
##        return "shakes head"
#    
##    if profileEndOffset > 0:
##        theta = np.arcsin(profileEndOffset/chord)
##    else:
##        theta = 0
#    
#    #initiate all the points that describe the profile
#    A = np.array([0, 0, z], dtype = float)
#    B = np.array([0, 0, z], dtype = float)
#    C = np.array([0, 0, z], dtype = float)
#    D = np.array([0, 0, z], dtype = float)
#    E = np.array([0, 0, z], dtype = float)
#    F = np.array([0, 0, z], dtype = float)
#    
#    #max thickness points
#    t = chord*percentageChordMaxThickness/100
#    elevonLength = chord*percentageChordElevon/100
#    
#    xBF = chord*percentageChordMaxThicknessLocation/100
#    yB = t*percentageTopT/100.0
#    yF = -1*t*(1 - percentageTopT/100.0)
#    B[0] = xBF
#    F[0] = xBF
#    B[1] = yB
#    F[1] = yF
#    
#    #elevon attachment points
#    xCE = chord*(1 - percentageChordElevon/100)
#    yC = (0.0 - B[1])/(chord - xBF)*(xCE - xBF) + B[1]
#    yE = (0.0 - F[1])/(chord - xBF)*(xCE - xBF) + F[1]
#    C[0] = xCE
#    E[0] = xCE
#    C[1] = yC
#    E[1] = yE
#    
#    #end point, based of deflection from xCE - i.e ;ength of elevon rotated aboput that point
#    xD = xCE + elevonLength*np.cos(elevonDeflection*np.pi/180)
#    
#    #positive deflection down
#    yD = -1*elevonLength*np.sin(elevonDeflection*np.pi/180)
#    
#    D[0] = xD
#    D[1] = yD
#    
#    #assemble the points into Panels (series of points)
#    panelsTop = [ [A, B], [B, C], [C,D] ]
#    panelsBot = [ [A, F], [F, E], [E,D] ]
#    
#    for panel in panelsTop:
#        x1 = panel[0][0]
#        x2 = panel[1][0]
#        y1 = panel[0][1]
#        y2 = panel[1][1]
#        plt.plot( [x1,x2], [y1,y2], color = 'k' )
#    
#    for panel in panelsBot:
#        x1 = panel[0][0]
#        x2 = panel[1][0]
#        y1 = panel[0][1]
#        y2 = panel[1][1]
#        plt.plot( [x1,x2], [y1,y2], color = 'k' )
#        
#    title = 'Wing Profile | Elevon Deflection = ' + str(elevonDeflection) + ' Deg'
#    plt.gca().set_aspect('equal')
#    plt.title(title)
#    plt.xlabel('Chord [m]')
#    plt.ylabel('Thickness [m]')
#    plt.ylim((2*yF, 2*yB))
#    plt.grid()
#    
#    return panelsTop, panelsBot







