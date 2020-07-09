# -*- coding: utf-8 -*-
"""
Created on Thu Jun 25 10:50:53 2020

@author: Doug
"""

import numpy as np
import matplotlib.pyplot as plt

def wingProfileGenerator(chord, percentageChordMaxThickness, percentageChordMaxThicknessLocation, percentageTopT, percentageChordElevon, elevonDeflection, plot=True, z=0):
    """
    when there is an offset - the length along primary axis is < chord length, use angle to account for this
        theta = sin^-1(offset/chord)
        
    Create a diamond style wing structure of six segments based on inputs
    
                     B(chord*chordMaxT, maxThickness/2*percentageTopT/100)
             ------------------------
         - -----------------------------------  C(chord(1 - percentageChordElevon/100)*cos(theta), )
    A----------------------------------------------------------------------------------------------D'(chord, profileEndOffset)
       -----------------------------------------E(chord(1 - percentageChordElevon/100), )--------
                -----F(chord*chordMaxT, maxThickness/2 * (1 - percentageTopT/100) )


    """
#    if percentageTopT > 99 or percentageTopT < 1:
#        return "shakes head"
#    elif percentageChordElevon > 99:
#        return "shakes head"
    
#    if profileEndOffset > 0:
#        theta = np.arcsin(profileEndOffset/chord)
#    else:
#        theta = 0
    
    #initiate all the points that describe the profile
    A = np.array([0, 0, z], dtype = float)
    B = np.array([0, 0, z], dtype = float)
    C = np.array([0, 0, z], dtype = float)
    D = np.array([0, 0, z], dtype = float)
    E = np.array([0, 0, z], dtype = float)
    F = np.array([0, 0, z], dtype = float)
    
    #max thickness points
    t = chord*percentageChordMaxThickness/100
    elevonLength = chord*percentageChordElevon/100
    
    xBF = chord*percentageChordMaxThicknessLocation/100
    yB = t*percentageTopT/100.0
    yF = -1*t*(1 - percentageTopT/100.0)
    B[0] = xBF
    F[0] = xBF
    B[1] = yB
    F[1] = yF
    
    #elevon attachment points
    xCE = chord*(1 - percentageChordElevon/100)
    yC = (0.0 - B[1])/(chord - xBF)*(xCE - xBF) + B[1]
    yE = (0.0 - F[1])/(chord - xBF)*(xCE - xBF) + F[1]
    C[0] = xCE
    E[0] = xCE
    C[1] = yC
    E[1] = yE
    
    #end point, based of deflection from xCE - i.e ;ength of elevon rotated aboput that point
    xD = xCE + elevonLength*np.cos(elevonDeflection*np.pi/180)
    
    #positive deflection down
    yD = -1*elevonLength*np.sin(elevonDeflection*np.pi/180)
    
    D[0] = xD
    D[1] = yD
    
    #assemble the points into Panels (series of points) - which can be resolved into vectors
    panelsTop = [ [A, B], [B, C], [C,D] ]
    panelsBot = [ [A, F], [F, E], [E,D] ]
    
    if plot == True:       
        plt.cla()
        for panel in panelsTop:
            x1 = panel[0][0]
            x2 = panel[1][0]
            y1 = panel[0][1]
            y2 = panel[1][1]
            plt.plot( [x1,x2], [y1,y2], color = 'k' )
        
        for panel in panelsBot:
            x1 = panel[0][0]
            x2 = panel[1][0]
            y1 = panel[0][1]
            y2 = panel[1][1]
            plt.plot( [x1,x2], [y1,y2], color = 'k' )
            
        plt.gca().set_aspect('equal')
        plt.title('Wing Profile | Elevon Deflection = ' + str(elevonDeflection) + ' Deg')
        plt.xlabel('Chord [m]')
        plt.ylabel('Thickness [m]')
        plt.ylim((2*yF, 2*yB))
        plt.grid(b=True)
        
    
    #return the vectors describing the panels
    panelsTop = [ B - A, C - B, D - C ]
    panelsBot = [ F - A, E - F, D - E ]
    
    #normals are rotated 90 degress in plane - 
    #we want the normals to be facing inwards for the pressure calculations
    #This means the rotation of the top should be CW and bot should be CCW
    normalsTop = [ twoDimensionalRotation(panel, -90.0, 'deg') for panel in panelsTop]
    normalsBot = [ twoDimensionalRotation(panel, 90.0, 'deg')  for panel in panelsBot]
    
    if plot == True:
        centre = np.array([0,0,0], dtype = float)
        centreTotal = np.array([0,0,0], dtype = float)
        for i in range(0, len(panelsTop) ):
            centre[0] = 0.5*panelsTop[i][0] + centreTotal[0]
            centre[1] = 0.5*panelsTop[i][1] + centreTotal[1]
            x1 = centre[0]
            x2 = x1 + normalsTop[i][0]
            y1 = centre[1]
            y2 = y1 + normalsTop[i][1]
            plt.plot( [x1,x2], [y1,y2], color = 'r' )
            centreTotal[0] += panelsTop[i][0]
            centreTotal[1] += panelsTop[i][1]
        
        centreTotal = np.array([0,0,0], dtype = float)
        for i in range(0, len(panelsTop) ):
            centre[0] = 0.5*panelsBot[i][0] + centreTotal[0]
            centre[1] = 0.5*panelsBot[i][1] + centreTotal[1]
            x1 = centre[0]
            x2 = x1 + normalsBot[i][0]
            y1 = centre[1]
            y2 = y1 + normalsBot[i][1]
            plt.plot( [x1,x2], [y1,y2], color = 'r' )
            centreTotal[0] += panelsBot[i][0]
            centreTotal[1] += panelsBot[i][1]
    
    return panelsTop, panelsBot, normalsTop, normalsBot


def twoDimensionalRotation(vector, theta, degOrRad = 'deg'):
    assert len(vector) == 3
    if degOrRad == 'deg':
        theta *= np.pi/180
    R = np.array([ [ np.cos(theta), -1*np.sin(theta) ],
                      [ np.sin(theta), np.sin(theta)] ])
    tempVector = vector.copy()[0:2]
    tempVector = np.dot(R, tempVector)
    returnVector = np.array([0.0,0.0,0.0], dtype = float)
    returnVector[0:2] = tempVector
    returnVector /= np.linalg.norm(returnVector)
    return returnVector
    


if __name__ == "__main__":
    #Chord Length [m]
    c = 9.2
    
    #Percentage of Chord Length the Maximum (Top to Bottom) Thickness of Wing is %
    kt = 4
    
    #Percentage of Chord Length the Maximum (Top to Bottom) Thickness Occurs Along Wing Profile
    ktx = 50
    
    #Percentage of thickness that resides in the top half of the wing, the 1-% is what is left for the underside
    ktTopT = 50
    
    #Percentage of Chord Length the elevon takes up
    kCe = 20 
    
    #Positive Deflection Down Elevon Deflection in Degrees
    elevonDeflection = 10
    
    if kCe > ktx:
        print('Error, cannot have the elevon going beyong wing midsection, unresolvable geometry')
    
    A, B, C, D = wingProfileGenerator(c, kt, ktx, ktTopT, kCe, elevonDeflection, True, z=0)    

















    
    