# -*- coding: utf-8 -*-
"""
Created on Sat Jun 27 21:26:02 2020

@author: Doug
"""

import numpy as np

def altitudeModel(altitude):
    if altitude > 25000:
        T = -131.21 + 0.00299*altitude
        P = 2.488*( (T +273.1)/216.6 )**-11.388
    elif altitude < 25000 and altitude > 11000:
        T = -56.46
        P = 22.65*np.exp(1.73 - 0.000157*altitude)    
    else:
        T = 15.05 - 0.00648*altitude
        P = 101.29*( (T+273.1)/288.08 )**5.256
    R = 0.286
    rho = P/(R*(T + 273.1))
    P *= 1000 #kPa -> Pa
    return T, P, rho
