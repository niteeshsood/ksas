import os
from math import cos, sin, pi
def rotate_vector(theta, vector):

#Rotation by anti-clocwise rotation matrix

    R = [ [cos(theta), -sin(theta)], [sin(theta), cos(theta)]]
    new_vector = list(); new_vector.append(0); new_vector.append(0);

    for i in xrange(2):
        for j in xrange(2):
            new_vector[i]+=R[i][j]*vector[j]


    return new_vector

def ConvertToRadians(angle):
    return angle*pi/180

def ConvertToAngles(angle):
    return angle*180/pi
