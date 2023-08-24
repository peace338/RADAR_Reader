import numpy as np
from radarEquipInfo import *

def getPhi(obj): #


    tmp = obj.z/obj.range

    if tmp > 1 :
        tmp  =1
    elif tmp <-1:
        tmp = -1

    phi = np.arcsin(tmp)

    return phi*180/np.pi

"""get Azimuth angle in Degree"""
def getTheta(obj): #
    tmp = obj.x / np.sqrt(obj.x * obj.x + obj.y * obj.y) 
    if tmp > 1 :
        tmp  =1
    elif tmp <-1:
        tmp = -1

    theta = np.arcsin(tmp)

    return theta*180/np.pi

def pruneTarget(objs):
    
    for obj in objs[:]:
        # print("gi")
        if (abs(getTheta(obj)) > HALF_AZIM_FOV) or (abs(getPhi(obj)) > HALF_ELEV_FOV) or obj.z < -EQUIP_HEIGHT or obj.z > MAX_HEIGHT:
            # breakpoint()
            # print("delete")
            objs.remove(obj)
    # print(len(objs))
    return objs
