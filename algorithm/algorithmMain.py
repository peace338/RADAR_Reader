from .radarPerception.egoMotionEstimation import egoMotionEst_3D
from .radarPerception.clustering import Clustering
from .radarPerception.IMM import Tracking
from .radarConfigure import configManager_MRR_DEMO as cfgMan
from . import dataStructure as dT
from .mmwavelib import conversion, populateTrackingList
from .utils import *
import numpy as np


class RadarAlgorithm():
    def __init__(self):
        config = cfgMan.config()
        aoaCfg = cfgMan.aoaCfg
        self.clusterCfg = cfgMan.clusterCfgMRR
        self.trackingCfg = cfgMan.trackingCfg
        domain = dT.domainGenerator(config, aoaCfg)
        self.RadarInfo = dT.radar_info()
        self.resList = domain.getResolution()
        self.egoMotionEstimator = egoMotionEst_3D()
        self.clusteringAlg = Clustering(self.clusterCfg, self.resList, domain, self.RadarInfo)
        self.trackingAlg = Tracking()
        self.tracker = None
        
    def __call__(self, objList):

        objs3d = []
        filteredObjs = []
        # breakpoint()
        # print("h",len(objList))
        objList = pruneTarget(objList)
        # print("f",len(objList))
        newObjList = self.parsing(objList)
        for obj in objList:
            filteredObjs.append([getTheta(obj), obj.speed, getPhi(obj)])
            objs3d.append([obj.x, obj.y, obj.z, obj.range])
        objs3d = np.array(objs3d)
        filteredObjs = np.array(filteredObjs)
        # breakpoint()
        flags, _,_,_,_ = self.egoMotionEstimator(filteredObjs)
        
        assert len(objs3d) ==  len(objList)
        assert len(objs3d) ==  len(newObjList)

        for newObj, flag in zip(newObjList, flags):
            newObj.status = flag
        
        _, clusterOutputList =self.clusteringAlg(newObjList)

        measList = populateTrackingList(clusterOutputList)
        self.tracker = self.trackingAlg(measList)

        return self.trackingAlg.getTracker()

    def parsing(self, objList):
        cfarOut3DList = conversion(objList, self.RadarInfo, 0, 0, self.resList)
        return cfarOut3DList

    

    
    
    