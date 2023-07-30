from .radarPerception.egoMotionEstimation import egoMotionEst
from .radarPerception.clustering import Clustering
from .radarPerception.IMM import Tracking
from .radarConfigure import configManager_MRR_DEMO as cfgMan
from . import dataStructure as dT
from .mmwavelib import conversion, populateTrackingList

import numpy as np
import pdb

class RadarAlgorithm():
    def __init__(self):
        config = cfgMan.config()
        aoaCfg = cfgMan.aoaCfg
        self.clusterCfg = cfgMan.clusterCfgMRR
        self.trackingCfg = cfgMan.trackingCfg
        domain = dT.domainGenerator(config, aoaCfg)
        self.RadarInfo = dT.radar_info()
        self.resList = domain.getResolution()
        self.egoMotionEstimator = egoMotionEst()
        self.clusteringAlg = Clustering(self.clusterCfg, self.resList, domain, self.RadarInfo)
        self.trackingAlg = Tracking()
    
    def __call__(self, objList):
        # pdb.set_trace()
        objs3d = []
        newObjList = self.parsing(objList)
        for obj in objList:
            objs3d.append([obj.x, obj.y, obj.z, obj.range])
        objs3d = np.array(objs3d)
        flags, _,_,_,_ = self.egoMotionEstimator(objs3d)
        
        assert len(objs3d) ==  len(objList)

        for newObj, flag in zip(newObjList, flags):
            newObj.status = flag
        
        _, clusterOutputList =self.clusteringAlg(newObjList)

        measList = populateTrackingList(clusterOutputList)
        self.trackingAlg(measList)

        return self.trackingAlg.getTracker()

    def parsing(self, objList):
        cfarOut3DList = conversion(objList, self.RadarInfo, 0, 0, self.resList)
        return cfarOut3DList

    

    
    
    