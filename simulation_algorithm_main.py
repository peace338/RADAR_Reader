import os
import sys
# sys.path.append(os.path.abspath(os.path.dirname(__file__)))
import IMM
import mmwavelib as ml
import dataStructure as DT
import clusteringTmp as CL
import configManager_MRR_DEMO as cfgMan
import clusteringTmp
import CONST
import DOA
import logOut
import mrr_config_chirp_design_MRR160_TDM_MIMO
import plot_data
import signalGen
import trackMaker
import importlib
import prune_object
import copy
# import numpy as np

reload_flag = 0

def radar_algorithm_simulation(object_list, tracker, track_frame_number, can_data):
    global reload_flag
    if reload_flag == 0:
        reload_modules()
        reload_flag = 1
    config = cfgMan.config()
    aoaCfg = cfgMan.aoaCfg
#    trackingCfg = cfgMan.trackingCfg()

    # tracker = DT.genTracker()
    domain = DT.domainGenerator(config, aoaCfg)
    #radar_object = DT.radar_obj()
    #radar_track = DT.radar_trk()

    resList = domain.getResolution()  # unit : [meter, meter/sec, radian]

    timeStamp = 0

    if track_frame_number == 275:
        aa = 1

    RadarAngle = 45.0
    RadarInfo = DT.radar_info()
    RadarInfo.mode = can_data[4]		                              # 1 : BCW, 2 : RCCW, 3 : SEW
    RadarInfo.position = can_data[5]                                  # 1 : Left, 2 : Right
    vehicle_speed = can_data[6]/3.6                                   # [m/s]
    vehicle_steer_angle = (can_data[14] * 256 + can_data[15])/10      # Steer Angle(deg)
    if vehicle_steer_angle > 3276.7:
        vehicle_steer_angle -= 6553.5
    vehicle_steer_angle = vehicle_steer_angle * 38.0 / 650.0
 
    if RadarInfo.mode == 2:
        if RadarInfo.position == 1:
            RadarInfo.angle = 0             # RadarAngle
            RadarInfo.StaticRemoveAngle = RadarAngle
        elif RadarInfo.position == 2:
            RadarInfo.angle = 0             # -RadarAngle
            RadarInfo.StaticRemoveAngle = -RadarAngle
        else:
            RadarInfo.angle = 0             # RadarAngle
            RadarInfo.StaticRemoveAngle = RadarAngle
    else:
        if RadarInfo.position == 1:
            RadarInfo.angle = -RadarAngle
            RadarInfo.StaticRemoveAngle = -RadarAngle
        elif RadarInfo.position == 2:
            RadarInfo.angle = RadarAngle
            RadarInfo.StaticRemoveAngle = RadarAngle
        else:
            RadarInfo.angle = RadarAngle
            RadarInfo.StaticRemoveAngle = RadarAngle

    # ============================= Clustering =====================================
#    print(track_frame_number)
    Radar_raw_measurement = ml.conversion(object_list, RadarInfo, vehicle_speed, vehicle_steer_angle)  # conversion data type to suit to this simulation code
    # Pruning : Do Something(input : Radar_raw_measurement, output :cfarOut3DList)
    # Radar_raw_measurement = prune_object.remove_far_obj(Radar_raw_measurement)
    # Radar_raw_measurement = prune_object.remove_recede_obj(Radar_raw_measurement)
    # Radar_raw_measurement = prune_object.remove_wall_obj(Radar_raw_measurement)
    
    # print(len(Radar_raw_measurement))
    cfarOut3DList = Radar_raw_measurement
    ##############################################################################

    ThresholdY = 40.0
    if RadarInfo.mode == 1:
        clusterCfg = cfgMan.clusterCfgMRR
        trackingCfg = cfgMan.trackingCfg
    elif RadarInfo.mode == 3:
        clusterCfg = cfgMan.clusterCfgSEW
        trackingCfg = cfgMan.trackingCfgSEW
    elif RadarInfo.mode == 2:
        clusterCfg = cfgMan.clusterCfgRCCW
        trackingCfg = cfgMan.trackingCfgRCCW
    else:
        clusterCfg = cfgMan.clusterCfgMRR
        trackingCfg = cfgMan.trackingCfg

    cfarOut3DListGuardRail = copy.deepcopy(cfarOut3DList)
    clusterList, clusterOutputList = CL.clusteringDBscanRun(cfarOut3DList, clusterCfg, resList, domain, RadarInfo)

    if RadarInfo.mode == 1:
        clusterListGuardRail, clusterOutputListGuardRail = CL.clusteringDBscanRun(cfarOut3DListGuardRail, cfgMan.clusterCfgGuardRail, resList, domain, RadarInfo)
        ThresholdY = ml.estimateGuardRail(clusterListGuardRail, RadarInfo)

    measList = ml.populateTrackingList(clusterOutputList, ThresholdY, RadarInfo)

    # ============================= Tracking =====================================
    IMM.imm_ekfRun(measList, tracker, trackingCfg, track_frame_number, RadarInfo)
    # ============================================================================

    return cfarOut3DList
    
    # if len(tracker.List) > 0 :
    #     print(track_frame_number)
    #     print("ID : " +str(tracker.List[0].ID))
    #     print("SNR : " +str(tracker.List[0].SNR))
    #     print("age : " +str(tracker.List[0].age))
    #     try : 
    #         print("associatedObj :" + str(tracker.List[0].associatedObj.ID))
    #     except : 
    #         print("nothing")
    #     print("dopplerSNR : " +str(tracker.List[0].dopplerSNR))
    #     print("measVectorRRD[0].range : "+str(tracker.List[0].measVectorRRD[0]))
    #     print("measVectorRRD[1].speed : "+str(tracker.List[0].measVectorRRD[1]))
    #     print("measVectorRRD[2].theta : "+str(tracker.List[0].measVectorRRD[2]))
    #     print("peakVal : " +str(tracker.List[0].peakVal))
    #     print("prevXd : " +str(tracker.List[0].prevXd))
    #     print("prevYd : " +str(tracker.List[0].prevYd))
    #     print("rangeSNR : " +str(tracker.List[0].rangeSNR))
    #     print("tick : " +str(tracker.List[0].tick))
    #     print("xSize : " +str(tracker.List[0].xSize))
    #     print("ySize : " +str(tracker.List[0].ySize))
    # else :
    #     print("nothing")

def reload_modules() :
    importlib.reload(IMM)
    importlib.reload(ml)
    importlib.reload(DT)
    importlib.reload(CL)
    importlib.reload(cfgMan)
    importlib.reload(clusteringTmp)
    importlib.reload(CONST)
    importlib.reload(DOA)
    importlib.reload(logOut)
    importlib.reload(mrr_config_chirp_design_MRR160_TDM_MIMO)
    importlib.reload(plot_data)
    importlib.reload(signalGen)
    importlib.reload(trackMaker)
    importlib.reload(importlib)
    importlib.reload(prune_object)

