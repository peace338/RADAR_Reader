import os
import math
import numpy as np
from abc import ABC, abstractmethod
from pathlib import Path
import glob

RDB_FORMAT = 'rdb'

NULL = 0

RADAR1 = 1
RADAR2 = 2
Q7_DIVISOR = 128
Q8_DIVISOR = 256
Q14_DIVISOR = 16384
IT_IS_NOT_SIMULATION = 0
IT_IS_SIMULATION = 1
CAR_WIDTH_M = 1.7
LEFT_ANGLE_DEG = 45.51
RIGHT_ANGLE_DEG = -44.93
# LEFT_ANGLE_DEG = 45
# RIGHT_ANGLE_DEG = -45
KBSD_BCW_MODE = 1
KBSD_RCCW_MODE = 2
KBSD_SEW_MODE = 3

RADAR_OBJECT_ATTRIBUTES_LIST = ["range_idx","doppler_idx","range","speed","sin_azim",\
                                "peak_val","range_snr_db","doppler_snr_db","sin_azim_srn_lin","x",\
                                "y","z","vel_disamb_fac_valid","vel_disamb_fac_valid_als","status_flag"]
RADAR_TRACK_ATTRIBUTES_LIST = ["id","x","y","xd","yd",\
                                "x_size","y_size","tick","age","flag",\
                                "reserved0"]
SIMULATED_OBJECT_ATTRIBUTES_LIST = ["detID","clusterId","rangeIdx","dopplerIdx","range",\
                                    "speed","sinAzim","rangeVal","rangeSNR","dopplerSNR",\
                                    "angleSNR","x","y","velDisambFacValid","status_flag",\
                                    "xd","yd","rot_x","rot_y","rot_dx","rot_dy"] 
SIMULATED_TRACK_ATTRIBUTES_LIST = ["ID","SNR","age","associ_ID","dopplerSNR",\
                                    "measVectorRRD_R","measVectorRRD_vel","measVectorRRD_Theta","statVecXYZ_x","statVecXYZ_y",\
                                    "statVecXYZ_xd","statVecXYZ_yd","peakVal","prevXd","prevYd",\
                                    "rangeSNR","tick","xSize","ySize","plotValidity",\
                                    "Status_Flag0","Status_Flag1","rot_x","rot_y","rot_dx","rot_dy"]



class radar_object() :
    def __init__(self):
        self.attributes_list = RADAR_OBJECT_ATTRIBUTES_LIST
        for ii in range(len(self.attributes_list)) :
            globals()['self.'+self.attributes_list[ii]] = 0

    
    def get_attributes_list() :
        attributes_list = RADAR_OBJECT_ATTRIBUTES_LIST
        return attributes_list

    def uint2int_16(input_data) :
        if input_data > 32767 :
            input_data = -(65535 - input_data + 1)
        return input_data
    
    def int2uint_16(input_data) :
        if input_data < 0 :
            input_data = 65535 + input_data + 1
        return input_data
        

class radar_track() :
    def __init__(self):
        self.attributes_list  =  RADAR_TRACK_ATTRIBUTES_LIST
        for ii in range(len(self.attributes_list)) :
            globals()['self.'+self.attributes_list[ii]] = 0

    def get_attributes_list() :
        attributes_list = RADAR_TRACK_ATTRIBUTES_LIST
        return attributes_list


class simulated_object() :
    def __init__(self):
        self.attributes_list  = SIMULATED_OBJECT_ATTRIBUTES_LIST
        for ii in range(len(self.attributes_list)) :
            globals()['self.'+self.attributes_list[ii]] = 0
    
    def get_attributes_list() :
        attributes_list = SIMULATED_OBJECT_ATTRIBUTES_LIST
        return attributes_list
    
class simulated_track() :
    def __init__(self):
        self.attributes_list  =  SIMULATED_TRACK_ATTRIBUTES_LIST
        for ii in range(len(self.attributes_list)) :
            globals()['self.'+self.attributes_list[ii]] = 0
    
    def get_attributes_list() :
        attributes_list = SIMULATED_TRACK_ATTRIBUTES_LIST
        return attributes_list

class RadarDataManager():
    def __init__(self, dataDir):

        self.radar1_frame_index = []
        self.radar2_frame_index = []
        self.radar_end_frame_num = 0
        self.radar1_end_frame_num = 0
        self.radar2_end_frame_num = 0
        self.video_end_frame_num = 0
        self.video_current_frame_num = 0
        self.radar_current_frame_num = 0
        self.radar_video_frame_offset = 0
        self.dataDir = dataDir
        self.radar1_checkbox = True
        self.radar2_checkbox = True
        self.radar_flip_checkbox = False

    def loadCurser(self):
        # pdb.set_trace()
        load_flag = 0
        if os.path.getsize(self.dataDir) == 0 :

            print("RADAR FILE ERROR","Radar file is unavailable. Check the file.") 
            load_error_flag_neg = 0
        else :
            self.radar_file_handle = open(self.dataDir,'rb')
            load_flag = 1               # indicator for meeting end of file.
            load_error_flag_neg = 1     # 0 means error
        load_error_counter = 0
        while (load_flag * load_error_flag_neg):
            while True : 
                try :
                    data1 = self.radar_file_handle.read(1)
                    
                except : 
                    load_flag = 0
                    break
                if data1 == self.magicID[0] :
                    data1 = self.radar_file_handle.read(1)
                    
                    if data1 == self.magicID[1] :
                        data1 = self.radar_file_handle.read(1)
                        if data1 == self.magicID[2] :
                            data1 = self.radar_file_handle.read(1)
                            if data1 == self.magicID[3] :
                                data1 = self.radar_file_handle.read(6)
                                if data1 == b'_RPAS1' :
                                    file_curser = self.radar_file_handle.tell()
                                    load_error_counter = 0
                                    self.radar1_frame_index.append(file_curser)
                                if data1 == b'_RPAS2' :
                                    file_curser = self.radar_file_handle.tell()
                                    load_error_counter = 0
                                    self.radar2_frame_index.append(file_curser)
                                break
                elif data1 == b'' :
                    load_flag = 0
                    break
                else : 
                    load_error_counter += 1
                    if load_error_counter > 2048*4 :
                        load_error_flag_neg = 0
                        raise ("SIMULATION FILE ERROR","Radar simulation file is unavailable. \nCheck the file.") 

        self.radar1_end_frame_num = len(self.radar1_frame_index)
        self.radar2_end_frame_num = len(self.radar2_frame_index)
        if self.radar1_end_frame_num == NULL :
            self.radar_end_frame_num = self.radar2_end_frame_num
        elif self.radar2_end_frame_num == NULL :
            self.radar_end_frame_num = self.radar1_end_frame_num
        else :
            self.radar_end_frame_num = min(self.radar1_end_frame_num, self.radar2_end_frame_num)
    
    def set_to_this_frame(self, num_frame) :
        self.radar_current_frame_num = num_frame
        self.video_current_frame_num = num_frame + self.radar_video_frame_offset
        self.frame_update()
    
    @abstractmethod
    def get_radar_data_in_frame():
        pass

    def frame_update(self) :
        self.get_radar_data_in_frame(self.radar_current_frame_num)

class RDBManager(RadarDataManager):
    def __init__(self, dataDir):
        super().__init__(dataDir)
        self.radar1_checkbox = True
        self.radar2_checkbox = False
        self.magicID = [b'M',b'V',b'R',b'S']

        self.loadCurser()
    
    def set_frame_ret_data(self, num_frame):
        self.set_to_this_frame(num_frame)

        return (self.radar1_object_list, self.radar1_track_list), (self.radar2_object_list, self.radar2_track_list), self.can_data

    def get_radar_data_in_frame(self, frame_index) :
        # pdb.set_trace()
        data_flag = 0
        self.radar1_object_list = []
        self.radar1_track_list = []
        self.radar2_object_list = []
        self.radar2_track_list = []
        
        if self.radar1_checkbox :
            if self.radar1_frame_index != [] :
                self.radar_file_handle.seek(self.radar1_frame_index[frame_index])         # find specific frame
                data_flag = 1
            if data_flag == 1 :
                data_type = self.radar_file_handle.read(1)
                data_length_bin = self.radar_file_handle.read(2)
                data_length = np.frombuffer(data_length_bin, dtype=np.uint8)
                payload_len = data_length[1]*256+data_length[0]+1       # data length doesn't have end marker byte. so, we have to plus 1 byte to get the frame end marker
                data_payload = self.radar_file_handle.read(payload_len)
                data_payload_uint = np.frombuffer(data_payload,dtype=np.uint8)
                if data_payload_uint[payload_len-1] != 95 :
                    # self.general_error_window("RADAR FILE ERROR","Frame end marker is missing.")
                    # self.error_flag = 1
                    # return
                    pass
                track_frame_number = data_payload_uint[3]*16777216+data_payload_uint[2]*65535+data_payload_uint[1]*256+data_payload_uint[0] # uint32 frame_number
                cpu_cycle_time = data_payload_uint[7]*16777216+data_payload_uint[6]*65535+data_payload_uint[5]*256+data_payload_uint[4] # uint32 timecpucycles
                num_object = data_payload_uint[11]*16777216+data_payload_uint[10]*65535+data_payload_uint[9]*256+data_payload_uint[8] # uint32 timecpucycles
                num_track = data_payload_uint[15]*16777216+data_payload_uint[14]*65535+data_payload_uint[13]*256+data_payload_uint[12] # uint32 timecpucycles
                sub_frame_num = data_payload_uint[19]*16777216+data_payload_uint[18]*65535+data_payload_uint[17]*256+data_payload_uint[16] # uint32 timecpucycles
                # print("FrameNumber : ", track_frame_number, "Detection point Number : ", num_object)
                ii=20
                if data_payload_uint[13] == 0 :     # number of tracks should be less than 100(by the setting), so, if data_payload_uint[13] is larger than 0, it is error
                    self.radar1_object_list = []
                    for jj in range(num_object) :
                        obj = radar_object()
                        read_data = data_payload[ii:(ii+34)]
                        if len(read_data) == 34 :
                            obj.range_idx                  = (read_data[ 1]*256 + read_data[ 0])    # is np.frombuffer faster than calculation?
                            obj.doppler_idx                = (read_data[ 3]*256 + read_data[ 2])
                            obj.range                      = radar_object.uint2int_16(read_data[ 5]*256 + read_data[ 4])/Q7_DIVISOR
                            obj.speed                      = radar_object.uint2int_16(read_data[ 7]*256 + read_data[ 6])/Q7_DIVISOR
                            obj.sin_azim                   = radar_object.uint2int_16(read_data[ 9]*256 + read_data[ 8])/Q14_DIVISOR
                            obj.peak_val                   = (read_data[11]*256 + read_data[10])/Q8_DIVISOR * 6
                            obj.range_snr_db               = (read_data[13]*256 + read_data[12])/Q8_DIVISOR * 6
                            obj.doppler_snr_db             = (read_data[15]*256 + read_data[14])/Q8_DIVISOR * 6
                            obj.sin_azim_srn_lin           = (read_data[17]*256 + read_data[16])/Q8_DIVISOR * 6
                            obj.x                          = radar_object.uint2int_16(read_data[19]*256 + read_data[18])/Q7_DIVISOR
                            obj.y                          = radar_object.uint2int_16(read_data[21]*256 + read_data[20])/Q7_DIVISOR
                            obj.z                          = radar_object.uint2int_16(read_data[23]*256 + read_data[22])/Q7_DIVISOR
                            obj.rotate_x                   = radar_object.uint2int_16(read_data[25]*256 + read_data[24])/Q7_DIVISOR
                            obj.rotate_y                   = radar_object.uint2int_16(read_data[27]*256 + read_data[26])/Q7_DIVISOR
                            obj.vel_disamb_fac_valid       = (read_data[29]*256 + read_data[28])
                            obj.vel_disamb_fac_valid_als   = (read_data[31]*256 + read_data[30])
                            obj.status_flag                = (read_data[33]*256 + read_data[32])
                            self.radar1_object_list.append(obj)
                        ii = ii+34

                    self.radar1_track_list = []
                    for jj in range(num_track) :
                        trk = radar_track()
                        read_data = data_payload[ii:(ii+22)]
                        if len(read_data) == 22 :
                            trk.id                         = (read_data[ 1]*256 + read_data[ 0])
                            trk.x                          = radar_object.uint2int_16((read_data[ 3]*256 + read_data[ 2]))/Q7_DIVISOR
                            trk.y                          = radar_object.uint2int_16((read_data[ 5]*256 + read_data[ 4]))/Q7_DIVISOR
                            trk.xd                         = radar_object.uint2int_16((read_data[ 7]*256 + read_data[ 6]))/Q7_DIVISOR
                            trk.yd                         = radar_object.uint2int_16((read_data[ 9]*256 + read_data[ 8]))/Q7_DIVISOR
                            trk.x_size                     = (read_data[11]*256 + read_data[10])/Q7_DIVISOR * 2
                            trk.y_size                     = (read_data[13]*256 + read_data[12])/Q7_DIVISOR * 2
                            trk.tick                       = (read_data[15]*256 + read_data[14])
                            trk.age                        = (read_data[17]*256 + read_data[16])
                            trk.z                           = radar_object.uint2int_16((read_data[19]*256 + read_data[18]))/Q7_DIVISOR
                            trk.z_size                      = radar_object.uint2int_16((read_data[21]*256 + read_data[20]))/Q7_DIVISOR +0.2
                            self.radar1_track_list.append(trk)
                        ii = ii+22

                self.can_data = []
                read_data = np.frombuffer(data_payload[ii:(ii+16)],dtype=np.uint8)
                if len(read_data) == 16 :  
                    for ii in range(16) :
                        self.can_data.append(read_data[ii])
                vehicle_steer_angle = (self.can_data[14]*256 + self.can_data[15])/10   # Steer Angle(deg)
                if vehicle_steer_angle > 3276.7 :
                    vehicle_steer_angle -= 6553.5
                vehicle_steer_angle = vehicle_steer_angle * 38.0/650.0

                # print("CAN DATA : ",self.can_data, "Steering : ", round(vehicle_steer_angle, 1))
        if self.radar2_checkbox :
            if self.radar2_frame_index != [] :
                self.radar_file_handle.seek(self.radar2_frame_index[frame_index])         # find specific frame
                data_flag = 1
            if data_flag == 1 :
                data_type = self.radar_file_handle.read(1)
                data_length_bin = self.radar_file_handle.read(2)
                data_length = np.frombuffer(data_length_bin, dtype=np.uint8)
                payload_len = data_length[1]*256+data_length[0]+1       # data length doesn't have end marker byte. so, we have to plus 1 byte to get the frame end marker
                data_payload = self.radar_file_handle.read(payload_len)
                data_payload_uint = np.frombuffer(data_payload,dtype=np.uint8)
                if data_payload_uint[payload_len-1] != 95 :
                    self.general_error_window("RADAR FILE ERROR","Frame end marker is missing.")
                    self.error_flag = 1
                    return
                track_frame_number = data_payload_uint[3]*16777216+data_payload_uint[2]*65535+data_payload_uint[1]*256+data_payload_uint[0] # uint32 frame_number
                cpu_cycle_time = data_payload_uint[7]*16777216+data_payload_uint[6]*65535+data_payload_uint[5]*256+data_payload_uint[4] # uint32 timecpucycles
                num_object = data_payload_uint[11]*16777216+data_payload_uint[10]*65535+data_payload_uint[9]*256+data_payload_uint[8] # uint32 timecpucycles
                num_track = data_payload_uint[15]*16777216+data_payload_uint[14]*65535+data_payload_uint[13]*256+data_payload_uint[12] # uint32 timecpucycles
                sub_frame_num = data_payload_uint[19]*16777216+data_payload_uint[18]*65535+data_payload_uint[17]*256+data_payload_uint[16] # uint32 timecpucycles
                # print("FrameNumber : ", track_frame_number, "Detection point Number : ", num_object)
                ii=20
                if data_payload_uint[13] == 0 :     # number of tracks should be less than 100(by the setting), so, if data_payload_uint[13] is larger than 0, it is error
                    self.radar2_object_list = []
                    for jj in range(num_object) :
                        obj = radar_object()
                        read_data = data_payload[ii:(ii+34)]
                        if len(read_data) == 34 :
                            obj.range_idx                  = (read_data[ 1]*256 + read_data[ 0])    # is np.frombuffer faster than calculation?
                            obj.doppler_idx                = (read_data[ 3]*256 + read_data[ 2])
                            obj.range                      = radar_object.uint2int_16(read_data[ 5]*256 + read_data[ 4])/Q7_DIVISOR
                            obj.speed                      = radar_object.uint2int_16(read_data[ 7]*256 + read_data[ 6])/Q7_DIVISOR
                            obj.sin_azim                   = radar_object.uint2int_16(read_data[ 9]*256 + read_data[ 8])/Q14_DIVISOR
                            obj.peak_val                   = (read_data[11]*256 + read_data[10])/Q8_DIVISOR * 6
                            obj.range_snr_db               = (read_data[13]*256 + read_data[12])/Q8_DIVISOR * 6
                            obj.doppler_snr_db             = (read_data[15]*256 + read_data[14])/Q8_DIVISOR * 6
                            obj.sin_azim_srn_lin           = (read_data[17]*256 + read_data[16])/Q8_DIVISOR * 6
                            obj.x                          = radar_object.uint2int_16(read_data[19]*256 + read_data[18])/Q7_DIVISOR
                            obj.y                          = radar_object.uint2int_16(read_data[21]*256 + read_data[20])/Q7_DIVISOR
                            obj.z                          = radar_object.uint2int_16(read_data[23]*256 + read_data[22])/Q7_DIVISOR
                            obj.rotate_x                   = radar_object.uint2int_16(read_data[25]*256 + read_data[24])/Q7_DIVISOR
                            obj.rotate_y                   = radar_object.uint2int_16(read_data[27]*256 + read_data[26])/Q7_DIVISOR
                            obj.vel_disamb_fac_valid       = (read_data[29]*256 + read_data[28])
                            obj.vel_disamb_fac_valid_als   = (read_data[31]*256 + read_data[30])
                            obj.status_flag                = (read_data[33]*256 + read_data[32])
                            self.radar2_object_list.append(obj)
                        ii = ii+34

                    self.radar2_track_list = []
                    for jj in range(num_track) :
                        trk = radar_track()
                        read_data = data_payload[ii:(ii+22)]
                        if len(read_data) == 22 :
                            trk.id                         = (read_data[ 1]*256 + read_data[ 0])
                            trk.x                          = radar_object.uint2int_16((read_data[ 3]*256 + read_data[ 2]))/Q7_DIVISOR
                            trk.y                          = radar_object.uint2int_16((read_data[ 5]*256 + read_data[ 4]))/Q7_DIVISOR
                            trk.xd                         = radar_object.uint2int_16((read_data[ 7]*256 + read_data[ 6]))/Q7_DIVISOR
                            trk.yd                         = radar_object.uint2int_16((read_data[ 9]*256 + read_data[ 8]))/Q7_DIVISOR
                            trk.x_size                     = (read_data[11]*256 + read_data[10])/Q7_DIVISOR +0.2
                            trk.y_size                     = (read_data[13]*256 + read_data[12])/Q7_DIVISOR +0.2
                            trk.tick                       = (read_data[15]*256 + read_data[14])
                            trk.age                        = (read_data[17]*256 + read_data[16])
                            trk.z                           = radar_object.uint2int_16((read_data[19]*256 + read_data[18]))/Q7_DIVISOR
                            trk.z_size                      = radar_object.uint2int_16((read_data[21]*256 + read_data[20]))/Q7_DIVISOR +0.2
                            self.radar2_track_list.append(trk)
                        ii = ii+22

                

                # print("CAN DATA : ",self.can_data, "Steering : ", round(vehicle_steer_angle, 1))

class SDBManager(RadarDataManager):
    def __init__(self, dataDir):
        super().__init__(dataDir)

        self.magicID = [b'R',b'S',b'I',b'M']

        self.loadCurser()

    def get_radar_data_in_frame(self, frame_index) :
        data_flag = 0
        self.radar1_object_list = []
        self.radar1_track_list = []
        self.radar2_object_list = []
        self.radar2_track_list = []
        if self.radar1_checkbox :
            if self.radar1_frame_index != [] :
                self.radar_file_handle.seek(self.radar1_frame_index[frame_index])         # find specific frame
                data_flag = 1
            if data_flag == 1 :
                data_type = self.radar_file_handle.read(1)
                data_length_bin = self.radar_file_handle.read(2)
                data_length = np.frombuffer(data_length_bin, dtype=np.uint8)
                payload_len = data_length[1]*256+data_length[0]+1       # data length doesn't have end marker byte. so, we have to plus 1 byte to get the frame end marker
                data_payload = self.radar_file_handle.read(payload_len)
                data_payload_uint = np.frombuffer(data_payload,dtype=np.uint8)
                if data_payload_uint[payload_len-1] != 95 :
                    self.general_error_window("SIMULATED DATA FILE ERROR","Frame end marker is missing.")
                    self.error_flag = 1
                    return
                track_frame_number = data_payload_uint[3]*16777216+data_payload_uint[2]*65535+data_payload_uint[1]*256+data_payload_uint[0]     # uint32 frame_number
                cpu_cycle_time = data_payload_uint[7]*16777216+data_payload_uint[6]*65535+data_payload_uint[5]*256+data_payload_uint[4]         # uint32 timecpucycles
                num_object = data_payload_uint[11]*16777216+data_payload_uint[10]*65535+data_payload_uint[9]*256+data_payload_uint[8]           # uint32 timecpucycles
                num_track = data_payload_uint[15]*16777216+data_payload_uint[14]*65535+data_payload_uint[13]*256+data_payload_uint[12]          # uint32 timecpucycles
                sub_frame_num = data_payload_uint[19]*16777216+data_payload_uint[18]*65535+data_payload_uint[17]*256+data_payload_uint[16]      # uint32 timecpucycles
                ii=20
                if (frame_index != track_frame_number) :
                    print("simulation frame is wierd")
                
                if data_payload_uint[13] == 0 :
                    for jj in range(num_object) :
                        obj = simulated_object()
                        read_data = data_payload[ii:(ii+34)]
                        if len(read_data) == 34 :
                            obj.detID                       = (read_data[ 1]*256 + read_data[ 0])
                            obj.clusterId                   = (read_data[ 3]*256 + read_data[ 2])
                            obj.rangeIdx                    = (read_data[ 5]*256 + read_data[ 4])
                            obj.dopplerIdx                  = (read_data[ 7]*256 + read_data[ 6])
                            obj.range                       = radar_object.uint2int_16((read_data[ 9]*256 + read_data[ 8]))/Q7_DIVISOR
                            obj.speed                       = radar_object.uint2int_16((read_data[11]*256 + read_data[10]))/Q7_DIVISOR
                            obj.sinAzim                     = radar_object.uint2int_16((read_data[13]*256 + read_data[12]))/Q14_DIVISOR
                            obj.rangeVal                    = radar_object.uint2int_16((read_data[15]*256 + read_data[14]))/Q7_DIVISOR
                            obj.rangeSNR                    = (read_data[17]*256 + read_data[16])/Q8_DIVISOR
                            obj.dopplerSNR                  = (read_data[19]*256 + read_data[18])/Q8_DIVISOR
                            obj.angleSNR                    = (read_data[21]*256 + read_data[20])/Q8_DIVISOR
                            obj.x                           = radar_object.uint2int_16((read_data[23]*256 + read_data[22]))/Q7_DIVISOR
                            obj.y                           = radar_object.uint2int_16((read_data[25]*256 + read_data[24]))/Q7_DIVISOR
                            obj.velDisambFacValid           = (read_data[27]*256 + read_data[26])
                            obj.status_flag                 = (read_data[29]*256 + read_data[28])
                            obj.xd                          = radar_object.uint2int_16((read_data[31]*256 + read_data[30]))/Q7_DIVISOR
                            obj.yd                          = radar_object.uint2int_16((read_data[33]*256 + read_data[32]))/Q7_DIVISOR
                            self.radar1_object_list.append(obj)
                        ii = ii+34
                    for jj in range(num_track) :
                        trk = simulated_track()
                        read_data = data_payload[ii:(ii+44)]
                        if len(read_data) == 44 :
                            trk.ID                          = (read_data[ 1]*256 + read_data[ 0])
                            trk.SNR                         = (read_data[ 3]*256 + read_data[ 2])/Q8_DIVISOR
                            trk.age                         = (read_data[ 5]*256 + read_data[ 4])
                            trk.associ_ID                   = radar_object.uint2int_16(read_data[ 7]*256 + read_data[ 6])
                            trk.dopplerSNR                  = (read_data[ 9]*256 + read_data[ 8])/Q8_DIVISOR
                            trk.measVectorRRD_R             = radar_object.uint2int_16((read_data[11]*256 + read_data[10]))/Q7_DIVISOR
                            trk.measVectorRRD_vel           = radar_object.uint2int_16((read_data[13]*256 + read_data[12]))/Q7_DIVISOR
                            trk.measVectorRRD_Theta         = radar_object.uint2int_16((read_data[15]*256 + read_data[14]))/Q7_DIVISOR
                            trk.statVecXYZ_x                = radar_object.uint2int_16((read_data[17]*256 + read_data[16]))/Q7_DIVISOR
                            trk.statVecXYZ_y                = radar_object.uint2int_16((read_data[19]*256 + read_data[18]))/Q7_DIVISOR
                            trk.statVecXYZ_xd               = radar_object.uint2int_16((read_data[21]*256 + read_data[20]))/Q7_DIVISOR
                            trk.statVecXYZ_yd               = radar_object.uint2int_16((read_data[23]*256 + read_data[22]))/Q7_DIVISOR
                            trk.peakVal                     = (read_data[25]*256 + read_data[24])/Q8_DIVISOR
                            trk.prevXd                      = radar_object.uint2int_16((read_data[27]*256 + read_data[26]))/Q7_DIVISOR
                            trk.prevYd                      = radar_object.uint2int_16((read_data[29]*256 + read_data[28]))/Q7_DIVISOR
                            trk.rangeSNR                    = (read_data[31]*256 + read_data[30])/Q8_DIVISOR
                            trk.tick                        = (read_data[33]*256 + read_data[32])
                            trk.xSize                       = radar_object.uint2int_16((read_data[35]*256 + read_data[34]))/Q7_DIVISOR
                            trk.ySize                       = radar_object.uint2int_16((read_data[37]*256 + read_data[36]))/Q7_DIVISOR
                            trk.plotValidity                = (read_data[39]*256 + read_data[38])
                            trk.Status_Flag0                = (read_data[41]*256 + read_data[40]) # Static : 1, Comming : 2, Going : 4
                            trk.Status_Flag1                = (read_data[43]*256 + read_data[42]) # is associated : 1, is not associated : 0
                            self.radar1_track_list.append(trk)
                        ii = ii+44
                     
                    self.can_data = []
                    read_data = np.frombuffer(data_payload[ii:(ii+16)],dtype=np.uint8)
                    if len(read_data) == 16 :
                        for ii in range(16) :
                            self.can_data.append(read_data[ii])
                    print("CAN DATA : ", self.can_data)
        
        if self.radar2_checkbox :
            if self.radar2_frame_index != [] :
                self.radar_file_handle.seek(self.radar2_frame_index[frame_index])         # find specific frame
                data_flag = 1
            if data_flag == 1 :
                data_type = self.radar_file_handle.read(1)
                data_length_bin = self.radar_file_handle.read(2)
                data_length = np.frombuffer(data_length_bin, dtype=np.uint8)
                payload_len = data_length[1]*256+data_length[0]+1       # data length doesn't have end marker byte. so, we have to plus 1 byte to get the frame end marker
                data_payload = self.radar_file_handle.read(payload_len)
                data_payload_uint = np.frombuffer(data_payload,dtype=np.uint8)
                if data_payload_uint[payload_len-1] != 95 :
                    self.general_error_window("SIMULATED DATA FILE ERROR","Frame end marker is missing.")
                    self.error_flag = 1
                    return
                track_frame_number = data_payload_uint[3]*16777216+data_payload_uint[2]*65535+data_payload_uint[1]*256+data_payload_uint[0]     # uint32 frame_number
                cpu_cycle_time = data_payload_uint[7]*16777216+data_payload_uint[6]*65535+data_payload_uint[5]*256+data_payload_uint[4]         # uint32 timecpucycles
                num_object = data_payload_uint[11]*16777216+data_payload_uint[10]*65535+data_payload_uint[9]*256+data_payload_uint[8]           # uint32 timecpucycles
                num_track = data_payload_uint[15]*16777216+data_payload_uint[14]*65535+data_payload_uint[13]*256+data_payload_uint[12]          # uint32 timecpucycles
                sub_frame_num = data_payload_uint[19]*16777216+data_payload_uint[18]*65535+data_payload_uint[17]*256+data_payload_uint[16]      # uint32 timecpucycles
                ii=20
                if (frame_index != track_frame_number) :
                    print("simulation frame is wierd")
                
                if data_payload_uint[13] == 0 :
                    for jj in range(num_object) :
                        obj = simulated_object()
                        read_data = data_payload[ii:(ii+34)]
                        if len(read_data) == 34 :
                            obj.detID                       = (read_data[ 1]*256 + read_data[ 0])
                            obj.clusterId                   = (read_data[ 3]*256 + read_data[ 2])
                            obj.rangeIdx                    = (read_data[ 5]*256 + read_data[ 4])
                            obj.dopplerIdx                  = (read_data[ 7]*256 + read_data[ 6])
                            obj.range                       = radar_object.uint2int_16((read_data[ 9]*256 + read_data[ 8]))/Q7_DIVISOR
                            obj.speed                       = radar_object.uint2int_16((read_data[11]*256 + read_data[10]))/Q7_DIVISOR
                            obj.sinAzim                     = radar_object.uint2int_16((read_data[13]*256 + read_data[12]))/Q14_DIVISOR
                            obj.rangeVal                    = radar_object.uint2int_16((read_data[15]*256 + read_data[14]))/Q7_DIVISOR
                            obj.rangeSNR                    = (read_data[17]*256 + read_data[16])/Q8_DIVISOR
                            obj.dopplerSNR                  = (read_data[19]*256 + read_data[18])/Q8_DIVISOR
                            obj.angleSNR                    = (read_data[21]*256 + read_data[20])/Q8_DIVISOR
                            obj.x                           = radar_object.uint2int_16((read_data[23]*256 + read_data[22]))/Q7_DIVISOR
                            obj.y                           = radar_object.uint2int_16((read_data[25]*256 + read_data[24]))/Q7_DIVISOR
                            obj.velDisambFacValid           = (read_data[27]*256 + read_data[26])
                            obj.status_flag                 = (read_data[29]*256 + read_data[28])
                            obj.xd                          = radar_object.uint2int_16((read_data[31]*256 + read_data[30]))/Q7_DIVISOR
                            obj.yd                          = radar_object.uint2int_16((read_data[33]*256 + read_data[32]))/Q7_DIVISOR
                            self.radar2_object_list.append(obj)
                        ii = ii+34
        
                    for jj in range(num_track) :
                        trk = simulated_track()
                        read_data = data_payload[ii:(ii+44)]
                        if len(read_data) == 44 :
                            trk.ID                          = (read_data[ 1]*256 + read_data[ 0])
                            trk.SNR                         = (read_data[ 3]*256 + read_data[ 2])/Q8_DIVISOR
                            trk.age                         = (read_data[ 5]*256 + read_data[ 4])
                            trk.associ_ID                   = radar_object.uint2int_16(read_data[ 7]*256 + read_data[ 6])
                            trk.dopplerSNR                  = (read_data[ 9]*256 + read_data[ 8])/Q8_DIVISOR
                            trk.measVectorRRD_R             = radar_object.uint2int_16((read_data[11]*256 + read_data[10]))/Q7_DIVISOR
                            trk.measVectorRRD_vel           = radar_object.uint2int_16((read_data[13]*256 + read_data[12]))/Q7_DIVISOR
                            trk.measVectorRRD_Theta         = radar_object.uint2int_16((read_data[15]*256 + read_data[14]))/Q7_DIVISOR
                            trk.statVecXYZ_x                = radar_object.uint2int_16((read_data[17]*256 + read_data[16]))/Q7_DIVISOR
                            trk.statVecXYZ_y                = radar_object.uint2int_16((read_data[19]*256 + read_data[18]))/Q7_DIVISOR
                            trk.statVecXYZ_xd               = radar_object.uint2int_16((read_data[21]*256 + read_data[20]))/Q7_DIVISOR
                            trk.statVecXYZ_yd               = radar_object.uint2int_16((read_data[23]*256 + read_data[22]))/Q7_DIVISOR
                            trk.peakVal                     = (read_data[25]*256 + read_data[24])/Q8_DIVISOR
                            trk.prevXd                      = radar_object.uint2int_16((read_data[27]*256 + read_data[26]))/Q7_DIVISOR
                            trk.prevYd                      = radar_object.uint2int_16((read_data[29]*256 + read_data[28]))/Q7_DIVISOR
                            trk.rangeSNR                    = (read_data[31]*256 + read_data[30])/Q8_DIVISOR
                            trk.tick                        = (read_data[33]*256 + read_data[32])
                            trk.xSize                       = radar_object.uint2int_16((read_data[35]*256 + read_data[34]))/Q7_DIVISOR
                            trk.ySize                       = radar_object.uint2int_16((read_data[37]*256 + read_data[36]))/Q7_DIVISOR
                            trk.plotValidity                = (read_data[39]*256 + read_data[38])
                            trk.Status_Flag0                = (read_data[41]*256 + read_data[40])
                            trk.Status_Flag1                = (read_data[43]*256 + read_data[42])
                            self.radar2_track_list.append(trk)
                        ii = ii+44

    def radar_data_rotation_merge(self) : 
        self.radar_object_list = []
        self.radar_track_list = []
        if self.radar_flip_checkbox :
            angle1 = math.pi/180*(RIGHT_ANGLE_DEG)
            angle2 = math.pi/180*(LEFT_ANGLE_DEG + 180)
            offsetx1 = CAR_WIDTH_M*0.5
            offsetx2 = -CAR_WIDTH_M*0.5
        else :
            angle1 = math.pi/180*(LEFT_ANGLE_DEG + 180)
            angle2 = math.pi/180*(RIGHT_ANGLE_DEG)
            offsetx1 = -CAR_WIDTH_M*0.5
            offsetx2 = CAR_WIDTH_M*0.5
        cosa1 = math.cos(angle1)
        sina1 = math.sin(angle1)
        cosa2 = math.cos(angle2)
        sina2 = math.sin(angle2)

        if self.radar1_checkbox :
            for ii in range(len(self.radar1_object_list)) :
                self.radar1_object_list[ii].rot_x = self.radar1_object_list[ii].x * sina1 + self.radar1_object_list[ii].y * cosa1 + offsetx1
                self.radar1_object_list[ii].rot_dx = self.radar1_object_list[ii].xd * sina1 + self.radar1_object_list[ii].yd * cosa1
                self.radar1_object_list[ii].rot_y = -self.radar1_object_list[ii].x * cosa1 + self.radar1_object_list[ii].y * sina1
                self.radar1_object_list[ii].rot_dy = -self.radar1_object_list[ii].xd * cosa1 + self.radar1_object_list[ii].yd * sina1
                self.radar_object_list.append(self.radar1_object_list[ii])
            for ii in range(len(self.radar1_track_list)) :
                # if self.radar1_track_list[ii].plotValidity == 1 :
                self.radar1_track_list[ii].rot_x = self.radar1_track_list[ii].statVecXYZ_x * sina1 + self.radar1_track_list[ii].statVecXYZ_y * cosa1 + offsetx1
                self.radar1_track_list[ii].rot_dx = self.radar1_track_list[ii].statVecXYZ_xd * sina1 + self.radar1_track_list[ii].statVecXYZ_yd * cosa1
                self.radar1_track_list[ii].rot_y = -self.radar1_track_list[ii].statVecXYZ_x * cosa1 + self.radar1_track_list[ii].statVecXYZ_y * sina1
                self.radar1_track_list[ii].rot_dy = -self.radar1_track_list[ii].statVecXYZ_xd * cosa1 + self.radar1_track_list[ii].statVecXYZ_yd * sina1
                self.radar_track_list.append(self.radar1_track_list[ii])
        if self.radar2_checkbox :
            for ii in range(len(self.radar2_object_list)) :
                self.radar2_object_list[ii].rot_x = self.radar2_object_list[ii].x * sina2 + self.radar2_object_list[ii].y * cosa2 + offsetx2
                self.radar2_object_list[ii].rot_dx = self.radar2_object_list[ii].xd * sina2 + self.radar2_object_list[ii].yd * cosa2
                self.radar2_object_list[ii].rot_y = -self.radar2_object_list[ii].x * cosa2 + self.radar2_object_list[ii].y * sina2
                self.radar2_object_list[ii].rot_dy = -self.radar2_object_list[ii].xd * cosa2 + self.radar2_object_list[ii].yd * sina2
                self.radar_object_list.append(self.radar2_object_list[ii])
            for ii in range(len(self.radar2_track_list)) :
                # if self.radar2_track_list[ii].plotValidity == 1 :
                self.radar2_track_list[ii].rot_x = self.radar2_track_list[ii].statVecXYZ_x * sina2 + self.radar2_track_list[ii].statVecXYZ_y * cosa2 + offsetx2
                self.radar2_track_list[ii].rot_dx = self.radar2_track_list[ii].statVecXYZ_xd * sina2 + self.radar2_track_list[ii].statVecXYZ_yd * cosa2
                self.radar2_track_list[ii].rot_y = -self.radar2_track_list[ii].statVecXYZ_x * cosa2 + self.radar2_track_list[ii].statVecXYZ_y * sina2
                self.radar2_track_list[ii].rot_dy = -self.radar2_track_list[ii].statVecXYZ_xd * cosa2 + self.radar2_track_list[ii].statVecXYZ_yd * sina2
                self.radar_track_list.append(self.radar2_track_list[ii])   
    
    def frame_update(self) :
        self.get_radar_data_in_frame(self.radar_current_frame_num)       # simulation_flag = 0, radar_num = 0
        self.radar_data_rotation_merge()
        # if (self.video_current_frame_num >= self.video_end_frame_num) :
        #     self.update_video_frame(self.video_end_frame_num-1)
        # elif (self.video_current_frame_num < 0) :
        #     self.update_video_frame(0)
        # else :
        # 
        # 

class RadarDatasetManager():
    def __init__(self, inputDir, prefix = ''):
        try:
            radarFiles = []
            for p in inputDir if isinstance(inputDir, list) else [inputDir]:
                p_path = Path(p)  # os-agnostic
                if p_path.is_dir():  # dir
                    radarFiles += glob.glob(str(p_path / '**' / '*.*'), recursive=True)
                elif p_path.is_file():
                    radarFiles += [p]
                else:
                    raise Exception(f'{prefix}{p} does not exist')

            self.radarFiles = sorted(x.replace('/', os.sep) for x in radarFiles if x.split('.')[-1].lower() in RDB_FORMAT)

        except Exception as e:
            raise Exception(f'{prefix}Error loading data from {inputDir}: {e}')

        self.radarFiles.sort(key = lambda x: x.split('/')[-1]) # sort list by video File Name.

    def __len__(self):
        return len(self.radarFiles)

    def __getitem__(self, idx):
        radarFile = self.radarFiles[idx]

        radarManger = RDBManager(radarFile)

        return radarManger, radarFile

if __name__ == "__main__":
    objRDB = RDBManager('/mnt/d/Datasets/__RADAR/AMR/20230803_hmc_poc2/20230803_103652.rdb')
    # obj = SDBManager('/mnt/d/Datasets/__RADAR/KIA_BSD/test/20230327_161544.sdb')
    objRDB.set_to_this_frame(0)
    print(objRDB.radar1_object_list)
    # pdb.set_trace()