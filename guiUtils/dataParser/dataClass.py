

RADAR_OBJECT_ATTRIBUTES_LIST = ["range_idx","doppler_idx","range","speed","sin_azim",\
                                "peak_val","range_snr_db","doppler_snr_db","sin_azim_srn_lin","x",\
                                "y","z","rotate_x","rotate_y","vel_disamb_fac_valid",\
                                "vel_disamb_fac_valid_als","status_flag"]
RADAR_TRACK_ATTRIBUTES_LIST = ["id","x","y","xd","yd",\
                                "x_size","y_size","tick","age","flag",\
                                "reserved0"]
SIMULATED_OBJECT_ATTRIBUTES_LIST = ["detID","clusterId","rangeIdx","dopplerIdx","range",\
                                    "speed","sinAzim","rangeVal","rangeSNR","dopplerSNR",\
                                    "angleSNR","x","y","velDisambFacValid","status_flag",\
                                    "xd","yd"] 
SIMULATED_TRACK_ATTRIBUTES_LIST = ["ID","SNR","age","associ_ID","dopplerSNR",\
                                    "measVectorRRD_R","measVectorRRD_vel","measVectorRRD_Theta","statVecXYZ_x","statVecXYZ_y",\
                                    "statVecXYZ_xd","statVecXYZ_yd","peakVal","prevXd","prevYd",\
                                    "rangeSNR","tick","xSize","ySize","plotValidity",\
                                    "Status_Flag0","Status_Flag1"]

#####################################################################################
#   Class ==> Object and Track                                                      #
#####################################################################################
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