import sys
import CONST
import numpy as np
import enum
import configManager_MRR_DEMO as cfgMan
# import mmwavelib as ml

N_STATE = 4
N_MEAS = 3
if CONST.IMM:
	max_dim_state = 5
	if CONST.IMM_CA:
		max_dim_state = 6
else:
	max_dim_state = 4

class rrd(enum.Enum):
    RANGE = 0
    RANGE_RATE = 1
    SINAZIM = 2
    
class xyz(enum.Enum):
    x = 0
    y = 1
    xd = 2
    yd = 3

class dataManager:
	def __init__(self, config):
		self.adcData = np.zeros((config.numADCSamples, config.n_chirps, config.numRx*config.numTx), dtype = np.complex64)
		self.FFTData = np.zeros((config.numADCSamples, config.n_chirps, config.numRx*config.numTx), dtype = np.complex64)
		self.adcData_Slow = np.zeros((config.numADCSamples, config.n_chirps, config.numRx*config.numTx), dtype = np.complex64)
		self.FFTData_Slow = np.zeros((config.numADCSamples, config.n_chirps, config.numRx*config.numTx), dtype = np.complex64)
	def reset(self):
		self.adcData = np.zeros((cfgMan.config.numADCSamples, cfgMan.config.n_chirps, cfgMan.config.numRx*cfgMan.config.numTx), dtype = np.complex64)
		self.FFTData = np.zeros((cfgMan.config.numADCSamples, cfgMan.config.n_chirps, cfgMan.config.numRx*cfgMan.config.numTx), dtype = np.complex64)
		self.adcData_Slow = np.zeros((cfgMan.config.numADCSamples, cfgMan.config.n_chirps, cfgMan.config.numRx*cfgMan.config.numTx), dtype = np.complex64)
		self.FFTData_Slow = np.zeros((cfgMan.config.numADCSamples, cfgMan.config.n_chirps, cfgMan.config.numRx*cfgMan.config.numTx), dtype = np.complex64)

class domainGenerator:
	def __init__(self, config, aoaCfg):
		# self.rangeDomain = np.arange(-config.numADCSamples/2, config.numADCSamples/2)*config.SampleRate/config.numADCSamples/config.slope_Hz_per_sec*config.lightSpeed_meters_per_sec/2
		self.rangeDomain = np.arange(0, config.numADCSamples)*config.SampleRate/config.numADCSamples/config.slope_Hz_per_sec*config.lightSpeed_meters_per_sec/2
		# self.dopplerDomain = np.arange(-config.n_chirps/2, config.n_chirps/2)/config.n_chirps/77e9*config.lightSpeed_meters_per_sec/4/config.period_chirp*2*(3600/1000)
		self.dopplerDomain = np.arange(-config.n_chirps/2, config.n_chirps/2)/config.n_chirps/77e9*config.lightSpeed_meters_per_sec/4/config.period_chirp*2
		self.dopplerDomain_Slow = np.arange(-config.n_chirps/2, config.n_chirps/2)/config.n_chirps/77e9*config.lightSpeed_meters_per_sec/4/config.period_chirp_slow*2
		if aoaCfg.procFFT == True:
			self.angleDomain = np.arcsin(np.arange(-aoaCfg.numAngleBins/2, aoaCfg.numAngleBins/2) / aoaCfg.numAngleBins * 2)
			# print(self.angleDomain)

		else:
			self.angleDomain = np.arange(-1*aoaCfg.maxAngle,aoaCfg.maxAngle,aoaCfg.deltaTheta)


		self.angleDomainH = np.arange(-1 * aoaCfg.maxAngleH, aoaCfg.maxAngleH, aoaCfg.deltaTheta)
		# self.angle2rangeDomain = 

	def getResolution(self):

		self.rangeRes = self.rangeDomain[1] - self.rangeDomain[0]
		self.dopplerRes = self.dopplerDomain[1] - self.dopplerDomain[0]
		self.angleRes = self.angleDomain[1] - self.angleDomain[0]
		self.dopplerRes_Slow = self.dopplerDomain_Slow[1] - self.dopplerDomain_Slow[0]

		self.resList = [self.rangeRes, self.dopplerRes, self.angleRes, self.dopplerRes_Slow]

		return self.resList

class genTracker:
	def __init__(self):
		self.List = []
		self.IDlist  = []

	def appendDetObj(self, trackerElem):
		if len(self.IDlist) > cfgMan.trackingCfg.maxTracker:
			print("ERROR : Tracking Object is spill over!")
			sys.exit(1)
		ID = 0
		while 1:		
			if ID in self.IDlist:
				# print ("%d is in list" %ID)
				ID += 1
			else:
				trackerElem.ID = ID				
				break

		self.List.append(trackerElem)
		self.IDlist.append(ID)

	def rmObj(self, obj):
		self.IDlist.remove(obj.ID)
		self.List.remove(obj)
		del(obj)

	def resetTrackerAssociated(self):
		for tracker in self.List:
			tracker.associatedObj = None

	def reset(self):
		self.List = []

	def arrangeTracksByAge(self):

		if not (len(self.List) <= 1):
			i = 0
			j = len(self.List) - 1
			while (i != j):
				# print(i,j)
				currTrackFwd = self.List[i]
				currTrackRev = self.List[j]

				if currTrackFwd.validity == True and currTrackFwd.tick < cfgMan.trackingCfg.thresholdTick:
					if currTrackRev.tick >= cfgMan.trackingCfg.thresholdTick and currTrackRev.validity == True:
						tempObject = currTrackRev
						tempID = self.IDlist[j]
						self.List[j] = currTrackFwd
						self.IDlist[j] = self.IDlist[i]
						self.List[i] = tempObject
						self.IDlist[i] = tempID
						i += 1
						if i == j:
							break
						j -= 1
						if i == j:
							break
					else:
						j -= 1
						if i == j:
							break
				else:
					i += 1
					if i == j:
						break

class trackingInput:
	def __init__(self):
		# State Vector (x,y,xd,yd)
		self.measVectorRRD = np.zeros(N_MEAS, dtype=np.float)  # z_k # range, speed, theta
		self.measCovVec = np.zeros(N_MEAS, dtype=np.float)  #
		self.stateVectorXYZ = np.zeros(N_STATE, dtype=np.float)  # x_hat_k_-
		self.Covariance = np.zeros((N_STATE, N_STATE), dtype=np.float)  # P

# ==============IMM=====================
		self.state = [trackerState(max_dim_state), trackerState(max_dim_state)]

		self.mixingProb = np.zeros((2,2), dtype = np.float)
		self.mu = np.zeros(2, dtype = np.float)

		self.prevXd = 0
		self.prevYd = 0
#=========================================

		self.mahalanobisDistance = 0

#===========DEBUG=====================
		self.prevCovariance = np.zeros((N_STATE, N_STATE), dtype=np.float) #P(k|k-1) mode 0
		self.prevStateVectorXYZ = np.zeros(N_STATE, dtype=np.float) #x(k|k-1) mode 0
		self.H_tmp = np.zeros((3, max_dim_state), dtype=np.float)
		self.predictedXYZ0 = np.zeros(N_MEAS, dtype=np.float)
		self.predictedXYZ1 = np.zeros(N_MEAS, dtype=np.float)
#=====================================

		self.age = 0
		self.tick = 0

		self.xSize = 0
		self.ySize = 0

		self.validity = False  # Target Out validity tick이 10점을 넘으면 이후로 계속 플롯

		self.associatedObj = None
		self.isAssociated = False

		self.ID = None

		self.SNR = 0  # rangeSNR
		self.RCS = 0  # log2RCS = 4* log2Range + log2peakValRange
		self.RCS_variation = 0
		self.RCS_slope = 0
		self.angleSNR = 0

		self.plotValidity = False

		self.pointNum = 0
		self.cov_mahalanobis = np.zeros((2, 2), dtype=np.float)


		self.rangeSNR = 0
		self.dopplerSNR = 0
		self.angleSNR = 0
		self.peakVal = 0

		self.statusFlag = -1
		self.transitionScore = 0

		self.mode = 0

class trackerState:
	def __init__(self, ndim):
		self.stateVectorXYZ = np.zeros(ndim, dtype = np.float)  # x_hat_k_-
		self.Covariance     = np.zeros((ndim, ndim), dtype = np.float)  # P
		self.initCondstate  = np.zeros(ndim, dtype = np.float)
		self.initCondCov    = np.zeros((ndim, ndim), dtype = np.float)

class cfarOutFmt1D:
	def __init__(self):
		self.idx = 0
		self.val = 0
		self.SNR = 0
		self.isSlow = False

class cfarOutFmt2D:
	def __init__(self):
		self.detID = -1

		self.rangeVal = 0
		self.dopplerVal = 0 #unit is dB, (log2)
		
		self.rangeIdx = 0
		self.dopplerIdx = 0
			
		self.rangeSNR = 0		
		self.dopplerSNR = 0

		self.velDisambFacValidity = -1
		self.CRT_slow_doppler_indx = -1

		self.velDisambFacValidity_2 = -1
		self.CRT_slow_doppler_indx_2 = -1

		self.velDisambFacValidity_als = -1
		self.CRT_slow_doppler_indx_als = -1

		self.velDisambFacValidity_als_2 = -1
		self.CRT_slow_doppler_indx_als_2 = -1

		self.CRT_Flag = 0 #"NON_MATCHED = 0", "SINGLE_MATCHED = 1", "DOUBLE_MATCHED = 2", "DIVIDED = 3"
		self.CRT_Flag_als = 0 #"NON_MATCHED = 0", "SINGLE_MATCHED = 1", "DOUBLE_MATCHED = 2", "DIVIDED = 3"

		self.dopplerOffset = 0
		self.isSlow         = False

class clusterOutput:
	def __init__(self):

		self.strongestMember = None #this obj will be trackingInput // type : cfarOutFmt3D
		self.trackingInput = trackingInput() # Type : class trackingInput

		self.clusterId = -1
		self.xCenter = 0
		self.yCenter = 0
		self.xSize = 0
		self.ySize = 0
		self.avgVel = 0
		self.peakVal = 0

		self.isSlow = False

class cfarOutFmt3D:
	def __init__(self):
		self.detID = -1

		self.rangeIdx = 0
		self.dopplerIdx = 0
		self.angleIdx = 0

		self.rangeVal = 0
		self.rangeSNR = 0
		
		self.dopplerVal = 0
		self.dopplerSNR = 0
		
		self.angleVal = 0
		self.angleSNR = 0
		self.angleOffset = 0
		#=======measurement=====#
		self.range = 0
		self.speed = 0
		# self.theta = 0
		self.sinAzim = 0

		self.x = 0
		self.y = 0
		self.xd = 0
		self.yd = 0

		self.isAliasing = False
		self.isSlowProc = False
		#=====clustering=====#
		self.clusterId = -1 # if the clusterId is -1, then the obj is Noise Point.
		self.visited = False
		self.scope = False # is grouped?

		#=====EnhancedMaxVel=====#
		self.velDisambFacValidity = -1
		self.dopplerOffset = 0

		self.statusFlag = -1
		self.isSlow = False

		self.rotatex = 0
		self.rotatey = 0


class MmwDemo_objRaw2D_handMade: #for tracking DEBUG
	def __init__(self):
		self.x = 0
		self.y = 0
		self.xd = 0
		self.yd = 0

		self.peakVal = 0

		self.rangeSNRdB = 0
		self.dopplerSNRdB = 0
		self.sinAzimSNRLin = 0

		self.xSize = 0
		self.ySize = 0

class dopplerBoundary:
	def __init__(self):
		self.uB0 = 0
		self.lB0 = 0
		self.uB1 = 0
		self.lB1 = 0
		self.uB2 = 0
		self.lB2 = 0

class radar_obj():
    def __init__(self):
        self.range_idx                  = 0
        self.doppler_idx                = 0
        self.range                      = 0
        self.speed                      = 0
        self.sin_azim                   = 0
        self.peak_val                   = 0
        self.range_snr_db               = 0
        self.doppler_snr_db             = 0
        self.sin_azim_srn_lin           = 0 #dB
        self.x                          = 0
        self.y                          = 0
        self.z                          = 0
        self.rotate_x					= 0
        self.rotate_y 					= 0
        self.vel_disamb_fac_valid       = 0
        self.vel_disamb_fac_valid_als   = 0
        self.status_flag                = 0

    def uint2int(self, input_data):
        if input_data > 32767:
            input_data = -(65535 - input_data + 1)
        return input_data

class radar_trk():
    def __init__(self):
        self.id                         = 0
        self.x                          = 0
        self.y                          = 0
        self.xd                         = 0
        self.yd                         = 0
        self.x_size                     = 0
        self.y_size                     = 0
        self.tick                       = 0
        self.age                        = 0
        self.flag                       = 0
        self.reserved0                  = 0

class radar_info():
	def __init__(self):
		self.mode = 0
		self.position = 0
		self.angle = 0