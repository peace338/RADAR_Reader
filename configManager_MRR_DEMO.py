import numpy as np
import CONST
if CONST.TDM_MIMO:
	import mrr_config_chirp_design_MRR160_TDM_MIMO as MRRchirpCfg
else:
	import mrr_config_chirp_design_MRR160 as MRRchirpCfg

INF = 16383


class config:
	numRx = MRRchirpCfg.numRx
	numTx = MRRchirpCfg.numTx

	SampleRate = MRRchirpCfg.SampleRate  #unit SPS(Sample per seconds)
	numADCSamples = MRRchirpCfg.numADCSamples  #unit None

	numADCBits = 16
	numLanes = 2
	isReal = 0 
	
	
	n_chirps = MRRchirpCfg.n_chirps #half of number of chirp, subframe0 = fastchirp, subfrmae1 = slowchirp.
	
	slope_Hz_per_sec = MRRchirpCfg.slope_Hz_per_sec #Unit Hz per Sec

	period_chirp = MRRchirpCfg.period_chirp #Unit sec

	period_chirp_slow = MRRchirpCfg.period_chirp_slow #unit sec

	lightSpeed_meters_per_sec = 3e8 #Unit meter per seconds

	carrierFreq = MRRchirpCfg.carrierFreq #Unit Hz

	winType = "HANNING_RECT" # "HAMMING", "HANNING", "HANNING_RECT", "None"


class configUSRR:
	numRx = 4
	numTx = 3

	SampleRate = 4652e3
	numADCSamples = 512 
	numADCBits = 16 
	numLanes = 2 
	isReal = 0

	n_chirps = 96
	
	slope_Hz_per_sec = 4e12
	period_chirp = 65e-6
	period_chirp_slow = 114e-6
	lightSpeed_meters_per_sec = 3e8
	carrierFreq = 77e9


class harmonicClutterRmvCfg():
	L_threshold = 5


class cfarCfgRange(config):
	cfarType = 'CFAR_CA_SO' #'CFAR_CA', 'CFAR_CA_SO', 'CFAR_CA_GO', 'CFAR_OS', 'CFAR_CA_DC', 'CFAR_CA_LW'   (CFAR_CA_SO)
	OS_prun = 1/4
	thresholdScale = 15 / 6 # 6으로 나눈건 20log10(power)에서 log2(power)로 단위 변환
	guardLen = 4
	noiseLen = 12
	pruningType = "pruneToPeaksOrNeighbourOfPeaks" # "pruneToPeaks", "pruneToPeaksOrNeighbourOfPeaks", "pruneToPeaksOr2NeighbourOfPeaks","None"		(pruneToPeaksOrNeighbourOfPeaks)
	rangePrunning = True # range가 0 이하인건 계산하지 않아
	dbWrap = False


class cfarCfgDoppler:
	cfarType = 'CFAR_CA_SO'					# (CFAR_CA_SO)
	maxNumDet = 2			# (3)
	OS_prun = 1/4
	thresholdScale = 15 / 6 # 6으로 나눈건 20log10(power)에서 log2(power)로 단위 변환
	guardLen = 4
	noiseLen = 8
	pruningType = "pruneToPeaks" # "pruneToPeaks", "pruneToPeaksOrNeighbourOfPeaks", "pruneToPeaksOr2NeighbourOfPeaks","None"		(pruneToPeaks)
	dbWrap = True


class enhMaxVelCfg(config, cfarCfgDoppler):
	thresh = 2.0								#6/6 #3dB (1) --> 1.2 --> 1.6 --> 2.0
	threshMultiplier = 1.3						# (2) --> 1.4 --> 1.3
	MAX_VEL_ENH_NUM_NYQUIST = 2
	N_HYPOTHESIS = ((2 * (MAX_VEL_ENH_NUM_NYQUIST - 1)) + 1)
	MAX_VEL_IMPROVEMENT_NUM_SPREAD = 4			# (3) --> 4


class resolution:
	rangeResolution = config.SampleRate/config.numADCSamples/config.slope_Hz_per_sec*config.lightSpeed_meters_per_sec/2
	dopplerResolution = config.lightSpeed_meters_per_sec/config.n_chirps/config.carrierFreq/2/config.period_chirp


class aoaCfg:
	procFFT = True

	if CONST.TDM_MIMO:
		numAngleBins = 64
		multiPeakDet = True
	else:
		numAngleBins = 32
		multiPeakDet = False
	# numAngleBins = 180
	if config.numTx != 1:
		Mode = 'MIMO'
	else:
		Mode = 'SIMO'

	numAziChannel = config.numRx * config.numTx
	# numAziChannel = 4
	# multiPeakScale = 0.92 #0.85^(1/2) 러 해야지 스케일이 맞아 demo에서는 스펙트럼스케일이  abs^2이거든
	numTarget = 3
	multiPeakScale = 0.7
	maxAngle = 65*np.pi/180							# (55) --> 65

	SIN_55_DEGREES = np.sin(maxAngle)
	TRK_SIN_AZIM_THRESH = 1/256
	if config.numTx == 3:
		AWR1843 = True
	else:
		AWR1843 = False
	if CONST.TDM_MIMO:
		# rxChComp = np.array([18628 + 25894j, 7595 + 26175j, 7991 + 18836j, -3149 + 32616j, 4779 + 31839j, -4725 + 26108j, -1825 + 19973j, -17789 + 27174j], dtype = np.complex64)
		# rxChComp = np.array([-28448 -15261j, -17811 -17873j, -18320 -12844j, -18364 -27139j, -23803 -19823j, -14891 -19720j, -16290 -14483j, -14459 -27883j], dtype = np.complex64)
		# rxChComp = np.flip(rxChComp)
		# print(rxChComp)
		rxChComp = np.array([1, 1, 1, 1, 1, 1, 1, 1], dtype=np.complex64)
	else:

		rxChComp = np.array([3441 + 29564j, -5803 + 32250j, -3245 + 27766j, -5186 + 28091j], dtype=np.complex64)
	# for idx,val in enumerate(rxChComp):
		# rxChComp[idx] = val/abs(val) * abs(rxChComp[np.argmin(abs(rxChComp))])
		# rxChComp[idx] = val / abs(val) * abs(rxChComp[np.argmin(abs(rxChComp))])
	# rxChComp = np.array([-13206 -29989, -1559 -28132j, -2090 -24992j, -3088 -31961j], dtype=np.complex64)
	res_compensation_Type = "CAPON" #"MUSIC", "BARTLETT" or "CAPON". 셋중 하나.
	if CONST.AEB:

		azimuthSNRThresh = 7
	else:
		azimuthSNRThresh = 7

	#ILS_OMP Parmater
	MaxInteration = 10
	RayleighLimit = 30
	deltaTheta = 1 / 180 * np.pi  # unit is radian
	maxAngleH = 90 * np.pi / 180


class aoaCfgUSRR:
	if configUSRR.numTx != 1 :
		Mode = 'MIMO'
	else:
		Mode = 'SIMO'
	deltaTheta = 1 / 180 * np.pi #unit is radian
	numAziChannel = 4
	multiPeakScale = 0.92 #0.85^(1/2) 러 해야지 스케일이 맞아 demo에서는 스펙트럼스케일이  abs^2이거든
	maxAngle = 65 * np.pi / 180						# (55)
	SIN_55_DEGREES = np.sin(maxAngle)
	TRK_SIN_AZIM_THRESH = 1 / 256
	if configUSRR.numTx == 3:
		AWR1843 = True
	else:
		AWR1843 = False


class clusterCfgUSRR:
	maxCluster = 24
	minPointsInCluster = 3
	epsilon = 0.5
	epsilon2 = epsilon * epsilon
	weight = 1.3
	vFactor = 3.0
	dBScanNeighbourLim = 7


class clusterCfgMRR(config):
	isGuardRail = False
	maxCluster = 50				# (30)
	if CONST.CLUSTER_MIN_POINT_MODIFY:
		minPointsInCluster = 2					# (2)
	else:
		minPointsInCluster = 1
	dBScanNeighbourLim = 4

	ellipsoidA = 0.5				# X축: (2.0) --> 3.0
	ellipsoidB = 0.5				# Y축: (1.0) --> 1.5
	ellipsoidC = 1.0				# 속도: (3.0)


class clusterCfgSEW(config):
	isGuardRail = False
	maxCluster = 50				# (30)
	if CONST.CLUSTER_MIN_POINT_MODIFY:
		minPointsInCluster = 2					# (2)
	else:
		minPointsInCluster = 1
	dBScanNeighbourLim = 4

	ellipsoidA = 0.5				# X축: (2.0)
	ellipsoidB = 0.5				# Y축: (2.0)
	ellipsoidC = 1.0				# 속도: (4.0)


class clusterCfgRCCW(config):
	isGuardRail = False
	maxCluster = 50				# (30)
	if CONST.CLUSTER_MIN_POINT_MODIFY:
		minPointsInCluster = 2					# (2)
	else:
		minPointsInCluster = 1
	dBScanNeighbourLim = 4

	ellipsoidA =  0.5				# X축: (3.0)
	ellipsoidB =  0.5				# Y축: (3.0)
	ellipsoidC =  1.0				# 속도: (3.0)


class clusterCfgGuardRail(config):
	isGuardRail = True
	maxCluster = 50
	minPointsInCluster = 1

	ellipsoidA =  0.5				# X축: (4.0)
	ellipsoidB =  0.5				# Y축: (0.5)
	ellipsoidC =  1.0				# 속도: (100.0)


class rangeBasedPruningCfg():
	if cfarCfgRange.pruningType == "pruneToPeaksOrNeighbourOfPeaks" or cfarCfgRange.pruningType == "pruneToPeaksOr2NeighbourOfPeaks":
		minRange = 1.0
	else:
		minRange = 0.5

	maxRange = resolution.rangeResolution * config.numADCSamples * 0.9
	# print("maxRange : ", maxRange)
	# maxRange = 9000000
	MRR_MAX_OBJ_OUT = 150		# (200)
#	SNRThresh = [[5, 15/6], [10, 24/6], [55, 22/6], [maxRange, cfarCfgRange.thresholdScale]] #[[meter, ThersholdLevelIndB]] meter 이하 타겟은 해당 ThersholdLevelIndB 기준으로 자름.
	SNRThresh = [[5, 20 / 6], [10, 20 / 6], [55, 17 / 6], [maxRange, cfarCfgRange.thresholdScale]]  # [[meter, ThersholdLevelIndB]] meter 이하 타겟은 해당 ThersholdLevelIndB 기준으로 자름.
	#10log10 에서 log2 로 바꾸기 위해 6으로 나눔
	peakValThresh = [[10, 4352/256], [maxRange, 0]] #[[meter, peakValIndB(log2) 256으로 나눈건 spectrum이 Q8 Format]] python에서 board보다  peakVal 1dB(log10)정도 작음
#	peakValThresh = [[10, 4096/256], [maxRange, 0]] #[[meter, peakValIndB(log2) 256으로 나눈건 spectrum이 Q8 Format]] python에서 board보다  peakVal 1dB(log10)정도 작음
	# peakValThresh = [[1,4250/256],[maxRange,0]] #[[meter, peakValIndB(log2) 256으로 나눈건 spectrum이 Q8 Format]] for tracking debug


class trackingCfg(config, rangeBasedPruningCfg):
	max_detect_range_x = 40								# HSLee 추가 2022.01.21
	maxTracker = 50					# (30) --> 50

	associateGamma = 16.5 	# 정확도 90%				# (16.5)
	rangeAssocThresh = 0.5  # unit m					# (3.0)
	velAssocThresh = 1.0  # unit m/s					# (1.5)
	xAssocThresh = 0.5									# (3.0)
	yAssocThresh = 0.5  								# (2.0)
	azimAssocThresh = np.sin(20 * np.pi / 180)			# (20)

#	distAssocThreshSq = 1.65  	# unit m^2, original Value
	td = MRRchirpCfg.td 		# unit sec, 15frame

	HIGH_SNR_RVAR_THRESH = 6
	TRK_SIN_AZIM_THRESH = 1 / 256  # @brief We discard objects with poor azimuth SNR from the tracking procedure. */
	maxAge = 10
	maxTick = 40
	minTick = 0
	thresholdTick = 18			# (18)
	transitionProb = np.array([[0.99, 0.01], [0.01, 0.99]], dtype=np.float)

	CV_xVariance = 1.0			# (0.1) --> (1.0)
	CV_yVariance = 4.0			# (4.0) --> (4.0)
	CA_xVariance = 2.0			# (2.0) --> (2.0)
	CA_yVariance = 1.0			# (0.1) --> (1.0)

	distanceLimit = 1.42 		# 정확도 30% (No Use)
	multiplier = 2.5			# (No Use)


class trackingCfgSEW(config, rangeBasedPruningCfg):
	max_detect_range_x = 40								# HSLee 추가 2022.01.21
	maxTracker = 50					# (30) --> 50

	associateGamma = 16.5 	# 정확도 90%				# (16.5)
	rangeAssocThresh = 4.0  # unit m					# (4.0)
	velAssocThresh = 2.5  # unit m/s					# (2.5)
	xAssocThresh = 3.0									# (3.0)
	yAssocThresh = 3.0  								# (3.0)
	azimAssocThresh = np.sin(20 * np.pi / 180)			# (20)

#	distAssocThreshSq = 1.65  	# unit m^2, original Value
	td = MRRchirpCfg.td 		# unit sec, 15frame

	HIGH_SNR_RVAR_THRESH = 6
	TRK_SIN_AZIM_THRESH = 1 / 256  # @brief We discard objects with poor azimuth SNR from the tracking procedure. */
	maxAge = 10
	maxTick = 40
	minTick = 0
	thresholdTick = 18			# (18)
	transitionProb = np.array([[0.99, 0.01], [0.01, 0.99]], dtype=np.float)

	CV_xVariance = 1.0			# (0.1) --> (1.0)
	CV_yVariance = 4.0			# (4.0) --> (4.0)
	CA_xVariance = 2.0			# (2.0) --> (2.0)
	CA_yVariance = 1.0			# (0.1) --> (1.0)

	distanceLimit = 1.42 		# 정확도 30% (No Use)
	multiplier = 2.5			# (No Use)


class trackingCfgRCCW(config, rangeBasedPruningCfg):
	max_detect_range_x = 40								# HSLee 추가 2022.01.21
	maxTracker = 50					# (30) --> 50

	associateGamma = 16.5 	# 정확도 90%				# (16.5)
	rangeAssocThresh = 4.0  # unit m					# (4.0)
	velAssocThresh = 3.0   # unit m/s					# (3.0)
	xAssocThresh = 4.0									# (4.0)
	yAssocThresh = 4.0  								# (4.0)
	azimAssocThresh = np.sin(20 * np.pi / 180)			# (20)

#	distAssocThreshSq = 1.65  	# unit m^2, original Value
	td = MRRchirpCfg.td 		# unit sec, 15frame

	HIGH_SNR_RVAR_THRESH = 6
	TRK_SIN_AZIM_THRESH = 1 / 256  # @brief We discard objects with poor azimuth SNR from the tracking procedure. */
	maxAge = 10
	maxTick = 40
	minTick = 0
	thresholdTick = 18			# (18) --> 20
	transitionProb = np.array([[0.99, 0.01], [0.01, 0.99]], dtype=np.float)

	CV_xVariance = 1.0			# (0.1) --> (1.0)
	CV_yVariance = 4.0			# (4.0) --> (4.0)
	CA_xVariance = 2.0			# (2.0) --> (2.0)
	CA_yVariance = 1.0			# (0.1) --> (1.0)

	distanceLimit = 1.42 		# 정확도 30% (No Use)
	multiplier = 2.5			# (No Use)


class ROIpruneCfg:
	xScale = 10
	roadHalfSize = 7
	curvature = 400
	# roadHalfSize = 3
	# curvature = 1000


class guardRailDetCfg:
	minClusterNum = 5
	ratioThr = 20
	degree = 2  # must be 2 or 3
	process = "TLS"