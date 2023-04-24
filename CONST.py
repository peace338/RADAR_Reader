SIMULATOR                       = False

DO_RX_COMPENSATION              = False #True시 지정해준 상수로 Compensation
DO_ANGLE_COMPENSATION_HIGH_RES  = False
REMOVE_VARIABLE_VFACTOR         = True
ADD_ANGLE_VEL                   = False #EKF measurement Vector 3D->4D
ADD_ANGLE_VEL_SIMPLE_VAR        = False
Qmat_piecewiseConstantAccel     = True
SIZE_OSCILLATION                = True
Mahalnobis_Association          = True
Hmat_doppler2Ydot               = False			# (False)
ROIPruning                      = False
MeasCovVec_CONST                = False
staticClutterRmv                = False
predictedCalMod                 = False
maxProbAssoc                    = False
SPEED_COMP_BY_ANGLE             = True			# (True)
Calibration                     = False
ADAPTIVE_CFAR_THRESH            = True
R_modify                        = False
XVEL_penalty                    = False
R_modify_20210326               = True
TDM_MIMO                        = True
TMP_PROTOCOL                    = False  #true이면 RADAR v 1.6.87 이전 데이터에 대한 protocol로 read
SLOW_CFAR                       = False
doIsInBoundaryProcss            = True
DOPPLER_INTERPOLATION           = True
USING_TTC						= False		# HSLee 추가 2022.06.07 : TTC 산출

CRT_OVERLAP                     = False
CRT_OVERLAP_append              = False
choose_by_CRT                   = False
choose_by_CRT_slow              = False

DC_NOISE_RM_DOPPLER_FFT         = False

ILS_OMP                         = False
RISR                            = False
ILS_OMP_MMSE                    = False
FOCUSS                          = False

Qmat_modify                     = True
readDCA1000Python               = True
clusteringBeforeAOA             = False
proportionRange_CL_DA           = False
hungarian_Alg                   = False
IMM                             = True
IMM_CA                          = True
IMM_angle_vel_cal               = True
IMM_ACT                         = False
IMM_omega_0                     = False
IMM_transMat_Mod                = True
STATUS_FLAG                     = True
STATUS_FLAG_ASC                 = True
PRUNE_BY_FLAG                   = False
ADATIVE_Q                       = False			# (True)

RCS_slope_cal                   = False
onlyMRR                         = True
TRACKING_ALL_DEBUG              = True			# True : All

MEAS_COV_VEC_SCALAR_MODIFY      = False
MEAS_COV_VEC_SCALAR             = 2
MODIFIER_MODIFY                 = True

CLUSTER_MIN_POINT_MODIFY        = False			# (True)
TRACKING_INPUT_MIN_POINT_MODIFY = True
ellipsoidProcess                = True
xPruning                        = True # 검출된 타겟이 좌우 4m외에 있을 시 pruning
azimSNRBasedPrunning            = False #Angle SNR 10 dB 미만 시 제거
DEFAULT_SIZE_MODIFY             = True
ENH_MAX_VEL_MODIFY              = True

AZIMUTH_BASED_PRUNING           = False #tracking 에서 각도로 짤라주는거랑 부분적으로 겹쳐서 별로 의미가 없다.

DEBUG_DA                        = True # 모든

CAL_ANGLE_HIGH_RESOLUTION       = False # music capon 결과 비교용

angleMinPlot                    = False # 미완
xyMinPlot                       = False # 미완
plotMinTrackerDEBUG             = False # 미완
plotTrackerOption               = False # 미완
harmonicClutterRmvDEBUG         = False # 미완

RCSDEBUG                        = False     # True시 작동. RCS_DEBUG_ID_NUMBER에 정해준 ID의 RCS 결과값 출력
ekfDEBUG                        = False     # 추적 과정시 사용되는 Kalman Gain Matrix 등 출력
trackingLogDEBUG                = True      # (True) RCS_DEBUG_ID_NUMBER에 정해준 ID의 추적되는 타겟의 tracking Input 저장, variance 구하기 위해


slowRangeFFTPlot                = False # SlowChirp FFT 결과도 출력. cfarDEBUG = True이여야 동작
slowRangeFFTPlot_dopplerIdx     = 58
pruneTrackingInputDEBUG         = False
TRACKING_AGE_FUNCTION           = True			#(True)

calNoiseLevel                   = False # noiseLevel 계산
calNoiseLevelDEBUG              = False
calNoiseLevelLog                = False



QFormatFFT                                  = False # 작성중
clusteringDEBUG                             = False
rangeBasedPruningDEBUG                      = False
peakPruningDEBUG                            = False
pruningKLargestValDEBUG                     = False
trackingInputMODIFY                         = True		#######
trackingAssociationEllipsoid                = False
isTargetWithinDataAssociationThreshDEBUG    = False
get_mahalanobis_dist_DEBUG                  = False
transCoordDEBUG                             = False


logScaleTISDK               = True
calMatrixTISDK              = False
calMatrixTISDKDEBUG         = False

quadraticInterpLogMODIFY    = True # peakPruning 해제하면 peak 아닌 애들을 interpolation 해서 오류가 생겨서 수정.

pltClusterOutput            = False # size plot

pltDetTarget                = True # Detect 결과 표시
pltTrackerSize              = True
printDetTarget              = True # Detect 결과 print, pltDetTarget = True 일때 동작
plotClusterDEBUG            = True # Detect결과에 같은 cluster끼리 같은 색으로 표기, pltDetTarget가 True일 때 작동
pltMahalanobisContour       = True
pltTrackerDet               = True
plotTrackerDEBUG                = True      # RCS_DEBUG_ID_NUMBER에 정해준 ID Tracking 결과 추적
pltDopplerAzimuth               = False
pltGuardRailFromreadTargetRaw   = True
pltGuardrailTrk                 = False
OnlySaveHeatmap                 = False # heatMap 까지만 processing 하고 heatMap 저장


#plot Size
# targetPlotSize =
HIGH_steeringVec_AOA            = True
#==========DEBUG================================================================================================
#===============================================================================================================
printADCdata                    = False
pltTargetIdxPlotSpec            = False          #targetIdxPlotSpec에 해당하는 타겟은 +로 표시
cfarDEBUG                       = False                  #CFAR 결과 plot (spectrum plot) 아래 targetIdxPlotSpec 의 값을 변경하여 몇 번째 타겟을 플롯 할 것인지 결정. CONST.heatMapPlot값이 True여야지 작동
heatMapPlot                     = False         #heatMap 출력
AOADebug                        = False            #AOA spectrum 출력
FFTspectrumDEBUG                = False                 #각 채널별 SPECTRUM 출력
enhancedMaxVelDEBUG             = False         #enhance Max Vel 알고리즘 과정 중  spectrum 출력, targetIdxPlotSpec에 해당하는 타겟
#===============================================================================================================
#===============================================================================================================

RCS_DEBUG_ID_NUMBER                 = 5   #tracking debug, ekf debug, RCS debug
targetIdxPlotSpec                   = 3
targetIdxPlotSpec_range_offset      = 0
targetIdxPlotSpec_doppler_offset    = 0

#dataPath = "D://HSLee/Data/RawData/R.PAS/20210908/Cross_Adult_1L/Transform/"
#dataPath = "D://HSLee/Data/RawData/R.PAS/20210908/Cross_SantaFe_1L/0908_160630/Transform/"
#dataPath = "D://HSLee/Data/RawData/R.PAS/20210908/maxRange_SantaFe_2.3T/Transform/"
#dataPath = "D://HSLee/Data/RawData/R.PAS/20210915/AngleRes_(-1,15)_(1,15)/Transform/"
#dataPath = "D://HSLee/Data/RawData/R.PAS/20210915/AngleRes_(-1.5,15)_(1.5,15)/Transform/"
#dataPath = "D://HSLee/Data/RawData/R.PAS/20210915/RangeRes_(0.6,15)_(0,14.93)/Transform/"
#dataPath = "D://HSLee/Data/RawData/R.PAS/20210915/RangeRes_(0.6,15)_(0,14.95)/Transform/"
#dataPath = "D://HSLee/Data/RawData/R.PAS/20211026_Test/noRad_T23_-9deg_X/1026_134922/Transform/"

#dataPath = "//192.168.100.234/RADAR_Rawdata_Videodata_6F/Radar&Video_Data/RPAS/20220117-18/0118_080011/Transform/12/"
#dataPath = "D://HSLee/Data/RawData/R.PAS/20220308_EverLand/parallel_adult_walking_5m/0308_111654/Transform/"
#dataPath = "D://HSLee/Data/RawData/R.PAS/Temp/Transform/"
#dataPath = "D://HSLee/Data/RawData/R.PAS/20220510_JamSil/Afternoon_차량_장착/45deg_Wheel_Doppler/0510_135942/Transform/"
#dataPath = "D://HSLee/Data/RawData/R.PAS/20220510_JamSil/Afternoon_차량_장착/35deg_Wheel_Doppler/0510_142352/Transform/"
#dataPath = "D://HSLee/Data/RawData/R.PAS/20220404_N/"
#dataPath = "D://HSLee/Data/RawData/TEST/"
dataPath = "C://TEST_log/"

mrr_adc_data_name = "mrr_adc_data.bin"
targetRaw_name = "Target_Raw.bin"
targetRawFile_name = "Target_Raw_python.bin"
targetRawFileTXT_name = "Target_Raw_python.txt"

trackingtargetFile_name = "Target_Trk_python.bin"

startFrame = 1265		#3255 + 360	#1160(1219)
endFrame = 1700			#3370 + 360	#1161(1220)
if onlyMRR:
	numProcFrame = (endFrame - startFrame)
else:
	numProcFrame = (endFrame - startFrame) /2
#numProcFrame = 1
xPltSize = 32 # (-x PltSize, xPltSize) unit : m	(10)
xPltStep = 2 # unit : m							(2)
yPltSize = 60  # (0, yPltSize) unit : m			(60)
yPltStep = 5 # unit : m							(5)
figureSizeX = 8
figureSizeY = 16

#parameter change

AEB = False

if AEB:
	CLUSTER_MIN_POINT_MODIFY = True
	ROIPruning = False

reverseAntenna = False

if TDM_MIMO:

	reverseAntenna = False
	xPruning = True

APOLLO = False
if APOLLO:
	TDM_MIMO = False
	TMP_PROTOCOL = True

writeOutDetObj = True
writeOutTrk = True

pltMap = True		# False
if pltMap == False:

	pltDetTarget            = False
	pltTrackerSize          = False
	printDetTarget          = False
	plotClusterDEBUG        = False
	pltMahalanobisContour   = False
	pltTrackerDet           = False
	plotTrackerDEBUG        = False