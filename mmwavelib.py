import numpy as np
import matplotlib.pyplot as plt
import dataStructure as DT
import copy
import CONST
import os
import DOA
import IMM
import configManager_MRR_DEMO as cfg
import sys
import math

# from numpy.linalg import inv
N_STATES = 4
if CONST.ADD_ANGLE_VEL:
	N_MEAS = 4
else:
	N_MEAS = 3

N_UNIQ_ELEM_IN_SYM_COVMAT = 10
N_MEASUREMENTS =3

iX = 0
iY = iX + 1
iXd = iY + 1
iYd = iXd + 1


iXX = 0
iXY = (iXX + 1)
iXXd = (iXY + 1)
iXYd = (iXXd + 1)

iYY = (iXYd + 1)
iYXd = (iYY + 1)
iYYd = (iYXd + 1)

iXdXd = (iYYd + 1)
iXdYd = (iXdXd + 1)
iYdYd = (iXdYd + 1)

iRR = 0
iRRd = (iRR + 1)
iRAz = (iRRd + 1)

iRdRd = (iRAz + 1)
iRdAz = (iRdRd + 1)

iAzAz = (iRdAz + 1)

	# print(type(logDtype))

def InitQvec(td):

	if CONST.Qmat_piecewiseConstantAccel:

		MAX_ACCEL_X_M_P_SECSQ = 1
		MAX_ACCEL_Y_M_P_SECSQ = 5
		FRAME_PERIODICITY_SEC = td

		at_x = (MAX_ACCEL_X_M_P_SECSQ * FRAME_PERIODICITY_SEC);
		at_y = (MAX_ACCEL_Y_M_P_SECSQ * FRAME_PERIODICITY_SEC);
		halfatsq_x = 0.5 * (MAX_ACCEL_X_M_P_SECSQ * FRAME_PERIODICITY_SEC * FRAME_PERIODICITY_SEC);
		halfatsq_y = 0.5 * (MAX_ACCEL_Y_M_P_SECSQ * FRAME_PERIODICITY_SEC * FRAME_PERIODICITY_SEC);

		Qvec = np.zeros((3, 4), dtype = np.float)

		Qvec[0][0] = halfatsq_x
		Qvec[0][1] = halfatsq_y
		Qvec[0][2] = at_x
		Qvec[0][3] = at_y

		Qvec[1][0] = halfatsq_x
		Qvec[1][1] = halfatsq_y
		Qvec[1][2] = at_x
		Qvec[1][3] = at_y

		Qvec[2][0] = halfatsq_x
		Qvec[2][1] = halfatsq_y
		Qvec[2][2] = at_x
		Qvec[2][3] = at_y



	else:

		#============================Init Qvec==========================
		MAX_ACCEL_X_M_P_SECSQ = 5
		MAX_ACCEL_Y_M_P_SECSQ = 12
		# MAX_ACCEL_X_M_P_SECSQ = 5
		# MAX_ACCEL_Y_M_P_SECSQ = 12
		FRAME_PERIODICITY_SEC = td

		at_x = (MAX_ACCEL_X_M_P_SECSQ * FRAME_PERIODICITY_SEC);
		at_y = (MAX_ACCEL_Y_M_P_SECSQ * FRAME_PERIODICITY_SEC);
		halfatsq_x = 0.5 * (MAX_ACCEL_X_M_P_SECSQ * FRAME_PERIODICITY_SEC * FRAME_PERIODICITY_SEC);
		halfatsq_y = 0.5 * (MAX_ACCEL_Y_M_P_SECSQ * FRAME_PERIODICITY_SEC * FRAME_PERIODICITY_SEC);

		Qvec = np.zeros((3,4), dtype = np.float)

		Qvec[0][0] = 8 * (halfatsq_x * halfatsq_x)
		Qvec[0][1] = 8 * (halfatsq_y * halfatsq_y)
		Qvec[0][2] = 8 * (at_x * at_x)
		Qvec[0][3] = 8 * (at_y * at_y)

		Qvec[1][0] = 4 * (halfatsq_x * halfatsq_x)
		Qvec[1][1] = 4 * (halfatsq_y * halfatsq_y)
		Qvec[1][2] = 4 * (at_x * at_x)
		Qvec[1][3] = 4 * (at_y * at_y)

		Qvec[2][0] = 1 * (halfatsq_x * halfatsq_x)
		Qvec[2][1] = 1 * (halfatsq_y * halfatsq_y)
		Qvec[2][2] = 1 * (at_x * at_x)
		Qvec[2][3] = 1 * (at_y * at_y)

	# if CONST.Qvec_MODIFY:
	# 	Qvec[:][2] = Qvec[:][2] * 5
	# 	Qvec[:][0] = Qvec[:][2] * 5
	return Qvec
def xPruning(cfarOut3DList, scale):
	for obj in cfarOut3DList[:]:
		tmp1 = obj.x
		tmp2 = obj.speed

		if (abs(tmp1) > scale):
#		if (abs(tmp1) > scale) or (tmp2 >= 0.0):			# doppler >= 0 인 표적 제거
			cfarOut3DList.remove(obj)
			del(obj)

def ROIPruning(cfarOut3DList, scale, halfRoadScale,curvature):
	for obj in cfarOut3DList[:]:
		tmp = obj.x
		condition = abs(obj.x) < scale and \
		            ((obj.x - (curvature + halfRoadScale)) ** 2 + (obj.y) ** 2 > (curvature) ** 2 and \
		             (obj.x + (curvature + halfRoadScale)) ** 2 + (obj.y) ** 2 > (curvature) ** 2)

		if not(condition) :
			cfarOut3DList.remove(obj)
			del(obj)

def staticObjPruning(cfarOut3DList, vehicleSpeed):
	vehicleSpeedFlt = vehicleSpeed / 3.6
	for obj in cfarOut3DList[:]:
		condition = False
		if abs(obj.speed + vehicleSpeedFlt) <= 3.3:
			obj.statusFlag = 0
			if CONST.PRUNE_BY_FLAG:
				condition = True
		elif (obj.speed + vehicleSpeedFlt) < -3.3:
			obj.statusFlag = 2
			if CONST.PRUNE_BY_FLAG:
				condition = True
		elif (obj.speed + vehicleSpeedFlt) > 3.3:
			obj.statusFlag = 1

		if condition:
			cfarOut3DList.remove(obj)
			del(obj)

def azimuthBasedPruning(cfarOut3DList, aoaCfg):
	for obj in cfarOut3DList[:]:
		if not(abs(obj.sinAzim) < aoaCfg.SIN_55_DEGREES):
			cfarOut3DList.remove(obj)
			del (obj)

def findDopplerBoundary(config, velFlt, windowSize):
	if velFlt < 0:
		VelIndxTmp = -1 * velFlt

		numMult = VelIndxTmp // config.n_chirps
		velFlt += (config.n_chirps * (numMult + 1))
		VelIdx = int(velFlt)
	elif (velFlt >= config.n_chirps):
		VelIndxTmp = velFlt

		numMult = VelIndxTmp // config.n_chirps
		velFlt -= (config.n_chirps * numMult)
		VelIdx = int(velFlt)
	else:
		VelIdx = int(velFlt)

	upperBound = VelIdx + windowSize

	if (upperBound >= config.n_chirps):
		upperBound -= config.n_chirps
	lowerBound = VelIdx - windowSize
	if (lowerBound < 0):
		lowerBound += config.n_chirps

	return upperBound, lowerBound

def isInBoundary(upperBound, lowerBound, idx):
	tmp = False
	if upperBound > lowerBound:
		if ((idx < upperBound) and (idx > lowerBound)):
			tmp = True
	else:
		if ((idx < upperBound) or (idx > lowerBound)):
			tmp = True
	return tmp
	# return True


def findUsefulDopplerIndx(config, egoVehicleSpeed):
	egoVehicleSpeedFlt = -1 * egoVehicleSpeed / 3.6
	maxUnambiguousVel = config.numTx * config.lightSpeed_meters_per_sec / config.carrierFreq / 4 / config.period_chirp
	# print("asfsfs",maxUnambiguousVel)
	velResolutionSlowChirp = config.lightSpeed_meters_per_sec / config.carrierFreq / 2 / config.period_chirp_slow / config.n_chirps
	absVelIdxFlt1 = ((egoVehicleSpeedFlt + maxUnambiguousVel) / velResolutionSlowChirp) + config.n_chirps / 2
	absVelIdxFlt2 = ((egoVehicleSpeedFlt + maxUnambiguousVel * 2) / velResolutionSlowChirp) + config.n_chirps / 2

	# print("agag,", absVelIdxFlt1, absVelIdxFlt2)
	upperBound1, lowerBound1 = findDopplerBoundary(config, absVelIdxFlt1, 6)
	upperBound2, lowerBound2 = findDopplerBoundary(config, absVelIdxFlt2, 6)

	return upperBound1, lowerBound1, upperBound2, lowerBound2

def findStaticDopplerIndx(config, egoVehicleSpeed):
	egoVehicleSpeed = -1 * egoVehicleSpeed/3.6
	velResolutionFastChirp = config.lightSpeed_meters_per_sec / config.carrierFreq / 2 / config.period_chirp / config.n_chirps
	egoVelIndxFlt = (egoVehicleSpeed / velResolutionFastChirp) + config.n_chirps / 2

	if egoVelIndxFlt < 0:
		egoVelIndxTmp = -1 * egoVelIndxFlt

		numMult = egoVelIndxTmp // config.n_chirps
		egoVelIndxFlt += (config.n_chirps * (numMult + 1))
		egoVelIndx = int(egoVelIndxFlt)
	elif (egoVelIndxFlt >= config.n_chirps):
		egoVelIndxTmp = egoVelIndxFlt

		numMult = egoVelIndxTmp // config.n_chirps
		egoVelIndxFlt -= (config.n_chirps * numMult)
		egoVelIndx = int(egoVelIndxFlt)
	else:
		egoVelIndx = int(egoVelIndxFlt)

	upperBound = egoVelIndx + 5

	if (upperBound >= config.n_chirps):
		upperBound -= config.n_chirps
	lowerBound = egoVelIndx - 5
	if (lowerBound < 0):
		lowerBound += config.n_chirps

	return upperBound, lowerBound, egoVelIndx

def rangeBasedPruning(cfarOut2DList, rangeBasedPruningCfg, config, domain, egoVehicleSpeed):
	upperBound, lowerBound, egoVelIndx = findStaticDopplerIndx(config, egoVehicleSpeed)

	for obj in cfarOut2DList[:]:

		objRange = domain.rangeDomain[obj.rangeIdx]

		SNRThreshIdx = 0
		peakValThreshIdx = 0

		if objRange >= rangeBasedPruningCfg.minRange and objRange <= rangeBasedPruningCfg.SNRThresh[0][0]:
			SNRThreshIdx = 0

		elif objRange <= rangeBasedPruningCfg.SNRThresh[1][0] and objRange > rangeBasedPruningCfg.SNRThresh[0][0]:
			SNRThreshIdx = 1

		elif objRange <= rangeBasedPruningCfg.SNRThresh[2][0] and objRange > rangeBasedPruningCfg.SNRThresh[1][0]:
			SNRThreshIdx = 2
		elif objRange <= rangeBasedPruningCfg.maxRange and objRange > rangeBasedPruningCfg.SNRThresh[2][0]:
			SNRThreshIdx = 3
		else:
			if CONST.rangeBasedPruningDEBUG:

				print("rangeBasedPruning removed the target range : %f, SNR: %f, Val: %f"%(objRange, obj.rangeSNR*6, obj.rangeVal))
				print("because the target is out of min and max range")
			cfarOut2DList.remove(obj)
			del(obj)
			continue	

		if objRange >= rangeBasedPruningCfg.minRange and objRange <= rangeBasedPruningCfg.peakValThresh[0][0]:
			peakValThreshIdx = 0

		elif objRange <= rangeBasedPruningCfg.peakValThresh[1][0]:
			peakValThreshIdx = 1

		SNRThreshold = rangeBasedPruningCfg.SNRThresh[SNRThreshIdx][1]

		if (((SNRThreshIdx == 1) or (SNRThreshIdx == 2)) and CONST.ADAPTIVE_CFAR_THRESH):
			if isInBoundary(upperBound, lowerBound, obj.dopplerIdx):
				SNRThreshold -= 4 / 6					# (6 / 6) --> 4 / 6

		if (obj.rangeSNR > SNRThreshold and obj.rangeVal > rangeBasedPruningCfg.peakValThresh[peakValThreshIdx][1]) or \
			obj.rangeVal > (5120 / 256):

			pass

		else:
			if CONST.rangeBasedPruningDEBUG:
				# print(obj.rangeIdx)
				print("rangeBasedPruning removed the target : range : %f, SNR: %f, Val: %f"%(objRange, obj.rangeSNR*6, obj.rangeVal))
				print("because it has lawer value than SNR : %.2f or peak value %.2f."\
				      %(rangeBasedPruningCfg.SNRThresh[SNRThreshIdx][1]*6, rangeBasedPruningCfg.peakValThresh[peakValThreshIdx][1]))
			cfarOut2DList.remove(obj)
			del(obj)
			continue

	# for i in cfarOut2DList[:]:
	# 	print(domain.rangeDomain[i.rangeIdx])

def genHannRecWind(samples):
	winLen = int(samples/8)
	halfWinLen = int(samples/16)
	tmpWin = np.ones(samples, dtype = np.float)
	tmphanningWind = np.hanning(winLen)
	tmpWin[:halfWinLen] = tmphanningWind[:halfWinLen]
	tmpWin[-halfWinLen:] = tmphanningWind[halfWinLen:]
	tmpWin = tmpWin
	return tmpWin

def staticClutterRmv(data, config):
	for rangeIdx in range(config.numADCSamples):
		for antIdx in range(config.numRx * config.numTx):
			data.FFTData[rangeIdx, :, antIdx] = data.FFTData[rangeIdx, :, antIdx] - np.average(data.FFTData[rangeIdx, :, antIdx])


def RangeFFT(data, config, winType = "HANNING_RECT"):

	# plt.plot(data.adcData[:,0,1].real)
	# plt.plot(data.adcData[:, 0, 1].imag)
	# plt.show()
	if winType == "HANNING":
		win = np.hanning(config.numADCSamples)
	elif winType == "HANNING_RECT":
		win = genHannRecWind(config.numADCSamples)
	elif winType == "HAMMING":
		win = np.hamming(config.numADCSamples)
	else:
		win = 1
	for chirpIdx in range(config.n_chirps):
		for antIdx in range(config.numRx*config.numTx):
			data.adcData[:,chirpIdx,antIdx] = win * data.adcData[:,chirpIdx,antIdx]
				# pass

	data.FFTData = np.fft.fft(data.adcData, axis = 0)
	# data.FFTData = np.fft.fftshift(data.FFTData , axes = (0,))
	# data.FFTData[int(config.numADCSamples/2):,:,:] = np.flip(data.FFTData[int(config.numADCSamples/2):,:,:] , 0)
# def
def harmonicClutterRmv(data, harmonicClutterRmvCfg, config, domain, DEBUG = False):
	tmpHarmonogram = np.fft.fft(data.FFTData, axis=0)
	tmpHarmonogram = np.fft.fftshift(tmpHarmonogram, axes=0)
	if DEBUG:

		heatMap_hm = avgHeatMap(tmpHarmonogram, config, log2 = False)
		# heatMap_hm = abs(tmpHarmonogram[:,:,0])
		fig, ax = plt.subplots()
		c = ax.pcolormesh(heatMap_hm)

		ax.set_title('HeatMap H(h,m)')
		fig.colorbar(c, ax = ax)
		plt.xlabel("time axis(idx)")
		plt.ylabel("freq axis(idx)")

		plt.show()

		plt.plot(heatMap_hm[:,30])
		plt.show()




def DopplerFFT(data, config, winType = "HAMMING" ): #"HAMMING"

	if winType == "HAMMING":
		window = np.hamming(config.n_chirps)
	elif winType == "HANNING":
		window = np.hanning(config.n_chirps)
	else:
		window = 1

	if CONST.DC_NOISE_RM_DOPPLER_FFT:
		for rangeIdx in range(17):
			for antIdx in range(config.numRx * config.numTx):
				data.FFTData[rangeIdx, :, antIdx] = data.FFTData[rangeIdx, :, antIdx] - np.average(data.FFTData[rangeIdx, :, antIdx])


	for rangeIdx in range(config.numADCSamples):
		for antIdx in range(config.numRx * config.numTx):
			data.FFTData[rangeIdx,:,antIdx] = window * data.FFTData[rangeIdx,:, antIdx]

	data.FFTData = np.fft.fft(data.FFTData, axis = 1)
	data.FFTData = np.fft.fftshift(data.FFTData , axes = (1,))


def RangeFFT_Slow(data, config, winType = "HANNING_RECT" ):

	if winType == "HANNING":
		win = np.hanning(config.numADCSamples)
	elif winType == "HANNING_RECT":
		win = genHannRecWind(config.numADCSamples)
	elif winType == "HAMMING":
		win = np.hamming(config.numADCSamples)
	else:
		win = 1
	for chirpIdx in range(config.n_chirps):
		for antIdx in range(config.numRx * config.numTx):
			data.adcData_Slow[:,chirpIdx,antIdx] = win * data.adcData_Slow[:,chirpIdx,antIdx]
				# pass

	data.FFTData_Slow = np.fft.fft(data.adcData_Slow, axis = 0)


def DopplerFFT_Slow(data, config, winType = "HAMMING"):
	if winType == "HAMMING":
		window = np.hamming(config.n_chirps)
	elif winType == "HANNING":
		window = np.hanning(config.n_chirps)
	else:
		window = 1

	if CONST.DC_NOISE_RM_DOPPLER_FFT:
		for rangeIdx in range(17):
			for antIdx in range(config.numRx * config.numTx):
				data.FFTData[rangeIdx, :, antIdx] = data.FFTData[rangeIdx, :, antIdx] - np.average(data.FFTData[rangeIdx, :, antIdx])

	for rangeIdx in range(config.numADCSamples):
		for antIdx in range(config.numRx*config.numTx):
			data.FFTData_Slow[rangeIdx,:,antIdx] = window * data.FFTData_Slow[rangeIdx,:, antIdx]

	data.FFTData_Slow = np.fft.fft(data.FFTData_Slow, axis = 1)
	data.FFTData_Slow = np.fft.fftshift(data.FFTData_Slow , axes = (1,))

def abs_TI_SDK_cmplx(complexInp):
	realValue = abs(complexInp.real)
	imagValue = abs(complexInp.imag)

	A = np.maximum(realValue, imagValue)
	B = np.minimum(realValue, imagValue) * 3/8

	return (A + B)

def calNoiseLv(FFTData, config, log2 = True):

	# avgFFTData = abs(FFTData)
	# avgFFTData = np.average(avgFFTData, axis=2)
	# avgFFTData = np.log2(avgFFTData)
#################orig########################
	if log2 == True:
		if CONST.logScaleTISDK:
			absData = abs_TI_SDK_cmplx(FFTData)
			avgFFTData = np.log2(absData)
		else:
			avgFFTData = np.log2(abs(FFTData))
		if CONST.QFormatFFT:
			avgFFTData = np.round(avgFFTData * 128)
			avgFFTData = avgFFTData.astype('int32')

	else:
		avgFFTData = abs(FFTData)

	avgFFTData = np.average(avgFFTData, axis=2)
	avgFFTData = np.average(avgFFTData, axis=1)
########################
	# if log2 == True:
	# 	avgFFTData =  np.log2(avgFFTData)

	return avgFFTData

def avgHeatMap(FFTData, config, log2 = True):

	# avgFFTData = abs(FFTData)
	# avgFFTData = np.average(avgFFTData, axis=2)
	# avgFFTData = np.log2(avgFFTData)
#################orig########################
	if log2 == True:
		if CONST.logScaleTISDK:
			absData = abs_TI_SDK_cmplx(FFTData)
			avgFFTData = np.log2(absData)
		else:
			avgFFTData = np.log2(abs(FFTData))
		if CONST.QFormatFFT:
			avgFFTData = np.round(avgFFTData * 128)
			avgFFTData = avgFFTData.astype('int32')

	else:
		avgFFTData = abs(FFTData)

	avgFFTData = np.average(avgFFTData, axis=2)
########################
	# if log2 == True:
	# 	avgFFTData =  np.log2(avgFFTData)

	return avgFFTData

def cfarWindowRange_dbWrap(idxCUT, cfarConfig, length):

	### Oridinal Code
#	sumRightIdx1 = idxCUT + cfarConfig.guardLen + 1
#	sumRightIdx2 = idxCUT + cfarConfig.guardLen + cfarConfig.noiseLen + 1
#	sumLeftIdx1  = idxCUT - cfarConfig.guardLen
#	sumLeftIdx2  = idxCUT - cfarConfig.guardLen - cfarConfig.noiseLen
	### Oridinal Code

	### HSLee 수정 2022.06.15
	sumRightIdx1 = idxCUT + cfarConfig.guardLen + 1
	sumRightIdx2 = idxCUT + cfarConfig.guardLen + cfarConfig.noiseLen
	sumLeftIdx1  = idxCUT - cfarConfig.guardLen - 1
	sumLeftIdx2  = idxCUT - cfarConfig.guardLen - cfarConfig.noiseLen
	### HSLee 수정 2022.06.15

	if sumRightIdx1 > length:
		sumRightIdx1 = sumRightIdx1 % length
	if sumRightIdx2 > length:
		sumRightIdx2 = sumRightIdx2 % length

	return sumRightIdx1, sumRightIdx2, sumLeftIdx1, sumLeftIdx2

def cfarWindowRange(idxCUT, cfarConfig, length):

	sumRightIdx1 = idxCUT + cfarConfig.guardLen + 1
	sumRightIdx2 = idxCUT + cfarConfig.guardLen + cfarConfig.noiseLen + 1
	sumLeftIdx1  = idxCUT - cfarConfig.guardLen
	sumLeftIdx2  = idxCUT - cfarConfig.guardLen - cfarConfig.noiseLen

	if sumRightIdx1 > length:
		sumRightIdx1 = length
	if sumRightIdx2 > length:
		sumRightIdx2 = length
	if sumLeftIdx1 < 0:
		sumLeftIdx1 = 0
	if sumLeftIdx2 < 0:
		sumLeftIdx2 = 0

	return sumRightIdx1, sumRightIdx2, sumLeftIdx1, sumLeftIdx2

def cfar_CA(inp, cfarConfig, DEBUG = True, onePointProcess = None):

	if onePointProcess == None:
		length = len(inp)
		idxCUTList = range(length)
	else:
		idxCUTList = [onePointProcess]
	cfarOutList = []
	length = len(inp)

	if DEBUG:
		thresholdLine = np.zeros(length, dtype = float)
		# noiseLine = np.zeros(length, dtype = float)

	for idxCUT in idxCUTList:

		sumRightIdx1, sumRightIdx2, sumLeftIdx1, sumLeftIdx2 = cfarWindowRange(idxCUT, cfarConfig, length)

		windowLen = (sumRightIdx2 - sumRightIdx1) + (sumLeftIdx1 - sumLeftIdx2)

		sumRight = np.sum(inp[sumRightIdx1 : sumRightIdx2])

		sumLeft = np.sum(inp[sumLeftIdx2 : sumLeftIdx1])

		sumNoise = (sumRight+sumLeft)/(windowLen)



		if DEBUG == True:
			thresholdLine[idxCUT] = sumNoise + cfarConfig.thresholdScale

		if inp[idxCUT] > sumNoise + cfarConfig.thresholdScale:

			tmpCfarOut = DT.cfarOutFmt1D()
			# tmpCfarOut.rangeIdx = rangeIdx
			tmpCfarOut.idx = idxCUT
			tmpCfarOut.val = inp[idxCUT]
			tmpCfarOut.SNR = tmpCfarOut.val - sumNoise

			cfarOutList.append(tmpCfarOut)


	if DEBUG:
		return cfarOutList, thresholdLine
	else:
		return cfarOutList

def azimuthSNRBasedPrunning(cfarOut3DList, thresh):
	pruneList = []
	thresh = thresh / 6
	for obj in cfarOut3DList[:]:
		tmp = obj.angleSNR

		if tmp < thresh and obj.range < 50:
			pruneList.append(obj)
			cfarOut3DList.remove(obj)
			# del(obj)

	return pruneList

def cfar_OS(inp, cfarConfig, DEBUG = True, onePointProcess = None):

	if onePointProcess == None:
		length = len(inp)
		idxCUTList = range(length)
	else:
		idxCUTList = [onePointProcess]
	
	cfarOutList = []
	length = len(inp)
	DE = False

	if DEBUG:
		thresholdLine = np.zeros(length, dtype = float)
		# noiseLine = np.zeros(length, dtype = float)

	for idxCUT in idxCUTList:

		sumRightIdx1, sumRightIdx2, sumLeftIdx1, sumLeftIdx2 = cfarWindowRange(idxCUT, cfarConfig, length)

		windowLen = (sumRightIdx2 - sumRightIdx1) + (sumLeftIdx1 - sumLeftIdx2)

		noiseWin = np.append(inp[sumLeftIdx2:sumLeftIdx1], inp[sumRightIdx1:sumRightIdx2])
		noiseWin = np.sort(noiseWin)

		noiseIdx = int((cfarConfig.OS_prun) * windowLen)

		# sumRight = np.sum(inp[sumRightIdx1 : sumRightIdx2])

		# sumLeft = np.sum(inp[sumLeftIdx2 : sumLeftIdx1])

		sumNoise = noiseWin[noiseIdx]



		if DEBUG == True:
			thresholdLine[idxCUT] = sumNoise + cfarConfig.thresholdScale

		if inp[idxCUT] > sumNoise + cfarConfig.thresholdScale:

			tmpCfarOut = DT.cfarOutFmt1D()
			# tmpCfarOut.rangeIdx = rangeIdx
			tmpCfarOut.idx = idxCUT
			tmpCfarOut.val = inp[idxCUT]
			tmpCfarOut.SNR = tmpCfarOut.val - sumNoise

			cfarOutList.append(tmpCfarOut)

	if DEBUG:
		return cfarOutList, thresholdLine
	else:
		return cfarOutList

def cfar_CA_DC(_inp, cfarConfig, DEBUG = True, onePointProcess = None):

	inp = np.copy(_inp)
	if onePointProcess == None:
		length = len(inp)
		idxCUTList = range(length)
	else:
		idxCUTList = [onePointProcess]
	cfarOutList = []
	length = len(inp)
	DE = False

	if DEBUG:
		thresholdLine = np.zeros(length, dtype = float)
		# noiseLine = np.zeros(length, dtype = float)

	for idxCUT in idxCUTList:

		sumRightIdx1, sumRightIdx2, sumLeftIdx1, sumLeftIdx2 = cfarWindowRange(idxCUT, cfarConfig, length)

		windowLen = (sumRightIdx2 - sumRightIdx1) + (sumLeftIdx1 - sumLeftIdx2)

		# noiseWin = np.append(inp[sumLeftIdx2:sumLeftIdx1],inp[sumRightIdx1:sumRightIdx2])
		# noiseWin = np.sort(noiseWin)

		# noiseIdx = int((cfarConfig.OS_prun)*windowLen)

		sumRight = np.sum(inp[sumRightIdx1 : sumRightIdx2])

		sumLeft = np.sum(inp[sumLeftIdx2 : sumLeftIdx1])

		sumNoise = (sumRight + sumLeft) / windowLen



		# if DEBUG == True:
		# 	thresholdLine[idxCUT] = sumNoise + cfarConfig.thresholdScale

		if inp[idxCUT] > sumNoise + cfarConfig.thresholdScale:

			inp[idxCUT] = sumNoise

			# tmpCfarOut = DT.cfarOutFmt1D()
			# # tmpCfarOut.rangeIdx = rangeIdx
			# tmpCfarOut.idx = idxCUT
			# tmpCfarOut.val = inp[idxCUT]
			# tmpCfarOut.SNR = tmpCfarOut.val - sumNoise

			# cfarOutList.append(tmpCfarOut)
	
	#second cfar

	# plt.plot(_inp)
	# plt.plot(inp)
	# plt.show()

	for idxCUT in idxCUTList:

		sumRightIdx1, sumRightIdx2, sumLeftIdx1, sumLeftIdx2 = cfarWindowRange(idxCUT, cfarConfig, length)

		windowLen = (sumRightIdx2 - sumRightIdx1) + (sumLeftIdx1 - sumLeftIdx2)

		# noiseWin = np.append(inp[sumLeftIdx2:sumLeftIdx1],inp[sumRightIdx1:sumRightIdx2])
		# noiseWin = np.sort(noiseWin)

		# noiseIdx = int((cfarConfig.OS_prun)*windowLen)

		sumRight = np.sum(inp[sumRightIdx1 : sumRightIdx2])

		sumLeft = np.sum(inp[sumLeftIdx2 : sumLeftIdx1])

		sumNoise = (sumRight + sumLeft) / windowLen



		if DEBUG == True:
			thresholdLine[idxCUT] = sumNoise + cfarConfig.thresholdScale

		if _inp[idxCUT] > sumNoise + cfarConfig.thresholdScale:

			tmpCfarOut = DT.cfarOutFmt1D()
			# tmpCfarOut.rangeIdx = rangeIdx
			tmpCfarOut.idx = idxCUT
			tmpCfarOut.val = _inp[idxCUT]
			tmpCfarOut.SNR = tmpCfarOut.val - sumNoise

			cfarOutList.append(tmpCfarOut)

	if DEBUG:
		return cfarOutList, thresholdLine
	else:
		return cfarOutList


def cfar_CA_GO(inp, cfarConfig, DEBUG = True, onePointProcess = None):


	if onePointProcess == None:
		length = len(inp)
		idxCUTList = range(length)
	else:
		idxCUTList = [onePointProcess]
	cfarOutList = []
	length = len(inp)

	if DEBUG:
		thresholdLine = np.zeros(length, dtype = float)
		# noiseLine = np.zeros(length, dtype = float)

	for idxCUT in idxCUTList:

		sumRightIdx1, sumRightIdx2, sumLeftIdx1, sumLeftIdx2 = cfarWindowRange(idxCUT, cfarConfig, length)

		# windowLen = (sumRightIdx2 - sumRightIdx1) + (sumLeftIdx1 - sumLeftIdx2) 

		sumNoise = 0

		sumRight = np.sum(inp[sumRightIdx1 : sumRightIdx2])

		sumLeft = np.sum(inp[sumLeftIdx2 : sumLeftIdx1])

		if sumRight > sumLeft:
			sumNoise = sumRight / cfarConfig.noiseLen
		else:
			sumNoise = sumLeft / cfarConfig.noiseLen

		if DEBUG == True:
			thresholdLine[idxCUT] = sumNoise + cfarConfig.thresholdScale

		if inp[idxCUT] > sumNoise + cfarConfig.thresholdScale:

			tmpCfarOut = DT.cfarOutFmt1D()
			# tmpCfarOut.rangeIdx = rangeIdx
			tmpCfarOut.idx = idxCUT
			tmpCfarOut.val = inp[idxCUT]
			tmpCfarOut.SNR = tmpCfarOut.val - sumNoise

			cfarOutList.append(tmpCfarOut)



	if DEBUG:
		return cfarOutList, thresholdLine
	else:
		return cfarOutList

# def cfarCa_SO_dBwrap_withSNR(inp, cfarConfig, DEBUG = True, Prunning = True):
# 	outIdx = 0
# 	sumLeft = 0
# 	sumRight = 0
# 	length = len(inp)
#
# 	for idx in range(cfarConfig.noiseLen):
# 		sumLeft += inp[idxCUT - cfarConfig.guardLen - cfarConfig.noiseLen + idx]
# 		sumRight += inp[idxCUT + cfarConfig.guardLen + 1 + idx]
#
# 	if sumLeft < sumRight:
# 		sumNoise = sumLeft
# 	else:
# 		sumNoise = sumRight


def cfar_CA_SO(inp, cfarConfig, boundary = None, DEBUG = True, onePointProcess = None, isSlow = False, cfarType = "CFAR_CA_SO"):

	if onePointProcess == None:
		length = len(inp)
		idxCUTList = range(length)
	else:
		idxCUTList = [onePointProcess]
	
	cfarOutList = []
	length = len(inp)

	if DEBUG:
		thresholdLine = np.zeros(length, dtype = float)
		# noiseLine = np.zeros(length, dtype = float)

	for idxCUT in idxCUTList:
		if isSlow and cfarConfig.dbWrap == True:
			if CONST.doIsInBoundaryProcss == True and not(isInBoundary(boundary.uB1, boundary.lB1, idxCUT) or isInBoundary(boundary.uB2, boundary.lB2, idxCUT)):
				continue

		if cfarConfig.dbWrap:
			sumRightIdx1, sumRightIdx2, sumLeftIdx1, sumLeftIdx2 = cfarWindowRange_dbWrap(idxCUT, cfarConfig, length)
		else:
			sumRightIdx1, sumRightIdx2, sumLeftIdx1, sumLeftIdx2 = cfarWindowRange(idxCUT, cfarConfig, length)

		# windowLen = (sumRightIdx2 - sumRightIdx1) + (sumLeftIdx1 - sumLeftIdx2) 

		sumLeft = 0
		sumRight = 0
		if cfarConfig.dbWrap:
			for idx in range(cfarConfig.noiseLen):
				sumLeft += inp[idxCUT - cfarConfig.guardLen - cfarConfig.noiseLen + idx]
				sumRight += inp[(idxCUT + cfarConfig.guardLen + 1 + idx)%length]
		else:
			sumRight = np.sum(inp[sumRightIdx1: sumRightIdx2])
			sumLeft = np.sum(inp[sumLeftIdx2: sumLeftIdx1])

#cfarType===================================================
		if cfarConfig.dbWrap:
			windowLen = cfarConfig.noiseLen
			if cfarType == "CFAR_CA_SO":
				if sumLeft < sumRight:
					sumNoise = sumLeft / windowLen
				else:
					sumNoise = sumRight /windowLen
			elif cfarType == "CFAR_CA_LW":
				sumNoise = sumLeft / windowLen
			else:
				print("cfarType is wrong, please Check")
				sys.exit()
		else:
			if cfarType == "CFAR_CA_SO":
				if (sumLeftIdx1 - sumLeftIdx2) > (sumRightIdx2 - sumRightIdx1):
					windowLen = cfarConfig.noiseLen
					sumNoise = sumLeft / windowLen

				elif (sumLeftIdx1 - sumLeftIdx2) < (sumRightIdx2 - sumRightIdx1):
					windowLen = cfarConfig.noiseLen
					sumNoise = sumRight / windowLen

				elif sumRight > sumLeft:
					windowLen = cfarConfig.noiseLen
					sumNoise = sumLeft / windowLen

				else:
					windowLen = cfarConfig.noiseLen
					sumNoise = sumRight / windowLen

			elif cfarType == "CFAR_CA_LW":
				if (sumLeftIdx1 - sumLeftIdx2) < (sumRightIdx2 - sumRightIdx1):
					windowLen = cfarConfig.noiseLen
					sumNoise = sumRight / windowLen
				else:
					windowLen = cfarConfig.noiseLen
					sumNoise = sumLeft / windowLen
			else:
				print("cfarType is wrong, please Check")
				sys.exit()
#==========================================================================
		if DEBUG == True:
			thresholdLine[idxCUT] = sumNoise + cfarConfig.thresholdScale

		if inp[idxCUT] > sumNoise + cfarConfig.thresholdScale:

			tmpCfarOut = DT.cfarOutFmt1D()
			# tmpCfarOut.rangeIdx = rangeIdx
			tmpCfarOut.idx = idxCUT
			tmpCfarOut.val = inp[idxCUT]
			tmpCfarOut.SNR = tmpCfarOut.val - sumNoise
			tmpCfarOut.isSlow = isSlow
			# print("debug : ", tmpCfarOut.val, sumNoise)
			cfarOutList.append(tmpCfarOut)
	
	if cfarConfig.pruningType == "pruneToPeaks":
		pruneToPeaks(cfarOutList,inp, length)
	elif cfarConfig.pruningType == "pruneToPeaksOrNeighbourOfPeaks":
		pruneToPeaksOrNeighbourOfPeaks(cfarOutList, inp, length)
	elif cfarConfig.pruningType == "pruneToPeaksOr2NeighbourOfPeaks":
		pruneToPeaksOr2NeighbourOfPeaks(cfarOutList, inp, length)
	else :
		pass
	if DEBUG:
		return cfarOutList, thresholdLine
	else:
		return cfarOutList

def cfar2DPrunningForNegetiveRange(cfarOut2DList):
	for obj in cfarOut2DList[:]:
		if obj.rangeIdx < 64:
			cfarOut2DList.remove(obj)
			del(obj)
	return cfarOut2DList

def pruneToPeaksOr2NeighbourOfPeaks(cfarOutList, inp, length):
	for cfarOut in cfarOutList[:]:
		if cfarOut.idx == 0:
			prevIdx = length - 1
		else:
			prevIdx = cfarOut.idx - 1
		if prevIdx == 0:
			prevPrevIdx = length - 1
		else:
			prevPrevIdx = prevIdx - 1
		if prevPrevIdx == 0:
			prevPrevPrevIdx = length - 1
		else:
			prevPrevPrevIdx = prevPrevIdx -1

		if cfarOut.idx == length - 1 :
			nextIdx = 0
		else:
			nextIdx = cfarOut.idx + 1

		if nextIdx == length - 1:
			nextNextIdx = 0
		else:
			nextNextIdx = nextIdx + 1
		if nextNextIdx == length - 1:
			nextNextNextIdx = 0
		else:
			nextNextNextIdx = nextNextIdx + 1


		is_peak = (inp[cfarOut.idx] > inp[prevIdx]) and (inp[cfarOut.idx] > inp[nextIdx])
		is_neighbourOfPeakNext = (inp[cfarOut.idx] > inp[prevIdx]) and (inp[cfarOut.idx] > inp[nextNextIdx])
		is_neighbourOfPeakPrev = (inp[cfarOut.idx] > inp[prevPrevIdx]) and (inp[cfarOut.idx] > inp[nextIdx])
		is_neighbourOfPeakNextNext =  (inp[cfarOut.idx] > inp[prevIdx]) and (inp[cfarOut.idx] > inp[nextNextNextIdx])
		is_neighbourOfPeakPrevPrev = (inp[cfarOut.idx] > inp[prevPrevPrevIdx]) and (inp[cfarOut.idx] > inp[nextIdx])

		if is_peak or is_neighbourOfPeakNext or is_neighbourOfPeakPrev or is_neighbourOfPeakNextNext or is_neighbourOfPeakPrevPrev:
			continue
		else:

			if CONST.peakPruningDEBUG:
				if length == 128:
					cfarAxis = "doppler"
				else:
					cfarAxis = "range"
				print("the detected point %s.idx = %d is removed on peakPruning"%(cfarAxis, cfarOut.idx))
			cfarOutList.remove(cfarOut)
			del (cfarOut)

def pruneToPeaksOrNeighbourOfPeaks(cfarOutList, inp, length):
	for cfarOut in cfarOutList[:]:
		if cfarOut.idx == 0:
			prevIdx = length - 1
		else:
			prevIdx = cfarOut.idx - 1
		if prevIdx == 0:
			prevPrevIdx = length - 1
		else:
			prevPrevIdx = prevIdx - 1

		if cfarOut.idx == length - 1 :
			nextIdx = 0
		else:
			nextIdx = cfarOut.idx + 1

		if nextIdx == length - 1:
			nextNextIdx = 0
		else:
			nextNextIdx = nextIdx + 1

		is_peak = (inp[cfarOut.idx] > inp[prevIdx]) and (inp[cfarOut.idx] > inp[nextIdx])
		is_neighbourOfPeakNext = (inp[cfarOut.idx] > inp[prevIdx]) and (inp[cfarOut.idx] > inp[nextNextIdx])
		is_neighbourOfPeakPrev = (inp[cfarOut.idx] > inp[prevPrevIdx]) and (inp[cfarOut.idx] > inp[nextIdx])

		# if (is_peak or is_neighbourOfPeakNext or is_neighbourOfPeakPrev):
		# 	continue
#		if ((is_peak or is_neighbourOfPeakNext or is_neighbourOfPeakPrev) and cfarOut.idx < 72) or (is_peak and not(cfarOut.idx < 72)):
		if (is_peak or is_neighbourOfPeakNext or is_neighbourOfPeakPrev):
			continue
		else:

			if CONST.peakPruningDEBUG:
				if length == 128:
					cfarAxis = "doppler"
				else:
					cfarAxis = "range"
				print("the detected point %s.idx = %d is removed on peakPruning"%(cfarAxis, cfarOut.idx))
			cfarOutList.remove(cfarOut)
			del (cfarOut)

def pruneToPeaks(cfarOutList, inp, length):
	for cfarOut in cfarOutList[:]:

		prevIdx = cfarOut.idx - 1
		nextIdx = cfarOut.idx + 1
		if prevIdx < 0:
			pass
		elif nextIdx > length - 1:
			nextIdx = nextIdx % length


		if (inp[cfarOut.idx] > inp[prevIdx]) and (inp[cfarOut.idx] > inp[nextIdx]):
			continue
		else:
			cfarOutList.remove(cfarOut)
			del (cfarOut)

def pruningKLargestVal(tmpCfarOut1DList, K, rangeIdx):
	tmpList = []
	
	for k in range(K):
		tmpVal = 0
		tmpObj = None
		for obj in tmpCfarOut1DList[:]:
			if tmpVal < obj.val:
				tmpVal = obj.val
				tmpObj = obj
		tmpList.append(tmpObj)
		tmpCfarOut1DList.remove(tmpObj)
		del(tmpObj)
	if CONST.pruningKLargestValDEBUG:
		print("following target (dopplerIdx, rangeIdx) list is prunned by pruningKLargestVal")
		print([(i.idx, rangeIdx) for i in tmpCfarOut1DList])
	return tmpList

def cfar2D(cfarOut2DList, heatMap, heatMap_Slow, cfarCfgRange, cfarCfgDoppler, K , boundary, isSlow = False):

	shape = heatMap.shape

	for rangeIdx in range(shape[0]):

		tmpCfarOut1DList = cfar_CA_SO(heatMap[rangeIdx, :], cfarCfgDoppler, boundary, DEBUG = False, isSlow = False, cfarType= cfarCfgDoppler.cfarType)

		if CONST.SLOW_CFAR:
			tmpCfarOut1DList_slow = cfar_CA_SO(heatMap_Slow[rangeIdx, :], cfarCfgDoppler, boundary, DEBUG=False,
		                                        isSlow=True, cfarType= cfarCfgDoppler.cfarType)
			tmpCfarOut1DList = tmpCfarOut1DList + tmpCfarOut1DList_slow

		if len(tmpCfarOut1DList) > K :
			tmpCfarOut1DList = pruningKLargestVal(tmpCfarOut1DList, K, rangeIdx)

		for tmpCfarOut1D in tmpCfarOut1DList:
			if tmpCfarOut1D.isSlow:
				tmpHeatMap = heatMap_Slow
			else:
				tmpHeatMap = heatMap

			tmpOut = cfar_CA_SO(tmpHeatMap[:, tmpCfarOut1D.idx], cfarCfgRange,  DEBUG = False, onePointProcess = rangeIdx, isSlow = tmpCfarOut1D.isSlow, cfarType= cfarCfgRange.cfarType)

			if not tmpOut:
				continue

			tmpCfarOut2D = DT.cfarOutFmt2D()
			tmpCfarOut2D.rangeIdx = rangeIdx
			tmpCfarOut2D.rangeVal = tmpOut[0].val
			tmpCfarOut2D.rangeSNR = tmpOut[0].SNR
			tmpCfarOut2D.dopplerIdx = tmpCfarOut1D.idx
			tmpCfarOut2D.dopplerVal = tmpCfarOut1D.val
			tmpCfarOut2D.dopplerSNR = tmpCfarOut1D.SNR
			tmpCfarOut2D.isSlow     = tmpCfarOut1D.isSlow

			cfarOut2DList.append(tmpCfarOut2D)
			# for i in cfarOut2DList:
			# 	print(i.rangeIdx, i.dopplerIdx)

	for idx, obj in enumerate(cfarOut2DList):
		obj.detID = idx
		# print(obj.detID)
	return cfarOut2DList

def quadraticInterp(y, currIdxList):
	
	thetaPkList = []
	for currIdx in currIdxList:
		length = len(y)
		# print(length)
		thetaPk = 0
		if currIdx == length - 1:
			nextIdx = 0
		else:
			nextIdx = currIdx + 1
		prevIdx = currIdx - 1

		y0 = y[currIdx] * y[currIdx]
		yp1 = y[nextIdx] * y[nextIdx]
		ym1 = y[prevIdx] * y[prevIdx]

		Den =  2 * (2 * y0 - ym1 - yp1)


		if Den > 0.15:
			thetaPk = (yp1 - ym1) / Den

		thetaPkList.append(thetaPk)

	return thetaPkList

def quadraticInterpLog(y, currIdx):
	
	length = len(y)
	thetaPk = 0
	if currIdx == length - 1:
		nextIdx = 0
	else:
		nextIdx = currIdx + 1
	prevIdx = currIdx - 1

	y0 = np.power(2, y[currIdx])
	yp1 = np.power(2, y[nextIdx])
	ym1 = np.power(2, y[prevIdx])
	y0 = y0 * y0
	yp1 = yp1 * yp1
	ym1 = ym1 * ym1

	Den = 2 * (2 * y0 - ym1 - yp1)
	if CONST.quadraticInterpLogMODIFY:
		if (y0 > yp1) and (y0 > ym1):
			if Den > 0.15:
				thetaPk = (yp1 - ym1) / Den
	else:
		if Den > 0.15:
			thetaPk = (yp1 - ym1) / Den
	return thetaPk

def CRT(obj, heatMap_Slow_org, heatMap_Slow, enhMaxVelCfg, domain, heatMap, velResolutionFastChirp, velResolutionSlowChirp, maxUnambiguousVel, isAliasing,  slowObj = False, DEBUG = False):
	#heatMap_Slow : sub heatMap, heatMap : main heatMap
	obj.dopplerOffset = quadraticInterpLog(heatMap[obj.rangeIdx, :], obj.dopplerIdx)
	if DEBUG == True:
		print("dopplerOffset : %.2f" % obj.dopplerOffset)

	# HSLee 수정 2021.10.05
	if (obj.dopplerIdx != 0) and (obj.dopplerIdx != 16):
		fastChirpVel = (obj.dopplerIdx + obj.dopplerOffset - enhMaxVelCfg.n_chirps / 2) * velResolutionFastChirp
	else:
		fastChirpVel = (obj.dopplerIdx - enhMaxVelCfg.n_chirps / 2) * velResolutionFastChirp
	# HSLee 수정 2021.10.05

	if CONST.TDM_MIMO:
		if isAliasing:
			if fastChirpVel < 0:
				fastChirpVel = fastChirpVel + maxUnambiguousVel
			else:
				fastChirpVel = fastChirpVel - maxUnambiguousVel
	slowChirpPeakArr = np.zeros(enhMaxVelCfg.N_HYPOTHESIS, dtype=np.float)
	validArr = np.negative(np.ones(enhMaxVelCfg.N_HYPOTHESIS, dtype=int))

	for AmbIndx in range(enhMaxVelCfg.N_HYPOTHESIS):

		thresh = enhMaxVelCfg.thresh
		AmbIndxActual = AmbIndx - (enhMaxVelCfg.MAX_VEL_ENH_NUM_NYQUIST - 1)

		if not (AmbIndxActual == 0):
			thresh = enhMaxVelCfg.thresh * enhMaxVelCfg.threshMultiplier

		fastChirpAmbVel = AmbIndxActual * (2 * maxUnambiguousVel) + fastChirpVel

		velIndxFlt = (fastChirpAmbVel / velResolutionSlowChirp) + enhMaxVelCfg.n_chirps / 2

		if DEBUG == True:
			print("for AmbIndxActual %d velIndxFlt : %d " % (AmbIndxActual, velIndxFlt))
		# velIndxFlt = fastChirpAmbVel / velResolutionSlowChirp
		if velIndxFlt < 0:
			velIndxTmp = -1 * velIndxFlt

			numMult = velIndxTmp // enhMaxVelCfg.n_chirps
			velIndxFlt += (enhMaxVelCfg.n_chirps * (numMult + 1))
			velIndx = int(velIndxFlt)
		elif (velIndxFlt >= enhMaxVelCfg.n_chirps):
			velIndxTmp = velIndxFlt

			numMult = velIndxTmp // enhMaxVelCfg.n_chirps
			velIndxFlt -= (enhMaxVelCfg.n_chirps * numMult)
			velIndx = int(velIndxFlt)
		else:
			velIndx = int(velIndxFlt)

		if DEBUG == True:
			print("velIndx : %d " % velIndx)
		for spreadIndx in range(enhMaxVelCfg.MAX_VEL_IMPROVEMENT_NUM_SPREAD):
#			velIndxTmp = velIndx + spreadIndx
			velIndxTmp = velIndx + spreadIndx - 1					# HSLee 수정 2021.10.14

			if velIndxTmp > (enhMaxVelCfg.n_chirps - 1):
				velIndxTmp = velIndxTmp - enhMaxVelCfg.n_chirps
			if velIndxTmp == 0:
				prevIndx = enhMaxVelCfg.n_chirps - 1
			else:
				prevIndx = velIndxTmp - 1
			if velIndxTmp == (enhMaxVelCfg.n_chirps - 1):
				nextIndx = 0
			else:
				nextIndx = velIndxTmp + 1

			if DEBUG == True:



				if slowObj:
					print("velIndxTmp : %d " % velIndxTmp)
					if isAliasing:
						plt.scatter(domain.dopplerDomain[velIndxTmp], heatMap_Slow[obj.rangeIdx, velIndxTmp],
						            color='orange')
					else:
						plt.scatter(domain.dopplerDomain[velIndxTmp], heatMap_Slow[obj.rangeIdx, velIndxTmp],
						            color='red')
					# plt.text(velIndxTmp+2, heatMap_Slow[obj.rangeIdx, velIndxTmp], "AmbIndxActual : %d, velIndxTmp : %d, velSlow: %.2f"%(AmbIndxActual,velIndxTmp,velIndxTmp * velResolutionSlowChirp ))
					plt.text(domain.dopplerDomain[velIndxTmp] + 2 * domain.dopplerRes,
					         heatMap_Slow[obj.rangeIdx, velIndxTmp],
					         "AmbIndxActual : %d, velIndxTmp : %d" % (AmbIndxActual, velIndxTmp))
				else:
					print("velIndxTmp : %d " % velIndxTmp)
					if isAliasing:
						plt.scatter(domain.dopplerDomain_Slow[velIndxTmp], heatMap_Slow[obj.rangeIdx, velIndxTmp],
						            color='orange')
					else:
						plt.scatter(domain.dopplerDomain_Slow[velIndxTmp], heatMap_Slow[obj.rangeIdx, velIndxTmp], color='red')
				# plt.text(velIndxTmp+2, heatMap_Slow[obj.rangeIdx, velIndxTmp], "AmbIndxActual : %d, velIndxTmp : %d, velSlow: %.2f"%(AmbIndxActual,velIndxTmp,velIndxTmp * velResolutionSlowChirp ))
					plt.text(domain.dopplerDomain_Slow[velIndxTmp]+2*domain.dopplerRes, heatMap_Slow[obj.rangeIdx, velIndxTmp], "AmbIndxActual : %d, velIndxTmp : %d"%(AmbIndxActual,velIndxTmp ))

			# print("velIndxTmp : %d " %velIndxTmp)
			if (heatMap_Slow[obj.rangeIdx, velIndxTmp] <= heatMap_Slow[obj.rangeIdx, nextIndx]) or (
					heatMap_Slow[obj.rangeIdx, velIndxTmp] <= heatMap_Slow[obj.rangeIdx, prevIndx]):
				if DEBUG == True:
					print("heatMap_Slow[velIndxTmp : %d] is not peak" % velIndxTmp)
				continue
			if (heatMap_Slow[obj.rangeIdx, nextIndx]) == 0 or (heatMap_Slow[obj.rangeIdx, prevIndx]) == 0:
				if DEBUG == True:
					print("heatMap_Slow[velIndxTmp + 1] or heatMap_Slow[velIndxTmp - 1] is 0")
				continue
			diff = abs(heatMap_Slow[obj.rangeIdx, velIndxTmp] - obj.dopplerVal)
			if DEBUG == True:

				print("diff : %.2f"%diff)
			if diff > thresh:
				if DEBUG == True:
					print("diff > thresh : %.2f"%thresh)
				continue
			slowChirpPeakArr[AmbIndx] = heatMap_Slow[obj.rangeIdx, velIndxTmp]
			if CONST.CRT_OVERLAP:
				validArr[AmbIndx] = velIndxTmp
			else:
				# validArr[AmbIndx] = velIndxTmp #need to be modified
				validArr[AmbIndx] = velIndx

			break

	peakIndx = -1
	peakIndx_2 = -1
	peakVal = 0

	if DEBUG == True:
		print("slowChirpPeakArr")
		print(slowChirpPeakArr)
		print("validArr")
		print(validArr)

	tmpDiff = enhMaxVelCfg.thresh * enhMaxVelCfg.threshMultiplier
	cnt = 0
	maxval = 0
	for AmbIndx in range(enhMaxVelCfg.N_HYPOTHESIS):
		if validArr[AmbIndx] >= 0:
			if isAliasing:
				obj.CRT_Flag_als = 1
			else:
				obj.CRT_Flag = 1

			cnt += 1

			# HSLee 수정 2021.10.14
#			diff = abs(slowChirpPeakArr[AmbIndx] - obj.dopplerVal)
#			if tmpDiff > diff:
#				peakVal = slowChirpPeakArr[AmbIndx]
#				peakIndx = AmbIndx
#				tmpDiff = diff
#				if DEBUG == True:
#					print("tmpDiff : %f" % tmpDiff)

			if abs(slowChirpPeakArr[AmbIndx]) > maxval:
				peakval = slowChirpPeakArr[AmbIndx]
				peakIndx = AmbIndx
				maxval = abs(slowChirpPeakArr[AmbIndx])
				if DEBUG == True:
					print("maxval : %f" % maxval)
			# HSLee 수정 2021.10.14

	if cnt > 1:
		if isAliasing:
			obj.CRT_Flag_als = 2
		else:
			obj.CRT_Flag = 2

		maxval = 0
		for AmbIndx in range(enhMaxVelCfg.N_HYPOTHESIS):
			if validArr[AmbIndx] >= 0 and AmbIndx != peakIndx:
				# HSLee 수정 2021.10.14
#				diff = abs(slowChirpPeakArr[AmbIndx] - obj.dopplerVal)
#				if tmpDiff > diff:
#					peakVal = slowChirpPeakArr[AmbIndx]
#					peakIndx_2 = AmbIndx
#					tmpDiff = diff
#					if DEBUG == True:
#						print("tmpDiff : %f" % tmpDiff)

				if abs(slowChirpPeakArr[AmbIndx]) > maxval:
					peakval = slowChirpPeakArr[AmbIndx]
					peakIndx_2 = AmbIndx
					maxval = abs(slowChirpPeakArr[AmbIndx])
					if DEBUG == True:
						print("maxval : %f" % maxval)
				# HSLee 수정 2021.10.14

	if peakIndx != -1:
		velIndx = validArr[peakIndx]

		for spreadIndx in range(enhMaxVelCfg.MAX_VEL_IMPROVEMENT_NUM_SPREAD):
#			velIndxTmp = velIndx + spreadIndx
			velIndxTmp = velIndx + spreadIndx - 1					# HSLee 수정 2021.10.14
			if velIndxTmp > (enhMaxVelCfg.n_chirps - 1):
				velIndxTmp = velIndxTmp - enhMaxVelCfg.n_chirps
				if enhMaxVelCfg.pruningType == "pruneToPeaks":
					if not CONST.SLOW_CFAR:
						heatMap_Slow[obj.rangeIdx, velIndxTmp] = 0
	else:
		if isAliasing:
			obj.CRT_Flag_als = 0
		else:
			obj.CRT_Flag = 0
		peakIndx = 1
	if isAliasing:
		obj.velDisambFacValidity_als = peakIndx
		obj.CRT_slow_doppler_indx_als = validArr[peakIndx]
		obj.velDisambFacValidity_als_2 = peakIndx_2
		obj.CRT_slow_doppler_indx_als_2 = validArr[peakIndx_2]
	else:
		obj.velDisambFacValidity = peakIndx
		obj.CRT_slow_doppler_indx = validArr[peakIndx]
		obj.velDisambFacValidity_2 = peakIndx_2
		obj.CRT_slow_doppler_indx_2 = validArr[peakIndx_2]


def enhancedMaxVel(cfarOutNDList,heatMap_Slow_org, enhMaxVelCfg, domain, heatMap, DEBUG = False):

	maxUnambiguousVel = enhMaxVelCfg.lightSpeed_meters_per_sec/enhMaxVelCfg.carrierFreq/4/enhMaxVelCfg.period_chirp
	velResolutionSlowChirp = enhMaxVelCfg.lightSpeed_meters_per_sec/enhMaxVelCfg.carrierFreq/2/enhMaxVelCfg.period_chirp_slow/enhMaxVelCfg.n_chirps
	velResolutionFastChirp = enhMaxVelCfg.lightSpeed_meters_per_sec/enhMaxVelCfg.carrierFreq/2/enhMaxVelCfg.period_chirp/enhMaxVelCfg.n_chirps
	heatMap_Slow = np.copy(heatMap_Slow_org)
	maxUnambiguousVel_slow = enhMaxVelCfg.lightSpeed_meters_per_sec / enhMaxVelCfg.carrierFreq / 4 / enhMaxVelCfg.period_chirp_slow
	if CONST.TDM_MIMO:
		maxUnambiguousVel       =  enhMaxVelCfg.numTx * maxUnambiguousVel
		maxUnambiguousVel_slow  =  enhMaxVelCfg.numTx * maxUnambiguousVel_slow

	if DEBUG == True:

		print("enhancedMaxVel function start!")
		print("input.detID : ", cfarOutNDList[0].detID)
		print("range : %f"%(domain.rangeDomain[cfarOutNDList[0].rangeIdx]))
		print("maxUnambiguousVel: %.2f"%maxUnambiguousVel)
		print("velResolutionSlowChirp : %.2f"%velResolutionSlowChirp)
		print("velResolutionFastChirp : %.2f"%velResolutionFastChirp)

	for obj in cfarOutNDList:

		if DEBUG == True:
			# plt.legend(loc = 1)
			# plt.grid(linestyle = ":")
			# plt.title("enhancedMaxVel DEBUG")
			# plt.xlabel("Velocity(m/s)")
			# plt.ylabel("dB (log2 scale)")
			# plt.show()
			print("========================================================")
			print("currObjDopplerIdx: %d" % obj.dopplerIdx)
			# print("objInfo Range : %.2f, doppler : %.2f" % (obj.rangeIdx * domain.rangeRes, fastChirpVel))
			fig = plt.figure(figsize=(16, 9))

			# plt.figure()
			plt.plot(domain.dopplerDomain, heatMap[obj.rangeIdx, :], color='black', label='fast chirp')
			if obj.isSlow:
				plt.scatter(domain.dopplerDomain_Slow[obj.dopplerIdx], heatMap_Slow[obj.rangeIdx, obj.dopplerIdx], color='blue')
			else:
				# print("sagag",domain.dopplerDomain[obj.dopplerIdx], heatMap[obj.rangeIdx, obj.dopplerIdx])
				plt.scatter(domain.dopplerDomain[obj.dopplerIdx], heatMap[obj.rangeIdx, obj.dopplerIdx], color='blue')
			plt.plot(domain.dopplerDomain_Slow, heatMap_Slow_org[obj.rangeIdx, :], color='green', label='slow chirp')
		if not obj.isSlow:
			CRT(obj, heatMap_Slow_org, heatMap_Slow, enhMaxVelCfg, domain, heatMap, velResolutionFastChirp,
			    velResolutionSlowChirp, maxUnambiguousVel, False, DEBUG = DEBUG)
			CRT(obj, heatMap_Slow_org, heatMap_Slow, enhMaxVelCfg, domain, heatMap, velResolutionFastChirp,
			    velResolutionSlowChirp, maxUnambiguousVel, True,  DEBUG = DEBUG)
		else:
			CRT(obj, heatMap_Slow_org, heatMap, enhMaxVelCfg, domain, heatMap_Slow, velResolutionSlowChirp,
			    velResolutionFastChirp, maxUnambiguousVel_slow, False,  slowObj = obj.isSlow,DEBUG = DEBUG)
			CRT(obj, heatMap_Slow_org, heatMap, enhMaxVelCfg, domain, heatMap_Slow, velResolutionSlowChirp,
			    velResolutionFastChirp, maxUnambiguousVel_slow, True,  slowObj = obj.isSlow, DEBUG = DEBUG)
	if DEBUG == True:

		print("\nFinal result of CRT\n")
		print("velDisambFacValidity : %d\nCRT_slow_doppler_indx : %d\nvelDisambFacValidity_2 : %d\n	CRT_slow_doppler_indx_2 : %d\n \
		velDisambFacValidity_als : %d\n \
		CRT_slow_doppler_indx_als : %d\n \
		velDisambFacValidity_als_2 : %d\n \
		CRT_slow_doppler_indx_als_2 : %d\n"\
		      %(obj.velDisambFacValidity,\
		        obj.CRT_slow_doppler_indx,\
		        obj.velDisambFacValidity_2,\
		        obj.CRT_slow_doppler_indx_2,\
		        obj.velDisambFacValidity_als,\
		        obj.CRT_slow_doppler_indx_als,\
		        obj.velDisambFacValidity_als_2,\
		        obj.CRT_slow_doppler_indx_als_2))
		print("CRT_Flag : %d, CRT_Flag_als : %d"%(obj.CRT_Flag, obj.CRT_Flag_als))

	if DEBUG:
		if obj.isSlow:
			plt.title("enhancedMaxVel_slow DEBUG")
		else:
			plt.title("enhancedMaxVel_fast DEBUG")

		plt.xlabel("Velocity(m/s)")
		plt.ylabel("dB (log2 scale)")
		# print("peakIndx: %d"%peakIndx)
		print("enhancedMaxVel function is ended")
		plt.legend(loc = 1)
		plt.grid(linestyle = ":")

		plt.show()


def steeringVector_azimSin(azimSin, numChannel):

	a = np.exp(1j*np.arange(numChannel)*np.pi*np.sin(azimSin))
	a =np.matrix(a).T
	return a

def steeringVector(theta, numChannel):

	a = np.exp(1j*np.arange(numChannel)*np.pi*np.sin(theta))
	a =np.matrix(a).T
	return a


	#input structure : []


"""
*!*****************************************************************************************************************
 * brief
 * Function Name       :    dopplerCompensation
 *
 * par
 * <b>Description</b>  :    Performs dopplerCompensation befor AOA.
 *
 * @param[in]               inp      : input array (16 bit unsigned numbers) size : numTx*numRx
 * @param[in]               dopplerIdx      : doppler index of detected target.
 * @param[in]               config : config
 *
 * @param[out]              
 *
 * @return                  out      : output array (array after compensation)
 *
 * @pre                     
 * @ingroup                 
 *
 *******************************************************************************************************************
 """

def dopplerCompensation(inp, dopplerIdx, config, isSlow):
	if isSlow:
		selected_period_chirp = config.period_chirp_slow
	else:
		selected_period_chirp = config.period_chirp

	dopplerDomain = np.arange(-config.n_chirps / 2,
	                          config.n_chirps / 2) / config.n_chirps / 79e9 * config.lightSpeed_meters_per_sec / 4 / selected_period_chirp * 2
	alpha = 4 * np.pi * config.carrierFreq * selected_period_chirp / config.lightSpeed_meters_per_sec
	phaseDiff = alpha * dopplerDomain[dopplerIdx] #4 * pi * chirp_period * vel / lambda
	_inp = np.zeros(inp.shape,dtype = np.complex64)

	# print(np.angle(np.exp(-1j*phaseDiff)))
	if phaseDiff == 0:

		return inp
	
	else:

		for channelIdx in range(config.numTx*config.numRx):
			# print(inp[channelIdx])
			_inp[channelIdx] = inp[channelIdx]*np.exp(-1j*phaseDiff*(channelIdx//config.numRx)/config.numTx)
			# inp[index]와 같이 사용하면 클래스 정적변수처럼 사용되어 원하던 퍼포먼스가 안나옴 주의.
			# print(inp[channelIdx])
			# print((channelIdx//config.numRx))
		return _inp

def computeSinAzimSNR(xx, angleIdxList, numTx, numRx, numAngleBins):
	SNRList = []
	numVirtualAntAzim = int(numTx*numRx)
	stepSize = int(numAngleBins/numVirtualAntAzim)
	
	for angleIdx in angleIdxList:
		NoiseFloorSqrSum = 0		
		for i in range(1,numVirtualAntAzim):
			Idx = stepSize * i + angleIdx
			if Idx > numAngleBins - 1:
				Idx -= numAngleBins
			NoiseFloorSqrSum += xx[Idx]

		NoiseFloorSqrSum = NoiseFloorSqrSum / (numVirtualAntAzim - 1)

		SNRList.append(xx[angleIdx] / NoiseFloorSqrSum)

	return SNRList

def sin2theta(thetaPk):

	return np.arcsin(thetaPk)

def pltAngleSpectrum(data, config, cfarOut2D, aoaCfg, domain, frameNumber, procFFT = True):

	angleDomain = domain.angleDomain
	# print(angleDomain)
	isAliasing = False
	azimuthIn = np.zeros(aoaCfg.numAngleBins, dtype = np.complex64)
	p = np.zeros(angleDomain.shape[0], dtype = np.complex64)
	# print(angleDomain.shape[0])
	if CONST.TDM_MIMO:
		azimuthIn_als = np.zeros(aoaCfg.numAngleBins, dtype=np.complex64)
	outObj = cfarOut2D

	detObjRangeIdx = outObj.rangeIdx
	detObjDopplerIdx = outObj.dopplerIdx

	xx = data.FFTData[detObjRangeIdx + CONST.targetIdxPlotSpec_range_offset, detObjDopplerIdx + CONST.targetIdxPlotSpec_doppler_offset,:]
	plt.subplot(211)
	plt.plot(abs(xx), label = 'abs before')
	plt.subplot(212)
	plt.plot(np.arctan(xx.imag / xx.real), label = 'theta before')

	if CONST.TDM_MIMO:
		# print("dopplerCompensation is implemented")
		# pass
		xx = dopplerCompensation(xx, detObjDopplerIdx, config)

	if (aoaCfg.AWR1843 == True) and (aoaCfg.Mode == 'MIMO'):
		xx_ = np.zeros(8, dtype = np.complex64)
		for i in range(0,4):
			xx_[i] = xx[i]
			xx_[i+4] =xx[i+8]
		xx = xx_
	if CONST.DO_RX_COMPENSATION:
		# rxChComp = np.array([19158+5863j, 23451+2858j, 22093+4569j, 16399], dtype = np.complex64) #QFormat 15
		rxChComp = aoaCfg.rxChComp  # QFormat 15
	# xx = xx /8

		xx = MmwDemo_rxChanPhaseBiasCompensation(xx, rxChComp)

	if CONST.reverseAntenna:
		xx = np.flip(xx)
	if CONST.TDM_MIMO:
		xx_als = dopplerAliasingComp(xx, config)
	# Obtain Covariance Matrix
	plt.subplot(211)
	plt.plot(abs(xx), label = 'abs after')
	plt.title("AOA input(abs)")
	plt.grid(linestyle=":")
	plt.legend(loc=1)
	plt.subplot(212)
	plt.plot(np.arctan(xx.imag / xx.real), label = 'theta after')

	plt.title("AOA input(theta)")
	plt.grid(linestyle = ":")
	plt.legend(loc=1)
	plt.show()
	if procFFT != True:
		xx = np.matrix(xx).T
	
		Rxx = xx*xx.H

		idxP = 0

		for theta in angleDomain:

			a = steeringVector(theta, aoaCfg.numAziChannel)

			p[idxP] = (a.H*Rxx*a) / (a.H*a)

			idxP += 1
		
		# plt.plot(pbartlett)
		# plt.show()
	else:

		idx = 0
		for val in xx:
			azimuthIn[idx] = val / 8
			idx += 1

		# idx = 0
		# for i, v in enumerate(xx):
		# 	if i % 2 == 1:
		# 		azimuthIn[idx] = xx[i]
		# 		idx += 1

		tmp3DFFT = np.fft.fft(azimuthIn)
		tmp3DFFT = np.fft.fftshift(tmp3DFFT , axes = (0,))
		p = abs(tmp3DFFT)

		if CONST.TDM_MIMO:

			idx = 0
			for val in xx_als:
				azimuthIn_als[idx] = val / 8
				idx += 1
			# idx = 0
			# for i, v in enumerate(xx):
			# 	if i % 2 == 1:
			#
			# 		azimuthIn_als[idx] = xx_als[i]

			tmp3DFFT_als = np.fft.fft(azimuthIn_als)
			tmp3DFFT_als = np.fft.fftshift(tmp3DFFT_als, axes=(0,))
			p_als = abs(tmp3DFFT_als)
			p_org = copy.deepcopy(p)
			if max(p) < max(p_als):
				maxVal = max(p_als)

				p = p_als
				isAliasing = True
			else:
				maxVal = max(p)

		# pbartlett = np.log2(pbartlett)

	angleIdxList, angleValList = peakDetection(p, aoaCfg, multi =  aoaCfg.multiPeakDet)

	angleSNRList = computeSinAzimSNR(p, angleIdxList, config.numTx, config.numRx, aoaCfg.numAngleBins)

	angleSNRList = np.log2(angleSNRList)
	angleOffsetList = quadraticInterp(p, angleIdxList)
	# angleOffsetList = sin2theta(angleOffsetList)
	# print(detObjRangeIdx)
	rangeDetObj = domain.rangeDomain[detObjRangeIdx] - 0.075
	sinAzimOrg = 2 * (angleIdxList[0] + angleOffsetList[0] - aoaCfg.numAngleBins / 2) / aoaCfg.numAngleBins

	if CONST.TDM_MIMO:
		plt.plot(angleDomain*180/np.pi, p_org/maxVal, label = "3Dfft_org", color ='gray', linestyle="-")
		plt.plot(angleDomain*180/np.pi, p_als/maxVal, label = "3Dfft_alias", color ='gray', linestyle=":")
	else:
		plt.plot(angleDomain * 180 / np.pi, p/ max(p), label="3Dfft", color='gray', linestyle=":")

	plt.scatter(angleDomain[angleIdxList]*180/np.pi, p[angleIdxList]/max(p), color ='gray')
	for i in range(len(angleIdxList)):
		plt.text(angleDomain[angleIdxList[i]]*180/np.pi+5, p[angleIdxList[i]]/max(p), "angle : %.2f DEG, angleSNR : %.2fdB, angleOffset : %.2f, x : %.2fm"%(angleDomain[angleIdxList[i]]*180/np.pi, angleSNRList[i]*6, angleOffsetList[i], rangeDetObj * sinAzimOrg))
		# plt.text(angleDomain[angleIdxList[i]]*180/np.pi+5, pbartlett[angleIdxList[i]], "angle : %.2f DEG, angleSNR : %.2fdB"%(angleDomain[angleIdxList[i]]*180/np.pi, angleSNRList[i]*6))
	plt.title("rangeBin : %d, range: %.2f"%(outObj.rangeIdx, domain.rangeDomain[outObj.rangeIdx]))
	plt.xticks([i for i in range(-90, 90, 5)])
	plt.grid(linestyle = ":")

	# angleSpectrum = AOA_HIGH_RES(dataCube_1dFFT, angleDomain, aoaCfg, config, detObjRangeIdx + CONST.targetIdxPlotSpec_range_offset,
	#                              aoaCfg.res_compensation_Type)
	#
	#
	# angleIdxList, angleValList = peakDetection(angleSpectrum, aoaCfg, multi= True)
	# angleOffsetList = quadraticInterp(angleSpectrum, angleIdxList)
	# angleSpectrum = angleSpectrum / max(angleSpectrum)
	#
	# sinAzimHigh = 2 * (angleIdxList[0] + angleOffsetList[0] - aoaCfg.numAngleBins / 2) / aoaCfg.numAngleBins
	# if len(angleIdxList) > 1:
	# 	sinAzimHigh2 = 2 * (angleIdxList[1] + angleOffsetList[1] - aoaCfg.numAngleBins / 2) / aoaCfg.numAngleBins
	# 	if not CONST.HIGH_steeringVec_AOA:
	# 		plt.scatter(angleDomain[angleIdxList[1]] * 180 / np.pi, angleSpectrum[angleIdxList[1]], color='b')
	# 		plt.text(angleDomain[angleIdxList[1]] * 180 / np.pi + 5, angleSpectrum[angleIdxList[1]] - 0.05,
	# 		         "CAPON x : %.2fm" % (rangeDetObj * sinAzimHigh2))
	if CONST.HIGH_steeringVec_AOA:
		pass
		# b = np.arange(-1 * aoaCfg.maxAngleH, aoaCfg.maxAngleH, aoaCfg.deltaTheta)
		# plt.plot(b* 180 / np.pi, angleSpectrum, label=aoaCfg.res_compensation_Type)

	else:
		plt.plot(angleDomain * 180 / np.pi, angleSpectrum, label = aoaCfg.res_compensation_Type)
		plt.scatter(angleDomain[angleIdxList[0]] * 180 / np.pi, angleSpectrum[angleIdxList[0]], color='b')
		plt.text(angleDomain[angleIdxList[0]] * 180 / np.pi + 5, angleSpectrum[angleIdxList[0]] - 0.05,
	         "CAPON x : %.2fm" % (rangeDetObj * sinAzimHigh))


	# xx[0] = xx[0] * np.exp(1j * (3) * np.pi/180)
	# xx[1] = xx[1] * np.exp(1j * (-7) * np.pi / 180)
	# xx[2] = xx[2] * np.exp(1j * (2) * np.pi / 180)
	# xx[3] = xx[3] * np.exp(1j * (-15) * np.pi / 180)
	# #
	# xx[0] = xx[0] * 20.5/19
	# xx[1] = xx[1] * 1
	# xx[2] = xx[2] * 1.2
	# xx[3] = xx[3] * 20.5/19.5

	# xx[4] = xx[4] * np.exp(1j * (3) * np.pi / 180)
	# xx[5] = xx[5] * np.exp(1j * (-7) * np.pi / 180)
	# xx[6] = xx[6] * np.exp(1j * (2) * np.pi / 180)
	# xx[7] = xx[7] * np.exp(1j * (-15) * np.pi / 180)
	#
	# xx[4] = xx[4] * 20.5 / 19
	# xx[5] = xx[5] * 2
	# xx[6] = xx[6] * 1.5
	# xx[7] = xx[7] * 20.5 / 19.5

# 	# xx[0] = xx[0] / abs(xx[0]) * abs(xx[3])
# 	# xx[1] = xx[1] / abs(xx[1]) * abs(xx[3])
# 	# xx[2] = xx[2] / abs(xx[2]) * abs(xx[3])
# 	# xx[3] = xx[3] / abs(xx[3]) * abs(xx[3])
	#ILS_OMP
	# print("input of ILS_OMP : ", xx)
	# plt.clf()
	# plt.plot(np.arctan(xx.imag/xx.real) * 180 / np.pi)
	#
	# plt.show()
	# plt.plot(np.log2(abs(xx.imag)))
	# plt.plot(np.log2(abs(xx.real)))
	# plt.plot(np.log2(abs(xx)))
	# plt.title("abs")
	# plt.show()

	#make distortion




	pRISR, peakList_RISR = DOA.RISR(xx, aoaCfg)
	pILS, peakList_ILS = DOA.ILS_OMP(xx, aoaCfg)
	pFOCUSS, peakList_FOCUSS = DOA.FOCUSS(xx, aoaCfg)
	# pRISR_FFT, peakList_RISR_FFT = DOA.RISR_FFT(xx, aoaCfg, domain)
	# pILS_MMSE, peakList_ILS_MMSE = DOA.ILS_OMP_MMSE(xx, aoaCfg)
	# plt.plot(domain.angleDomainH * 180 / np.pi, pILS_MMSE / max(pILS_MMSE), label="ILS_OMP_MMSE", color = "red", linestyle = "--")
	# plt.scatter(domain.angleDomainH[peakList_ILS_MMSE] * 180 / np.pi, pILS_MMSE[peakList_ILS_MMSE] / max(pILS_MMSE), color="red",
	#             marker="x")
	plt.plot(domain.angleDomainH * 180 / np.pi, pILS / max(pILS), label="ILS_OMP", color = "blue")
	plt.scatter(domain.angleDomainH[peakList_ILS] * 180 / np.pi, pILS[peakList_ILS]/ max(pILS), color="blue")
	plt.plot(domain.angleDomainH * 180 / np.pi, pRISR / max(pRISR), label="RISR", color = "green", linestyle = "-.")
	plt.scatter(domain.angleDomainH[peakList_RISR] * 180 / np.pi, pRISR[peakList_RISR] / max(pRISR), color="green", marker = "x")
	# plt.plot(domain.angleDomainH * 180 / np.pi, pRISR_FFT / max(pRISR_FFT), label="RISR_FFT", color="purple", linestyle=":")
	# plt.scatter(domain.angleDomainH[peakList_RISR_FFT] * 180 / np.pi, pRISR_FFT[peakList_RISR_FFT] / max(pRISR_FFT),
	#             color="purple", marker="x")
	plt.plot(domain.angleDomainH * 180 / np.pi, pFOCUSS / max(pFOCUSS), label="FOCUSS", color = "red", linestyle = "--")
	plt.scatter(domain.angleDomainH[peakList_FOCUSS] * 180 / np.pi, pFOCUSS[peakList_FOCUSS] / max(pFOCUSS), color="red", marker = "v")

	# pSL0 = DOA.SL0(xx,aoaCfg, domain)
	# plt.plot(domain.angleDomainH * 180 / np.pi, pSL0 / max(pSL0), label="SL0")

	plt.legend(loc = 1)
	plt.savefig('{}image/angleSpectrum_{}.png'.format(CONST.dataPath, frameNumber))
	plt.show()

def MmwDemo_rxChanPhaseBiasCompensation(inp, rxChComp):
	
	# inp = inp - np.conj(rxChComp)
	# print(rxChComp)
	inp = inp * rxChComp

	return inp



def AOA_BARTLETT(dataCube_1dFFT, angleDomain, aoaCfg, config):

	pbartlett = np.zeros((config.numADCSamples, angleDomain.shape[0]), np.float)

	for rangeIdx in range(config.numADCSamples):
		xx = dataCube_1dFFT[rangeIdx,:,:]
		xx = np.matrix(xx)
		# print(xx.shape)
		Rxx = xx.T * xx.T.H
		invRxx = np.linalg.inv(Rxx)

		idxP = 0
		for theta in angleDomain:
			a = steeringVector_azimSin(theta, aoaCfg.numAziChannel)

			pbartlett[rangeIdx][idxP] = abs((a.H * Rxx * a) / (a.H * a))

			idxP += 1

	return pbartlett

def AOA_3DFFT(dataCube_1dFFT, angleDomain, aoaCfg, config):

	pmvdr = np.zeros((config.numADCSamples, angleDomain.shape[0]), np.float)

	for rangeIdx in range(config.numADCSamples):
		xx = dataCube_1dFFT[rangeIdx,:,:]
		xx = np.matrix(xx)
		# print(xx.shape)
		Rxx = xx.T * xx.T.H
		invRxx = np.linalg.inv(Rxx)

		idxP = 0
		for theta in angleDomain:
			a = steeringVector_azimSin(theta, aoaCfg.numAziChannel)

			pmvdr[rangeIdx][idxP] = abs(1 / (a.H * invRxx * a))

			idxP += 1

	return pmvdr

def AOA_HIGH_RES(dataCube_1dFFT, angleDomain, aoaCfg, config, rangeDetIdx, procType):
	if CONST.HIGH_steeringVec_AOA:
		b = np.arange(-1 * aoaCfg.maxAngleH, aoaCfg.maxAngleH, aoaCfg.deltaTheta)
		powerSpectrum = np.zeros(len(b), np.float)
	else:
		powerSpectrum = np.zeros(len(angleDomain), np.float)
	xx = dataCube_1dFFT[rangeDetIdx,:,:]
	xx = np.matrix(xx)
	# print(xx.shape)
	if procType == "CAPON":
		Rxx = xx.T * xx.T.H
		invRxx = np.linalg.inv(Rxx)
	elif procType == "MUSIC":
		Rxx = xx.T * xx.T.H
		eigVal, eigVec = np.linalg.eig(Rxx)
		idx = eigVal.argsort()[::-1]
		eigVal = eigVal[idx]
		eigVec = eigVec[:, idx]
		print(abs(eigVal))
		numberOfTarget = aoaCfg.numTarget # number of Target 계산하는 부분 알고리즘 추가 필요!!
		En = eigVec[:, numberOfTarget:]

		En = np.matrix(En)
	elif procType == "BARTLETT":
		Rxx = xx.T * xx.T.H

	idxP = 0
	if CONST.HIGH_steeringVec_AOA:

		for azimSin in b:

			a = steeringVector(azimSin, aoaCfg.numAziChannel)

			if procType == "CAPON":
				powerSpectrum[idxP] = abs(1 / (a.H * invRxx * a))
			elif procType == "MUSIC":
				powerSpectrum[idxP] = abs(1 / (a.H * En * En.H * a))
			elif procType == "BARTLETT":
				powerSpectrum[idxP] = abs((a.H * Rxx * a) / (a.H * a))
			idxP += 1
	else:

		for azimSin in angleDomain:
			if CONST.HIGH_steeringVec_AOA:
				a = steeringVector(0.1*180/np.pi,aoaCfg.numAziChannel)
			else:
				a = steeringVector_azimSin(azimSin, aoaCfg.numAziChannel)
			if procType == "CAPON":
				powerSpectrum[idxP] = abs(1 / (a.H * invRxx * a))
			elif procType == "MUSIC":
				powerSpectrum[idxP] = abs(1 / (a.H * En * En.H * a))
			elif procType == "BARTLETT":
				powerSpectrum[idxP] = abs((a.H * Rxx * a) / (a.H * a))
			idxP += 1

	# print(powerSpectrum)
	return powerSpectrum

def AOA_CAPPON(dataCube_1dFFT, angleDomain, aoaCfg, config):

	pmvdr = np.zeros((config.numADCSamples, angleDomain.shape[0]), np.float)

	for rangeIdx in range(config.numADCSamples):
		xx = dataCube_1dFFT[rangeIdx,:,:]
		xx = np.matrix(xx)
		# print(xx.shape)
		Rxx = xx.T * xx.T.H
		invRxx = np.linalg.inv(Rxx)

		idxP = 0
		for azimSin in angleDomain:
			a = steeringVector_azimSin(azimSin, aoaCfg.numAziChannel)

			pmvdr[rangeIdx][idxP] = abs(1 / (a.H * invRxx * a))

			idxP += 1

	return pmvdr

def AOA_MUSIC(dataCube_1dFFT, angleDomain, aoaCfg, config):

	pmusic = np.zeros((config.numADCSamples, angleDomain.shape[0]), np.float)

	for rangeIdx in range(config.numADCSamples):
		xx = dataCube_1dFFT[rangeIdx,:,:]
		xx = np.matrix(xx)
		# print(xx.shape)
		Rxx = xx.T * xx.T.H
		eigVal, eigVec = np.linalg.eig(Rxx)
		idx = eigVal.argsort()[::-1]
		eigVal = eigVal[idx]
		eigVec = eigVec[:, idx]
		# print(eigVal)
		numberOfTarget = 1
		En = eigVec[:,numberOfTarget:]

		En = np.matrix(En)

		idxP = 0
		for theta in angleDomain:
			a = steeringVector_azimSin(theta, aoaCfg.numAziChannel)

			pmusic[rangeIdx][idxP] = abs(1 / (a.H * En * En.H * a))

			idxP += 1

	return pmusic

def getAssociatonContour(currTrack, distance, td):

	Center = copy.deepcopy(currTrack.stateVectorXYZ)
	Center[0] = Center[0] + td * Center[2]
	Center[1] = Center[1] + td * Center[3]
	Center = Center[:2]
	# print("Center", Center)

	t = np.linspace(0, 2 * np.pi, 20)
	# print("DEBUG", currTrack.cov_mahalanobis)
	invCov = copy.deepcopy(currTrack.cov_mahalanobis)
	# print("debug", invCov)
	# invCov = invCov[:2,:2]
	eigVal, eigVec = np.linalg.eig(invCov)
	# eigVal = distance * np.sqrt(eigVal)
	eigVal = np.sqrt(distance * eigVal)

	xt = eigVal[0] * np.cos(t)
	yt = eigVal[1] * np.sin(t)
	xtyt = np.array([xt, yt])
	x_associationDistance = np.dot(eigVec[0, :], xtyt)
	y_associationDistance = np.dot(eigVec[1, :], xtyt)
	x_associationDistance = x_associationDistance + Center[0]
	y_associationDistance = y_associationDistance + Center[1]

	return x_associationDistance, y_associationDistance,

def getRContour(currTrack, distance):
	t = np.linspace(0, 2 * np.pi, 20)
	centorX = currTrack.associatedObj.measVectorRRD[0] * currTrack.associatedObj.measVectorRRD[2]
	centorY = np.sqrt((currTrack.associatedObj.measVectorRRD[0]) ** 2 - (centorX) ** 2)
	Rmat = np.zeros((N_MEAS, N_MEAS), dtype=np.float)
	for i in range(N_MEAS):
		Rmat[i][i] = currTrack.associatedObj.measCovVec[i]

	invHmat = np.linalg.pinv(currTrack.H_tmp)
	cov_mahal_tmp = np.matmul(invHmat, Rmat)
	cov_mahal_tmp = np.matmul(cov_mahal_tmp, invHmat.T)
	cov_mahalanobis = cov_mahal_tmp[:2, :2]
	invCov = copy.deepcopy(cov_mahalanobis)

	# print("debug", invCov)
	# invCov = invCov[:2,:2]
	eigVal, eigVec = np.linalg.eig(invCov)
	# eigVal = np.sqrt(distance *eigVal)
	eigVal = np.sqrt(distance * eigVal)

	xt = eigVal[0] * np.cos(t)
	yt = eigVal[1] * np.sin(t)
	xtyt = np.array([xt, yt])
	x = np.dot(eigVec[0, :], xtyt)
	y = np.dot(eigVec[1, :], xtyt)
	x = x + centorX
	y = y + centorY

	return x, y

def getPContour(currTrack, distance):
	t = np.linspace(0, 2 * np.pi, 20)
	invCov = copy.deepcopy(currTrack.prevCovariance[:2, :2])

	# print("debug", invCov)
	# invCov = invCov[:2,:2]
	eigVal, eigVec = np.linalg.eig(invCov)
	# eigVal = np.sqrt(distance *eigVal)
	eigVal = np.sqrt(distance * eigVal)

	xt = eigVal[0] * np.cos(t)
	yt = eigVal[1] * np.sin(t)
	xtyt = np.array([xt, yt])
	x_prev = np.dot(eigVec[0, :], xtyt)
	y_prev = np.dot(eigVec[1, :], xtyt)
	x_prev = x_prev + currTrack.prevStateVectorXYZ[0] #P(k|k-1)
	y_prev = y_prev + currTrack.prevStateVectorXYZ[1] #P(k|k-1)

	invCov = copy.deepcopy(currTrack.Covariance[:2, :2])

	# print("debug", invCov)
	# invCov = invCov[:2,:2]
	eigVal, eigVec = np.linalg.eig(invCov)
	# eigVal = np.sqrt(distance *eigVal)
	eigVal = np.sqrt(distance * eigVal)

	xt = eigVal[0] * np.cos(t)
	yt = eigVal[1] * np.sin(t)
	xtyt = np.array([xt, yt])
	x_curr = np.dot(eigVec[0, :], xtyt)
	y_curr = np.dot(eigVec[1, :], xtyt)
	x_curr = x_curr + currTrack.stateVectorXYZ[0] #P(k|k)
	y_curr = y_curr + currTrack.stateVectorXYZ[1] #P(k|k)

	return x_prev, y_prev, x_curr,y_curr

def dopplerAliasingComp(inp, config):
	_inp = np.zeros(inp.shape, dtype=np.complex64)

	for numTxIdx in range(config.numTx):
		for numRxIdx in range(config.numRx):
			if not CONST.reverseAntenna:
				_inp[config.numRx * numTxIdx + numRxIdx] = inp[config.numRx * numTxIdx + numRxIdx] * np.exp(-1j * np.pi * numTxIdx)

			else:
				_inp[config.numRx * numTxIdx + numRxIdx] = inp[config.numRx * numTxIdx + numRxIdx] * np.exp(-1j * np.pi * (1 - numTxIdx))
			# _inp[config.numRx * numTxIdx + numRxIdx] = inp[config.numRx * numTxIdx + numRxIdx] * np.exp(
			# 	1j * np.pi * (numTxIdx - 1))


	return _inp

# def AOA_Slow(data, config, cfarOut2DList, cfarOut2DList_len, cfarOut3DList, aoaCfg, domain, DEBUG = False):

def rearrangeByCRT(cfarOut2DList):
	for obj in cfarOut2DList[:]:
		if (obj.CRT_Flag == 1 and obj.CRT_Flag_als == 1) and CONST.CRT_OVERLAP_append:
			cfarOut2DList.append(copy.deepcopy(obj))

			obj.CRT_Flag = 3
			obj.CRT_Flag_als = 0

			cfarOut2DList[-1].CRT_Flag = 0
			cfarOut2DList[-1].CRT_Flag_als = 3

		elif (obj.CRT_Flag == 2 and obj.CRT_Flag_als == 0)and CONST.CRT_OVERLAP_append:
			cfarOut2DList.append(copy.deepcopy(obj))

			obj.CRT_Flag = 3
			obj.CRT_Flag_als = 0

			cfarOut2DList[-1].CRT_Flag = 3
			cfarOut2DList[-1].CRT_Flag_als = 0

			cfarOut2DList[-1].CRT_slow_doppler_indx = obj.CRT_slow_doppler_indx_2
			cfarOut2DList[-1].velDisambFacValidity = obj.velDisambFacValidity_2

		elif (obj.CRT_Flag == 0 and obj.CRT_Flag_als == 2) and CONST.CRT_OVERLAP_append:
			cfarOut2DList.append(copy.deepcopy(obj))

			obj.CRT_Flag = 0
			obj.CRT_Flag_als = 3

			cfarOut2DList[-1].CRT_Flag = 0
			cfarOut2DList[-1].CRT_Flag_als = 3

			cfarOut2DList[-1].CRT_slow_doppler_indx_als = obj.CRT_slow_doppler_indx_als_2
			cfarOut2DList[-1].velDisambFacValidity_als = obj.velDisambFacValidity_als_2

		elif (obj.CRT_Flag == 1 and obj.CRT_Flag_als == 0) and CONST.choose_by_CRT:

			obj.CRT_Flag = 3
			obj.CRT_Flag_als = 0


		elif (obj.CRT_Flag == 0 and obj.CRT_Flag_als == 1) and CONST.choose_by_CRT:
			obj.CRT_Flag = 0
			obj.CRT_Flag_als = 3

def phaseShift(inp, degree):
	tempRxChComp = np.zeros(len(inp), dtype=np.complex64)
	degreeInRAD = degree * np.pi / 180
	for idx, val in enumerate(tempRxChComp):
		tempRxChComp[idx] = np.exp(1j*np.pi * np.sin(degreeInRAD) * idx)

	inp = inp * tempRxChComp

	return inp
def AOA(data, config, cfarOut2DList, aoaCfg, domain, procFFT=True, DEBUG = False):
	cfarOut3DList = []

	angleDomain = domain.angleDomain
	# print(angleDomain)
	azimuthIn = np.zeros(aoaCfg.numAngleBins, dtype=np.complex64)

	if CONST.TDM_MIMO:
		azimuthIn_als = np.zeros(aoaCfg.numAngleBins, dtype=np.complex64)

	p = np.zeros(angleDomain.shape[0], dtype=np.complex64)
	# print(angleDomain.shape[0])
	for outObj in cfarOut2DList:
		isAliasing = False
		isSlowProc = 0
		detObjRangeIdx = outObj.rangeIdx
		detObjDopplerIdx = outObj.dopplerIdx

		if (outObj.CRT_Flag == 3) and CONST.CRT_OVERLAP:
			if DEBUG and outObj.detID == CONST.targetIdxPlotSpec:
				print("AOA input is from the slow")
				if False:

					fig, ax = plt.subplots()

					c = ax.pcolormesh(domain.dopplerDomain_Slow, domain.rangeDomain, data.heatMap_Slow)

					ax.set_title('1 / ' + 'cfarType = ')
					fig.colorbar(c, ax=ax)
					plt.axvline(
						x=domain.dopplerDomain_Slow[outObj.CRT_slow_doppler_indx],
						color='orange', linestyle=':')
					plt.axhline(
						y=domain.rangeDomain[detObjRangeIdx],
						color='orange', linestyle=':')
					# plt.scatter(dopplerDomain[dopplerIdxList], rangeDomain[rangeIdxList], facecolors="none", edgecolors='r', s=10)

					plt.show()
			if CONST.choose_by_CRT_slow:
				xx = data.FFTData_Slow[detObjRangeIdx, outObj.CRT_slow_doppler_indx, :]
			else:
				xx = data.FFTData[detObjRangeIdx, detObjDopplerIdx, :]
			isAliasing = False
			isSlowProc = 1
		elif (outObj.CRT_Flag_als == 3) and CONST.CRT_OVERLAP:
			if DEBUG and outObj.detID == CONST.targetIdxPlotSpec:
				if False:

					fig, ax = plt.subplots()

					c = ax.pcolormesh(domain.dopplerDomain_Slow, domain.rangeDomain, data.heatMap_Slow)

					ax.set_title('2 / ' + 'cfarType = ')
					fig.colorbar(c, ax=ax)
					plt.axvline(
						x=domain.dopplerDomain_Slow[outObj.CRT_slow_doppler_indx_als],
						color='orange', linestyle=':')
					plt.axhline(
						y=domain.rangeDomain[detObjRangeIdx],
						color='orange', linestyle=':')
					# plt.scatter(dopplerDomain[dopplerIdxList], rangeDomain[rangeIdxList], facecolors="none", edgecolors='r', s=10)

					plt.show()
				print("AOA input is from the slow")
			if CONST.choose_by_CRT_slow:
				xx = data.FFTData_Slow[detObjRangeIdx, outObj.CRT_slow_doppler_indx, :]
			else:
				xx = data.FFTData[detObjRangeIdx, detObjDopplerIdx, :]
			isAliasing = True
			isSlowProc = 1
		else:

			if outObj.isSlow and CONST.SLOW_CFAR:
				xx = data.FFTData_Slow[detObjRangeIdx, detObjDopplerIdx, :]
			else:

				xx = data.FFTData[detObjRangeIdx, detObjDopplerIdx, :]

		if DEBUG and outObj.detID == CONST.targetIdxPlotSpec:
			xx_bfr = copy.deepcopy(xx)

		if CONST.TDM_MIMO:
			# print("dopplerCompensation is implemented")
			xx = dopplerCompensation(xx, detObjDopplerIdx, config, outObj.isSlow)

		if (aoaCfg.AWR1843 == True) and (aoaCfg.Mode == 'MIMO'):
			xx_ = np.zeros(8, dtype=np.complex64)
			for i in range(0, 4):
				xx_[i] = xx[i]
				xx_[i + 4] = xx[i + 8]
			xx = xx_

		# onlt for MRR, You need another rxChComp Value for USRR.
		if CONST.DO_RX_COMPENSATION:
			# rxChComp = np.array([19158 + 5863j, 23451 + 2858j, 22093 + 4569j, 16399], dtype = np.complex64)
			rxChComp = aoaCfg.rxChComp
			xx = MmwDemo_rxChanPhaseBiasCompensation(xx, rxChComp)

		if CONST.reverseAntenna:
			xx = np.flip(xx)
			if DEBUG and outObj.detID == CONST.targetIdxPlotSpec:
				xx_bfr = np.flip(xx_bfr)

#		xx = phaseShift(xx, 45)
		if CONST.TDM_MIMO:

			xx_als = dopplerAliasingComp(xx, config)


		if DEBUG and outObj.detID == CONST.targetIdxPlotSpec:

			print("AOA input : \n", xx)
			print("CRT_Flag : %d, CRT_Flag_als : %d"%(outObj.CRT_Flag, outObj.CRT_Flag_als))
			plt.subplot(211)
			plt.plot(abs(xx_bfr), label='abs before')
			plt.plot(abs(xx), label='abs after')
			if CONST.TDM_MIMO:
				plt.plot(abs(xx_als), label='abs xx_als')
			plt.title("AOA input(abs)")
			plt.grid(linestyle=":")
			plt.legend(loc=1)

			plt.subplot(212)
			plt.plot(np.arctan(xx_bfr.imag / xx_bfr.real), label='theta before')
			plt.plot(np.arctan(xx.imag / xx.real), label='theta after')
			if CONST.TDM_MIMO:
				plt.plot(np.arctan(xx_als.imag / xx_als.real), label='theta xx_als')
			plt.title("AOA input(theta)")
			plt.grid(linestyle=":")
			plt.legend(loc=1)

			plt.show()


		# Obtain Covariance Matrix
		if procFFT != True:
			xx = np.matrix(xx).T

			Rxx = xx * xx.H

			idxP = 0

			for theta in angleDomain:
				a = steeringVector(theta, aoaCfg.numAziChannel)

				p[idxP] = (a.H * Rxx * a) / (a.H * a)

				idxP += 1

		elif CONST.RISR:
			_, angleIdxList = DOA.RISR(xx, aoaCfg)
			angleValList = []
			angleSNRList = []
			angleOffsetList = []
			for i in angleIdxList:
				angleValList.append(20)
				angleSNRList.append(20)
				angleOffsetList.append(0)

		elif CONST.ILS_OMP:
			_, angleIdxList = DOA.ILS_OMP(xx, aoaCfg)
			angleValList = []
			angleSNRList = []
			angleOffsetList = []
			for i in angleIdxList:
				angleValList.append(20)
				angleSNRList.append(20)
				angleOffsetList.append(0)

		elif CONST.ILS_OMP_MMSE:

			_, angleIdxList = DOA.ILS_OMP_MMSE(xx, aoaCfg)
			angleValList = []
			angleSNRList = []
			angleOffsetList = []
			for i in angleIdxList:
				angleValList.append(20)
				angleSNRList.append(20)
				angleOffsetList.append(0)

		elif CONST.FOCUSS:

			_, angleIdxList = DOA.FOCUSS(xx, aoaCfg)
			angleValList = []
			angleSNRList = []
			angleOffsetList = []
			for i in angleIdxList:
				angleValList.append(20)
				angleSNRList.append(20)
				angleOffsetList.append(0)
		else:

			idx = 0
			for val in xx:
				azimuthIn[idx] = val / 8
				idx += 1
			# print(azimuthIn)
			tmp3DFFT = np.fft.fft(azimuthIn)
			tmp3DFFT = np.fft.fftshift(tmp3DFFT, axes=(0,))

			p = abs(tmp3DFFT)

			if CONST.TDM_MIMO:
				idx = 0
				for val in xx_als:
					azimuthIn_als[idx] = val / 8
					idx += 1
				tmp3DFFT_als = np.fft.fft(azimuthIn_als)
				tmp3DFFT_als = np.fft.fftshift(tmp3DFFT_als, axes=(0,))
				p_als = abs(tmp3DFFT_als)
				p_org = copy.deepcopy(p)

				if CONST.CRT_OVERLAP:
					if outObj.CRT_Flag == 3:
						maxVal = max(p)
						isAliasing = False

					elif outObj.CRT_Flag_als == 3:
						maxVal = max(p_als)
						p = p_als
						isAliasing = True

					else:
						if max(p) < max(p_als):
							maxVal = max(p_als)
							p = p_als
							isAliasing = True
						else:
							maxVal = max(p)
				else:

					if max(p) < max(p_als):
						maxVal = max(p_als)
						p = p_als
						isAliasing = True
					else:
						maxVal = max(p)

		if DEBUG and True and outObj.detID == CONST.targetIdxPlotSpec:
			if isAliasing:
				xx_CS = xx_als
			else:
				xx_CS = xx

			pILS, peakList_ILS = DOA.ILS_OMP(xx, aoaCfg)
			pRISR, peakList_RISR = DOA.RISR(xx, aoaCfg)
			pFOCUSS, peakList_FOCUSS = DOA.FOCUSS(xx, aoaCfg)
			if outObj.isSlow:
				plt.title("Angle Spectrum_slow")
			else:
				plt.title("Angle Spectrum_fast")
			if False:
				plt.plot(domain.angleDomainH * 180 / np.pi, pILS / max(pILS), label="ILS_OMP", color="blue")
				plt.scatter(domain.angleDomainH[peakList_ILS] * 180 / np.pi, pILS[peakList_ILS] / max(pILS), color="blue")
				plt.plot(domain.angleDomainH * 180 / np.pi, pRISR / max(pRISR), label="RISR", color="green", linestyle="-.")
				plt.scatter(domain.angleDomainH[peakList_RISR] * 180 / np.pi, pRISR[peakList_RISR] / max(pRISR),
							color="green", marker="x")
				plt.plot(domain.angleDomainH * 180 / np.pi, pFOCUSS / max(pFOCUSS), label="FOCUSS", color="red", linestyle="-.")
				plt.scatter(domain.angleDomainH[peakList_FOCUSS] * 180 / np.pi, pFOCUSS[peakList_FOCUSS] / max(pFOCUSS),
							color="red", marker="D")

		if not (CONST.ILS_OMP or CONST.RISR or CONST.ILS_OMP_MMSE or CONST.FOCUSS):
			angleIdxList, angleValList = peakDetection(p, aoaCfg, multi=aoaCfg.multiPeakDet)
			# plt.plot(pbartlett)
			# plt.show()
			angleSNRList = computeSinAzimSNR(p, angleIdxList, config.numTx, config.numRx, aoaCfg.numAngleBins)
			angleSNRList = np.log2(angleSNRList)

			# angleOffsetList = [0]
			angleOffsetList = quadraticInterp(p, angleIdxList)
			# angleOffsetList = sin2theta(angleOffsetList) #sin2radian

		if DEBUG and outObj.detID == CONST.targetIdxPlotSpec:
			if CONST.TDM_MIMO:
				plt.plot(angleDomain * 180 / np.pi, p_org / maxVal, label="3Dfft_org", color='gray', linestyle="-")
				plt.plot(angleDomain * 180 / np.pi, p_als / maxVal, label="3Dfft_alias", color='gray', linestyle=":")
			else:
				plt.plot(angleDomain * 180 / np.pi, p / max(p), label="3Dfft", color='gray', linestyle="-")

			plt.scatter(angleDomain[angleIdxList] * 180 / np.pi, p[angleIdxList] / max(p), color='gray')
			for i in range(len(angleIdxList)):
				plt.text(angleDomain[angleIdxList[i]] * 180 / np.pi + 5, p[angleIdxList[i]] / max(p),
				         "angle : %.2f DEG, angleSNR : %.2fdB, angleOffset : %.2f" % (
				         angleDomain[angleIdxList[i]] * 180 / np.pi, angleSNRList[i] * 6, angleOffsetList[i]))
			# plt.text(angleDomain[angleIdxList[i]]*180/np.pi+5, pbartlett[angleIdxList[i]], "angle : %.2f DEG, angleSNR : %.2fdB"%(angleDomain[angleIdxList[i]]*180/np.pi, angleSNRList[i]*6))

			plt.title("rangeBin : %d, range: %.2f" % (outObj.rangeIdx, domain.rangeDomain[outObj.rangeIdx]))
			plt.xticks([i for i in range(-90, 90, 5)])
			plt.grid(linestyle=":")
			plt.legend(loc=1)
			plt.show()

		tmpIdx = 0
		for angleDetIdx in angleIdxList:
			tmpAoaOut = DT.cfarOutFmt3D()
			tmpAoaOut.detID = outObj.detID
			tmpAoaOut.rangeIdx = outObj.rangeIdx
			tmpAoaOut.dopplerIdx = outObj.dopplerIdx
			tmpAoaOut.rangeVal = outObj.rangeVal
			tmpAoaOut.dopplerVal = outObj.dopplerVal
			tmpAoaOut.rangeSNR = outObj.rangeSNR
			tmpAoaOut.dopplerSNR = outObj.dopplerSNR
			tmpAoaOut.angleIdx = angleDetIdx
			tmpAoaOut.angleVal = angleValList[tmpIdx]
			tmpAoaOut.angleSNR = angleSNRList[tmpIdx]
			tmpAoaOut.angleOffset = angleOffsetList[tmpIdx]
			tmpAoaOut.isSlowProc = isSlowProc
			tmpAoaOut.isSlow = outObj.isSlow
			if CONST.TDM_MIMO:
				tmpAoaOut.isAliasing = isAliasing
				# print("isAliasing", isAliasing)
			tmpAoaOut.velDisambFacValidity = outObj.velDisambFacValidity
			tmpAoaOut.velDisambFacValidity_als = outObj.velDisambFacValidity_als
			tmpAoaOut.dopplerOffset = outObj.dopplerOffset
			cfarOut3DList.append(tmpAoaOut)

			tmpIdx += 1


	return cfarOut3DList

def AOA_Compensation(cfarOut3DList, domain, dataCube_1dFFT, aoaCfg, config):
	angleDomain = domain.angleDomain

	for detObj in cfarOut3DList:

		angleSpectrum = AOA_HIGH_RES(dataCube_1dFFT, angleDomain, aoaCfg, config, detObj.rangeIdx, aoaCfg.res_compensation_Type)
		angleIdxList, angleValList = peakDetection(angleSpectrum, aoaCfg, multi=aoaCfg.multiPeakDet)
		angleOffsetList = quadraticInterp(angleSpectrum, angleIdxList)

		rangeDetObj = domain.rangeDomain[detObj.rangeIdx] -0.075
		sinAzimOrg  = 2*(detObj.angleIdx + detObj.angleOffset - aoaCfg.numAngleBins/2) / aoaCfg.numAngleBins
		sinAzimHigh = 2*(angleIdxList[0] + angleOffsetList[0] - aoaCfg.numAngleBins/2) / aoaCfg.numAngleBins
		condition = abs(rangeDetObj * sinAzimOrg - rangeDetObj * sinAzimHigh) < 1
		if len(angleIdxList) > 1:
			sinAzimHigh2 = 2 * (angleIdxList[1] + angleOffsetList[1] - aoaCfg.numAngleBins / 2) / aoaCfg.numAngleBins
			if abs(sinAzimHigh2 - sinAzimOrg) < abs(sinAzimHigh - sinAzimOrg):
				condition = abs(rangeDetObj * sinAzimOrg - rangeDetObj * sinAzimHigh2) < 1

		if condition:
			detObj.angleIdx = angleIdxList[0]
			detObj.angleOffset = angleOffsetList[0]

def anotherPeakDet(inputSpect, refIdx):
	firstMaxVal = inputSpect[refIdx]
	inp = np.copy(inputSpect)
	inpLen = inp.shape[0]
	# print(inpLen)
	inp[refIdx] = 0
	rightValley = (refIdx + 1) % inpLen
	leftValley = (refIdx - 1) % inpLen
	# print("start loop, inpLen : %d, refIdx : %d, rightValley : %d"%(inpLen, refIdx, rightValley))
	
	while True:

		tmp = (rightValley + 1) % inpLen
		# print("rightValley : %d" %(rightValley))
		# print(rightValley)
		if inp[rightValley] <= inp[tmp]:
			inp[rightValley] = 0
			break
		else : 
			inp[rightValley] = 0
			rightValley = tmp

	while True:
		
		tmp = (leftValley - 1) % inpLen
		# print("leftValley : %d" %(leftValley))
		
		if inp[leftValley] <= inp[tmp]:
			inp[leftValley] = 0
			break
		else : 
			inp[leftValley] = 0
			leftValley = tmp		

	secondMax = np.argmax(inp)

	# if inp[secondMax]<0.85*firstMaxVal:
		# secondMax = None
	# if rightValley > leftValley:
	# 	inp = np.delete(inp, np.arange(leftValley,rightValley+1))
	# 	print(inp)
	# 	secondMax = np.argmax(inp)
	# else:
	# 	inp = inp[rightValley:leftValley+1]
	# 	# print(inp.shape)
	# 	secondMax = np.argmax(inp)

	return inp, secondMax

def peakDetection(inp, aoaCfg, multi = True):

	# plt.plot(inp)
	# plt.show()

	angleIdx = []
	angleVal = []
	angleSNR = []

	inpLen = inp.shape[0]

	inp = abs(inp)

	firstMax = np.argmax(inp)
	firstMaxValue = inp[firstMax]
	# firstMaxSNR = inp[firstMax]
	angleIdx.append(firstMax)
	angleVal.append(firstMaxValue)
	# angleSNR.append(angleSNR)

	if multi == True:
		_,secondMax = anotherPeakDet(inp, firstMax)
		secondMaxValue = inp[secondMax]
	# print("hi")
		if secondMaxValue > firstMaxValue * aoaCfg.multiPeakScale:
			angleIdx.append(secondMax)
			angleVal.append(secondMaxValue)
			# print("hi")
			# print("firstMax : %d, secondMax : %d"%(firstMax, secondMax))

	return angleIdx, angleVal

def transCoord(cfarOut3DList, domain, resList, n_chirps, aoaCfg):

	for obj in cfarOut3DList:
		if obj.isSlow:
			dopplerRes = resList[3]
		else:
			dopplerRes = resList[1]

		if CONST.transCoordDEBUG:
			print("obj.dopplerIdx : %d, obj.velDisambFacValidity : %d"%(obj.dopplerIdx, obj.velDisambFacValidity))
			print("obj.velDisambFacValidity_als : %d" % (obj.velDisambFacValidity_als))
			print("obj.isAliasing",obj.isAliasing)
		obj.range = domain.rangeDomain[obj.rangeIdx] - 0.075            #MIN_RANGE_OFFSET_METERS = 0.075

		if CONST.TDM_MIMO:

			dopplerIdx = obj.dopplerIdx
			if (dopplerIdx != 0) and (dopplerIdx != 16):								# HSLee 수정
				if CONST.DOPPLER_INTERPOLATION:											# HSLee 수정
					dopplerIdx += obj.dopplerOffset										# HSLee 수정
			if obj.isAliasing:
				if dopplerIdx < n_chirps / 2 and dopplerIdx >= 0:						# HSLee 수정
					dopplerIdx += n_chirps
				else:
					dopplerIdx -= n_chirps
				obj.speed = (dopplerIdx - n_chirps / 2 + n_chirps * 2 * (
						obj.velDisambFacValidity_als - 1)) * dopplerRes
			else:
				obj.speed = (dopplerIdx - n_chirps / 2 + n_chirps * 2 * (
						obj.velDisambFacValidity - 1)) * dopplerRes

		else:
			obj.speed = (obj.dopplerIdx - n_chirps / 2 + n_chirps * (
				obj.velDisambFacValidity - 1)) * dopplerRes
		# print(obj.angleIdx)
		# 
		if aoaCfg.procFFT == True:
			# print(obj.angleIdx,obj.angleOffset,aoaCfg.numAngleBins)
			# obj.theta = np.arcsin(2*(obj.angleIdx + obj.angleOffset - aoaCfg.numAngleBins/2) / aoaCfg.numAngleBins)
			if CONST.ILS_OMP or CONST.RISR or CONST.ILS_OMP_MMSE or CONST.FOCUSS:
				obj.sinAzim = np.sin(domain.angleDomainH[obj.angleIdx])


			else:

				obj.sinAzim = 2*(obj.angleIdx + obj.angleOffset - aoaCfg.numAngleBins/2) / aoaCfg.numAngleBins
				# sinAzim offset추가되는 바람에 90도 넘어가는 현상 발생하여 아래 부분 추가
				if obj.sinAzim > 1:
					obj.sinAzim = 1
				elif obj.sinAzim < -1:
					obj.sinAzim = -1

		else:
			obj.theta = domain.angleDomain[obj.angleIdx]
			# print("obj.theta : %.2f"%obj.theta)
		obj.x = obj.range * obj.sinAzim
		# print("objRamge, obj.x, obj.sinAzim",obj.range,obj.x,obj.sinAzim)
		obj.y = np.sqrt(obj.range*obj.range - obj.x*obj.x)
		# print("obj : %f , %f, %f, %f"%(obj.y,obj.range*obj.range - obj.x*obj.x, obj.x, obj.range))
		if obj.y < 0:
			obj.y = 0

		### Original Code
#		obj.xd = 0
#		if CONST.SPEED_COMP_BY_ANGLE:
#			if obj.sinAzim == 1 or obj.sinAzim == -1:
#				obj.yd = obj.speed
#			else:
#				obj.yd = obj.speed / np.sqrt(1 - obj.sinAzim * obj.sinAzim)
#		else:
#			obj.yd = obj.speed
		### Original Code
		
		### HSLee 수정 2022.06.17
		if CONST.SPEED_COMP_BY_ANGLE:
			obj.xd = obj.speed * obj.sinAzim
			obj.yd = obj.speed * np.sqrt(1 - obj.sinAzim * obj.sinAzim)
		else:
			obj.xd = 0
			obj.yd = obj.speed
		### HSLee 수정 2022.06.17

def computePredictedMeas(stateXYZ):
	tmpStateRRD = np.zeros(N_MEAS, dtype = np.float)
	
	tmpStateRRD[0] = stateXYZ[0]*stateXYZ[0] + stateXYZ[1]*stateXYZ[1]
	if tmpStateRRD[0] < 0.005:
		return tmpStateRRD, False
	tmpStateRRD[0] = np.sqrt(tmpStateRRD[0])
	tmpStateRRD[1] = (stateXYZ[0]*stateXYZ[2] + stateXYZ[1]*stateXYZ[3])/tmpStateRRD[0]
	tmpStateRRD[2] = stateXYZ[0] / tmpStateRRD[0]
	if CONST.ADD_ANGLE_VEL:
		tmpStateRRD[3] = (stateXYZ[2] * stateXYZ[1] * stateXYZ[1] + stateXYZ[0] * stateXYZ[1] * stateXYZ[3])\
	                 /(tmpStateRRD[0] ** 3)
	return tmpStateRRD, True

def pinvHmat(currTrack, ResidCovmat_, Hmat):

	invHmat = np.linalg.pinv(Hmat)
	cov_mahal_tmp = np.matmul(invHmat, ResidCovmat_)
	cov_mahal_tmp = np.matmul(cov_mahal_tmp, invHmat.T)
	currTrack.cov_mahalanobis = cov_mahal_tmp[:2, :2]

def cal_invCovMat(currTrack, Hmat):

	tmp = copy.deepcopy(currTrack.Covariance)
	# A = np.linalg.inv(tmp)
	# Rmat = np.zeros((N_MEAS, N_MEAS), dtype=np.float)
	# for i in range(N_MEAS):
	# 	Rmat[i][i] = currTrack.measCovVec[i]
	invHmat = np.linalg.pinv(Hmat)
	# print(invHmat)
	tmp = np.matmul(Hmat, tmp)
	# print(Hmat, "\n",Hmat.T)
	tmp = np.matmul(tmp, Hmat.T)


	# tmp = tmp + Rmat

	# tmp = tmp[:2,:2]
	# print("debug+tmp", tmp)
	invCov = np.linalg.inv(tmp)
	# tmp = tmp[:2,:2]
	# print("debug_invCov",invCov)

	cov_mahal_tmp = np.matmul(invHmat, invCov)
	cov_mahal_tmp = np.matmul(cov_mahal_tmp, invHmat.T)
	# print(A,"\n",cov_mahal_tmp)
	# currTrack.cov_mahalanobis = cov_mahal_tmp[:2,:2]
	# print("debug2", currTrack.Covariance)
	isCovmatInversible = np.allclose(np.matmul(tmp, invCov), np.eye(3))
	#
	# for i in range(4):
	# 	for j in range(4):
	# 		if i > 1 or j > 1:
	# 			invCov[i][j] = 0



	return invCov, isCovmatInversible

def isTargetWithinDataAssociationThresh_mahalanobis(currMeas, tmpPredictedRRD, tmpPredictedXYZ, currTrack, trackingCfg, invCov):
	#dataType (clusteringOut,trackerElem)
	rrdCurrMeas = currMeas.measVectorRRD
	xyzCurrMeas = currMeas.stateVectorXYZ

	# if CONST.isTargetWithinDataAssociationThreshDEBUG:
	# 	print("meas_rrd\n" , rrdCurrMeas)
	# 	print("state_rrd\n", tmpPredictedRRD)

	rrdEstimationError = rrdCurrMeas - tmpPredictedRRD



	trigger = False
	pdistSq = 655350.0

	# if invCov[0][0] > 1:
	# 	invCov[0][0] = 1
	# if invCov[0][0] < 1/4:
	# 	invCov[0][0] = 1/4
	# if invCov[1][1]> 1:
	# 	invCov[1][1] = 1
	# if invCov[1][1]< 1/4:
	# 	invCov[1][1] = 1/4
	if CONST.ekfDEBUG and currTrack.ID == CONST.RCS_DEBUG_ID_NUMBER:
		print("invCov")
		print(invCov)
		print("\n")
	tmp = np.dot(rrdEstimationError, invCov)

	mahalanobisDistance = np.dot(tmp, rrdEstimationError)


	if mahalanobisDistance < trackingCfg.associateGamma:
		pdistSq = mahalanobisDistance
		trigger = True
		return pdistSq, trigger

		if CONST.isTargetWithinDataAssociationThreshDEBUG and currTrack.ID == CONST.RCS_DEBUG_ID_NUMBER:
			print(
				"the current track (ID, x, y, xd, xy) = (%d, %.2f, %.2f, %.2f, %.2f) is matched with the measurement point(x, y, xd, xy) = (%.2f, %.2f, %.2f, %.2f): mahalanobisDistance = %f, gamma = %f, abs(rrdEstimationError[1]) = %f, modifier = %f, velAssocThresh = %f."
				% (currTrack.ID, currTrack.stateVectorXYZ[0], currTrack.stateVectorXYZ[1],
				   currTrack.stateVectorXYZ[2], currTrack.stateVectorXYZ[3],
				   xyzCurrMeas[0], xyzCurrMeas[1], xyzCurrMeas[2],
				   xyzCurrMeas[3],mahalanobisDistance, trackingCfg.associateGamma, abs(rrdEstimationError[1]), modifier, trackingCfg.velAssocThresh))
	else:
		if CONST.isTargetWithinDataAssociationThreshDEBUG and currTrack.ID == CONST.RCS_DEBUG_ID_NUMBER:
			print(
				"the current track (ID, x, y, xd, xy) = (%d, %.2f, %.2f, %.2f, %.2f) is not matched with the measurement point(x, y, xd, xy) = (%.2f, %.2f, %.2f, %.2f).\n"
				"because mahalanobisDistance : %f is bigger than associateGamma : %f or velError : %f is bigger than modiier(:%f)*velAssocThresh(:%f)."

				% (currTrack.ID, currTrack.stateVectorXYZ[0], currTrack.stateVectorXYZ[1],
				   currTrack.stateVectorXYZ[2], currTrack.stateVectorXYZ[3],
				   xyzCurrMeas[0], xyzCurrMeas[1], xyzCurrMeas[2],
				   xyzCurrMeas[3], mahalanobisDistance, trackingCfg.associateGamma, abs(rrdEstimationError[1]), modifier, trackingCfg.velAssocThresh))
	return 99999999, trigger

def isTargetWithinDataAssociationThresh(currMeas, tmpPredictedRRD, tmpPredictedXYZ, currTrack, trackingCfg):
	#dataType (clusteringOut,trackerElem)
	rrdCurrMeas = currMeas.measVectorRRD
	xyzCurrMeas = currMeas.stateVectorXYZ

	# if CONST.isTargetWithinDataAssociationThreshDEBUG:
	# 	print("meas_rrd\n" , rrdCurrMeas)
	# 	print("state_rrd\n", tmpPredictedRRD)

	rrdEstimationError = rrdCurrMeas - tmpPredictedRRD
	xyzEstimationError = xyzCurrMeas - tmpPredictedXYZ
	modifier = 1.0

	trigger = False
	pdistSq = 655350.0

	if CONST.MODIFIER_MODIFY:
		if currTrack.tick < 10:
			modifier = 2
		elif tmpPredictedRRD[0] < 20:
			modifier = 1.5
		else:
			modifier = 1.0
		tmpX = tmpPredictedRRD[0] * tmpPredictedRRD[2]
		# if tmpX < 2 and tmpX > 2:
		# 	angleModifier = 1
		# 	modifier = 1
		# else:
		# 	angleModifier = 1

		if ((currTrack.age > 0) and (currTrack.tick > 10)):  # /* If the object hasn't been associated in the previous tick, search in a wider range. */
			modifier = modifier * 2.0
	elif CONST.proportionRange_CL_DA:
		if ((currTrack.age > 0)):  # /* If the object hasn't been associated in the previous tick, search in a wider range. */
			if tmpPredictedRRD[0] > 100:
				modifier = modifier * 4.0
			else:
				modifier = modifier * 2.0
	# 	if currTrack.stateVectorXYZ[1] > 4.0:
	# 		modifier = currTrack.stateVectorXYZ[1] / 4.0

	# else:
	# 	if currTrack.tick < 10:
	# 		modifier = 4
	# 	elif tmpPredictedRRD[0] < 20:
	# 		modifier = 1.5
	# 	else:
	# 		modifier = 1.0
	#
	# 	if ((currTrack.age > 0) and (currTrack.tick > 10)): #/* If the object hasn't been associated in the previous tick, search in a wider range. */
	# 		modifier = modifier * 2.0

	# modifier = 1.0
	if CONST.proportionRange_CL_DA:
		a = abs(rrdEstimationError[0]) < (trackingCfg.rangeAssocThresh)
		b = abs(rrdEstimationError[1]) < (trackingCfg.velAssocThresh)
		c = abs(rrdEstimationError[2]) < (trackingCfg.azimAssocThresh)

	else:

		a = abs(rrdEstimationError[0]) < (modifier * trackingCfg.rangeAssocThresh)
		b = abs(rrdEstimationError[1]) < (modifier * trackingCfg.velAssocThresh)
		if CONST.MODIFIER_MODIFY:
			c = abs(rrdEstimationError[2]) < (modifier * trackingCfg.azimAssocThresh)
			# c = abs(rrdEstimationError[2]) < (modifier * angleModifier *trackingCfg.azimAssocThresh)
		else:
			c = abs(rrdEstimationError[2]) < (modifier *trackingCfg.azimAssocThresh)

	# if CONST.isTargetWithinDataAssociationThreshDEBUG:
	# 	print("rrdEstimationError: ", rrdEstimationError)
	# 	print("Tresh : %f, %f, %f" % (
	# 	(modifier * trackingCfg.rangeAssocThresh), (modifier * trackingCfg.velAssocThresh),
	# 	(modifier * trackingCfg.azimAssocThresh)))
	if a and b and c:
		distSq = xyzEstimationError[0] * xyzEstimationError[0] + xyzEstimationError[1] * xyzEstimationError[1]

		# if CONST.proportionRange_CL_DA:
		# 	pass
		# else:
		# 	modifier = modifier * modifier
		#
		# if ((currTrack.age > 0)):  # /* If the object hasn't been associated in the previous tick, search in a wider range. */
		# 	modifier = modifier * 2.0

		if (distSq < trackingCfg.distAssocThreshSq * modifier* modifier):
			pdistSq = distSq + rrdEstimationError[1] * rrdEstimationError[1]
			trigger = True

			return pdistSq, trigger
		if CONST.isTargetWithinDataAssociationThreshDEBUG and currTrack.ID == CONST.RCS_DEBUG_ID_NUMBER:
			print(
				"the current track (ID, x, y, xd, xy) = (%d, %.2f, %.2f, %.2f, %.2f) is not matched with the measurement point(x, y, xd, xy) = (%.2f, %.2f, %.2f, %.2f): distSq = %f."
				% (currTrack.ID, currTrack.stateVectorXYZ[0], currTrack.stateVectorXYZ[1],
				   currTrack.stateVectorXYZ[2], currTrack.stateVectorXYZ[3],
				   xyzCurrMeas[0], xyzCurrMeas[1], xyzCurrMeas[2],
				   xyzCurrMeas[3],distSq))
		return None, trigger
	else:
		if CONST.isTargetWithinDataAssociationThreshDEBUG and currTrack.ID == CONST.RCS_DEBUG_ID_NUMBER:
			print(
				"the current track (ID, x, y, xd, xy) = (%d, %.2f, %.2f, %.2f, %.2f) is not matched with the measurement point(x, y, xd, xy) = (%.2f, %.2f, %.2f, %.2f).\n"
				"a : %s, b : %s, c: %s, modifier : %f, diffDoppler : % f, diffAzim : %f, threshAzim : %f. "

				% (currTrack.ID, currTrack.stateVectorXYZ[0], currTrack.stateVectorXYZ[1],
				   currTrack.stateVectorXYZ[2], currTrack.stateVectorXYZ[3],
				   xyzCurrMeas[0], xyzCurrMeas[1], xyzCurrMeas[2],
				   xyzCurrMeas[3],a,b,c, modifier, abs(rrdEstimationError[1]), abs(rrdEstimationError[2]), (modifier * trackingCfg.azimAssocThresh)))
	return None, trigger


def genFmatrix(td):

	F = np.zeros((4,4), dtype = np.float)
	for i in range(4):
		F[i][i] = 1
	F[0][2] = td
	F[1][3] = td

	return F



def stateVecTimeUpdate(currTrack, td):

	currTrack.stateVectorXYZ[0] = currTrack.stateVectorXYZ[0] + td * currTrack.stateVectorXYZ[2]
	currTrack.stateVectorXYZ[1] = currTrack.stateVectorXYZ[1] + td * currTrack.stateVectorXYZ[3]
	if CONST.IMM:
		for idx, state in enumerate(currTrack.state):
			if idx == 0:
				state.stateVectorXYZ[0] = state.initCondstate[0] + td * state.initCondstate[2]
				state.stateVectorXYZ[1] = state.initCondstate[1] + td * state.initCondstate[3]
			elif idx == 1:
				state.stateVectorXYZ[0] = state.initCondstate[0] + td * state.initCondstate[2]
				state.stateVectorXYZ[1] = state.initCondstate[1] + td * state.initCondstate[3]

	if currTrack.stateVectorXYZ[1] < 0:
		return False
	return True

def stateCovmatTimeUpdate_TI(currTrack, td, Qmat):
	covmattmp = np.zeros(10, dtype=np.float)
	covmat = np.zeros(10, dtype=np.float)

	covmat[iXX] = currTrack.Covariance[0][0]
	covmat[iXY] = currTrack.Covariance[0][1]
	covmat[iXXd] =currTrack.Covariance[0][2]
	covmat[iXYd] =currTrack.Covariance[0][3]
	covmat[iYY] = currTrack.Covariance[1][1]
	covmat[iYXd] =currTrack.Covariance[1][2]
	covmat[iYYd] =currTrack.Covariance[1][3]
	covmat[iXdXd] = currTrack.Covariance[2][2]
	covmat[iXdYd] = currTrack.Covariance[2][3]
	covmat[iYdYd] = currTrack.Covariance[3][3]

	Q = np.zeros(4, dtype=np.float)
	Q[iX] = Qmat[0][0]
	Q[iY] = Qmat[1][1]
	Q[iXd] = Qmat[2][2]
	Q[iYd] = Qmat[3][3]

	tdsq = td* td

	covmattmp[iXX] = covmat[iXX] + 2 * td * covmat[iXXd] + tdsq * covmat[iXdXd] + Q[iX]
	covmattmp[iXY] = covmat[iXY] + td * (covmat[iXYd] + covmat[iYXd]) + tdsq * covmat[iXdYd]

	covmattmp[iXXd] = covmat[iXdXd] * td + covmat[iXXd]
	covmattmp[iXYd] = covmat[iXdYd] * td + covmat[iXYd]

	covmattmp[iYY] = tdsq * covmat[iYdYd] + 2 * td * covmat[iYYd] + covmat[iYY] + Q[iY]

	covmattmp[iYXd] = covmat[iXdYd] * td + covmat[iYXd]
	covmattmp[iYYd] = covmat[iYdYd] * td + covmat[iYYd]

	covmattmp[iXdXd] = covmat[iXdXd] + Q[iXd]
	covmattmp[iXdYd] = covmat[iXdYd]
	covmattmp[iYdYd] = covmat[iYdYd] + Q[iYd]

	currTrack.Covariance[0][0] = covmattmp[iXX]
	currTrack.Covariance[0][1] = covmattmp[iXY]
	currTrack.Covariance[0][2] = covmattmp[iXXd]
	currTrack.Covariance[0][3] = covmattmp[iXYd]
	currTrack.Covariance[1][1] = covmattmp[iYY]
	currTrack.Covariance[1][2] = covmattmp[iYXd]
	currTrack.Covariance[1][3] = covmattmp[iYYd]
	currTrack.Covariance[2][2] = covmattmp[iXdXd]
	currTrack.Covariance[2][3] = covmattmp[iXdYd]
	currTrack.Covariance[3][3] = covmattmp[iYdYd]

	currTrack.Covariance[1][0] = currTrack.Covariance[0][1]
	currTrack.Covariance[2][0] = currTrack.Covariance[0][2]
	currTrack.Covariance[3][0] = currTrack.Covariance[0][3]
	currTrack.Covariance[2][1] = currTrack.Covariance[1][2]
	currTrack.Covariance[3][1] = currTrack.Covariance[1][3]
	currTrack.Covariance[3][2] = currTrack.Covariance[2][3]




def stateCovmatTimeUpdate(currTrack, td, Qmat):

	F = np.zeros((4,4), dtype = np.float)
	for i in range(4):
		F[i][i] = 1
	F[0][2] = td
	F[1][3] = td

	currTrack.Covariance = np.matmul(F, currTrack.Covariance)
	currTrack.Covariance = np.matmul(currTrack.Covariance, F.T) + Qmat

	if CONST.IMM:



		for state, mode in zip(currTrack.state, Filter.mode_list):
			state.Covariance = np.matmul(mode.F,state.initCondCov)
			state.Covariance = np.matmul(state.Covariance, mode.F.T)
			state.Covariance = state.Covariance + mode.Q


def calQmat(Qvec):
	Qmat = np.zeros((4, 4), dtype=np.float)
	for i in range(4):
		for j in range(4):
			Qmat[i][j] = Qvec[i] * Qvec[j]

	Qmat[1][0] = 0
	Qmat[0][1] = 0
	Qmat[3][0] = 0
	Qmat[0][3] = 0
	Qmat[1][2] = 0
	Qmat[2][1] = 0
	Qmat[2][3] = 0
	Qmat[3][2] = 0
	return Qmat


def select_QVec_and_cal_Qmat(QvecList, currTrack):
	#QvecList's type is np.array with 3x4 size.
	# if currTrack.ID == 5:
	# 	print("select_QVec_and_cal_Qmat function is started!")
	# 	print("QvecList")
	# 	print(QvecList)
	if CONST.Qmat_modify:

		if CONST.proportionRange_CL_DA:
			if currTrack.stateVectorXYZ[1] < 50:
				Qvec = QvecList[0][:]

			elif currTrack.stateVectorXYZ[1] < 100:
				Qvec = QvecList[1][:]
			else:
				Qvec = QvecList[2][:]
		else:
			if currTrack.tick < 2 or currTrack.stateVectorXYZ[1] < 160 or currTrack.age > 1:
				Qvec = QvecList[0][:]

			elif currTrack.tick < 4 or currTrack.stateVectorXYZ[1] < 15 or currTrack.age > 0:
				Qvec = QvecList[1][:]
			else:
				Qvec = QvecList[2][:]


	else:

		if currTrack.tick < 2 or currTrack.stateVectorXYZ[1] < 10 or currTrack.age > 1:
			Qvec = QvecList[0][:]

		elif currTrack.tick < 4 or currTrack.stateVectorXYZ[1] < 15 or currTrack.age > 0:
			Qvec = QvecList[1][:]

		else:
			Qvec = QvecList[2][:]
	# if currTrack.ID == 5:
	# 	print("selected Qvec")
	# 	print(Qvec)
	# Qmat = np.matmul(Qvec.T,Qvec)
	if CONST.Qmat_piecewiseConstantAccel:
		Qmat = calQmat(Qvec)
	else:
		Qmat = np.zeros((4,4),dtype = np.float)
		for i in range(4):
			Qmat[i][i] = Qvec[i]
	if CONST.ekfDEBUG:
		if currTrack.ID == CONST.RCS_DEBUG_ID_NUMBER:
			print("selected Qmat")
			print(Qmat)
	# 	print("select_QVec_and_cal_Qmat function is ended!")
		
	# Qmat = np.eye(4) * 0.5
	return Qmat
def computeHmat_TI(predictedXYZ, predictedRRD):
	Hmat = np.zeros((3, 4), dtype=np.float)
	Hmat[0][0] = predictedRRD[2]
	Hmat[0][1] = predictedXYZ[1] / predictedRRD[0]
	Hmat[1][0] = (predictedXYZ[2] - (predictedRRD[2] * predictedRRD[1])) / predictedRRD[0]
	Hmat[1][1] =(predictedXYZ[3] - (predictedRRD[1]*Hmat[0][1])) / predictedRRD[0]
	Hmat[1][2] = Hmat[0][0]
	Hmat[1][3] = Hmat[0][1]

	# For r, rd, theta coordinate
	# Hmat[2][0] = predictedXYZ[1]/(predictedRRD[0]*predictedRRD[0])
	# Hmat[2][1] = -1*predictedXYZ[0]/(predictedRRD[0]*predictedRRD[0])

	# For r, rd, sinAzim coordinate
	Hmat[2][0] = (1 - predictedRRD[2] * predictedRRD[2]) / predictedRRD[0]
	Hmat[2][1] = -1 * (predictedRRD[2] * Hmat[0][1]) / predictedRRD[0]
	return Hmat

def computeHmat(predictedXYZ, predictedRRD): # predictedXYZ : x_k_pred, predictedRRD : z_k_pred = H_k * x_k_pred
	if CONST.IMM:
		Hmat = np.zeros((3, DT.max_dim_state), dtype=np.float)
	else:
		Hmat = np.zeros((N_MEAS, N_STATES), dtype=np.float)
	Hmat[0][0] = predictedRRD[2]
	Hmat[0][1] = predictedXYZ[1] / predictedRRD[0]
	if CONST.Hmat_doppler2Ydot:
		Hmat[1][0] = 0
		Hmat[1][1] = 0
		Hmat[1][2] = 0
		Hmat[1][3] = 1
	else:
		Hmat[1][0] = (predictedXYZ[2] - (predictedRRD[2] * predictedRRD[1])) / predictedRRD[0]
		Hmat[1][1] = (predictedXYZ[3] - (predictedRRD[1] * Hmat[0][1])) / predictedRRD[0]
		Hmat[1][2] = Hmat[0][0]
		Hmat[1][3] = Hmat[0][1]

	# For r, rd, theta coordinate
	# Hmat[2][0] = predictedXYZ[1]/(predictedRRD[0]*predictedRRD[0])
	# Hmat[2][1] = -1*predictedXYZ[0]/(predictedRRD[0]*predictedRRD[0])
	
	# For r, rd, sinAzim coordinate
	Hmat[2][0] = (1 - predictedRRD[2] * predictedRRD[2]) / predictedRRD[0]
	Hmat[2][1] = -1*(predictedRRD[2] * Hmat[0][1]) / predictedRRD[0]
	if CONST.ADD_ANGLE_VEL:
		Hmat[3][0] = (-3*predictedXYZ[0]*(predictedXYZ[1]**2)*predictedXYZ[2] \
		              - (predictedXYZ[1]**3)*predictedXYZ[3]\
		              + 2 *(predictedXYZ[0]**2)*predictedXYZ[1]*predictedXYZ[3])\
		             /(predictedRRD[0] ** 5)
		Hmat[3][1] = (2*(predictedXYZ[0]**2)*predictedXYZ[1]*predictedXYZ[2]\
		              +2*predictedXYZ[0]*(predictedXYZ[1]**2)*predictedXYZ[3]\
		              -predictedXYZ[2]*(predictedXYZ[1]**3)\
		              -(predictedXYZ[0]**3)*predictedXYZ[3])\
					/(predictedRRD[0] ** 5)
		Hmat[3][2] = (predictedXYZ[1]**2)/(predictedRRD[0] ** 3)
		Hmat[3][3] = -1*(predictedXYZ[0]*predictedXYZ[1])/(predictedRRD[0] ** 3)
	if CONST.predictedCalMod:
		Hmat[0][0] = predictedXYZ[0] / np.sqrt((predictedXYZ[0] ** 2) + (predictedXYZ[1] ** 2))
		Hmat[0][1] = predictedXYZ[1] / np.sqrt((predictedXYZ[0] ** 2) + (predictedXYZ[1] ** 2))
		Hmat[1][0] = 0
		Hmat[1][1] = 0
		Hmat[1][2] = 0
		Hmat[1][3] = 1
		Hmat[2][0] = (predictedXYZ[1]**2) / (np.sqrt((predictedXYZ[0] ** 2) + (predictedXYZ[1] ** 2)) ** (3))
		Hmat[2][1] = -(predictedXYZ[1]*predictedXYZ[0]) / (np.sqrt((predictedXYZ[0] ** 2) + (predictedXYZ[1] ** 2)) ** (3))


	return Hmat

def computeResidCovmat(Hmat, currTrack_cov, measCovVec, currTrackID):
	# residualCovariance = H * currTrack.Covariance * HT + R #R noise modelingㅡ을 안했네	clustering() 부분에서 하도록
	Rmat =np.zeros((N_MEAS,N_MEAS), dtype = np.float)
	for i in range(N_MEAS):
		Rmat[i][i] = measCovVec[i]
	# Rmat[0][0] = measCovVec[0]
	# Rmat[1][1] = measCovVec[1]
	# Rmat[2][2] = measCovVec[2]
	# Rmat = np.zeros((3, 3), dtype=np.float)
	# Rmat = np.eye(3) * 0.5
	if CONST.ekfDEBUG:
		if currTrackID == CONST.RCS_DEBUG_ID_NUMBER:
			print("Rmat")
			print(Rmat)
	# for i in range(3):
	# 	Rmat[i][i] =1
		# print(Rmat)
	residualCovariance = np.matmul(Hmat, currTrack_cov)
	residualCovariance = np.matmul(residualCovariance, Hmat.T) 
	# print(residualCovariance)
	residualCovariance = residualCovariance + Rmat

	return residualCovariance

def kalmanGainComputation(currTrack, Hmat, invResidCovmat):
	if CONST.ekfDEBUG:
		if currTrack.ID == CONST.RCS_DEBUG_ID_NUMBER:
			print("P_k_pred")
			print(currTrack.Covariance)
			print("H")
			print(Hmat)
			print("invResidCovmat")
			print(invResidCovmat)
			#
			# temp = np.matmul(Hmat.T, invResidCovmat)
			# print("temp")
			# print(temp, "\n")
			#
			# kalmanGain3 = np.matmul(currTrack.Covariance[3,:],temp[:,0])
			# print("kalmanGain3",kalmanGain3)
			# print(currTrack.Covariance[3,:],temp[:,0])


	kalmanGain = np.matmul(currTrack.Covariance, Hmat.T)
	kalmanGain = np.matmul(kalmanGain,invResidCovmat)

	# kalmanGain[0][:] = kalmanGain[0][:] * 0.4

	return kalmanGain
def computeMeasResidual(currTrack, tmpPredictedMeas):

	# associatedObj = currTrack.associatedObj
	rrdCurrMeas = currTrack.associatedObj.measVectorRRD
	measResidual = rrdCurrMeas - tmpPredictedMeas
	if CONST.ekfDEBUG:
		if currTrack.ID == CONST.RCS_DEBUG_ID_NUMBER:
			print("z_k")
			print(rrdCurrMeas)
			print("z_k_pred")
			print(tmpPredictedMeas)
			print("z_k - z_k_pred : measResidual")
			print(measResidual)
	# measResidual[2] = -1*measResidual[2]

	# measResidual = rrdCurrMeas = rrdCurrMeas
	return measResidual

def stateVecMeasurementUpdat_TI(currTrack, kalmanGainMatrix, meas_residual):
	kalmanGain = np.zeros(12, dtype = np.float)
	kalmanGain[0] = kalmanGainMatrix[0][0]
	kalmanGain[1] = kalmanGainMatrix[1][0]
	kalmanGain[2] = kalmanGainMatrix[2][0]
	kalmanGain[3] = kalmanGainMatrix[3][0]
	kalmanGain[4] = kalmanGainMatrix[0][1]
	kalmanGain[5] = kalmanGainMatrix[1][1]
	kalmanGain[6] = kalmanGainMatrix[2][1]
	kalmanGain[7] = kalmanGainMatrix[3][1]
	kalmanGain[8] = kalmanGainMatrix[0][2]
	kalmanGain[9] = kalmanGainMatrix[1][2]
	kalmanGain[10] = kalmanGainMatrix[2][2]
	kalmanGain[11] = kalmanGainMatrix[3][2]

	for ik in range(N_STATES):
		sum = 0
		for kk in range(N_MEASUREMENTS):
			sum += kalmanGain[(kk*N_STATES) + ik] * meas_residual[kk]
		currTrack.stateVectorXYZ[ik] += sum

	if currTrack.stateVectorXYZ[1] < 0:
		return False
	else:
		return True

def stateVecMeasurementUpdate(currTrack, kalmanGain, measurmentResidual):
	if CONST.ekfDEBUG:
		if currTrack.ID == CONST.RCS_DEBUG_ID_NUMBER:
			print("kalmanGain")
			print(kalmanGain)
			print("measurmentResidual.T")
			print(measurmentResidual.T)
			print("currTrack.stateVectorXYZ before stateVecMeasurementUpdate")
			print(currTrack.stateVectorXYZ)
	currTrack.stateVectorXYZ = currTrack.stateVectorXYZ + np.matmul(kalmanGain, measurmentResidual.T)
	if CONST.ekfDEBUG:
		if currTrack.ID == CONST.RCS_DEBUG_ID_NUMBER:
			print("currTrack.stateVectorXYZ after stateVecMeasurementUpdate")
			print(currTrack.stateVectorXYZ)
	# print(currTrack.stateVectorXYZ)
	if currTrack.stateVectorXYZ[1] < 0:

		return False
	else:
		return True


def stateCovmatMeasurementUpdate_TI(currTrack, kalmanGainMatrix, Hmat):
	kalmanGain = np.zeros(12, dtype=np.float)
	hMat = np.zeros(6, dtype=np.float)
	temp = np.zeros(16, dtype=np.float)
	tempP = np.zeros(N_UNIQ_ELEM_IN_SYM_COVMAT, dtype=np.float)
	covmat = np.zeros(N_UNIQ_ELEM_IN_SYM_COVMAT, dtype=np.float)

	covmat[iXX]   = currTrack.Covariance[0][0]
	covmat[iXY]   = currTrack.Covariance[0][1]
	covmat[iXXd]  = currTrack.Covariance[0][2]
	covmat[iXYd]  = currTrack.Covariance[0][3]
	covmat[iYY]   = currTrack.Covariance[1][1]
	covmat[iYXd]  = currTrack.Covariance[1][2]
	covmat[iYYd]  = currTrack.Covariance[1][3]
	covmat[iXdXd] = currTrack.Covariance[2][2]
	covmat[iXdYd] = currTrack.Covariance[2][3]
	covmat[iYdYd] = currTrack.Covariance[3][3]

	kalmanGain[0]  = kalmanGainMatrix[0][0]
	kalmanGain[1]  = kalmanGainMatrix[1][0]
	kalmanGain[2]  = kalmanGainMatrix[2][0]
	kalmanGain[3]  = kalmanGainMatrix[3][0]
	kalmanGain[4]  = kalmanGainMatrix[0][1]
	kalmanGain[5]  = kalmanGainMatrix[1][1]
	kalmanGain[6]  = kalmanGainMatrix[2][1]
	kalmanGain[7]  = kalmanGainMatrix[3][1]
	kalmanGain[8]  = kalmanGainMatrix[0][2]
	kalmanGain[9]  = kalmanGainMatrix[1][2]
	kalmanGain[10] = kalmanGainMatrix[2][2]
	kalmanGain[11] = kalmanGainMatrix[3][2]

	hMat[0] = Hmat[0][0]
	hMat[1] = Hmat[0][1]
	hMat[2] = Hmat[1][0]
	hMat[3] = Hmat[1][1]
	hMat[4] = Hmat[2][0]
	hMat[5] = Hmat[2][1]

	for ik in range(N_STATES):
		for jk in range(int(N_STATES/2)):
			if ik == jk:
				sum = 1
			else:
				sum = 0
			for kk in range(N_MEASUREMENTS):
				sum -= kalmanGain[int((kk*N_STATES) + ik)] * hMat[int(jk + kk*(N_STATES/2))]
			temp[(ik * N_STATES) + jk] = sum

	kk = 1

	for ik in range(N_STATES):
		for jk in range(int(N_STATES/2), N_STATES):
			if ik == jk:
				sum = 1
			else:
				sum = 0
			sum -= kalmanGain[int((kk*N_STATES) + (ik))] * hMat[int(((jk - (N_STATES / 2))) + 0 * (N_STATES / 2))]
			temp[(ik * N_STATES) + jk] = sum

	tempP[0] = covmat[iXYd] * temp[3] + covmat[iXXd] * temp[2] + covmat[iXY] * temp[1] + covmat[iXX] * temp[0]
	tempP[1] = covmat[iXYd] * temp[7] + covmat[iXXd] * temp[6] + covmat[iXY] * temp[5] + covmat[iXX] * temp[4]
	tempP[2] = covmat[iXY] * temp[9] + covmat[iXX] * temp[8] + covmat[iXYd] * temp[11] + covmat[iXXd] * temp[10]
	tempP[3] = covmat[iXYd] * temp[15] + covmat[iXXd] * temp[14] + covmat[iXY] * temp[13] + covmat[iXX] * temp[12]
	tempP[4] = covmat[iYYd] * temp[7] + covmat[iYXd] * temp[6] + covmat[iYY] * temp[5] + covmat[iXY] * temp[4]
	tempP[5] = covmat[iYY] * temp[9] + covmat[iXY] * temp[8] + covmat[iYYd] * temp[11] + covmat[iYXd] * temp[10]
	tempP[6] = covmat[iYYd] * temp[15] + covmat[iYXd] * temp[14] + covmat[iYY] * temp[13] + covmat[iXY] * temp[12]
	tempP[7] = covmat[iYXd] * temp[9] + covmat[iXXd] * temp[8] + covmat[iXdYd] * temp[11] + covmat[iXdXd] * temp[10]
	tempP[8] = covmat[iXdYd] * temp[15] + covmat[iXdXd] * temp[14] + covmat[iYXd] * temp[13] + covmat[iXXd] * temp[12]
	tempP[9] = covmat[iYdYd] * temp[15] + covmat[iXdYd] * temp[14] + covmat[iYYd] * temp[13] + covmat[iXYd] * temp[12]

	for ik in range(N_UNIQ_ELEM_IN_SYM_COVMAT):
		covmat[ik] = tempP[ik]

	currTrack.Covariance[0][0] = covmat[iXX]
	currTrack.Covariance[0][1] = covmat[iXY]
	currTrack.Covariance[0][2] = covmat[iXXd]
	currTrack.Covariance[0][3] = covmat[iXYd]
	currTrack.Covariance[1][1] = covmat[iYY]
	currTrack.Covariance[1][2] = covmat[iYXd]
	currTrack.Covariance[1][3] = covmat[iYYd]
	currTrack.Covariance[2][2] = covmat[iXdXd]
	currTrack.Covariance[2][3] = covmat[iXdYd]
	currTrack.Covariance[3][3] = covmat[iYdYd]

	currTrack.Covariance[1][0] = currTrack.Covariance[0][1]
	currTrack.Covariance[2][0] = currTrack.Covariance[0][2]
	currTrack.Covariance[3][0] = currTrack.Covariance[0][3]
	currTrack.Covariance[2][1] = currTrack.Covariance[1][2]
	currTrack.Covariance[3][1] = currTrack.Covariance[1][3]
	currTrack.Covariance[3][2] = currTrack.Covariance[2][3]


def stateCovmatMeasurementUpdate(currTrack, kalmanGain, Hmat):
	a = (np.eye(4)-np.matmul(kalmanGain, Hmat))
	currTrack.Covariance = np.matmul(a, currTrack.Covariance)


def estimateGuardRail(clusterList, RadarInfo):
	ThY = 40.0
	Angle_Left = 0
	Angle_Right = 0
	Left_Idx = 0
	Right_Idx = 0
	for cluster in clusterList:
		idx = 0
		targetX = []
		targetY = []
		targetV_Plus = []
		targetV_Minus = []
		Idx_plus = 0
		Idx_minus = 0
		Tar_Plus = 1
		diffV = 10
		autoCal_valid = 0

		for target in cluster:
			if abs(target.rotatex) > 18.0:
				continue
			idx += 1
			targetX.append(target.rotatex)
			targetY.append(target.rotatey)

			if abs(target.speed) >= 20:
				continue
			if target.speed >= 0:
				Idx_plus += 1
				targetV_Plus.append(target.speed)
			else:
				Idx_minus += 1
				targetV_Minus.append(target.speed)

		if (Idx_plus > 2) or (Idx_minus > 2):
			if Idx_plus > Idx_minus:
				Tar_Plus = 1
				diffV = max(targetV_Plus) - min(targetV_Plus)
			else:
				Tar_Plus = 0
				diffV = max(targetV_Minus) - min(targetV_Minus)

		if idx > 0:
			diffX = max(targetX) - min(targetX)
			diffY = max(targetY) - min(targetY)

			minX = min(targetX)
			maxX = max(targetX)
			minX_Idx = targetX.index(minX)
			maxX_Idx = targetX.index(maxX)
			minY = targetY[minX_Idx]
			maxY = targetY[maxX_Idx]

			if Tar_Plus == 1:
				if (idx > 2) and (diffX > 6.0) and (diffY < 0.7) and (diffV > 7):			# (4.5, 1.0) --> (6.0, 0.7)
					TmpY = min(targetY)
					autoCal_valid = 1
				elif (idx > 2) and (diffX >= 8.0) and (diffV > 7):						    # (6.0) --> (8.0)
					TmpY = min(targetY)
				else:
					TmpY = 40.0
			else:
				TmpY = 40.0

			if ThY > TmpY:
				ThY = TmpY

			if (autoCal_valid == 1) and (idx > 5):
				DevX = maxX - minX
				DevY = maxY - minY
				Cal_Angle_Deg = math.atan2(DevY, DevX)*180/np.pi

				if RadarInfo.position == 1:
					Left_Idx = Left_Idx + 1
					Angle_Left = Angle_Left + Cal_Angle_Deg
				elif RadarInfo.position == 2:
					Right_Idx = Right_Idx + 1
					Angle_Right = Angle_Right + Cal_Angle_Deg
				else:
					Angle_Left = Angle_Left
					Angle_Right = Angle_Right

	if Left_Idx != 0:
		Angle_Left = Angle_Left/Left_Idx
#		CalAngleLeft_Idx = CalAngleLeft_Idx + 1
#		if CalAngleLeft_Idx <= RadarInfo.CalNum:
#			CalAngleLeft = CalAngleLeft + Angle_Left/RadarInfo.CalNum
	if Right_Idx != 0:
		Angle_Right = Angle_Right/Right_Idx
#		CalAngleRight_Idx = CalAngleRight_Idx + 1
#		if CalAngleRight_Idx <= RadarInfo.CalNum:
#			CalAngleRight = CalAngleRight + Angle_Right/RadarInfo.CalNum

	ThresholdY = ThY

	return ThresholdY


def populateTrackingList(clusterOutputList, ThresholdY, RadarInfo):
	tmpTrackingList = []

	for cluster in clusterOutputList:
		### HSLee 추가 2022.10.28 : Radar 전방 2[m] Clutter 제거
		clusterNum = cluster.trackingInput.pointNum

		if CONST.trackingInputMODIFY:
			clusterX = cluster.xCenter
			clusterY = cluster.yCenter
		else:
			clusterX = cluster.strongestMember.x
			clusterY = cluster.strongestMember.y

		if (clusterNum == 1) and (-0.8 <= clusterX) and (clusterX <= 0.5) and (1.5 <= clusterY) and (clusterY <= 2.5):
			continue
		### HSLee 추가 2022.10.28 : Radar 전방 2[m] Clutter 제거

		### HSLee 추가 2022.09.16
		Deg2_rad = RadarInfo.angle * np.pi / 180
		rotateY = -clusterX * np.sin(Deg2_rad) + clusterY * np.cos(Deg2_rad)
		if (RadarInfo.mode == 1) and (rotateY >= (ThresholdY - 0.2)):
			cluster.trackingInput.statusFlag =  8
		### HSLee 추가 2022.09.16

		tmpTrackingList.append(cluster.trackingInput)
	
	return tmpTrackingList


def initNewTracker(currMeas):
	# /* The covariance mat doesn't really matter at init, but we want some reasonabble numbers in it */ 
	currMeas.Covariance[0][0] = (1 + currMeas.measCovVec[0]) * (1 + currMeas.measCovVec[2])
	# currMeas.Covariance[1][1] = 1
	currMeas.Covariance[1][1] = (1 + currMeas.measCovVec[0]) * (1 + currMeas.measCovVec[2])
	currMeas.Covariance[2][2] = (3 * currMeas.measVectorRRD[1] * currMeas.measVectorRRD[1]) + ((1 + currMeas.measCovVec[1]) * (1 + currMeas.measCovVec[2]))
	# currMeas.Covariance[2][2] = 0.01
	currMeas.Covariance[3][3] = (3 * currMeas.measVectorRRD[1] * currMeas.measVectorRRD[1]) + ((1 + currMeas.measCovVec[1]) * (1 + currMeas.measCovVec[2]))

	# currMeas.Covariance[0][0] = 1
	# currMeas.Covariance[1][1] = 1
	# currMeas.Covariance[2][2] = 1
	# currMeas.Covariance[3][3] = 1
	# if CONST.ekfDEBUG:
	# 	print("=========initNewTracker========================================")
	# 	print("initNewTracker measVectorRRD\n", currMeas.measVectorRRD)
	# 	print("initNewTracker stateVectorXYZ\n", currMeas.stateVectorXYZ)
	# 	print("initNewTracker measCovVec\n", currMeas.measCovVec)
	# 	print("initNewTracker Covariance\n", currMeas.Covariance)
	# 	print("================================================================")
	# currMeas.tick += 1
	# currMeas.Covariance = np.eye(4) * 0.5


def pruneTrackingInput(measList, aoaCfg, trackingCfg, vehicleSpeed):
	# print(aoaCfg.SIN_55_DEGREES)
	vehicleSpeedFlt = vehicleSpeed / 3.6
	for trackInput in measList[:]:

		condition = ((abs(trackInput.measVectorRRD[2]) < aoaCfg.SIN_55_DEGREES)\
			or (trackInput.measCovVec[2] < trackingCfg.TRK_SIN_AZIM_THRESH))
#		    and ( trackInput.measVectorRRD[1] < 0.0 )

		if condition:
			if CONST.pruneTrackingInputDEBUG:
				print("point",trackInput.measVectorRRD,"is not removed in pruneTrackingInput.")
				print("measCovVec", trackInput.measCovVec)
		else:
			if CONST.pruneTrackingInputDEBUG:
				print("point",trackInput.measVectorRRD,"is removed in pruneTrackingInput.")
				print("measCovVec", trackInput.measCovVec)
			measList.remove(trackInput)
			# print("deleted")
			del(trackInput)

	return measList

def isWithinBoundingBox(currTrack, trackingCfg):
	isValid = False
	maxRange = trackingCfg.maxRange
	maxRange_x = trackingCfg.max_detect_range_x							# HSLee 추가 2022.01.21

	if ((currTrack.stateVectorXYZ[0] > -maxRange_x) and (currTrack.stateVectorXYZ[0] < maxRange_x)):
		isValid = True
	else:
		return False
	if ((currTrack.stateVectorXYZ[1] > -maxRange) and (currTrack.stateVectorXYZ[1] < maxRange)):
		isValid = True
	else:
		return False

	return isValid

def invalidateCurrTrack(currTrack, nFree, freeTrackerList):
	currTrack.validity = False
	freeTrackerList.append(currTrack)
	nFree += 1

	return nFree

def selectMeas(measList, nFree, numUnassociatedMeas, numMeasTotal, selectedMeasList, trackingCfg):
	if numUnassociatedMeas == 0 or numMeasTotal == 0:
		return 0
	numSelected = 0

	if numUnassociatedMeas > nFree:
		for currMeas in measList[:]:
			if not (numSelected < nFree):
				break
			if currMeas.isAssociated == False:
				if currMeas.measCovVec[0] < trackingCfg.HIGH_SNR_RVAR_THRESH:
					numSelected += 1
					currMeas.isAssociated = True

					selectedMeasList.append(currMeas)
	for currMeas in measList[:]:
		if not (numSelected < nFree):
			break
		if currMeas.isAssociated == False:
			numSelected += 1
			currMeas.isAssociated = True
			selectedMeasList.append(currMeas)

	return numSelected

def updateAngleVelocityVariance(currTrack, prevStateVec, prevCovariance, Hmat, trackingCfg):
	currTrackAssociatedObj = currTrack.associatedObj


	if CONST.ADD_ANGLE_VEL_SIMPLE_VAR:
		currTrackAssociatedObj.measCovVec[3] = currTrackAssociatedObj.measCovVec[2] * 15*15*2
	else:
		cosTheta = (currTrackAssociatedObj.stateVectorXYZ[1]/currTrackAssociatedObj.measVectorRRD[0])
		var0 = ((currTrackAssociatedObj.measVectorRRD[2] ** 2)*currTrackAssociatedObj.measCovVec[0] \
		    + (currTrackAssociatedObj.measVectorRRD[0] ** 2)*currTrackAssociatedObj.measCovVec[2])
		var1 = (cosTheta ** 2) * currTrackAssociatedObj.measCovVec[0] \
				+ (currTrackAssociatedObj.measVectorRRD[0] ** 2) * (currTrackAssociatedObj.measVectorRRD[2] ** 2) * currTrackAssociatedObj.measCovVec[2] / (cosTheta ** 2)
		currTrackAssociatedObj.measCovVec[3] \
			= (Hmat[3][0] ** 2) * var0	+ (Hmat[3][1] ** 2) * var1 \
		    + ((Hmat[3][2]/trackingCfg.td) ** 2) * (var0 + prevCovariance[0][0])\
			+ ((Hmat[3][3]/trackingCfg.td) ** 2) * (var1 + prevCovariance[1][1])

		if CONST.MEAS_COV_VEC_SCALAR_MODIFY:
			currTrackAssociatedObj.measCovVec[3] = currTrackAssociatedObj.measCovVec[3] * 1


def updateAngleVelocity(currTrack, prevStateVec, prevCovariance, trackingCfg):

	currTrackAssociatedObj = currTrack.associatedObj

	if CONST.ekfDEBUG and currTrack.ID == CONST.RCS_DEBUG_ID_NUMBER:
		print("updateAngleVelocity is start")
		print("z_k",currTrackAssociatedObj.stateVectorXYZ)
		print("x_k-1", prevStateVec)

	x = currTrackAssociatedObj.stateVectorXYZ[0]
	y = currTrackAssociatedObj.stateVectorXYZ[1]
	xd = (currTrackAssociatedObj.stateVectorXYZ[0] - prevStateVec[0]) / trackingCfg.td
	yd = (currTrackAssociatedObj.stateVectorXYZ[1] - prevStateVec[1]) / trackingCfg.td

	# measXYZCovariance[0] = (1 + currTrack.measCovVec[0]) * (1 + currTrack.measCovVec[2])
	# measXYZCovariance[1] = (1 + currTrack.measCovVec[0]) * (1 + currTrack.measCovVec[2])
	# measXYZCovariance[2] = (3 * currTrack.measVectorRRD[1] * currTrack.measVectorRRD[1]) + (
	# 			(1 + currTrack.measCovVec[1]) * (1 + currTrack.measCovVec[2]));
	# measXYZCovariance[3] = (3 * currTrack.measVectorRRD[1] * currTrack.measVectorRRD[1]) + (
	# 			(1 + currTrack.measCovVec[1]) * (1 + currTrack.measCovVec[2]));

	r = np.sqrt(x * x + y * y)
	currTrackAssociatedObj.measVectorRRD[3] = \
		(xd * y * y - x * y * yd) / (r ** 3)

	# r_prev_sq = (prevStateVec[0] ** 2) + (prevStateVec[1] ** 2)
	# r_prev = np.sqrt(r_prev_sq)
	# rd_prev = (prevStateVec[0] * prevStateVec[2] + prevStateVec[1] * prevStateVec[3]) / r_prev
	# sinAzim_prev = prevStateVec[0] / r_prev


	# currTrackAssociatedObj.measCovVec[3] = currTrackAssociatedObj.measCovVec[2] * CONST.MEAS_COV_VEC3_SCALAR
	# currTrackAssociatedObj.measCovVec[3] = (currTrackAssociatedObj.measCovVec[2] + prevCovariance[2][2])/(trackingCfg.td ** 2)
	# print("measVectorRRD[3]",currTrackAssociatedObj.measVectorRRD[3], "currTrackAssociatedObj.measCovVec[2]", currTrackAssociatedObj.measCovVec[2], "prevCovariance[2][2]", prevCovariance[2][2])
	# currTrackAssociatedObj.measCovVec[3] = (currTrackAssociatedObj.measCovVec[2] * 2)/(trackingCfg.td ** 2)
	# currTrackAssociatedObj.measCovVec[3] = (prevCovariance[2][2] \
	#                                         + (rd_prev ** 2) * currTrackAssociatedObj.measCovVec[2]\
	#                                         + (sinAzim_prev ** 2) * currTrackAssociatedObj.measCovVec[1]\
	#                                         + currTrackAssociatedObj.measCovVec[2] * currTrackAssociatedObj.measCovVec[1])/r_prev_sq

	if CONST.ekfDEBUG and currTrack.ID == CONST.RCS_DEBUG_ID_NUMBER:
		print("xd, yd :",xd, yd)

def computeResidCovmat_addR(residCovmat_, measCovVec):
	Rmat = np.zeros((N_MEAS, N_MEAS), dtype=np.float)
	for i in range(N_MEAS):
		Rmat[i][i] = measCovVec[i]

	residCovmat = residCovmat_ + Rmat

	return residCovmat


def ekfRun_Hungarian(measList, trackingList, trackingCfg, QvecList, frameNumber):  # td is period time for frame

	trackingList.resetTrackerAssociated()
	freeTrackerList = []
	selectedMeasList = []
	nFree = 0
	numProc_ekf = 0
	nAssociated = 0

	lenMeas = len(measList)
	lenTracker = len(trackingList.List)
	print("lentracker : ",lenTracker)
	print("lenMeas : ", lenMeas)
	if lenMeas > lenTracker:
		lenList = lenMeas
	else:
		lenList = lenTracker

	matrixEdgeWeights = 99999999 * np.ones((lenList, lenList), dtype = np.float)
	trackerIdx = -1


	for currTrack in trackingList.List[:]:
		trackerIdx += 1
		if numProc_ekf >= trackingCfg.maxTracker:
			break
		else:
			numProc_ekf += 1

		if ((currTrack.validity == False) or not(isWithinBoundingBox(currTrack, trackingCfg))):
			nFree = invalidateCurrTrack(currTrack, nFree, freeTrackerList)
			continue

		if stateVecTimeUpdate(currTrack, trackingCfg.td) == False:  # F is time update matrix 여기서 currTrack.stateVectorXYZ가 predicted state가 된다.
			nFree = invalidateCurrTrack(currTrack, nFree, freeTrackerList)
			continue

		Qmat = select_QVec_and_cal_Qmat(QvecList, currTrack)

		stateCovmatTimeUpdate(currTrack, trackingCfg.td, Qmat)

		tmpPredictedMeas, isValidity = computePredictedMeas(currTrack.stateVectorXYZ)  # transform the cartesian coordinate to rrd coodrinate (stateVectorXYZ -> stateVectorRRD)
		if CONST.IMM:
			pass
		Hmat = computeHmat(currTrack.stateVectorXYZ, tmpPredictedMeas)
		residCovmat_ = residualCovarianceComputation_notAddR(currTrack, Hmat)

		if isValidity == False:
			nFree = invalidateCurrTrack(currTrack, nFree, freeTrackerList)
			continue

		distMetricMin = 655350.0
		iAssocMeas = None

		measIdx = -1
		for currMeas in measList[:]:
			measIdx += 1


			invResidCovmat_ = symMatInv_addR(residCovmat_, currMeas.measCovVec)
			pdistSq, canBeAssociated = isTargetWithinDataAssociationThresh_mahalanobis(currMeas, tmpPredictedMeas,
				                                                               currTrack.stateVectorXYZ, currTrack,
				                                                               trackingCfg, invResidCovmat_)

			matrixEdgeWeights[trackerIdx][measIdx] = pdistSq

			if canBeAssociated == True:

				if pdistSq < distMetricMin:

					distMetricMin = pdistSq
					iAssocMeas = currMeas

					if CONST.Mahalnobis_Association:
						pinvHmat(currTrack, invResidCovmat_, Hmat)



		if iAssocMeas != None:
			currTrack.associatedObj = iAssocMeas
			iAssocMeas.isAssociated = True

		if currTrack.associatedObj != None:

			measurmentResidual = computeMeasResidual(currTrack, tmpPredictedMeas)
			# residCovmat_org = computeResidCovmat(Hmat, currTrack.Covariance, currTrack.associatedObj.measCovVec,
			#                                  currTrack.ID)
			residCovmat = computeResidCovmat_addR(residCovmat_, currTrack.associatedObj.measCovVec)
			# print(residCovmat, "\n", residCovmat_org)
			invResidCovmat = np.linalg.inv(residCovmat)
			# print(invResidCovmat, "\n", invResidCovmat_)
			# invResidCovmat = invResidCovmat_
			isResidCovmatInversible = np.allclose(np.matmul(residCovmat, invResidCovmat), np.eye(N_MEAS))
			# isResidCovmatInversible = np.allclose(np.matmul(residCovmat_, invResidCovmat_), np.eye(N_MEAS))
			# isResidCovmatInversible = True
			if isResidCovmatInversible:

				kalmanGain = kalmanGainComputation(currTrack, Hmat, invResidCovmat)

				if stateVecMeasurementUpdate(currTrack, kalmanGain, measurmentResidual) == False:
					nFree = invalidateCurrTrack(currTrack, nFree, freeTrackerList)
					continue

				stateCovmatMeasurementUpdate(currTrack, kalmanGain, Hmat)

				if CONST.trackingLogDEBUG and currTrack.ID == CONST.RCS_DEBUG_ID_NUMBER:
					logList.append((frameNumber, currTrack.ID, currTrack.associatedObj.measVectorRRD[0], \
					                currTrack.stateVectorXYZ[0], \
					                currTrack.stateVectorXYZ[1], \
					                currTrack.stateVectorXYZ[2],\
					                currTrack.stateVectorXYZ[3],\
					                currTrack.associatedObj.stateVectorXYZ[0], \
					                currTrack.associatedObj.measVectorRRD[1], \
					                currTrack.associatedObj.measVectorRRD[2], \
					                np.arcsin(currTrack.associatedObj.measVectorRRD[2]) * 180 /np.pi, \
					                currTrack.associatedObj.angleSNR * 6,\
					                currTrack.associatedObj.rangeSNR * 6,\
					                currTrack.Covariance[0][0],\
					                currTrack.Covariance[1][1],\
					                currTrack.Covariance[2][2],\
					                currTrack.Covariance[3][3],\
					                currTrack.associatedObj.measCovVec[0],\
					                currTrack.associatedObj.measCovVec[1],\
					                currTrack.associatedObj.measCovVec[2]\

					                ))

				nAssociated += 1

				if currTrack.age > 1:
					currTrack.age = 1
				else:
					currTrack.age = 0

				if currTrack.tick < trackingCfg.maxTick:
					if currTrack.stateVectorXYZ[1] > 80 and currTrack.tick < trackingCfg.maxTick:
						currTrack.tick += 1
					elif not(currTrack.stateVectorXYZ[1] > 80) and currTrack.tick < trackingCfg.maxTick -1:
						currTrack.tick += 2
				if CONST.SIZE_OSCILLATION:
					currTrack.xSize = (15 * currTrack.xSize + currTrack.associatedObj.xSize) / 16
					currTrack.ySize = (15 * currTrack.ySize + currTrack.associatedObj.ySize) / 16
				else:
					currTrack.xSize = (7 * currTrack.xSize + currTrack.associatedObj.xSize) / 8
					currTrack.ySize = (7 * currTrack.ySize + currTrack.associatedObj.ySize) / 8
				currTrack.RCS = currTrack.associatedObj.RCS
				currTrack.RCS_variation = currTrack.associatedObj.RCS_variation
				currTrack.RCS_slope = currTrack.associatedObj.RCS_slope
				currTrack.SNR = currTrack.associatedObj.SNR

			else:

				currTrack.associatedObj = None
				iAssocMeas.isAssociated = False

		if currTrack.associatedObj == None :

			if CONST.TRACKING_AGE_FUNCTION:
				if currTrack.tick == 0 or currTrack.age > 2:
					currTrack.age += 2
				# elif currTrack.age > 5:
				# 	currTrack.age +=2
				else:
					currTrack.age += 1
			else:
				currTrack.age += 1
			if CONST.TRACKING_AGE_FUNCTION:

				if currTrack.tick > 1 and currTrack.age > 3:
					currTrack.tick -= 2

				elif currTrack.tick > 0 and currTrack.age > 1:
					currTrack.tick -= 1

			else:
				if currTrack.tick > 0 and currTrack.age > 1:
					currTrack.tick -= 1

			if currTrack.tick < trackingCfg.theresholdTick:
				currTrack.plotValidity = False

			if currTrack.age > trackingCfg.maxAge:
				nFree = invalidateCurrTrack(currTrack, nFree, freeTrackerList)

	# print(matrixEdgeWeights, lenList)

	hA.hungarianAlgorithm(matrixEdgeWeights, lenList)

	if nFree > 0:
		for currTrack in freeTrackerList[:]:
			trackingList.rmObj(currTrack)

	nFree = trackingCfg.maxTracker - len(trackingList.List)

	numNewTracks = selectMeas(measList, nFree, len(measList) - nAssociated, len(measList), selectedMeasList, trackingCfg)


	for currMeas in selectedMeasList:

		if len(trackingList.List) > trackingCfg.maxTracker: # tracking 개수 30로 한정
			break
		currMeas.validity = True
		if CONST.TRACKING_INPUT_MIN_POINT_MODIFY:
			if currMeas.pointNum < 2:
				continue
		#append currMeas to currTracklist
		tmpTrack = copy.deepcopy(currMeas)
		initNewTracker(tmpTrack)
		# tmpTrack.tick = 1
		# print(tmpTrack.stateVectorXYZ)

		trackingList.appendDetObj(tmpTrack)
		# print(trackingList.List)
	trackingList.arrangeTracksByAge()

def interaction(currTrack):
	for idx, state in enumerate(currTrack.state):
		state.initCondstate = currTrack.state[0].stateVectorXYZ * currTrack.mixingProb[0][idx] + currTrack.state[1].stateVectorXYZ * currTrack.mixingProb[1][idx]
		state.initCondCov = currTrack.mixingProb[0][idx] * (currTrack.state[0].Covariance + np.dot(currTrack.state[0].stateVectorXYZ - state.initCondstate, currTrack.state[0].stateVectorXYZ - state.initCondstate))\
							+ currTrack.mixingProb[1][idx] * (currTrack.state[1].Covariance + np.dot(currTrack.state[1].stateVectorXYZ - state.initCondstate, currTrack.state[1].stateVectorXYZ - state.initCondstate))


def ekfRun(measList, trackingList, trackingCfg, QvecList, frameNumber):  # td is period time for frame

	trackingList.resetTrackerAssociated()
	freeTrackerList = []
	selectedMeasList = []
	nFree = 0
	numProc_ekf = 0
	nAssociated = 0

	for currTrack in trackingList.List[:]:

		if numProc_ekf >= trackingCfg.maxTracker:
			break
		else:
			numProc_ekf += 1

		if ((currTrack.validity == False) or not(isWithinBoundingBox(currTrack, trackingCfg))):
			nFree = invalidateCurrTrack(currTrack, nFree, freeTrackerList)
			continue

		canBeAssociated = 0
		if CONST.ekfDEBUG:
			if currTrack.ID == CONST.RCS_DEBUG_ID_NUMBER:
				print("Pk_pre_frame")
				print(currTrack.Covariance)

		# print("before", prevStateVec, currTrack.stateVectorXYZ)
		if stateVecTimeUpdate(currTrack, trackingCfg.td) == False:  # F is time update matrix 여기서 currTrack.stateVectorXYZ가 predicted state가 된다.
			nFree = invalidateCurrTrack(currTrack, nFree, freeTrackerList)
			continue
		if CONST.ekfDEBUG:
			if currTrack.ID == CONST.RCS_DEBUG_ID_NUMBER:
				print("xkHat_minus\n", currTrack.stateVectorXYZ)
		Qmat = select_QVec_and_cal_Qmat(QvecList, currTrack)
		if CONST.calMatrixTISDK:

			if CONST.calMatrixTISDKDEBUG and (currTrack.ID == CONST.RCS_DEBUG_ID_NUMBER):

				tmpTracker_forDEBUG = copy.deepcopy(currTrack)
				stateCovmatTimeUpdate(tmpTracker_forDEBUG, trackingCfg.td, Qmat)
				print("stateCovmatTimeUpdate_currTrack.Covariance_numPy")
				print(tmpTracker_forDEBUG.Covariance)

				stateCovmatTimeUpdate_TI(currTrack, trackingCfg.td, Qmat)
				print("stateCovmatTimeUpdate_currTrack.Covariance_TI")
				print(currTrack.Covariance)
			else:
				stateCovmatTimeUpdate_TI(currTrack, trackingCfg.td, Qmat)
		else:
			stateCovmatTimeUpdate(currTrack, trackingCfg.td, Qmat)

		if CONST.Mahalnobis_Association:
			tmpPredictedMeas, isValidity = computePredictedMeas(currTrack.stateVectorXYZ)  # transform the cartesian coordinate to rrd coodrinate (stateVectorXYZ -> stateVectorRRD)
			Hmat = computeHmat(currTrack.stateVectorXYZ, tmpPredictedMeas)
			residCovmat_ = residualCovarianceComputation_notAddR(currTrack, Hmat)
			if CONST.ekfDEBUG:
				if currTrack.ID == CONST.RCS_DEBUG_ID_NUMBER:
					print("residCovmat_\n")
					print(residCovmat_)
					print("\n")
		if CONST.ekfDEBUG:
			if currTrack.ID == CONST.RCS_DEBUG_ID_NUMBER:
				print("Pk_before_kalmanUpdate")
				print(currTrack.Covariance)
		if not CONST.Mahalnobis_Association:
			tmpPredictedMeas, isValidity = computePredictedMeas(currTrack.stateVectorXYZ)  # transform the cartesian coordinate to rrd coodrinate (stateVectorXYZ -> stateVectorRRD)
		if CONST.ekfDEBUG:
			if currTrack.ID == CONST.RCS_DEBUG_ID_NUMBER:
				print("zkHat_minus\n", tmpPredictedMeas)
		if isValidity == False:
			nFree = invalidateCurrTrack(currTrack, nFree, freeTrackerList)
			continue

		distMetricMin = 655350.0
		iAssocMeas = None

		for currMeas in measList[:]:
			canBeAssociated = False
			if currTrack.associatedObj == None and not (currMeas.isAssociated):
				if CONST.Mahalnobis_Association:

					invResidCovmat_ = symMatInv_addR(residCovmat_, currMeas.measCovVec)
					pdistSq, canBeAssociated = isTargetWithinDataAssociationThresh_mahalanobis(currMeas, tmpPredictedMeas,
						                                                               currTrack.stateVectorXYZ, currTrack,
						                                                               trackingCfg, invResidCovmat_)

				elif CONST.DA_Only_RangeVel:
					pdistSq, canBeAssociated = isTargetWithinDataAssociationThresh_DA_Only_RangeVel(currMeas, tmpPredictedMeas,
					                                                               currTrack.stateVectorXYZ, currTrack,
					                                                               trackingCfg)
				else:

					pdistSq, canBeAssociated = isTargetWithinDataAssociationThresh(currMeas, tmpPredictedMeas,
						                                                       currTrack.stateVectorXYZ, currTrack,
						                                                       trackingCfg)
				if CONST.isTargetWithinDataAssociationThreshDEBUG and currTrack.ID == CONST.RCS_DEBUG_ID_NUMBER and pdistSq != None:
					print(
						"pdistSq with the current track (ID, x, y, xd, xy) = (%d, %.2f, %.2f, %.2f, %.2f) and the measurement point(x, y, xd, xy) = (%.2f, %.2f, %.2f, %.2f) is %f."
						% (currTrack.ID, currTrack.stateVectorXYZ[0], currTrack.stateVectorXYZ[1],
						   currTrack.stateVectorXYZ[2], currTrack.stateVectorXYZ[3],
						   currMeas.stateVectorXYZ[0], currMeas.stateVectorXYZ[1], currMeas.stateVectorXYZ[2],
						   currMeas.stateVectorXYZ[3], pdistSq))


				if canBeAssociated == True:

					if pdistSq < distMetricMin:
						distMetricMin = pdistSq
						iAssocMeas = currMeas
						if CONST.Mahalnobis_Association:
							pinvHmat(currTrack, invResidCovmat_, Hmat)

		if iAssocMeas != None:
			currTrack.associatedObj = iAssocMeas
			iAssocMeas.isAssociated = True

		if currTrack.associatedObj != None:


				# print("tracking Input : ")
			# print("currTrack.associatedObj",currTrack.associatedObj.measVectorRRD)
			if not CONST.Mahalnobis_Association:
				Hmat = computeHmat(currTrack.stateVectorXYZ, tmpPredictedMeas)
			# compute measurmentResidual for currTrack.assocciatedObj.
			# Rmat = np.eye(3)*1 #clusteringDBscan.c 의 clusteringDBscan_calcInfoFixed() 에서 구하네


			measurmentResidual = computeMeasResidual(currTrack, tmpPredictedMeas)
			# measurmentResidual = [0,0,0]
			# print(currMeas.measCovVec)
			# residualCovariance = H * currTrack.Covariance * HT + R
			residCovmat = computeResidCovmat(Hmat, currTrack.Covariance, currTrack.associatedObj.measCovVec, currTrack.ID)

			if CONST.ekfDEBUG:
				if currTrack.ID == CONST.RCS_DEBUG_ID_NUMBER:
					# print("H * x")
					# print(np.dot(Hmat, currTrack.stateVectorXYZ )) #왜 sin이 0이지.. 극좌표계에서 theta 방향은 각에 수직하기 때문에 theta방향 값은 항상 0임
					print("residCovmat : H_k * P_k * H_k.T + R_k ")
					print(residCovmat)

			# have to check residCovmat is inversible matrix.

			invResidCovmat = np.linalg.inv(residCovmat)

			isResidCovmatInversible = np.allclose(np.matmul(residCovmat, invResidCovmat), np.eye(N_MEAS))

			if isResidCovmatInversible:

				kalmanGain = kalmanGainComputation(currTrack, Hmat, invResidCovmat)

				if stateVecMeasurementUpdate(currTrack, kalmanGain, measurmentResidual) == False:
					nFree = invalidateCurrTrack(currTrack, nFree, freeTrackerList)
					continue

				stateCovmatMeasurementUpdate(currTrack, kalmanGain, Hmat)

				if CONST.trackingLogDEBUG and currTrack.ID == CONST.RCS_DEBUG_ID_NUMBER:
					logList.append((frameNumber, currTrack.ID, currTrack.associatedObj.measVectorRRD[0], \
					                currTrack.stateVectorXYZ[0], \
					                currTrack.stateVectorXYZ[1], \
					                currTrack.stateVectorXYZ[2],\
					                currTrack.stateVectorXYZ[3],\
					                currTrack.associatedObj.stateVectorXYZ[0], \
					                currTrack.associatedObj.measVectorRRD[1], \
					                currTrack.associatedObj.measVectorRRD[2], \
					                np.arcsin(currTrack.associatedObj.measVectorRRD[2]) * 180 /np.pi, \
					                currTrack.associatedObj.angleSNR * 6,\
					                currTrack.associatedObj.rangeSNR * 6,\
					                currTrack.Covariance[0][0],\
					                currTrack.Covariance[1][1],\
					                currTrack.Covariance[2][2],\
					                currTrack.Covariance[3][3],\
					                currTrack.associatedObj.measCovVec[0],\
					                currTrack.associatedObj.measCovVec[1],\
					                currTrack.associatedObj.measCovVec[2]))

				if CONST.ekfDEBUG:
					if currTrack.ID == CONST.RCS_DEBUG_ID_NUMBER:
						print("Pk")
						print(currTrack.Covariance, "\n")
				nAssociated += 1

				if currTrack.age > 1:
					currTrack.age = 1
				else:
					currTrack.age = 0

				if currTrack.tick < trackingCfg.maxTick:
					if currTrack.stateVectorXYZ[1] > 80 and currTrack.tick < trackingCfg.maxTick:
						currTrack.tick += 1
					elif not(currTrack.stateVectorXYZ[1] > 80) and currTrack.tick < trackingCfg.maxTick -1:
						currTrack.tick += 2
				if CONST.SIZE_OSCILLATION:
					currTrack.xSize = (15 * currTrack.xSize + currTrack.associatedObj.xSize) / 16
					currTrack.ySize = (15 * currTrack.ySize + currTrack.associatedObj.ySize) / 16
				else:
					currTrack.xSize = (7 * currTrack.xSize + currTrack.associatedObj.xSize) / 8
					currTrack.ySize = (7 * currTrack.ySize + currTrack.associatedObj.ySize) / 8
				currTrack.RCS = currTrack.associatedObj.RCS
				currTrack.RCS_variation = currTrack.associatedObj.RCS_variation
				currTrack.RCS_slope = currTrack.associatedObj.RCS_slope
				currTrack.SNR = currTrack.associatedObj.SNR

			else:

				currTrack.associatedObj = None
				iAssocMeas.isAssociated = False

		if currTrack.associatedObj == None :

			if CONST.TRACKING_AGE_FUNCTION:
				if currTrack.tick == 0 or currTrack.age > 2:
					currTrack.age += 2
				# elif currTrack.age > 5:
				# 	currTrack.age +=2
				else:
					currTrack.age += 1
			else:
				currTrack.age += 1
			if CONST.TRACKING_AGE_FUNCTION:

				if currTrack.tick > 1 and currTrack.age > 3:
					currTrack.tick -= 2

				elif currTrack.tick > 0 and currTrack.age > 1:
					currTrack.tick -= 1

			else:
				if currTrack.tick > 0 and currTrack.age > 1:
					currTrack.tick -= 1

			if currTrack.tick < trackingCfg.thresholdTick:
				currTrack.plotValidity = False

			if currTrack.age > trackingCfg.maxAge:
				nFree = invalidateCurrTrack(currTrack, nFree, freeTrackerList)

	if nFree > 0:
		for currTrack in freeTrackerList[:]:
			trackingList.rmObj(currTrack)

	nFree = trackingCfg.maxTracker - len(trackingList.List)

	numNewTracks = selectMeas(measList, nFree, len(measList) - nAssociated, len(measList), selectedMeasList, trackingCfg)


	for currMeas in selectedMeasList:

		if len(trackingList.List) > trackingCfg.maxTracker: # tracking 개수 30로 한정
			break
		currMeas.validity = True
		if CONST.TRACKING_INPUT_MIN_POINT_MODIFY:
			if currMeas.pointNum < 2:
				continue
		#append currMeas to currTracklist
		tmpTrack = copy.deepcopy(currMeas)
		initNewTracker(tmpTrack)
		# tmpTrack.tick = 1
		# print(tmpTrack.stateVectorXYZ)dasda

		trackingList.appendDetObj(tmpTrack)
		# print(trackingList.List)
	trackingList.arrangeTracksByAge()



def symMatInv_addR(residCovmat_, measCovVec):

	Rmat = np.zeros((N_MEAS, N_MEAS), dtype=np.float)
	for i in range(N_MEAS):
		Rmat[i][i] = measCovVec[i]

	invResidCovmat_ = np.linalg.inv(residCovmat_ + Rmat)

	return invResidCovmat_
def residualCovarianceComputation_notAddR(currTrack, Hmat):
	tmp = copy.deepcopy(currTrack.Covariance)
	# A = np.linalg.inv(tmp)
	# Rmat = np.zeros((N_MEAS, N_MEAS), dtype=np.float)
	# for i in range(N_MEAS):
	# 	Rmat[i][i] = currTrack.measCovVec[i]
	# invHmat = np.linalg.pinv(Hmat)
	# print(invHmat)
	tmp = np.matmul(Hmat, tmp)
	# print(Hmat, "\n",Hmat.T)
	tmp = np.matmul(tmp, Hmat.T)

	return tmp

def kalmanGainComputation_TI(currTrack, Hmat, invResidCovmat_mat):
	temp = np.zeros(12, dtype= np.float)
	state_covmat = np.zeros(10, dtype= np.float)
	invResidCovmat = np.zeros(6, dtype= np.float)
	kalmanGainMatrix = np.zeros((4,3), dtype= np.float)
	kalmanGain = np.zeros(12, dtype = np.float)


	state_covmat[iXX]   =  currTrack.Covariance[0][0]
	state_covmat[iXY]   =  currTrack.Covariance[0][1]
	state_covmat[iXXd]  =  currTrack.Covariance[0][2]
	state_covmat[iXYd]  =  currTrack.Covariance[0][3]
	state_covmat[iYY]   =  currTrack.Covariance[1][1]
	state_covmat[iYXd]  =  currTrack.Covariance[1][2]
	state_covmat[iYYd]  =  currTrack.Covariance[1][3]
	state_covmat[iXdXd] =  currTrack.Covariance[2][2]
	state_covmat[iXdYd] =  currTrack.Covariance[2][3]
	state_covmat[iYdYd] =  currTrack.Covariance[3][3]

	invResidCovmat[iRR] = invResidCovmat_mat[0][0]
	invResidCovmat[iRRd] = invResidCovmat_mat[0][1]
	invResidCovmat[iRAz] = invResidCovmat_mat[0][2]
	invResidCovmat[iRdRd] = invResidCovmat_mat[1][1]
	invResidCovmat[iRdAz] = invResidCovmat_mat[1][2]
	invResidCovmat[iAzAz] = invResidCovmat_mat[2][2]

	A = Hmat[0][0]
	B = Hmat[0][1]
	C = Hmat[1][0]
	D = Hmat[1][1]
	E = Hmat[2][0]
	F = Hmat[2][1]


	temp[0] = A * invResidCovmat[iRR] + C * invResidCovmat[iRRd] + E * invResidCovmat[iRAz]
	temp[1] = B * invResidCovmat[iRR] + D * invResidCovmat[iRRd] + F * invResidCovmat[iRAz]
	temp[2] = A * invResidCovmat[iRRd]
	temp[3] = B * invResidCovmat[iRRd]
	temp[4] = C * invResidCovmat[iRdRd] + E * invResidCovmat[iRdAz] + A * invResidCovmat[iRRd]
	temp[5] = D * invResidCovmat[iRdRd] + F * invResidCovmat[iRdAz] + B * invResidCovmat[iRRd]
	temp[6] = A * invResidCovmat[iRdRd]
	temp[7] = B * invResidCovmat[iRdRd]
	temp[8] = C * invResidCovmat[iRdAz] + A * invResidCovmat[iRAz] + E * invResidCovmat[iAzAz]
	temp[9] = D * invResidCovmat[iRdAz] + B * invResidCovmat[iRAz] + F * invResidCovmat[iAzAz]
	temp[10] = A * invResidCovmat[iRdAz]
	temp[11] = B * invResidCovmat[iRdAz]


	kalmanGain[0] = state_covmat[iXYd] * temp[3] + state_covmat[iXXd] * temp[2] + state_covmat[iXY] * temp[1] + \
	                state_covmat[iXX] * temp[0]
	kalmanGain[1] = state_covmat[iYYd] * temp[3] + state_covmat[iYXd] * temp[2] + state_covmat[iYY] * temp[1] + \
	                state_covmat[iXY] * temp[0]
	kalmanGain[2] = state_covmat[iXdYd] * temp[3] + state_covmat[iXdXd] * temp[2] + state_covmat[iYXd] * temp[1] + \
	                state_covmat[iXXd] * temp[0]
	kalmanGain[3] = state_covmat[iYdYd] * temp[3] + state_covmat[iXdYd] * temp[2] + state_covmat[iYYd] * temp[1] + \
	                state_covmat[iXYd] * temp[0]
	kalmanGain[4] = state_covmat[iXYd] * temp[7] + state_covmat[iXXd] * temp[6] + state_covmat[iXY] * temp[5] + \
	                state_covmat[iXX] * temp[4]
	kalmanGain[5] = state_covmat[iYYd] * temp[7] + state_covmat[iYXd] * temp[6] + state_covmat[iYY] * temp[5] + \
	                state_covmat[iXY] * temp[4]
	kalmanGain[6] = state_covmat[iXdYd] * temp[7] + state_covmat[iXdXd] * temp[6] + state_covmat[iYXd] * temp[5] + \
	                state_covmat[iXXd] * temp[4]
	kalmanGain[7] = state_covmat[iYdYd] * temp[7] + state_covmat[iXdYd] * temp[6] + state_covmat[iYYd] * temp[5] + \
	                state_covmat[iXYd] * temp[4]
	kalmanGain[8] = state_covmat[iXY] * temp[9] + state_covmat[iXX] * temp[8] + state_covmat[iXYd] * temp[11] + \
	                state_covmat[iXXd] * temp[10]
	kalmanGain[9] = state_covmat[iYY] * temp[9] + state_covmat[iXY] * temp[8] + state_covmat[iYYd] * temp[11] + \
	                state_covmat[iYXd] * temp[10]
	kalmanGain[10] = state_covmat[iYXd] * temp[9] + state_covmat[iXXd] * temp[8] + state_covmat[iXdYd] * temp[11] + \
	                 state_covmat[iXdXd] * temp[10]
	kalmanGain[11] = state_covmat[iYYd] * temp[9] + state_covmat[iXYd] * temp[8] + state_covmat[iYdYd] * temp[11] + \
	                 state_covmat[iXdYd] * temp[10]

	kalmanGainMatrix[0][0] = kalmanGain[0]
	kalmanGainMatrix[1][0] = kalmanGain[1]
	kalmanGainMatrix[2][0] = kalmanGain[2]
	kalmanGainMatrix[3][0] = kalmanGain[3]
	kalmanGainMatrix[0][1] = kalmanGain[4]
	kalmanGainMatrix[1][1] = kalmanGain[5]
	kalmanGainMatrix[2][1] = kalmanGain[6]
	kalmanGainMatrix[3][1] = kalmanGain[7]
	kalmanGainMatrix[0][2] = kalmanGain[8]
	kalmanGainMatrix[1][2] = kalmanGain[9]
	kalmanGainMatrix[2][2] = kalmanGain[10]
	kalmanGainMatrix[3][2] = kalmanGain[11]

	return kalmanGainMatrix

def residualCovarianceComputation(Hmat, currTrack_cov, measCovVec, currTrackID):
	residCovmat =  np.zeros((3,3), dtype= np.float)
	state_covmat = np.zeros(10, dtype= np.float)

	state_covmat[iXX] = currTrack_cov[0][0]
	state_covmat[iXY] = currTrack_cov[0][1]
	state_covmat[iXXd] = currTrack_cov[0][2]
	state_covmat[iXYd] = currTrack_cov[0][3]
	state_covmat[iYY] = currTrack_cov[1][1]
	state_covmat[iYXd] = currTrack_cov[1][2]
	state_covmat[iYYd] = currTrack_cov[1][3]
	state_covmat[iXdXd] = currTrack_cov[2][2]
	state_covmat[iXdYd] = currTrack_cov[2][3]
	state_covmat[iYdYd] = currTrack_cov[3][3]


	A = Hmat[0][0]
	B = Hmat[0][1]
	C = Hmat[1][0]
	D = Hmat[1][1]
	E = Hmat[2][0]
	F = Hmat[2][1]

	Asq = A * A
	Bsq = B * B
	Csq = C * C
	Dsq = D * D
	Esq = E * E
	Fsq = F * F

	residCovmat[0][0] = Bsq * state_covmat[iYY] + 2 * A * B * state_covmat[iXY] + Asq * state_covmat[iXX];
	residCovmat[0][1] = B * D * state_covmat[iYY] + Bsq * state_covmat[iYYd] + A * B * state_covmat[iYXd] + (
				A * D + B * C) * state_covmat[iXY] + A * B * state_covmat[iXYd] + A * C * state_covmat[iXX] + Asq * \
	                    state_covmat[iXXd];
	residCovmat[0][2] = B * F * state_covmat[iYY] + (A * F + B * E) * state_covmat[iXY] + A * E * state_covmat[iXX];
	residCovmat[1][1] = Bsq * state_covmat[iYdYd] + Dsq * state_covmat[iYY] + 2 * B * D * state_covmat[
		iYYd] + 2 * A * D * state_covmat[iYXd] + 2 * A * B * state_covmat[iXdYd] + Asq * state_covmat[
		                     iXdXd] + 2 * C * D * state_covmat[iXY] + 2 * B * C * state_covmat[iXYd] + Csq * \
	                     state_covmat[iXX] + 2 * A * C * state_covmat[iXXd];
	residCovmat[1][2] = D * F * state_covmat[iYY] + B * F * state_covmat[iYYd] + A * F * state_covmat[iYXd] + (
				C * F + D * E) * state_covmat[iXY] + B * E * state_covmat[iXYd] + C * E * state_covmat[iXX] + A * E * \
	                     state_covmat[iXXd];
	residCovmat[2][2] = Fsq * state_covmat[iYY] + 2 * E * F * state_covmat[iXY] + Esq * state_covmat[iXX];

	residCovmat[1][0] = residCovmat[0][1]
	residCovmat[2][0] = residCovmat[0][2]
	residCovmat[2][1] = residCovmat[1][2]
	print("measCovVec", measCovVec)
	print("\n\n")
	print("residCovmat123123\n", residCovmat)
	print("\n")
	print("measCovVec\n", measCovVec)
	print("\n")
	residCovmat[0][0] += measCovVec[0];
	residCovmat[1][1] += measCovVec[1];
	residCovmat[2][2] += measCovVec[2];

	return residCovmat

def symMatInv(residCovmat):
	invResidCovmat = np.zeros((3,3), dtype= np.float)
	inv = np.zeros(6, dtype = np.float)
	m = np. zeros(6, dtype = np.float)

	m[0] = residCovmat[0][0]
	m[1] = residCovmat[0][1]
	m[2] = residCovmat[0][2]
	m[3] = residCovmat[1][1]
	m[4] = residCovmat[1][2]
	m[5] = residCovmat[2][2]

	inv[0] = (m[3] * m[5]) - (m[4] * m[4])
	inv[1] = (m[4] * m[2]) - (m[1] * m[5])
	inv[2] = (m[1] * m[4]) - (m[3] * m[2])
	inv[3] = (m[0] * m[5]) - (m[2] * m[2])
	inv[4] = (m[1] * m[2]) - (m[0] * m[4])
	inv[5] = (m[0] * m[3]) - (m[1] * m[1])

	det = m[0] * inv[0] + m[1] * inv[1] + m[2] * inv[2]

	if det > 0.00001 :

		invDet = 1 / det

		inv = inv * invDet

		invResidCovmat[0][0] = inv[0]
		invResidCovmat[0][1] = inv[1]
		invResidCovmat[1][0] = inv[1]
		invResidCovmat[0][2] = inv[2]
		invResidCovmat[2][0] = inv[2]
		invResidCovmat[1][1] = inv[3]
		invResidCovmat[1][2] = inv[4]
		invResidCovmat[2][1] = inv[4]
		invResidCovmat[2][2] = inv[5]

		return invResidCovmat, True
	else:
		return invResidCovmat, False

#======= orig
	# for currMeas in measList:
	# 	# print(currMeas.r)
	# 	if len(trackingList.List) > trackingCfg.maxTracker:  # tracking 개수 30로 한정
	# 		break
	# 	if currMeas.isAssociated == False:
	# 		# append currMeas to currTracklist
	# 		tmpTrack = copy.deepcopy(currMeas)
	# 		initNewTracker(tmpTrack)
	# 		# tmpTrack.tick = 1
	# 		# print(tmpTrack.stateVectorXYZ)
	#
	# 		trackingList.appendDetObj(tmpTrack)

def findIDforDEBUG(cfarOutNDList, targetIdxPlotSpec):
	# for obj in cfarOutNDList:
	# 	print("cfarOut3DList ID :", obj.detID)

	tmpDetID = None
	for idx, obj in enumerate(cfarOutNDList):
		if obj.detID == targetIdxPlotSpec:
			tmpDetID = idx
	# print("out : ", tmpDetID)
	return tmpDetID


def findIDlistforDEBUG(cfarOutNDList, targetIdxPlotSpec):
	# for obj in cfarOutNDList:
	# 	print("cfarOut3DList ID :", obj.detID)

	tmpDetIDList = None
	tmpDetIDList = [obj for obj in cfarOutNDList if obj.detID == targetIdxPlotSpec]
	# print("out : ", tmpDetID)
	return tmpDetIDList


def conversion(object_list, RadarInfo, vehicle_speed, vehicle_steer_angle):
	cfarOut3DList = []

	mode = RadarInfo.mode
	remove_ang = RadarInfo.StaticRemoveAngle

	tmpIdx = 1
	for obj in object_list:
		tmpObj = DT.cfarOutFmt3D()

		tmpObj.detID = tmpIdx
		tmpObj.rangeIdx = obj.range_idx
		tmpObj.dopplerIdx = obj.doppler_idx
		tmpObj.range = obj.range
		tmpObj.speed = obj.speed

		if obj.sin_azim > 1:
			obj.sin_azim = 1
		elif obj.sin_azim < -1:
			obj.sin_azim = -1
		else:
			obj.sin_azim = obj.sin_azim

		tmpObj.rangeVal = obj.peak_val
		tmpObj.rangeSNR = obj.range_snr_db
		tmpObj.dopplerSNR = obj.doppler_snr_db
		tmpObj.angleSNR = obj.sin_azim_srn_lin
		tmpObj.x = obj.x
		tmpObj.y = obj.y

		tmpObj.rotatex = obj.rotate_x
		tmpObj.rotatey = obj.rotate_y

		tmpObj.velDisambFacValidity = obj.vel_disamb_fac_valid
#		tmpObj.statusFlag = obj.status_flag

		obj_ang_rad = math.atan2(tmpObj.x, tmpObj.y)
		tmpObj.sinAzim = np.sin(obj_ang_rad)
		if CONST.SPEED_COMP_BY_ANGLE:
			tmpObj.xd = tmpObj.speed * np.sin(obj_ang_rad)
			tmpObj.yd = tmpObj.speed * np.cos(obj_ang_rad)
		else:
			tmpObj.xd = 0
			tmpObj.yd = tmpObj.speed

		### HSLee 추가 2022.10.24 : Static Object Process - Object의 Flag 적용
#		static_angle = remove_ang
		static_angle = remove_ang + (obj_ang_rad * 180 / np.pi)
		if static_angle < -75.0:
			static_angle = -75.0
		elif static_angle > 75.0:
			static_angle = 75.0

		if vehicle_speed == 0.0:
			th_vel = 0.5  # (0.5)
		elif (0.0 < vehicle_speed) and (vehicle_speed < 5.0):
			th_vel = 1.2  # (1.2)
		else:
			th_vel = 2.4  # (3.3)

		if mode == 2:		# RCCW
#			staticVel_y = tmpObj.speed
			staticVel_y = tmpObj.speed / np.cos(static_angle * np.pi / 180)
			vehicleVel_y = vehicle_speed * np.cos(vehicle_steer_angle * np.pi / 180)
			Vel_y = staticVel_y + vehicleVel_y

			if (abs(Vel_y) <= th_vel) or (staticVel_y == 0.0):
				tmpObj.statusFlag = (tmpObj.statusFlag | 1)				 # Static Obj
			elif staticVel_y < 0.0:
				tmpObj.statusFlag = (tmpObj.statusFlag | 2)				 # Coming Obj
			else:		# staticVel_y > 0
				tmpObj.statusFlag = (tmpObj.statusFlag | 4)				 # Going Obj
		else:				# BCW or SEW
#			staticVel_y = tmpObj.speed
			staticVel_y = tmpObj.speed / np.cos(static_angle * np.pi / 180)
			vehicleVel_y = vehicle_speed * np.cos(vehicle_steer_angle * np.pi / 180)

			if (RadarInfo.position == 1 and tmpObj.rotatex >= 0) or (RadarInfo.position == 2 and tmpObj.rotatex <= 0):		# 전방 Obj
				Vel_y = staticVel_y + vehicleVel_y
			elif (RadarInfo.position == 1 and tmpObj.rotatex < 0) or (RadarInfo.position == 2 and tmpObj.rotatex > 0):		# 후방 Obj
				Vel_y = staticVel_y - vehicleVel_y
			else:
				Vel_y = staticVel_y - vehicleVel_y

#			th_vel = 3.3
			if abs(Vel_y) <= th_vel:
				tmpObj.statusFlag = (tmpObj.statusFlag | 1)				 # Static Obj
			elif staticVel_y < 0.0:
				tmpObj.statusFlag = (tmpObj.statusFlag | 2)				 # Coming Obj
			else:		# staticVel_y >= 0
				tmpObj.statusFlag = (tmpObj.statusFlag | 4)				 # BCW(or SEW) Obj (Coming or Going Obj)
		### HSLee 추가 2022.10.24 : Static Object Process - Object의 Flag 적용

		tmpIdx += 1
		cfarOut3DList.append(tmpObj)

	return cfarOut3DList
