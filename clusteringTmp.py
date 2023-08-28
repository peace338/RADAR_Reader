import numpy as np
import dataStructure as DT
import CONST
import math
from radarEquipInfo import MAX_Z, MIN_Z

DEBUG = False

def convertSNRdBToVar(SNRdB, n_samples, resolution):
	# print(SNRdB)
	scaleFac = n_samples*resolution
	invSNRlin = 1 / pow(2, SNRdB ) * 2 # board 에서 스펙트럼은 QFormat 8인데 7인걸로 연산 We assume our estimator is 3dB worse than the CRLB.
										# 근데 Qformat 8인거처럼 계산한게 더 잘맞네 확인해봐야겠다.
	fvar = invSNRlin * (6 / ((2 * np.pi) * (2 * np.pi))) /(n_samples*n_samples -1)
	Rvar = fvar * scaleFac * scaleFac
	resTresh = 2 * resolution * resolution
	# print("resTresh : %.2f, n_samples : %.2f" % (resTresh, n_samples))
	# print("before if : %.2f"%Rvar)
	if Rvar < resTresh:
		Rvar = resTresh
	# print("after if : %.2f"%Rvar)
	return Rvar


def convertSNRLinToVar(SNRdB, n_samples, resolution):
	if DEBUG:
		print("convertSNRLinToVar start")
		print("input SNRdB : %f" % (SNRdB * 6))

	invSNRlin = 1 / (2 ** SNRdB) # board 에서 스펙트럼은 QFormat 8인데 7인걸로 연산
	scaleFac = n_samples * resolution
	fVar = invSNRlin * (6 / ((2 * np.pi) * (2 * np.pi))) / (n_samples * n_samples - 1)

	Rvar = fVar * scaleFac * scaleFac
	if DEBUG:
		print("Rvar : %f"%Rvar)
	# print("before if : %.2f"%Rvar)
	# print("after if : %.2f"%Rvar)
	return Rvar


def getSigmatheta(SNRdB, n_samples, targetAzim, scale, maxSizeflt, targetRange):
	if DEBUG:
		print("convertSNRLinToVar start")
		print("input SNRdB : %f" % (SNRdB * 6))

	invSNRlin = 2 / (2 ** SNRdB) # board 에서 스펙트럼은 QFormat 8인데 7인걸로 연산
	# invSNRlin = 2 / (SNRdB)  # board 에서 스펙트럼은 QFormat 8인데 7인걸로 연산
	# resolution = 2  / (n_samples* (np.sqrt(1 - targetAzim**2)))
	resolution = 2 / (n_samples * np.sqrt(1 - targetAzim ** 2))
	fVar = invSNRlin * (resolution ** 2)
	# fVar = invSNRlin * (2 ** resolution)
	Rvar = fVar * scale * scale
	tmpThresh = maxSizeflt / targetRange
	tmpThresh = (tmpThresh ** 2)
	if Rvar < tmpThresh:
		Rvar = tmpThresh

	if DEBUG:
		print("Rvar_theta : %f"%Rvar)
	# print("before if : %.2f"%Rvar)
	# print("after if : %.2f"%Rvar)
	return Rvar


def clusteringDBscan_calcInfoFixed(clusterList, resList, clusteringCfg, domain):
	clusterOutputList = []

	for cluster in clusterList:
		lengthOfCluster = len(cluster)
		tmpClusterOutput = DT.clusterOutput()
		if CONST.trackingInputMODIFY:
			peakValSum = 0
			clusterLength = len(cluster)
			for obj in cluster:
				tmpClusterOutput.xCenter += obj.x
				tmpClusterOutput.yCenter += obj.y
				tmpClusterOutput.zCenter += obj.z
				tmpClusterOutput.avgVel += obj.speed

				# Cente of Mass
				# tmpClusterOutput.xCenter += obj.x * obj.rangeVal
				# tmpClusterOutput.yCenter += obj.y * obj.rangeVal
				# tmpClusterOutput.avgVel += obj.speed * obj.rangeVal
				# peakValSum += obj.rangeVal

			# tmpClusterOutput.xCenter = tmpClusterOutput.xCenter / peakValSum
			# tmpClusterOutput.yCenter = tmpClusterOutput.yCenter / peakValSum
			# tmpClusterOutput.avgVel = tmpClusterOutput.avgVel / peakValSum

			tmpClusterOutput.xCenter = tmpClusterOutput.xCenter / clusterLength
			tmpClusterOutput.yCenter = tmpClusterOutput.yCenter / clusterLength
			tmpClusterOutput.zCenter = tmpClusterOutput.zCenter / clusterLength
			tmpClusterOutput.avgVel  = tmpClusterOutput.avgVel / clusterLength

			trackingInputRangeAvg   = np.sqrt(tmpClusterOutput.xCenter*tmpClusterOutput.xCenter + tmpClusterOutput.yCenter * tmpClusterOutput.yCenter)
			trackingInputSpeedAvg   = tmpClusterOutput.avgVel
			trackingInputsinAzimAvg = tmpClusterOutput.xCenter / trackingInputRangeAvg

		else:
			for obj in cluster:  # obj Type : cfarOutFmt3D
				tmpClusterOutput.xCenter += obj.x
				tmpClusterOutput.yCenter += obj.y
				tmpClusterOutput.zCenter += obj.z
				tmpClusterOutput.avgVel += obj.speed

			tmpClusterOutput.xCenter = tmpClusterOutput.xCenter / lengthOfCluster
			tmpClusterOutput.yCenter = tmpClusterOutput.yCenter / lengthOfCluster
			tmpClusterOutput.zCenter = tmpClusterOutput.zCenter / lengthOfCluster
			tmpClusterOutput.avgVel = tmpClusterOutput.avgVel / lengthOfCluster

		peakVal = 0
		for obj in cluster:
			tmpSize = abs(obj.x - tmpClusterOutput.xCenter)
			if tmpSize > tmpClusterOutput.xSize:
				tmpClusterOutput.xSize = tmpSize

			tmpSize = abs(obj.y - tmpClusterOutput.yCenter)
			if tmpSize > tmpClusterOutput.ySize:
				tmpClusterOutput.ySize = tmpSize

			tmpSize = abs(obj.z - tmpClusterOutput.zCenter)
			if tmpSize > tmpClusterOutput.zSize:
				tmpClusterOutput.zSize = tmpSize

			if obj.rangeVal >= peakVal:
				peakVal = obj.rangeVal
				tmpClusterOutput.strongestMember = obj
		if CONST.DEFAULT_SIZE_MODIFY:
			if (tmpClusterOutput.xSize == 0) or (tmpClusterOutput.ySize == 0):
				tmpClusterOutput.xSize = 1 * resList[0]
				tmpClusterOutput.ySize = 1 * resList[0]
				tmpClusterOutput.zSize = 1 * resList[0]
			else:
				tmpClusterOutput.xSize += 0.5 * resList[0]
				tmpClusterOutput.ySize += 0.5 * resList[0]
				tmpClusterOutput.zSize += 0.5 * resList[0]
		else:
			if (tmpClusterOutput.xSize == 0) or (tmpClusterOutput.ySize == 0):
				tmpClusterOutput.xSize = 3 * resList[0]
				tmpClusterOutput.ySize = 3 * resList[0]
				tmpClusterOutput.zSize = 3 * resList[0]

		# ======================================================================================================================================================
		# populate trackingInput from tmpClusterOutput.strongestMember
		tmpClusterOutput.trackingInput.xSize = tmpClusterOutput.xSize
		tmpClusterOutput.trackingInput.ySize = tmpClusterOutput.ySize
		tmpClusterOutput.trackingInput.zSize = tmpClusterOutput.zSize

		# ======================================================================================================================================================
		if CONST.trackingInputMODIFY:
			tmpClusterOutput.trackingInput.measVectorRRD[0] = trackingInputRangeAvg
			tmpClusterOutput.trackingInput.measVectorRRD[1] = trackingInputSpeedAvg
			tmpClusterOutput.trackingInput.measVectorRRD[2] = trackingInputsinAzimAvg

		else:
			tmpClusterOutput.trackingInput.measVectorRRD[0] = tmpClusterOutput.strongestMember.range
			tmpClusterOutput.trackingInput.measVectorRRD[1] = tmpClusterOutput.strongestMember.speed
			tmpClusterOutput.trackingInput.measVectorRRD[2] = tmpClusterOutput.strongestMember.sinAzim

		# ======================================================================================================================================================
		tmpClusterOutput.clusterId = cluster[0].clusterId
		tmpClusterOutput.trackingInput.ID = tmpClusterOutput.strongestMember.detID
		tmpClusterOutput.trackingInput.SNR = tmpClusterOutput.strongestMember.rangeSNR
		tmpClusterOutput.trackingInput.angleSNR   = tmpClusterOutput.strongestMember.angleSNR
		tmpClusterOutput.trackingInput.rangeSNR   = tmpClusterOutput.strongestMember.rangeSNR
		tmpClusterOutput.trackingInput.dopplerSNR = tmpClusterOutput.strongestMember.dopplerSNR
		tmpClusterOutput.trackingInput.peakVal = tmpClusterOutput.strongestMember.rangeVal
		tmpClusterOutput.trackingInput.isSlowProc = tmpClusterOutput.strongestMember.isSlowProc
		tmpClusterOutput.trackingInput.isSlow = tmpClusterOutput.strongestMember.isSlow
		tmpClusterOutput.trackingInput.statusFlag = tmpClusterOutput.strongestMember.statusFlag

		if CONST.R_modify:
			tmpClusterOutput.trackingInput.measCovVec[0] = \
				convertSNRdBToVar(tmpClusterOutput.strongestMember.rangeSNR, len(domain.rangeDomain), resList[0])

			maxSizeflt = 0
			if tmpClusterOutput.xSize > tmpClusterOutput.ySize:
				maxSizeflt = tmpClusterOutput.xSize
			else:
				maxSizeflt = tmpClusterOutput.ySize

			tmpThresh = 0.5 * maxSizeflt * maxSizeflt

			if tmpClusterOutput.trackingInput.measCovVec[0] < tmpThresh:
				tmpClusterOutput.trackingInput.measCovVec[0] = tmpThresh

			tmpClusterOutput.trackingInput.measCovVec[1] = \
				convertSNRdBToVar(tmpClusterOutput.strongestMember.dopplerSNR,
				                  len(domain.dopplerDomain), resList[1])

			tmpClusterOutput.trackingInput.measCovVec[2] = \
				getSigmatheta(tmpClusterOutput.strongestMember.rangeSNR,
								clusteringCfg.numTx * clusteringCfg.numRx,
								tmpClusterOutput.trackingInput.measVectorRRD[2], 0.1, tmpClusterOutput.xSize,
				                tmpClusterOutput.trackingInput.measVectorRRD[0])

		else:
			tmpClusterOutput.trackingInput.measCovVec[0] = \
				convertSNRdBToVar(tmpClusterOutput.strongestMember.rangeSNR, len(domain.rangeDomain), resList[0])

			maxSizeflt = 0
			if tmpClusterOutput.xSize > tmpClusterOutput.ySize:
				maxSizeflt = tmpClusterOutput.xSize
			else:
				maxSizeflt = tmpClusterOutput.ySize

			if CONST.R_modify_20210326:
				if tmpClusterOutput.trackingInput.statusFlag == 0:
					if maxSizeflt > 0.4:
						maxSizeflt = 0.4
				else:
					if maxSizeflt > 1:						# (1) Modify
						maxSizeflt = 1						# (1) Modify

			tmpThresh = 3 * maxSizeflt * maxSizeflt			# measCovVec[0] : (3) Modify

			if tmpClusterOutput.trackingInput.measCovVec[0] < tmpThresh:
				tmpClusterOutput.trackingInput.measCovVec[0] = tmpThresh

			tmpClusterOutput.trackingInput.measCovVec[1] = \
				convertSNRdBToVar(tmpClusterOutput.strongestMember.dopplerSNR,
				                  len(domain.dopplerDomain), resList[1])

			tmpClusterOutput.trackingInput.measCovVec[2] = \
				convertSNRLinToVar(tmpClusterOutput.strongestMember.angleSNR,
			                        clusteringCfg.numTx * clusteringCfg.numRx,
			                        1 / (clusteringCfg.numTx * clusteringCfg.numRx))

			tmpThresh = maxSizeflt / tmpClusterOutput.trackingInput.measVectorRRD[0]

			if CONST.R_modify_20210326:
				tmpThresh = 3 * tmpThresh * tmpThresh					# measCovVec[2] : (3) Modify
			else:
				tmpThresh = 3 * tmpThresh * tmpThresh

			if tmpClusterOutput.trackingInput.measCovVec[2] < tmpThresh:
				tmpClusterOutput.trackingInput.measCovVec[2] = tmpThresh
			if CONST.R_modify_20210326:
				tmpClusterOutput.trackingInput.measCovVec[1] = tmpClusterOutput.trackingInput.measCovVec[1] * 6.0		# measCovVec[1] : (4.0) Modify

			# if tmpClusterOutput.strongestMember.statusFlag == 0:
			# 	tmpClusterOutput.trackingInput.measCovVec[0] = tmpClusterOutput.trackingInput.measCovVec[0]/6
			# 	tmpClusterOutput.trackingInput.measCovVec[2] = tmpClusterOutput.trackingInput.measCovVec[2]/6

		# =================================================1===========================================================
#		tmpClusterOutput.trackingInput.measCovVec[0] = 2.0*tmpClusterOutput.trackingInput.measCovVec[0]
#		tmpClusterOutput.trackingInput.measCovVec[1] = 2.0*tmpClusterOutput.trackingInput.measCovVec[1]
#		tmpClusterOutput.trackingInput.measCovVec[2] = 2.0*tmpClusterOutput.trackingInput.measCovVec[2]

		if CONST.trackingInputMODIFY:
			tmpClusterOutput.trackingInput.stateVectorXYZ[0] = tmpClusterOutput.xCenter
			tmpClusterOutput.trackingInput.stateVectorXYZ[1] = tmpClusterOutput.yCenter
			tmpClusterOutput.trackingInput.z = tmpClusterOutput.zCenter
			if CONST.SPEED_COMP_BY_ANGLE:
				compang_rad = math.atan2(tmpClusterOutput.xCenter, tmpClusterOutput.yCenter)
				tmpClusterOutput.trackingInput.stateVectorXYZ[2] = trackingInputSpeedAvg * np.sin(compang_rad)
				tmpClusterOutput.trackingInput.stateVectorXYZ[3] = trackingInputSpeedAvg * np.cos(compang_rad)
			else:
				tmpClusterOutput.trackingInput.stateVectorXYZ[2] = 0
				tmpClusterOutput.trackingInput.stateVectorXYZ[3] = trackingInputSpeedAvg
		else:
			tmpClusterOutput.trackingInput.stateVectorXYZ[0] = tmpClusterOutput.strongestMember.x
			tmpClusterOutput.trackingInput.stateVectorXYZ[1] = tmpClusterOutput.strongestMember.y
			tmpClusterOutput.trackingInput.stateVectorXYZ[2] = tmpClusterOutput.strongestMember.xd
			tmpClusterOutput.trackingInput.stateVectorXYZ[3] = tmpClusterOutput.strongestMember.yd

		if CONST.TRACKING_INPUT_MIN_POINT_MODIFY:
			tmpClusterOutput.trackingInput.pointNum = lengthOfCluster
		clusterOutputList.append(tmpClusterOutput)

	return clusterOutputList


def clusteringDBscan_findNeighbors2Fixed(objList, currObj, clusteringCfg, RadarInfo):
	tempCluster = []
	neighCount = 0

	epsilon2WithSpeed = 1			# (1)

	for compObj in objList:
		if compObj.visited:
			continue

		if ((currObj.z) > MAX_Z):
			continue
        
		if ((currObj.z) < MIN_Z):
			continue
        
		if abs(currObj.speed - compObj.speed) > clusteringCfg.ellipsoidC:
			continue

		if abs(currObj.x - compObj.x) > clusteringCfg.ellipsoidA:
			continue

		if abs(currObj.y - compObj.y) > clusteringCfg.ellipsoidB:
			continue
		
		if abs(currObj.z - compObj.z) > clusteringCfg.ellipsoidD:
			continue

		a = currObj.x - compObj.x
		b = currObj.y - compObj.y
		c = currObj.speed - compObj.speed
		d = currObj.z - compObj.z

		sumA = (a * a) / (clusteringCfg.ellipsoidA*clusteringCfg.ellipsoidA)
		sumB = (b * b) / (clusteringCfg.ellipsoidB*clusteringCfg.ellipsoidB)
		sumC = (c * c) / (clusteringCfg.ellipsoidC*clusteringCfg.ellipsoidC)
		sumD = (d * d) / (clusteringCfg.ellipsoidD*clusteringCfg.ellipsoidD)

		if (sumA + sumB + sumC) < 1:
			if compObj.scope:
				continue
			tempCluster.append(compObj)
			neighCount += 1

	return tempCluster, neighCount


def clusteringDBscanRun(objList, clusteringCfg, resList, domain, RadarInfo):  # objList consist with object of class cfarOutFmt3D.
	clusterId = 0
	clusterList = []
	for currObj in objList:
		if currObj.visited:
			continue

		currObj.visited = True

		tempCluster, neighCount = clusteringDBscan_findNeighbors2Fixed(objList, currObj, clusteringCfg, RadarInfo)
		tempCluster.append(currObj)
		tmpConditon = neighCount < (clusteringCfg.minPointsInCluster - 1)

		if tmpConditon:
			# This point is Noise
			# if the clusterId is 0, then the obj is Noise Point.
			continue
		else:
			clusterId += 1
			clusterList.append(tempCluster)

			for clusterObj in clusterList[-1]:
				clusterObj.scope = True
				clusterObj.clusterId = clusterId
				# clusterObj.visited = True

			for clusterObj in clusterList[-1]: # tag all the neighbors as visited in scope so that it will not be found again when searching neighbor's neighbor.
				clusterObj.visited = True
				tempCluster, neighCount = clusteringDBscan_findNeighbors2Fixed(objList, clusterObj, clusteringCfg, RadarInfo)

				if neighCount >= clusteringCfg.minPointsInCluster:
					if CONST.ellipsoidProcess:
						tmpRef = clusteringCfg.ellipsoidC
					else:
						tmpRef = clusteringCfg.vFactor

					if clusteringCfg.isGuardRail == False:
						if not (abs(currObj.speed - clusterObj.speed) < tmpRef):
							continue

					for clusterObj2 in tempCluster:
						clusterObj2.scope = True
						clusterObj2.clusterId = clusterId
						clusterList[-1].append(clusterObj2)

			if clusterId >= clusteringCfg.maxCluster:  # cluster 개수 30개로 한정
				break

	clusterOutputList = clusteringDBscan_calcInfoFixed(clusterList, resList, clusteringCfg, domain) #calculate the clustering center and edge information.

	return clusterList, clusterOutputList
