import numpy as np
import algorithm.dataStructure as dT
import os
import matplotlib.pyplot as plt
from ..radarConfigure import configManager_MRR_DEMO as cfg
from .. import mmwavelib as ml
import copy
import csv
from ..radarConfigure import CONST as CONST
import trackMaker as tm
from . import clustering as CL

N_MEAS = 3
N_STATE = 4
DEBUG = False
if DEBUG:
	np.set_printoptions(precision = 3)

def accCal(currTrack, td):
	if DEBUG and currTrack.ID == CONST.RCS_DEBUG_ID_NUMBER:
		print("accCal start")
		print("x0(k-1|k-1)  vx : %f, vy : %f"%(currTrack.prevXd, currTrack.prevYd))

	currTrack.state[0].stateVectorXYZ[4] = (currTrack.state[0].stateVectorXYZ[2] - currTrack.prevXd) / td
	currTrack.state[0].stateVectorXYZ[5] = (currTrack.state[0].stateVectorXYZ[3] - currTrack.prevYd) / td
	if DEBUG and currTrack.ID == CONST.RCS_DEBUG_ID_NUMBER:
		print("x0(k|k)  ax : %f, ay : %f" % (currTrack.state[0].stateVectorXYZ[4], currTrack.state[0].stateVectorXYZ[5]))


def omegaCal(currTrack, td):
	if DEBUG and currTrack.ID == CONST.RCS_DEBUG_ID_NUMBER:
		print("omegaCal Start")
		print("x0(k-1|k-1)  vx : %f, vy : %f"%(currTrack.prevXd, currTrack.prevYd))
		print("x0(k|k)      vx : %f, vy : %f" % (currTrack.state[0].stateVectorXYZ[2], currTrack.state[0].stateVectorXYZ[3]))
	thresh = 0.1
	if abs(currTrack.state[0].stateVectorXYZ[3]) > thresh:
		tmp0 = -(currTrack.state[0].stateVectorXYZ[2] - currTrack.prevXd) / (
					(currTrack.state[0].stateVectorXYZ[3]) * td)
	else:

		tmp0 = -(currTrack.state[0].stateVectorXYZ[2] - currTrack.prevXd) / (thresh * td)

		if currTrack.state[0].stateVectorXYZ[3] < 0:
			tmp0 = - tmp0
	if abs(currTrack.state[0].stateVectorXYZ[2]) > thresh:
		tmp1 = (currTrack.state[0].stateVectorXYZ[3] - currTrack.prevYd) / (currTrack.state[0].stateVectorXYZ[2] * td)
	else:
		tmp1 = (currTrack.state[0].stateVectorXYZ[3] - currTrack.prevYd) / (thresh * td)
		if currTrack.state[0].stateVectorXYZ[2] < 0:
			tmp1 = - tmp1
	if DEBUG and currTrack.ID == CONST.RCS_DEBUG_ID_NUMBER:
		print("tmp0 : %f, tmp1 : %f"%(tmp0, tmp1))

	if CONST.IMM_omega_0:
		tmp0 = 0
		tmp1 = 0
		if not (currTrack.state[0].stateVectorXYZ[3] == 0):
			tmp0 = -(currTrack.state[0].stateVectorXYZ[2] - currTrack.prevXd) / (
					(currTrack.state[0].stateVectorXYZ[3]) * td)

		if not(currTrack.state[0].stateVectorXYZ[2] == 0):

			tmp1 = (currTrack.state[0].stateVectorXYZ[3] - currTrack.prevYd) / (currTrack.state[0].stateVectorXYZ[2] * td)

	if abs(tmp0) > abs(tmp1):
		currTrack.state[0].stateVectorXYZ[4] = tmp0
	else:
		currTrack.state[0].stateVectorXYZ[4] = tmp1

	# currTrack.state[0].stateVectorXYZ[4] = (currTrack.state[0].stateVectorXYZ[1] * currTrack.state[0].stateVectorXYZ[2] - currTrack.state[0].stateVectorXYZ[0]* currTrack.state[0].stateVectorXYZ[3])\
	# 											/(currTrack.state[0].stateVectorXYZ[0]*currTrack.state[0].stateVectorXYZ[0] + currTrack.state[0].stateVectorXYZ[1]* currTrack.state[0].stateVectorXYZ[1])


def initNewTracker(currMeas, trkCfg, RadarInfo):
	# /* The covariance mat doesn't really matter at init, but we want some reasonabble numbers in it */
	# if DEBUG:
	# 	print("initNewTracker")
	# 	print("R(0) : \n",currMeas.measCovVec)

	currMeas.mode = RadarInfo.mode

	currMeas.Covariance[0][0] = (1 + currMeas.measCovVec[0]) * (1 + currMeas.measCovVec[2])
	currMeas.Covariance[1][1] = (1 + currMeas.measCovVec[0]) * (1 + currMeas.measCovVec[2])
	currMeas.Covariance[2][2] = (3 * currMeas.measVectorRRD[1] * currMeas.measVectorRRD[1]) + ((1 + currMeas.measCovVec[1]) * (1 + currMeas.measCovVec[2]))
	currMeas.Covariance[3][3] = (3 * currMeas.measVectorRRD[1] * currMeas.measVectorRRD[1]) + ((1 + currMeas.measCovVec[1]) * (1 + currMeas.measCovVec[2]))

	#IMM
	currMeas.mu[0] = 0.95
	currMeas.mu[1] = 0.05

	c_0 = currMeas.mu[0] * trkCfg.transitionProb[0][0] + currMeas.mu[1] * trkCfg.transitionProb[1][0]
	c_1 = currMeas.mu[0] * trkCfg.transitionProb[0][1] + currMeas.mu[1] * trkCfg.transitionProb[1][1]

	currMeas.mixingProb[0][0] = trkCfg.transitionProb[0][0] * currMeas.mu[0] / c_0
	currMeas.mixingProb[0][1] = trkCfg.transitionProb[0][1] * currMeas.mu[0] / c_1
	currMeas.mixingProb[1][0] = trkCfg.transitionProb[1][0] * currMeas.mu[1] / c_0
	currMeas.mixingProb[1][1] = trkCfg.transitionProb[1][1] * currMeas.mu[1] / c_1

	for state in currMeas.state:
		state.stateVectorXYZ[0] = currMeas.stateVectorXYZ[0]
		state.stateVectorXYZ[1] = currMeas.stateVectorXYZ[1]
		state.stateVectorXYZ[2] = currMeas.stateVectorXYZ[2]
		state.stateVectorXYZ[3] = currMeas.stateVectorXYZ[3]

		state.Covariance[0][0] = currMeas.Covariance[0][0]
		state.Covariance[1][1] = currMeas.Covariance[1][1]
		state.Covariance[2][2] = currMeas.Covariance[2][2]
		state.Covariance[3][3] = currMeas.Covariance[3][3]

	# if CONST.IMM_angle_vel_cal:
	# 	currMeas.state[0].stateVectorXYZ[4] = (currMeas.state[0].stateVectorXYZ[1] *currMeas.state[0].stateVectorXYZ[2] - currMeas.state[0].stateVectorXYZ[0]*currMeas.state[0].stateVectorXYZ[3])\
	# 										/(currMeas.state[0].stateVectorXYZ[0] * currMeas.state[0].stateVectorXYZ[0] + currMeas.state[0].stateVectorXYZ[1]* currMeas.state[0].stateVectorXYZ[1])


	# if DEBUG and  currTrack.ID== CONST.RCS_DEBUG_ID_NUMBER:
	# 	print("======initNewTracker======")
	# 	print("mixing Prob : \n", currMeas.mixingProb)
	# 	print("stateVector at mode 0 : \n", currMeas.state[0].stateVectorXYZ)
	# 	print("stateVector at mode 1 : \n", currMeas.state[1].stateVectorXYZ)
	# 	print("Covariance at mode 0 : \n", currMeas.state[0].Covariance)
	# 	print("Covariance at mode 1 : \n", currMeas.state[1].Covariance)

def imm_ekfRun(measList, trackingList, trackingCfg, frameNumber, RadarInfo):

	trackingList.resetTrackerAssociated()
	freeTrackerList = []
	selectedMeasList = []
	nFree = 0
	numProc_ekf = 0
	nAssociated = 0
	trackerIdx = -1

	for currTrack in trackingList.List[:]:
		trackerIdx += 1
		if numProc_ekf >= trackingCfg.maxTracker:
			break
		else:
			numProc_ekf += 1

#		if currTrack.mode != RadarInfo.mode:
#			nFree = ml.invalidateCurrTrack(currTrack, nFree, freeTrackerList)
#			continue

		if ((currTrack.validity == False) or not (ml.isWithinBoundingBox(currTrack, trackingCfg))):
			nFree = ml.invalidateCurrTrack(currTrack, nFree, freeTrackerList)
			continue

		FilterObj = Filter()

		if CONST.IMM_angle_vel_cal or CONST.IMM_CA:
			##### 이전 Frame Track의 X, Y, Xd, Yd 저장
			currTrack.preprepreX = currTrack.prepreX
			currTrack.preprepreY = currTrack.prepreY
			currTrack.preprepreXd = currTrack.prepreXd
			currTrack.preprepreYd = currTrack.prepreYd

			currTrack.prepreX = currTrack.preX
			currTrack.prepreY = currTrack.preY
			currTrack.prepreXd = currTrack.preXd
			currTrack.prepreYd = currTrack.preYd

			currTrack.preX = currTrack.stateVectorXYZ[0]
			currTrack.preY = currTrack.stateVectorXYZ[1]
			currTrack.preXd = currTrack.stateVectorXYZ[2]
			currTrack.preYd = currTrack.stateVectorXYZ[3]
			##### 이전 Frame Track의 X, Y, Xd, Yd 저장

		interaction(currTrack)
		FilterObj.initialize_mode(currTrack, cfg.trackingCfg)

		if FilterObj.stateVecTimeUpdate(currTrack, cfg.trackingCfg) == False:
			nFree = ml.invalidateCurrTrack(currTrack, nFree, freeTrackerList)
			continue
		currTrack.prevStateVectorXYZ = copy.deepcopy(currTrack.state[0].stateVectorXYZ)

		FilterObj.stateCovmatTimeUpdate(currTrack, trackingCfg.distanceLimit, trackingCfg.multiplier)
		currTrack.prevCovariance = copy.deepcopy(currTrack.state[0].Covariance)

		if FilterObj.calPredictedMeas(currTrack) == False:
			nFree = ml.invalidateCurrTrack(currTrack, nFree, freeTrackerList)
			continue

		FilterObj.calHmat(currTrack)
		currTrack.H_tmp = copy.deepcopy(FilterObj.mode_list[0].H)

		distMetricMin = 655350.0
		iAssocMeas = None
		currTrack.associatedValid = 0

		for currMeas in measList[:]:
			if currTrack.associatedObj == None and not (currMeas.isAssociated):
				innovation = FilterObj.mode_list[0].predictedMeas - currMeas.measVectorRRD

				if abs(innovation[0]) > trackingCfg.rangeAssocThresh:
					continue
				if abs(innovation[1]) > trackingCfg.velAssocThresh:
					continue
#				if abs(innovation[2]) > trackingCfg.azimAssocThresh:
#					continue

				innovation1 = currTrack.stateVectorXYZ - currMeas.stateVectorXYZ
				if abs(innovation1[0]) > trackingCfg.xAssocThresh:
					continue
				if abs(innovation1[1]) > trackingCfg.yAssocThresh:
					continue

				### Object Flag 적용
				if RadarInfo.mode == 2:
					if (currTrack.statusFlag & 15) != (currMeas.statusFlag & 15):
						continue
				### Object Flag 적용

				FilterObj.calInvS(currTrack, currMeas)
				pdistSq, canBeAssociated = FilterObj.get_mahalanobis_dist(currMeas, currTrack, trackingCfg)

				if canBeAssociated == True:

					if pdistSq < distMetricMin:
						distMetricMin = pdistSq
						iAssocMeas = currMeas

						ml.pinvHmat(currTrack, FilterObj.mode_list[0].S, FilterObj.mode_list[0].H)

		if iAssocMeas != None:
			currTrack.associatedObj = iAssocMeas
			iAssocMeas.isAssociated = True

		if currTrack.associatedObj != None:
			FilterObj.getInnovation(currTrack)

			if FilterObj.calInvS(currTrack, currTrack.associatedObj):
				FilterObj.get_mahalanobis_dist_(currTrack.associatedObj, currTrack, trackingCfg)
				FilterObj.gainComputation(currTrack)
				FilterObj.stateUpdate(currTrack)

				nAssociated += 1

				if CONST.STATUS_FLAG_ASC:
					if (currTrack.statusFlag & 15) != (currTrack.associatedObj.statusFlag & 15):
						currTrack.transitionScore += 1
					else:
						currTrack.transitionScore = 0

					if currTrack.transitionScore > 2:
						currTrack.statusFlag = currTrack.associatedObj.statusFlag
						currTrack.transitionScore = 0

				if currTrack.age > 1:
					currTrack.age = 1
				else:
					currTrack.age = 0

				if abs(currTrack.stateVectorXYZ[0]) > 25 and currTrack.tick < trackingCfg.maxTick:			# HSLee 수정 2022.07.22 : > 40 --> 15 --> 25
					currTrack.tick += 1
				elif not (abs(currTrack.stateVectorXYZ[0]) > 25) and currTrack.tick < trackingCfg.maxTick:	# HSLee 수정 2022.07.22 : > 40 --> 15 --> 25
					currTrack.tick += 2

				if currTrack.tick > trackingCfg.maxTick:		# 추가함 (2021.08.31)
					currTrack.tick = trackingCfg.maxTick		# 추가함 (2021.08.31)

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
				currTrack.Status_Flag0 = currTrack.statusFlag

				##### Track의 Association 유무 Flag
				currTrack.associatedValid = 1
				##### Track의 Association 유무 Flag

				FilterObj.likelihoodFunc()

				if CONST.IMM_CA:
					accCal(currTrack, cfg.trackingCfg.td)
				else:
					omegaCal(currTrack, cfg.trackingCfg.td)

				modeProbUpdate(currTrack, FilterObj.mode_list, cfg.trackingCfg)

			else:
				currTrack.associatedObj = None
				iAssocMeas.isAssociated = False

		if currTrack.associatedObj == None:
			currTrack.associatedValid = 0

			if CONST.STATUS_FLAG_ASC:
				currTrack.transitionScore = 0

			if CONST.TRACKING_AGE_FUNCTION:
				##### HSLee 수정 2022.01.21
				# Original Code
#				if currTrack.tick == 0 or currTrack.age > 2:
#					currTrack.age += 2
#				else:
#					currTrack.age += 1
				# Original Code

				if currTrack.tick == 0:
					currTrack.age = trackingCfg.maxAge + 1
				elif currTrack.age > 2:
					currTrack.age += 4					# (4)
				else:
					currTrack.age += 1
				##### HSLee 수정 2022.01.21
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

			if currTrack.age > trackingCfg.maxAge:
				nFree = ml.invalidateCurrTrack(currTrack, nFree, freeTrackerList)
			
		FilterObj.combination(currTrack)
		
		if currTrack.tick < trackingCfg.thresholdTick:
			currTrack.plotValidity = False
		else:
			currTrack.plotValidity = True

		currTrack.prevXd = (currTrack.stateVectorXYZ[0] - currTrack.preX)/(1*cfg.trackingCfg.td)
		currTrack.prevYd = (currTrack.stateVectorXYZ[1] - currTrack.preY)/(1*cfg.trackingCfg.td)
		currTrack.prevprevXd = (currTrack.stateVectorXYZ[0] - currTrack.prepreX)/(2*cfg.trackingCfg.td)
		currTrack.prevprevYd = (currTrack.stateVectorXYZ[1] - currTrack.prepreY)/(2*cfg.trackingCfg.td)
		currTrack.prevprevprevXd = (currTrack.stateVectorXYZ[0] - currTrack.preprepreX)/(3*cfg.trackingCfg.td)
		currTrack.prevprevprevYd = (currTrack.stateVectorXYZ[1] - currTrack.preprepreY)/(3*cfg.trackingCfg.td)

		currTrack.prevXd = (currTrack.stateVectorXYZ[0] - currTrack.preprepreX)/(3*cfg.trackingCfg.td)
		currTrack.prevYd = (currTrack.stateVectorXYZ[1] - currTrack.preprepreY)/(3*cfg.trackingCfg.td)

		currTrack.Status_Flag1 = currTrack.associatedValid

	if nFree > 0:
		for currTrack in freeTrackerList[:]:
			trackingList.rmObj(currTrack)

	nFree = trackingCfg.maxTracker - len(trackingList.List)
	numNewTracks = ml.selectMeas(measList, nFree, len(measList) - nAssociated, len(measList), selectedMeasList, trackingCfg)

	for currMeas in selectedMeasList:
		### HSLee 추가 2022.08.22
		if RadarInfo.mode == 1:
			if RadarInfo.position == 1:				# Left
				leftscale1 = -0.5					# (-0.5)
				rightscale1 = 0.3					# (0.3)
				leftscale2 = -4.0					# (-4.0)
				rightscale2 = 3.0					# (3.0)
				leftscale3 = -7.0					# (-7.0)
				rightscale3 = 7.0					# (7.0)
			elif RadarInfo.position == 2:			# Right
				leftscale1 = -0.3					# (-0.3)
				rightscale1 = 0.5					# (0.5)
				leftscale2 = -3.0					# (-3.0)
				rightscale2 = 4.0					# (4.0)
				leftscale3 = -7.0					# (-7.0)
				rightscale3 = 7.0					# (7.0)
			else:
				leftscale1 = -0.7					# (-0.7)
				rightscale1 = 0.7					# (0.7)
				leftscale2 = -3.0					# (-3.0)
				rightscale2 = 3.0					# (3.0)
				leftscale3 = -7.0					# (-7.0)
				rightscale3 = 7.0					# (7.0)

			measX = currMeas.stateVectorXYZ[0]
			measY = currMeas.stateVectorXYZ[1]

			Deg2_rad = RadarInfo.angle * np.pi / 180
			rotateX = measX * np.cos(Deg2_rad) + measY * np.sin(Deg2_rad)
			rotateY = -measX * np.sin(Deg2_rad) + measY * np.cos(Deg2_rad)

			if currMeas.pointNum == 1:
				continue
			# Step 1
			if (0.0 <= rotateY) and (rotateY < 2.1) and (leftscale1 <= rotateX) and (rotateX <= rightscale1):			# (0.0, 2.1)
				continue
			# Step 2
			elif (2.1 <= rotateY) and (rotateY <= 3.3) and (leftscale2 <= rotateX) and (rotateX <= rightscale2):		# (2.1, 3.3)
				continue
			# Step 3
			elif (3.3 < rotateY) and (leftscale3 <= rotateX) and (rotateX <= rightscale3):								# (3.3)
				continue
		### HSLee 추가 2022.08.22

		if len(trackingList.List) > trackingCfg.maxTracker: # tracking 개수 30로 한정
			break
		currMeas.validity = True
		tmpTrack = copy.deepcopy(currMeas)
		initNewTracker(tmpTrack, cfg.trackingCfg, RadarInfo)
		trackingList.appendDetObj(tmpTrack)

	trackingList.arrangeTracksByAge()

def interaction(currTrack):
	if DEBUG and currTrack.ID == CONST.RCS_DEBUG_ID_NUMBER:

		print("\n==== interaction ====")

		print("R(k-1) : \n", currTrack.measCovVec)

		print("input : ")
		print("x0(k-1|k-1) : ", currTrack.state[0].stateVectorXYZ)
		print("x1(k-1|k-1) : ", currTrack.state[1].stateVectorXYZ)
		print("mixing Problem : \n", currTrack.mixingProb)
		print("P0(k-1|k-1) : \n", currTrack.state[0].Covariance)
		print("P1(k-1|k-1) : \n", currTrack.state[1].Covariance)

	for idx, state in enumerate(currTrack.state):
		state.initCondstate = currTrack.state[0].stateVectorXYZ * currTrack.mixingProb[0][idx] + currTrack.state[1].stateVectorXYZ * currTrack.mixingProb[1][idx]
		state.initCondCov = currTrack.mixingProb[0][idx] * (currTrack.state[0].Covariance + np.matmul((currTrack.state[0].stateVectorXYZ - state.initCondstate).reshape(-1,1), (currTrack.state[0].stateVectorXYZ - state.initCondstate).reshape(-1,1).T))\
							+ currTrack.mixingProb[1][idx] * (currTrack.state[1].Covariance \
							        + np.matmul((currTrack.state[1].stateVectorXYZ - state.initCondstate).reshape(-1,1), (currTrack.state[1].stateVectorXYZ - state.initCondstate).reshape(-1,1).T))
		# print("at mode %d :\n"%idx, np.matmul((currTrack.state[1].stateVectorXYZ - state.initCondstate).reshape(-1,1), (currTrack.state[1].stateVectorXYZ - state.initCondstate).reshape(-1,1).T))
		# print("at mode %d :\n"%idx, np.matmul((currTrack.state[0].stateVectorXYZ - state.initCondstate).reshape(-1,1), (currTrack.state[0].stateVectorXYZ - state.initCondstate).reshape(-1,1).T))


	if DEBUG and currTrack.ID == CONST.RCS_DEBUG_ID_NUMBER:

		print("x00(k-1|k-1) : \n", currTrack.state[0].initCondstate)
		print("x01(k-1|k-1) : \n", currTrack.state[1].initCondstate)
		print("P00(k-1|k-1) : \n", currTrack.state[0].initCondCov)
		print("P01(k-1|k-1) : \n", currTrack.state[1].initCondCov)


def modeProbUpdate(currTrack, modelist, trkCfg):
	if DEBUG and currTrack.ID == CONST.RCS_DEBUG_ID_NUMBER:
		print("modeProbUpdate start\n")
		print("currTrack.mu : ", currTrack.mu)
		print("likelihood")
		print(modelist[0].likelihood, modelist[1].likelihood)

	c = 0

	tmp0 = modelist[0].likelihood * (trkCfg.transitionProb[0][0] * currTrack.mu[0] + trkCfg.transitionProb[1][0] * currTrack.mu[1])
	tmp1 = modelist[1].likelihood * (trkCfg.transitionProb[0][1] * currTrack.mu[0] + trkCfg.transitionProb[1][1] * currTrack.mu[1])
	if DEBUG and currTrack.ID == CONST.RCS_DEBUG_ID_NUMBER:
		print("tmp0 : %f, tmp1 : %f"%(tmp0, tmp1))
	# modelist[0].c_ = trkCfg.transitionProb[0][0] * currTrack.mu[0] + trkCfg.transitionProb[1][0] * currTrack.mu[1]
	# modelist[1].c_ = trkCfg.transitionProb[0][1] * currTrack.mu[0] + trkCfg.transitionProb[1][1] * currTrack.mu[1]
	currTrack.mu[0] = tmp0
	currTrack.mu[1] = tmp1
	c = currTrack.mu[0] + currTrack.mu[1]
	currTrack.mu = currTrack.mu / c

	# modelist[0].c_ = trkCfg.transitionProb[0][0] * currTrack.mu[0] + trkCfg.transitionProb[1][0] * currTrack.mu[1]
	# modelist[1].c_ = trkCfg.transitionProb[0][1] * currTrack.mu[0] + trkCfg.transitionProb[1][1] * currTrack.mu[1]

	if DEBUG and currTrack.ID ==CONST.RCS_DEBUG_ID_NUMBER:
		print("before modeProbUpdate")
		print("currTrack.mixingProb : \n", currTrack.mixingProb)

	currTrack.mixingProb[0][0] = trkCfg.transitionProb[0][0] * currTrack.mu[0]
	currTrack.mixingProb[1][0] = trkCfg.transitionProb[1][0] * currTrack.mu[1]
	c = currTrack.mixingProb[0][0] + currTrack.mixingProb[1][0]
	currTrack.mixingProb[0][0] = currTrack.mixingProb[0][0]/c
	currTrack.mixingProb[1][0] = currTrack.mixingProb[1][0]/c

	currTrack.mixingProb[0][1] = trkCfg.transitionProb[0][1] * currTrack.mu[0]
	currTrack.mixingProb[1][1] = trkCfg.transitionProb[1][1] * currTrack.mu[1]
	c = currTrack.mixingProb[0][1] + currTrack.mixingProb[1][1]
	currTrack.mixingProb[0][1] = currTrack.mixingProb[0][1]/c
	currTrack.mixingProb[1][1] = currTrack.mixingProb[1][1]/c

	if DEBUG and currTrack.ID == CONST.RCS_DEBUG_ID_NUMBER:
		print("after modeProbUpdate")
		print("currTrack.mu : ", currTrack.mu)
		print("currTrack.mixingProb : \n", currTrack.mixingProb)

class Mode:
	def __init__(self, ndim):
		self.F = np.zeros((ndim, ndim), dtype=np.float)
		self.Q = np.zeros((ndim, ndim), dtype=np.float)
		self.predictedMeas = np.zeros(3, dtype = np.float)
		self.H = np.zeros((3, dT.max_dim_state), dtype=np.float)
		self.S = np.zeros((N_MEAS, N_MEAS), dtype=np.float)
		self.S_= np.zeros((N_MEAS, N_MEAS), dtype=np.float)
		self.invS = np.zeros((N_MEAS, N_MEAS), dtype=np.float)
		self.W = np.zeros((5, N_MEAS), dtype=np.float)
		self.innovation = np.zeros(N_MEAS, dtype = np.float)
		self.likelihood = 0
		self.mahalanobisDist = 0
		self.c_ = 0

class Filter:
	def __init__(self):
		self.mode_list = [Mode(dT.max_dim_state), Mode(dT.max_dim_state)]

	def stateCovmatTimeUpdate(self, currTrack, limit, scale):
		if currTrack.mahalanobisDistance > 2.0 and CONST.ADATIVE_Q:
			fudgeFactor = scale * scale
			# print("hi")
		else:
			fudgeFactor = 1

		for state, mode in zip(currTrack.state, self.mode_list):
			tmp = np.matmul(mode.F, state.initCondCov)
			state.Covariance = np.matmul(tmp, mode.F.T)
			state.Covariance = state.Covariance + fudgeFactor * mode.Q

		if DEBUG and currTrack.ID == CONST.RCS_DEBUG_ID_NUMBER:
			print("Q at mode 0 : \n", self.mode_list[0].Q)
			print("Q at mode 1 : \n", self.mode_list[1].Q)

	def stateVecTimeUpdate(self, currTrack, trkCfg):
		for idx, (mode, state) in enumerate(zip(self.mode_list, currTrack.state)):
			if idx == 0:
				state.stateVectorXYZ[0] = state.initCondstate[0] + trkCfg.td * state.initCondstate[2]
				state.stateVectorXYZ[1] = state.initCondstate[1] + trkCfg.td * state.initCondstate[3]
				state.stateVectorXYZ[2] = state.initCondstate[2]
				state.stateVectorXYZ[3] = state.initCondstate[3]
				state.stateVectorXYZ[4] = 0
			elif idx == 1:
				state.stateVectorXYZ = np.matmul(mode.F, state.initCondstate)

		### HSLee 수정 2022.07.19
		if currTrack.state[0].stateVectorXYZ[1] < 0 or currTrack.state[1].stateVectorXYZ[1] < 0:
			return False
		return True

	def initialize_mode(self, currTrack, trkCfg):
		for idx, (mode,state) in enumerate(zip(self.mode_list, currTrack.state)):
			if idx == 0:
				mode.F[0][0] = 1
				mode.F[1][1] = 1
				mode.F[2][2] = 1
				mode.F[3][3] = 1
				mode.F[0][2] = trkCfg.td
				mode.F[1][3] = trkCfg.td

				noiseGain = np.zeros((dT.max_dim_state, 2), dtype=np.float)
				noiseGain[0][0] = 0.5 * trkCfg.td ** 2
				noiseGain[1][1] = 0.5 * trkCfg.td ** 2
				noiseGain[2][0] = trkCfg.td
				noiseGain[3][1] = trkCfg.td

				noiseVar = np.zeros((2, 2), dtype=np.float)
				noiseVar[0][0] = trkCfg.CV_xVariance
				noiseVar[1][1] = trkCfg.CV_yVariance

				noiseVar[0][0] = noiseVar[0][0] * noiseVar[0][0]
				noiseVar[1][1] = noiseVar[1][1] * noiseVar[1][1]
				mode.Q = np.matmul(noiseGain, noiseVar)
				mode.Q = np.matmul(mode.Q, noiseGain.T)

			elif idx == 1:
				if CONST.IMM_CA:
					mode.F[0][0] = 1
					mode.F[0][2] = trkCfg.td
					mode.F[0][4] = 0.5 * (trkCfg.td) ** 2

					mode.F[1][1] = 1
					mode.F[1][3] = trkCfg.td
					mode.F[1][5] = 0.5 * (trkCfg.td) ** 2

					mode.F[2][2] = 1
					mode.F[2][4] = trkCfg.td

					mode.F[3][3] = 1
					mode.F[3][5] = trkCfg.td

					mode.F[4][4] = 1

					mode.F[5][5] = 1

					noiseGain = np.zeros((dT.max_dim_state, 2), dtype=np.float)
					noiseGain[0][0] = 0.5 * trkCfg.td ** 2
					noiseGain[1][1] = 0.5 * trkCfg.td ** 2
					noiseGain[2][0] = trkCfg.td
					noiseGain[3][1] = trkCfg.td
					noiseGain[4][0] = 1
					noiseGain[5][1] = 1

					noiseVar = np.zeros((2, 2), dtype=np.float)
					noiseVar[0][0] = trkCfg.CA_xVariance
					noiseVar[1][1] = trkCfg.CA_yVariance

					noiseVar[0][0] = noiseVar[0][0] * noiseVar[0][0]
					noiseVar[1][1] = noiseVar[1][1] * noiseVar[1][1]
					mode.Q = np.matmul(noiseGain, noiseVar)
					mode.Q = np.matmul(mode.Q, noiseGain.T)
				else:
					if DEBUG and currTrack.ID == CONST.RCS_DEBUG_ID_NUMBER:
						print("omega!! : %f"%state.initCondstate[4])
					if abs(state.initCondstate[4]) < 0.0001:
						if CONST.IMM_transMat_Mod:
							mode.F[0][0] = 1
							mode.F[0][2] = trkCfg.td
							mode.F[0][3] = -state.initCondstate[4] * (trkCfg.td) ** 2

							mode.F[1][1] = 1
							mode.F[1][2] = state.initCondstate[4] * (trkCfg.td) ** 2
							mode.F[1][3] = trkCfg.td

							mode.F[2][2] = 1 - 0.5 * (state.initCondstate[4] * trkCfg.td) ** 2
							mode.F[2][3] = -state.initCondstate[4] * trkCfg.td

							mode.F[3][2] = state.initCondstate[4] * trkCfg.td
							mode.F[3][3] = 1 - 0.5 * (state.initCondstate[4] * trkCfg.td) ** 2

							mode.F[4][4] = 1
						else:

							mode.F[0][0] = 1
							mode.F[0][2] = trkCfg.td
							mode.F[0][3] = -0.5 * state.initCondstate[4] * (trkCfg.td) ** 2

							mode.F[1][1] = 1
							mode.F[1][2] = 0.5 * state.initCondstate[4] * (trkCfg.td) ** 2
							mode.F[1][3] = trkCfg.td

							mode.F[2][2] = 1 - 0.5 * (state.initCondstate[4] * trkCfg.td) ** 2
							mode.F[2][3] = -state.initCondstate[4] * trkCfg.td

							mode.F[3][2] = state.initCondstate[4] * trkCfg.td
							mode.F[3][3] = 1 - 0.5 * (state.initCondstate[4] * trkCfg.td) ** 2

							mode.F[4][4] = 1


					else:

						if CONST.IMM_transMat_Mod:
							mode.F[0][0] = 1
							mode.F[0][2] = np.sin(state.initCondstate[4]* trkCfg.td)  / state.initCondstate[4]
							mode.F[0][3] = -(1 - np.cos(state.initCondstate[4]* trkCfg.td))  / state.initCondstate[4]
							mode.F[1][1] = 1
							mode.F[1][2] = (1 - np.cos(state.initCondstate[4]* trkCfg.td))  / state.initCondstate[4]
							mode.F[1][3] = np.sin(state.initCondstate[4]* trkCfg.td)  / state.initCondstate[4]
							mode.F[2][2] = np.cos(state.initCondstate[4]* trkCfg.td)
							mode.F[2][3] = -np.sin(state.initCondstate[4]* trkCfg.td)
							mode.F[3][2] = np.sin(state.initCondstate[4]* trkCfg.td)
							mode.F[3][3] = np.cos(state.initCondstate[4]* trkCfg.td)
							mode.F[4][4] = 1
						else:
							mode.F[0][0] = 1
							mode.F[0][2] = np.sin(state.initCondstate[4]* trkCfg.td)  / state.initCondstate[4]
							mode.F[0][3] = -(1 - np.cos(state.initCondstate[4]* trkCfg.td))  / state.initCondstate[4]
							mode.F[1][1] = 1
							mode.F[1][2] = (1 - np.cos(state.initCondstate[4]* trkCfg.td))  / state.initCondstate[4]
							mode.F[1][3] = np.sin(state.initCondstate[4]* trkCfg.td)  / state.initCondstate[4]
							mode.F[2][2] = np.cos(state.initCondstate[4]* trkCfg.td)
							mode.F[2][3] = -np.sin(state.initCondstate[4]* trkCfg.td)
							mode.F[3][2] = np.sin(state.initCondstate[4]* trkCfg.td)
							mode.F[3][3] = np.cos(state.initCondstate[4]* trkCfg.td)
							mode.F[4][4] = 1
						if CONST.IMM_ACT:
							# if abs(state.initCondstate[4]) > 0.1:
							if True:
								mode.F[0][4] = (state.initCondstate[2] * (state.initCondstate[4] * trkCfg.td * np.cos(
									state.initCondstate[4] * trkCfg.td) - np.sin(state.initCondstate[4] * trkCfg.td)) -
								                state.initCondstate[3] * (state.initCondstate[4] * trkCfg.td * np.sin(
											state.initCondstate[4] * trkCfg.td) + np.cos(
											state.initCondstate[4] * trkCfg.td) - 1)) / (
											               state.initCondstate[4] * state.initCondstate[4])
								mode.F[1][4] = (state.initCondstate[2] * (state.initCondstate[4] * trkCfg.td * np.sin(
									state.initCondstate[4] * trkCfg.td) + np.cos(state.initCondstate[4] * trkCfg.td) - 1)) - \
								               state.initCondstate[3] * (state.initCondstate[4] * trkCfg.td * np.cos(
									state.initCondstate[4] * trkCfg.td) - np.sin(state.initCondstate[4] * trkCfg.td)) / (
											               state.initCondstate[4] * state.initCondstate[4])
								mode.F[2][4] = -trkCfg.td * (
											state.initCondstate[2] * np.sin(state.initCondstate[4] * trkCfg.td) +
											state.initCondstate[3] * np.cos(state.initCondstate[4] * trkCfg.td))
								mode.F[3][4] = trkCfg.td * (
										state.initCondstate[2] * np.cos(state.initCondstate[4] * trkCfg.td) -
										state.initCondstate[3] * np.sin(state.initCondstate[4] * trkCfg.td))
							# else:
							# 	mode.F[0][4] = - 0.5 * trkCfg.td * trkCfg.td * state.initCondstate[3]
							# 	mode.F[1][4] = 0.5 * trkCfg.td * trkCfg.td * state.initCondstate[2]
							# 	mode.F[2][4] = - trkCfg.td * state.initCondstate[3]
							# 	mode.F[3][4] = trkCfg.td * state.initCondstate[2]
							##clockwise
							# mode.F[0][4] = (state.initCondstate[2]*(-state.initCondstate[4]*trkCfg.td*np.cos(state.initCondstate[4]*trkCfg.td) + np.sin(state.initCondstate[4]*trkCfg.td)) - state.initCondstate[3]*(state.initCondstate[4]*trkCfg.td * np.sin(state.initCondstate[4]*trkCfg.td) + np.cos(state.initCondstate[4]*trkCfg.td) - 1)) / ( state.initCondstate[4] * state.initCondstate[4])
							# mode.F[1][4] = (state.initCondstate[2]*(state.initCondstate[4]*trkCfg.td * np.sin(state.initCondstate[4]*trkCfg.td) + np.cos(state.initCondstate[4]*trkCfg.td) - 1)) - state.initCondstate[3]*(-state.initCondstate[4]*trkCfg.td*np.cos(state.initCondstate[4]*trkCfg.td) + np.sin(state.initCondstate[4]*trkCfg.td)) / ( state.initCondstate[4] * state.initCondstate[4])
							# mode.F[2][4] = trkCfg.td *(state.initCondstate[2] * np.sin(state.initCondstate[4]*trkCfg.td) - state.initCondstate[3] * np.cos(state.initCondstate[4]*trkCfg.td))
							# mode.F[3][4] = trkCfg.td * (
							# 			state.initCondstate[2] * np.cos(state.initCondstate[4] * trkCfg.td) +
							# 			state.initCondstate[3] * np.sin(state.initCondstate[4] * trkCfg.td))


					noiseGain = np.zeros((dT.max_dim_state, 3), dtype=np.float)
					noiseGain[0][0] = 0.5 * trkCfg.td ** 2
					noiseGain[1][1] = 0.5 * trkCfg.td ** 2
					noiseGain[2][0] = trkCfg.td
					noiseGain[3][1] = trkCfg.td
					noiseGain[4][2] = 1

					noiseVar = np.zeros((3, 3), dtype=np.float)
					# noiseVar[0][0] = 5
					noiseVar[0][0] = 8
					noiseVar[1][1] = 8
					noiseVar[2][2] = 1

					noiseVar[0][0] = noiseVar[0][0]* noiseVar[0][0]
					noiseVar[1][1] = noiseVar[1][1] * noiseVar[1][1]
					noiseVar[2][2] = noiseVar[2][2] * noiseVar[2][2]
					mode.Q = np.matmul(noiseGain, noiseVar)
					mode.Q = np.matmul(mode.Q, noiseGain.T)

	def calPredictedMeas(self, currTrack):
		if DEBUG and currTrack.ID == CONST.RCS_DEBUG_ID_NUMBER:
			print("\n calPredictedMeas Start!")
		for idx, (mode, state) in enumerate(zip(self.mode_list, currTrack.state)):

			if DEBUG and currTrack.ID == CONST.RCS_DEBUG_ID_NUMBER and CONST.predictedCalMod:
				print("\nH%d(k) : \n"%idx,mode.H)
				print("x%d(k|k-1) : "%idx, state.stateVectorXYZ)


			mode.predictedMeas[0] = state.stateVectorXYZ[0] * state.stateVectorXYZ[0] + state.stateVectorXYZ[1] * state.stateVectorXYZ[1]
			if mode.predictedMeas[0] < 0.005:
				return False

			mode.predictedMeas[0] = np.sqrt(mode.predictedMeas[0])

			if CONST.predictedCalMod:
				# mode.predictedMeas[0] = (mode.H @ state.stateVectorXYZ.reshape(-1,1))[0]
				# mode.predictedMeas[1] = (mode.H @ state.stateVectorXYZ.reshape(-1,1))[1]
				# mode.predictedMeas[2] = (mode.H @ state.stateVectorXYZ.reshape(-1,1))[2]

				mode.predictedMeas[1] = state.stateVectorXYZ[3]
				mode.predictedMeas[2] = 0
			else:

				mode.predictedMeas[1] = (state.stateVectorXYZ[0]* state.stateVectorXYZ[2] + state.stateVectorXYZ[1] * state.stateVectorXYZ[3])/mode.predictedMeas[0]
				mode.predictedMeas[2] = state.stateVectorXYZ[0]/mode.predictedMeas[0]

#				theta_rad = math.atan2(state.stateVectorXYZ[0], state.stateVectorXYZ[1])
#				radial_xd = state.stateVectorXYZ[2] * np.sin(theta_rad)
#				radial_yd = state.stateVectorXYZ[3] * np.cos(theta_rad)

#				mode.predictedMeas[1] = radial_xd + radial_yd
#				mode.predictedMeas[2] = np.sin(theta_rad)

			if DEBUG and currTrack.ID == CONST.RCS_DEBUG_ID_NUMBER:
				print("z%d(k|k-1) : "%idx, mode.predictedMeas)
		return True

	def calHmat(self, currTrack):
		if DEBUG and currTrack.ID == CONST.RCS_DEBUG_ID_NUMBER:
			print("\ncalHmat start!")
		for idx, (mode, state) in enumerate(zip(self.mode_list, currTrack.state)):
			if DEBUG and currTrack.ID == CONST.RCS_DEBUG_ID_NUMBER:
				print("x%d(k|k-1) : "%idx, state.stateVectorXYZ)
			mode.H = ml.computeHmat(state.stateVectorXYZ, mode.predictedMeas)
			if DEBUG and currTrack.ID == CONST.RCS_DEBUG_ID_NUMBER:
				print("H%d(k) : \n"%idx, mode.H)

	def calS_(self, currTrack):
		for mode, state in zip(self.mode_list, currTrack.state):
			mode.S_ = np.matmul(mode.H, state.Covariance)
			mode.S_ = np.matmul(mode.S_, mode.H.T)

	def calInvS(self, currTrack, currMeas):

		Rmat = np.zeros((N_MEAS, N_MEAS), dtype=np.float)
		for i in range(N_MEAS):
			Rmat[i][i] = currMeas.measCovVec[i]

		if DEBUG and currTrack.ID == CONST.RCS_DEBUG_ID_NUMBER:
			print("\nRmat : \n", Rmat)
		for idx, (mode, state) in enumerate(zip(self.mode_list, currTrack.state)):
			mode.S = np.matmul(mode.H, state.Covariance)
			mode.S = np.matmul(mode.S, mode.H.T)

			if DEBUG and currTrack.ID == CONST.RCS_DEBUG_ID_NUMBER:
				print("\nS not add R at mode %d : \n"%(idx), mode.S)

			mode.S = mode.S + Rmat
			mode.invS = np.linalg.inv(mode.S)

			#check inversable
			if np.allclose(np.matmul(mode.S, mode.invS), np.eye(N_MEAS)) == False:
				return False

		if DEBUG and currTrack.ID == CONST.RCS_DEBUG_ID_NUMBER:
			print("\n==== calInvS ====")
			print("invS at mode 0 : \n", self.mode_list[0].invS)
			print("invS at mode 1 : \n", self.mode_list[1].invS)
		return True

	def get_mahalanobis_dist_(self, currMeas, currTrack, trackingCfg):
		for idx, mode in enumerate(self.mode_list):

			rrdCurrMeas = currMeas.measVectorRRD
			rrdEstimationError = rrdCurrMeas - mode.predictedMeas
			tmp = np.dot(rrdEstimationError, mode.invS)
			mode.mahalanobisDist = np.dot(tmp, rrdEstimationError)

		currTrack.mahalanobisDistance = self.mode_list[0].mahalanobisDist


	def get_mahalanobis_dist(self, currMeas, currTrack, trackingCfg):
		if CONST.maxProbAssoc:
			idx = np.argmax(currTrack.mu)
		else:
			idx = 0
		trigger = False

		rrdCurrMeas = currMeas.measVectorRRD
		predTrk = self.mode_list[idx].predictedMeas
		rrdError = predTrk - rrdCurrMeas

		### HSLee 수정 2022.07.21
		stateTrk = currTrack.stateVectorXYZ
		xyzError = stateTrk - currMeas.stateVectorXYZ

		a = abs(rrdError[0]) > trackingCfg.rangeAssocThresh
		b = abs(rrdError[1]) > trackingCfg.velAssocThresh

		c = abs(xyzError[0]) > trackingCfg.xAssocThresh
		d = abs(xyzError[1]) > trackingCfg.yAssocThresh
		### HSLee 수정 2022.07.21

		if a or b or c or d:
			return 99999999, trigger

		tmp = np.dot(rrdError, self.mode_list[idx].invS)
		mahalanobisDistance = np.dot(tmp, rrdError)

		if mahalanobisDistance < trackingCfg.associateGamma:
			pdistSq = mahalanobisDistance
			trigger = True
			return pdistSq, trigger

		return 99999999, trigger


	def getInnovation(self, currTrack):

		meas = currTrack.associatedObj.measVectorRRD

		for mode in self.mode_list:
			mode.innovation = meas - mode.predictedMeas

		if DEBUG and currTrack.ID == CONST.RCS_DEBUG_ID_NUMBER:
			print("innovation at mode 0", self.mode_list[0].innovation)
			print("innovation at mode 1", self.mode_list[1].innovation)

	def gainComputation(self, currTrack):
		for idx, (mode, state) in enumerate(zip(self.mode_list, currTrack.state)):
			if DEBUG and currTrack.ID == CONST.RCS_DEBUG_ID_NUMBER:
				tmp = np.matmul(mode.H.T, mode.invS)
				print("tmp %d\n"%idx, tmp)

			mode.W = np.matmul(state.Covariance, mode.H.T)
			mode.W = np.matmul(mode.W, mode.invS)
			if idx == 0:
				mode.W[4][:] = 0

		if DEBUG and currTrack.ID == CONST.RCS_DEBUG_ID_NUMBER:
			print("\n==== gain Computation ====")
			print("W at mode 0 : \n", self.mode_list[0].W)
			print("W at mode 1 : \n", self.mode_list[1].W)

	def stateUpdate(self, currTrack):
		if DEBUG and currTrack.ID == CONST.RCS_DEBUG_ID_NUMBER:
			print("\n==== state Update start====")

		for idx, (mode, state) in enumerate(zip(self.mode_list, currTrack.state)):
			state.stateVectorXYZ = state.stateVectorXYZ + np.matmul(mode.W, mode.innovation)
			state.Covariance = np.matmul((np.eye(dT.max_dim_state)-np.matmul(mode.W, mode.H)), state.Covariance)

			if DEBUG and currTrack.ID == CONST.RCS_DEBUG_ID_NUMBER:

				print("temp%d\n"%idx, (np.eye(dT.max_dim_state)-np.matmul(mode.W, mode.H)))


				print("x%d(k|k)\n"%(idx), state.stateVectorXYZ)
				print("P%d(k|k)\n"%idx, state.Covariance)



	def likelihoodFunc(self):
		if DEBUG:
			print("likelihoodFunc start")
		for idx, mode  in enumerate(self.mode_list):
			# if np.linalg.det(mode.invS) == 0:
			# 	print(np.linalg.det(mode.invS))
			if DEBUG:
				print("input \nmahalanobisDist : %f, invdetS %d : %f"%(mode.mahalanobisDist, idx, (1/np.sqrt(np.linalg.det(mode.S)))))
			mode.likelihood = (1/np.sqrt(((2* np.pi)) * np.linalg.det(mode.S))) * np.exp(-0.5 * mode.mahalanobisDist)
			# mode.likelihood = np.exp(-0.5 * mode.mahalanobisDist)
			if DEBUG:
				print("likelihood%d : %f"%(idx, mode.likelihood) )
			# mode.likelihood = 1/mode.mahalanobisDist



	def combination(self, currTrack):
		currTrack.stateVectorXYZ[0] = 0
		currTrack.stateVectorXYZ[1] = 0
		currTrack.stateVectorXYZ[2] = 0
		currTrack.stateVectorXYZ[3] = 0

		currTrack.Covariance = np.zeros((N_STATE, N_STATE), dtype=np.float)


		if DEBUG and currTrack.ID == CONST.RCS_DEBUG_ID_NUMBER:
			print("combination() Start")
			print("mode0 state: ",currTrack.state[0].stateVectorXYZ)
			print("mode1 state: ", currTrack.state[1].stateVectorXYZ)
		for idx, (mode, state) in enumerate(zip(self.mode_list, currTrack.state)):

			currTrack.stateVectorXYZ += currTrack.mu[idx] * state.stateVectorXYZ[:4]

		for idx, (mode, state) in enumerate(zip(self.mode_list, currTrack.state)):

			currTrack.Covariance += currTrack.mu[idx]*(state.Covariance[:4, :4]\
			+ np.matmul((state.stateVectorXYZ[:4] - currTrack.stateVectorXYZ).reshape(-1, 1), (state.stateVectorXYZ[:4] - currTrack.stateVectorXYZ).reshape(-1, 1).T))
	def out(self):
		return self.mode_list

class Tracking():
	def __init__(self):
		self.trackingCfg 	= cfg.trackingCfg()
		self.tracker 		= dT.genTracker()
		self.RadarInfo 		= dT.radar_info()

	def __call__(self, measList):
		imm_ekfRun(measList, self.tracker, self.trackingCfg, 0, self.RadarInfo)

		return self.getTracker()
	
	def getTracker(self):
		return self.tracker	

def main():
	config = cfg.config()
	aoaCfg = cfg.aoaCfg
	domain = dT.domainGenerator(config, aoaCfg)
	resList = domain.getResolution()
	DT = cfg.trackingCfg.td

	#차선변경 모델
	#FCW 모델 : FLAG  = 4
	FLAG = 4
	if FLAG == 0:
		SIM_TIME = 13
	#차선 변경 모델

	elif FLAG == 1:
		SIM_TIME = 20
	elif FLAG == 4:
		SIM_TIME = 15
	else:
		SIM_TIME = 15
	trackingCfg = cfg.trackingCfg()
	tracker = dT.genTracker()
	print(__file__ + " start!!")
	show_animation = True
	time = 0.0

	# State Vector [x y yaw v]'
	xEst = np.zeros((4, 1))
	xTrue = np.zeros((4, 1))
	if FLAG == 0:
		xTrue[0][0] = 5  # x
		xTrue[1][0] = 8  # y
		xTrue[2][0] = 90 * np.pi/180  # pi, 진행방향
		xTrue[3][0] = 0  # velocity
	elif FLAG == 1:

		xTrue[0][0] = 15  # x
		xTrue[1][0] = 62  # y
		xTrue[2][0] = 260 * np.pi/180  # pi, 진행방향
		xTrue[3][0] = 20  # velocity
	elif FLAG == 4:

		xTrue[0][0] = 0  # x
		xTrue[1][0] = 38  # y
		xTrue[2][0] = 90 * np.pi/180  # pi, 진행방향
		xTrue[3][0] = 0  # velocity

	else:
		#차선변경 모댈
		xTrue[0][0] = 0  # x
		xTrue[1][0] = 26  # y
		xTrue[2][0] = -1 * np.pi / 2  # pi, 진행방향
		xTrue[3][0] = 0  # velocity

	xEst = copy.deepcopy(xTrue)
	xEst0 = copy.deepcopy(xTrue)
	xEst1 = copy.deepcopy(xTrue)
	# history
	hxEst = xEst
	hxEst0 = xEst0
	hxEst1 = xEst1
	hxTrue = xTrue
	hz = np.zeros((2, 1))
	hz[0][0] = xTrue[0][0]
	hz[1][0] = xTrue[1][0]
	v= 0
	while SIM_TIME >= time:
		if DEBUG:
			print("=============================================== time : %f==========================================="%time)
		time += DT
		if FLAG == 0:
			if time < 4:
				v = 10
				w = 0
			elif time < 10:
				v = 10
				w = 1
			else:
				v = 10
				w = 0
		elif FLAG == 1:
			v = 20
			w = 0.08
		elif FLAG == 4:
			if time < 2:
				a = 0
			elif time <10:
				a = -0.7
			elif time <15:
				a = 1.2

			v += a * DT
			w = 0
		else:
			#차선변경 모댈
			if time < 10:
				v = 1  # m/s
				w = 0  # rad/s
			elif time < 11:
				v = 3
				w = -0.8
			elif time < 12:
				v = 3
				w = 0.8
			else:
				v = 1
				w = 0
		u = tm.calc_input(v, w)

		xTrue, z, z_rrd = tm.observation(xTrue, u, resList)
		# print(z_rrd)
		cfarOut3DList = tm.make_cfarOutFmt3D(z_rrd)

		clusterList, clusterOutputList = CL.clusteringDBscanRun(cfarOut3DList, cfg.clusterCfgMRR, resList, domain)
		measList = ml.populateTrackingList(clusterOutputList)  # measList is list consisted with class trackingInput
		measList = ml.pruneTrackingInput(measList, aoaCfg, trackingCfg, 0)
		print("det")
		for cluster in clusterList:
			for target in cluster:
				print(
					"ID : %2d, x : %5.2f, y : %6.2f, v : %5.2f, rangeSNR : %5.2f, dopplerSNR : %5.2f, angleSNR : %5.2f, peakVal : %5.2f " % \
					(target.clusterId, target.x, target.y, target.yd, target.rangeSNR * 6, target.dopplerSNR * 6,
					 target.angleSNR * 6, target.rangeVal))


		# print(measList)

		if CONST.IMM:
			imm_ekfRun(measList, tracker, trackingCfg, time)
		else:
			Qvec = ml.InitQvec(trackingCfg.td)
			ml.ekfRun(measList, tracker, trackingCfg, Qvec, time)
		print("\ntracker")
		for track in tracker.List:
			xSize = 2 * track.xSize
			ySize = 2 * track.ySize
			print("ID : %2d, x : %6.2f, y : %7.2f, vx : %6.2f, vy : %7.2f, xSize : %6.2f, ySize : %6.2f, age : %3d, tick : %3d" %(track.ID, track.stateVectorXYZ[0], track.stateVectorXYZ[1], track.stateVectorXYZ[2], track.stateVectorXYZ[3], xSize, ySize, track.age, track.tick))


		xEst[0][0] = tracker.List[0].stateVectorXYZ[0]
		xEst[1][0] = tracker.List[0].stateVectorXYZ[1]
		xEst0[0][0] = tracker.List[0].state[0].stateVectorXYZ[0]
		xEst0[1][0] = tracker.List[0].state[0].stateVectorXYZ[1]
		xEst1[0][0] = tracker.List[0].state[1].stateVectorXYZ[0]
		xEst1[1][0] = tracker.List[0].state[1].stateVectorXYZ[1]
		hxEst = np.hstack((hxEst, xEst))
		hxEst0 = np.hstack((hxEst0, xEst0))
		hxEst1 = np.hstack((hxEst1, xEst1))
		hxTrue = np.hstack((hxTrue, xTrue))

		hz = np.hstack((hz, z))
		# print(hz)
		if show_animation:
			plt.cla()
			# for stopping simulation with the esc key.
			plt.gcf().canvas.mpl_connect('key_release_event',
			                             lambda event: [exit(0) if event.key == 'escape' else None])
			plt.plot(hz[0, :], hz[1, :], ".g")
			plt.plot(hxTrue[0, :].flatten(),
			         hxTrue[1, :].flatten(), "-b")
			plt.plot(hxEst[0, :].flatten(),
			         hxEst[1, :].flatten(), "-r")
			plt.plot(hxEst0[0, :].flatten(),
			         hxEst0[1, :].flatten(), "--k")
			plt.plot(hxEst1[0, :].flatten(),
			         hxEst1[1, :].flatten(), "-k")
			# plot_covariance_ellipse(xEst, PEst)
			plt.axis("equal")
			plt.grid(True)
			plt.pause(0.001)
	plt.savefig('{}image/{}.png'.format(CONST.dataPath, time))



	if CONST.trackingLogDEBUG:
		print("hi")
		logNp = np.array(ml.logList, dtype=ml.logDtype)
		csvWriter = csv.writer(ml.logTXT)
		# ml.logTXT.write("%f"%logNp[:]["x"])
		# print(ml.logDtype[:])
		csvWriter.writerow(ml.logDtype[:])
		for row in logNp:
			csvWriter.writerow(row)
		# plt.title("{} log".format(CONST.dataPath))
		# plt.grid(linestyle=":")
		# plt.plot(logNp[:]["frameNumber"], logNp[:]["x"])
		# plt.show()

		ml.logTXT.close()
	plt.show()


if __name__ == '__main__':
	if not (os.path.isdir("{}image".format(CONST.dataPath))):
		os.makedirs(os.path.join("{}image".format(CONST.dataPath)))
	if not (os.path.isdir("{}image/heatmap".format(CONST.dataPath))):
		os.makedirs(os.path.join("{}image/heatmap".format(CONST.dataPath)))
	main()