import numpy as np
import dataStructure as DT
import matplotlib.pyplot as plt
import configManager_MRR_DEMO as cfgMan
import struct
import CONST
import mmwavelib as ml
import matplotlib.colors as mcolors

def pltRCS(trackList, RCSList):
	for track in trackList:
		if track.ID != CONST.RCS_DEBUG_ID_NUMBER:
			continue
		print(
			"ID : %3d, x : %8.2f, y : %7.2f, vx : %7.2f, vy : %7.2f, xSize : %7.2f, ySize : %7.2f, age : %4d, tick : %4d" % (
				track.ID, track.stateVectorXYZ[0], track.stateVectorXYZ[1], track.stateVectorXYZ[2],
				track.stateVectorXYZ[3], track.xSize, track.ySize, track.age, track.tick))
		print("RCS : {:2.2f}, SNR : {:2.2f} RCS_var : {:2.2f}, RCS_slop : {:2.2f}".format(track.RCS, track.SNR, track.RCS_variation, track.RCS_slope))
		# print("{},{},{}".format(track.RCS, track.RCS_variation, track.measVectorRRD[0]))

		RCSList.append((track.stateVectorXYZ[1], track.RCS, track.RCS_variation, track.RCS_slope))

def plotMinTracker(trackList, xMaxRange, xStep, DEBUG = False):
	xCutline = np.arange(-xMaxRange ,xMaxRange, xStep)
	xCutlineLength = len(xCutline)
	tmpMinYList = np.ones(xCutlineLength) * 180
	pltList = np.zeros(xCutlineLength, dtype = DT.trackingInput)
	tmpList = []

	for track in trackList:
		if track.validity != True:
			continue
		for i in range(xCutlineLength):
			if track.stateVectorXYZ[0] >= xCutline[i] and track.stateVectorXYZ[0] < xCutline[i] + xStep:
				if track.stateVectorXYZ[1] < tmpMinYList[i]:
					tmpMinYList[i] = track.stateVectorXYZ[1]
					pltList[i] = track
					break

	if DEBUG == True:
		for track in pltList:
			if track != 0:
				plt.scatter(track.stateVectorXYZ[0],
				            track.stateVectorXYZ[1],
				            marker='o',
				            facecolors='none',
				            edgecolors='red',
				            s = 150)
	else:
		return pltList


def writeOutRun(targetRawFile, frameNumber, cfarOut3DList):
	writeOutFRAMEHeader(targetRawFile, frameNumber)
	writeOutTL(targetRawFile, 1, len(cfarOut3DList))
	writeOutDetObj(targetRawFile, cfarOut3DList)


def writeOutFRAMEHeader(targetRawFile, frameNumber):
	vehicle_speed = 0 
	totalPacketLen = 0
	TLV_xyz_QFormat = 7
	platform = 0
	timeStamp = 0
	numTLVs=0
	subFrameNumber = 0
	numDetectedObj = 0
	magicID0 = struct.pack('<H', 0x0102)
	targetRawFile.write(magicID0)
	magicID1 = struct.pack('<H', 0x0304)
	targetRawFile.write(magicID1)
	magicID2 = struct.pack('<H', 0x0506)
	targetRawFile.write(magicID2)
	magicID3 = struct.pack('<H', 0x0708)
	targetRawFile.write(magicID3)

	vehicle_speed = struct.pack('<I', vehicle_speed)
	targetRawFile.write(vehicle_speed)
	totalPacketLen = struct.pack('<I', totalPacketLen)
	targetRawFile.write(totalPacketLen)
	platform = struct.pack('<I', platform)
	targetRawFile.write(platform)
	frameNumber = struct.pack('<I', frameNumber)
	targetRawFile.write(frameNumber)
	timeStamp = struct.pack('<I', timeStamp)
	targetRawFile.write(timeStamp)
	numDetectedObj = struct.pack('<I', numDetectedObj)
	targetRawFile.write(numDetectedObj)
	numTLVs = struct.pack('<I', numTLVs)
	targetRawFile.write(numTLVs)
	subFrameNumber = struct.pack('<I', subFrameNumber)
	targetRawFile.write(subFrameNumber)

def writeOutTL(targetRawFile, type, TLV_ObjNum):
	length = 0 
	TLV_xyz_QFormat = 7

	type = struct.pack('<I', type)
	targetRawFile.write(type)
	length = struct.pack('<I', length)
	targetRawFile.write(length)
	TLV_ObjNum = struct.pack('<H', TLV_ObjNum)
	targetRawFile.write(TLV_ObjNum)
	TLV_xyz_QFormat = struct.pack('<H', TLV_xyz_QFormat)
	targetRawFile.write(TLV_xyz_QFormat)

def writeOutDetObj(targetRawFile, cfarOut3DList):
	for detObj in cfarOut3DList:

		speed = struct.pack('<h', int(detObj.speed * (1 << 7)))
		targetRawFile.write(speed)
		peakVal = struct.pack('<H', int(detObj.rangeVal * (1 << 7)))
		targetRawFile.write(peakVal)
		x = struct.pack('<h', int(detObj.x * (1 << 7)))
		targetRawFile.write(x)
		y = struct.pack('<h', int(detObj.y * (1 << 7)))
		targetRawFile.write(y)
		z = struct.pack('<h', int(0 * (1 << 7)))
		targetRawFile.write(z)
		rangeSNRdB = struct.pack('<H', int(detObj.rangeSNR * (1 << 8)))
		targetRawFile.write(rangeSNRdB)
		dopplerSNRdB = struct.pack('<H', int(detObj.dopplerSNR * (1 << 8)))
		targetRawFile.write(dopplerSNRdB)
		angleSNRdB = struct.pack('<H', int(detObj.angleSNR * (1 << 8)))
		targetRawFile.write(angleSNRdB)
		clusterID = struct.pack('<H', 0)
		targetRawFile.write(clusterID)


def writeOutTrkRun(trackingtargetFile, frameNumber, measList, vehicle_speed):
	vehicle_speed = struct.pack('<I', vehicle_speed)
	trackingtargetFile.write(vehicle_speed)
	frameNumber = struct.pack('<I', frameNumber)
	trackingtargetFile.write(frameNumber)
	TrkNum = struct.pack('<H', int(len(measList)))
	trackingtargetFile.write(TrkNum)

	for currMeas in measList[:]:
		x = struct.pack('<h', int(currMeas.stateVectorXYZ[0] * (1 << 7)))
		trackingtargetFile.write(x)
		y = struct.pack('<h', int(currMeas.stateVectorXYZ[1] * (1 << 7)))
		trackingtargetFile.write(y)


def plotTracker(TrackList, targetRawFileTXT, frameNumber, timeStamp, xMaxRange, xStep, DEBUG = False, plotTrackerOption = True):


	# print("Tracking Data")
	counterTrackingData = 0
	for track in TrackList:
		if track.tick >= cfgMan.trackingCfg.thresholdTick:
			counterTrackingData += 1
			track.plotValidity = True
	TRACKING_DATA_NUMBER = struct.pack('<I',counterTrackingData)

	print("Tracking Data Number : %d"%counterTrackingData)
	targetRawFileTXT.write("Tracking Data Number : %d\n"%counterTrackingData)

	if plotTrackerOption == True:
		TrackList = plotMinTracker(TrackList, xMaxRange, xStep)

	for track in TrackList:
		if plotTrackerOption == True:
			if track == 0:
				continue

		if track.plotValidity == True:

			if CONST.DEFAULT_SIZE_MODIFY:
				xSize = 2 * track.xSize
				ySize = 2 * track.ySize
			print("ID : %2d, x : %6.2f, y : %7.2f, vx : %6.2f, vy : %7.2f, xSize : %6.2f, ySize : %6.2f, age : %3d, tick : %3d, FLAG : %3d, TTC : %6.2f" %(track.ID, track.stateVectorXYZ[0], track.stateVectorXYZ[1], track.stateVectorXYZ[2], track.stateVectorXYZ[3], xSize, ySize, track.age, track.tick, track.statusFlag, track.ttc))
			# print("track.measCovVec : ", track.measCovVec)
			# if track.associatedObj != None:
			# 	print("track.associatedObj.measCovVec : ",track.associatedObj.measCovVec)
			targetRawFileTXT.write("ID : %3d, x : %7.2f, y : %7.2f, vx : %7.2f, vy : %7.2f, xSize : %7.2f, ySize : %7.2f, age : %4d, tick : %4d, FLAG : %3d, TTC : %7.2f\n" %(track.ID, track.stateVectorXYZ[0], track.stateVectorXYZ[1], track.stateVectorXYZ[2], track.stateVectorXYZ[3], track.xSize, track.ySize, track.age, track.tick, track.statusFlag, track.ttc))


			if np.argmax(track.mu) == 1:
				color = 'blue'
			else:
				color = "red"
			if track.statusFlag == 0:
				markerVal = "^"
			elif track.statusFlag == 2:
				markerVal = "v"
			else:
				markerVal = "d"
			plt.scatter(track.stateVectorXYZ[0],
			            track.stateVectorXYZ[1],
			            marker=markerVal,
			            facecolors='none',
			            edgecolors =color,
			            s = 50)

			if CONST.pltTrackerSize:
				rectangle = plt.Rectangle(
					(track.stateVectorXYZ[0] - track.xSize,
					 track.stateVectorXYZ[1]),
					xSize, ySize, ec=color, fc="None")
				plt.gca().add_patch(rectangle)
		if DEBUG == True:

			if track.ID != CONST.RCS_DEBUG_ID_NUMBER and not(CONST.TRACKING_ALL_DEBUG):
				continue


			# print(
			# 	"ID : %3d, x : %8.2f, y : %7.2f, vx : %7.2f, vy : %7.2f, xSize : %7.2f, ySize : %7.2f, age : %4d, tick : %4d" % (
			# 	track.ID, track.stateVectorXYZ[0], track.stateVectorXYZ[1], track.stateVectorXYZ[2],
			# 	track.stateVectorXYZ[3], track.xSize, track.ySize, track.age, track.tick))
			# print("RCS : {:2.2f}, SNR : {:2.2f}".format(track.RCS, track.SNR))
			if CONST.IMM:
				plt.scatter(track.predictedXYZ0[0],
				            track.predictedXYZ0[1],
				            marker='x',
				            facecolors='red',
				            s=40
				            )
				plt.scatter(track.predictedXYZ1[0],
				            track.predictedXYZ1[1],
				            marker='x',
				            facecolors='blue',
				            s=40
				            )

			if track.associatedObj == None:


				plt.scatter(track.stateVectorXYZ[0],
				            track.stateVectorXYZ[1],
				            marker='^',
				            facecolors='white',
				            s=40 * (track.age)/10
				            )

			else:

				plt.scatter(track.stateVectorXYZ[0],
				            track.stateVectorXYZ[1],
				            marker='d',
				            facecolors = "None",
				            edgecolors='black',
				            s=50
				            )

				plt.scatter(track.associatedObj.stateVectorXYZ[0],
				            track.associatedObj.stateVectorXYZ[1],
				            marker='o',
				            facecolors = "None",
				            edgecolors='black',
				            s=50
				            )

				plt.plot([track.stateVectorXYZ[0], track.associatedObj.stateVectorXYZ[0]],
				         [track.stateVectorXYZ[1], track.associatedObj.stateVectorXYZ[1]],
				         color = 'black', linestyle = '-')

			# buff = struct.pack('<H',track.ID)
			# targetRawFile.write(buff)
			# buff = struct.pack('<h',int(track.stateVectorXYZ[0]*128)) #x #128 is Qformat
			# # print(struct.unpack('<h',buff)[0]/128)
			# targetRawFile.write(buff)
			# buff = struct.pack('<h',int(track.stateVectorXYZ[1]*128)) #y #128 is Qformat
			# targetRawFile.write(buff)
			# buff = struct.pack('<h',int(track.stateVectorXYZ[2]*128))
			# targetRawFile.write(buff)
			# buff = struct.pack('<h',int(track.stateVectorXYZ[3]*128))
			# targetRawFile.write(buff)
			# buff = struct.pack('<h',int(track.xSize*128))
			# targetRawFile.write(buff)
			# buff = struct.pack('<h',int(track.ySize*128))
			# targetRawFile.write(buff)
			# buff = struct.pack('<H',track.tick)
			# targetRawFile.write(buff)
			# buff = struct.pack('<H',track.age)
			# targetRawFile.write(buff)

			# plt.set_facecolor("none")
			# if not(track.associatedObj == None):
			# 	print("associatedObj  xRange : %8.2f, yRange : %7.2f, xSpeed : %7.2f, ySpeed : %7.2f, measCovVec[0] : %4.2f, measCovVec[1] : %4.2f, measCovVec[2] : %4.2f"%(track.associatedObj.stateVectorXYZ[0], track.associatedObj.stateVectorXYZ[1], track.associatedObj.stateVectorXYZ[2], track.associatedObj.stateVectorXYZ[3], track.associatedObj.measCovVec[0], track.associatedObj.measCovVec[1], track.associatedObj.measCovVec[2]))


			# 	plt.scatter(track.associatedObj.stateVectorXYZ[0], track.associatedObj.stateVectorXYZ[1], c = 'white', s = 10)
	return counterTrackingData
def plotDetObj__x(cfarOut3DList):
	for detObj in cfarOut3DList:
		print("targetIdxPlotSpec -> xRange : %8.2f, yRange : %7.2f, xSpeed : %7.2f, ySpeed : %7.2f, rangeSNR : %7.2f, dopplerSNR : %7.2f, angleSNR : %7.2f, peakVal : %5.2f" %(detObj.x, detObj.y, detObj.xd, detObj.yd, detObj.rangeSNR*6, detObj.dopplerSNR*6, detObj.angleSNR*6, detObj.rangeVal))
		print("rangeIdx : %d, dopplerIdx : %d"%(detObj.rangeIdx, detObj.dopplerIdx))
		plt.scatter(detObj.x, detObj.y, c = 'red', marker = "+", s = 200)
	# plt.scatter(0, 50, c='gray', marker="o", s=30)
	# plt.scatter(-4.37, 50, c='gray', marker="o", s=30)
def plotDetObj__(cfarOut3DList):
	for detObj in cfarOut3DList:
		# print("xRange : %8.2f, yRange : %7.2f, xSpeed : %7.2f, ySpeed : %7.2f, rangeSNR : %7.2f, dopplerSNR : %7.2f, angleSNR : %7.2f" %(detObj.x, detObj.y, detObj.xd, detObj.yd, detObj.rangeSNR*6, detObj.dopplerSNR*6, detObj.angleSNR*6))
		plt.scatter(detObj.x, detObj.y, c = 'powderblue', s = 30)

def plotDetObj(cfarOut3DList):

	for detObj in cfarOut3DList:

		# print("xRange : %8.2f, yRange : %7.2f, xSpeed : %7.2f, ySpeed : %7.2f, rangeSNR : %7.2f, dopplerSNR : %7.2f, angleSNR : %7.2f" %(detObj.x, detObj.y, detObj.xd, detObj.yd, detObj.rangeSNR*6, detObj.dopplerSNR*6, detObj.angleSNR*6))
		plt.scatter(detObj.x, detObj.y, c = 'pink', s = 30)

def plotClusterOutput(clusterOutputList):
	if CONST.pltClusterOutput:
		print("Cluster Data")
		for clusterOutput in clusterOutputList:
			print("ID : %2d, xCenter : %5.2f, yCenter : %6.2f, vAvg : %5.2f, xSize : %5.2f, ySize : %5.2f, peakVal : %5.2f "
			      %(clusterOutput.clusterId, clusterOutput.xCenter, clusterOutput.yCenter, clusterOutput.avgVel, clusterOutput.xSize, clusterOutput.ySize, clusterOutput.peakVal))

			plt.scatter(clusterOutput.xCenter, clusterOutput.yCenter, marker = "+", c = 'black', s = 40)
			rectangle = plt.Rectangle((clusterOutput.xCenter - clusterOutput.xSize, clusterOutput.yCenter-clusterOutput.ySize),clusterOutput.xSize * 2, clusterOutput.ySize * 2, ec = "blue", fc = "None")
			plt.gca().add_patch(rectangle)

def plotCluster(clusterList, targetRawFileTXT, isGuardRail = False,DEBUG = False):
	idx = 0

	for cluster in clusterList:

		targetX = []
		targetY = []
		targetIsSlow = []

		colorList = sorted(mcolors.TABLEAU_COLORS.items())
		colorListLen = len(colorList)

		for target in cluster:
			# plt.scatter(target.x, target.y, facecolor = color[idx], s = 50)
			targetX.append(target.x)
			targetY.append(target.y)

			if CONST.printDetTarget:
				targetRawFileTXT.write("ID : %2d, x : %5.2f, y : %6.2f, v : %5.2f, rangeSNR : %5.2f, dopplerSNR : %5.2f, angleSNR : %5.2f, peakVal : %5.2f\n"%\
				      (target.clusterId, target.x, target.y, target.yd, target.rangeSNR * 6, target.dopplerSNR * 6,  target.angleSNR * 6, target.rangeVal))
				print("ID : %2d, x : %5.2f, y : %6.2f, v : %5.2f, rangeSNR : %5.2f, dopplerSNR : %5.2f, angleSNR : %5.2f, peakVal : %5.2f, FLAG : %d, isSlow : %d"%\
				      (target.clusterId, target.x, target.y, target.yd, target.rangeSNR * 6, target.dopplerSNR * 6, target.angleSNR * 6, target.rangeVal, target.statusFlag, target.isSlow))

			if target.isSlow == 1 and target.statusFlag == 1:
				markerVal = 'X'
			elif target.isSlow == 0 and target.statusFlag == 1:
				markerVal = 'o'
			elif target.isSlow == 1 and target.statusFlag == 0:
				markerVal = 'x'
			elif target.isSlow == 0 and target.statusFlag == 0:
				markerVal = 'D'
			else:
				markerVal = 'v'
			sizeVal = 50
			if isGuardRail:
				markerVal = "1"
				sizeVal = 500
			targetIsSlow.append(markerVal)

			plt.scatter(target.x, target.y, marker=markerVal, c=colorList[(target.clusterId) % (colorListLen)][0], s=sizeVal)

		# if DEBUG == False:
		# 	plt.scatter(targetX, targetY, marker = targetIsSlow,facecolors ='green', s=50)
		# else:
		# 	plt.scatter(targetX, targetY ,  marker = targetIsSlow, s = 50)


		idx += 1

def ROIplt(scale, curvature):
    theta = np.linspace(0, 2 * np.pi, 100)
    x1 = curvature * np.cos(theta) + curvature + scale
    x2 = curvature * np.sin(theta)
    x1l = curvature * np.cos(theta) - (curvature + scale)
    x2l = curvature * np.sin(theta)

    plt.plot(x1,x2, c = "black")
    plt.plot(x1l, x2l, c = "black")

def plotGuardRail(term, guardRailDetCfg):

	degree = guardRailDetCfg.degree
	# print(term)
	if guardRailDetCfg.process == "OLS_xError":
		yDomain = np.linspace(0, 160, 160)

		if degree == 2:
			x = term[0] + term[1] * yDomain
		elif degree == 3:
			x = term[0] + term[1] * yDomain + term[2] *yDomain * yDomain

		plt.plot(x, yDomain)
		plt.text(0, 5, "%d/ %.2f/ %.2f"%(1/(2*term[2]), np.arctan(term[1]) * 180 / np.pi, term[0]))

	else:
		if term[0] / term[1] < 0:
			xDomain = np.linspace(0, 10, 100)
		else:
			xDomain = np.linspace(-10, 0, 100)
		if degree == 2:
			y = term[0] + term[1] * xDomain
		elif degree == 3:
			y = term[0] + term[1] * xDomain + term[2] *xDomain * xDomain

		plt.plot(xDomain,y)

def plotTrackingInput(measList):
	for trackingInput in measList:

		plt.scatter(trackingInput.stateVectorXYZ[0],
		            trackingInput.stateVectorXYZ[1],
		            marker='o',facecolors ='white',
		            edgecolor ='black', s=5)

def pltMahalanobisContour(TrackList, trackCfg):

	for track in TrackList:

		if track.ID != CONST.RCS_DEBUG_ID_NUMBER and not(CONST.TRACKING_ALL_DEBUG):
		# if (track.ID != CONST.RCS_DEBUG_ID_NUMBER or not (track.tick < trackCfg.theresholdTick and (CONST.TRACKING_ALL_DEBUG))):
			continue
		if not ((track.tick >= trackCfg.thresholdTick) or (track.ID == CONST.RCS_DEBUG_ID_NUMBER)):
			continue

		x_ma,y_ma = ml.getAssociatonContour(track, trackCfg.associateGamma, trackCfg.td)

		x_P_prev, y_P_prev, x_P, y_P = ml.getPContour(track, trackCfg.associateGamma)
		# print(x,y)
		plt.plot(x_ma, y_ma,color='royalblue', linestyle = ":")
		if track.associatedObj != None:
			x_R, y_R = ml.getRContour(track, trackCfg.associateGamma)
			plt.plot(x_R, y_R, color='lightgreen', linestyle = ":")
		plt.plot(x_P, y_P, color='orangered', linestyle = ":")
		# plt.plot(x_P_prev, y_P_prev, color='gray', linestyle=":")
		# plt.show()
def pltPredictedTracker(TrackList,td):
	for track in TrackList:
		# print(track.tick)
		if track.ID != CONST.RCS_DEBUG_ID_NUMBER and not(CONST.TRACKING_ALL_DEBUG):
			continue

		xPred = track.stateVectorXYZ[0] + td * track.stateVectorXYZ[2]
		yPred = track.stateVectorXYZ[1] + td * track.stateVectorXYZ[3]

		plt.scatter(xPred, yPred,
		            marker='^', facecolors ='green', edgecolor = None, s = 50, label = "estimation before kalman")
		plt.plot([track.stateVectorXYZ[0],xPred],[track.stateVectorXYZ[1], yPred],
		         color = 'blue')
