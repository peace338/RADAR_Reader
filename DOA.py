import numpy as np
import matplotlib.pyplot as plt
import dataStructure as DT
import configManager_MRR_DEMO as cfgMan
import copy
import mmwavelib as ml
import sys
import CONST
import signalGen as sg
import math
# Functions

def steeringVector(theta, numChannel):

	a = np.exp(1j*np.arange(numChannel)*np.pi*np.sin(theta))
	a = a.reshape(-1,1)
	a = a
	# print("a",a)
	return a

#=============================================================
def DOA_3DFFT__(xx, aoaCfg, domain):

	angleDomain = domain
	azimuthIn = np.zeros(aoaCfg.numAngleBins, dtype=np.complex64)
	p = np.zeros(angleDomain.shape[0], dtype=np.complex64)

	idx = 0
	for val in xx:
		azimuthIn[idx] = val / 8
		idx += 1

	tmp3DFFT = np.fft.fft(azimuthIn)
	tmp3DFFT = np.fft.fftshift(tmp3DFFT, axes=(0,))
	p = tmp3DFFT

	# print(p)

	return p
def DOA_3DFFT(xx, aoaCfg, domain):

	angleDomain = domain.angleDomain
	azimuthIn = np.zeros(aoaCfg.numAngleBins, dtype=np.complex64)
	p = np.zeros(angleDomain.shape[0], dtype=np.complex64)

	idx = 0
	for val in xx:
		azimuthIn[idx] = val / 8
		idx += 1

	tmp3DFFT = np.fft.fft(azimuthIn)
	tmp3DFFT = np.fft.fftshift(tmp3DFFT, axes=(0,))
	p = abs(tmp3DFFT)
	idxList, _ = ml.peakDetection(p, aoaCfg, multi=True)
	# print(p)

	return p, idxList

def genA(channelNum, angleDomain, aoaCfg):

	A = np.zeros((channelNum, len(angleDomain)),dtype = np.complex64)

	for idx, theta in enumerate(angleDomain):
		A[:, idx] = steeringVector(theta, aoaCfg.numAziChannel).reshape(aoaCfg.numAziChannel,)
	return A

def mutualCoherence(A):
	print(A.shape[1])
	maxVal = 0
	for i in range(A.shape[1] - 1):
		for j in range(i+1, A.shape[1]):

			tmp = np.conjugate(np.transpose(A[:,i])) @ A[:,j]
			# print(tmp)
			# print(A[:,i])
			# print(np.linalg.norm(A[:,i]))
			tmp = abs(tmp) / (np.linalg.norm(A[:,i])) / (np.linalg.norm(A[:,j]))
			# print(tmp)


			if tmp > maxVal:
				maxVal = tmp

	return maxVal

def SL0 (xx, aoaCfg, domain):

	sigma = np.array([1, 0.5,0.3, 0.2, 0.1, 0.05, 0.03, 0.02,0.01,0.008,0.006,0.005,0.003, 0.001, 0.0006])
	# sigma = np.array([1, 0.5,0.3, 0.2, 0.1, 0.05, 0.03, 0.02,0.01])

	mu = 3
	loopTime = 3
	angleDomainRadian = np.arange(-1 * aoaCfg.maxAngleH, aoaCfg.maxAngleH, aoaCfg.deltaTheta)
	# angleDomainRadian = domain.angleDomain
	A = genA(aoaCfg.numAziChannel, angleDomainRadian, aoaCfg)
	y = xx
	y = y.reshape(-1,1)

	x = np.conjugate(np.transpose(A))@np.linalg.inv(A@np.conjugate(np.transpose(A)))@y
	x = OMP__(xx, aoaCfg, domain)
	x = x.reshape(-1,1)
	# x = DOA_3DFFT(xx, aoaCfg, domain)
	# x = x.reshape(-1, 1)
	# print("afs",x.shape)
	v = x
	delta_s = np.zeros((x.shape[0],1), dtype= np.complex64)
	for sig in sigma:
		s = v
		for j in range(loopTime):

			for i in range(x.shape[0]):

				delta_s[i][0] = s[i][0] * np.exp(-1 * s[i][0] * np.conjugate(s[i][0]) / (2*sig**2))

			s = s - mu * delta_s

			s = s - np.conjugate(np.transpose(A))@np.linalg.inv(A@np.conjugate(np.transpose(A)))@(A@s - y)
		v = s


	s = v

	s = s.reshape(-1)
	# print(abs(s))
	# s.reshape()
	return abs(s)
def rangeSelcet(q, RayleighLimit, angleDomain):
	angleDomainResolution = angleDomain[1] - angleDomain[0]

	lowerAngle = angleDomain[q] - RayleighLimit/2
	if lowerAngle < angleDomain[0]:
		lowerAngle = angleDomain[0]
	upperAnlge = angleDomain[q] + RayleighLimit/2
	if upperAnlge > angleDomain[-1]:
		upperAnlge = angleDomain[-1]
	# print("range",lowerAngle*180/np.pi, upperAnlge*180/np.pi)
	lowerBound = round((lowerAngle - angleDomain[0])/angleDomainResolution)
	upperBound = round((upperAnlge - angleDomain[0])/angleDomainResolution)
	# print(range(int(lowerBound), int(upperBound)))
	ILSRange = list(range(int(lowerBound), int(upperBound)))

	return ILSRange
def Moore_Penrose_inv(A):

	tmp = np.conjugate(np.transpose(A)) @ np.linalg.inv(A @ np.conjugate(np.transpose(A)))
	return tmp

def FOCUSS(xx, aoaCfg, l = 1, general = False):
	MaxInteration = 10
	SNR = 20

	sigma = abs(xx[0]) * 10 **(-SNR/20)
	# l = 1
	scale = 10 ** -5
	angleDomainRadian = np.arange(-1 * aoaCfg.maxAngleH, aoaCfg.maxAngleH, aoaCfg.deltaTheta)
	A = genA(aoaCfg.numAziChannel, angleDomainRadian, aoaCfg)
	R = np.eye(aoaCfg.numAziChannel) * (sigma ** 2)
	y = xx
	y = y.reshape(-1, 1)

	iterNum = 0


	#init x with CAPON
	x = CAPON(xx, angleDomainRadian, aoaCfg, sigma)
	x = x.reshape(-1,1)

	# init x with FFT
	# x = DOA_3DFFT__(xx, aoaCfg, angleDomainRadian)
	# x = x.reshape(-1, 1)
	# init x with OMP

	# x = OMP(xx, aoaCfg)
	# x = x.reshape(-1, 1)

	# plt.plot(abs(x))
	# plt.show()
	while iterNum < MaxInteration:
		iterNum += 1
		if general:
			W = np.diag(x.reshape(-1) ** l)
			q = Moore_Penrose_inv(A @ W) @ y
			x = W @ q

		else:
			W = np.diag(x.reshape(-1))
			q = Moore_Penrose_inv(A@W)@y
			x = W @ q

			threshold = max(abs(x)) * scale
			for idx, val in enumerate(x):
				if abs(val) < threshold:
					x[idx]= 0

	idxList, _ = ml.peakDetection(abs(x), aoaCfg, multi=True)

	return abs(x), idxList


def ILS_OMP(xx, aoaCfg):

	MaxInteration = 10
	RayleighLimit = 30
	angleDomainRadian = np.arange(-1 * aoaCfg.maxAngleH, aoaCfg.maxAngleH, aoaCfg.deltaTheta)
	A = genA(aoaCfg.numAziChannel, angleDomainRadian, aoaCfg)

	y = xx
	y = y.reshape(-1,1)
	r = y.reshape(-1,1)
	Ai = np.empty((aoaCfg.numAziChannel, 0))
	s = np.zeros(len(angleDomainRadian), dtype = np.complex64)

	idxList = np.zeros(len(angleDomainRadian))
	lambdaList = []
	for i in range(2):
		l = 0
		maxIdx = 0

		for j in range(len(angleDomainRadian)):

			if idxList[j] != 1:
				tmp_l = abs(np.conjugate(A[:, j]).reshape(1, -1) @ r)

				if tmp_l[0,0] > l:
					l = tmp_l
					maxIdx = j

		idxList[maxIdx] = 1
		lambdaList.append(maxIdx)
		Ai = np.append(Ai, A[:, maxIdx].reshape(-1,1), axis = 1)
		si = np.linalg.inv(np.transpose(np.conjugate(Ai)) @ Ai) @ np.transpose(np.conjugate(Ai)) @ y
		r = y - Ai @ si

		s[maxIdx] = si[i][0]

	iterNum = 0
	# s = np.zeros(len(angleDomainRadian), dtype=np.complex64)
	while iterNum < MaxInteration:
		maxVal = 0
		maxIdx = 0

		for q in lambdaList:

			lambdaList.remove(q)
			# s[q] = 0
			ILSRange = rangeSelcet(q, RayleighLimit * np.pi / 180, angleDomainRadian)
			minVal = 655350
			minIdx = None
			for qd in ILSRange:
				# if qd in lambdaList or qd+1 in lambdaList or qd-1 in lambdaList:
				if qd in lambdaList:
					continue
				# print(qd)
				lambdaList.append(qd)
				Aqd = A[:, lambdaList]
				# print(lambdaList)
				# print(Aqd)
				# print(np.transpose(np.conjugate(Aqd)) @ Aqd)
				sqd = np.linalg.inv(np.transpose(np.conjugate(Aqd)) @ Aqd) @ np.transpose(np.conjugate(Aqd)) @ y


				r = y - Aqd @ sqd
				# r = y - Aqd @ sqd + 0.1 * sum(sqd)
				# r = y - Aqd @ sqd
				r_norm = np.linalg.norm(r)
				# r_norm = np.linalg.norm(r) + 0 * sum(abs(sqd))
				# r_norm = np.sqrt(np.linalg.norm(r))*0.5 + 6 * sum(abs(r)**0.5)**(1/0.5)
				# print("afss",qd, r_norm)
				lambdaList.remove(qd)
				if r_norm < minVal:
					minVal = r_norm
					minIdx = qd

			Aqd = A[:, lambdaList]
			sqd = np.linalg.inv(np.transpose(np.conjugate(Aqd)) @ Aqd) @ np.transpose(np.conjugate(Aqd)) @ y
			# s[qd] = sqd[-1][0]
			if minIdx == None:
				# print("hi")
				minIdx = q
			lambdaList.append(minIdx)

		iterNum += 1

	Aqd = A[:, lambdaList]
	sqd = np.linalg.inv(np.transpose(np.conjugate(Aqd)) @ Aqd) @ np.transpose(np.conjugate(Aqd)) @ y
	# print(lambdaList)
	s = np.zeros(len(angleDomainRadian), dtype=np.complex64)
	s[lambdaList] = sqd.reshape(-1)

	#<======================peak detection`===========================

	tmps = abs(s)

	tmp_idx = np.argmax(tmps[lambdaList])

	if tmp_idx == 0:
		another_idx = 1
	else:
		another_idx = 0
	# print(lambdaList[tmp_idx],lambdaList[another_idx])
	if aoaCfg.multiPeakScale * tmps[lambdaList[tmp_idx]] <  tmps[lambdaList[another_idx]]:
		pass
	else:
		lambdaList.remove(lambdaList[another_idx])
	# =====================peak detection`===========================>
	return abs(s), lambdaList

def CAPON(input, angleDomainRadian, aoaCfg, sigma):

	pmvdr = np.zeros(len(angleDomainRadian), dtype=np.complex64)
	xx = np.matrix(input)
	# print(xx.shape)
	Rxx = xx.T * xx.T.H
	R = np.eye(aoaCfg.numAziChannel) * (sigma ** 2)
	invRxx = np.linalg.inv(Rxx + R)

	idxP = 0
	for theta in angleDomainRadian:
		a = ml.steeringVector(theta, aoaCfg.numAziChannel)

		pmvdr[idxP] = 1 / (a.H * invRxx * a)

		idxP += 1

	return pmvdr

def ILS_OMP_MMSE(xx, aoaCfg):

	MaxInteration = 10
	RayleighLimit = 30

	threshold = 0.1
	sigma = 0.1

	angleDomainRadian = np.arange(-1 * aoaCfg.maxAngleH, aoaCfg.maxAngleH, aoaCfg.deltaTheta)
	A = genA(aoaCfg.numAziChannel, angleDomainRadian, aoaCfg)
	R = np.eye(aoaCfg.numAziChannel) * (sigma ** 2)
	# print("R", R)
	y = xx
	y = y.reshape(-1,1)
	r = y.reshape(-1,1)
	Ai = np.empty((aoaCfg.numAziChannel, 0))
	s = np.zeros(len(angleDomainRadian), dtype = np.complex64)

	idxList = np.zeros(len(angleDomainRadian))
	lambdaList = []
	for i in range(2):
		l = 0
		maxIdx = 0

		for j in range(len(angleDomainRadian)):

			if idxList[j] != 1:
				tmp_l = abs(np.conjugate(A[:, j]).reshape(1, -1) @ r)

				if tmp_l[0,0] > l:
					l = tmp_l
					maxIdx = j

		idxList[maxIdx] = 1
		lambdaList.append(maxIdx)
		Ai = np.append(Ai, A[:, maxIdx].reshape(-1,1), axis = 1)
		si = np.linalg.inv(np.transpose(np.conjugate(Ai)) @ Ai) @ np.transpose(np.conjugate(Ai)) @ y
		r = y - Ai @ si

		s[maxIdx] = si[i][0]

	iterNum = 0
	# sqd = np.transpose(np.conjugate(A)) @ y
	# s = np.zeros(len(angleDomainRadian), dtype=np.complex64)
	while iterNum < MaxInteration:
		maxVal = 0
		maxIdx = 0

		for q in lambdaList:

			lambdaList.remove(q)
			# s[q] = 0
			ILSRange = rangeSelcet(q, RayleighLimit * np.pi / 180, angleDomainRadian)
			minVal = 655350
			minIdx = None


			for qd in ILSRange:
				# if qd in lambdaList or qd+1 in lambdaList or qd-1 in lambdaList:
				if qd in lambdaList:
					continue
				# print(qd)
				lambdaList.append(qd)
				Aqd = A[:, lambdaList]
				# print(lambdaList)
				# print(Aqd)
				# print(np.transpose(np.conjugate(Aqd)) @ Aqd)
				sqd = np.linalg.inv(np.transpose(np.conjugate(Aqd)) @ Aqd) @ np.transpose(np.conjugate(Aqd)) @ y

				P = sqd @ np.conjugate(np.transpose(sqd))
				P = np.diag(np.diag(P))
				if(np.linalg.matrix_rank(Aqd @ P @ np.conjugate(np.transpose(Aqd))+ R) < 4):
					print("error")
					print(lambdaList)
					print(np.linalg.matrix_rank(Aqd @ P @ np.conjugate(np.transpose(Aqd))+ R))
					print(P)
					print(Aqd)
					print(Aqd @ P @ np.conjugate(np.transpose(Aqd)) + R)
					print(np.linalg.matrix_rank(Aqd @ P @ np.conjugate(np.transpose(Aqd)) + R))
					print(np.linalg.inv(Aqd @ P @ np.conjugate(np.transpose(Aqd)) + R))
				W = np.linalg.inv(Aqd @ P @ np.conjugate(np.transpose(Aqd)) + R) @ Aqd @ P
				sqd = np.conjugate(np.transpose(W)) @ y
				# r = sqd - np.conjugate(np.transpose(W)) @ y
				r = y - Aqd @ sqd
				r_norm = np.linalg.norm(r)
				# print("afss",sqd, r,r_norm)
				lambdaList.remove(qd)
				if r_norm < minVal:
					minVal = r_norm
					minIdx = qd

			# Aqd = A[:, lambdaList]
			# sqd = np.linalg.inv(np.transpose(np.conjugate(Aqd)) @ Aqd) @ np.transpose(np.conjugate(Aqd)) @ y
			# s[qd] = sqd[-1][0]
			if minIdx == None:
				# print("hi")
				minIdx = q
			lambdaList.append(minIdx)

		iterNum += 1

	Aqd = A[:, lambdaList]
	sqd = np.linalg.inv(np.transpose(np.conjugate(Aqd)) @ Aqd) @ np.transpose(np.conjugate(Aqd)) @ y
	# print(lambdaList)
	s = np.zeros(len(angleDomainRadian), dtype=np.complex64)
	s[lambdaList] = sqd.reshape(-1)

	#<======================peak detection`===========================

	tmps = abs(s)

	tmp_idx = np.argmax(tmps[lambdaList])

	if tmp_idx == 0:
		another_idx = 1
	else:
		another_idx = 0
	# print(lambdaList[tmp_idx],lambdaList[another_idx])
	if aoaCfg.multiPeakScale * tmps[lambdaList[tmp_idx]] <  tmps[lambdaList[another_idx]]:
		pass
	else:
		lambdaList.remove(lambdaList[another_idx])
	# =====================peak detection`===========================>
	return abs(s), lambdaList


def domainTransform(peakList_FFT, domain, angleDomainRadian):

	theta = domain.angleDomain[peakList_FFT]
	theta = theta - angleDomainRadian[0]
	angleDomainResolution = angleDomainRadian[1] - angleDomainRadian[0]
	idx = int(round(theta / angleDomainResolution))

	return idx
		# if idxList == idxList:
		# 	break
def RISR_FFT(xx, aoaCfg, domain):

	MaxInteration = 10
	RayleighLimit = 30
	threshold = 0.00000009
	SNR = 20

	sigma = abs(xx[0]) * 10 ** (-SNR / 20)

	R = np.eye(aoaCfg.numAziChannel) * (sigma **2)
	_, peakList_FFT = DOA_3DFFT(xx, aoaCfg, domain)
	angleDomainRadian = np.arange(-1 * aoaCfg.maxAngleH, aoaCfg.maxAngleH, aoaCfg.deltaTheta)
	A = genA(aoaCfg.numAziChannel, angleDomainRadian, aoaCfg)
	ILSRange = rangeSelcet(domainTransform(peakList_FFT[0],domain, angleDomainRadian), RayleighLimit * np.pi / 180, angleDomainRadian)
	spectrum_extend = np.zeros(len(angleDomainRadian))

	A = A[:, ILSRange]

	y = xx
	y = y.reshape(-1,1)

	s = np.conjugate(np.transpose(A)) @ y

	condition = True
	iterNum = 0
	while condition:
		iterNum+=1

		P = s @ np.conjugate(np.transpose(s))
		P = np.diag(np.diag(P))
		# print(P)

		#update
		W = np.linalg.inv(A @ P @ np.conjugate(np.transpose(A)) + R) @ A @ P

		s_prev = copy.deepcopy(s)
		s = np.conjugate(np.transpose(W)) @ y

		# condition = np.linalg.norm(s - s_prev) > threshold
		# print(np.linalg.norm(s - s_prev))
		condition = iterNum < MaxInteration and np.linalg.norm(s - s_prev) > threshold

	P = s @ np.conjugate(np.transpose(s))
	# print("hoi",iterNum)
	spectrum = abs(np.sqrt(np.diag(P)))
	# print(ILSRange)
	# print(spectrum_extend[ILSRange])
	spectrum_extend[ILSRange] = spectrum
	idxList, _ = ml.peakDetection(spectrum_extend, aoaCfg, multi=True)

	return spectrum_extend, idxList

def RISR(xx, aoaCfg):

	threshold = 0.00000009
	SNR = 20

	sigma = abs(xx[0]) * 10 ** (-SNR / 20)
	R = np.eye(aoaCfg.numAziChannel) * (sigma **2)

	angleDomainRadian = np.arange(-1 * aoaCfg.maxAngleH, aoaCfg.maxAngleH, aoaCfg.deltaTheta)
	A = genA(aoaCfg.numAziChannel, angleDomainRadian, aoaCfg)

	y = xx
	y = y.reshape(-1,1)

	s = np.conjugate(np.transpose(A)) @ y

	condition = True
	iterNum = 0
	while condition:
		iterNum+=1

		P = s @ np.conjugate(np.transpose(s))
		P = np.diag(np.diag(P))
		# print(P)

		#update
		W = np.linalg.inv(A @ P @ np.conjugate(np.transpose(A)) + R) @ A @ P

		s_prev = copy.deepcopy(s)
		s = np.conjugate(np.transpose(W)) @ y

		# condition = np.linalg.norm(s - s_prev) > threshold
		# print(np.linalg.norm(s - s_prev))
		condition = iterNum < 10 and np.linalg.norm(s - s_prev) > threshold

	P = s @ np.conjugate(np.transpose(s))
	# print("hoi",iterNum)
	spectrum = abs(np.sqrt(np.diag(P)))
	# print(spectrum)

	idxList, _ = ml.peakDetection(spectrum, aoaCfg, multi=True)

	return spectrum, idxList

def OMP__(xx, aoaCfg, domain):

	angleDomainRadian = np.arange(-1 * aoaCfg.maxAngleH, aoaCfg.maxAngleH, aoaCfg.deltaTheta)
	# angleDomainRadian = domain.angleDomain
	A = genA(aoaCfg.numAziChannel, angleDomainRadian, aoaCfg)
	# print("MC",mutualCoherence(A))

	y = xx
	y = y.reshape(-1,1)
	# print(A)
	r = y.reshape(-1,1)
	Ai = np.empty((aoaCfg.numAziChannel, 0))
	s = np.zeros(len(angleDomainRadian), dtype = np.complex64)

	idxList = np.zeros(len(angleDomainRadian))
	for i in range(2):
		l = 0
		maxIdx = 0

		for j in range(len(angleDomainRadian)):
			# print(A[:, j].reshape(-1,1))
			# print(np.conjugate(A[:, j].reshape(-1,1)).reshape(1,-1))
			if idxList[j] != 1:
				tmp_l = abs(np.conjugate(A[:, j]).reshape(1, -1) @ r)
				# print("np.conjugate(A[:, j]).reshape(1, -1)",np.conjugate(A[:, j]).reshape(1, -1))
				# print("r", r)
				# print("tmp-l",tmp_l)


				# tmp_l = abs(A[:, j].reshape(1,-1) @ r)
				# print(tmp_l[0,0])
				if tmp_l[0,0] > l:
					l = tmp_l
					maxIdx = j

		idxList[maxIdx] = 1

		# print(A[:, maxIdx].reshape(-1, 1))

		Ai = np.append(Ai, A[:, maxIdx].reshape(-1,1), axis = 1)
		# si = np.linalg.inv(np.transpose(Ai)@ Ai) @ np.transpose(Ai) @ y
		# si = np.linalg.inv(np.transpose(np.conjugate(Ai)) @ Ai) @ np.transpose(np.conjugate(Ai)) @ y
		si = np.linalg.inv(np.transpose(np.conjugate(Ai)) @ Ai) @ np.transpose(np.conjugate(Ai)) @ y
		# print("\nsagsa",si.shape, "\n",si)
		# r = y - Ai@si
		r = y - Ai @ si

		# if np.linalg.norm(r) < 0.0000000000002:
		# 	break
		# print(Ai@si)
		# print("r,", r)
		# s = np.append(s, si[-1])
		# print("asfas",si[-1][0])
		s[maxIdx] = si[i][0]
	# print("asfa",s)
	return s
def OMP(xx, aoaCfg):

	angleDomainRadian = np.arange(-1 * aoaCfg.maxAngleH, aoaCfg.maxAngleH, aoaCfg.deltaTheta)
	# angleDomainRadian = domain.angleDomain
	A = genA(aoaCfg.numAziChannel, angleDomainRadian, aoaCfg)
	# print("MC",mutualCoherence(A))

	y = xx
	y = y.reshape(-1,1)
	# print(A)
	r = y.reshape(-1,1)
	Ai = np.empty((aoaCfg.numAziChannel, 0))
	s = np.zeros(len(angleDomainRadian))

	idxList = np.zeros(len(angleDomainRadian))
	for i in range(2):
		l = 0
		maxIdx = 0

		for j in range(len(angleDomainRadian)):
			# print(A[:, j].reshape(-1,1))
			# print(np.conjugate(A[:, j].reshape(-1,1)).reshape(1,-1))
			if idxList[j] != 1:
				tmp_l = abs(np.conjugate(A[:, j]).reshape(1, -1) @ r)
				# print("np.conjugate(A[:, j]).reshape(1, -1)",np.conjugate(A[:, j]).reshape(1, -1))
				# print("r", r)
				# print("tmp-l",tmp_l)


				# tmp_l = abs(A[:, j].reshape(1,-1) @ r)
				# print(tmp_l[0,0])
				if tmp_l[0,0] > l:
					l = tmp_l
					maxIdx = j

		idxList[maxIdx] = 1

		# print(A[:, maxIdx].reshape(-1, 1))

		Ai = np.append(Ai, A[:, maxIdx].reshape(-1,1), axis = 1)
		# si = np.linalg.inv(np.transpose(Ai)@ Ai) @ np.transpose(Ai) @ y
		# si = np.linalg.inv(np.transpose(np.conjugate(Ai)) @ Ai) @ np.transpose(np.conjugate(Ai)) @ y
		si = np.linalg.inv(np.transpose(np.conjugate(Ai)) @ Ai) @ np.transpose(np.conjugate(Ai)) @ y
		# print("\nsagsa",si.shape, "\n",si)
		# r = y - Ai@si
		r = y - Ai @ si

		# if np.linalg.norm(r) < 0.0000000000002:
		# 	break
		# print(Ai@si)
		# print("r,", r)
		# s = np.append(s, si[-1])
		# print("asfas",si[-1][0])
		s[maxIdx] = si[i][0]
	# print("asfa",s)
	return s

class DOA_RMSE:
	def __init__(self, DOAtype):
		self.DOA_type = DOAtype
		self.RMSEsq = 0
		self.RMSE = 0
		self.RMSE_list = []
		self.numDetPeak = 0
	def cal_RMSE(self, estimateList, angleDomain, trueList):

		self.numDetPeak +=  len(estimateList)

		for estimate in estimateList:
			minVal = 65355

			for true in trueList:
				tmp = (true - angleDomain[estimate]) ** 2

				if tmp < minVal:
					minVal = tmp

			self.RMSEsq += minVal

	def appendRMSE(self):
		print(self.DOA_type, " : ", self.numDetPeak)
		self.RMSE_list.append(np.sqrt(self.RMSEsq/ self.numDetPeak) )

	def resetRMSE(self):
		self.RMSEsq = 0
		self.RMSE = 0
		self.numDetPeak = 0

def randSource(numSource):
	tmp = []
	for i in range(numSource):
		tmpAmp = np.random.rand() + 1j*np.random.rand()
		tmpAmp = tmpAmp/abs(tmpAmp)
		tmp.append(tmpAmp)

	return tmp
def main_RMSE():
	config = cfgMan.config()
	aoaCfg = cfgMan.aoaCfg
	domain = DT.domainGenerator(config, aoaCfg)
	angleDomainRadian = np.arange(-1 * aoaCfg.maxAngleH, aoaCfg.maxAngleH, aoaCfg.deltaTheta)
	iteration_RMSE = 100
	# np.random.seed(6)
	noisePower = 10000
	SNR = -20
	data = DT.dataManager(cfgMan.config)
	domain = DT.domainGenerator(cfgMan.config, cfgMan.aoaCfg)
	resList = domain.getResolution()
	dopplerDomain = domain.dopplerDomain
	rangeDomain = domain.rangeDomain

	lamda = 1  # wavelength
	kappa = np.pi / lamda  # wave number

	M = aoaCfg.numAziChannel  # number of ULA elements
	snr = -20  # signal to noise ratio
	# domain_Error = range(1, 30, 0.5)
	domain_Error = np.arange(5, 15, 0.25)
	DOA_List = ["3DFFT", "ILS_OMP", "RISR", "RISR_FFT", "FOCUSS"]
	DOAobj_List = []
	for doaType in DOA_List:
		globals()["DOAobj_{}".format(doaType)] = DOA_RMSE(doaType)
		DOAobj_List.append(globals()["DOAobj_{}".format(doaType)])

	for angle in domain_Error:
		print("hi",angle)
		for i in range(iteration_RMSE):
			Thetas = np.array([0,angle]) * np.pi / 180  # random source directions



			# Alphas = np.array([1 + 0j, 1 - 0j])  # random source powers
			Alphas = randSource(len(Thetas))
			# Alphas = np.sqrt(1 / 2) * Alphas
			# print(Thetas)
			# print(Alphas)
			L = Thetas.shape[0]  # number of sources
			h = np.zeros(M)
			h = h.reshape(-1, 1)

			A_Measure = np.empty((aoaCfg.numAziChannel, 0))

			for i in range(L):
				# print(h,"\n",i)
				h = h + Alphas[i] * steeringVector(Thetas[i], M)
				A_Measure = np.append(A_Measure, steeringVector(Thetas[i], M).reshape(-1, 1), axis=1)

			h = h + noiseGen(1, snr, M)
			h = h.reshape(-1,)

			if CONST.DO_RX_COMPENSATION:
				h = ml.MmwDemo_rxChanPhaseBiasCompensation(h, aoaCfg.rxChComp)

			for DOAobj in DOAobj_List:
				if DOAobj.DOA_type == "3DFFT":
					_, angleList = DOA_3DFFT(h, aoaCfg, domain)
				elif DOAobj.DOA_type == "ILS_OMP":
					_, angleList = ILS_OMP(h, aoaCfg)
				elif DOAobj.DOA_type == "RISR":
					_, angleList = RISR(h, aoaCfg)
				elif DOAobj.DOA_type == "RISR_FFT":
					_, angleList = RISR_FFT(h, aoaCfg, domain)
				elif DOAobj.DOA_type == "FOCUSS":
					_, angleList = FOCUSS(h, aoaCfg)
				else:
					print("ERROR : It's not a available DOA type")
					sys.exit(1)

				if DOAobj.DOA_type == "3DFFT":
					DOAobj.cal_RMSE(angleList, domain.angleDomain, Thetas)
				else:
					DOAobj.cal_RMSE(angleList, angleDomainRadian, Thetas)

		for DOAobj in DOAobj_List:
			DOAobj.appendRMSE()
			DOAobj.resetRMSE()

	for DOAobj in DOAobj_List:
		plt.plot(domain_Error, np.array(DOAobj.RMSE_list, dtype = np.float)*180/np.pi, label = DOAobj.DOA_type)
	# plt.plot(domain_Error, np.array(Error_ILS_OMP_MMSE_List, dtype=np.float) * 180 / np.pi, label="ILS_OMP_MMSE")
	plt.ylabel('RMSE(deg)')
	plt.xlabel('deg')
	plt.grid(linestyle=":")
	plt.legend(loc=1)
	plt.show()

def main_RMSE_FFT():
	config = cfgMan.config()
	aoaCfg = cfgMan.aoaCfg
	domain = DT.domainGenerator(config, aoaCfg)
	angleDomainRadian = np.arange(-1 * aoaCfg.maxAngleH, aoaCfg.maxAngleH, aoaCfg.deltaTheta)
	iteration_RMSE = 100
	# np.random.seed(6)
	noisePower = 10000
	SNR = -20
	data = DT.dataManager(cfgMan.config)
	domain = DT.domainGenerator(cfgMan.config, cfgMan.aoaCfg)
	resList = domain.getResolution()
	dopplerDomain = domain.dopplerDomain
	rangeDomain = domain.rangeDomain

	lamda = 1  # wavelength
	kappa = np.pi / lamda  # wave number

	M = aoaCfg.numAziChannel  # number of ULA elements

	domain_Error = np.arange(5, 50, 2)
	DOA_List = ["3DFFT", "ILS_OMP", "RISR", "FOCUSS"]
	DOAobj_List = []
	for doaType in DOA_List:
		globals()["DOAobj_{}".format(doaType)] = DOA_RMSE(doaType)
		DOAobj_List.append(globals()["DOAobj_{}".format(doaType)])

	for angle in domain_Error:
		print("hi",angle)
		for i in range(iteration_RMSE):
			print("iter : ", i)
			ThetasInDEG = np.array([0, angle])
			Thetas = ThetasInDEG * np.pi / 180  # random source directions

			data.reset()
			for theta in ThetasInDEG:
				data.adcData += sg.signalGenerator(15, 0, theta, noisePower, SNR)
			data.adcData += sg.noiseGenerator(noisePower)
			ml.RangeFFT(data, config, winType=config.winType)
			ml.DopplerFFT(data, config, "HAMMING")

			heatMap = ml.avgHeatMap(data.FFTData, config, True)
			cfarOut2DList = ml.cfar2D(heatMap, cfgMan.cfarCfgRange, cfgMan.cfarCfgDoppler,
			                          K=cfgMan.cfarCfgDoppler.maxNumDet)
			cfarOut2D = cfarOut2DList[CONST.targetIdxPlotSpec]
			h = data.FFTData[cfarOut2D.rangeIdx, cfarOut2D.dopplerIdx, :]
			if CONST.DO_RX_COMPENSATION:
				h = ml.MmwDemo_rxChanPhaseBiasCompensation(h, aoaCfg.rxChComp)

			for DOAobj in DOAobj_List:
				if DOAobj.DOA_type == "3DFFT":
					_, angleList = DOA_3DFFT(h, aoaCfg, domain)
				elif DOAobj.DOA_type == "ILS_OMP":
					_, angleList = ILS_OMP(h, aoaCfg)
				elif DOAobj.DOA_type == "RISR":
					_, angleList = RISR(h, aoaCfg)
				elif DOAobj.DOA_type == "RISR_FFT":
					_, angleList = RISR_FFT(h, aoaCfg, domain)
				elif DOAobj.DOA_type == "FOCUSS":
					_, angleList = FOCUSS(h, aoaCfg)
				else:
					print("ERROR : It's not a available DOA type")
					sys.exit(1)

				if DOAobj.DOA_type == "3DFFT":
					DOAobj.cal_RMSE(angleList, domain.angleDomain, Thetas)
				else:
					DOAobj.cal_RMSE(angleList, angleDomainRadian, Thetas)

		for DOAobj in DOAobj_List:
			DOAobj.appendRMSE()
			DOAobj.resetRMSE()

	for DOAobj in DOAobj_List:
		plt.plot(domain_Error, np.array(DOAobj.RMSE_list, dtype = np.float)*180/np.pi, label = DOAobj.DOA_type)
	# plt.plot(domain_Error, np.array(Error_ILS_OMP_MMSE_List, dtype=np.float) * 180 / np.pi, label="ILS_OMP_MMSE")
	plt.ylabel('RMSE(deg)')
	plt.xlabel('deg')
	plt.grid(linestyle=":")
	plt.legend(loc=1)
	plt.show()

def noiseGen(amp, snr, channel):

	noiseAmp = amp * 10 **(-snr/20)
	# noiseAmp * np.sqrt(1/2) *
	noise = (np.random.randn(channel) + 1j* np.random.randn(channel))
	for idx, noiseVal in enumerate(noise):
		# tmp = np.sqrt(noiseVal * np.conjugate(noiseVal))
		noise[idx] = noiseVal / abs(noiseVal)
	# print("hi",noise)
	noise = noiseAmp * noise.reshape(-1,1)
	return noise

def main():
	config = cfgMan.config()
	aoaCfg = cfgMan.aoaCfg
	domain = DT.domainGenerator(config, aoaCfg)
	data = DT.dataManager(cfgMan.config)

	noisePower = 10000
	SNR = -20
	# np.random.seed(30)

	lamda = 1 # wavelength
	kappa = np.pi/lamda # wave number

	M = aoaCfg.numAziChannel  # number of ULA elements
	snr = 20 # signal to noise ratio

	ThetasInDEG = np.array([0])
	Thetas = ThetasInDEG * np.pi / 180  # random source directions

	data.reset()
	for theta in ThetasInDEG:
		data.adcData += sg.signalGenerator(15, 0, theta, noisePower, SNR)
	data.adcData += sg.noiseGenerator(noisePower)
	ml.RangeFFT(data, config, winType=config.winType)
	ml.DopplerFFT(data, config, "HAMMING")

	heatMap = ml.avgHeatMap(data.FFTData, config, True)
	cfarOut2DList = ml.cfar2D(heatMap, cfgMan.cfarCfgRange, cfgMan.cfarCfgDoppler,
	                          K=cfgMan.cfarCfgDoppler.maxNumDet)
	cfarOut2D = cfarOut2DList[CONST.targetIdxPlotSpec]
	h = data.FFTData[cfarOut2D.rangeIdx, cfarOut2D.dopplerIdx, :]
	if CONST.DO_RX_COMPENSATION:
		h = ml.MmwDemo_rxChanPhaseBiasCompensation(h, aoaCfg.rxChComp)

	if CONST.DO_RX_COMPENSATION:
		# rxChComp = np.array([19158+5863j, 23451+2858j, 22093+4569j, 16399], dtype = np.complex64) #QFormat 15
		rxChComp = aoaCfg.rxChComp  # QFormat 15
	# xx = xx /8
		print("rxChComp : ", (rxChComp))
		print("Amp of rxChComp : ", abs(rxChComp))
		h = ml.MmwDemo_rxChanPhaseBiasCompensation(h, rxChComp)
	# print(h)
	# h =np.array([-1.810e+09-5.556e+08j, -2.343e+09-1.306e+09j, -2.288e+09-7.876e+08j, -2.106e+09+1.731e+08j])
	##################################################################################################################################
	Thetas = np.array([0]) * np.pi / 180  # random source directions

	# Alphas = np.array([1 + 0j, 1 - 0j])  # random source powers
	Alphas = randSource(len(Thetas))
	# Alphas = np.sqrt(1 / 2) * Alphas
	# print(Thetas)
	# print(Alphas)
	L = Thetas.shape[0]  # number of sources
	h = np.zeros(M)
	h = h.reshape(-1, 1)

	A_Measure = np.empty((aoaCfg.numAziChannel, 0))

	for i in range(L):
		# print(h,"\n",i)
		h = h + Alphas[i] * steeringVector(Thetas[i], M)
		A_Measure = np.append(A_Measure, steeringVector(Thetas[i], M).reshape(-1, 1), axis=1)

	# h = h + noiseGen(1, snr, M)
	h = h
	h = h.reshape(-1, )
	h[0] = h[0] * np.exp(1j * (3) * np.pi / 180)
	h[1] = h[1] * np.exp(1j * (-7) * np.pi / 180)
	h[2] = h[2] * np.exp(1j * (2) * np.pi / 180)
	h[3] = h[3] * np.exp(1j * (-15) * np.pi / 180)
	plt.clf()
	plt.plot(np.arctan(h.imag/h.real) * 180 / np.pi)

	plt.show()

	# h[0] = h[0] / abs(h[0]) * abs(h[3])
	# h[1] = h[1] / abs(h[1]) * abs(h[3])
	# h[2] = h[2] / abs(h[2]) * abs(h[3])
	# h[3] = h[3] / abs(h[3]) * abs(h[3])
	plt.plot(abs(h))
	plt.title("abs")
	plt.show()



######################################################################################################################################################################################
	p3DFFT, peakList_3DFFT = DOA_3DFFT(h, aoaCfg, domain)
	# pOMP = OMP(h, aoaCfg, domain)

	# pSL0 = SL0(h,aoaCfg, domain)
	angleDomainRadian = np.arange(-1 * aoaCfg.maxAngleH, aoaCfg.maxAngleH, aoaCfg.deltaTheta)
	pCAPON = CAPON(h,angleDomainRadian, aoaCfg, 0.1)
	pCAPON = abs(pCAPON)
	pILS, peakList_ILS = ILS_OMP(h, aoaCfg)
	pRISR, peakList_RISR= RISR(h, aoaCfg)
	# pRISR_FFT, peakList_RISR_FFT = RISR_FFT(h, aoaCfg, domain)
	# pFOCUSS, peakList_FOCUSS = FOCUSS(h,aoaCfg)

	# pILS_MMSE, peakList_ILS_MMSE = ILS_OMP_MMSE(h, aoaCfg)
	# print("haha",peakList_RISR)
	# pOMP = pOMP ** 2
	plt.plot(domain.angleDomain*180/np.pi, p3DFFT/max(p3DFFT), label = "3dfft", color = "gray", linestyle = ":")
	plt.scatter(domain.angleDomain[peakList_3DFFT]*180/np.pi,p3DFFT[peakList_3DFFT]/max(p3DFFT), color = "gray")
	# pOMP = OMP(h, aoaCfg, domain)
	# angleDomainRadian = domain.angleDomain
	# plt.plot(angleDomainRadian * 180 / np.pi, pOMP/max(pOMP), label = "OMP")

	#

	# plt.plot(angleDomainRadian * 180 / np.pi, pCAPON / max(pCAPON), label="CAPON", color = "pink")
	plt.plot(angleDomainRadian * 180 / np.pi, pILS / max(pILS), label="ILS_OMP", color="blue")
	plt.plot(angleDomainRadian * 180 / np.pi, pRISR / max(pRISR), label="RISR", color = "green", linestyle = "-.")
	# plt.plot(angleDomainRadian * 180 / np.pi, pILS_MMSE / max(pILS_MMSE), label="ILS_OMP_MMSE", color = "red", linestyle = "--")
	# plt.plot(angleDomainRadian * 180 / np.pi, pRISR_FFT / max(pRISR_FFT), label="RISR_FFT", color="grey", linestyle=":")

	# plt.plot(angleDomainRadian * 180 / np.pi, pSL0 / max(pSL0), label="SL0")
	plt.scatter(angleDomainRadian[peakList_ILS] * 180 / np.pi, pILS[peakList_ILS]/ max(pILS), color="blue")
	# plt.plot(angleDomainRadian * 180 / np.pi, pFOCUSS / max(pFOCUSS), label="FOCUSS", color="red", linestyle="--")
	# plt.scatter(angleDomainRadian[peakList_FOCUSS] * 180 / np.pi, pFOCUSS[peakList_FOCUSS] / max(pFOCUSS),
	#             color="red", marker="v")
	plt.scatter(angleDomainRadian[peakList_RISR] * 180 / np.pi, pRISR[peakList_RISR] / max(pRISR), color="green", marker = "x")
	# plt.scatter(angleDomainRadian[peakList_ILS_MMSE] * 180 / np.pi, pILS_MMSE[peakList_ILS_MMSE] / max(pILS_MMSE), color="red", marker="x")
	# plt.scatter(angleDomainRadian[peakList_RISR_FFT] * 180 / np.pi, pRISR_FFT[peakList_RISR_FFT] / max(pRISR_FFT),
	#             color="grey", marker="x")

	for i in Thetas:
		plt.axvline(x = i * 180 /np.pi, color = 'grey', linestyle=":", label = "true Position")
	# plt.axvline(x = Thetas[1] * 180 /np.pi, color='black', linestyle=":")
	# plt.axvline(x = Thetas[2] * 180 / np.pi, color='red', linestyle=":")
	plt.grid(linestyle=":")
	plt.legend(loc = 1)
	plt.show()

if __name__ == '__main__':
	main()
	# main_RMSE()
	# main_RMSE_FFT()