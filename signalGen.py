import numpy as np
import matplotlib.pyplot as plt
import configManager_MRR_DEMO as cfgMan
import CONST
if CONST.TDM_MIMO:
	import mrr_config_chirp_design_MRR160_TDM_MIMO as MRRchirpCfg
else:
	import mrr_config_chirp_design_MRR160 as MRRchirpCfg
import dataStructure as DT
import mmwavelib as ml
import copy
import plot_data as pltd

alpha = 2 * cfgMan.config.slope_Hz_per_sec / cfgMan.config.lightSpeed_meters_per_sec
beta = 2 * cfgMan.config.carrierFreq * cfgMan.config.period_chirp / cfgMan.config.lightSpeed_meters_per_sec
beta_slow = 2 * cfgMan.config.carrierFreq * cfgMan.config.period_chirp_slow / cfgMan.config.lightSpeed_meters_per_sec
gamma = 4 * np.pi * cfgMan.config.carrierFreq / cfgMan.config.lightSpeed_meters_per_sec
delta = 2 * np.pi * beta / MRRchirpCfg.numTx
def gainPhaseError(data):

	phaseError = np.array([3,72,2,15])# in degree
	gainError = np.array([20.5/19, 2, 1.5,20.5/19.5])

	Error = gainError * np.exp(1j*phaseError*np.pi/180)


	for chirpIdx in range(cfgMan.config.n_chirps):
		for TxantIdx in range(cfgMan.config.numTx):
			for RxantIdx in range(cfgMan.config.numRx):
				antIdx = TxantIdx * cfgMan.config.numRx + RxantIdx
				for sampleIdx in range(0, cfgMan.config.numADCSamples, 1):
					# noiseData[sampleIdx, chirpIdx, antIdx] = (np.random.randn() + 1j* np.random.randn())
					data.adcData[sampleIdx, chirpIdx, antIdx] = Error[RxantIdx] * data.adcData[sampleIdx, chirpIdx, antIdx]

def noiseGenerator(noisePower):
	noiseData = np.zeros((cfgMan.config.numADCSamples, cfgMan.config.n_chirps, cfgMan.config.numRx * cfgMan.config.numTx),
	                   dtype=np.complex64)
	sigma = np.sqrt(noisePower/2)
	# print("sigma : ", sigma)
	for chirpIdx in range(cfgMan.config.n_chirps):
		for TxantIdx in range(cfgMan.config.numTx):
			for RxantIdx in range(cfgMan.config.numRx):
				antIdx = TxantIdx * cfgMan.config.numRx + RxantIdx
				for sampleIdx in range(0, cfgMan.config.numADCSamples, 1):
					# noiseData[sampleIdx, chirpIdx, antIdx] = (np.random.randn() + 1j* np.random.randn())
					noiseData[sampleIdx, chirpIdx, antIdx] = (np.random.normal(0, sigma) + 1j * np.random.normal(0, sigma))


	return noiseData

def signalGenerator(targetRange, velocity, angleDEG, noisePower, SNR, burstPeriod, gpError = False): #unit : range : m ,velocity : m/s, angle : rad, amplitude : V, td(frame period) : sec
	if gpError:
		phaseError = np.array([3, 72, 2, 15])  # in degree
		gainError = np.array([20.5 / 19, 2, 1.5, 20.5 / 19.5])

		phaseError = np.array([0, 0, 0, 0])  # in degree
		gainError = np.array([1, 1, 1, 1])
		Error = gainError * np.exp(1j * phaseError * np.pi / 180)

		print("input gainError : ", gainError)
		print("input phaseError : ", phaseError)
	else:
		Error = np.array([1,1,1,1])

	adcData = np.zeros((cfgMan.config.numADCSamples, cfgMan.config.n_chirps, cfgMan.config.numRx * cfgMan.config.numTx), dtype=np.complex64)
	signalPower = noisePower * (10 ** (SNR / 10))
	# print(signalPower)
	amplitude = np.sqrt(signalPower)
	# print("amp : ", amplitude)
	angle = angleDEG * np.pi / 180
	for chirpIdx in range(cfgMan.config.n_chirps):
		for TxantIdx in range(cfgMan.config.numTx):
			for RxantIdx in range(cfgMan.config.numRx):
				antIdx = TxantIdx * cfgMan.config.numRx + RxantIdx
				for sampleIdx in range(0, cfgMan.config.numADCSamples, 1):

					adcData[sampleIdx, chirpIdx, antIdx] = \
						Error[RxantIdx]*amplitude * np.exp(1j * (2 * np.pi * \
						                                         (alpha * targetRange * sampleIdx / MRRchirpCfg.SampleRate \
		                                                        + beta * velocity * chirpIdx+ antIdx*np.sin(angle)/2) \
                                                                + gamma * targetRange\
						                                        + delta * velocity * TxantIdx))

	adcData_Slow = np.zeros((cfgMan.config.numADCSamples, cfgMan.config.n_chirps, cfgMan.config.numRx * cfgMan.config.numTx),
	                   dtype=np.complex64)

	for chirpIdx in range(cfgMan.config.n_chirps):
		for TxantIdx in range(cfgMan.config.numTx):
			for RxantIdx in range(cfgMan.config.numRx):
				antIdx = TxantIdx * cfgMan.config.numRx + RxantIdx
				for sampleIdx in range(0, cfgMan.config.numADCSamples, 1):
					adcData_Slow[sampleIdx, chirpIdx, antIdx] = \
						Error[RxantIdx] * amplitude * np.exp(1j * (2 * np.pi * \
						                                           (alpha * (targetRange + burstPeriod * velocity) * sampleIdx / MRRchirpCfg.SampleRate \
									                               + beta_slow * velocity * chirpIdx + antIdx * np.sin(angle) / 2) \
						                                           + gamma * (targetRange + burstPeriod * velocity) \
						                                           + delta * velocity * TxantIdx))

	# adcData_Slow[:,35:,:] = 0

	return adcData, adcData_Slow

def main():
	np.random.seed(6)
	config = cfgMan.config()

	noisePower = 10000
	SNR = -5
	burstPeriod = 0.0069
	# burstPeriod = 0.022
	data = DT.dataManager(cfgMan.config)
	domain = DT.domainGenerator(cfgMan.config, cfgMan.aoaCfg)
	resList = domain.getResolution()
	dopplerDomain = domain.dopplerDomain
	rangeDomain = domain.rangeDomain
	boundary = DT.dopplerBoundary()
	# adcFast, adcSlow = signalGenerator(50, -18.48, 0, noisePower, SNR, td)
	adcFast, adcSlow = signalGenerator(50, 0, 0, noisePower, SNR, burstPeriod)
	data.adcData += adcFast
	data.adcData_Slow += adcSlow

	# adcFast, adcSlow = signalGenerator(50, 0, -5, noisePower, SNR, burstPeriod)
	# data.adcData += adcFast
	# data.adcData_Slow += adcSlow

	data.adcData += noiseGenerator(noisePower)
	data.adcData_Slow += noiseGenerator(noisePower)

	# print(data.adcData[:,0,0])
	# plt.plot(data.adcData[:,0,0].real)
	# plt.plot(data.adcData[:, 0, 0].imag)
	# plt.show()
	#
	ml.RangeFFT(data, config, winType=config.winType)
	ml.RangeFFT_Slow(data, config, winType=config.winType)
	# plt.plot(rangeDomain, np.log10(abs(data.FFTData[:, 0, 0])))
	# plt.show()
	if CONST.DO_ANGLE_COMPENSATION_HIGH_RES or CONST.AOADebug:
		dataCube_1dFFT = copy.deepcopy(data.FFTData)
		# dataCube_1dFFT = dataCube_1dFFT * cfgMan.aoaCfg.rxChComp
	ml.DopplerFFT(data, config, "HAMMING")
	ml.DopplerFFT_Slow(data, config, "HAMMING")
	heatMap = ml.avgHeatMap(data.FFTData, config, True)
	heatMap_Slow = ml.avgHeatMap(data.FFTData_Slow, config)

	if CONST.SLOW_CFAR:
		boundary.uB1, boundary.lB1, boundary.uB2, boundary.lB2 = ml.findUsefulDopplerIndx(config, 0)
	cfarOut2DList = []
	cfarOut2DList = ml.cfar2D(cfarOut2DList, heatMap, cfgMan.cfarCfgRange, cfgMan.cfarCfgDoppler, cfgMan.cfarCfgDoppler.maxNumDet, boundary,
	                          isSlow=False)
	if CONST.SLOW_CFAR:
		cfarOut2DList = ml.cfar2D(cfarOut2DList, heatMap, cfgMan.cfarCfgRange, cfgMan.cfarCfgDoppler, cfgMan.cfarCfgDoppler.maxNumDet,
		                          boundary, isSlow=True)
	if CONST.heatMapPlot == True or CONST.OnlySaveHeatmap:

		rangeIdxList = []
		dopplerIdxList = []

		for obj in cfarOut2DList:
			rangeIdxList.append(obj.rangeIdx)
			dopplerIdxList.append(obj.dopplerIdx)
		if cfarOut2DList == []:
			rangeIdxList.append(0)
			dopplerIdxList.append(0)

		fig, ax = plt.subplots()
		isHeatMapFastPlot = False
		if isHeatMapFastPlot:
			c = ax.pcolormesh(dopplerDomain, rangeDomain, heatMap)
		else:

			c = ax.pcolormesh(dopplerDomain, rangeDomain, heatMap_Slow)
		if isHeatMapFastPlot:
			ax.set_title('HeatMap / ' + 'cfarType = ' + cfgMan.cfarCfgRange.cfarType)
		else:
			ax.set_title('HeatMap slow / ' + 'cfarType = ' + cfgMan.cfarCfgRange.cfarType)
		fig.colorbar(c, ax=ax)
		if isHeatMapFastPlot:
			if not (CONST.OnlySaveHeatmap):
				plt.axvline(
					x=dopplerDomain[dopplerIdxList[CONST.targetIdxPlotSpec] + CONST.targetIdxPlotSpec_doppler_offset],
					color='orange', linestyle=':')
				plt.axhline(y=rangeDomain[rangeIdxList[CONST.targetIdxPlotSpec] + CONST.targetIdxPlotSpec_range_offset],
				            color='orange', linestyle=':')
			plt.scatter(dopplerDomain[dopplerIdxList], rangeDomain[rangeIdxList], facecolors="none", edgecolors='r', s=10)
		plt.xlim(dopplerDomain[0], dopplerDomain[cfgMan.config.n_chirps - 1])
		plt.ylim(rangeDomain[0], rangeDomain[cfgMan.config.numADCSamples - 1])

		plt.show()
		# print(rangeDomain[217])

		print(rangeIdxList, dopplerIdxList)
		print(rangeDomain[rangeIdxList])
		# plt.savefig('{}image/heatmap/heatmap_{}.png'.format(dataPath, frameNumber))
	if CONST.FFTspectrumDEBUG == True:
		plt.figure(figsize=(16, 16))

		for a in range(len(rangeIdxList)):

			if a != CONST.targetIdxPlotSpec:
				continue

			dopplerLine = rangeIdxList[a] + CONST.targetIdxPlotSpec_range_offset
			rangeLine = dopplerIdxList[a] + CONST.targetIdxPlotSpec_doppler_offset
			_, thresholdLine_CA_SO = ml.cfar_CA_SO(np.log2(abs(data.FFTData[rangeIdxList[a]+ CONST.targetIdxPlotSpec_range_offset, :, 0])), cfgMan.cfarCfgDoppler)
			plt.plot(dopplerDomain, np.log2(abs(data.FFTData[rangeIdxList[a], :, 0])), color='black', label='signal')
			plt.axvline(x=domain.dopplerDomain[dopplerIdxList[a]], color='orange', linestyle=":",
			            label='targetIdxPlotSpec')


			plt.plot(dopplerDomain, thresholdLine_CA_SO, color='green', linestyle=':', label=cfgMan.cfarCfgDoppler.cfarType)
			plt.title(
				'dopplerSpectrum threshold dB : ' + str(
					cfgMan.cfarCfgDoppler.thresholdScale * 6) + ' / (Range,Doppler) = (' + str(
					dopplerLine) + ',' + str(rangeLine) + ')')
			plt.legend(loc=1)
			plt.grid(linestyle=':')
			# plt.savefig('{}image/dopplerSpectrum_{}.png'.format(dataPath, frameNumber))
			plt.show()
			# rangeLine = 79

			_, thresholdLine_CA_SO = ml.cfar_CA_SO(heatMap[:, rangeLine], cfgMan.cfarCfgRange)
			_, thresholdLine_OS = ml.cfar_OS(heatMap[:, rangeLine], cfgMan.cfarCfgRange)
			_, thresholdLine_CA_DC = ml.cfar_CA_DC(heatMap[:, rangeLine], cfgMan.cfarCfgRange)
			plt.plot(rangeDomain, heatMap[:, rangeLine], color='black', label='signal')
			plt.axvline(x=domain.rangeDomain[dopplerLine], color='orange', linestyle=":",
			            label='targetIdxPlotSpec')


			plt.plot(rangeDomain, thresholdLine_CA_SO, color='green', linestyle=':', label=cfgMan.cfarCfgRange.cfarType)

			# plt.plot(rangeDomain[:14], thresholdLine_CA_SO[:14]+(4/6),  color = 'red', label = cfarCfgRange.cfarType +"4db")
			# plt.plot(rangeDomain, thresholdLine_OS, linestyle=':', color='red', label = 'thresholdLine_OS')
			# plt.plot(rangeDomain, thresholdLine_CA_DC, linestyle=':', color='blue', label = 'thresholdLine_CA_DC')
			# plt.plot(rangeDomain[:50], thresholdLine_CA_SO[:50]+(2/6),  color = 'orange', label = cfarCfgRange.cfarType +"2db")
			# plt.axvline(x = 32, color = 'red')
			# plt.axvline(x = 20, color = 'red')
			plt.title(
				'rangeSpectrum threshold dB : ' + str(cfgMan.cfarCfgRange.thresholdScale * 6) + ' / (Range,Doppler) = (' + str(
					dopplerLine) + ',' + str(rangeLine) + ')')
			plt.legend(loc=1)
			plt.grid(linestyle=':')

			plt.show()

	if CONST.cfarDEBUG == True:

		for a in range(len(rangeIdxList)):

			if a != CONST.targetIdxPlotSpec:
				continue

			dopplerLine = rangeIdxList[a] + CONST.targetIdxPlotSpec_range_offset

			rangeLine = dopplerIdxList[a] + CONST.targetIdxPlotSpec_doppler_offset

			_, thresholdLine_CA_SO = ml.cfar_CA_SO(heatMap[dopplerLine, :], cfgMan.cfarCfgDoppler)
			plt.plot(dopplerDomain, heatMap[dopplerLine, :], color='black', label='signal')
			plt.axvline(x=domain.dopplerDomain[rangeLine], color='orange', linestyle=":",
			            label='targetIdxPlotSpec')

			# if CONST.slowRangeFFTPlot:
				# plt.plot(domain.dopplerDomain_Slow, heatMap_Slow[dopplerLine, :], color='blue', label='slow signal')
				# plt.axvline(x=domain.dopplerDomain_Slow[CONST.slowRangeFFTPlot_dopplerIdx], color='orange',
				#             linestyle=":", label='slowRangeFFTPlot_dopplerIdx')

			plt.plot(dopplerDomain, thresholdLine_CA_SO, color='green', linestyle=':', label=cfgMan.cfarCfgDoppler.cfarType)
			plt.title(
				'dopplerSpectrum threshold dB : ' + str(
					cfgMan.cfarCfgDoppler.thresholdScale * 6) + ' / (Range,Doppler) = (' + str(
					dopplerLine) + ',' + str(rangeLine) + ')')
			plt.legend(loc=1)
			plt.grid(linestyle=':')
			# plt.savefig('{}image/dopplerSpectrum_{}.png'.format(dataPath, frameNumber))
			plt.show()
			# rangeLine = 79

			_, thresholdLine_CA_SO = ml.cfar_CA_SO(heatMap[:, rangeLine], cfgMan.cfarCfgRange)
			_, thresholdLine_OS = ml.cfar_OS(heatMap[:, rangeLine], cfgMan.cfarCfgRange)
			_, thresholdLine_CA_DC = ml.cfar_CA_DC(heatMap[:, rangeLine], cfgMan.cfarCfgRange)
			plt.plot(rangeDomain, heatMap[:, rangeLine], color='black', label='signal')
			plt.axvline(x=domain.rangeDomain[dopplerLine], color='orange', linestyle=":",
			            label='targetIdxPlotSpec')
			# if CONST.slowRangeFFTPlot:
			# 	# plt.plot(rangeDomain, heatMap_Slow[:, CONST.slowRangeFFTPlot_dopplerIdx], color='blue',
				#          label='slow signal')
			# plt.plot(rangeDomain, np.average(abs(tmpDataCube.FFTData),axis = (1,2)), label = "rms_chirp_antenna")

			plt.plot(rangeDomain, thresholdLine_CA_SO, color='green', linestyle=':', label=cfgMan.cfarCfgRange.cfarType)

			# plt.plot(rangeDomain[:14], thresholdLine_CA_SO[:14]+(4/6),  color = 'red', label = cfarCfgRange.cfarType +"4db")
			# plt.plot(rangeDomain, thresholdLine_OS, linestyle=':', color='red', label = 'thresholdLine_OS')
			# plt.plot(rangeDomain, thresholdLine_CA_DC, linestyle=':', color='blue', label = 'thresholdLine_CA_DC')
			# plt.plot(rangeDomain[:50], thresholdLine_CA_SO[:50]+(2/6),  color = 'orange', label = cfarCfgRange.cfarType +"2db")
			# plt.axvline(x = 32, color = 'red')
			# plt.axvline(x = 20, color = 'red')
			plt.title(
				'rangeSpectrum threshold dB : ' + str(cfgMan.cfarCfgRange.thresholdScale * 6) + ' / (Range,Doppler) = (' + str(
					dopplerLine) + ',' + str(rangeLine) + ')')
			plt.legend(loc=1)
			plt.grid(linestyle=':')
			# plt.savefig('{}image/rangeSpectrum_{}.png'.format(CONST.dataPath, frameNumber))
			plt.show()


	if CONST.enhancedMaxVelDEBUG:
		ml.enhancedMaxVel([cfarOut2DList[CONST.targetIdxPlotSpec]], heatMap_Slow, cfgMan.enhMaxVelCfg, domain, heatMap,
		                  DEBUG=CONST.enhancedMaxVelDEBUG)
	ml.enhancedMaxVel(cfarOut2DList, heatMap_Slow, cfgMan.enhMaxVelCfg, domain, heatMap)
	# print(cfarOut2DList)
	if CONST.CRT_OVERLAP:
		ml.rearrangeByCRT(cfarOut2DList)
	# print(cfarOut2DList)
	cfarOut3DList = ml.AOA(data, config, cfarOut2DList, cfgMan.aoaCfg, domain, procFFT=cfgMan.aoaCfg.procFFT, DEBUG=CONST.AOADebug)
	# print(cfarOut2DList)
	# if CONST.TDM_MIMO:
	# 	if CONST.enhancedMaxVelDEBUG:
	# 		ml.enhancedMaxVel([cfarOut3DList[CONST.targetIdxPlotSpec]], heatMap_Slow, cfgMan.enhMaxVelCfg, domain, heatMap,
	# 		                  DEBUG=CONST.enhancedMaxVelDEBUG)
	# 	ml.enhancedMaxVel(cfarOut3DList, heatMap_Slow, cfgMan.enhMaxVelCfg, domain, heatMap)

	ml.transCoord(cfarOut3DList, domain, resList[1], config.n_chirps, cfgMan.aoaCfg)
	pltd.plotDetObj__x(ml.findIDlistforDEBUG(cfarOut3DList, CONST.targetIdxPlotSpec))
	plt.title("result of the python")
	plt.xlim(-CONST.xPltSize, CONST.xPltSize)
	plt.ylim(0, CONST.yPltSize)
	plt.xticks([i for i in range(-CONST.xPltSize, CONST.xPltSize, CONST.xPltStep)])
	plt.yticks([i for i in range(0, CONST.yPltSize, CONST.yPltStep)])
	plt.grid(linestyle=':')
	plt.show()
if __name__ == '__main__':
	main()