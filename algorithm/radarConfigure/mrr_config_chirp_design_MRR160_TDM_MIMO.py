numRx = 4
numTx = 3

SampleRate = 5209e3  # unit SPS(Sample per seconds) / PROFILE_MRR_DIGOUT_SAMPLERATE_VAL *e3     (5447e3)
numADCSamples = 256  # unit None / PROFILE_MRR_ADC_SAMPLE_VAL                                   (512)

numADCBits = 16
numLanes = 2
isReal = 0

n_chirps = 64  # half of number of chirp, subframe0 = fastchirp, subfrmae1 = slowchirp. / CHIRP_MRR_0_END_INDEX + 1     (32)
slope_Hz_per_sec = 70.0e12  # Unit Hz per Sec / PROFILE_MRR_FREQ_SLOPE_MHZ_PER_US * e12        (14.64e12) (7.32e12)

period_chirp = numTx*101.14e-6  # Unit sec /계산해야하네... / ((CHIRP_MRR_0_IDLE_TIME_VAL + PROFILE_MRR_IDLE_TIME_VAL + PROFILE_MRR_RAMP_END_TIME_VAL)/100.0f) : 65 usec.  (116.8e-6)
period_chirp_slow = numTx*115.7e-6  # unit sec / (CHIRP_MRR_1_IDLE_TIME_VAL + PROFILE_MRR_IDLE_TIME_VAL + PROFILE_MRR_RAMP_END_TIME_VAL)/100.0f         (126.8e-6)

carrierFreq = 77.0e9  # Unit Hz /PROFILE_MRR_START_FREQ_GHZ *e9
# carrierFreq = 76.01e9  # Unit Hz /PROFILE_MRR_START_FREQ_GHZ *e9

td = 0.06666666 #unit sec, 15frame

numOfBurst = 1
numOfSubFrame = 1