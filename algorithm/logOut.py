import os
import CONST
import csv
import numpy as np


# print(os.path.isdir("{}image/log".format(CONST.dataPath)))
if not (os.path.isdir("{}image/log".format(CONST.dataPath))):
	os.makedirs(os.path.join("{}image/log".format(CONST.dataPath)))
if not (os.path.isdir("{}image_readTargetRaw/log".format(CONST.dataPath))):
	os.makedirs(os.path.join("{}image_readTargetRaw/log".format(CONST.dataPath)))


class log:
	def __init__(self, type):

		if type == "TRACK_DCA1000":
			self.logTXT = open(CONST.dataPath + "image_readTargetRaw/log/logTXT.csv", 'w', newline="")
			self.logDtype = [("frameNumber", int), ("ID", int), ("x_tracking", float), ("y_tracking", float),
			                 ("xd_tracking", float), ("yd_tracking", float), \
			                 ("xSize", float), ("ySize", float), ("tick", int), ("age", int), ("FLAG", int),
			                 ("reserved", int)]
		elif type == "GUARDRAIL":
			self.logTXT = open(CONST.dataPath + "image/log/log_guardRail.csv", 'w', newline="")
			self.logDtype = [("frameNumber", int), ("h", float), ("k", float), ("ySize/xSize", float), ("theta(DEG)", float), ("distance", float)]

		elif type == "TRACK_PYTHON":
			self.logTXT = open(CONST.dataPath + "image/log/logTXT_python.csv", 'w', newline="")
			self.logDtype = [("frameNumber", float), ("ID", int), ("x_tracking", float), ("y_tracking", float),
			            ("xd_tracking", float), ("yd_tracking", float), \
			            ("xSize", float), ("ySize", float), ("xSize_det", float), ("ySize_det", float), \
			            ("range_det", float), ("doppler_det", float), \
			            ("theta_sin_det", float), ("x_det", float), ("theta_DEG", float), \
			            ("rangeSNR", float), ("dopplerSNR", float), ("angleSNR", float), ("peakVal", float), \
			            ("mu0", float), ("mu1", float), ("reserved", float), ("mahala_distm0", float),
			            ("mahala_distm1", float), ("innovation0", float), ("innovation1", float),
			            ("innovation2", float),
			            ("likelihood0", float), ("likelihood1", float),
			            ("CV_x_trk", float), ("CV_y_trk", float),
			            ("CV_xd_trk", float), ("CV_yd_trk", float),
			            ("CA_x_trk", float), ("CA_y_trk", float),
			            ("CA_xd_trk", float), ("CA_yd_trk", float),
			            ("CA_xdd_trk", float), ("CA_ydd_trk", float),
			            ("xdd_final", float), ("ydd_final", float),
			            ("R00", float), ("R11", float), ("R22", float), ("test", float), ("isSlow", float),
			            ("TransitionScore", float),
			            ("statusFlag_trk", int), ("statusFlag_det", int), ("tick", int), ("age", int)]
		self.logList = []

	def log_write(self):
		logNp = np.array(self.logList, dtype=self.logDtype)
		# print(logNp)
		csvWriter = csv.writer(self.logTXT)
		# ml.logTXT.write("%f"%logNp[:]["x"])
		# print(self.logDtype[:])
		csvWriter.writerow(self.logDtype[:])
		for row in logNp:
			csvWriter.writerow(row)

		self.logTXT.close()

	def appendLog(self,listDtype):
		self.logList.append(listDtype)

logObj = log("TRACK_DCA1000")
logObj_python = log("TRACK_PYTHON")
logObj_guardRail = log("GUARDRAIL")