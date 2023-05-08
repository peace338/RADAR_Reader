import configManager_MRR_DEMO as cfg
import numpy as np
import math
import matplotlib.pyplot as plt
import dataStructure as dT
import copy
INPUT_NOISE = np.diag([1.0, np.deg2rad(30.0)]) ** 2
GPS_NOISE = np.diag([0.5, 0.5]) ** 2

RADAR_NOISE = np.diag([0.1,0.1,0.02]) ** 2
# RADAR_NOISE = np.diag([0.0,0.0,0.0]) ** 2
DT = cfg.trackingCfg.td
SIM_TIME = 50.0
show_animation = True
addNoise        = False
addResolution   = False

np.random.seed(0)

def convCoord(xTrue, resList):
	z_rrd = np.zeros((3,1), dtype = np.float)

	z_rrd[0][0] = np.sqrt(xTrue[0]**2 + xTrue[1]**2)
	z_rrd[2][0] = xTrue[0]/z_rrd[0][0]
	z_rrd[1][0] = xTrue[3] * (  np.sin(xTrue[2])*np.sqrt(1 - z_rrd[2][0]**2) + np.cos(xTrue[2])*z_rrd[2][0])

	if addNoise:
		z_rrd = z_rrd + RADAR_NOISE @ np.random.randn(3, 1)
	if addResolution:
		z_rrd[0][0] = round(z_rrd[0][0]/resList[0]) * resList[0]
		z_rrd[1][0] = round(z_rrd[1][0] / resList[1]) * resList[1]

	# print(z_rrd)
	return z_rrd

def invConvCoord(z_rrd):
	z = np.zeros((2,1), dtype = np.float)
	# print(z_rrd)
	z[0][0] = z_rrd[0][0] * z_rrd[2][0]
	z[1][0] = z_rrd[0][0] * np.sqrt(1 - z_rrd[2][0]**2)



	return z
def make_cfarOutFmt3D(z_rrd):
	tmpList = []
	fmt3D = dT.cfarOutFmt3D()


	fmt3D.rangeSNR = 30
	fmt3D.dopplerSNR = 30
	fmt3D.angleSNR = 15
	fmt3D.rangeVal = 20
	fmt3D.dopplerVal = 20
	fmt3D.angleVal = 15
	fmt3D.isSlow = False
	fmt3D.range = z_rrd[0][0]
	fmt3D.speed = z_rrd[1][0]
	fmt3D.sinAzim = z_rrd[2][0]

	fmt3D.x = fmt3D.range * fmt3D.sinAzim
	fmt3D.y = np.sqrt(fmt3D.range * fmt3D.range - fmt3D.x * fmt3D.x)
	if fmt3D.y < 0:
		fmt3D.y = 0
	fmt3D.xd = 0
	fmt3D.yd = fmt3D.speed
	tmpList.append(fmt3D)

	return tmpList



def observation(xTrue, u, resList):
	xTrue = motion_model(xTrue, u)

	# add noise to gps x-y
	# z = observation_model(xTrue) + GPS_NOISE @ np.random.randn(2, 1)
	# print("orig",z)
	# print(RADAR_NOISE @ np.random.randn(3, 1))

	z_rrd = convCoord(xTrue, resList)
	
	# print(z_rrd)
	z = invConvCoord(z_rrd)
	# print("mod",z)
	return xTrue, z, z_rrd

def motion_model(x, u):
	F = np.array([[1.0, 0, 0, 0],
              [0, 1.0, 0, 0],
              [0, 0, 1.0, 0],
              [0, 0, 0, 0]])

	B = np.array([[DT * math.cos(x[2, 0]), 0],
              [DT * math.sin(x[2, 0]), 0],
              [0.0, DT],
              [1.0, 0.0]])

	x = F @ x + B @ u

	return x

def observation_model(x):
	H = np.array([
	[1, 0, 0, 0],
	[0, 1, 0, 0]
	])

	z = H @ x

	return z
def calc_input(v, yawrate):
	# v = 1.0  # [m/s]
	# yawrate = 0.0  # [rad/s]
	u = np.array([[v], [yawrate]])
	return u
def main():
	print(__file__ + " start!!")

	time = 0.0

	# State Vector [x y yaw v]'
	xEst = np.zeros((4, 1))
	xTrue = np.zeros((4, 1))

	#initial state
	xTrue[0][0] = 3 #x
	xTrue[1][0] = 3 # y
	xTrue[2][0] = np.pi/4 # pi, 진행방향
	xTrue[3][0] = 0 # velocity


	xEst = copy.deepcopy(xTrue)
	# history
	hxEst = xEst
	hxTrue = xTrue
	hz = np.zeros((2, 1))

	while SIM_TIME >= time:
		time += DT
		if time <10:
			v = 1
			w = 0
		else:
			v= 1
			w =1
		u = calc_input(v, w)

		xTrue, z, z_rrd = observation(xTrue, u)
		cfarOut3DList = make_cfarOutFmt3D(z_rrd)
		hxEst = np.hstack((hxEst, xEst))
		hxTrue = np.hstack((hxTrue, xTrue))

		hz = np.hstack((hz, z))

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
			# plot_covariance_ellipse(xEst, PEst)
			plt.axis("equal")
			plt.grid(True)
			plt.pause(0.001)

if __name__ == '__main__':
    main()