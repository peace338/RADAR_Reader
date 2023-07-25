
import pyqtgraph as pg
import numpy as np
from pyqtgraph.Qt import QtGui, QtCore
from PyQt6.QtCore import *
GRAPH_MIN_X = -90          # (-20)
GRAPH_MAX_X = 90           # (20)
GRAPH_MIN_Y = -4.5
GRAPH_MAX_Y = 4.5

YZ_GRAPH_MIN_X = 0          # (-20)
YZ_GRAPH_MAX_X = 10           # (20)
YZ_GRAPH_MIN_Y = -4.5
YZ_GRAPH_MAX_Y = 4.5

def drawGridPolarCoord(pyqtGraph):
    radius = np.linspace(0, 1, 5)  # 반지름 값
    theta = np.linspace(0, 2 * np.pi, 360)  # 각도 값

    # 극 좌표계 그리드 그리기
    for r in radius:
        x = r * np.cos(theta)
        y = r * np.sin(theta)
        # pyqtGraph.plot(x, y, pen='b', antialias=True)


def drawFOV_XY(pyqtGraph, theta):


    # 극 좌표계 그리드 그리기
    # for r in radius:
    #     x = r * np.cos(theta)
    #     y = r * np.sin(theta)
    pyqtGraph.plot([0,15], [0,15 * np.tan((90 - theta) * np.pi /180)], pen ='w')
    pyqtGraph.plot([0,-15], [0,15 * np.tan((90 - theta) * np.pi /180)], pen ='w')

def drawFOV_YZ(pyqtGraph, theta):


    # 극 좌표계 그리드 그리기
    # for r in radius:
    #     x = r * np.cos(theta)
    #     y = r * np.sin(theta)
    pyqtGraph.plot([0,15], [0,15 * np.tan((theta) * np.pi /180)], pen ='w')
    pyqtGraph.plot([0,15], [0,-15 * np.tan((theta) * np.pi /180)], pen ='w')

def getAzim(sinAzim):
    if sinAzim > 1.0:
        ret = 1.0
    elif sinAzim < -1.0:
        ret = -1.0
    else:
        ret = sinAzim
    return ret

def setPlot(pgplot, format = "ThetaDoppler"):
    pgplot.showGrid(True, True, 0.5)
    
    if format == "ThetaDoppler":
        pgplot.setLabel('left', 'speed(m/s)')
        pgplot.setLabel('bottom', 'angle(Degree)')
        pgplot.setRange(QRectF(GRAPH_MIN_X, GRAPH_MIN_Y, GRAPH_MAX_X-GRAPH_MIN_X, GRAPH_MAX_Y-GRAPH_MIN_Y),disableAutoRange = True)
    elif format == "YZ":
        pgplot.setLabel('left', 'z(m)')
        pgplot.setLabel('bottom', 'y(m)')
        pgplot.setRange(QRectF(YZ_GRAPH_MIN_X, YZ_GRAPH_MIN_Y, YZ_GRAPH_MAX_X-YZ_GRAPH_MIN_X, YZ_GRAPH_MAX_Y-YZ_GRAPH_MIN_Y),disableAutoRange = True)

