
import pyqtgraph as pg
import numpy as np
from pyqtgraph.Qt import QtGui, QtCore


def drawGridPolarCoord(pyqtGraph):
    radius = np.linspace(0, 1, 5)  # 반지름 값
    theta = np.linspace(0, 2 * np.pi, 360)  # 각도 값

    # 극 좌표계 그리드 그리기
    for r in radius:
        x = r * np.cos(theta)
        y = r * np.sin(theta)
        # pyqtGraph.plot(x, y, pen='b', antialias=True)


def drawGridPolarCoord(pyqtGraph, theta):


    # 극 좌표계 그리드 그리기
    # for r in radius:
    #     x = r * np.cos(theta)
    #     y = r * np.sin(theta)
    pyqtGraph.plot([0,15], [0,15 * np.tan((90 - theta) * np.pi /180)], pen ='w')
    pyqtGraph.plot([0,-15], [0,15 * np.tan((90 - theta) * np.pi /180)], pen ='w')

def getAzim(sinAzim):
    if sinAzim > 1.0:
        ret = 1.0
    elif sinAzim < -1.0:
        ret = -1.0
    else:
        ret = sinAzim
    return ret