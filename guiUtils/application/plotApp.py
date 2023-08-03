from PyQt6.QtWidgets import *                       # python -m pip install pyqt6
from PyQt6.QtCore import *
from PyQt6.QtGui import *
from PyQt6.QtMultimedia import *
from PyQt6.QtMultimediaWidgets import *
from pyqtgraph import Vector
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.pyplot as plt

import pyqtgraph as pg   
from ..guiConfigure import *
import numpy as np

import pyqtgraph.opengl as gl
from matplotlib import cm
import pdb

class CuboidItem(gl.GLLinePlotItem):
    def __init__(self, start_point, end_point, color=(1, 0, 1, 1), width=1, minZ = None):

        if minZ is not None:
            # pdb.set_trace()
            if start_point[2] < minZ:
                start_point[2] = minZ
        
        if start_point[1] < GRAPH_MIN_Y:
            start_point[1] = GRAPH_MIN_Y
        
        

        # start_point = tuple(start_point)

        vertices = [
            start_point,                                    #0
            [start_point[0], end_point[1], start_point[2]], #1
            [start_point[0], end_point[1], end_point[2]],   #2
            [start_point[0], start_point[1], end_point[2]], #3
            start_point,                                    #4
            [end_point[0], start_point[1], start_point[2]], #5
            [end_point[0], end_point[1], start_point[2]],   #6
            [end_point[0], end_point[1], end_point[2]], #7
            [end_point[0], start_point[1], end_point[2]], #8
            [end_point[0], start_point[1], start_point[2]],   #9 = #5
            [end_point[0], start_point[1], end_point[2]], #10 = 8
            [start_point[0], start_point[1], end_point[2]], #3
            [start_point[0], end_point[1], end_point[2]],   #2
            [end_point[0], end_point[1], end_point[2]], #7
            [end_point[0], end_point[1], start_point[2]],   #6
            [start_point[0], end_point[1], start_point[2]], #1


        ]

        super().__init__(pos=np.array(vertices), color=color, width=width, antialias=True)

class GroundRectItem(gl.GLLinePlotItem):
    def __init__(self, start_point, end_point, color=(1, 0, 1, 1), width=1, minZ = None):

        if minZ is not None:
            start_point[2] = minZ
        # start_point = tuple(start_point)
        
        if start_point[1] < GRAPH_MIN_Y:
            start_point[1] = GRAPH_MIN_Y

        vertices = [
            start_point,                                    #0
            [start_point[0], end_point[1], start_point[2]], #1
            [end_point[0], end_point[1], start_point[2]],   #2
            [end_point[0], start_point[1], start_point[2]], #3
            start_point,                                    #4
        ]

        super().__init__(pos=np.array(vertices), color=color, width=width, antialias=True)

class Scatter3DPlot(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):

        self.scatter = -1
        self.cuboidWritenFlag = -1
        layout = QVBoxLayout(self)
        self.plot_widget = gl.GLViewWidget()

        assert SCATTER_FRAME > 0
        self.scatterFrame = SCATTER_FRAME 
        self.pingpongIdx = 0
        self.sp = []
        for i in range(self.scatterFrame):
            tmp = gl.GLScatterPlotItem(pos=np.array([[0, 0, 0]]), size=10, color=(1, 0, 0, 1))
            tmp.setGLOptions('additive')   
            self.sp.append(tmp)
        # pdb.set_trace()
        for sp in self.sp:
            self.plot_widget.addItem(sp)
        
        x = [ROI_MIN_X, ROI_MAX_X, ROI_MAX_X, ROI_MIN_X, ROI_MIN_X]
        y = [ROI_MIN_Y, ROI_MIN_Y, ROI_MAX_Y, ROI_MAX_Y, ROI_MIN_Y]
        z = [-EQUIP_HEIGHT , -EQUIP_HEIGHT, -EQUIP_HEIGHT, -EQUIP_HEIGHT, -EQUIP_HEIGHT]

        ROI_item = gl.GLLinePlotItem()
        self.plot_widget.addItem(ROI_item)
        ROI_item.setData(pos=np.column_stack((x, y, z)), color=(255, 255, 255, 10), width=1)

        x = [GRAPH_MIN_X, GRAPH_MAX_X, GRAPH_MAX_X, GRAPH_MIN_X, GRAPH_MIN_X]
        y = [GRAPH_MIN_Y, GRAPH_MIN_Y, GRAPH_MAX_Y, GRAPH_MAX_Y, GRAPH_MIN_Y]
        z = [-EQUIP_HEIGHT , -EQUIP_HEIGHT, -EQUIP_HEIGHT, -EQUIP_HEIGHT, -EQUIP_HEIGHT]
        ROI_item = gl.GLLinePlotItem()
        self.plot_widget.addItem(ROI_item)
        ROI_item.setData(pos=np.column_stack((x, y, z)), color=(255, 255, 255, 10), width=1)

        x = [GRAPH_MAX_X, GRAPH_MAX_X, GRAPH_MIN_X, GRAPH_MIN_X]
        y = [GRAPH_MAX_Y, GRAPH_MAX_Y, GRAPH_MAX_Y, GRAPH_MAX_Y]
        z = [5-EQUIP_HEIGHT, -EQUIP_HEIGHT, -EQUIP_HEIGHT, 5-EQUIP_HEIGHT]
        ROI_item = gl.GLLinePlotItem()
        self.plot_widget.addItem(ROI_item)
        ROI_item.setData(pos=np.column_stack((x, y, z)), color=(255, 255, 255, 10), width=1)

        fov_angle = 130
        fov_distance = 10  # FOV distance from the radar position
        fov_rad = np.radians(fov_angle)
        x_fov = [fov_distance * np.sin(-fov_rad / 2), 0, fov_distance * np.sin(fov_rad / 2)]
        y_fov = [fov_distance * np.cos(-fov_rad / 2), 0, fov_distance * np.cos(fov_rad / 2)]
        z_fov = [0 ,0, 0]
        
        
                              
        self.fov_item = gl.GLLinePlotItem()
        self.plot_widget.addItem(self.fov_item)
        self.fov_item.setData(pos=np.column_stack((x_fov, y_fov, z_fov)), color=(255, 255, 255, 10), width=1)
        # self.fov_item.setData(pos=np.column_stack((x_fov_left, y_fov_left, z_fov_left)), color=(255, 255, 255, 125), width=1)
        grid_x = gl.GLGridItem()
        grid_x.setSize(x=20, y=10, z=4)
        grid_x.setSpacing(x=1, y=1, z=1)
        equipHeihgt = EQUIP_HEIGHT
        grid_x.translate(0,5,-equipHeihgt)
        self.plot_widget.addItem(grid_x)

        grid_y = gl.GLGridItem()
        grid_y.setSize(x=5, y=10, z=4)
        grid_y.rotate(90,0,1,0)
        grid_y.setSpacing(x=1, y=1, z=1)
        equipHeihgt = EQUIP_HEIGHT
        grid_y.translate(10,5,2.5 - equipHeihgt)
        self.plot_widget.addItem(grid_y)

        grid_y2 = gl.GLGridItem()
        grid_y2.setSize(x=5, y=10, z=4)
        grid_y2.rotate(90,0,1,0)
        grid_y2.setSpacing(x=1, y=1, z=1)

        grid_y2.translate(-10,5,2.5 - equipHeihgt)
        self.plot_widget.addItem(grid_y2)

        grid_z = gl.GLGridItem()
        grid_z.setSize(x=20, y=5, z=4)
        grid_z.rotate(90,1,0,0)
        grid_z.setSpacing(x=1, y=1, z=1)

        grid_z.translate(0,10,2.5 - equipHeihgt)
        self.plot_widget.addItem(grid_z)

        self.plot_widget.setCameraPosition(pos = Vector(0,5,0), distance=15, elevation=20, azimuth=270)
        
        layout.addWidget(self.plot_widget)
        
        self.colormap = cm.get_cmap('jet') # 'plasma'
        
        # bar = pg.ColorBarItem( values= (0, 10), cmap=self.colormap )
        # layout.addWd(bar)
    def writePoint(self, objs):
        
        z_normalized = (GRAPH_MAX_Y - objs[:,3]) / (GRAPH_MAX_Y - GRAPH_MIN_Y)
        colors = self.colormap(z_normalized)

        # z_normalized = (GRAPH_MAX_Z - objs[:,2]) / (GRAPH_MAX_Z - GRAPH_MIN_Z)
        # colors = self.colormap(z_normalized)

        colors[:,3] = 1 #alpha
        colors[np.where(objs[:,4] == 1)] = [0,0,1,1]
        # colors[np.where(objs[:,4] == -1)] = [0,1,0,1]
        sizes = np.ones(len(objs))*13
        sizes[np.where(objs[:,4] == 1)] = 10
        # pdb.set_trace()
        # print(colors)
        self.sp[self.pingpongIdx].setData(pos=np.column_stack((objs[:,0], objs[:,1], objs[:,2])), color = colors, size = sizes)
        
        self.scatterWrittenFlag = 1

        self.pingpongIdx += 1
        self.pingpongIdx = self.pingpongIdx % self.scatterFrame

    def writeCuboid(self, objs):
        self.removeCuboid()
        for obj in objs:
            item = CuboidItem((obj[:3] - obj[3:]/2), (obj[:3] + obj[3:]/2), color = (1,0,1,1), width = 1, minZ = -EQUIP_HEIGHT)
            self.cuboidList.append(item)
            self.plot_widget.addItem(item)
            
            item = GroundRectItem((obj[:3] - obj[3:]/2), (obj[:3] + obj[3:]/2), color = (1,0,1,0.8), width = 1, minZ = -EQUIP_HEIGHT)
            self.cuboidList.append(item)
            self.plot_widget.addItem(item)
        self.cuboidWritenFlag = 1
    
    def removeCuboid(self):
        if self.cuboidWritenFlag != 1:
            self.cuboidList = []
            return
        for item in self.cuboidList:
            # print('remove')
            self.plot_widget.removeItem(item)
        self.cuboidList = []
        
class ScatterThetaDoppler(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):

        self.scatter = -1
        self.cuboidWritenFlag = -1
        layout = QVBoxLayout(self)
        self.develop_plot1 = pg.plot()
        # self.sp = gl.GLScatterPlotItem(pos=np.array([[0, 0]]), size=10, color=(1, 0, 0, 0.75)) 
        self.develop_plot1.showGrid(True, True, 0.5)
        self.develop_plot1.setLabel('left', 'speed(m/s)')
        self.develop_plot1.setLabel('bottom', 'angle(Degree)')
        self.develop_plot1.setRange(QRectF(EGO_GRAPH_MIN_X, EGO_GRAPH_MIN_Y, EGO_GRAPH_MAX_X-EGO_GRAPH_MIN_X, EGO_GRAPH_MAX_Y-EGO_GRAPH_MIN_Y), disableAutoRange = True)
        self.scatter = pg.ScatterPlotItem(size=1, brush=pg.mkBrush(255, 255, 255, 255))
        self.develop_plot1.addItem(self.scatter)
        layout.addWidget(self.develop_plot1)

        self.colormap = cm.get_cmap('jet')
        self.curve = None
    def writePoint(self, objs, flags):
        self.scatter.clear()
        obj_spots = []
        for obj, flag in zip(objs, flags) :
            if flag == 1:
                # print(obj.sin_azim)
                obj_spots.append({'pos':[obj[0], obj[1]], 'size' : 5, 'pen' : (0, 0, 0, 0), 'brush' : (125,125,255,255), 'data': 1})
            else:
                obj_spots.append({'pos':[obj[0], obj[1]], 'size' : 5, 'pen' : (0, 0, 0, 0), 'brush' : (0,255,0,255), 'data': 1})
        
        self.scatterWrittenFlag = 1
        self.scatter.addPoints(obj_spots, brush=(255,0,0,255))

    def writeCurve(self, x,y):
        if self.curve is not None:
            self.removeCurve()
        self.curve = self.develop_plot1.plot(x, y)

    def removeCurve(self):
        self.develop_plot1.removeItem(self.curve)