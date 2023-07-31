from PyQt6.QtWidgets import *                       # python -m pip install pyqt6
from PyQt6.QtCore import *
from PyQt6.QtGui import *
from PyQt6.QtMultimedia import *
from PyQt6.QtMultimediaWidgets import *
from pyqtgraph import Vector
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.pyplot as plt

from ..guiConfigure import *
import numpy as np

import pyqtgraph.opengl as gl
from matplotlib import cm
import pdb
class Scatter3DPlot(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        plt.style.use('dark_background')
        self.scatter = -1
        layout = QVBoxLayout(self)
        self.plot_widget = gl.GLViewWidget()
        self.sp = gl.GLScatterPlotItem(pos=np.array([[0, 0, 0]]), size=10, color=(1, 0, 0, 0.75))        
        self.plot_widget.addItem(self.sp)
        
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

        self.plot_widget.setCameraPosition(pos = Vector(0,5,0), distance=15, elevation=20, azimuth=270)
        
        layout.addWidget(self.plot_widget)
        
        self.colormap = cm.get_cmap('viridis')

    def writePoint(self, objs):
        z_normalized = (objs[:,2] - GRAPH_MIN_Z) / (GRAPH_MAX_Z - GRAPH_MIN_Z)
        colors = self.colormap(z_normalized)

        colors[:,3] = 1 #alpha
        colors[np.where(objs[:,4] == 1)] = [0,0,1,1]
        colors[np.where(objs[:,4] == -1)] = [0,1,0,1]
        sizes = np.ones(len(objs))*10
        sizes[np.where(objs[:,4] == 1)] = 10
        self.sp.setData(pos=np.column_stack((objs[:,0], objs[:,1], objs[:,2])), color = colors, size = sizes)
        
        self.scatterWrittenFlag = 1
    def clearPlot(self):
        self.scatter.remove()
        # self.canvas.draw()     
