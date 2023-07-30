from PyQt6.QtWidgets import *                       # python -m pip install pyqt6
from PyQt6.QtCore import *
from PyQt6.QtGui import *
from PyQt6.QtMultimedia import *
from PyQt6.QtMultimediaWidgets import *

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.pyplot as plt

from ..guiConfigure import *
import numpy as np

import pdb
class Scatter3DPlot(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        plt.style.use('dark_background')
        self.scatter = -1
        layout = QVBoxLayout(self)

        # Create the 3D scatter plot
        self.fig = plt.figure()
        self.canvas = FigureCanvas(self.fig)
        layout.addWidget(self.canvas)
        
        self.ax = self.fig.add_subplot(111, projection='3d')
        # pdb.set_trace()
        # self.ax.set_xscale(5)
        # self.ax.set_aspect('equal')
        self.ax.set_box_aspect((GRAPH_MAX_X - GRAPH_MIN_X, GRAPH_MAX_Y, GRAPH_MAX_Z))
        # self.ax.auto_scale_xyz([GRAPH_MIN_X, GRAPH_MAX_X], [GRAPH_MIN_Y, GRAPH_MAX_Y], [0, 4])
        self.ax.view_init(azim=-80, elev=20)
        self.ax.disable_mouse_rotation()
        # Plot the scatter points
        self.fig.subplots_adjust(left=-0.15, right=1.15, bottom=-0.15, top=1.15)

        # Set labels for the axes
        self.ax.set_xlabel('X(m)')
        self.ax.set_ylabel('Y(m)')
        self.ax.set_zlabel('Z(m)')

        self.ax.set_xlim3d(GRAPH_MIN_X, GRAPH_MAX_X)
        self.ax.set_ylim3d(0,GRAPH_MAX_Y)
        self.ax.set_zlim3d(0,GRAPH_MAX_Z)

        self.ax.set_xticks(np.arange(GRAPH_MIN_X, GRAPH_MAX_X, 1))
        self.ax.set_yticks(np.arange(GRAPH_MIN_Y, GRAPH_MAX_Y, 1))
        self.ax.set_zticks(np.arange(GRAPH_MIN_Z, GRAPH_MAX_Z, 1))

        # scalarmappaple = plt.cm.ScalarMappable(cmap='jet')
        # scalarmappaple.set_array(range(0,11))
        # plt.colorbar(scalarmappaple, ax=self.ax, label='Range(m)', location =  'left', shrink=0.4)
        # self.ax.colorbar(label='Z Value')
       
        # Show the plot
        # pdb.set_trace()
        # self.writePlot()

    def writePlot(self, objs):
        if self.scatter != -1:
            self.clearPlot()
        cmap_custom = plt.get_cmap('jet')
        # cmap_custom.set_under(color = 'k', alpha = None)  # Color for values less than vmin
        # colors = np.where(objs[:,4] == -1, objs[:,3], -1)
        # pdb.set_trace()
        colors = np.where(objs[:,4] == -1, 'greenyellow', 'blue')
        # print(colors)
        # pdb.set_trace()
        sizes = np.where(objs[:,4]==-1, 100, 20)
        # print(colors)
        # x = np.random.rand(5)
        # y = np.random.rand(5)
        # z = np.random.rand(5)
        # pdb.set_trace()
        self.scatter = self.ax.scatter(objs[:,0], objs[:,1], objs[:,2], c= colors ,s=sizes, alpha=0.75)

        # self.scatter = self.ax.scatter(objs[:,0], objs[:,1], objs[:,2], c= colors, cmap = cmap_custom ,s=sizes, alpha=0.75)
        self.canvas.draw()
        
        self.scatterWrittenFlag = 1
    def clearPlot(self):
        self.scatter.remove()
        # self.canvas.draw()     
