from PyQt6.QtWidgets import *                       # python -m pip install pyqt6
from PyQt6.QtCore import *
from PyQt6.QtGui import *
from PyQt6.QtMultimedia import *
from PyQt6.QtMultimediaWidgets import *

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.pyplot as plt

from ..guiConfigure import *

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
        self.ax.set_box_aspect((20, 10, 4))
        self.ax.auto_scale_xyz([GRAPH_MIN_X, GRAPH_MAX_X], [GRAPH_MIN_Y, GRAPH_MAX_Y], [0, 4])
        self.ax.view_init(azim=-90, elev=20)
        # Plot the scatter points
        self.fig.subplots_adjust(left=-0.15, right=1.15, bottom=-0.15, top=1.15)

        # Set labels for the axes
        self.ax.set_xlabel('X(m)')
        self.ax.set_ylabel('Y(m)')
        self.ax.set_zlabel('Z(m)')

        # self.ax.set_xlim3d(-5,5)
        # self.ax.set_ylim3d(0,12)
        # self.ax.set_zlim3d(-2,2)

        # Show the plot
        # pdb.set_trace()
        # self.writePlot()

    def writePlot(self, objs):
        if self.scatter != -1:
            self.clearPlot()

        # x = np.random.rand(5)
        # y = np.random.rand(5)
        # z = np.random.rand(5)
        self.scatter = self.ax.scatter(objs[:,0], objs[:,1], objs[:,2], c='b', s=50)
        self.canvas.draw()
        self.scatterWrittenFlag = 1
    def clearPlot(self):
        self.scatter.remove()
        # self.canvas.draw()     
