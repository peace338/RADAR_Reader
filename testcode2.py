import sys
import numpy as np
import pyqtgraph.opengl as gl
from PyQt6.QtWidgets import QApplication
from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.graphicsItems.GradientEditorItem import ColorBarItemWidget
from pyqtgraph import ColorMap

class ColorMapBarApp(QtGui.QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Color Map Bar Example")
        self.resize(800, 600)

        layout = QtGui.QVBoxLayout(self)

        # 컬러 맵 생성
        colors = [(0.0, (0, 0, 255, 255)), (1.0, (255, 0, 0, 255))]  # 파란색 -> 빨간색
        cmap = ColorMap(pos=[x[0] for x in colors], color=[x[1] for x in colors])

        # 컬러 맵 바 생성
        colormap_bar = ColorBarItemWidget(cmap.getLookupTable(), label='Z Value', width=20, height=200)
        layout.addWidget(colormap_bar)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    colormap_bar_app = ColorMapBarApp()
    colormap_bar_app.show()
    sys.exit(app.exec())
