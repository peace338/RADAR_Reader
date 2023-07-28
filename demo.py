from asyncio.windows_events import NULL
from lzma import is_check_supported
from PyQt6.QtWidgets import *                       # python -m pip install pyqt6
from PyQt6.QtCore import *
from PyQt6.QtGui import *
from PyQt6.QtMultimedia import *
from PyQt6.QtMultimediaWidgets import *
import simulation_algorithm_main as SAM
import dataStructure as DT
import pyqtgraph as pg                              # python -m pip install pyqtgraph
import os
import sys
import numpy as np                                  # python -m pip install numpy
import cv2                                          # python -m pip install opencv-python
import pickle                                       # python -m pip install pickle
import importlib
import pdb
import math
from anomalyDetection import *
# from guiUtils.drawHelper import *
import guiUtils

import matplotlib.pyplot as plt

from PyQt6.QtWidgets import * 

def main():
    app = QApplication(sys.argv)
    my_window = guiUtils.application.mainApp.App()
    my_window.show()
    app.exec()


if __name__ == '__main__':
    
    main()

