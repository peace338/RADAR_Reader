from PyQt6.QtWidgets import *                       # python -m pip install pyqt6
from PyQt6.QtCore import *
from PyQt6.QtGui import *
from PyQt6.QtMultimedia import *
from PyQt6.QtMultimediaWidgets import *               # python -m pip install pyqtgraph

import sys
import guiUtils

def main():
    app = QApplication(sys.argv)
    my_window = guiUtils.application.mainApp.App()
    my_window.show()
    app.exec()


if __name__ == '__main__':
    
    main()

