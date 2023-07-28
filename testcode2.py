import sys
from PyQt6.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QFrame

class ExampleFrames(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        self.setWindowTitle('Multiple Frames Example')
        main_layout = QVBoxLayout(self)

        # Create the top frame
        top_frame = QFrame(self)
        top_frame.setFrameShape(QFrame.Shape.Box)
        top_frame.setLineWidth(1)
        top_frame.setStyleSheet("background-color: lightblue")
        main_layout.addWidget(top_frame, 1)  # Add the top frame with stretch 1

        # Create the horizontal layout for the left and main frames
        left_main_layout = QHBoxLayout()

        # Create the left frame
        left_frame = QFrame(self)
        left_frame.setFrameShape(QFrame.Shape.Box)
        left_frame.setLineWidth(1)
        left_frame.setStyleSheet("background-color: lightgreen")
        left_main_layout.addWidget(left_frame, 1)  # Add the left frame with stretch 1

        # Create the main frame and its layout
        main_frame = QFrame(self)
        main_frame.setFrameShape(QFrame.Shape.Box)
        main_frame.setLineWidth(2)
        main_frame.setStyleSheet("background-color: lightpink")

        # Create layout for the main frame
        main_frame_layout = QVBoxLayout(main_frame)
        main_frame_layout.addWidget(QWidget())  # Add some example widget to main_frame layout

        left_main_layout.addWidget(main_frame, 3)  # Add the main frame with stretch 3

        main_layout.addLayout(left_main_layout)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = ExampleFrames()
    window.setGeometry(100, 100, 800, 600)
    window.show()
    sys.exit(app.exec())
