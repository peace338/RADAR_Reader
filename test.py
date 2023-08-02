import sys
from PyQt6.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QPushButton
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.pyplot as plt
import numpy as np

class ExampleProgram(QMainWindow):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        self.setWindowTitle('Example Program')
        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)

        self.canvas_layout = QVBoxLayout()

        self.create_canvas('Bar Graph')
        self.create_canvas('Pie Chart')
        self.create_canvas('Line Plot')
        self.create_canvas('Scatter Plot')
        self.create_canvas('Histogram')

        layout.addLayout(self.canvas_layout)

    def create_canvas(self, graph_title):
        fig = plt.figure()
        canvas = FigureCanvas(fig)
        self.canvas_layout.addWidget(canvas)

        button_layout = QVBoxLayout()

        button = QPushButton(graph_title, self)
        button.clicked.connect(lambda _, title=graph_title: self.show_graph(title, fig, canvas))
        button_layout.addWidget(button)

        self.canvas_layout.addLayout(button_layout)

    def show_graph(self, graph_title, fig, canvas):
        plt.figure(fig.number)
        plt.clf()

        if graph_title == 'Bar Graph':
            self.plot_bar_graph()
        elif graph_title == 'Pie Chart':
            self.plot_pie_chart()
        elif graph_title == 'Line Plot':
            self.plot_line_plot()
        elif graph_title == 'Scatter Plot':
            self.plot_scatter_plot()
        elif graph_title == 'Histogram':
            self.plot_histogram()

        canvas.draw()

    def plot_bar_graph(self):
        data = {'Apples': 30, 'Bananas': 40, 'Grapes': 25, 'Oranges': 20}
        labels = list(data.keys())
        values = list(data.values())
        plt.bar(labels, values)
        plt.xlabel('Fruits')
        plt.ylabel('Quantity')
        plt.title('Bar Graph')

    def plot_pie_chart(self):
        data = {'Apples': 30, 'Bananas': 40, 'Grapes': 25, 'Oranges': 20}
        labels = list(data.keys())
        values = list(data.values())
        plt.pie(values, labels=labels, autopct='%1.1f%%')
        plt.title('Pie Chart')

    def plot_line_plot(self):
        x = np.linspace(0, 10, 100)
        y = np.sin(x)
        plt.plot(x, y)
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Line Plot')

    def plot_scatter_plot(self):
        x = np.random.rand(50)
        y = np.random.rand(50)
        plt.scatter(x, y)
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Scatter Plot')

    def plot_histogram(self):
        data = np.random.normal(0, 1, 1000)
        plt.hist(data, bins=30)
        plt.xlabel('Value')
        plt.ylabel('Frequency')
        plt.title('Histogram')

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = ExampleProgram()
    window.setGeometry(100, 100, 800, 600)
    window.show()
    app.exec()