import sys
from multiprocessing import Queue
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg
import time

class LoggerWindow(QtWidgets.QWidget):
    def __init__(self, data_queue):
        super().__init__()
        self.setWindowTitle("📊 Live Fish Logger")
        self.resize(1200, 700)

        self.queue = data_queue
        self.data_buffers = {
            "time": [],
            "orientation": {'x': [], 'y': [], 'z': []},
            "position": {'x': [], 'y': [], 'z': []},
            "angular_velocity": {'x': [], 'y': [], 'z': []},
            "linear_velocity": {'x': [], 'y': [], 'z': []}
        }

        layout = QtWidgets.QVBoxLayout()
        grid = QtWidgets.QGridLayout()

        self.plots = {}
        titles = ["Orientation", "Position", "Angular Velocity", "Linear Velocity"]
        keys = ["orientation", "position", "angular_velocity", "linear_velocity"]

        for idx, (title, key) in enumerate(zip(titles, keys)):
            plot = pg.PlotWidget(title=title)
            plot.addLegend()
            plot.setLabel('left', title)
            plot.setLabel('bottom', 'Time (s)')
            plot.setBackground('w')
            plot.showGrid(x=True, y=True, alpha=0.3)
            self.plots[key] = {
                'widget': plot,
                'curves': {
                    'x': plot.plot(pen=pg.mkPen('r', width=2), name='X'),
                    'y': plot.plot(pen=pg.mkPen('g', width=2), name='Y'),
                    'z': plot.plot(pen=pg.mkPen('b', width=2), name='Z'),
                }
            }
            grid.addWidget(plot, idx // 2, idx % 2)

        layout.addLayout(grid)
        self.setLayout(layout)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_data)
        self.timer.start(100)  # 10 fps

    def update_data(self):
        while not self.queue.empty():
            data = self.queue.get()
            t = data['time']
            self.data_buffers['time'].append(t)

            for key in ['orientation', 'position', 'angular_velocity', 'linear_velocity']:
                for axis, value in zip(['x', 'y', 'z'], data[key]):
                    self.data_buffers[key][axis].append(value)

        if len(self.data_buffers['time']) < 2:
            return

        for key in ['orientation', 'position', 'angular_velocity', 'linear_velocity']:
            for axis in ['x', 'y', 'z']:
                self.plots[key]['curves'][axis].setData(
                    self.data_buffers['time'], self.data_buffers[key][axis]
                )
            self.plots[key]['widget'].setXRange(max(0, self.data_buffers['time'][-1] - 30), self.data_buffers['time'][-1])

def run_logger_gui(queue: Queue):
    app = QtWidgets.QApplication(sys.argv)
    win = LoggerWindow(queue)
    win.show()
    sys.exit(app.exec_())
