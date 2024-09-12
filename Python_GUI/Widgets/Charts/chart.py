import datetime as dt
import tkinter as tk

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


class BasePlot:
    def __init__(self, master, title):
        self.master = master
        self.title = title
        self.figure = plt.Figure(figsize=(7, 2.5))
        self.ax = self.figure.add_subplot(1, 1, 1)
        self.xValues = []
        self.yValues = []
        self.secondY = []

        self.canvas = FigureCanvasTkAgg(self.figure, master=self.master)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack()

    def animate(self):
        raise NotImplementedError("Subclasses should implement this method")

    def update_plot(self, xValues, yValues, secondY, title):
        xValues = xValues[-20:]
        yValues = yValues[-20:]
        secondY = secondY[-20:]

        self.ax.clear()
        self.ax.plot(xValues, yValues)
        self.ax.plot(xValues, secondY)
        self.ax.set_ylim(auto=True)
        self.ax.set_xticks([])

        self.ax.set_title(title)

        self.canvas.draw()


class TopPlot(BasePlot):
    def __init__(self, master):
        super().__init__(master, "Left Torque")

    def animate(self, chartSelection):
        topController = None
        title = " "
        if chartSelection == "Controller":
            topController = (
                self.master.controller.deviceManager._realTimeProcessor._chart_data.leftTorque
            )
            topMeasure = (
                self.master.controller.deviceManager._realTimeProcessor._chart_data.leftSet
            )
            title = "Controller"
        elif chartSelection == "Sensor":
            topController = (
                self.master.controller.deviceManager._realTimeProcessor._chart_data.leftState
            )
            topMeasure = (
                self.master.controller.deviceManager._realTimeProcessor._chart_data.leftFsr
            )
            title = "Sensor"
        if topController is None or topMeasure is None:
            topController = 0
            topMeasure = 0

        self.xValues.append(dt.datetime.now())
        self.yValues.append(topController)
        self.secondY.append(topMeasure)

        self.update_plot(self.xValues, self.yValues, self.secondY, title)


class BottomPlot(BasePlot):
    def __init__(self, master):
        super().__init__(master, "Right Torque")

    def animate(self, chartSelection):
        topController = None
        title = " "
        if chartSelection == "Controller":
            topController = (
                self.master.controller.deviceManager._realTimeProcessor._chart_data.rightTorque
            )
            topMeasure = (
                self.master.controller.deviceManager._realTimeProcessor._chart_data.rightSet
            )
            title = "Controller"
        elif chartSelection == "Sensor":
            topController = (
                self.master.controller.deviceManager._realTimeProcessor._chart_data.rightState
            )
            topMeasure = (
                self.master.controller.deviceManager._realTimeProcessor._chart_data.rightFsr
            )
            title = "Sensor"

        if topController is None or topMeasure is None:
            topController = 0
            topMeasure = 0

        self.xValues.append(dt.datetime.now())
        self.yValues.append(topController)
        self.secondY.append(topMeasure)

        self.update_plot(self.xValues, self.yValues, self.secondY, title)
