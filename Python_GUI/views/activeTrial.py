import tkinter as tk
from tkinter import (BOTTOM, CENTER, LEFT, RIGHT, TOP, E, IntVar, N, StringVar,
                     W, X, Y, ttk)

from async_tkinter_loop import async_handler

from Widgets.Charts.chart import BottomPlot, TopPlot


# Active Trial Frame
class ActiveTrial(tk.Frame):
    # Constructor for frame
    def __init__(self, parent, controller):
        super().__init__(parent)
        self.controller = controller
        self.var = IntVar()
        self.chartVar = StringVar()
        self.chartVar.set("Controller")

        self.chartDropdown = ttk.Combobox(
            self,
            textvariable=self.chartVar,
            state="readonly",
            values=[
                "Controller",
                "Sensor",
            ],
        )
        # Active Trial title label
        calibrationMenuLabel = tk.Label(self, text="Active Trial", font=("Arial", 40))
        calibrationMenuLabel.pack(side=TOP, anchor=N, pady=20)

        self.topPlot = TopPlot(self)
        self.bottomPlot = BottomPlot(self)

        self.chartDropdown.bind("<<ComboboxSelected>>", self.newSelection)
        self.chartDropdown.pack()

        self.currentPlots = [self.topPlot, self.bottomPlot]
        self.plot_update_job = None  # Store the job reference

        self.create_widgets()

    # Frame UI elements
    def create_widgets(self):
        # Update torque button
        updateTorqueButton = tk.Button(
            self,
            text="Update Torque",
            height=2,
            width=10,
            command=lambda: self.controller.show_frame("UpdateTorque"),
        )
        updateTorqueButton.pack(side=BOTTOM, anchor=W, padx=7, pady=7)

        # End Trial Button
        endTrialButton = tk.Button(
            self,
            text="End Trial",
            height=2,
            width=10,
            command=async_handler(self.on_end_trial_button_clicked),
        )
        endTrialButton.pack(side=BOTTOM, anchor=E, pady=7, padx=7)

    def newSelection(self, event=None):
        # Determine which plots to show
        selection = self.chartVar.get()
        self.update_plots(selection)

    def update_plots(self, selection):
        # Cancel the previous update job if it exists
        if self.plot_update_job:
            self.after_cancel(self.plot_update_job)

        # Animate all current plots
        for plot in self.currentPlots:
            plot.animate(selection)

        # Schedule the next update
        self.plot_update_job = self.after(
            20, self.update_plots, selection
        )  # Schedule with a delay

    def stop_plot_updates(self):
        if self.plot_update_job:
            self.after_cancel(self.plot_update_job)
            self.plot_update_job = None

    def show(self):
        self.newSelection()
    async def on_end_trial_button_clicked(self):
        await self.endTrialButtonClicked()

    async def endTrialButtonClicked(self):
        await self.ShutdownExo()
        self.controller.show_frame("ScanWindow")

    async def ShutdownExo(self):
        # End trial
        await self.controller.deviceManager.motorOff()  # Turn off motors
        await self.controller.deviceManager.stopTrial()  # End trial
        # Disconnect from Exo
        self.controller.trial.loadDataToCSV(
            self.controller.deviceManager
        )  # Load data from Exo into CSV
