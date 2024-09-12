import tkinter as tk
from tkinter import (BOTH, BOTTOM, CENTER, DISABLED, LEFT, RIGHT, TOP,
                     PhotoImage, StringVar, X, Y)

from async_tkinter_loop import async_handler


# Frame to scan for exo
class ScanWindow(tk.Frame):
    # Initialize class
    def __init__(self, parent, controller):
        super().__init__(parent)
        self.controller = controller
        self.deviceNameText = StringVar()
        self.startTrialButton = None
        self.create_widgets()

    # Holds all UI elements
    def create_widgets(self):
        titleLabel = tk.Label(self, text="ExoSkeleton Controller", font=("Arial", 50))
        titleLabel.pack(expand=False, fill=X, ipady=50)
        startScanLabel = tk.Label(
            self, text="Begin Scanning for Exoskeletons", font=("Arial", 30)
        )
        startScanLabel.pack(expand=False, fill=X, ipady=10)
        self.deviceNameText.set("Not Connected")
        deviceNameLabel = tk.Label(
            self, textvariable=self.deviceNameText, font=("Arial", 20)
        )
        deviceNameLabel.pack(expand=False, pady=10)
        startScanButton = tk.Button(
            self,
            height=2,
            width=10,
            text="Start Scan",
            command=async_handler(self.on_start_scan_button_clicked),
        )
        startScanButton.pack(expand=False, fill=Y, ipadx=40, pady=100)
        self.startTrialButton = tk.Button(
            self,
            text="Start Trial",
            height=3,
            command=async_handler(self.on_start_trial_button_clicked),
            state=DISABLED,
        )
        self.startTrialButton.pack(
            expand=False, fill=BOTH, side=BOTTOM, pady=20, padx=20
        )

    # Async function to handle button click
    async def on_start_scan_button_clicked(self):
        self.deviceNameText.set("Scanning...")
        await self.startScanButtonClicked()

    # Start scanning for exo
    async def startScan(self):
        await self.controller.deviceManager.scanAndConnect()

    # Start scan and set device name flip start trial state
    async def startScanButtonClicked(self):
        await self.startScan()
        self.deviceNameText.set(self.controller.deviceManager.device)
        # Un-comment this line to enable connected validation
        if self.deviceNameText.get() != "None":
            self.startTrialButton.config(state="normal")

    # Handle start trial button clicked
    async def on_start_trial_button_clicked(self):
        await self.StartTrialButtonClicked()

    # Switch frame to active trial and begin trial
    async def StartTrialButtonClicked(self):
        self.controller.show_frame("ActiveTrial")
        await self.controller.trial.calibrate(self.controller.deviceManager)
        await self.controller.trial.beginTrial(self.controller.deviceManager)
