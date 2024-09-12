import tkinter as tk
from tkinter import (BOTTOM, CENTER, LEFT, RIGHT, TOP, E, N, S, StringVar, W,
                     X, Y, ttk)

from async_tkinter_loop import async_handler

jointMap = {
    "Right hip": 1,
    "Left hip": 2,
    "Right knee": 3,
    "Left knee": 4,
    "Right ankle": 5,
    "Left ankle": 6,
    "Right elbow": 7,
    "Left elbow": 8,
}


class UpdateTorque(tk.Frame):  # Frame to start exo and calibrate
    def __init__(self, parent, controller):  # Constructor for Frame
        super().__init__(parent)  # Correctly initialize the tk.Frame part
        # Initialize variables
        self.controller = controller  # Controller object to switch frames
        self.bilateralButtonVar = StringVar()
        self.bilateralButtonVar.set("Bilateral Mode On")
        self.jointVar = StringVar()

        # Joint select
        self.jointSelector = ttk.Combobox(
            self,
            textvariable=self.jointVar,
            state="readonly",
            values=[
                "Left hip",
                "Left knee",
                "Left ankle",
                "Left elbow",
                "Right hip",
                "Right knee",
                "Right ankle",
                "Right elbow"
            ],
        )

        self.isBilateral = True

        self.create_widgets()

    def create_widgets(self):  # Frame UI elements
        # Back button to go back to Scan Window
        backButton = tk.Button(
            self, text="Back", command=lambda: self.controller.show_frame("ActiveTrial")
        )
        backButton.pack(side=TOP, anchor=W, pady=20, padx=10)

        # Calibrate Menu label
        calibrationMenuLabel = tk.Label(
            self, text="Update Torque Settings", font=("Arial", 40)
        )
        calibrationMenuLabel.pack(anchor=CENTER, side=TOP, pady=15)

        # Controller label
        controllerInputLabel = tk.Label(self, text="Controller", font=("Arial", 30))
        controllerInput = tk.Text(self, height=2, width=10)
        # Parameter Label
        parameterInputLabel = tk.Label(self, text="Parameter", font=("Arial", 30))
        parameterInput = tk.Text(self, height=2, width=10)
        # Value label
        valueInputLabel = tk.Label(self, text="Value", font=("Arial", 30))
        valueInput = tk.Text(self, height=2, width=10)

        self.jointSelector.bind("<<ComboboxSelected>>", self.newSelection)

        bilateralButton = tk.Button(
            self,
            textvariable=self.bilateralButtonVar,
            height=2,
            width=10,
            command=self.toggleBilateral,
        )

        jointLabel = tk.Label(self, text="Select Joint", font=("Arial", 30)).pack()
        self.jointSelector.pack(pady=5)
        bilateralButton.pack(pady=5)
        controllerInputLabel.pack(pady=5)
        controllerInput.pack(pady=5)
        parameterInputLabel.pack(pady=5)
        parameterInput.pack(pady=5)
        valueInputLabel.pack(pady=5)
        valueInput.pack(pady=5)

        # Button to start trial
        updateTorqueButton = tk.Button(
            self,
            text="Update Torque",
            height=2,
            width=10,
            command=async_handler(
                self.on_update_button_clicked,
                controllerInput,
                parameterInput,
                valueInput,
            ),
        )
        updateTorqueButton.pack(side=BOTTOM, fill=X, padx=20, pady=20)

    async def on_update_button_clicked(
        self, controllerInput, parameterInput, valueInput
    ):
        await self.UpdateButtonClicked(
            self.isBilateral,
            jointMap[self.jointVar.get()],
            controllerInput,
            parameterInput,
            valueInput,
        )

    async def UpdateButtonClicked(
        self, isBilateral, joint, controllerInput, parameterInput, valueInput
    ):

        controllerVal = float(controllerInput.get(1.0, "end-1c"))
        parameterVal = float(parameterInput.get(1.0, "end-1c"))
        valueVal = float(valueInput.get(1.0, "end-1c"))

        print(f"bilateral: {isBilateral}")
        print(f"joint: {joint}")
        print(f"controller: {controllerVal}")
        print(f"paramter: {parameterVal}")
        print(f"value: {valueVal}")

        # Set Torque
        await self.controller.deviceManager.updateTorqueValues(
            [isBilateral, joint, controllerVal, parameterVal, valueVal]
        )

        self.controller.show_frame("ActiveTrial")

    def newSelection(self, event):
        self.jointVar.set(self.jointSelector.get())

    def toggleBilateral(self):
        if self.isBilateral is True:
            self.isBilateral = False
            self.bilateralButtonVar.set("Bilateral Mode Off")
        else:
            self.isBilateral = True
            self.bilateralButtonVar.set("Bilateral Mode On")
