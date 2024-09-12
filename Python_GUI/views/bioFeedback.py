import tkinter as tk
from tkinter import (BOTTOM, CENTER, LEFT, RIGHT, TOP, E, N, S, IntVar, StringVar, W,
                     X, Y, ttk)

from async_tkinter_loop import async_handler
from Widgets.Charts.chart import (TopPlot,BottomPlot)


class BioFeedback(tk.Frame):  # Frame to start exo and calibrate
    def __init__(self, parent, controller):  # Constructor for Frame
        super().__init__(parent)  # Correctly initialize the tk.Frame part
        # Initialize variables
        self.controller = controller  # Controller object to switch frames
        self.var = IntVar()
        self.chartVar = StringVar()
        self.chartVar.set("Leg")

        #Drop down menu for legs
        self.chartDropdown = ttk.Combobox(
            self,
            textvariable=self.chartVar,
            state="readonly",
            values=[
                "Left Leg",
                "Right Leg",
            ],
        )

        self.create_widgets()


    def create_widgets(self):  # Frame UI elements
         # Back button to go back to Scan Window
        backButton = tk.Button(
             self, text="Back", command=lambda: self.controller.show_frame("ActiveTrial")
         )
        backButton.pack(side=TOP, anchor=W, pady=10, padx=10)

        # Top label for frame
        calibrationMenuLabel = tk.Label(self, text="Biofeedback", font=("Arial", 40))
        calibrationMenuLabel.pack(side=TOP, anchor=N, pady=20)

        #Plot for fram
        self.topPlot = TopPlot(self)

        self.chartDropdown.bind()
        self.chartDropdown.pack()

