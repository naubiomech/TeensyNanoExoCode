import tkinter as tk
from tkinter import (BOTTOM, CENTER, LEFT, RIGHT, TOP, E, N, S, IntVar, StringVar, W,
                     X, Y, ttk, simpledialog)
import pygame  # Import pygame for sound
from async_tkinter_loop import async_handler
from PIL import ImageTk, Image, ImageEnhance

from Widgets.Charts.chart import FSRPlot
from custom_keyboard import CustomKeyboard

# Initialize Pygame for sound
pygame.mixer.init()

# Load sound file (ensure the path is correct)
# Replace 'notification.wav' with your sound file
notification_sound = pygame.mixer.Sound('notification.wav')

# Biofeedback Frame
class BioFeedback(tk.Frame):
    # Constructor for frame
    def __init__(self, parent, controller):
        super().__init__(parent)
        self.controller = controller  # Reference to the main application controller
        self.is_plotting = False  # Flag to control if plotting should happen

        self.chartVar = StringVar()  # Variable for storing the selected leg
        self.chartVar.set("Left Leg")  # Default selection

        # Set the disconnection callback
        self.controller.deviceManager.on_disconnect = self.BioFeedback_on_device_disconnected

        # Counter variable to track the number of goals reached
        self.counter = 0
        self.counter_var = IntVar(value=self.counter)

        #UI Styling
        self.fontstyle = 'Segoe UI'
        
        # Target value variable
        self.target_value = None
        self.target_var = StringVar(value="No target set")  # Display the target value

        # Create the buttons and labels
        self.create_widgets()

    def create_widgets(self):
        style = ttk.Style()
        style.configure("Custom.TCombobox", font=(self.fontstyle, 16), padding=10)

        # Back button to return to the Active Trial frame
        backButton = ttk.Button(self, text="Back", command=self.handle_back_button)
        backButton.grid(row=0, column=0, pady=10)

        # Biofeedback title label
        calibrationMenuLabel = ttk.Label(self, text="Biofeedback", font=(self.fontstyle, 40))
        calibrationMenuLabel.grid(row=0, column=0, columnspan=8, pady=20)

        # Load and place the smaller image besides the timer and battery
        small_image = Image.open("./Resources/Images/OpenExo.png").convert("RGBA")
        small_image = small_image.resize((80, 40))  # Resize the image to a smaller size
        self.small_bg_image = ImageTk.PhotoImage(small_image)

        # Create a Canvas for the smaller image
        small_canvas = tk.Canvas(self, width=80, height=50, highlightthickness=0)
        small_canvas.create_image(0, 0, image=self.small_bg_image, anchor="nw")
        small_canvas.grid(row=0, column=7, sticky="N", padx=5, pady=10)  # Top-right corner

        # For battery Label
        batteryPercentLabel = ttk.Label(self, 
            textvariable=self.controller.
            deviceManager._realTimeProcessor._exo_data.BatteryPercent, 
                font=(self.fontstyle, 12))
        batteryPercentLabel.grid(row=0, column=7,sticky="E", padx=5, pady=(20, 0)) 

        # Initialize the FSR plot
        self.FSRPlot = FSRPlot(self)
        self.currentPlots = self.FSRPlot  # Current plot reference
        self.FSRPlot.canvas.get_tk_widget().grid(row=1, column=0, rowspan= 3, columnspan=8, sticky="NSEW", pady=5, padx=5)

        self.plot_update_job = None  # Store the job reference for plot updates

        # Label to display targets reached
        self.targets_reached_label = ttk.Label(self, text="Targets Reached: 0",
                                               font=(self.fontstyle, 17))
        self.targets_reached_label.grid(row=4, column=0, columnspan=8, pady=(0,30), sticky="n")

        # Label to display the target value
        self.target_label = ttk.Label(self, textvariable=self.target_var, font=(self.fontstyle, 17))
        self.target_label.grid(row=4, column=0, columnspan=8, pady=(50,0), sticky = "n")

        # Frame for target value buttons
        target_frame = ttk.Frame(self)
        target_frame.grid(row=4, column=0, columnspan=8, pady=(107,0),sticky = 'N')

        # Button to set target value
        self.target_button = ttk.Button(target_frame, text="Set Target Value", 
            command=self.ask_target_value)
        self.target_button.pack(side=LEFT, padx=5)

        # Reset button for target value
        self.reset_button = ttk.Button(target_frame, 
            text="Reset Target Value", command=self.reset_target, state="disabled")
        self.reset_button.pack(side=RIGHT, padx=5)

        # Frame for advanced buttons
        advanced_frame = ttk.Frame(self)
        advanced_frame.grid(row=4, column=0, pady=20)
        
        # Chart selection button
        self.chartButton = ttk.Button(
            advanced_frame,
            text="Left Leg",
            command=self.toggle_chart,
            style="Custom.TButton",
        )
        self.chartButton.pack(side=TOP, padx=5, pady = 10)

        # Mark Trial Button
        markButton = ttk.Button(
            advanced_frame,
            textvariable=self.controller.deviceManager._realTimeProcessor._exo_data.MarkLabel,
            command=async_handler(self.on_mark_button_clicked),
        )
        markButton.pack(side=TOP, padx=5, pady = 10)

        # Recalibrate FSRs Button
        self.recalibrateFSRButton = ttk.Button(
            advanced_frame,
            text="Recalibrate FSRs",
            command=async_handler(self.on_recal_FSR_button_clicked),
        )
        self.recalibrateFSRButton.pack(side=TOP, padx=5, pady = 10)

        # Configure grid weights for centering
        for i in range(6):
            self.grid_rowconfigure(i, weight=1)
        for j in range(8):
            self.grid_columnconfigure(j, weight=1)

    def toggle_chart(self):
        """Toggle between 'Left Leg' and 'Right Leg' for the chart."""
        current = self.chartVar.get()
        if current == "Left Leg":
            self.chartVar.set("Right Leg")
            self.chartButton.config(text="Right Leg")
        else:
            self.chartVar.set("Left Leg")
            self.chartButton.config(text="Left Leg")
        self.newSelection()

    def ask_target_value(self):
        # Prompt the user for a target value
        user_input = simpledialog.askstring("Input", "Please enter a target value:")
        
        if user_input is not None:
            try:
                # Attempt to convert the input to a float
                self.target_value = float(user_input)
                self.update_target_label()  # Update the label with the new target value
                print(f"Target value set to: {self.target_value}")

                # Enable the reset button
                self.reset_button.config(state="normal")

                # Set the goal in the current plot
                self.currentPlots.set_goal(self.target_value)
                self.update_plots(self.chartVar.get())  # Update the plot with the new goal

            except ValueError:
                print("Invalid input. Please enter a numeric value.")

    def handle_back_button(self):
        # Stops plotting and goes back to Active Trial
        self.stop_plot_updates()  # Stop any ongoing plot updates
        self.controller.show_frame("ActiveTrial")  # Switch to ActiveTrial frame
        active_trial_frame = self.controller.frames["ActiveTrial"]
        active_trial_frame.newSelection(self)  # Start the plotting on active trial

    def newSelection(self, event=None):
        # Determine which plots to show based on user selection
        selection = self.chartVar.get()
        self.update_plots(selection)  # Update the plots based on selection
    
    def reset_background(self):
        self.config(bg="SystemButtonFace")  # Reset to default color

    def reset_target(self):
        # Reset the target value to None and update the UI
        self.target_value = None
        self.update_target_label()  # Update the target label
        self.reset_button.config(state="disabled")  # Disable the reset button
        self.currentPlots.set_goal(None)  # Reset the goal in the current plot

    def stop_plot_updates(self):
        # Stop any ongoing plot updates
        if self.plot_update_job:
            self.after_cancel(self.plot_update_job)
            self.plot_update_job = None
        self.is_plotting = False


    def update_counter_label(self):
        # Update the counter variable and label when the goal is reached
        self.counter += 1  # Increment the counter
        self.counter_var.set(self.counter)  # Update the IntVar
        self.targets_reached_label.config(text=f"Targets Reached: {self.counter}")  # Update the label
        
        # Play the notification sound
        notification_sound.play()
        # Change background color to indicate success
        self.config(bg="lightgreen")  # Change to light green
        self.after(1000, self.reset_background)  # Reset after 1 second

    def enable_interactions(self):
        try:
            # Enable other widgets
            for widget in self.winfo_children():
                if isinstance(widget, tk.Button) or isinstance(widget, ttk.Combobox):
                    widget.config(state='normal')
        except Exception as e:
            print(f"Error in enable_interactions: {e}")

    def update_plots(self, selection):
        # Animate the current plot and schedule the next update
        # Cancel the previous update job if it exists
        if self.plot_update_job:
            self.after_cancel(self.plot_update_job)
            self.plot_update_job = None

        # Only continue updating plots if the flag is set to True
        if not self.is_plotting:
            return
        
        # Animate the current plot
        self.currentPlots.animate(selection)

        # Schedule the next update
        self.plot_update_job = self.after(20, self.update_plots, selection)
        
        # Enable interactions after the first plot update is complete
        self.after(20, self.enable_interactions)
        
    def update_target_label(self):
        # Update the target label with the current target value
        if self.target_value is not None:
            self.target_var.set(f"Target value: {self.target_value}")
        else:
            self.target_var.set("No target set")

    def show(self):
        # Show the current selection in the plots
        self.is_plotting = True
        self.newSelection()  # Update the plots based on current selection

    def hide(self):
        # This method is called when switching away from this frame
        self.stop_plot_updates()
        
    async def on_mark_button_clicked(self):
        self.controller.deviceManager._realTimeProcessor._exo_data.MarkVal += 1
        self.controller.deviceManager._realTimeProcessor._exo_data.MarkLabel.set(
            "Mark: " + str(self.controller.
                deviceManager._realTimeProcessor._exo_data.MarkVal))

    # Handle Recalibrate FSRs Button click
    async def on_recal_FSR_button_clicked(self):
        await self.recalibrateFSR()

    # Recalibrate FSRs
    async def recalibrateFSR(self):
        await self.controller.deviceManager.calibrateFSRs()

    def BioFeedback_on_device_disconnected(self):
        tk.messagebox.showwarning("Device Disconnected", "Please Reconnect")
        
        self.controller.trial.loadDataToCSV(
            self.controller.deviceManager, True
        )  # Load data from Exo into CSV
        self.controller.show_frame("ScanWindow")# Navigate back to the scan page
        self.controller.frames["ScanWindow"].show()  # Call show method to reset elements
