import tkinter as tk

from async_tkinter_loop import async_handler, async_mainloop

from Device import exoDeviceManager, exoTrial
from views.activeTrial import ActiveTrial
from views.activeTrialSettings import UpdateTorque
from views.scanWindow import ScanWindow


class ControllerApp(tk.Tk):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.trial = exoTrial.ExoTrial(True, 1, True)
        self.deviceManager = exoDeviceManager.ExoDeviceManager()
        self.title("NAU Lab of Biomechatronics")
        self.geometry("920x720")
        self.minsize(900, 700)

        container = tk.Frame(self)
        container.pack(side="top", fill="both", expand=True)
        container.grid_rowconfigure(0, weight=1)
        container.grid_columnconfigure(0, weight=1)

        self.frames = {}
        # Names of each frame goes here
        for F in (ScanWindow, ActiveTrial, UpdateTorque):
            page_name = F.__name__
            frame = F(parent=container, controller=self)
            self.frames[page_name] = frame

            frame.grid(row=0, column=0, sticky="nsew")

        self.show_frame("ScanWindow")  # Switch to Scan Window Frame

    def show_frame(self, page_name):  # Method to switch frames
        frame = self.frames[page_name]
        if frame == "ActiveTrial":
            self.frame[frame].show()

        # Stop plot updates when switching frames
        if hasattr(self.frames.get("ActiveTrial", None), "stop_plot_updates"):
            self.frames["ActiveTrial"].stop_plot_updates()

        frame.tkraise()


def exec():
    controller = ControllerApp()
    async_mainloop(controller)


if __name__ == "__main__":
    exec()
