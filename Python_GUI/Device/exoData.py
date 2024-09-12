class ExoData:
    def __init__(self):
        self.tStep = []
        self.rTorque = []
        self.rSetP = []
        self.rState = []
        self.lTorque = []
        self.lSetP = []
        self.lState = []
        self.lFsr = []
        self.rFsr = []

    def addDataPoints(
        self,
        x_Time,
        rightToque,
        rightState,
        rightSet,
        leftTorque,
        leftState,
        leftSet,
        rightFsr,
        leftFsr,
    ):
        self.tStep.append(x_Time)
        self.rTorque.append(rightToque)
        self.rSetP.append(rightSet)
        self.rState.append(rightState)
        self.lTorque.append(leftTorque)
        self.lSetP.append(leftSet)
        self.lState.append(leftState)
        self.lFsr.append(leftFsr)
        self.rFsr.append(rightFsr)
