class ChartData:
    def __init__(self):
        self.rightTorque = 0.0
        self.rightState = 0.0
        self.leftTorque = 0.0
        self.leftState = 0.0
        self.rightSet = 0.0
        self.leftSet = 0.0
        self.rightFsr = 0.0
        self.leftFsr = 0.0

    def updateValues(
        self,
        rightTorque,
        rightState,
        leftTorque,
        leftState,
        rightSet,
        leftSet,
        rightFsr,
        leftFsr,
    ):
        self.rightTorque = rightTorque
        self.rightState = rightState
        self.leftTorque = leftTorque
        self.leftState = leftState
        self.rightSet = rightSet
        self.leftSet = leftSet
        self.rightFsr = rightFsr
        self.leftFsr = leftFsr
