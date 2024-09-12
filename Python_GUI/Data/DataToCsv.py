import csv

from _datetime import datetime


class CsvWritter:
    def writeToCsv(self, exoData):
        print("Creating filedata")
        # initialize array for output file
        fileData = []
        # establish field arrays for output file
        tStep = ["TStep"]
        rTorque = ["RTorque"]
        rSetP = ["RSetP"]
        rState = ["RState"]
        lTorque = ["LTorque"]
        lSetP = ["LSetP"]
        LState = ["LState"]
        lFsr = ["LFsr"]
        rFsr = ["RFsr"]

        # append data to field array
        for xt in exoData.tStep:
            tStep.append(xt)
        for rT in exoData.rTorque:
            rTorque.append(rT)
        for rSP in exoData.rSetP:
            rSetP.append(rSP)
        for rS in exoData.rState:
            rState.append(rS)
        for lT in exoData.lTorque:
            lTorque.append(lT)
        for lSP in exoData.lSetP:
            lSetP.append(lSP)
        for lS in exoData.lState:
            LState.append(lS)
        for rF in exoData.rFsr:
            rFsr.append(rF)
        for lF in exoData.lFsr:
            lFsr.append(lF)
        for tS in exoData.tStep:
            tStep.append(tS)

        # add field array with data to output file
        fileData.append(tStep)
        fileData.append(rTorque)
        fileData.append(rSetP)
        fileData.append(rState)
        fileData.append(lTorque)
        fileData.append(lSetP)
        fileData.append(LState)
        fileData.append(lFsr)
        fileData.append(rFsr)

        # rotate 2D array to place lables on top
        fileDataTransposed = self.rotateArray(fileData)
        print("flipping array")

        today = datetime.now()  # Pull system time and date
        fileName = today.strftime(
            "%Y-%b-%d-%H-%M-%S"
        )  # Format file name based on YYYY-MM-DD-HH:MM:SS
        fileName += ".csv"  # Add .csv to file name
        print("file is: ", fileName)

        with open(fileName, "w") as csvFile:  # Open file with file name
            csvwriter = csv.writer(csvFile)  # Prep file for csv data
            print("creating and opening file")

            # Write flipped 2D array to file
            csvwriter.writerows(fileDataTransposed)

            csvFile.close  # Close file

    def rotateArray(self, arrayToFlip):
        return [
            list(row) for row in zip(*arrayToFlip)
        ]  # Roate array so labels on left are on top
