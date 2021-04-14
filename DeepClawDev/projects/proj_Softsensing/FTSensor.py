import FTReading


class FTSensor():
    def __init__(self,IP = '192.168.1.101'):
        self.FTSensor = FTReading.FTReading('192.168.1.101')
        self.FTSensor.InitFT()
        self.data = self.FTSensor.GetReading(100)

    def GetData(self):
        self.data = self.FTSensor.GetReading(100)

        return self.data
