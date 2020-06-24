
# Read ground_truth_route File
class GroundTruthReader:
    def __init__(self, dataPath):
        self.dataPath = dataPath
        self.groundTruthList = []
        self.initGroundTruth()
    def initGroundTruth(self):
        start = True
        f = open(self.dataPath, "r")
        lines = f.readlines()
        for line in lines:
            if start:
                start = False
                continue
            else:
                self.groundTruthList.append(line.split()[0])
        f.close()
    def getGroundTruth(self):
        return self.groundTruthList