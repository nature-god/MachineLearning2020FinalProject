import os
import datetime
from tools import greatCircleDistance


# Reading GPS data from local txt file
# @param dataPath is the gps_data.txt file's path
# @param sigma is the parameter used to remove points that are too close with each other
class GpsDataReader:
    def __init__(self, dataPath, sigma=4.07):
        self.dataPath = dataPath
        self.sigma = sigma
        self.gpsDataList = []
        self.initGpsData()

    # Init GpsData, read all lines in gps_data.txt file and convert it to dict format
    def initGpsData(self):
        month = {'Jan': 1, 'Feb': 2, 'Mar': 3, 'Apr': 4, 'May': 5, 'June': 6,
                 'July': 7, 'Aug': 8, 'Sept': 9, 'Oct': 10, 'Nov': 11, 'Dec': 12}
        with open(self.dataPath) as f:
            data = f.readlines()
            for i, line in enumerate(data):
                if i == 0:
                    continue
                else:
                    element = line.split('\t')
                    ymd = element[0].split('-')
                    hms = element[1].split(':')
                    time = datetime.datetime(int(ymd[2]), month[ymd[1]], int(ymd[0]),
                                             int(hms[0]), int(hms[1]), int(hms[2]))
                    pos = (float(element[2]), float(element[3]))
                    self.gpsDataList.append({'time': time, 'pos': pos})
            f.close()

    # Return the GpsData after sigma point remove and time degrade
    # @param sample_rate_sec means the minimum time interval between two sampled points
    # @return type is the list of data:
    #            data['pos']:  data position in (latitude, longitude)
    #            data['time']:  data sampled time
    #            data['speed']:  speed of data point
    def getGpsData(self, sample_rate_sec=1):
        # process the time degrade
        downSampleData = []
        for i in range(len(self.gpsDataList)):
            if not downSampleData:
                downSampleData.append(self.gpsDataList[i])
            else:
                interval = self.gpsDataList[i]['time'] - downSampleData[-1]['time']
                if interval.seconds >= sample_rate_sec:
                    downSampleData.append(self.gpsDataList[i])
        # process the sigma point remove and caculate the speed of point
        finalData = []
        for i in range(len(downSampleData)):
            if not finalData:
                downSampleData[i]['speed'] = 0
                finalData.append(downSampleData[i])
            else:
                interval = downSampleData[i]['time'] - finalData[-1]['time']
                distance = greatCircleDistance(downSampleData[i]['pos'], finalData[-1]['pos'])
                if distance >= 2 * self.sigma:
                    downSampleData[i]['speed'] = distance / interval.seconds
                    finalData.append(downSampleData[i])
        return finalData