import re
from tools import greatCircleDistance


# Reading road data from local txt file
# @param dataPath is the road_network.txt file's path
class roadDataReader:
    def __init__(self, dataPath):
        self.dataPath = dataPath
        self.roadDataList = []
        self.idToNodes = {}
        self.initRoadData()

    # Init road data, read all lines in road_network.txt file and convert it to dict format
    def initRoadData(self):
        with open(self.dataPath) as f:
            data = f.readlines()
            for i, line in enumerate(data):
                if i == 0:
                    continue
                else:
                    element = line.split('\t')
                    newElement = {}
                    newElement['edgeId'] = element[0]
                    newElement['fromNodeId'] = element[1]
                    newElement['toNodeId'] = element[2]
                    newElement['isTwoWay'] = (element[3] == '1')
                    newElement['speed'] = float(element[4])
                    ways = re.findall(r'[(](.*?)[)]', element[6])[0].split(',')
                    newElement['ways'] = []
                    for way in ways:
                        way = way.strip()
                        newElement['ways'].append((float(way.split(' ')[1]), float(way.split(' ')[0])))
                    newElement['waysLength'] = 0
                    newElement['segLength'] = []
                    segLength = 0
                    newElement['segLength'].append(segLength)
                    for j in range(1, len(newElement['ways'])):
                        segLength += greatCircleDistance(newElement['ways'][j - 1], newElement['ways'][j])
                        newElement['segLength'].append(segLength)
                    newElement['waysLength'] = segLength
                    self.idToNodes[element[0]] = {'fromNode': element[1], 'toNode': element[2]}
                    self.roadDataList.append(newElement)
            f.close()

    # Return the roadData
    # @return type is the list of data:
    #            data['edgeId']:  the id of this road
    #            data['fromNodeId']:  the id of the road's start node
    #            data['toNodeId']:  the id of the road's end node
    #            data['isTwoWay']:  whether the road could be traveled the opposite direction
    #            data['speed']:  speed limit of this road
    #            data['ways']:  the node list of this road
    #            data['segLength']:  the list of the length from the start node to the ith node
    #            data['waysLength']:  the length from the start point to the end ndoe
    def getRoadData(self):
        return self.roadDataList

    # Return the edges fromnode and tonode by its edgeId
    def getRoadSegNode(self, edgeId):
        if edgeId not in self.idToNodes:
            return None
        else:
            return self.idToNodes[edgeId]