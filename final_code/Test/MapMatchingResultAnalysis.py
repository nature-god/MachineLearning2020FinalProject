import operator
import re
import matplotlib.pyplot as plt
import math

class Node:
    def __init__(self, nodeID):
        self.nodeId = nodeID
        self.outList = []
        self.know = False
        self.dist = float('INF')
        self.prev = -3

    def appendOutList(self, toNode):
        self.outList.append(toNode)
class Edge:
    def __init__(self, startNodeId, endNodeId, weight, value):
        self.startNodeId = startNodeId
        self.endNodeId = endNodeId
        self.weight = weight
        self.edgevalue = value
def greatCircleDistance(posA, posB):
    EARTH_RADIUS = 6378.137
    radLatA = (posA[0] * math.pi) / 180.0
    radLatB = (posB[0] * math.pi) / 180.0
    radLonA = (posA[1] * math.pi) / 180.0
    radLonB = (posB[1] * math.pi) / 180.0

    a = radLatA - radLatB
    b = radLonA - radLonB
    s = 2 * math.asin(math.sqrt(math.pow(math.sin(a / 2), 2) \
                                + math.cos(radLatA) * math.cos(radLatB) * math.pow(math.sin(b / 2), 2)))
    s = s * EARTH_RADIUS * 1000;
    return s
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

    def getRoadSegNode(self, edgeId):
        if edgeId not in self.idToNodes:
            return None
        else:
            return self.idToNodes[edgeId]
class roadGraph:
    def __init__(self, roadEdges, maxIterDepth=50):
        self.maxIterDepth = maxIterDepth
        self.nodes = {}
        self.edges = {}
        self.nodeSet = set()
        self.roadEdges = roadEdges
        self.reset()

    # Reset the road graph
    # Using nodeSet to represent the priority queue in dijkstra algorithm
    def reset(self):
        self.nodes = {}
        self.edges = {}
        self.nodeSet = set()
        for seg in self.roadEdges:
            startId = seg['fromNodeId']
            endId = seg['toNodeId']
            if startId not in self.nodes:
                self.nodes[startId] = Node(startId)
                self.nodeSet.add(startId)
            if endId not in self.nodes:
                self.nodes[endId] = Node(endId)
                self.nodeSet.add(endId)
            self.nodes[startId].appendOutList(endId)
            self.edges[(startId, endId)] = Edge(startId, endId, seg['waysLength'], seg)
            # If this road is two-way, add an opposite direction edge
            if seg['isTwoWay']:
                self.nodes[endId].appendOutList(startId)
                self.edges[(endId, startId)] = Edge(endId, startId, seg['waysLength'], seg)

    # Return the edges using the index of (startId, endId)
    def getEdgesValue(self, startId, endId):
        return self.edges[(startId, endId)].edgevalue

    # The dijkstra algorithm to find the shortest distance between two point
    def dijkstra(self, fromId, toId):
        nodes = self.nodes
        edges = self.edges
        nodeSet = self.nodeSet

        # Using this method to simulate the pop process of the priority queue
        def get_unknown_min():
            minDist = float('INF')
            minIdx = 0
            for i in nodes:
                if nodes[i].know is True:
                    continue
                elif nodes[i].dist < minDist:
                    minDist = nodes[i].dist
                    minIdx = i
            if minIdx == 0:
                return None
            else:
                nodeSet.remove(minIdx)
                return nodes[minIdx]

        # Backtrack to find the shortest path, if not, return Nsone
        def real_get_traj(start, index):
            travelPath = []

            def get_traj(index):
                if index == start:
                    travelPath.append(index)
                    return
                if nodes[index].dist == float('INF'):
                    travelPath.append(None)
                    return
                travelPath.append(index)
                get_traj(nodes[index].prev)

            get_traj(index)
            if travelPath[-1] is None:
                return float('INF'), None
            return nodes[index].dist, travelPath[::-1]

        # Main part of the dijkstra algorithm
        nodes[fromId].dist = 0
        iterationDepth = 0
        while len(nodeSet) != 0:
            if iterationDepth > self.maxIterDepth:
                break
            node = get_unknown_min()
            if node is None:
                break
            node.know = True
            for w in node.outList:
                if nodes[w].know is True:
                    continue
                edgesLength = edges[(node.nodeId, nodes[w].nodeId)].weight
                if nodes[w].dist == float('INF'):
                    nodes[w].dist = node.dist + edgesLength
                    nodes[w].prev = node.nodeId
                elif node.dist + edgesLength < nodes[w].dist:
                    nodes[w].dist = node.dist + edgesLength
                    nodes[w].prev = node.nodeId
                else:
                    pass
            iterationDepth += 1
        return real_get_traj(fromId, toId)

    # Every time we have two candidate nodes in the edges, we need to add
    # two virtual Node in the road graph for the convenience of calculating
    # the shortest path between these two node
    def addVirtualNode(self, candidateNode, virtualId):
        nodes = self.nodes
        edges = self.edges
        nodeSet = self.nodeSet
        s = candidateNode['edge'].roadEdges['fromNodeId']
        t = candidateNode['edge'].roadEdges['toNodeId']
        roadSegLength1 = candidateNode['edge'].roadEdges['waysLength'] * candidateNode['fraction']
        roadSegLength2 = candidateNode['edge'].roadEdges['waysLength'] * (1 - candidateNode['fraction'])
        e1 = {'edgeId': virtualId, 'fromNodeId': s, 'toNodeId': virtualId, 'waysLength': roadSegLength1}
        e2 = {'edgeId': virtualId, 'fromNodeId': virtualId, 'toNodeId': t, 'waysLength': roadSegLength2}
        nodes[virtualId] = Node(virtualId)
        nodeSet.add(virtualId)
        nodes[s].appendOutList(virtualId)
        nodes[virtualId].appendOutList(t)
        edges[(s, virtualId)] = Edge(s, virtualId, roadSegLength1, e1)
        edges[(virtualId, t)] = Edge(virtualId, t, roadSegLength2, e2)
        if candidateNode['edge'].roadEdges['isTwoWay']:
            nodes[t].appendOutList(virtualId)
            nodes[virtualId].appendOutList(s)
            edges[(t, virtualId)] = Edge(t, virtualId, roadSegLength2, e2)
            edges[(virtualId, s)] = Edge(virtualId, s, roadSegLength1, e1)

            # Compute the route distance between two candidates

    def computePathDistance(self, fromNode, toNode):
        if fromNode['edge'].roadEdges['edgeId'] == toNode['edge'].roadEdges['edgeId']:
            d = (fromNode['fraction'] - toNode['fraction']) * fromNode['edge'].roadEdges['waysLength']
            path = [fromNode['edge'].roadEdges['fromNodeId'], fromNode['edge'].roadEdges['toNodeId']]
            return d, path
        # Reset the road graph because the dijkstra algorithm will change the origin road graph
        # Add the vitura node
        self.reset()
        vituralId1 = fromNode['edge'].roadEdges['fromNodeId'] + '_' + fromNode['edge'].roadEdges['toNodeId'] + '_v1'
        vituralId2 = toNode['edge'].roadEdges['fromNodeId'] + '_' + toNode['edge'].roadEdges['toNodeId'] + '_v2'
        self.addVirtualNode(fromNode, vituralId1)
        self.addVirtualNode(toNode, vituralId2)
        d, path = self.dijkstra(vituralId1, vituralId2)
        # Return the route distance and the corresponding node path
        return d, path
class GroundTruthRouterReader:
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

class ResultAnalysis:
    def __init__(self,labelList,fileList,roadNetworkPath,groundTruthPath):
        roadOrigin = roadDataReader(roadNetworkPath)
        roadData = roadOrigin.getRoadData()
        roadNetwork  = roadGraph(roadData)
        Res = GroundTruthRouterReader(groundTruthPath)
        res_true = Res.getGroundTruth()
        self.labelList = labelList
        self.error_list = [] # 存储所有计算出的误差
        self.recallNo_List = [] # 计算所有不匹配召回率：不匹配点个数/点总数
        self.precision_List = [] #精准度:匹配的路径长度/路径总长度
        self.recall_List = [] # 计算召回率: 匹配点个数/总点个数
        AllLength = 0
        for i in res_true:
            nodeInfo = roadOrigin.getRoadSegNode(i)
            AllLength =  AllLength + roadNetwork.getEdgesValue(nodeInfo['fromNode'],nodeInfo['toNode'])['waysLength']

        print("All: "+ str(AllLength))
        print("==============Matching Result=================")
        '''
        Label_List = ['1','2','5','10','20','30','40','45','60']
        fileList = ['./MapMatchingResultData/result_raw_Length_1.txt',
                    './MapMatchingResultData/result_raw_Length_2.txt',
                    './MapMatchingResultData/result_raw_Length_5.txt',
                    './MapMatchingResultData/result_raw_Length_10.txt',
                    './MapMatchingResultData/result_raw_Length_20.txt',
                    './MapMatchingResultData/result_raw_Length_30.txt',
                    './MapMatchingResultData/result_raw_Length_40.txt',
                    './MapMatchingResultData/result_raw_Length_45.txt',
                    './MapMatchingResultData/result_raw_Length_60.txt']
        '''
        tmpCounter = "" #存储变量
        res = []    # 存储计算结果
        datas = []
        for tmpFile in fileList:
            datas.clear()
            res.clear()
            with open(tmpFile,"r") as f:
                data = f.readlines()
                for line in data:
                    datas.append(line)
            f.close()
            tmpCounter = datas[0]
            for i in datas:
                if operator.eq(i, tmpCounter):
                    continue
                else:
                    if tmpCounter.split()[0] in res_true:
                        res.append(tmpCounter)
                    tmpCounter = i

            res_index = []
            res_value = []
            for i in res:
                res_index.append(i.split()[0])
                res_value.append(float(i.split()[1]))

            errorLength = 0
            precisionLength = 0
            recallNum = 0;
            pointer1 = 0
            pointer2 = 0
            allCount = 0
            errorCount = 0
            while (pointer1 < len(res)) and (pointer2 < len(res_true)):

                allCount = allCount + 1
                if operator.eq(res_index[pointer1],res_true[pointer2]):
                    precisionLength = precisionLength + float(res_value[pointer1])
                    recallNum = recallNum + 1
                    pointer1 = pointer1+1
                    pointer2 = pointer2+1
                else:
                    if res_true[pointer2] not in res_index[pointer1:]:
                        pointer2 = pointer2 + 1
                    else:
                        pointer1 = pointer1 + 1
                    errorLength = errorLength + float(res_value[pointer1])
                    errorCount = errorCount + 1
            print("Error Length : " + str(errorLength))
            print("All Length : " + str(AllLength))
            print("error rate is: %.2f" %(errorLength/AllLength*100) + "%")
            print("recall rate is: %.2f" %(errorCount/allCount*100) + "%")
            self.recallNo_List.append(errorCount/allCount)
            self.error_list.append(errorLength/AllLength)
            self.precision_List.append(precisionLength/AllLength)
            self.recall_List.append(recallNum/allCount)
            print("=======================================================")
    def DrawBarGraph(self):
        x = list(range(len(self.labelList)))
        total_width, n = 0.8, 2
        width = total_width / n
        plt.ylim(0, 1)
        plt.bar(x, self.recallNo_List, width=width, label='Recall Rate', fc='#ffff66')
        for i in range(len(x)):
            x[i] = x[i] + width
        plt.bar(x, self.error_list, width=width, label='Error Rate', tick_label=self.labelList, fc='#3399ff')
        plt.legend()
        plt.show()
    def DrawLineGraph(self):
        plt.ylim(0, 1)
        lableError1 = []
        lableError2 = []
        labelReCall1 = []
        labelReCall2 = []
        labelPrecision1 = []
        labelPrecision2 = []
        for i in range(len(self.labelList)):
            lableError1.append(self.error_list[i])
            labelReCall1.append(self.recall_List[i])
            labelPrecision1.append(self.precision_List[i])
        for i in range(len(self.labelList)):
            lableError2.append(self.error_list[len(self.labelList)-1])
            labelReCall2.append(self.recall_List[len(self.labelList)-1])
            labelPrecision2.append(self.precision_List[len(self.labelList)-1])

        #错误率 error
        plt.ylim(0, 1)
        l1 = plt.plot(self.labelList,lableError1,'r--',label='w_turn')
        l2 = plt.plot(self.labelList,lableError2,'y--',label='no_w')
        plt.title('Route mismatched fraction(error)')
        plt.xlabel('W_turn')
        plt.ylabel('ErrorRate')
        plt.legend()
        plt.show()

        #精准度 Precision
        plt.ylim(0, 1)
        l3 = plt.plot(self.labelList,labelPrecision1,'r--',label='w_turn')
        l4 = plt.plot(self.labelList,labelPrecision2,'y--',label='no_w')
        plt.title('Precision')
        plt.xlabel('W_turn')
        plt.ylabel('Precison')
        plt.legend()
        plt.show()

        #召回率 Recall
        plt.ylim(0, 1)
        l3 = plt.plot(self.labelList,labelReCall1,'r--',label='w_turn')
        l4 = plt.plot(self.labelList,labelReCall2,'y--',label='no_w')
        plt.title('Recall')
        plt.xlabel('W_turn')
        plt.ylabel('Recall')
        plt.legend()
        plt.show()


