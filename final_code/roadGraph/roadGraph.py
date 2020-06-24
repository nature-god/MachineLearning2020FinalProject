from tools import convertIncludedAngle


# The node in road graph
class Node:
    def __init__(self, nodeID, pos):
        self.nodeId = nodeID
        self.pos = pos
        self.outList = []
        self.know = False
        self.cost = float('INF')
        self.prev = -3

    def appendOutList(self, toNode):
        self.outList.append(toNode)


# The edge in road graph
class Edge:
    def __init__(self, startNodeId, endNodeId, weight, value):
        self.startNodeId = startNodeId
        self.endNodeId = endNodeId
        self.weight = weight
        self.edgevalue = value


# Convert the road data into graph format
# @param roadNetwork is the road edge in format of "the element in roadData list"
# @param maxIterDepth is the max iteration depth when we using the
#        dijkstra algorithm to find the route distance
class roadGraph:
    def __init__(self, roadNetwork, maxIterDepth=10):
        self.maxIterDepth = maxIterDepth
        self.nodes = {}
        self.edges = {}
        self.nodeSet = set()
        self.roadNetwork = roadNetwork
        self.reset()

    # Reset the road graph
    # Using nodeSet to represent the priority queue in dijkstra algorithm
    def reset(self):
        self.nodes = {}
        self.edges = {}
        self.nodeSet = set()
        for seg in self.roadNetwork:
            startId = seg['fromNodeId']
            endId = seg['toNodeId']
            startPos = seg['ways'][0]
            endPos = seg['ways'][-1]
            if startId not in self.nodes:
                self.nodes[startId] = Node(startId, startPos)
                self.nodeSet.add(startId)
            if endId not in self.nodes:
                self.nodes[endId] = Node(endId, endPos)
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
    def dijkstra(self, fromId, toId, Wu):
        nodes = self.nodes
        edges = self.edges
        nodeSet = self.nodeSet

        # Using this method to simulate the pop process of the priority queue
        def get_unknown_min():
            minCost = float('INF')
            minIdx = 0
            for i in nodes:
                if nodes[i].know is True:
                    continue
                elif nodes[i].cost < minCost:
                    minCost = nodes[i].cost
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
                if nodes[index].cost == float('INF'):
                    travelPath.append(None)
                    return
                travelPath.append(index)
                get_traj(nodes[index].prev)
            get_traj(index)
            if travelPath[-1] is None:
                return float('INF'), None
            return nodes[index].cost, travelPath[::-1]

        # Main part of the dijkstra algorithm
        nodes[fromId].cost = 0
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
                if Wu is None or node.prev == -3:
                    curCost = edgesLength
                else:
                    curCost = edgesLength + Wu * convertIncludedAngle(nodes[node.prev].pos, node.pos, nodes[w].pos)
                if nodes[w].cost == float('INF'):
                    nodes[w].cost = node.cost + curCost
                    nodes[w].prev = node.nodeId
                elif node.cost + edgesLength < nodes[w].cost:
                    nodes[w].cost = node.cost + curCost
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
        nodes[virtualId] = Node(virtualId, candidateNode['pos'])
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

    # If we use the distance as the cost metrics only, then set w to None
    def computePathDistance(self, fromNode, toNode, w=None):
        if fromNode['edge'].roadEdges['edgeId'] == toNode['edge'].roadEdges['edgeId']:
            d = (fromNode['fraction'] - toNode['fraction']) * fromNode['edge'].roadEdges['waysLength']
            path = [fromNode['edge'].roadEdges['fromNodeId'], fromNode['edge'].roadEdges['toNodeId']]
            return d, path
        # Reset the road graph because the dijkstra algorithm will change the origin road graph
        # Add the vitura node
        self.reset()
        vituralId1 = fromNode['edge'].roadEdges['fromNodeId'] + '_' + \
                     fromNode['edge'].roadEdges['toNodeId'] + '_v1'
        vituralId2 = toNode['edge'].roadEdges['fromNodeId'] + '_' + \
                     toNode['edge'].roadEdges['toNodeId'] + '_v2'
        self.addVirtualNode(fromNode, vituralId1)
        self.addVirtualNode(toNode, vituralId2)
        d, path = self.dijkstra(vituralId1, vituralId2, w)
        # Return the route distance and the corresponding node path
        return d, path