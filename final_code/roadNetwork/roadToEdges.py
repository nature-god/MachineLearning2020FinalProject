from rtree import index
from tools import greatCircleDistance, distanceToLatitude, distanceToLongitude, findFootPoint


# Road edge index
# @param roadEdges is the road edge in format of "the element in roadData list"
# @param idx means the candidate is on the idx th line segment in this road
class roadEdgeIdx:
    def __init__(self, roadEdges, idx):
        self.roadEdges = roadEdges
        self.idx = idx


# Convert the road data into road edges using RTree
class roadToEdges:
    def __init__(self):
        self.RTree = index.Index()
        self.treeIdx = 0

    # Add all the line segment in this road to the RTree in a rectangle way
    def addRoadEdge(self, roadEdges):
        for i in range(1, len(roadEdges['ways'])):
            fromPointLat, fromPointLon = roadEdges['ways'][i - 1]
            toPointLat, toPointLon = roadEdges['ways'][i]
            leftPos = min(fromPointLat, toPointLat)
            rightPos = max(fromPointLat, toPointLat)
            bottomPos = min(fromPointLon, toPointLon)
            topPos = max(fromPointLon, toPointLon)
            objIdx = roadEdgeIdx(roadEdges, i)
            self.RTree.insert(self.treeIdx, (leftPos, bottomPos, rightPos, topPos), obj=objIdx)
            self.treeIdx += 1

    # Find the road segment crossing the gpsPoint within the searchRadius in RTree
    # Return the candidates list
    def searchRoadEdge(self, gpsPoint, searchRadius):
        pos = gpsPoint['pos']
        deltaLon = distanceToLongitude(pos[0], searchRadius);
        deltaLat = distanceToLatitude(searchRadius)
        leftPos = pos[0] - deltaLat
        rightPos = pos[0] + deltaLat
        bottomPos = pos[1] - deltaLon
        topPos = pos[1] + deltaLon
        crossEdges = list(self.RTree.intersection((leftPos, bottomPos, rightPos, topPos), objects=True))
        return self.computeCandidates(gpsPoint, crossEdges, searchRadius)

    # compute the candidates on the found crossEdges for the gpsPoint (x_{t,j} in paper)
    # @return type is the list of data:
    #            data['edge']:  the road which the candidate located, the format is roadEdgeIdx
    #            data['pos']:  the candidate position in (latitude, longitude)
    #            data['fraction']:  (the length between start node to candidate points) / (the road length)
    def computeCandidates(self, gpsPoint, crossEdges, searchRadius):
        result = []
        candidatesMap = {}
        pos = gpsPoint['pos']
        speed = gpsPoint['speed']
        for i in range(len(crossEdges)):
            edgeId = crossEdges[i].object.roadEdges['edgeId']
            limitSpeed = crossEdges[i].object.roadEdges['speed']
            if speed < limitSpeed * 3:
                if edgeId not in candidatesMap:
                    candidatesMap[edgeId] = [crossEdges[i].object]
                else:
                    candidatesMap[edgeId].append(crossEdges[i].object)
            else:
                # Speed out of limit
                pass
        for road in candidatesMap:
            minDistance = float('INF')
            candidate = None
            for edge in candidatesMap[road]:
                pointA = edge.roadEdges['ways'][edge.idx - 1]
                pointB = edge.roadEdges['ways'][edge.idx]
                footPoint = findFootPoint(pos, pointA, pointB)
                # if the candidate is on the line but not on the line segment, continue
                if footPoint is None:
                    continue
                else:
                    distance = greatCircleDistance(pos, footPoint)
                    # each road could have one candidate at most
                    if distance <= searchRadius and distance < minDistance:
                        minDistance = distance
                        candidate = {}
                        candidate['edge'] = edge
                        candidate['pos'] = footPoint
                        d = greatCircleDistance(footPoint, pointA)
                        candidate['fraction'] = (edge.roadEdges['segLength'][edge.idx - 1] + d) \
                                                / edge.roadEdges['waysLength']
            if candidate is not None:
                result.append(candidate)
        return result
