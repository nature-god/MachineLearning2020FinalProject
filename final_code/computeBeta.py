import numpy as np
from timeStep import timeStep
from tools import greatCircleDistance
from HMM import normalizedTransitionMetric
from dataReader import GroundTruthReader

# compute all the beta distribution for the gps data using the MAD method
def computeMADofBeta(gpsData, roadNetwork, roadGraph, searchRadius=50):
    betaData = []
    prevTimeStep = None
    groundTruthRouter = GroundTruthReader('./data/ground_truth_route.txt').getGroundTruth();
    for i, ob in enumerate(gpsData):
        print("Processing with the %dth gpsData"%i)
        candidates = roadNetwork.searchRoadEdge(ob, searchRadius)
        if len(candidates) == 0:
            prevTimeStep = None
            print("No candidates for the %dth gpsData"%i)
        if len(candidates) == 1:
            if candidates[0]['edge'].roadEdges['edgeId'] not in groundTruthRouter:
                print(candidates[0]['edge'].roadEdges['edgeId'])
                continue
            curTimeStep = timeStep(ob, candidates)
            if prevTimeStep is None:
                pass
            else:
                linearDist = greatCircleDistance(prevTimeStep.observation['pos'],
                                                 curTimeStep.observation['pos'])
                timeDiff = curTimeStep.observation['time'] - prevTimeStep.observation['time']
                timeDiff = timeDiff.seconds
                prevCandidate = prevTimeStep.candidates[0]
                curCandidate = curTimeStep.candidates[0]
                routeDist, _ = roadGraph.computePathDistance(prevCandidate, curCandidate, 200)
                absDist = normalizedTransitionMetric(routeDist, linearDist, timeDiff)
                betaData.append(absDist)
            prevTimeStep = curTimeStep
    length = len(betaData)
    betaData = np.array(betaData)
    mad = np.median(betaData)
    for j in range(length):
        betaData[j] = abs(betaData[j] - mad)
    beta = np.median(betaData)/np.log(2)
    return beta


if __name__ == '__main__':
    pass
