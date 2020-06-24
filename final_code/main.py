from dataReader import GpsDataReader, roadDataReader
from roadNetwork import roadToEdges
from roadGraph import roadGraph
from computeBeta import computeMADofBeta
from HMM import computeViterbiSequence

if __name__ == '__main__':
    print("Start reading the road data")
    roadOrigin = roadDataReader('./data/road_network.txt')
    roadData = roadOrigin.getRoadData()
    print("Reading the road data is done")

    print("Start reading the GPS data")
    gpsOrigin = GpsDataReader('./data/gps_data.txt')
    gpsData = gpsOrigin.getGpsData(sample_rate_sec=120)
    print("End reading the GPS data is done")

    print("Start initializing the roadNetwork")
    roadNetwork = roadToEdges()
    for seg in roadData:
        roadNetwork.addRoadEdge(seg)
    print("Initializing the roadNetwork is done")

    print("Start initializing the roadGraph")
    roadGraph = roadGraph(roadData, maxIterDepth=120)
    print("Initializing the roadGraph is done")

    # beta = computeMADofBeta(gpsData, roadNetwork, roadGraph, searchRadius=50)
    # this beta is from the sample_rate_sec=1 situation
    beta = 0.0970628347
    print("Using beta is:", beta)

    # If we use the distance as the cost metrics only, then set Wu to None, otherwise it is a hyper-parameter
    print("Start processing:")
    seq = computeViterbiSequence(gpsData, beta, roadNetwork, roadGraph, searchRadius=100, Wu=10)
    # Store the res
    res = []
    print("The predicted results is:")
    for i in range(len(seq)):
        if seq[i].transition_descriptor is not None:
            edges = seq[i].transition_descriptor
            for e in edges:
                res.append(e['edgeId'] + " " + str(e['waysLength']))

    f = open('./data/result_Sample_120_WU_10.txt', 'a')
    for i in res:
        f.write(i)
        f.write('\n')
    f.close()

