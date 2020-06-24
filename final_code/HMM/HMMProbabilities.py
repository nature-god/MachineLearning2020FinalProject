from tools import logNormalDistribution, logExponentialDistribution
from tools import greatCircleDistance


# In contrast to Newson & Krumm the absolute distance difference is divided by the quadratic
# time difference to make the beta parameter of the exponential distribution independent of the
# sampling interval.
def normalizedTransitionMetric(routeLength, linearDistance, timeDiff):
    if timeDiff < 0.0:
        raise ValueError('Time difference between subsequent \
                        location measurements must be >= 0')
    return abs(linearDistance - routeLength) / (timeDiff * timeDiff)


# Sigma taken from Newson&Krumm.
# Beta empirically computed from the Microsoft ground truth data for shortest route
# lengths and 60 s sampling interval but also works for other sampling intervals.
class HmmProbabilities:
    def __init__(self, sigma=4.07, beta=0.00959442):
        self.sigma = sigma
        self.beta = beta

    # Returns the logarithmic emission probability density.
    # @param distance is Absolute distance [m] between GPS measurement and
    # map matching candidate.
    def emissionLogProbability(self, distance):
        return logNormalDistribution(self.sigma, distance)

    # Returns the logarithmic transition probability density for
    # the given transition parameters.
    # @param routeLength is the Length of the shortest route [m] between
    # two consecutive map matching candidates.
    # @param linearDistance is the Linear distance [m] between
    # two consecutive GPS measurements.
    # @param timeDiff is the time difference [s] between two consecutive GPS measurements.
    def transitionLogProbability(self, routeLength, linearDistance, timeDiff):
        transitionMetric = normalizedTransitionMetric(routeLength, linearDistance, timeDiff)
        return logExponentialDistribution(self.beta, transitionMetric)


# Compute the emission probabilities and add it to the timeStep
def computeEmissionProbabilities(timeStep, h):
    for candidate in timeStep.candidates:
        candidatePos = candidate['pos']
        gpsPos = timeStep.observation['pos']
        dist = greatCircleDistance(candidatePos, gpsPos)
        timeStep.addEmissionLogProbability(candidatePos, h.emissionLogProbability(dist))


# Compute the transition probabilities and add it to the timeStep
def computeTransitionProbabilities(prevTimeStep, timeStep, h, roadGraph, Wu):
    linearDist = greatCircleDistance(prevTimeStep.observation['pos'], timeStep.observation['pos'])
    timeDiff = timeStep.observation['time'] - prevTimeStep.observation['time']
    timeDiff = timeDiff.seconds
    for prevCandidate in prevTimeStep.candidates:
        for curCandidate in timeStep.candidates:
            routeDist, nodePath = roadGraph.computePathDistance(prevCandidate, curCandidate, Wu)
            # if the node path is None, we do nothing
            if nodePath is not None:
                timeStep.addRoadPath(prevCandidate['pos'], curCandidate['pos'], nodePath, roadGraph)
                timeStep.addTransitionLogProbability(prevCandidate['pos'], curCandidate['pos'],
                                                     h.transitionLogProbability(routeDist, linearDist, timeDiff))