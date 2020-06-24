# The time step for each gps data
class timeStep:
    def __init__(self, observation, candidates):
        if observation is None or candidates is None:
            raise ValueError('Observation and Candidates must not be null')
        self.observation = observation
        self.candidates = candidates
        self.emissionLogProbabilities = {}
        self.transitionLogProbabilities = {}
        self.roadPaths = {}

    def addEmissionLogProbability(self, candidate, emissionLogProbability):
        if candidate in self.emissionLogProbabilities:
            raise ValueError('Candidate has already been added in emissionLogProbability')
        self.emissionLogProbabilities[candidate] = emissionLogProbability

    def addTransitionLogProbability(self, fromPosition, toPosition, transitionLogProbability):
        transition = (fromPosition, toPosition)
        if transition in self.transitionLogProbabilities:
            raise ValueError('Transition has already been added in transitionLogProbability')
        self.transitionLogProbabilities[transition] = transitionLogProbability

    # In this method, because we use dijkstra algorithm to find all the node in the
    # shortest path, we need convert it into the edge list
    def addRoadPath(self, fromPosition, toPosition, roadPath, roadGraph):
        transition = (fromPosition, toPosition)
        if transition in self.roadPaths:
            raise ValueError('Transition has already been added in roadPath')
        def findEdges(a, b):
            if (a, b) in roadGraph.edges:
                return roadGraph.getEdgesValue(a, b)
            elif (b, a) in roadGraph.edges:
                return roadGraph.getEdgesValue(b, a)
            else:
                raise ValueError("Something wrong with edges")
        n = len(roadPath)
        if n == 2:
            self.roadPaths[transition] = [findEdges(roadPath[0], roadPath[1])]
        else:
            self.roadPaths[transition] = []
            edge1 = findEdges(roadPath[0].split('_')[0], roadPath[0].split('_')[1])
            edge2 = findEdges(roadPath[-1].split('_')[0], roadPath[-1].split('_')[1])
            for i in range(1, n - 1):
                if i == 1:
                    self.roadPaths[transition].append(edge1)
                else:
                    edge = findEdges(roadPath[i], roadPath[i - 1])
                    self.roadPaths[transition].append(edge)
            self.roadPaths[transition].append(edge2)

