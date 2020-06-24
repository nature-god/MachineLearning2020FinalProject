from .HMMProbabilities import HmmProbabilities, computeEmissionProbabilities, computeTransitionProbabilities
from .TIViterbi import ViterbiAlgorithm
from timeStep import timeStep


# Calculate the most likely path state using Viterbi algorithm
def computeViterbiSequence(gpsData, beta, roadNetwork, roadGraph, searchRadius=100, Wu=200):
    seq = []
    h = HmmProbabilities(beta=beta)
    viterbi = ViterbiAlgorithm(keep_message_history=True)
    # last time step
    prevTimeStep = None

    for i, ob in enumerate(gpsData):
        print("Processing with the %dth gpsData" % i)
        candidates = roadNetwork.searchRoadEdge(ob, searchRadius)
        # No candidates
        if len(candidates) == 0:
            seq.extend(viterbi.compute_most_likely_sequence())
            viterbi = ViterbiAlgorithm(keep_message_history=True)
            prevTimeStep = None
            print("No candidates for the %dth gpsData" % i)

        else:
            try:
                # compute the emission probabilities and transition probabilities
                curTimeStep = timeStep(ob, candidates)
                computeEmissionProbabilities(curTimeStep, h)
            except ValueError:
                print("Skip the %dth gpsData because of some crashes" % i)
                continue
            if prevTimeStep is None:
                viterbi.start_with_initial_observation(ob, candidates,
                                                       curTimeStep.emissionLogProbabilities)
            else:
                computeTransitionProbabilities(prevTimeStep, curTimeStep, h, roadGraph, Wu)
                if curTimeStep.transitionLogProbabilities:
                    viterbi.next_step(ob, candidates, curTimeStep.emissionLogProbabilities,
                                      curTimeStep.transitionLogProbabilities,
                                      curTimeStep.roadPaths)
            # If it's not the first time step and it has no transition probabilities
            # then it will trigger a break
            noTrans = (prevTimeStep is not None) and (not curTimeStep.transitionLogProbabilities)
            if viterbi.is_broken or noTrans:
                print("Broken at the %dth gpsData and restart" % i)
                # construct the sequence ended at t-1, and start a new matching at t (no transition error)
                seq.extend(viterbi.compute_most_likely_sequence())
                viterbi = ViterbiAlgorithm(keep_message_history=True)
                viterbi.start_with_initial_observation(ob, candidates,
                                                       curTimeStep.emissionLogProbabilities)
            prevTimeStep = curTimeStep
    if len(seq) < len(gpsData):
        seq.extend(viterbi.compute_most_likely_sequence())
    return seq