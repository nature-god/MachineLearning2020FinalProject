import math


# Normal distribution
def normalDistribution(sigma, x):
    return 1.0 / (math.sqrt(2.0 * math.pi) * sigma) * math.exp(-0.5 * math.pow(x / sigma, 2))


# Log normal distribution
def logNormalDistribution(sigma, x):
    return math.log(1.0 / (math.sqrt(2.0 * math.pi) * sigma)) + (-0.5 * math.pow(x / sigma, 2))


# Exponential distribution
def exponentialDistribution(beta, x):
    return 1.0 / beta * math.exp(-x / beta)


# Log exponential distribution
def logExponentialDistribution(beta, x):
    return math.log(1.0 / beta) - (x / beta)