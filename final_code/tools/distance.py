import math
# from geopy.distance import great_circle


# Returns the great circle distance [m] between two GPS points
# def greatCircleDistance(posA, posB):
#     return geopy.distance.great_circle(posA, posB).meters
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
    s = s * EARTH_RADIUS * 1000
    return s


# convert distance difference [m] to longitude difference [°]
def distanceToLongitude(latitude, radius):
    return radius / (math.cos((latitude * math.pi) / 180.0) * 111318.078)


# convert distance difference [m] to latitude difference [°]
def distanceToLatitude(radius):
    return radius / 111319.5


# Returns the shortest great circle distance [m] from GPS point p1 to line segment
# determined by GPS point p2 and GPS point p3.
def computeDistance(p1, p2, p3):
    a = greatCircleDistance(p2, p3)
    b = greatCircleDistance(p1, p2)
    c = greatCircleDistance(p1, p3)
    cosAlpha = (a * a + b * b - c * c) / (2 * a * b)
    cosBeta = (a * a + c * c - b * b) / (2 * a * c)
    if cosAlpha <= 0:
        return b
    if cosBeta <= 0:
        return c
    return math.sqrt((a + b + c) * (a + b - c) * (a + c - b) * (b + c - a)) / (2 * a)


# Returns the foot point on the line segment determined by pointA and pointB
def findFootPoint(point, pointA, pointB):
    a = greatCircleDistance(pointA, pointB)
    b = greatCircleDistance(point, pointA)
    c = greatCircleDistance(point, pointB)
    cosAlpha = (a * a + b * b - c * c) / (2 * a * b)
    cosBeta = (a * a + c * c - b * b) / (2 * a * c)
    if 0 <= cosAlpha and 0 <= cosBeta:
        h = computeDistance(point, pointA, pointB)
        t = math.sqrt(b * b - h * h)
        xA = pointA[0]
        yA = pointA[1]
        xB = pointB[0]
        yB = pointB[1]
        x = xA + (xB - xA) / a * t
        y = yA + (yB - yA) / a * t
        return (x, y)
    else:
        # Not find a foot point, which means the foot point is not on this line segment
        return None


# find the included angle between edge(p1, p2) and edge(p2, p3)
# then convert it into uvv': |θvv′|<π/4 is 0, π/4≤|θvv′|≤3π/4 is 1, 3π/4<|θvv′| is 2
def convertIncludedAngle(pointA, pointB, pointC):
    a = greatCircleDistance(pointA, pointB)
    b = greatCircleDistance(pointB, pointC)
    c = greatCircleDistance(pointA, pointC)

    if a == 0.0 or b == 0.0 or c == 0.0:
        return 0
    cosAlpha = (a * a + b * b - c * c) / (2 * a * b)
    if cosAlpha > 1:
        return 0
    elif cosAlpha < -1:
        return 2
    angle = abs(math.acos(cosAlpha))
    if angle < math.pi * 0.25:
        return 0
    elif angle >= math.pi * 0.25 and angle <= math.pi * 0.75:
        return 1
    elif angle > math.pi * 0.75:
        return 2