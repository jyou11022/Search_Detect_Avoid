import math, sys, random
from math import *

#Conversion factors for degrees of latitude and longitude depend
#on the location of said coordinates. For further information as
#to how the ratios are defined, see 
#https://gis.stackexchange.com/questions/75528/understanding-terms-in-length-of-degree-formula .
#The conversion functions take and return a latitude and longitude
#in degrees, although internally it is converted to radians for
#compatability with Python's math.cos function. Conversion to meters
#uses the supplied lat or long as the origin
def degreesLatToMeters(degrees, lat):
	latR = math.radians(lat)
	return (degrees - lat) * (111132.92 - (559.82 * math.cos(2 * latR)) +
	(1.175 * math.cos(4 * latR)) - (0.0023 * math.cos(6 * latR)))

def degreesLongToMeters(degrees, lng):
	lngR = math.radians(lng)
	return (degrees - lng) * ((111412.84 * math.cos(lngR)) -
	(93.5 * math.cos(3 * lngR)) + (0.118 * math.cos(5 * lngR)))

def metersToDegreesLat(meters, lat):
	latR = math.radians(lat)
	return (meters / (111132.92 - (559.82 * math.cos(2 * latR)) +
	(1.175 * math.cos(4 * latR)) - (0.0023 * math.cos(6 * latR)))) + lat

def metersToDegreesLong(meters, lng):
	lngR = math.radians(lng)
	return (meters / ((111412.84 * math.cos(lngR)) -
	(93.5 * math.cos(3 * lngR)) + (0.118 * math.cos(5 * lngR)))) + lng

class Node(object):
    def __init__(self, point, parent):
        super(Node, self).__init__()
        self.point = point
        self.parent = parent

#A bounding area, in meters
XDIM = 1500
YDIM = 500
delta = 30
NUMNODES = 50000

#Distance between two points
def cost(p1,p2):
    return sqrt((p1[0]-p2[0])*(p1[0]-p2[0])+(p1[1]-p2[1])*(p1[1]-p2[1]))

#Checks if a point is within a circle
def point_circle_collision(p1, p2, radius):
    distance = cost(p1,p2)
    if (distance <= radius):
        return True
    return False

#If p2 is greater than delta distance from p1, return a point in the 
#direction of p2 that is delta away from p1, else return p2
def steer(p1,p2):
    if cost(p1,p2) < delta:
        return p2
    else:
        return ((p1[0] + ((p2[0] - p1[0])*delta)/cost(p1,p2)), (p1[1] + ((p2[1] - p1[1])*delta)/cost(p1,p2)))

#check if point collides with the obstacle
def collides(p1, obsts):
    for obst in obsts:
        if point_circle_collision(p1, (obst[0], obst[1]), obst[2]):
            return True
    return False

#Returns a random point that does not collide with an obstacle
def get_random_clear(obsts):
    while True:
        p = (random.random()*XDIM) - (XDIM/2), (random.random()*YDIM) - (YDIM/2)
        if not collides(p, obsts):
            return p

#Takes start position and end position in lat, long, as well as the locations of obstacles
#given as [lat, long, radius] and returns path as list of nodes in lat, long
def rrt(startPos, endPos, obsts):    
    nodes = []
    count = 0

    #initialPoint = Node((50, 50), None)
    initialPoint = Node((degreesLatToMeters(startPos[0], startPos[0]), degreesLongToMeters(startPos[1], startPos[1])), None)
    nodes.append(initialPoint)
    #goalPoint = Node((400, 400), None)
    goalPoint = Node((degreesLatToMeters(endPos[0], startPos[0]), degreesLongToMeters(endPos[1], startPos[1])), None)          
    #convert obsts coordinates to meters
    for o in obsts:
        o[0] = metersToDegreesLat(o[0], startPos[0])
        o[1] = metersToDegreesLong(o[1], startPos[1])

    while True:
        #increment amount of nodes
        count = count + 1
        #check if the maximum amount of nodes has been exceded
        if count < NUMNODES:
            foundNext = False
            #finding a new node and adding it to the tree
            while not foundNext:
                rand = get_random_clear(obsts)
                parentNode = nodes[0]
                for p in nodes:
                    if cost(p.point,rand) <= cost(parentNode.point,rand):
                        newPoint = steer(p.point,rand)
                        if not collides(newPoint, obsts):
                            parentNode = p
                            foundNext = True

            newNode = steer(parentNode.point,rand)
            nodes.append(Node(newNode, parentNode))

            #Check if the point is within a distance delta of the goal area
            if point_circle_collision(newNode, goalPoint.point, delta):
                goalNode = nodes[len(nodes)-1]
                currNode = goalNode
                pathPoints = []
                while currNode.parent != None:
                    pathPoints = pathPoints + [(metersToDegreesLat(currNode.point[0], startPos[0]), metersToDegreesLong(currNode.point[1], startPos[1]))]
                    currNode = currNode.parent
                break

                
        else:
            break
    return pathPoints

def main():
    obsts = [[38.14784, -76.42839, 15]] #obstacles in the format [lat, long, radius (meters)]
    l = rrt((38.149223, -76.429489), (38.144676, -76.428108), obsts)
    f = open("test.txt", "w")
    for p in l:
        f.write(str(p[0]) + "," + str(p[1]) + "\n")
    f.close()
