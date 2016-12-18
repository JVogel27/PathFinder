__author__ = 'Jesse Vogel'
__date__ = 'Sep 13th, 2016'

import os
import sys
from PIL import Image
from queue import PriorityQueue
from math import sqrt, atan2, pi
from itertools import permutations


def main():
	pixelMap = Image.open("terrain.png").load()  # assumed that these files are present in the same directory as source
	elevationMap = loadelevationmap("mpp.txt")

	filename = sys.argv[1]  # read file name containing event data from arguments
	with open(filename) as pathFile:
		if "Classic" in pathFile.readline():
			secondLine = pathFile.readline().split()
			init = (int(secondLine[0]), int(secondLine[1]))  # hold onto starting location
			controls = []
			for line in pathFile:  # parse file and store control points in controls list
				lineArray = line.split()
				controls.append((int(lineArray[0]), int(lineArray[1])))
			start = init
			totalTime = 0
			resultsMap = Image.open("terrain.png")  # draw path onto this file
			resultsText = open("directions.txt", 'w')  # generate plain english directions in this file
			buffer = []  # accumulate directions as they are generated
			for control in controls:
				time, parents = findPath(pixelMap, elevationMap, start, control)  # find best path between two control points
				generatePath(resultsMap, control, parents)  # draw path onto map
				buffer = createDirections(pixelMap, controls, control, parents, buffer)  # generate directions for path
				totalTime += time
				start = control
			resultsText.writelines(buffer)  # save directions to file
			resultsText.write("\ntime: %.0f minutes" % (totalTime/60))  # save time to file
		else:  # score-o event
			allowedTime = int(pathFile.readline().strip())
			array = pathFile.readline().split()
			startFinish = (int(array[0]), int(array[1]))  # start and end at this point
			controls = []
			for line in pathFile:
				lineArray = line.split()
				controls.append((int(lineArray[0]), int(lineArray[1])))
			c = controls[:]  # copy controls - will be modified in createGraph
			graph = createGraph(startFinish, c, pixelMap, elevationMap)  # initialize graph representation of best paths
			time, p, visited = scoreo(graph, controls, allowedTime, startFinish)  # use graph to generate best path and time
			try:
				os.remove("directions.txt")  # when running program multiple times, we want a clean file every time
			except OSError:
				pass
			resultsText = open("directions.txt", 'w')
			resultsMap = Image.open("terrain.png")
			controls.insert(0, startFinish)
			buffer = []
			order = []  # keep track of the order in which we visited controls
			for i in range(len(p)-1, 0, -1):  # iterate backwards
				order.insert(0, controls.index(p[i]))
				generatePath(resultsMap, p[i], graph[p[i-1]][p[i]][1])  # draw path onto the map
				buffer = createDirections(pixelMap, controls, p[i], graph[p[i-1]][p[i]][1], buffer)  # create text output
			buffer.reverse()  # needed so the directions are in the right order!
			resultsText.writelines(buffer)
			resultsText.write("\nvisited controls: " + str(order[:-1]))  # decided not to add startFinish to this list
			resultsText.write("\ntime: %.0f minutes" % (time/60))


# use the weighted graph to find the best path through as many control points as possible within the allowed time
def scoreo(graph, controls, allowedTime, startFinish):
	numControls = len(controls)
	while numControls >= 0:  # reverse iterative deepening, iterative shalowing??
		best = None  # keep track of the shortest path
		perm = permutations(controls, numControls)  # don't worry, we shouldn't have to try them all
		for p in perm:
			p = list(p)
			p.insert(0, startFinish)  # add startFinish to the beginning and end of the list
			p.insert(len(p), startFinish)
			time = 0
			visited = 1
			for i in range(1, len(p)):
				time += graph[p[i-1]][p[i]][0]  # sum the pre-computed best times
				visited += 1
			if time > allowedTime:  # optimize by only considering valid paths
					continue
			if visited == len(p):  # all controls reached
				if best:
					if time < best[0]:
						best = (time, p, visited)  # save the path if the time is better than the best seen so far
				else:
					best = (time, p, visited)
		if best:  # stop the whole search once we have a best path that visits every control in the permutation
			return best
		numControls -= 1


# create a complete graph with the controls as vertices. Edges have weights equal to the shortest path between controls
def createGraph(startFinish, controls, pixelMap, elevationMap):
	graph = {}  # adjacency list representation
	controls.insert(0, startFinish)
	for control in controls:
		neighbors = {}
		for neighbor in controls:
			if control[0] != neighbor[0] or control[1] != neighbor[1]:  # controls are not connected to themselves
				time, parents = findPath(pixelMap, elevationMap, control, neighbor)  # find shortest path
				neighbors[neighbor] = (time, parents)  # hold onto the parents array as well. Used to draw the path
		graph[control] = neighbors
	return graph

# draw red line on the map
def generatePath(image, state, parents):
	pathMap = image.load()
	while True:
		pathMap[state[0], state[1]] = (255, 0, 0, 255)
		if parents[state] is None:
			break
		state = parents[state]
	image.save("map.png")


# return true if the point is on a road or path, false otherwise
def onPathOrRoad(pixelMap, x, y):
	return pixelMap[x, y] == (0, 0, 0, 255) or pixelMap[x, y] == (71, 51, 3, 255)


# find the angle between the two given points (North: 0, East: 90, South: 180, West: 270)
def angle(p1, p2):
	dx = p2[0] - p1[0]
	dy = (p2[1] - p1[1]) * -1   # reverse y to match image coordinate system!
	angle = atan2(1, 0) - atan2(dy, dx)  # yay math...
	angle = angle * 360 / (2*pi)
	if angle < 0:
		angle += 360
	return angle


# function to generate the textual directions based on the best path
def createDirections(pixelMap, controls, state, parents, buffer):
	if controls.index(state) != 0:
		buffer.append("at control " + str(controls.index(state)) + "\n")
	end = state  # start a new path
	current = parents[end]  # take a step backwards
	while True:
		endOnPath = onPathOrRoad(pixelMap, end[0], end[1])
		currentOnPath = onPathOrRoad(pixelMap, current[0], current[1])
		if (endOnPath and not currentOnPath) or (not endOnPath and currentOnPath) or parents[current] is None:
			# end the path
			dx = end[0]-current[0]  # calculate distance and angle
			dy = end[1]-current[1]
			dist = sqrt((dx*dx)+(dy*dy))
			a = angle(current, end)
			if endOnPath:
				buffer.append("follow path at angle %.0f for %.0fm\n" % (a, dist))  # write to buffer
			else:
				buffer.append("go at angle %.0f for %.0fm\n" % (a, dist))
			end = current  # start a new path
			if parents[current] is None:  # our step backwards landed on the other control
				break
		current = parents[current]  # take a step backwards
	return buffer


# find the shortest path from one control to another
def findPath(pixelMap, elevationMap, init, goal):
	timeToNode = {}
	timeToNode[init] = 0
	parents = {init: None}
	PQ = PriorityQueue(maxsize=0)
	PQ.put((0, init))
	while not PQ.empty():  # A* algorithm from lecture
		state = PQ.get()[1]
		if isgoal(state, goal):
			return timeToNode[goal], parents
		for si in successors(state):
			if (si[0], si[1]) not in parents:
				next = (int(si[0]), int(si[1]))
				heading = si[2]
				t = (costFunction(pixelMap, elevationMap, timeToNode, state, next, heading, goal), next)  # heuristic
				PQ.put(t)
				parents[(si[0], si[1])] = (state[0], state[1])


# check whether a state is a goal state
def isgoal(state, goal):
	return state[0] == goal[0] and state[1] == goal[1]


# generate valid successors for the given state
def successors(state):
	x = int(state[0])
	y = int(state[1])
	successors = []                     # heading is necessary to keep as it will determine the distance traveled
	if x > 0:                           # west
		successors.append((x-1, y, 'w'))
	if x < 399:                         # east
		successors.append((x+1, y, 'e'))
	if y > 0:                           # north
		successors.append((x, y-1, 'n'))
	if y < 499:                         # south
		successors.append((x, y+1, 's'))
	if x > 0 and y > 0:                 # north west
		successors.append((x-1, y-1, 'nw'))
	if x < 399 and y > 0:               # north east
		successors.append((x+1, y-1, 'ne'))
	if x > 0 and y < 499:               # south west
		successors.append((x-1, y+1, 'sw'))
	if x < 399 and y < 499:             # south east
		successors.append((x+1, y+1, 'se'))
	return successors


# determine the estimated cost f() = g() + h()
def costFunction(pixelMap, elevationMap, timeToNode, curr, next, heading, goal):
	currTerrain = pixelMap[curr[0], curr[1]]
	currElevation = elevationMap[curr[1]][curr[0]-5]  # offset elevation by 5 to repair the snafu!
	nextTerrain = pixelMap[next[0], next[1]]
	nextElevation = elevationMap[curr[1]][next[0]-5]
	rise = nextElevation - currElevation

	# determine which direction we're going
	if heading == 'n' or heading == 's':
		run = 7.55     # moving up
	elif heading == 'e' or heading == 'w':
		run = 10.29    # moving down
	else:
		run = 12.76    # moving diagonal
	grade = (rise/run)*100  # how steep it is
	dist = run
	# speeds in m/s based on personal evaluation of how fast I can move on a given terrain
	if currTerrain == (5, 73, 24, 255) or currTerrain == (0, 0, 255, 255) or currTerrain == (205, 0, 101, 255):
		currSpeed = 0
	elif currTerrain == (255, 192, 0, 255):
		currSpeed = 2.0
	elif currTerrain == (255, 255, 255, 255):
		currSpeed = 2.5
	elif currTerrain == (2, 208, 60):
		currSpeed = 2.0
	elif currTerrain == (2, 136, 40, 255):
		currSpeed = 1.0
	elif currTerrain == (0, 0, 0, 255):
		currSpeed = 3.0
	elif currTerrain == (71, 51, 3, 255):
		currSpeed = 4.0
	else:
		currSpeed = 3.0  # open land
	if nextTerrain == (5, 73, 24, 255) or nextTerrain == (0, 0, 255, 255) or nextTerrain == (205, 0, 101, 255):
		nextSpeed = 0
	elif nextTerrain == (255, 192, 0, 255):
		nextSpeed = 2.0
	elif nextTerrain == (255, 255, 255, 255):
		nextSpeed = 2.5
	elif nextTerrain == (2, 208, 60):
		nextSpeed = 2.0
	elif nextTerrain == (2, 136, 40, 255):
		nextSpeed = 1.0
	elif nextTerrain == (0, 0, 0, 255):
		nextSpeed = 3.0
	elif nextTerrain == (71, 51, 3, 255):
		nextSpeed = 4.0
	else:
		nextSpeed = 3.0
	if currSpeed == 0 or nextSpeed == 0:  # we're in the water or something bad
		time = float("inf")
	else:
		time = ((dist/2)/currSpeed) + ((dist/2)/nextSpeed)  # moving from the center of one pixel to the center of the next
	if grade >= 0:  # going uphill
		time += (dist*grade*0.0080529707)  # see write up
	else:  # going downhill
		time -= (dist*grade*0.004970964566929)
	timeToNode[next] = timeToNode[curr] + time
	return timeToNode[next] + heuristic(next, goal)


# estimate cost from state to goal
def heuristic(next, goal):
	D = 1.8875  # best case cost for 4 way directional
	D2 = 3.19  # best case cost for diagonal
	dx = abs(next[0] - goal[0])
	dy = abs(next[1] - goal[1])
	return D * (dx + dy) + (D2 - 2 * D) * min(dx, dy)  # diagonal distance heuristic equation


# utility function
def loadelevationmap(filename):
	with open(filename) as mapFile:
		return [[float(num) for num in line.split()] for line in mapFile]

main()