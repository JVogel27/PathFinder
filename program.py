__author__ = 'Jesse Vogel'
__date__ = 'Sep 13th, 2016'

###
# Problem Environment
# State space: dynamic - (x,y) pixel location, static - map data (terrain, elevation)
# Init: starting point (x, y)
# Goal: last point in sequence (w/ all other points visited in order)
# Actions: move to neighboring pixel (increment/decrement x and/or y by 1)
#
# Heuristic Planning
# - Average jogging speed: 1.7 - 2.7 m/s
# - Average walking speed: 1.4 m/s
#
# Link about calculating velocity for graded paths
# http://www.letsrun.com/forum/flat_read.php?thread=197366
#
# Good link about different heuristics
# http://theory.stanford.edu/~amitp/GameProgramming/Heuristics.html
###

from PIL import Image
from queue import PriorityQueue
from math import sqrt
from itertools import permutations


def main():
	pixelMap = loadpixelmap("terrain.png")
	elevationMap = loadelevationmap("mpp.txt")

	with open('one.txt') as pathFile:
		if "Classic" in pathFile.readline():
			secondLine = pathFile.readline().split()
			init = (int(secondLine[0]), int(secondLine[1]))
			controls = []
			for line in pathFile:
				lineArray = line.split()
				controls.append((int(lineArray[0]), int(lineArray[1])))
			start = init
			totalTime = 0
			resultsMap = Image.open("terrain.png")
			for control in controls:
				time, parents = findPath(pixelMap, elevationMap, start, control)  # return parents
				generatePath(resultsMap, control, parents)
				totalTime += time
				start = control
			print("total time (minutes): ", totalTime/60)
		else:
			allowedTime = int(pathFile.readline().strip())
			array = pathFile.readline().split()
			startFinish = (int(array[0]), int(array[1]))

			controls = []
			for line in pathFile:
				lineArray = line.split()
				controls.append((int(lineArray[0]), int(lineArray[1])))
			c = controls[:]
			graph = createGraph(startFinish, c, pixelMap, elevationMap)   # initialize graph

			result = scoreo(graph, controls, allowedTime, startFinish)


def scoreo(graph, controls, allowedTime, startFinish):
	numControls = len(controls)
	while numControls >= 0:
		perm = permutations(controls, numControls)
		for p in perm:
			p = list(p)
			p.insert(0, startFinish)
			p.insert(len(p), startFinish)
			time = 0
			visited = 1
			for i in range(1, len(p)):
				time += graph[p[i-1]][p[i]][0]
				visited += 1
			if time > allowedTime:
					continue
			if visited == len(p):
				print("visited: %d controls" % (visited-1))
				print("time: %.0f minutes" % (time/60))
				resultsMap = Image.open("terrain.png")
				for i in range(1, len(p)):
					generatePath(resultsMap, p[i], graph[p[i-1]][p[i]][1])
				return 0
		numControls -= 1


def createGraph(startFinish, controls, pixelMap, elevationMap):
	graph = {}
	controls.insert(0, startFinish)
	for control in controls:
		neighbors = {}
		for neighbor in controls:
			if control[0] != neighbor[0] or control[1] != neighbor[1]:
				time, parents = findPath(pixelMap, elevationMap, control, neighbor)
				neighbors[neighbor] = (time, parents)
		graph[control] = neighbors
	return graph


def generatePath(image, state, parents):
	pathMap = image.load()
	while True:
		pathMap[state[0], state[1]] = (255, 0, 0, 255)
		if parents[state] is None:
			break
		state = parents[state]
	image.save("results.png")


def findPath(pixelMap, elevationMap, init, goal):
	timeToNode = {}
	timeToNode[init] = 0
	parents = {init: None}
	PQ = PriorityQueue(maxsize=0)
	PQ.put((0, init))
	while not PQ.empty():
		state = PQ.get()[1]
		if isgoal(state, goal):
			return timeToNode[goal], parents
		for si in successors(state):
			if (si[0], si[1]) not in parents:
				next = (int(si[0]), int(si[1]))
				heading = si[2]
				t = (costFunction(pixelMap, elevationMap, timeToNode, state, next, heading, goal), (int(si[0]), int(si[1])))
				PQ.put(t)
				parents[(si[0], si[1])] = (state[0], state[1])


# check whether a state is a goal state
def isgoal(state, goal):
	return state[0] == goal[0] and state[1] == goal[1]


# generate valid successors for the given state
def successors(state):
	x = int(state[0])
	y = int(state[1])
	successors = []                     # heading is necessary to keep as it will effect the distance traveled
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


def costFunction(pixelMap, elevationMap, timeToNode, curr, next, heading, goal):
	currTerrain = pixelMap[curr[0], curr[1]]
	currElevation = elevationMap[curr[1]][curr[0]-5]  # offset elevation by 5
	nextTerrain = pixelMap[next[0], next[1]]
	nextElevation = elevationMap[curr[1]][next[0]-5]
	rise = nextElevation - currElevation

	# computer cost from curr to next
	if heading == 'n' or heading == 's':
		run = 7.55     # moving up
	elif heading == 'e' or heading == 'w':
		run = 10.29    # moving down
	else:
		run = 12.76    # moving diagonal

	dist = sqrt((run*run)+(rise*rise))  # distance traveled taking into account the elevation change

	if currTerrain == (5, 73, 24, 255) or currTerrain == (0, 0, 255, 255) or currTerrain == (205, 0, 101, 255):
		currSpeed = 0
	elif currTerrain == (255, 192, 0, 255):
		currSpeed = 2.0
	elif currTerrain == (255, 255, 255, 255):
		currSpeed = 2.7
	elif currTerrain == (2, 208, 60):
		currSpeed = 2.0
	elif currTerrain == (2, 136, 40, 255):
		currSpeed = 1.0
	elif currTerrain == (0, 0, 0, 255):
		currSpeed = 2.5
	elif currTerrain == (71, 51, 3, 255):
		currSpeed = 4.0
	else:
		currSpeed = 3.0
	if nextTerrain == (5, 73, 24, 255) or currTerrain == (0, 0, 255, 255) or currTerrain == (205, 0, 101, 255):
		nextSpeed = 0
	elif nextTerrain == (255, 192, 0, 255):
		nextSpeed = 2.0
	elif nextTerrain == (255, 255, 255, 255):
		nextSpeed = 2.7
	elif nextTerrain == (2, 208, 60):
		nextSpeed = 2.0
	elif nextTerrain == (2, 136, 40, 255):
		nextSpeed = 1.0
	elif nextTerrain == (0, 0, 0, 255):
		nextSpeed = 2.5
	elif nextTerrain == (71, 51, 3, 255):
		nextSpeed = 4.0
	else:
		nextSpeed = 3.0

	if currSpeed == 0 or nextSpeed == 0:
		time = float("inf")
	else:
		time = ((dist/2)/currSpeed) + ((dist/2)/nextSpeed)  # this method assumes moving from the center of one pixel to
	timeToNode[next] = timeToNode[curr] + time              # the center of the next pixel
	return timeToNode[next] + heuristic(next, goal)


def heuristic(next, goal):
	dx = abs(next[0] - goal[0])
	dy = abs(next[1] - goal[1])
	return 1.14 * (dx + dy) + (1.93 - 2 * 1.14) * min(dx, dy)


def loadpixelmap(filename):
	return Image.open(filename).load()


def loadelevationmap(filename):
	with open('mpp.txt') as mapFile:
		return [[float(num) for num in line.split()] for line in mapFile]


def addControls():
	image = Image.open("results.png")
	resultsMap = image.load()
	with open('one.txt') as inputFile:
		lineOne = inputFile.readline()
		lineOne = inputFile.readline()
		for line in inputFile:
			array = line.split()
			resultsMap[int(array[0]), int(array[1])] = (0, 255, 255, 255)
		image.save("results.png")

main()
addControls()