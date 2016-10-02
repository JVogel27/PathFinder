import os

__author__ = 'Jesse Vogel'
__date__ = 'Sep 13th, 2016'

from PIL import Image
from queue import PriorityQueue
from math import sqrt, atan2, pi
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
			resultsText = open("directions.txt", 'w')
			buffer = []
			order = []
			for control in controls:
				order.append(controls.index(control))
				time, parents = findPath(pixelMap, elevationMap, start, control)  # return parents
				generatePath(resultsMap, control, parents)
				buffer = createDirections(pixelMap, controls, control, parents, buffer)
				totalTime += time
				start = control
			resultsText.writelines(buffer)
			resultsText.write("\nvisited controls: " + str(order))
			resultsText.write("\ntime: %.0f minutes" % (totalTime/60))
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
			time = result[0]
			p = result[1]
			try:
				os.remove("directions.txt")
			except OSError:
				pass
			resultsText = open("directions.txt", 'w')
			resultsMap = Image.open("terrain.png")
			controls.insert(0, startFinish)
			buffer = []
			order = []
			for i in range(len(p)-1, 0, -1):
				order.insert(0, controls.index(p[i]))
				generatePath(resultsMap, p[i], graph[p[i-1]][p[i]][1])
				buffer = createDirections(pixelMap, controls, p[i], graph[p[i-1]][p[i]][1], buffer)
			buffer.reverse()
			resultsText.writelines(buffer)
			resultsText.write("\nvisited controls: " + str(order[:-1]))
			resultsText.write("\ntime: %.0f minutes" % (time/60))


def scoreo(graph, controls, allowedTime, startFinish):
	numControls = len(controls)
	while numControls >= 0:
		best = None
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
				if best:
					if time < best[0]:
						best = (time, p, visited)
				else:
					best = (time, p, visited)
		if best:
			return best
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
	image.save("map.png")


def onPathOrRoad(pixelMap, x, y):
	return pixelMap[x, y] == (0, 0, 0, 255) or pixelMap[x, y] == (71, 51, 3, 255)


def angle(p1, p2):
	dx = p2[0] - p1[0]
	dy = (p2[1] - p1[1]) * -1
	angle = atan2(1, 0) - atan2(dy, dx)
	angle = angle * 360 / (2*pi)
	if angle < 0:
		angle += 360
	return angle


def createDirections(pixelMap, controls, state, parents, buffer):
	if controls.index(state) != 0:
		buffer.append("at control " + str(controls.index(state)) + "\n")
	end = state  # start a new path
	current = parents[end]  # take a step backwards
	while True:
		endOnPath = onPathOrRoad(pixelMap, end[0], end[1])
		currentOnPath = onPathOrRoad(pixelMap, current[0], current[1])
		if (endOnPath and not currentOnPath) or (not endOnPath and currentOnPath) or parents[current] is None:
			# time to end the path!
			dx = end[0]-current[0]  # calculate distance and angle
			dy = end[1]-current[1]
			dist = sqrt((dx*dx)+(dy*dy))
			a = angle(current, end)
			if endOnPath:
				buffer.append("follow path at angle %.0f for %.0fm\n" % (a, dist))  # write to buffer
			else:
				buffer.append("go at angle %.0f for %.0fm\n" % (a, dist))
			end = current  # start a new path
			if parents[current] is None:  # our step backwards landed on the control
				break
		current = parents[current]  # take a step backwards
	return buffer



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
				t = (costFunction(pixelMap, elevationMap, timeToNode, state, next, heading, goal), next)
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
	grade = (rise/run)*100
	#dist = sqrt((run*run)+(rise*rise))  # distance traveled taking into account the elevation change
	dist = run
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
	if currSpeed == 0 or nextSpeed == 0:
		time = float("inf")
	else:
		time = ((dist/2)/currSpeed) + ((dist/2)/nextSpeed)  # moving from the center of one pixel to the center of the next
	if grade >= 0:
		time += (dist*grade*0.0080529707)
	else:
		time -= (dist*grade*0.004970964566929)
	timeToNode[next] = timeToNode[curr] + time
	return timeToNode[next] + heuristic(next, goal)


def heuristic(next, goal):
	D = 1.8875
	D2 = 3.19
	dx = abs(next[0] - goal[0])
	dy = abs(next[1] - goal[1])
	#return 1.14 * (dx + dy) + (1.93 - 2 * 1.14) * min(dx, dy)
	return D * (dx + dy) + (D2 - 2 * D) * min(dx, dy)


def loadpixelmap(filename):
	return Image.open(filename).load()


def loadelevationmap(filename):
	with open('mpp.txt') as mapFile:
		return [[float(num) for num in line.split()] for line in mapFile]


def addControls():
	image = Image.open("map.png")
	resultsMap = image.load()
	with open('one.txt') as inputFile:
		lineOne = inputFile.readline()
		lineOne = inputFile.readline()
		for line in inputFile:
			array = line.split()
			resultsMap[int(array[0]), int(array[1])] = (0, 255, 255, 255)
		image.save("map.png")

main()
addControls()