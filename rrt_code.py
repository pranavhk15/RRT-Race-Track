import numpy as np, time
from shapely.geometry import LineString, Polygon, Point

MAX_ITER = 100000
MAX_DIST = 15
OBS_WIDTH = 5
def planner(blue_cones, yellow_cones):
	"""plans a path through the track given the blue and yellow cones.

	Args:
		blue_cones (list): blue (left) cone locations. shape (n, 2).
		yelllow_cones (list): yellow (right) cone locations. shape (n, 2).

	Returns:
		list: output path, with shape (n, 2)
	"""

	def newPoint(botLeft, topRight):
		x = np.random.uniform(botLeft[0], topRight[0])
		y = np.random.uniform(botLeft[1], topRight[1])
		return np.array([x,y])

	def closest_idx(target, nodes):
		return np.argmin(np.linalg.norm(nodes - target, axis = 1))

	def determineNew(old, new, maxDist):
		delta = new - old
		dist = np.linalg.norm(delta)
		if dist > maxDist:
			delta = delta / dist * maxDist
		return old + delta

	def noCollision(old, new, obstacles):
		line = LineString([old, new])
		for o in obstacles:
			if line.intersects(o):
				return False
		return True
	
	# def nodeAngle(new, idx):
	# 	prev = nodes[parents[idx]]
	# 	curr = nodes[idx]

	# 	AB = np.array(curr - prev)
	# 	BC = np.array(new - curr)
	# 	cosine_angle = np.dot(AB, BC) / (np.linalg.norm(AB) * np.linalg.norm(BC))
	# 	radian_angle = np.arccos(cosine_angle)
	# 	degrees_angle = np.degrees(radian_angle)
	# 	return degrees_angle


	start = time.time()


	bottomLeft = [min(i[0] for i in blue_cones + yellow_cones), min(i[1] for i in blue_cones + yellow_cones)]
	topRight = [max(i[0] for i in blue_cones + yellow_cones), max(i[1] for i in blue_cones + yellow_cones)]

	rightObstacles = [LineString([blue_cones[i], blue_cones[i+1]]).buffer(OBS_WIDTH) for i in range(len(blue_cones) - 1)]
	leftObstacles = [LineString([yellow_cones[i], yellow_cones[i+1]]).buffer(OBS_WIDTH) for i in range(len(yellow_cones) - 1)]
	

	lastBlue = blue_cones[-2]
	firstBlue = blue_cones[-1]
	lastYellow = yellow_cones[-2]
	firstYellow = yellow_cones[-1]

	barrierBlue = lastBlue - firstBlue
	barrierBlue = barrierBlue * 0.03
	barrierBlue = barrierBlue + firstBlue

	barrierYellow = lastYellow - firstYellow
	barrierYellow = barrierYellow * 0.03
	barrierYellow = barrierYellow + firstYellow

	barrier = [LineString([barrierBlue, barrierYellow])]

	obstacles = leftObstacles + rightObstacles + barrier

	outerPoly = Polygon(yellow_cones)
	innerPoly = Polygon(blue_cones)
	allowedSpace = outerPoly.difference(innerPoly)


	firstBlue = blue_cones[-1]
	firstYellow = yellow_cones[-1]
	startNode = np.array([(firstBlue[0] + firstYellow[0]) / 2, (firstBlue[1] + firstYellow[1]) / 2])

	nodes = [startNode]
	parents = [-1]
	costs = [0]
	

	for k in range(MAX_ITER):
		rand = newPoint(bottomLeft, topRight)
		closest = closest_idx(rand, nodes)
		closestNode = nodes[closest]
		newNode = determineNew(closestNode, rand, MAX_DIST)

		if allowedSpace.contains(Point(newNode)) and noCollision(closestNode, newNode, obstacles):
			nodes.append(newNode)
			parents.append(closest)
			cost = costs[closest] + np.linalg.norm(newNode - closestNode)
			costs.append(cost)

			distances = np.linalg.norm(nodes - newNode, axis=1)
			nearNodes = np.argwhere(distances < 2.5 * MAX_DIST)
			nearNodes = [i[0] for i in nearNodes]
			bestIdx = closest
			for nearIdx in nearNodes:
				near = nodes[nearIdx]
				currCost = costs[nearIdx] + np.linalg.norm(near - newNode)
				if currCost < cost and noCollision(newNode, near, obstacles):
					cost = currCost
					bestIdx = nearIdx
			costs[-1] = cost
			parents[-1] = bestIdx

		
			if (not noCollision(newNode, startNode, barrier) and np.linalg.norm(startNode - newNode) <= 2 * MAX_DIST 
				and noCollision(newNode, startNode, leftObstacles + rightObstacles)):
				
				i = len(nodes) - 1
				path = [nodes[i]]
				pathcost = [costs[i]]
				
				while parents[i] != -1:
					i = parents[i]
					path.append(nodes[i])
					pathcost.append(costs[i])
				pathcost = pathcost[::-1]
				end = time.time()
				print(f'{k} iterations |', f'{int(pathcost[-1])} cost |', f'{int(end - start)} seconds')
				return path[::-1] + [startNode]

			



	return nodes
