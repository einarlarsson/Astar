from math import sqrt, pow
import heapq

#f(n) = g(n) + h(n), where

#g(n) the cost (so far) to reach the node
#h(n) estimated cost to get from the node to the goal
#f(n) estimated total cost of path through n to goal. It is implemented using priority queue by increasing f(n).

def euclidean_distance(p, q):
    return sqrt(pow(q[1] - p[1], 2) + (pow(q[0] - p[0], 2)))

def reconstruct_path(cameFrom, current):
    total_path = {current}
    while current in cameFrom.keys():
        current = cameFrom[current]
        total_path.add(current)
    return total_path


def find_neighbor(maze, current):
    neighbors = []
    # [y][x]
    neighbors.append((current[0], current[1] + 1))
    neighbors.append((current[0] - 1, current[1]))
    neighbors.append((current[0], current[1] - 1))
    neighbors.append((current[0] + 1, current[1]))
    result = []
    for n in neighbors:
        if n[0] < 0 or n[0] > 9 or n[1] < 0 or n[1] > 9:
            continue
        if maze[n[0]][n[1]] == 1:
            continue
        result.append(n)
    return result


def astar(maze, start, end):

    closedSet = []

    openSet = []

    cameFrom = {}

    gScore = {}

    heapq.heappush(openSet, (0, start))

    gScore[start] = 0;

    fScore = {}

    fScore[start] = euclidean_distance(start, end)

    while not openSet.count(0):
        current = heapq.heappop(openSet)[1]

        if current == end:
            return reconstruct_path(cameFrom, current)

        closedSet.append(current)

        for neighbor in find_neighbor(maze, current):
            if neighbor in closedSet:
                continue

            new_cost = gScore[current] + 1

            if neighbor in gScore:
                if new_cost >= gScore[neighbor]:
                    continue

            cameFrom[neighbor] = current
            gScore[neighbor] = new_cost
            fScore[neighbor] = gScore[neighbor] + euclidean_distance(neighbor, end)
            heapq.heappush(openSet, (fScore[neighbor], neighbor))

def main():

    # [y][x]
    maze = [[0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]

    start = (0, 0)
    end = (0, 7)

    path = astar(maze, start, end)
    print(path)


main()