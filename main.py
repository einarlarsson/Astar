

from math import sqrt, pow
import heapq

#f(n) = g(n) + h(n), where

#g(n) the cost (so far) to reach the node
#h(n) estimated cost to get from the node to the goal
#f(n) estimated total cost of path through n to goal. It is implemented using priority queue by increasing f(n).

def euclidean_distance(x1, x2, y1, y2):
    return sqrt(pow(x2 - x1, 2) + (pow(y2 - y1, 2)))


def reconstruct_path(cameFrom, current):
    total_path = {current}
    while current in cameFrom.keys():
        current = cameFrom[current]
        total_path.add(current)
    return total_path


def find_neighbor(maze, current):
    neighbors = []
    neighbors.append((current[0] + 1, current[1]))
    neighbors.append((current[0], current[1] - 1))
    neighbors.append((current[0] - 1, current[1]))
    neighbors.append((current[0], current[1] + 1))
    result = []
    for n in neighbors:
        if n[0] < 0 or n[0] > 9 or n[1] < 0 or n[1] > 9:
            continue
        if maze[n[0]][n[1]] == 1:
            continue
        result.append(n)
    return result


def astar(maze, start, end):
    openSet = []
    closedSet = []

    heapq.heappush(openSet, (0, start))


    cameFrom = {}

    gScore = {}

    gScore[start] = 0;

    fScore = {}

    fScore[start] = euclidean_distance(start[0], start[1], end[0], end[1])

    while openSet:
        current = heapq.heappop(openSet)[1]

        if current == end:
            return reconstruct_path(cameFrom, current)

        closedSet.append(current)

        neighbors = find_neighbor(maze, current)
        for neighbor in neighbors:
            if neighbor in closedSet:
                continue
            tentative_gScore = gScore[current] + 1

            if neighbor not in openSet:
                openSet.append(neighbor)
            elif tentative_gScore >= gScore[neighbor]:
                continue

            cameFrom[neighbor] = current
            gScore[neighbor] = tentative_gScore
            score = gScore[neighbor] + euclidean_distance(neighbor[0], neighbor[1], end[0], end[1])
            fScore[neighbor] = score
            heapq.heappush(openSet, (score, neighbor))

def main():

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
    end = (9, 0)

    path = astar(maze, start, end)
    print(path)


main()