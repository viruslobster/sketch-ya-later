import numpy as np
import cv2
import math
import heapq
from itertools import *
import sys
import time
import math
import random
import pytsp

img = cv2.imread('images/testudo.jpg', 1)
####################################################################################################
### Image Preprocessing ############################################################################
####################################################################################################
def extract_edges(img, blur=None, thresh1=100, thresh2=200):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blurred = cv2.medianBlur(gray, blur) if blur else gray
    edges = cv2.Canny(blurred, thresh1, thresh2).astype(bool)
    return edges

####################################################################################################
### Graph Utility Functions ########################################################################
####################################################################################################

def inbounds(img, i, j):
    m, n = img.shape
    proposal = [(i+1, j), (i-1, j), (i+1, j+1), (i+1, j-1), (i-1, j+1), (i-1, j-1), (i, j+1), (i, j-1)]
    return ((k, l) for (k, l) in proposal if 0 <= k < m and 0 <= l < n)

def neighbors(img, i, j):
    m, n = img.shape
    proposal = [(i+1, j), (i-1, j), (i+1, j+1), (i+1, j-1), (i-1, j+1), (i-1, j-1), (i, j+1), (i, j-1)]
    return ((k, l) for (k, l) in proposal if 0 <= k < m and 0 <= l < n and img[k, l])

def reconstruct((i, j), came_from):
    connection = [(i,j)]
    while (i, j) in came_from.keys():
        i, j = came_from[(i, j)]
        connection.insert(0, (i, j))
    return connection

def bfs(img, start, adjacent, finished, heuristic=None, max_cost=float('inf')):
    queue = []
    gscore = np.zeros_like(img) + float('inf')
    if heuristic:
        fscore = np.zeros_like(img) + float('inf')
    for index in start:
        gscore[index] = 0
        if heuristic:
            fscore[index] = heuristic(*index)
            heapq.heappush(queue, (fscore[index], index))
        else:
            heapq.heappush(queue, (gscore[index], index))
    came_from = {}

    while True:
        if len(queue) == 0:
            return None, None
        cost, (i, j) = heapq.heappop(queue)
        if cost > max_cost:
            return [], None
        if finished(i, j):
            break

        for (k, l) in adjacent(img, i, j):
            new_g = cost + (1 if i == k or j == l else math.sqrt(2))
            if new_g < gscore[k, l]:
                gscore[k, l] = new_g
                if heuristic:
                    fscore[k, l] = new_g + heuristic(k, l)
                    heapq.heappush(queue, (fscore[k, l], (k, l)))
                else:
                    heapq.heappush(queue, (gscore[k, l], (k, l)))
                came_from[(k, l)] = (i, j)
    connection = reconstruct((i, j), came_from)
    return connection, gscore

####################################################################################################
### Contour Connection #############################################################################
####################################################################################################
def find_component(img, i, j, result, unconnected):
    m, n = img.shape
    result.add((i, j))
    if (i, j) in unconnected: unconnected.remove((i, j))
    for (k, l) in neighbors(img, i, j):
        if (k, l) in result: continue
        find_component(img, k, l, result, unconnected)

def find_components(img):
    n, m = img.shape
    sys.setrecursionlimit(n*m)
    i, j = np.argwhere(img)[0]
    unconnected = set((k, l) for k, l in np.argwhere(img))
    components = []
    while len(unconnected) > 0:
        component = set()
        find_component(img, i, j, component, unconnected)
        components.append(component)
        if len(unconnected) > 0:
            i, j = unconnected.pop()
    components[-1].add((i, j))
    return components

def sample(component):
    n = len(component)
    size = max(int(n), 1)
    component = np.array(list(component))
    i = np.random.choice(np.arange(n), size=size, replace=False)
    return component

def connect(img, components):
    m, n = img.shape
    unconnected = set((k, l) for k, l in np.argwhere(img))
    giant_component = set()
    component = components.pop(0)
    while len(components) > 0:
        giant_component = giant_component | component
        for (j, l) in component:
            unconnected.remove((j, l))

        giant_approx = sample(giant_component)
        best_dist = float('inf')
        best_point = None
        for c in components:
            approx = sample(c)
            for point in approx:
                dists = np.sum(np.square(giant_approx - point), 1)
                l = np.argmin(dists)
                if dists[l] < best_dist:
                    best_dist = dists[l]
                    best_point = point

        finished = lambda x, y: (x, y) in unconnected
        heuristic = lambda x, y: (x - best_point[0])**2 + (y - best_point[1])**2
        connection, _ = bfs(img, giant_component, inbounds, finished, heuristic)
        index = connection.pop(-1)
        x = np.array(connection)
        img[x[:,0],x[:,1]] = True
        # index, connection = nearest_component(img, giant_component, unconnected, best_point)
        j = np.argwhere([index in c for c in components])
        component = components.pop(j)

        giant_component = giant_component | set(connection)

def nn_tsp2(img, start, visited):
    vertices = set((i, j) for (i, j) in np.argwhere(img))
    vid = dict(zip(vertices, range(len(vertices))))
    n = np.sum(img)
    path = [start]
    visited.add(start)
    paths = {}
    while True:
        i, j = path[-1]

        found = False
        for (k, l) in neighbors(img, i, j):
            if (k, l) in visited: continue
            path.append((k, l))
            visited.add((k, l))
            for x in set(neighbors(img, i, j)).intersection(neighbors(img, k, l)):
                visited.add(x)

            found = True

            break
        if not found:
            sub_path, gscore = bfs(img, [(i, j)], neighbors, lambda x, y: (x, y) not in visited and img[x, y])
            if sub_path == None:
                # import IPython; IPython.embed()
                return path
            sub_path = sub_path[1:]

            visited.add(sub_path[-1])
            path.extend(sub_path)
    return path

def connect2(img, components):
    m, n = img.shape
    # component = components.pop(0)
    A = np.zeros((len(components)+1, len(components)+1))
    hpoints = {}
    samples = [np.array([(0,0)])] + [sample(c) for c in components]
    components = [set([(0,0)])] + components
# [0, 1, 2, 5, 16, 47, 39, 42, 31, 41, 44, 18, 10,
# 23, 36, 32, 37, 6, 17, 38, 40, 34, 19, 30, 11, 22,
# 27, 45, 35, 21, 46, 29, 24, 26, 14, 9, 12, 13, 25,
# 43, 33, 28, 15, 20, 4, 7, 3, 8]
    for i in range(len(components)-1):
        for j in range(i, len(components)):
            best_dist = float('inf')
            best_point = None
            for point in samples[j]:
                dists = np.sum(np.square(samples[i] - point), 1)
                l = np.argmax(dists)
                if dists[l] < best_dist:
                    best_dist = dists[l]
                    best_point = point
            # if i > 0 and best_dist > 1000:
            #     best_dist = 999999
            A[i, j] = best_dist**1.1
            A[j, i] = best_dist**1.1
            hpoints[(i, j)] = best_point
            hpoints[(j, i)] = best_point
    with open('/tmp/matrix', 'w') as dest:
        dest.write(pytsp.dumps_matrix(A, name="Route"))
    tour = pytsp.run('/tmp/matrix', start=0, solver='concorde')['tour']
    # import IPython; IPython.embed()

    # tour = [0, 3, 5, 10, 4, 6, 2, 7, 9, 8, 1]
    giant_component = set()
    unconnected = set((k, l) for k, l in np.argwhere(img))
    path = []
    visited = set()
    start = (0, 0)
    img[start] = True
    # import pdb;pdb.set_trace()
    print len(tour)
    a = 6
    print tour
    for i in range(len(tour)):
        j = (i+1) % len(tour)

        # compute the nn_tsp of the component
        sub_path1 = nn_tsp2(img, start, visited)
        path.extend(sub_path1)

        # compute the best path to the next one
        finished = lambda x, y: (x, y) in components[j]
        best_point = hpoints[(tour[i], tour[j])]
        heuristic = lambda x, y: math.sqrt((x - best_point[0])**2 + (y - best_point[1])**2)
        sub_path2, _ = bfs(img, path, inbounds, finished, heuristic)
        x = np.array(sub_path2)
        img[x[:,0],x[:,1]] = True

        start = sub_path2[-1]
        # import pdb;pdb.set_trace()

        connection, _ = bfs(img, [sub_path1[-1]], neighbors, lambda x, y: (x, y) == start)
        connection.pop()
        if connection:
            path.extend(connection)

        for pt in connection:
            visited.add(pt)

        # import IPython;IPython.embed()




    return path



    while len(components) > 0:
        giant_component = giant_component | component
        for (j, l) in component:
            unconnected.remove((j, l))

        giant_approx = sample(giant_component)
        best_dist = float('inf')
        best_point = None
        for c in components:
            approx = sample(c)
            for point in approx:
                dists = np.sum(np.square(giant_approx - point), 1)
                l = np.argmin(dists)
                if dists[l] < best_dist:
                    best_dist = dists[l]
                    best_point = point

        finished = lambda x, y: (x, y) in unconnected
        heuristic = lambda x, y: (x - best_point[0])**2 + (y - best_point[1])**2
        connection, _ = bfs(img, giant_component, inbounds, finished, heuristic)
        index = connection.pop(-1)
        x = np.array(connection)
        img[x[:,0],x[:,1]] = True
        # index, connection = nearest_component(img, giant_component, unconnected, best_point)
        j = np.argwhere([index in c for c in components])
        component = components.pop(j)

        giant_component = giant_component | set(connection)

####################################################################################################
### TSP Solver #####################################################################################
####################################################################################################
def find_component(img, i, j, result, unconnected):
    m, n = img.shape
    result.add((i, j))
    if (i, j) in unconnected: unconnected.remove((i, j))
    for (k, l) in neighbors(img, i, j):
        if (k, l) in result: continue
        find_component(img, k, l, result, unconnected)

def local_search(img, path, visited, unvisited, maxlen):
    if len(path) == maxlen:
        # print unvisited
        return path, unvisited
    i, j = path[-1]
    n = [n for n in neighbors(img, i, j) if n not in visited]
    best_path, best_unvisited = path, unvisited
    for i in range(len(n)):
        path2, unvisited2 = local_search(img, path + [n[i]], visited.union([n[i]]), unvisited.difference([n[i]]).union(n[:i]+n[i+1:]), maxlen)
        # if unvisited2 == None:
        #     import IPython;IPython.embed()
        if len(best_path) < len(path2) or len(unvisited2) < len(best_unvisited):
            best_path = path2
            best_unvisited = unvisited2
    return best_path, best_unvisited

def nn_tsp(img, start):
    vertices = set((i, j) for (i, j) in np.argwhere(img))
    vid = dict(zip(vertices, range(len(vertices))))
    n = np.sum(img)
    path = [start]
    costs = [0]
    visited = set([start])
    paths = {}
    # A = np.zeros((n, n))
    while len(visited) < n:
        i, j = path[-1]
        # print 'local search...'
        # sub_path, _ = local_search(img, [(i, j)], visited, set(), 3)
        # if len(sub_path) > 1:
        #     for (k, l) in sub_path[1:]:
        #         path.append((k, l))
        #         visited.add((k, l))
        found = False
        for (k, l) in neighbors(img, i, j):
            if (k, l) in visited: continue
            path.append((k, l))
            visited.add((k, l))
            for x in set(neighbors(img, i, j)).intersection(neighbors(img, k, l)):
                visited.add(x)
            # costs.append(1 if i == k or j == l else math.sqrt(2))

            found = True

            break
        if not found:
            # sub_path, gscore = bfs(img, [(i, j)], inbounds, lambda x, y: (x, y) not in visited and img[x, y], max_cost=20)
            # if gscore == None:
            sub_path, gscore = bfs(img, [(i, j)], neighbors, lambda x, y: (x, y) not in visited and img[x, y])
            if sub_path == None:
                sub_path, _ = bfs(img, [path[-1]], neighbors, lambda x, y: (x, y) == (0, 0))
                path.extend(sub_path)
                return path, costs, paths
            sub_path = sub_path[1:]
            x = np.array(sub_path)
            # costs.extend(gscore[x[:,0],x[:,1]].tolist())
            costs.append(gscore[sub_path[-1]])
            # paths[(i, j)] = sub_path
            # A[vid[i, j],vid[sub_path[-1]]] = cost
            # A[vid[sub_path[-1]],vid[i, j]] = cost

            for node in sub_path:
                visited.add(node)
                path.append(node)

            # paths[(i, j)] = sub_path
    sub_path, _ = bfs(img, [path[-1]], neighbors, lambda x, y: (x, y) == (0, 0))
    path.extend(sub_path)

    return path, costs, paths

def simplify_path(path, x=0.9):
    simple_path = np.squeeze(cv2.approxPolyDP(path, x, False))
    # new_costs = []
    # j = 0
    # for i in range(simple_path.shape[0]):
    #     while not np.array_equal(simple_path[i], path[j]):
    #         j += 1
    #     new_costs.append(costs[j])
    return simple_path

def reconstruct_path(compressed_path, paths):
    path = [tuple(compressed_path[0])]
    for i in range(1, compressed_path.shape[0]):
        v = tuple(compressed_path[i])
        if path[-1] in paths:
            path.extend(paths[path[-1]])
            # path.append(v)
        else:
            path.append(v)
    return np.array(path)

def improve_path2(img, path, costs, paths):
    # compress path

    for i in np.arange(1, path.shape[0]):
        for j in np.arange(i+2, path.shape[0]):
            path, _, flag = opt2(img, path, costs, paths, i, j)
            if flag:
                return path
        # for j in range(i+2, path.shape[0]):
        #     # print path[i-1], list(neighbors(img, *path[j-1]))
        #     if tuple(path[i-1]) not in neighbors(img, *path[j-1]): continue
        #     path, _, flag = opt2(img, path, costs, paths, i, j)
        #     if flag:
        #         return reconstruct_path(img, path, paths)
    path = reconstruct_path(img, path, paths)

    return path

def opt2(img, path, costs, subpaths, i, j):
    # import pdb;pdb.set_trace()
    v1 = tuple(path[i-1])
    v2 = tuple(path[i])
    cost1 = costs[i]
    v3 = tuple(path[j-1])
    v4 = tuple(path[j])
    cost2 = costs[j]
    # if cost1 + cost2 < 20:
    #     return path, costs, False
    subpath1, new_cost1 = bfs(img, [v1], neighbors, lambda x, y: (x, y) == v3, max_cost=cost1+cost2)
    if new_cost1 == None:
        return path, costs, False
    new_cost1 = new_cost1[subpath1[-1]]
    subpath2, new_cost2 = bfs(img, [v2], neighbors, lambda x, y: (x, y) == v4, max_cost=cost1+cost2-new_cost1)
    if new_cost2 == None:
        return path, costs, False
    new_cost2 = new_cost2[subpath2[-1]]
    # print new_cost2



    if cost1 + cost2 <= new_cost1 + new_cost2 + 10:
        return path, costs, False

    subpaths[v1] = subpath1[1:]
    subpaths[v3] = subpath2[1:]
    print '--------'
    print cost1, cost2, cost1 + cost2
    print new_cost1, new_cost2, new_cost1+new_cost2

    print j
    print v3
    print v4


    new_path = []
    new_costs = []
    new_path = np.vstack((path[:i], path[::-1][i:j], path[j:]))
    # i = 0
    # while not np.array_equal(path[i], v2):
    #     new_path.append(path[i])
    #     new_costs.append(costs[i])
    #     i += 1
    # j = i
    # while not np.array_equal(path[i], v4):
    #     new_path.insert(j, path[i])
    #     new_costs.insert(j, costs[i])
    #     # if tuple(path[i]) in subpaths:
    #     #     subpaths[tuple(path[i])] = list(reversed(subpaths[tuple(path[i])]))
    #     i += 1
    #
    # new_path.extend((m, n) for (m, n) in path[i:])
    # new_costs.extend(costs[i:])

    costs = np.array(costs)

    costs[i:] -= cost1
    costs[i:] += new_cost1
    costs[j:] -= cost2
    costs[j:] += new_cost2
    costs = costs.tolist()

    return new_path, costs, True
