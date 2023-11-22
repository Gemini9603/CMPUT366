import heapq
import sys
from search.algorithms import State
from search.map import Map

def Dijkstra(start, goal, gridded_map):
    open = []
    closed = {}
    n_nodes = 0
    cost = start.set_cost(0)
    heapq.heappush(open, start)
    closed[start.state_hash()] = start

    while len(open)!=0:
        node = heapq.heappop(open)
        n_nodes += 1
        if node == goal:
            cost = closed[node.state_hash()].get_cost()
            return cost, n_nodes
        children = gridded_map.successors(node)
        for child in children:
            hash_value = child.state_hash()
            if hash_value not in closed:
                child.set_cost(child.get_g())
                heapq.heappush(open, child)
                closed[hash_value] = child
            if (hash_value in closed) and (child.get_g() < closed[hash_value].get_cost()):
                child.set_cost(child.get_g())
                closed[hash_value] = child
                heapq.heapify(open)

    return -1, n_nodes

def set_h_value(node, goal):
    delta_x = abs(node.get_x() - goal.get_x())
    delta_y = abs(node.get_y() - goal.get_y())
    h = 1.5 * min(delta_x, delta_y) + abs(delta_x - delta_y)
    return h

def Astar(start, goal, gridded_map):
    open = []
    closed = {}
    n_nodes = 0
    h = set_h_value(start, goal)
    cost = start.set_cost(h)
    heapq.heappush(open, start)
    closed[start.state_hash()] = start
    heapq.heapify(open)

    while len(open)!=0:
        node = heapq.heappop(open)
        n_nodes += 1
        if node == goal:
            cost = node.get_cost()
            return cost, n_nodes
        children = gridded_map.successors(node)
        for child in children:
            hash_value = child.state_hash()
            h = set_h_value(child, goal)
            f = child.get_g() + h
            child.set_cost(f)
            if hash_value not in closed:
                h = set_h_value(child, goal)
                child.set_cost(child.get_g() + h)
                heapq.heappush(open, child)
                closed[hash_value] = child
            temp = closed[hash_value]
            if (hash_value in closed) and (child.get_cost() < temp.get_cost()):
                h = set_h_value(child, goal)
                child.set_cost(child.get_g() + h)
                temp.set_cost(child.get_g() + h)
                temp.set_g(child.get_g())
                heapq.heapify(open)

    return -1, n_nodes
