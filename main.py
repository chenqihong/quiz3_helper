# encoding:utf-8

# This file implements "A star" algorithm - Author: Marcos Castro

# A* or "A star" is the combination of Uniform-cost and Greedy
# Uniform-cost orders by path cost or backward cost - g(n)
# Greedy orders by goal proximity or forward cost - h(n)
# A* Search orders by the sum: f(n) = g(n) + h(n)
# Reference: https://www.youtube.com/watch?v=b9fH-j_yNHU

import heapq  # priority queue
from collections import defaultdict  # dictionary of lists


class PriorityQueue:
    def __init__(self):
        self._queue = []
        self._index = 0

    def insert(self, item, priority):
        heapq.heappush(self._queue, (priority, self._index, item))
        self._index += 1

    def remove(self):
        return heapq.heappop(self._queue)[-1]

    def isEmpty(self):
        return len(self._queue) == 0

    def getSize(self):
        return self._index

    def getItems(self):
        result_list = []
        for node_object in self._queue:
            if type(node_object) != tuple:
                result_list.append(node_object.getKey())
            else:
                f_val, index, mytuple = node_object
                my_node, g_cost, h_cost = mytuple
                result_list.append(my_node.getKey() + ':' + str(f_val))
        return result_list


# class that represents a node
class Node:

    # "key" is the identifier of node
    # "forward_cost" is h(n) (cost of the heuristic)
    # "forward_cost" is used in the calculate of "A* search": f(n) = g(n) + h(n) where
    # h(n) is the forward cost and g(n) is the backward cost
    # remember: "A* search" is the combination of Uniform-cost (UCS) and Greedy
    def __init__(self, key, forward_cost):
        self.key = key
        self.forward_cost = forward_cost

    def getKey(self):
        return self.key

    def getForwardCost(self):
        return self.forward_cost


class Graph:

    def __init__(self):
        self.nodes = {}  # dictionary of the nodes
        self.edges = []  # list of 3-tuple (source, destination, weight)
        self.path = []  # path

        # dictionary with the lists of successors of each node, faster for get the successors
        # each item of list is a 2-tuple: (destination, weight)
        self.successors = defaultdict(list)

    # function that checks if edge exists
    def existsEdge(self, edge):
        for e in self.edges:
            # compares source's key, destionation's key and weight of edge
            if e[0].getKey() == edge[0].getKey() and e[1].getKey() == edge[1].getKey() and e[2] == edge[2]:
                return True
        return False

    # function that adds edges
    def addEdge(self, source, destination, weight):
        edge = (source, destination, weight)  # creates tuple (3-tuple)
        if not self.existsEdge(edge):  # adds edge if not exists
            self.nodes[source], self.nodes[destination] = source, destination  # adds the nodes
            self.edges.append(edge)  # adds edge
            self.successors[source.getKey()].append((destination, weight))  # adds successor
        else:
            print('Error: edge (%s -> %s with weight %s) already exists!!' % (
                edge[0].getKey(), edge[1].getKey(), edge[2]))

    # function that returns the path
    def getPath(self):
        return self.path

    def preprocess_check(self, initial_node, goal_node_list):
        if not self.edges:
            print("Error: empty graph !!!")
            return False
        for goal_node in goal_node_list:
            if initial_node not in self.nodes or goal_node not in self.nodes or initial_node == goal_node:
                return False
        return True

    def find_path_cost(self):
        start_node = self.path[0]
        path_cost = 0
        for node in self.path[1:]:
            for start, end, weight in self.edges:
                if start.getKey() == start_node and end.getKey() == node:
                    path_cost += weight
                    break
            start_node = node
        return path_cost

    def executeAStar(self, initial_node, goal_node_list):
        preprocess_check_result = self.preprocess_check(initial_node, goal_node_list)
        if not preprocess_check_result:
            return 0

        queue = PriorityQueue()  # creates a priority queue (min heap)

        # "distance_vector" and "ancestors" are used for reconstruct the path
        distance_vector, antecessors = {}, {}
        for node in self.nodes:
            distance_vector[node.getKey()] = None  # initializes with None
            antecessors[node.getKey()] = None
        distance_vector[initial_node.getKey()] = 0

        # calculates costs
        g_cost, h_cost = 0, initial_node.getForwardCost()
        f_cost = g_cost + h_cost  # g = cost so far to reach current node and h is the heuristic value
        queue.insert((initial_node, g_cost, h_cost), f_cost)
        total_cost = None
        while True:
            # a item of the queue is a 3-tuple: (current_node, g_cost, h_cost)
            current_node, g_cost, h_cost = queue.remove()
            print("currentNode = ", current_node.getKey())
            # gets all the successors of "current_node"
            successors = self.successors[current_node.getKey()]
            for successor in successors:
                destination, weight = successor
                # calculates costs
                new_g_cost = g_cost + weight
                h_cost = destination.getForwardCost()
                f_cost = new_g_cost + h_cost
                queue.insert((destination, new_g_cost, h_cost), f_cost)

                # updates "distance_vector"
                if distance_vector[destination.getKey()]:
                    if distance_vector[destination.getKey()] > new_g_cost:
                        distance_vector[destination.getKey()] = new_g_cost
                        antecessors[destination.getKey()] = current_node.getKey()
                else:
                    distance_vector[destination.getKey()] = new_g_cost
                    antecessors[destination.getKey()] = current_node.getKey()

                # verifies that reached the goal
                for goal_node in goal_node_list:
                    if destination.getKey() == goal_node.getKey():
                        # updated "total_cost"
                        if not total_cost:
                            total_cost = f_cost
                        elif f_cost < total_cost:
                            total_cost = f_cost

            if queue.isEmpty() or current_node in goal_node_list:  # verifies if the queue is empty
                # reconstruct the path
                path_cost_dict = dict()
                for goal_node in goal_node_list:
                    self.path = []
                    curr_node = goal_node.getKey()
                    while curr_node:
                        self.path.append(curr_node)
                        curr_node = antecessors[curr_node]
                    self.path = self.path[::-1]
                    path_cost = self.find_path_cost()
                    path_cost_dict[str(self.path)] = path_cost
                return path_cost_dict


# creates the nodes
nodeS = Node('S', 23)
nodeA = Node('A', 500)
nodeB = Node('B', 21)
nodeG1 = Node('G1', 0)
nodeG2 = Node('G2', 0)


# creates graph
graph = Graph()

# add the edges
graph.addEdge(nodeS, nodeA, 20)
graph.addEdge(nodeS, nodeB, 5)



graph.addEdge(nodeA, nodeG1, 30)
graph.addEdge(nodeA, nodeG2, 26)


graph.addEdge(nodeB, nodeB, 100)
graph.addEdge(nodeB, nodeA, 14)




path_cost_result_dict = graph.executeAStar(nodeS, [nodeG1, nodeG2])  # executes the algorithm
print("path_cost_result_dict = ", path_cost_result_dict)
path_cost_result_dict = dict(sorted(path_cost_result_dict.items(), key=lambda item: item[1]))
if path_cost_result_dict:
    path = list(path_cost_result_dict.keys())[0]
    total_cost = path_cost_result_dict[path]
    print('Total cost of graph 1: %s. Path: %s' % (total_cost, path))
else:
    print('Did not reach the goal!')

