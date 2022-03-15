import numpy as np
from collections import deque

class Graph:
    def __init__(self, root_node):
        self.__graph = {tuple(root_node) : None }
        self.__root = tuple(root_node)

    def Append(self, node: np.ndarray, parent: np.ndarray):
        try:
            self.__graph[tuple(parent)]
        except KeyError:
            print("no parent exists")
            return
        self.__graph[tuple(node)] = tuple(parent)

    def GetBranch(self, endState):
        res = deque()
        res.appendleft(endState)
        parentState = self.__graph.get(tuple(endState))
        while (parentState != None):
            res.appendleft(np.array(parentState, dtype=np.uint))
            parentState = self.__graph.get(parentState)
        return res