from enum import Enum
import numpy as np

from navigation.graph import Graph


class NavigationController:
    def __init__(self, graph, start_pos):
        self.graph = graph
        self.current_pos = start_pos

        self._turns_it = None

    def plan_path(self, target):
        path, turns, dist = self.graph.shortest_path(self.current_pos, target)
        self._turns_it = iter(turns)

    def next_turn(self):
        return next(self._turns_it)
