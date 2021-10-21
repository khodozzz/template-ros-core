from enum import Enum
import numpy as np

from road_map import RoadMap


class NavigationController:
    def __init__(self, road_map, current_location):
        self.road_map = road_map
        self.current_location = current_location

        self.path_from = None
        self.path_to = None
        self.current_action = None

    def set_path(self, path_from, path_to):
        self.path_from = path_from
        self.path_to = path_to

    def set_path(self, path_to):
        self.path_from = self.current_location
        self.path_to = path_to

    def shortest_path(self):
        # TODO
        return []

    def next_turn(self):
        # TODO
        return 'left'
