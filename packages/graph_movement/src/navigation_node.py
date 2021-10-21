#!/usr/bin/env python3
import rospy
from duckietown.dtros import DTROS, NodeType


class NavigationNode(DTROS):
    def __init__(self, node_name):
        super(NavigationNode, self).__init__(node_name=node_name, node_type=NodeType.DEBUG)
        self.pub = rospy.Publisher("~shortest_path", Twist2DStamped,  # TODO: msg left/right
        , queue_size = 1)
