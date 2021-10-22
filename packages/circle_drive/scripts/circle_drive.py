#!/usr/bin/env python3
import rospy
from duckietown.dtros import DTROS, NodeType
from navig.controller import NavigationController
from navig import dt_etu


class NavigationNode(DTROS):
    def __init__(self, node_name):
        super(NavigationNode, self).__init__(node_name=node_name, node_type=NodeType.DEBUG)

        self.controller = NavigationController(dt_etu.build_graph(), dt_etu.start_pos())

        # self.red_line_sub = rospy.Subcriber("~red_line", Bool, queue_size=1)
        # self.turn_pub = rospy.Publisher("~turn", Int32, queue_size=1)
        # self.state_pub = rospy.Publisher("~state", Int32, queue_size=1)

    def run(self):
        self.controller.plan_path(dt_etu.target_pos())
        rospy.loginfo(f'Builded path {self.controller._path}')
        rospy.loginfo(f'Distance {self.controller._dist}')

        rate = rospy.Rate(1)  # 1Hz
        while not rospy.is_shutdown():
            turn = self.controller.next_turn()
            road = self.controller.next_road()
            rospy.loginfo(f'Next turn is {turn}')
            rospy.loginfo(f'Next road is {road}')
            rate.sleep()



if __name__ == '__main__':
    # create the node
    node = NavigationNode(node_name='navigation_node')
    # run node
    node.run()
    # keep spinning
    rospy.spin()
