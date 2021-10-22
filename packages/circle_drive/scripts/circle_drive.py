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
        rospy.loginfo('Building path...')
        self.controller.plan_path(dt_etu.target_pos())

        while not rospy.is_shutdown():
            if self.red_line_sub:  # TODO: if on red line
                # msg = Int32()
                # msg.data = 0  # turn
                # self.state_pub.publish(msg)

                turn = self.controller.next_turn()
                rospy.loginfo(f'Next turn is {turn}')
                # msg = Int32()
                # msg.data = turn
                # self.turn_pub.publish(msg)
            else:
                pass
                # msg = Int32()
                # msg.data = 1  # lane following
                # self.state_pub.publish(msg)


if __name__ == '__main__':
    # create the node
    node = NavigationNode(node_name='navigation_node')
    # run node
    node.run()
    # keep spinning
    rospy.spin()
