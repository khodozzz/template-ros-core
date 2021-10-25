#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import sys
import time

import rospy
from duckietown_msgs.msg import AprilTagsWithInfos, FSMState, TurnIDandType
from std_msgs.msg import Int16  # Imports msg

from my_controller.controller import NavigationController
from my_controller import dt_etu


class RandomAprilTagTurnsNode:
    def __init__(self):
        # Save the name of the node
        self.node_name = rospy.get_name()
        self.turn_type = -1

        rospy.loginfo(f"[{self.node_name}] Initializing.")

        # Setup my controller
        self.controller = NavigationController(dt_etu.build_graph())

        # Plan path
        self.controller.plan_path(dt_etu.start_pos(), dt_etu.target_pos())
        rospy.loginfo(f'[{self.node_name}] Path: {self.controller.path}')
        rospy.loginfo(f'[{self.node_name}] Roads: {self.controller.edges}')
        rospy.loginfo(f'[{self.node_name}] Turns (deg): {self.controller.turns}')

        # Initialize last turn time
        self.last_turn_time = None

        # Setup publishers
        # self.pub_topic_a = rospy.Publisher("~topic_a",String, queue_size=1)
        self.pub_turn_type = rospy.Publisher("~turn_type", Int16, queue_size=1, latch=True)
        self.pub_id_and_type = rospy.Publisher("~turn_id_and_type", TurnIDandType, queue_size=1, latch=True)

        # Setup subscribers
        # self.sub_topic_b = rospy.Subscriber("~topic_b", String, self.cbTopic)
        self.sub_topic_mode = rospy.Subscriber("~mode", FSMState, self.cbMode, queue_size=1)
        # self.fsm_mode = None #TODO what is this?
        self.sub_topic_tag = rospy.Subscriber("~tag", AprilTagsWithInfos, self.cbTag, queue_size=1)

        # Read parameters
        self.pub_timestep = self.setupParameter("~pub_timestep", 1.0)
        # Create a timer that calls the cbTimer function every 1.0 second
        # self.timer = rospy.Timer(rospy.Duration.from_sec(self.pub_timestep),self.cbTimer)

        rospy.loginfo(f"[{self.node_name}] Initialzed.")
        sys.stdout.flush()

        self.rate = rospy.Rate(30)  # 10hz

    def cbMode(self, mode_msg):
        # print mode_msg
        # TODO PUBLISH JUST ONCE
        self.fsm_mode = mode_msg.state
        if self.fsm_mode != mode_msg.INTERSECTION_CONTROL:
            self.turn_type = -1
            self.pub_turn_type.publish(self.turn_type)
            # rospy.loginfo("Turn type now: %i" %(self.turn_type))

    def cbTag(self, tag_msgs):
        if (
                self.fsm_mode == "INTERSECTION_CONTROL"
                or self.fsm_mode == "INTERSECTION_COORDINATION"
                or self.fsm_mode == "INTERSECTION_PLANNING"
        ):
            # loop through list of april tags

            # filter out the nearest apriltag
            dis_min = 999
            idx_min = -1
            for idx, taginfo in enumerate(tag_msgs.infos):
                if taginfo.tag_type == taginfo.SIGN:
                    tag_det = (tag_msgs.detections)[idx]
                    pos = tag_det.transform.translation
                    distance = math.sqrt(pos.x ** 2 + pos.y ** 2 + pos.z ** 2)
                    if distance < dis_min:
                        dis_min = distance
                        idx_min = idx

            if idx_min != -1:

                # check if last choice of the turn was min_time_diff sec ago
                min_time_diff = 15
                time_diff = 100 if self.last_turn_time is None else time.time() - self.last_turn_time
                if time_diff < min_time_diff:
                    # rospy.loginfo(f'[{self.node_name}] Last choice of the turn was {time_diff} sec ago')
                    return

                chosenTurn = self.controller.next_turn()

                if chosenTurn is not None:
                    self.turn_type = chosenTurn[0]
                    self.pub_turn_type.publish(self.turn_type)

                    taginfo = (tag_msgs.infos)[idx_min]
                    id_and_type_msg = TurnIDandType()
                    id_and_type_msg.tag_id = taginfo.id
                    id_and_type_msg.turn_type = self.turn_type
                    self.pub_id_and_type.publish(id_and_type_msg)

                    self.last_turn_time = time.time()

                    rospy.loginfo(f'[{self.node_name}] Chosen turn is {chosenTurn[1]} ({chosenTurn[0]})')
                else:
                    rospy.loginfo(f'[{self.node_name}] Target {self.controller.target} was achieved)')

                sys.stdout.flush()

    def setupParameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)  # Write to parameter server for transparancy
        # rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def on_shutdown(self):
        rospy.loginfo(f"[{self.node_name}] Shutting down.")


if __name__ == "__main__":
    # Initialize the node with rospy
    rospy.init_node("random_april_tag_turns_node", anonymous=False)

    # Create the NodeName object
    node = RandomAprilTagTurnsNode()

    # Setup proper shutdown behavior
    rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()
