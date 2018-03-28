#! /usr/bin/env python

import rospy

import actionlib

import dynamic_reconfigure.client as dyncfg

import re

from move_base_msgs.msg import MoveBaseFeedback, MoveBaseResult, MoveBaseAction, MoveBaseActionGoal

NODE_NAME = "mummer_navigation_node"
MOVE_TO_AS_NAME = "m_move_to"
APPROACH_AS_NAME = "m_approach"

MOVE_BASE_AS_NAME = "move_base"

REGEX_FRAME_TO_HUMAN_ID = "(?:mocap_)?human-([0-9]+)"


class NavigationMummerAction(object):
    def __init__(self):
        self._move_base_as = actionlib.SimpleActionClient(MOVE_BASE_AS_NAME, MoveBaseAction)
        rospy.loginfo(NODE_NAME + " waiting for move_base action server.")
        self._move_base_as.wait_for_server()
        rospy.loginfo(NODE_NAME + " found move_base action server.")
        self._move_base_as_recfg_client = dyncfg.Client("/move_base_node/TebLocalPlannerROS")
        self._as_move_to = actionlib.SimpleActionServer(MOVE_TO_AS_NAME, MoveBaseAction,
                                                execute_cb=self.execute_move_to_cb, auto_start=False)
        self._feedback_move_to = None
        self._feedback_rate = rospy.Rate(10)
        self._as_move_to.start()

        self.regex_frame_to_human_id = re.compile(REGEX_FRAME_TO_HUMAN_ID)
        self._as_approach = actionlib.SimpleActionServer(APPROACH_AS_NAME, MoveBaseAction,
                                                         execute_cb=self.execute_approach_cb, auto_start=False)
        self._feedback_approach = None
        self._as_approach.start()
        rospy.loginfo(NODE_NAME + " server started.")

    def execute_move_to_cb(self, goal):
        # helper variables
        self._move_base_as_recfg_client.update_configuration({"use_human_robot_visi_c": False, "planning_mode": 1})
        self._move_base_as.send_goal(goal,done_cb=self.done_move_to_cb, feedback_cb=self.feedback_move_to_cb)

        while self._move_base_as.get_state() == actionlib.SimpleGoalState.ACTIVE or self._move_base_as.get_state() == actionlib.SimpleGoalState.PENDING:
            if self._as_move_to.is_preempt_requested():
                self._as_move_to.set_preempted()
                self._move_base_as.cancel_all_goals()
                break
            if self._feedback_move_to is not None:
                self._as_move_to.publish_feedback(self._feedback_move_to)
            self._feedback_rate.sleep()

    def feedback_move_to_cb(self, feedback):
        self._feedback_move_to = feedback

    def done_move_to_cb(self, state, result):
        self._as_move_to.set_succeeded(result)

    def execute_approach_cb(self, goal):
        self.regex_frame_to_human_id.match(goal.target_pose.header.frame_id)
        self._move_base_as_recfg_client.update_configuration({"use_human_robot_visi_c": True, "planning_mode": 2})

        self._move_base_as.send_goal(goal, done_cb=self.done_approach_cb, feedback_cb=self.feedback_approach_cb)

        while self._move_base_as.get_state() == actionlib.SimpleGoalState.ACTIVE or self._move_base_as.get_state() == actionlib.SimpleGoalState.PENDING:
            if self._as_approach.is_preempt_requested():
                self._as_approach.set_preempted()
                self._move_base_as.cancel_all_goals()
                break
            if self._feedback_approach is not None:
                self._as_approach.publish_feedback(self._feedback_approach)
            self._feedback_rate.sleep()

    def feedback_approach_cb(self, feedback):
        self._feedback_approach = feedback

    def done_approach_cb(self, state, result):
        self._as_approach.set_succeeded(result)




if __name__ == '__main__':
    rospy.init_node(NODE_NAME)
    server = NavigationMummerAction()
    rospy.spin()
