#! /usr/bin/env python
import math
import re
import signal
import time

import actionlib
import dynamic_reconfigure.client as dyncfg
import rospy
import tf
import tf.transformations as t
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from mummer_navigation.msg import RotateGoal, RotateAction
from naoqi import ALProxy
from perspectives_msgs.srv import StartFact, EndFact
from std_srvs.srv import SetBool
from multiprocessing import Lock

NODE_NAME = "mummer_navigation_node"
MOVE_TO_AS_NAME = "m_move_to"
APPROACH_AS_NAME = "m_approach"
ROTATE_AS_NAME = "m_rotate"

MOVE_BASE_AS_NAME = "move_base"

START_FACT_SRV_NAME = "/uwds_ros_bridge/start_fact"
STOP_FACT_SRV_NAME = "/uwds_ros_bridge/end_fact"

TOGGLE_HUMAN_MONITORING_SRV = "/multimodal_human_monitor/global_monitoring"

REGEX_FRAME_TO_HUMAN_ID = "(?:mocap_)?human-([0-9]+)(_footprint)?"

CMD_VEL_TOPIC = "/cmd_vel"


class NavigationMummerAction(object):
    def __init__(self, nao_ip, nao_port):
        self.cmd_vel_topic = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=100)
        self.move_base_as = actionlib.SimpleActionClient(MOVE_BASE_AS_NAME, MoveBaseAction)
        rospy.loginfo(NODE_NAME + " waiting for underworlds start fact service server.")
        rospy.wait_for_service(START_FACT_SRV_NAME)
        rospy.loginfo(NODE_NAME + " found underworlds start fact service server.")
        rospy.loginfo(NODE_NAME + " waiting for underworlds stop fact service server.")
        rospy.wait_for_service(STOP_FACT_SRV_NAME)
        rospy.loginfo(NODE_NAME + " found underworlds stop fact service server.")
        self.move_to_fact_id = None
        self.approach_fact_id = None
        self.rotate_fact_id = None

        rospy.loginfo(NODE_NAME + " waiting for multimodal human monitor service.")
        rospy.wait_for_service(TOGGLE_HUMAN_MONITORING_SRV)
        rospy.loginfo(NODE_NAME + " found multimodal human monitor service.")
        self.toggle_human_monitor = rospy.ServiceProxy(TOGGLE_HUMAN_MONITORING_SRV, SetBool)

        self.nao_ip = nao_ip
        self.nao_port = nao_port
        self.al_motion_mtx = Lock()
        self.al_motion = ALProxy("ALMotion", nao_ip, nao_port)
        #self.al_proxy = ALProxy.

        self.tf_listener = tf.TransformListener()

        rospy.loginfo(NODE_NAME + " waiting for move_base action server.")
        self.move_base_as.wait_for_server()
        rospy.loginfo(NODE_NAME + " found move_base action server.")
        self._move_base_as_recfg_client = dyncfg.Client("/move_base_node/TebLocalPlannerROS")
        self.as_move_to = actionlib.SimpleActionServer(MOVE_TO_AS_NAME, MoveBaseAction,
                                                       execute_cb=self.execute_move_to_cb, auto_start=False)
        self.need_final_rotation = False

        self._feedback_move_to = None
        self._feedback_rate = rospy.Rate(10)
        self.as_move_to.start()

        self.regex_frame_to_human_id = re.compile(REGEX_FRAME_TO_HUMAN_ID)
        self.as_approach = actionlib.SimpleActionServer(APPROACH_AS_NAME, MoveBaseAction,
                                                        execute_cb=self.execute_approach_cb, auto_start=False)
        self._feedback_approach = None
        self.as_approach.start()

        self.as_rotate = actionlib.SimpleActionServer(ROTATE_AS_NAME, RotateAction,
                                                        execute_cb=self.execute_rotate_cb, auto_start=False)
        self._feedback_rotate = None
        self.as_rotate.start()

        rospy.loginfo(NODE_NAME + " server started.")

    def execute_move_to_cb(self, goal):
        # helper variables
        self._move_base_as_recfg_client.update_configuration({"use_human_robot_visi_c": False, "planning_mode": 1,
                                                              "use_human_robot_ttc_c": False,
                                                              "yaw_goal_tolerance": 2.0, "xy_goal_tolerance": 0.5})

        if goal.target_pose.header.frame_id != "map":
            goal.target_pose = self.tf_listener.transformPose("map", goal.target_pose)

        goal_2d = MoveBaseGoal()
        goal_2d.target_pose.header = goal.target_pose.header
        goal_2d.target_pose.pose.position.x = goal.target_pose.pose.position.x
        goal_2d.target_pose.pose.position.y = goal.target_pose.pose.position.y
        roll, pitch, yaw = t.euler_from_quaternion((goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y,
                                 goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w))
        x2d, y2d, z2d, w2d = t.quaternion_from_euler(0, 0, yaw)
        rospy.loginfo("roll, pitch, yaw : {}\tquaternion2d : {}".format((roll, pitch, yaw), (x2d, y2d, z2d, w2d)))
        goal_2d.target_pose.pose.orientation.x = x2d
        goal_2d.target_pose.pose.orientation.y = y2d
        goal_2d.target_pose.pose.orientation.z = z2d
        goal_2d.target_pose.pose.orientation.w = w2d


        resp = self.toggle_human_monitor(False)
        if not resp.success:
            rospy.logwarn(NODE_NAME + " could not stop human monitoring, returning failure.")
            self.as_move_to.set_aborted(text="Could not stop monitoring")
            return

        begin_nav_fact = rospy.ServiceProxy(START_FACT_SRV_NAME, StartFact)
        resp = begin_nav_fact("base", "isNavigating(robot)", rospy.Time.now().to_sec(), False)
        if resp.success:
            self.move_to_fact_id = resp.fact_id


        self.move_base_as.send_goal(goal_2d, done_cb=self.done_move_to_cb, feedback_cb=self.feedback_move_to_cb)

        r_pose, _ = self.tf_listener.lookupTransform("base_footprint", "map", rospy.Time(0))
        last_distance = math.hypot(goal_2d.target_pose.pose.position.x - r_pose[0],
                                   goal_2d.target_pose.pose.position.y - r_pose[1])
        last_decrease_time = rospy.Time.now()
        while self.move_base_as.get_state() == GoalStatus.ACTIVE or self.move_base_as.get_state() == GoalStatus.PENDING:
            if self.as_move_to.is_preempt_requested():
                self.as_move_to.set_preempted()
                self.move_base_as.cancel_all_goals()
                break
            r_pose, _ = self.tf_listener.lookupTransform("base_footprint", "map", rospy.Time(0))
            d = math.hypot(goal_2d.target_pose.pose.position.x - r_pose[0],
                            goal_2d.target_pose.pose.position.y - r_pose[1])
            rospy.loginfo_throttle(1, "[mummer_navigation] distance : {}, old_distance : {}".format(d, last_distance))
            if d < last_distance:
                last_decrease_time = rospy.Time.now()
                last_distance = d
            elif rospy.Time.now() - last_decrease_time >= rospy.Duration(9000):
                self.move_base_as.cancel_all_goals()
                self.need_final_rotation = True
                break
            if self._feedback_move_to is not None:
                self.as_move_to.publish_feedback(self._feedback_move_to)
            self._feedback_rate.sleep()

        robot_t, robot_r = self.tf_listener.lookupTransform("base_footprint", "map", rospy.Time(0))
        _, _, yaw_r = t.euler_from_quaternion(robot_r)
        self.need_final_rotation = True
        rospy.loginfo(NODE_NAME + " final rotation needed : {}, angle wanted : {}, angle : {}, diff : {}".format(
            self.need_final_rotation, yaw, yaw_r, abs(yaw-yaw_r)
        ))
        if self.need_final_rotation and abs(((yaw - yaw_r) + math.pi) % math.pi * 2 - math.pi) > -1:
            rospy.loginfo(NODE_NAME + " now doing final rotation")
            with self.al_motion_mtx:
                try:
                    self.al_motion.moveTo(0, 0, yaw)
                except RuntimeError as e:
                    rospy.logwarn(NODE_NAME + " MoveTo crash ! Retrying...")
                    self.al_motion = ALProxy("ALMotion", self.nao_ip, self.nao_port)
                    self.al_motion.moveTo(0, 0, yaw)

        self.need_final_rotation = False

        time.sleep(1.0)

        self.as_move_to.set_succeeded()
        rospy.loginfo(NODE_NAME + " Action server succeeded !")

        resp = self.toggle_human_monitor(True)


    def feedback_move_to_cb(self, feedback):
        self._feedback_move_to = feedback

    def done_move_to_cb(self, state, result):
        end_nav_fact = rospy.ServiceProxy(STOP_FACT_SRV_NAME, EndFact)
        if self.move_to_fact_id is not None:
            resp = end_nav_fact("base", self.move_to_fact_id)
            if resp.success:
                self.move_to_fact_id = None
        if state == GoalStatus.PREEMPTED:
            self.as_move_to.set_preempted(result)
        elif state == GoalStatus.SUCCEEDED:
            self.need_final_rotation = True
            #self.as_move_to.set_succeeded(result)
        elif state == GoalStatus.ABORTED:
            self.as_move_to.set_aborted(result)

# =================================================

    def execute_approach_cb(self, goal):
        """

        :param goal:
        :type goal: MoveBaseGoal
        :return:
        """

        match = self.regex_frame_to_human_id.match(goal.target_pose.header.frame_id)
        id = int(match.group(1))
        # if match.group(2) is None:
        #     self.tf_listener.transformPose(goal.target_pose, )
        d = math.hypot(goal.target_pose.pose.position.x, goal.target_pose.pose.position.y)
        goal_2d = MoveBaseGoal()
        goal_2d.target_pose.header = goal.target_pose.header
        robot_t_map, robot_r_map = self.tf_listener.lookupTransform("base_footprint", "map", rospy.Time(0))
        human_t, human_r = self.tf_listener.lookupTransform(goal.target_pose.header.frame_id, "map", rospy.Time(0))
        if d <= 0.1:
            d = 0.8
            a = math.atan2(human_t[1] - robot_t_map[1], human_t[0] - robot_t_map[0])
            goal.target_pose.pose.position.x = d
            goal.target_pose.pose.position.y = 0
            goal.target_pose.pose.position.z = 0

        else:
            _, _, a = t.euler_from_quaternion([goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y,
                                               goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w])
            # a = math.atan2(goal.target_pose.pose.position.y, goal.target_pose.pose.position.x)

            #goal_2d.target_pose.pose.position = goal.target_pose.pose.position
        goal_2d.target_pose = self.tf_listener.transformPose("map", goal.target_pose)

        goal_2d.target_pose.pose.orientation.x, goal_2d.target_pose.pose.orientation.y, \
            goal_2d.target_pose.pose.orientation.z, goal_2d.target_pose.pose.orientation.w = t.quaternion_from_euler(0.0,
                                                                                                                 0.0, a)

        self._move_base_as_recfg_client.update_configuration({"use_human_robot_visi_c": True, "planning_mode": 2,
                                                              "approach_id": id, "approach_dist": d,
                                                              "approach_angle": a, "yaw_goal_tolerance": 0.1, "xy_goal_tolerance": 0.5})

        resp = self.toggle_human_monitor(False)
        if not resp.success:
            rospy.logwarn(NODE_NAME + " could not stop human monitoring, returning failure.")
            self.as_approach.set_aborted(text="Could not stop monitoring")
            return

        begin_fact = rospy.ServiceProxy(START_FACT_SRV_NAME, StartFact)
        resp = begin_fact("base", "isNavigating(robot)", rospy.Time.now().to_sec(), False)
        if resp.success:
            self.move_to_fact_id = resp.fact_id
        resp = begin_fact("base", "isApproaching(robot, {})".format(
            goal.target_pose.header.frame_id
        ), rospy.Time.now().to_sec(), False)
        if resp.success:
            self.approach_fact_id = resp.fact_id

        self.move_base_as.send_goal(goal_2d, done_cb=self.done_approach_cb, feedback_cb=self.feedback_approach_cb)

        while self.move_base_as.get_state() == GoalStatus.ACTIVE or self.move_base_as.get_state() == GoalStatus.PENDING:
            if self.as_approach.is_preempt_requested():
                self.as_approach.set_preempted()
                self.move_base_as.cancel_all_goals()
                break
            if self._feedback_approach is not None:
                self.as_approach.publish_feedback(self._feedback_approach)
            robot_t, robot_r = self.tf_listener.lookupTransform("base_footprint", goal.target_pose.header.frame_id, rospy.Time(0))
            print("dist : {}, angle : {}, orientation : {}".format(math.hypot(robot_t[0], robot_t[1]), math.atan2(
                    robot_t[1], robot_t[0]), abs(t.euler_from_quaternion(robot_r)[2] - math.pi)))

            if math.hypot(robot_t[0], robot_t[1]) <= 0.8 and abs(math.atan2(
                    robot_t[1], robot_t[0])) <= 1.0:
                self.move_base_as.cancel_all_goals()
                break
            self._feedback_rate.sleep()

        robot_t, robot_r = self.tf_listener.lookupTransform("base_footprint", goal.target_pose.header.frame_id,
                                                            rospy.Time(0))

        p = rospy.Publisher("/approach_ending_pose", PoseStamped, queue_size=10)
        msg = PoseStamped()
        msg.header.frame_id = goal.target_pose.header.frame_id
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = robot_t[0]
        msg.pose.position.y = robot_t[1]
        msg.pose.position.z = robot_t[2]
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = t.quaternion_from_euler(
            0, 0, math.pi + math.atan2(
                    robot_t[1], robot_t[0])
        )
        if abs(t.euler_from_quaternion(robot_r)[2] - (math.pi + math.atan2(
                    robot_t[1], robot_t[0]))) > 0.2:
            rospy.loginfo(NODE_NAME + " Go to final oriantation")
            with self.al_motion_mtx:
                try:
                    self.al_motion.moveTo(0, 0, math.pi + math.atan2(
                        robot_t[1], robot_t[0]))
                except RuntimeError as e:
                    self.al_motion = ALProxy("ALMotion", self.nao_ip, self.nao_port)

        end_fact = rospy.ServiceProxy(STOP_FACT_SRV_NAME, EndFact)
        if self.move_to_fact_id is not None:
            resp = end_fact("base", self.move_to_fact_id)
            if resp.success:
                self.move_to_fact_id = None
        if self.approach_fact_id is not None:
            resp = end_fact("base", self.approach_fact_id)
            if resp.success:
                self.approach_fact_id = None

        self.as_approach.set_succeeded()

        self.toggle_human_monitor(True)

    def feedback_approach_cb(self, feedback):
        self._feedback_approach = feedback

    def done_approach_cb(self, state, result):
        if state == GoalStatus.PREEMPTED:
            self.as_approach.set_preempted(result)
        elif state == GoalStatus.SUCCEEDED:
            pass
        elif state == GoalStatus.ABORTED:
            self.as_approach.set_aborted(result)


#================================================

    def execute_rotate_cb(self, goal):
        """

        :param goal:
        :type goal: RotateGoal
        :return:
        """
        q = [goal.rotation.quaternion.x, goal.rotation.quaternion.y, goal.rotation.quaternion.z, goal.rotation.quaternion.w]
        t_r, r_r = self.tf_listener.lookupTransform("map", "base_footprint", rospy.Time(0))
        delta_rot = t.quaternion_multiply(q, t.quaternion_inverse(r_r))
        _, _, goal_angle = t.euler_from_quaternion(q)
        _, _, delta_angle = t.euler_from_quaternion(delta_rot)
        _, _, current_angle = t.euler_from_quaternion(r_r)
        rospy.logdebug("{} : Rotating to {}, delta : {}, current : {}.".format(NODE_NAME, goal_angle, delta_angle, current_angle))

        begin_fact = rospy.ServiceProxy(START_FACT_SRV_NAME, StartFact)
        resp = begin_fact("base", "isMoving(robot)", rospy.Time.now().to_sec(), False)
        if resp.success:
            self.move_to_fact_id = resp.fact_id
        with self.al_motion_mtx:
            try:
                self.al_motion.moveTo(0, 0, delta_angle)
            except RuntimeError as e:
                rospy.logwarn(NODE_NAME + " MoveTo crash ! Retrying...")
                self.al_motion = ALProxy("ALMotion", self.nao_ip, self.nao_port)
                self.al_motion.moveTo(0, 0, delta_angle)

        rospy.logdebug("{} : translation : {}\trotation : {}".format(NODE_NAME, t_r, r_r))
        rospy.logdebug("{} : Ending rotation.".format(NODE_NAME))

        end_fact = rospy.ServiceProxy(STOP_FACT_SRV_NAME, EndFact)
        if self.move_to_fact_id is not None:
            resp = end_fact("base", self.move_to_fact_id)
            if resp.success:
                self.move_to_fact_id = None

        self.as_rotate.set_succeeded()


server = None

def on_sigint(*_):
    global server
    server.as_approach.set_preempted()
    server.as_move_to.set_preempted()
    server.move_base_as.cancel_all_goals()
    server.toggle_human_monitor(True)
    msg = Twist()
    server.cmd_vel_topic.publish(msg)  # Stop the robot
    end_fact = rospy.ServiceProxy(STOP_FACT_SRV_NAME, EndFact)
    if server.move_to_fact_id is not None:
        resp = end_fact("base", server.move_to_fact_id)
    if server.approach_fact_id is not None:
        resp = end_fact("base", server.approach_fact_id)

if __name__ == '__main__':
    signal.signal(signal.SIGTERM, on_sigint)
    rospy.init_node(NODE_NAME)
    server = NavigationMummerAction("mummer-eth0.laas.fr", 9559)
    rospy.spin()
    server.move_base_as.cancel_all_goals()
    server.as_move_to.set_preempted()
    server.as_approach.set_preempted()
    msg = Twist()
    server.cmd_vel_topic.publish(msg)  # Stop the robot


