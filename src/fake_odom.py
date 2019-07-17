#!/usr/bin/env python
import rospy
import tf2_ros
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


NODE_NAME = "fake_odom_node"
SPEED_SUB_TOPIC = "/cmd_vel"
FRAME_ID = "odom"
CHILD_FRAME_ID = "base_footprint"
ODOM_PERIOD = 0.050  # s

ODOM_PUB_TOPIC = "/fake_odom"


class FakeOdom:
    def __init__(self):
        self.last_speed = (0, 0, 0)
        self.odom_pub = rospy.Publisher(ODOM_PUB_TOPIC, Odometry, queue_size=1)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.speed_sub = rospy.Subscriber(SPEED_SUB_TOPIC, Twist, self.onSpeedReceive)
        self.timer = rospy.Timer(rospy.Duration(ODOM_PERIOD), self.onTimer)

    def onSpeedReceive(self, speed):
        """

        :param speed:
        :type speed: Twist
        :return:
        """
        self.last_speed = (speed.linear.x, speed.linear.y, speed.angular.z)

    def onTimer(self, timer_event):
        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = FRAME_ID
        msg.child_frame_id = CHILD_FRAME_ID
        msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z = self.last_speed
        t = self.tf_buffer.lookup_transform(FRAME_ID, CHILD_FRAME_ID, rospy.Time(0))
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = t.transform.translation.x, t.transform.translation.y, t.transform.translation.z
        msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w
        self.odom_pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node(NODE_NAME)
    fo = FakeOdom()
    rospy.spin()

