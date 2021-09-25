#!/usr/bin/env python
# coding: utf-8

import rospy
from geometry_msgs.msg import PoseStamped, Vector3, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

import tf

import numpy as np

import time


class Odom:
    def __init__(self):
        L = 0.135   # Meters
        r = 0.040   # Meters
        # Kinematic model
        self.Mdir = np.array([[-r/np.sqrt(3), 0, r/np.sqrt(3)], [r/3, -2*r/3, r/3], [r/(3*L), r/(3*L), r/(3*L)]])

        self.pose_covariance = [0, 0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0]

        self.twist_covariance = [0, 0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0]

        self.tf_prefix = ""
        self.odom_broadcaster = tf.broadcaster.TransformBroadcaster()

        self.pose_stamped = PoseStamped()
        self.joint0_state = JointState()
        self.joint1_state = JointState()
        self.joint2_state = JointState()

        pose_stamped_sub = rospy.Subscriber('gt_relative_pose', PoseStamped, self.pose_stamped_callback)
        joint0_state_sub = rospy.Subscriber('joint0_state', JointState, self.joint0_state_callback)
        joint1_state_sub = rospy.Subscriber('joint1_state', JointState, self.joint1_state_callback)
        joint2_state_sub = rospy.Subscriber('joint2_state', JointState, self.joint2_state_callback)

        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=100)

    def pose_stamped_callback(self, msg):
        self.pose_stamped = msg

    def joint0_state_callback(self, msg):
        self.joint0_state = msg

    def joint1_state_callback(self, msg):
        self.joint1_state = msg

    def joint2_state_callback(self, msg):
        self.joint2_state = msg

    def get_velocity(self):
        w0, w1, w2 = 0., 0., 0.
        try:
            w0 = self.joint0_state.velocity[0]
            w1 = self.joint1_state.velocity[0]
            w2 = self.joint2_state.velocity[0]
        except IndexError:
            rospy.logwarn("Could not index joint state velocity. Probably no message was received yet.")
            time.sleep(1)
            

        w_vec = np.array([w0, w1, w2])

        q_vel = np.matmul(self.Mdir, w_vec)

        return q_vel

    def publish_odom(self):
        stamp = rospy.Time.now()
        pose = self.pose_stamped.pose
        vel = self.get_velocity()
        
        pos = np.array([pose.position.x, pose.position.y, 0.])
        quat = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        self.odom_broadcaster.sendTransform (
            pos,
            quat,
            stamp,
            self.tf_prefix + "base_link",
            self.tf_prefix + "odom"
        )

        # fill message
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.tf_prefix + "odom"

        msg.child_frame_id = self.tf_prefix + "base_link"

        msg.pose.pose = pose
        msg.pose.covariance = self.pose_covariance

        msg.twist.twist = Twist(Vector3(vel[0], vel[1], 0.), Vector3(0., 0., vel[2]))
        msg.twist.covariance = self.twist_covariance

        self.odom_pub.publish(msg)


node = Odom()
node_name = 'odom'
rospy.init_node(node_name)
rospy.loginfo(node_name + " inicializado com sucesso.")

node.tf_prefix = rospy.get_param("tf_prefix") + "/"

rate = rospy.Rate(10)
while not rospy.is_shutdown():
     node.publish_odom()
     rate.sleep()