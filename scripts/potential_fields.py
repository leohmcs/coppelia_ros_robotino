#!/usr/bin/env python
#coding: utf-8

import rospy
import tf.transformations as tft
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from interconnected_robots.msg import *
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import tf2_ros

import numpy as np
import matplotlib.pyplot as plt


class PotentialFields:
    def __init__(self, ns, connection_radius):
        # tf
        self.NAMESPACE = ns
        self.TF_PREFIX = ns + '_tf/'
        self.tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # initialize pose and velocity variables
        self.pose = None
        self.goal = None
        self.curr_vel = np.array([0.0, 0.0, 0.0])   # vx, vy, w
        self.mates_poses = {} # positions of other robots
        
        self.max_linear_speed = 0.3
        self.min_linear_speed = 0.1
        self.max_angular_speed = np.deg2rad(30)

        self.CONNECTION_RADIUS = connection_radius
        self.MAX_SENSOR_RANGE = 4

        # list of detected obstacles' positions
        self.obs_pos = []    # [x, y]

        # ros publishers and subscribers
        goal_sub = rospy.Subscriber('goal', PoseStamped, callback=self.goal_callback)
        laser_sub = rospy.Subscriber('scan', LaserScan, callback=self.laser_callback)
        odom_sub = rospy.Subscriber('odom', Odometry, callback=self.odom_callback)
        vel_sub = rospy.Subscriber('cmd_vel', Twist, callback=self.vel_callback)

        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=100)

    def goal_callback(self, msg):
        source_frame = msg.header.frame_id
        target_frame = self.TF_PREFIX + 'odom'
        try:
            tf = self.tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time.now())
            pos = msg.pose.position
            self.goal = np.array([pos.x + tf.transform.translation.x, pos.y + tf.transform.translation.y])

            # in case the orientation is needed
            # ori = msg.pose.orientation
            # ...

        except Exception as e:
            rospy.logerr('Error transforming goal from frame {} to {}: {}'.format(source_frame, target_frame, str(e)))

    def laser_callback(self, msg):
        if self.pose is None:
            return

        robot_pos = self.pose[:2]
        robot_ori = self.pose[2]

        aux = []
        nearest_obs_point = []

        ranges = np.array(msg.ranges)
        it = np.nditer(ranges, flags=['c_index'])
        for r in it:
            if r >= self.MAX_SENSOR_RANGE or it.index == ranges.shape[0] - 1:
                if len(aux) > 15:
                    aux = np.array(aux)
                    min_dist_reading = aux[np.argmin(aux[:, 1])]
                    th, r = min_dist_reading[0], min_dist_reading[1]
                    reading_pos = [r * np.cos(th + robot_ori), r * np.sin(th + robot_ori)] + robot_pos
                    nearest_obs_point.append(reading_pos)
                    aux = []

                continue
            
            ang = (it.index * msg.angle_increment) + msg.angle_min
            aux.append([ang, r])
        
        self.obs_pos = np.array(nearest_obs_point)

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        quat = msg.pose.pose.orientation
        euler = tft.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        self.pose = np.array([pos.x, pos.y, euler[2]])

    def vel_callback(self, msg):
        self.curr_vel = np.array([msg.linear.x, msg.linear.y, msg.angular.z])

    def publish_velocity(self, vel):
        speed = np.linalg.norm(vel[:2])
        if speed > self.max_linear_speed and speed > 0:
            vel[:2] = vel[:2] / speed * self.max_linear_speed
        elif speed < self.min_linear_speed and speed > 0:
            vel[:2] = vel[:2] / speed * self.min_linear_speed

        msg = Twist()
        msg.linear.x = vel[0]
        msg.linear.y = vel[1]
        msg.angular.z = vel[2]
        
        self.vel_pub.publish(msg)

    def num_obs(self, obs_pos):
        return len(obs_pos)
    
    def ang(self, a, b):
        '''
        Returns the angle between vectors a and b
        '''
        dot = np.dot(a, b)
        norm_a = np.linalg.norm(a)
        norm_b = np.linalg.norm(b)
        if norm_a == 0 or norm_b == 0:
            rospy.logwarn('Found 0-length vector when calculating angle between vectors')
            return np.inf
        return np.arccos(dot / (norm_a * norm_b))

    def go(self, goal_err=0.7):
        '''
        Move to goal using potential fields
        Input 
        - min_goal_err: minimum distance to goal to consider the robot has arrived
        '''
        pose = self.pose
        goal = self.goal

        if pose is None:
            rospy.logerr('{}\'s pose is unknown. Cannot move.'.format(self.NAMESPACE))
            rospy.sleep(1)
            return

        if goal is None:
            rospy.loginfo('{} is waiting for goal'.format(self.NAMESPACE))
            rospy.sleep(1)
            return

        dist_to_goal = np.linalg.norm(pose[:2] - goal)
        while dist_to_goal > goal_err:
            v = self.linear_velocity(pose[:2], goal)  # [dx, dy]
            # w = self.angular_velocity(pose[:2], goal) # [dth]
            w = 0
            q = np.append(v, w)
            self.publish_velocity(q)

            # update distance to goal
            dist_to_goal = np.linalg.norm(self.pose[:2] - goal)

    def linear_velocity(self, robot_pos, goal):
        return self.resultant_force(robot_pos, goal)

    def angular_velocity(self, robot_pos, goal):
        target_ori = self.ang(goal - robot_pos, np.array([1, 0]))
        dth = target_ori - robot_pos
        dist_to_goal = np.linalg.norm(self.pose[:2] - self.goal)
        dt = self.max_linear_speed * dist_to_goal
        return dth / dt

    # def rep_gain(self, x, R, a=2, b=2):
    #     k = -a * (np.e ** -x)/(x * (x - R)) + b
    #     dk = np.e**-x * (x**2 - R*x + 2*x - R)/((x**2 - R*x)**2)
    #     dir = -np.sign(dk)
    #     return k * dir

    def resultant_force(self, robot_pos, goal):
        att = self.att_force(robot_pos, goal)
        rep = self.rep_force(robot_pos, self.obs_pos, min_dist=self.MAX_SENSOR_RANGE)
        res = att + rep
        # res += self.noise_force(att, rep)     more tests needed

        print('{} att:{} rep: {}'.format(rospy.get_namespace(), np.linalg.norm(att), np.linalg.norm(rep)))
        # self.animate(att, rep, res)
        return res

    def att_force(self, robot_pos, goal, katt=.2):        
        return katt * (goal - robot_pos)

    def rep_force(self, robot_pos, obs_pos, krep=7, min_dist=3):
        forces = np.array([0.0, 0.0])

        for p in obs_pos:
            d = np.linalg.norm(p - robot_pos)
            if d > min_dist:
                forces += [0.0, 0.0]
            else:
                force = (krep / d**2) * ((1/d) - (1/min_dist)) * (robot_pos - p)/d
                forces += force
        
        return np.array(forces)

    def noise_force(self, att, rep, deg_bound=1):
        '''
        If necessary, adds an extra noise to help the robot escape local minima when the angle between repulsion 
        and atraction is around 180 degrees.

        The noise is in the direction the robot was moving and the module is the maximum robot speed.

        Input: attraction force, repulsion force, minimun bound between attraction and repulsion
        Output: TODO explain output
        '''
        if np.linalg.norm(rep) == 0:
            # there is no need to add noise if there is no repulsion
            return np.array([0.0, 0.0])
        
        rad_bound = np.deg2rad(deg_bound)
        if -rad_bound < np.abs(self.ang(att, rep)) - np.pi < rad_bound:
            prev_lin_vel = self.curr_vel[:2]
            return self.max_linear_speed * (prev_lin_vel) / np.linalg.norm(prev_lin_vel)

    def animate(self, att_force, rep_force, res_force):
        plt.cla()
        plt.quiver(self.pose[:2][0], self.pose[:2][1], att_force[0], att_force[1], color='g', label="Attraction")
        
        plt.quiver(self.pose[:2][0], self.pose[:2][1], rep_force[0], rep_force[1], color='r', label="Repulsion")
        
        # plt.quiver(self.pose[:2][0], self.pose[:2][1], res_force[0], res_force[1], color='purple', label="Resultant Force")

        plt.plot(self.pose[:2][0], self.pose[:2][1], 'bo', label="Robot Position")
        plt.plot(self.goal[0], self.goal[1], 'go', label="Goal")
        obstacles = np.transpose(self.obs_pos)
        plt.scatter(obstacles[0], obstacles[1], color='r', label="Obstacles")

        plt.title(self.NAMESPACE)
        plt.legend()
        pad = 8
        width = (self.pose[:2][0] - pad, self.pose[:2][0] + pad)
        height = (self.pose[:2][1] - pad, self.pose[:2][1] + pad) 
        plt.axis((width[0], width[1], height[0], height[1]), 'equal')
        plt.draw()
        plt.pause(0.001)



rospy.init_node('potential_fields')

ns = rospy.get_namespace().replace('/', '')
connection_radius = rospy.get_param('connection_radius', default=4)
node = PotentialFields(ns, connection_radius)

rate = rospy.Rate(1)
while not rospy.is_shutdown():
    node.go()
    rate.sleep()