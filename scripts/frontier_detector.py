#!/usr/bin/env python
#coding: utf-8

import rospy
from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from coppelia_ros_robotino.msg import PointArray

import numpy as np
import cv2
import matplotlib.pyplot as plt

from argparse import ArgumentParser

class FrontierDetector:
    def __init__(self):
        self.map = None
        self.map_origin = None
        self.map_resolution = None

        self.frontiers = None

        self.robot_position = None

        map_sub = rospy.Subscriber('map', OccupancyGrid, callback=self.map_callback)
        odom_sub = rospy.Subscriber('odom', Odometry, callback=self.odom_callback)

        self.frontiers_pub = rospy.Publisher('map/frontiers', PointArray, queue_size=100)
        self.nearest_frontier_pub = rospy.Publisher('map/frontiers/nearest', PointStamped, queue_size=100)

    def map_callback(self, msg):
        rr = msg.info.height
        cc = msg.info.width
        self.map = np.reshape(msg.data, (rr, cc))
        self.map_origin = np.array([msg.info.origin.position.x, msg.info.origin.position.y])
        self.map_resolution = msg.info.resolution

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        self.robot_position = np.array([position.x, position.y])
    
    def array2point_msg(self, array, is_3d=False):
        try:
            points = []
            for p in array:
                msg = Point()
                if is_3d:
                    msg.x = array[0]
                    msg.y = array[1]
                    msg.z = array[2]
                
                msg.x = array[0]
                msg.y = array[1]

                points.append(msg)

            return points

        except TypeError:
            msg = Point()
            if is_3d:
                msg.x = array[0]
                msg.y = array[1]
                msg.z = array[2]
                return msg
            
            msg.x = array[0]
            msg.y = array[1]

            return msg

    def occmap2img(self):
        img = np.zeros(self.map.shape)
        img = (1 - (self.map/100)) * 255
        unknown = np.where(self.map == -1)
        img[unknown] = 155
        cv2.imshow('i', img.astype('uint8'))
        cv2.waitKey(10)
        return img.astype('uint8')

    def cell2position(self, cell):
        map_resolution = self.map_resolution
        pos = []
        try:
            for c in cell:
                pos.append(c * map_resolution)
        
        except TypeError:
            pos = cell * map_resolution

        return np.array(pos)

    def get_neighbours(self, ind, map_shape, n=1):
        rr, cc = ind

        rows = np.arange(rr - n, rr + n + 1)
        cols = np.arange(cc - n, cc + n + 1)

        # remove invalid indexes
        aux = np.where((rows < 0) | (rows >= map_shape[0]))
        rows = np.delete(rows, aux)
        aux = np.where((cols < 0) | (cols >= map_shape[1]))
        cols = np.delete(cols, aux)

        neighbours = np.meshgrid(cols, rows)

        rows_inds = np.concatenate(neighbours[1])
        cols_inds = np.concatenate(neighbours[0])

        neighbours = np.vstack((rows_inds, cols_inds))

        return neighbours

    def detect_frontiers(self):
        m_img = self.occmap2img()
        candidates = self.get_frontier_candidates(m_img)
        frontiers_points = self.remove_obstacles_edges(candidates, occ_thresh=55)
        # self.frontiers = frontiers_points
        self.frontiers = self.process_result(frontiers_points)
        # frontiers_lines = self.get_frontiers_lines(frontiers_points, threshold=40)
        # self.frontiers = self.draw_frontiers_lines(frontiers_lines)

    def process_result(self, frontier_points):
        img = (frontier_points * 255).astype('uint8')
        img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, np.ones((3, 3)))
        img = cv2.dilate(img, np.ones((3, 3)))
        return img / 255

    def get_frontier_candidates(self, m_img):
        img = np.copy(m_img)
        edges = cv2.Canny(img, 0, 170)
        # cv2.imshow('edges', edges)
        # cv2.imshow('edges c', 255 - edges)
        # candidates = np.where(edges != 0)
        return edges

    def remove_obstacles_edges(self, candidates, unknown_flag=-1, occ_thresh=49):
        '''
        Input: all edges returned by Canny algorithm
        Output: real frontier pixels
        '''
        m = self.map
        frontier_points = np.zeros(m.shape)
        it = np.nditer(candidates, flags=['multi_index'])
        for i in it:
            if m[it.multi_index] == unknown_flag:
                continue

            neighbours = self.get_neighbours(it.multi_index, m.shape)
            cand_region = self.map[neighbours[0], neighbours[1]]
            if np.all(cand_region < occ_thresh) and np.any(cand_region == unknown_flag):
                    frontier_points[it.multi_index] = 1
                    print(m[it.multi_index])
        
        return frontier_points

    def get_frontiers_lines(self, frontier_points, use_probabilistc=True, rho=1, th=np.pi/180, threshold=80, min_line_length=30, max_line_gap=20):
        '''
        Apply Hough Lines algorithm to get frontiers as lines instead os points
        '''
        img = (frontier_points * 255).astype('uint8')
        frontiers = cv2.HoughLinesP(img, rho, th, threshold, minLineLength=min_line_length, maxLineGap=max_line_gap)

        return frontiers

    def draw_frontiers_lines(self, lines=None):
        img = np.zeros(self.map.shape)
        if lines is not None:
            for i in range(0, len(lines)):
                l = lines[i][0]
                cv2.line(img, (l[0], l[1]), (l[2], l[3]), 255, 3, cv2.LINE_AA)

        return img

    def num_frontiers(self):
        return len(self.frontiers)

    def nearest_frontier(self):
        robot_position = self.robot_position
        if robot_position is None:
            rospy.logerr('Robot position is unknown. Cannot calculate nearest frontier.')
            return None

        frontiers_ind = np.transpose(np.where(self.frontiers != 0))
        frontiers_pos = self.cell2position(frontiers_ind)
        try:
            frontiers_pos = frontiers_pos + self.map_origin
        except ValueError:
            rospy.loginfo('No frontier detected. Therefore, could not obtain nearest frontier.')
            return None
        nearest = np.argmin(np.linalg.norm(frontiers_pos, axis=1))
        return frontiers_pos[nearest] 
    
    # TODO
    def biggest_frontier(self):
        pass


def parse_args():
    parser = ArgumentParser()
    parser.add_argument('-i', '--show-image', action='store_true', default='True', required=False, help='Show image with detected frontiers',)
    parser.add_argument('-c', '--publish-nearest', action='store_true', default=True, required=False, help='Publish closest frontier to robot.')
    return parser.parse_args()


# args = parse_args()
fig = plt.figure(figsize=(10, 5))
fig.suptitle(rospy.get_namespace().replace('/', ''))
frontiers_axes = fig.add_subplot(111)
# state_axes = fig.add_subplot(122)

node_name = "frontier_detector"
rospy.init_node(node_name)
rospy.loginfo(node_name + ' initialized.')

detector = FrontierDetector()
rate = rospy.Rate(10)
while not rospy.is_shutdown():
    if detector.map is None:
        rospy.loginfo('Waiting for map.')
        rospy.sleep(1)
        continue

    detector.detect_frontiers()
    frontiers = detector.frontiers
    
    if frontiers is None:
        rospy.loginfo('No frontiers detected.')
        rospy.sleep(1)
        continue

    # frontiers_msg = detector.array2point_msg(frontiers)
    # detector.publish_frontiers(PointArray(frontiers_msg))

    nearest = detector.nearest_frontier()
    if nearest is None:
        continue

    msg = PointStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = rospy.get_param("tf_prefix") + "/base_link"
    msg.point.x = nearest[0]
    msg.point.y = nearest[1]
    # msg.point.x = detector.map_origin[0]
    # msg.point.y = detector.map_origin[1]
    detector.nearest_frontier_pub.publish(msg)

    frontiers_img = (frontiers * 255).astype('uint8')
    frontiers_axes.imshow(frontiers_img, cmap='gray', origin='lower')
    frontiers_axes.set_title('{} frontiers'.format(rospy.get_namespace().replace('/', ''), detector.num_frontiers))

    plt.draw()
    plt.pause(0.001)
    plt.cla()

    rate.sleep()