#!/usr/bin/env python
#coding: utf-8

import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from coppelia_ros_robotino.msg import PointArray

import numpy as np
import cv2

from argparse import ArgumentParser

class FrontierDetector:
    def __init__(self):
        self.map = None
        self.frontiers = None

        map_sub = rospy.Subscriber('map', OccupancyGrid, callback=self.map_callback)

        self.frontiers_pub = rospy.Publisher('map/frontiers', PointArray, queue_size=100)

    def map_callback(self, msg):
        rr = msg.info.height
        cc = msg.info.width
        self.map = np.reshape(msg.data, (rr, cc))

    def publish_frontiers(self, msg):
        if not isinstance(msg, PointArray):
            rospy.logerr('Frontiers message is not valid.', logger_name=rospy.get_namespace())
            return

        self.frontiers_pub.publish(msg)
    
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

    def occmap2img(self, encoding='grey'):
        img = np.zeros(self.map.shape)
        img = (100 - self.map) * 255
        unknown = np.where(self.map == -1)
        img[unknown] = 100
        return img.astype('uint8')

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

        # # remove the point itself
        # aux = np.where((neighbours[0] == rr) & (neighbours[1] == cc))
        # neighbours = np.delete(neighbours, aux, axis=1)

        return neighbours

    def get_frontier_candidates(self, m_img):
        img = np.copy(m_img)
        edges = cv2.Canny(img, 0, 100)
        candidates = np.where(edges != 0)
        return candidates

    def get_real_frontiers(self, candidates, unknown_flag=-1, threshold = 49):
        '''
        Input: all possible frontier pixels
        Output: real frontier pixels
        '''

        frontier_map = np.zeros(self.map.shape)
        it_r = np.nditer(candidates[0])
        it_c = np.nditer(candidates[1])
        for rr, cc in zip(it_r, it_c):
            candidate = self.map[rr, cc]
            neighbours = self.get_neighbours((rr, cc), self.map.shape, n=1)
            candidate_region = self.map[neighbours[0], neighbours[1]]
            if candidate < threshold and np.any(candidate_region == unknown_flag):
                    frontier_map[rr, cc] = 1
        
        return frontier_map

    def closest_frontier(self, robot_position):
        frontiers = self.frontiers
        closest = np.argmin(np.linalg.norm(robot_position - frontiers))
        return frontiers[closest]
    
    # TODO
    def biggest_frontier(self):
        pass

    def detect_frontiers(self):
        m_img = self.occmap2img(self.map)
        candidates = self.get_frontier_candidates(m_img)
        self.frontiers = self.get_real_frontiers(candidates)
        return self.frontiers

def parse_args():
    parser = ArgumentParser()
    parser.add_argument('-i', '--show-image', action='store_true', default='True', required=False, help='Show image with detected frontiers',)
    parser.add_argument('-c', '--publish-closest', action='store_true', default=True, required=False, help='Publish closest frontier to robot.')
    return parser.parse_args()


# args = parse_args()

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

    frontiers = detector.detect_frontiers()
    frontiers = cv2.dilate((frontiers*255).astype('uint8'), np.ones((3, 3)))
    frontiers_msg = detector.array2point_msg(frontiers)
    detector.publish_frontiers(PointArray(frontiers_msg))

    # if args.show_image:
    #     cv2.imshow('{}\'s frontiers'.format(rospy.get_namespace().replace('/', '')), frontiers)
    #     cv2.waitKey(10)

    cv2.imshow('{}\'s frontiers'.format(rospy.get_namespace().replace('/', '')), frontiers)
    cv2.waitKey(10)

    teste = np.zeros((1000, 1000, 3))
    lines = cv2.HoughLinesP((frontiers*255).astype('uint8'), 1, np.pi / 180, 80)
    if lines is not None:
        print(rospy.get_namespace() + str(len(lines)))
        for i in range(0, len(lines)):
            l = lines[i][0]
            cv2.line(teste, (l[0], l[1]), (l[2], l[3]), (0, 255, 0), 3, cv2.LINE_AA)

    cv2.imshow('Lines', teste)
    rate.sleep()