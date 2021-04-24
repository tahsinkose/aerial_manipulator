#!/usr/bin/env python2.7

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import SetBool, SetBoolResponse
from sensor_msgs import point_cloud2 as pc2
from geometry_msgs.msg import Point
import tf2_ros

PATTERN_CREATION_SERVER = '/pattern_creation_server'


class PatternCreationServer():
    def __init__(self):
        self.pattern_server = rospy.Service(
            PATTERN_CREATION_SERVER, SetBool, self.handle_pattern_creation)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.earth_t_trees = []

    def retrieve_trees(self, msg):
        for pt in pc2.read_points(msg):
            self.earth_t_trees.append(Point(pt[0], pt[1], pt[2]))

    def sort_trees(self, earth_t_base_link):
      earth_t_trees = np.array([[p.x, p.y] for p in self.earth_t_trees])
      tmp = earth_t_trees - np.array([earth_t_base_link.x, earth_t_base_link.y])
      trees_to_visit = np.argsort(np.linalg.norm(tmp, axis=1), axis=1)
      for ind in trees_to_visit:
        rospy.loginfo('Visit tree at 2D position %lf, %lf' %
                      (self.earth_t_trees[ind].x, self.earth_t_trees[ind].y))
    def handle_pattern_creation(self, req):
      can_transform = self.tf_buffer.can_transform(
          'earth', 'uav1/base_link', rospy.Time(0), rospy.Duration(5.0))
      if can_transform:
        earth_T_base_link = self.tf_buffer.lookup_transform(
            'earth', 'uav1/base_link', rospy.Time(0))
        self.retrieve_trees(rospy.wait_for_message(
            '/orchard/plants', PointCloud2, 1))
        earth_t_base_link = earth_T_base_link.transform.translation
        # Reset them
        self.sort_trees(earth_t_base_link)
        self.earth_t_trees = []
        return SetBoolResponse(True, "successful")
      else:
        return SetBoolResponse(False, "Either robot or earth transform is missing")


if __name__ == '__main__':
    rospy.init_node('pattern_creator')
    pattern_creation_server = PatternCreationServer()
    rospy.loginfo("Waiting for pattern creation requests")
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        rate.sleep()
