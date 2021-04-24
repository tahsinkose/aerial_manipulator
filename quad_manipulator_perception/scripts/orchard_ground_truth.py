#!/usr/bin/env python3

import rospy
from pathlib import Path
import numpy as np
import json
import struct
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2 as pc2
from std_msgs.msg import Header
from quad_manipulator_msgs.msg import Detections
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Vector3, Pose
APPLE_TREE_TYPE_1 = "apple_tree"
APPLE_TREE_TYPE_2 = "apple_tree_2"
DETECTION_TOLERANCE = 0.12

class TracePoints():
    def __init__(self, topic="debug_points", frame_id="earth"):
        self.topic = topic
        self.publisher = rospy.Publisher(self.topic, PointCloud2, queue_size=10)
        self.frame_id = frame_id

        self.fields = [PointField('x', 0, PointField.FLOAT32, 1),
                       PointField('y', 4, PointField.FLOAT32, 1),
                       PointField('z', 8, PointField.FLOAT32, 1),
                       PointField('rgba', 12, PointField.UINT32, 1),
                       ]
        self.reset()

    def reset(self):
        self.points = []

    def add_point(self, point, color):
        r = int(color[0] * 255.0)
        g = int(color[1] * 255.0)
        b = int(color[2] * 255.0)
        a = 255
        rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
        pt = [point.x, point.y, point.z, rgb]
        self.points.append(pt)

    def publish(self):
        rospy.logdebug('Publishing to %s',self.topic)
        header = Header()
        header.frame_id = self.frame_id
        header.stamp = rospy.Time.now()
        msg = pc2.create_cloud(header, self.fields, self.points)
        self.publisher.publish(msg)



class OrchardGroundTruth():
    def __init__(self):
        self.tp_fruits = TracePoints('/orchard/fruits')
        self.tp_plants = TracePoints('/orchard/plants')
        self.tp_detections = TracePoints('/orchard/detections')
        self.id_matched_fruit_markers = set()
        self.fruits = {
            'positions': [],
            'ids': []
        }
        model_states = rospy.wait_for_message('/gazebo/model_states', ModelStates)
        self.get_model_state_proxy = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.type_1_trees = []
        self.type_2_trees = []
        # Retrieve tree names
        for mn in model_states.name:
          if mn.startswith(APPLE_TREE_TYPE_2):
            self.type_2_trees.append(mn)
          elif mn.startswith(APPLE_TREE_TYPE_1):
            self.type_1_trees.append(mn)
        self.load_trees(APPLE_TREE_TYPE_1, self.type_1_trees)
        self.load_trees(APPLE_TREE_TYPE_2, self.type_2_trees)
        self.publish_marker_traces()
        self.sub = rospy.Subscriber("detections", Detections, self.handle_detection)

    def load_trees(self, tree_type, tree_holder):
      models_dir = rospy.get_param('~models_dir')
      with open(Path(models_dir) / tree_type / 'markers.json', 'r') as fruit_markers:
        markers = json.loads(fruit_markers.read())
        for tree in tree_holder:
          resp = self.get_model_state_proxy(tree, "link")
          earth_t_tree = resp.pose.position
          self.tp_plants.add_point(earth_t_tree, [.3, .1, .3])
          for fruit_marker in markers:
            if fruit_marker['marker_type'] == 'FRUIT':
              earth_t_fruit = Vector3()
              earth_t_fruit.x = earth_t_tree.x + fruit_marker['translation'][0]
              earth_t_fruit.y = earth_t_tree.y + fruit_marker['translation'][1]
              earth_t_fruit.z = earth_t_tree.z + fruit_marker['translation'][2]
              self.tp_fruits.add_point(earth_t_fruit, [1, .3, .3])

    def update_marker_traces(self):
        self.fruits['positions'] = []
        self.fruits['ids'] = []
        for marker in self.markers:
            if marker['marker_type'] == 'FRUIT':
                color = [0, 1, 0] if marker['id'] in self.id_matched_fruit_markers else [1, .3, .3]
                self.tp_fruits.add_point(marker['translation'], color)
                self.fruits['positions'].append(marker['translation'])
                self.fruits['ids'].append(marker['id'])
            elif marker['marker_type'] == 'PLANT':
                self.tp_plants.add_point(marker['translation'], [.3, .1, .3])

        self.fruits['positions'] = np.array(self.fruits['positions'])
        self.publish_marker_traces()

    def publish_marker_traces(self):
        self.tp_fruits.publish()
        self.tp_plants.publish()
        self.tp_detections.publish()

    def handle_detection(self, data):
        self.tp_detections.reset()
        for d in data.detections:
            x = d.position.pose.position.x
            y = d.position.pose.position.y
            z = d.position.pose.position.z
            self.tp_detections.add_point([x, y, z], [0, 0, 1])

            pos = np.array([x, y, z])
            dists = np.sqrt(np.sum((self.fruits['positions'] - pos) ** 2, axis=1))
            match = np.argmax(dists < DETECTION_TOLERANCE)
            if match >= 0:
                self.id_matched_fruit_markers.add(self.fruits['ids'][match])

        self.update_marker_traces()

if __name__=='__main__':
    rospy.init_node('orchard_ground_truth_manager')
    orchard_ground_truth = OrchardGroundTruth()
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        orchard_ground_truth.publish_marker_traces()
        rate.sleep()
