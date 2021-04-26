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
from quad_manipulator_msgs.srv import RecoStats, RecoStatsResponse
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Vector3, Pose
from recordtype import recordtype

Fruit = recordtype("Fruit", "position id detected distance tp_id")

APPLE_TREE_TYPE = "apple_tree"
DETECTION_TOLERANCE = 0.02

RECO_RESULTS_SERVER = '/reco_results'

class TracePoints():
    def __init__(self, topic="debug_points", frame_id="earth"):
        self.topic = topic
        self.publisher = rospy.Publisher(
            self.topic, PointCloud2, queue_size=10)
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
        return len(self.points) - 1

    def publish(self):
        try:
            rospy.logdebug('Publishing to %s', self.topic)
            header = Header()
            header.frame_id = self.frame_id
            header.stamp = rospy.Time.now()
            msg = pc2.create_cloud(header, self.fields, self.points)
            self.publisher.publish(msg)
        except struct.error as err:
            rospy.loginfo(
                'Struct error occured. Ignore this publish %s' % (str(err)))


class OrchardGroundTruth():
    def __init__(self):
        self.tp_fruits = TracePoints('/orchard/fruits')
        self.tp_plants = TracePoints('/orchard/plants')
        self.tp_detections = TracePoints('/orchard/detections')
        self.id_matched_fruit_markers = set()
        self.fruits = []
        self.fruit_cnt = 0
        model_states = rospy.wait_for_message(
            '/gazebo/model_states', ModelStates)
        self.get_model_state_proxy = rospy.ServiceProxy(
            '/gazebo/get_model_state', GetModelState)
        # Retrieve tree names
        for mn in model_states.name:
            if mn.startswith(APPLE_TREE_TYPE):
                self.load_trees(mn)
        self.publish_marker_traces()
        self.sub = rospy.Subscriber(
            "detections", Detections, self.handle_detection)
        self.reco_results_server = rospy.Service(
            RECO_RESULTS_SERVER, RecoStats, self.retrieve_reco_results)

    def load_trees(self, tree_type):
        models_dir = rospy.get_param('~models_dir')
        with open(Path(models_dir) / tree_type / 'markers.json', 'r') as markers:
            fruit_markers = json.loads(markers.read())
            resp = self.get_model_state_proxy(tree_type, "link")
            earth_t_tree = resp.pose.position
            self.tp_plants.add_point(earth_t_tree, [.3, .1, .3])
            for fruit_marker in fruit_markers:
                if fruit_marker['marker_type'] == 'FRUIT':
                    earth_t_fruit = Vector3()
                    earth_t_fruit.x = earth_t_tree.x + \
                        fruit_marker['translation'][0]
                    earth_t_fruit.y = earth_t_tree.y + \
                        fruit_marker['translation'][1]
                    earth_t_fruit.z = earth_t_tree.z + \
                        fruit_marker['translation'][2]
                    self.tp_fruits.add_point(earth_t_fruit, [1, .3, .3])
                    self.fruits.append(
                        Fruit(position=earth_t_fruit, id=self.fruit_cnt, detected=False, distance=float('inf'), tp_id=-1))
                    self.fruit_cnt += 1
    def update_marker_traces(self):
        self.tp_fruits.reset()
        for fruit in self.fruits:
            color = [0, 1, 0] if fruit.id in self.id_matched_fruit_markers else [
                1, .3, .3]
            self.tp_fruits.add_point(fruit.position, color)
        self.publish_marker_traces()

    def publish_marker_traces(self):
        self.tp_fruits.publish()
        self.tp_plants.publish()
        self.tp_detections.publish()

    def handle_detection(self, data):
        for d in data.detections:
            earth_t_fruits = np.array(
                [[f.position.x, f.position.y, f.position.z] for f in self.fruits])
            earth_t_detection = np.array(
                [d.position.x, d.position.y, d.position.z])
            detection_t_fruits = earth_t_fruits - earth_t_detection
            dists = np.linalg.norm(detection_t_fruits, axis=1)

            min_dist_ind = np.argmin(dists)
            if dists[min_dist_ind] <= DETECTION_TOLERANCE:
                match = min_dist_ind
            else:
                continue
            if match >= 0:
                if self.fruits[match].detected == False:
                    tp_id = self.tp_detections.add_point(d.position, [0, 0, 1])
                    self.fruits[match].tp_id = tp_id
                    self.fruits[match].distance = dists[match]
                    self.id_matched_fruit_markers.add(self.fruits[match].id)
                elif self.fruits[match].distance > dists[match]:
                    self.tp_detections.add_point(d.position, [0, 0, 1])
                    self.fruits[match].distance = dists[match]
        self.update_marker_traces()

    def retrieve_reco_results(self, req):
        cnt_detected_fruit = len(self.id_matched_fruit_markers)
        cnt_all_fruits = len(self.fruits)
        rospy.loginfo("Returning reco results: %d/%d."%(cnt_detected_fruit, cnt_all_fruits))
        # Reset tp detections.
        rospy.loginfo("Resetting detections")
        self.tp_detections.reset()
        self.id_matched_fruit_markers.clear()
        # Reset all fruits.
        for fruit in self.fruits:
            fruit.detected = False
            fruit.distance = float('inf')
            fruit.tp_id = -1
        return RecoStatsResponse(cnt_detected_fruit, cnt_all_fruits)

if __name__ == '__main__':
    rospy.init_node('orchard_ground_truth_manager')
    orchard_ground_truth = OrchardGroundTruth()
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        orchard_ground_truth.update_marker_traces()
        rate.sleep()
