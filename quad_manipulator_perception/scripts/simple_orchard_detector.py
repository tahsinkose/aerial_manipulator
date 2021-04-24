#!/usr/bin/env python2.7
import rospy
import tf2_ros
import tf
import cv2
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField, Image
from sensor_msgs import point_cloud2 as pc2
from cv_bridge import CvBridge
from matplotlib import pyplot as plt
from geometry_msgs.msg import Point
from std_msgs.msg import String
from quad_manipulator_msgs.msg import Detection, Detections

IMAGE_TOPIC = '/uav1/rgbd_camera/color/image_raw'
DEPTH_TOPIC = '/uav1/rgbd_camera/depth/points'
DETECTIONS_TOPIC = '/detections'
MAX_DETECTION_DISTANCE = 1.5

class MatchingMessageBuffer():
    def __init__(self):
        self.sub_img_msg = rospy.Subscriber(
            IMAGE_TOPIC, Image, self.handle_img)
        self.sub_depth_msg = rospy.Subscriber(
            DEPTH_TOPIC, PointCloud2, self.handle_depth)
        self.imgs = []
        self.depths = []
        self.max_buff = 5

    def handle_img(self, img):
        self.imgs.append(img)
        self.imgs = self.imgs[-self.max_buff:]

    def handle_depth(self, depth):
        self.depths.append(depth)
        self.depths = self.depths[-self.max_buff:]

    def get_next(self):
        if not self.imgs or not self.depths:
            return None, None

        def time_dist(img, depth):
            return np.abs(img.header.stamp.to_time() - depth.header.stamp.to_time())
        min_time_dist = float('inf')
        pair = None
        for img in self.imgs[::-1]:
            for depth in self.depths[::-1]:
                td = time_dist(img, depth)
                if td < min_time_dist:
                    min_time_dist = td
                    pair = (img, depth)
        self.imgs.remove(pair[0])
        self.depths.remove(pair[1])
        return pair


class OrchardDetector():
    def __init__(self):
        self.mmbuff = MatchingMessageBuffer()
        self.bridge = CvBridge()
        self.tfBuffer = tf2_ros.Buffer(rospy.Duration(5.0), True)
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.detect_pub = rospy.Publisher(
            DETECTIONS_TOPIC, Detections, queue_size=1)

    def mask_to_detections(self, mask, depth):
        x_cords, y_cords = np.mgrid[0:mask.shape[1],
                                    0:mask.shape[0]].astype(np.int)

        mask = mask.T.reshape(-1).astype(np.bool)
        if not mask.any():
            return []
        uvs = np.column_stack([x_cords.ravel(), y_cords.ravel()])[mask]
        cam_T_points = list(pc2.read_points(
            depth, field_names=("x", "y", "z"), uvs=uvs.tolist()))
        cam_T_points = np.array(cam_T_points)
        dists = np.sqrt(np.sum(cam_T_points ** 2, axis=1))
        cam_T_points = cam_T_points[dists < MAX_DETECTION_DISTANCE]
        # NaNs are discarded with this line.
        cam_T_points = cam_T_points[np.isfinite(cam_T_points).all(axis=1)]
        earth_T_cam = self.tfBuffer.lookup_transform(
            'earth', depth.header.frame_id, depth.header.stamp, rospy.Duration(2.0))

        t = tf.transformations
        m_translation = t.translation_matrix([
            earth_T_cam.transform.translation.x,
            earth_T_cam.transform.translation.y,
            earth_T_cam.transform.translation.z
        ])
        m_rotation = t.quaternion_matrix([
            earth_T_cam.transform.rotation.x,
            earth_T_cam.transform.rotation.y,
            earth_T_cam.transform.rotation.z,
            earth_T_cam.transform.rotation.w
        ])
        m_transform = t.concatenate_matrices(m_translation, m_rotation)

        detections = []
        for cam_T_point in cam_T_points:
            pos = m_transform.dot(
                np.array([cam_T_point[0], cam_T_point[1], cam_T_point[2], 1]))[0:3]
            msg = Detection()
            msg.position.x = pos[0]
            msg.position.y = pos[1]
            msg.position.z = pos[2]
            detections.append(msg)
        return detections

    def detect(self):
        img_msg, depth_msg = self.mmbuff.get_next()
        if img_msg is None or depth_msg is None:
            return
        img = self.bridge.imgmsg_to_cv2(
            img_msg, desired_encoding='passthrough')

        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        lower_red = np.array([0, 2, 100])
        upper_red = np.array([9, 244, 162])
        mask = cv2.inRange(hsv, lower_red, upper_red)
        # plt.imshow(mask)
        detections = self.mask_to_detections(mask, depth_msg)
        msg = Detections()
        for d in detections:
            msg.detections.append(d)
        self.detect_pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('simple_orchard_detector')
    orchard_detector = OrchardDetector()
    rospy.loginfo('Initialized orchard detector!')
    rate = rospy.Rate(10)
    rospy.sleep(rospy.Duration(2.0))
    while not rospy.is_shutdown():
        orchard_detector.detect()
        rate.sleep()
