#! /usr/bin/python

from __future__ import absolute_import

import rospy
from geometry_msgs.msg import Quaternion, Pose
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from assistive_cane.constants import BOX_SIZE_X, BOX_SIZE_Y, BOX_SIZE_Z
import copy

class WorldVisualizer():
    
    def __init__(self):

        topic = 'visualization_marker_array'
        self.publisher = rospy.Publisher(topic, MarkerArray, queue_size=10)

    def visualize_boxes(self, box_poses):

        markerArray = MarkerArray()

        # print("Number of boxes: ",len(box_poses))

        marker_id = 2

        for box_pose in box_poses:

            # print("Box Pose: ",box_pose)

            marker = Marker()
            marker.header.frame_id = "map"
            marker.type = marker.CUBE
            marker.action = marker.ADD
            marker.lifetime = rospy.Duration(0.5)
            marker.scale.x = BOX_SIZE_X
            marker.scale.y = BOX_SIZE_Y
            marker.scale.z = BOX_SIZE_Z
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.pose = box_pose

            marker.pose.position.z -= BOX_SIZE_Z/2.0

            marker.ns = "boxes"
            marker.id =  marker_id

            marker_id += 1

            markerArray.markers.append(marker)

        # Publish the MarkerArray
        self.publisher.publish(markerArray)

    def visualize_cane(self, cane_pose, current = True, marker_id = 0):

        if current:
            r = 1.0
            g = 0.0
            b = 0.0
        else:
            r = 0.0
            g = 1.0
            b = 0.0

        # print("Cane pose: ",cane_pose)

        markerArray = MarkerArray()

        marker_cane_base = Marker()
        marker_cane_base.header.frame_id = "map"
        marker_cane_base.type = marker_cane_base.CUBE
        marker_cane_base.action = marker_cane_base.ADD
        marker_cane_base.lifetime = rospy.Duration(0.5)
        marker_cane_base.scale.x = 0.1
        marker_cane_base.scale.y = 0.1
        marker_cane_base.scale.z = 1.0
        marker_cane_base.color.a = 1.0
        marker_cane_base.color.r = r
        marker_cane_base.color.g = g
        marker_cane_base.color.b = b
        marker_cane_base.pose = copy.deepcopy(cane_pose)

        marker_cane_base.pose.position.z -= 0.5

        marker_cane_base.ns = "cane"
        marker_cane_base.id = marker_id

        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = marker.ARROW
        marker.action = marker.ADD
        marker.lifetime = rospy.Duration(0.5)
        marker.scale.x = 0.1
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.pose = cane_pose

        marker.ns = "cane_top"
        marker.id = marker_id

        markerArray.markers.append(marker_cane_base)
        markerArray.markers.append(marker)

        # Publish the MarkerArray
        self.publisher.publish(markerArray)
