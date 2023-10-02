
#! /usr/bin/python

import math
import message_filters
import rospy

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Quaternion, Pose, PoseArray
from tf.transformations import quaternion_from_euler

from assistive_cane.utils import transform_pose
from assistive_cane.constants import CAMERA_TO_BOX_TOP, CAMERA_TO_CANE_TOP
from assistive_cane.world_visualizer import WorldVisualizer

bridge = CvBridge()
visualize_detected_objects = True

"""
Code Implementation Overview
Day 1: TODO 1, TODO 2, TODO 3
Day 2: TODO 4, TODO 5
Day 3: None
Day 4: None
Day 5: Demo!
"""

def pixel2world(camera_info, image_x, image_y, orie_x, orie_y, depth):
    """
    Returns pixel location and yaw in world space using camera and depth info
    Parameters
    ----------
    camera_info: CameraInfo
        contains meta information for overhead cameras. 
        For more info see: http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/CameraInfo.html
    image_x: int
        x center coordinate of marker
    image_y: int
        y center coordinate of marker
    orie_x: int
        x orientation of marker
    orie_y: int
        y oreintatin of marker
    depth: int
        depth of pixel (x,y)
    Returns
    -------
    tuple
        a tuple of pixel location in world space and yaw (world_x, world_y, world_z, yaw)
    """

    # Get focal lengths in pixel units (fx, fy) and principal 
    # and points (cx, cy)
    # Reference: https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
    fx = camera_info.K[0]
    fy = camera_info.K[4]
    cx = camera_info.K[2]
    cy = camera_info.K[5]

    """
    [Day 2] TODO 5: Convert from pixel to world space and calculate yaw
        - Use the reference above to understand the math needed to convert
    """
    ####### Insert Code Here #######

    # Convert to world space
    world_x = (depth / fx) * (image_x - cx)
    world_y = (depth / fy) * (image_y - cy)
    world_z = depth
    ##########################

    # Calculate yaw, make sure to use radians!
    yaw = math.atan2(orie_y, orie_x)
    
    ################################

    return (world_x, world_y, world_z, yaw)


def image_callback(rgb_msg1, rgb_msg2, rgb_msg3, rgb_msg4, camera_info1, camera_info2, camera_info3, camera_info4):

    print("coming in")

    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img1 = bridge.imgmsg_to_cv2(rgb_msg1, "bgr8")
        cv2_img2 = bridge.imgmsg_to_cv2(rgb_msg2, "bgr8")
        cv2_img3 = bridge.imgmsg_to_cv2(rgb_msg3, "bgr8")
        cv2_img4 = bridge.imgmsg_to_cv2(rgb_msg4, "bgr8")
    except e:
        print(e)
    else:
        # Using this ArUCo tutorial: https://pyimagesearch.com/2020/12/21/detecting-aruco-markers-with-opencv-and-python/

        images = [cv2_img1, cv2_img2, cv2_img3, cv2_img4]
        camera_infos = [camera_info1, camera_info2, camera_info3, camera_info4]
        
        source_frames = ["camera1_color_optical_frame", "camera2_color_optical_frame", "camera3_color_optical_frame", "camera4_color_optical_frame"]
        # NOTE: Verify on end of day 1 image is displayed

        box_poses = []

        is_cane_detected = False
        cane_pose = Pose()

        for i in range(0,4):

            image = images[i]
            camera_info = camera_infos[i]

            source_frame = source_frames[i]

            """
            [Day 2] TODO 4: Detect markers
                Use the tutorial above to
                - Get an aruco dictionary
                - Get aruco parameters
                - Perform detection
            """
            ####### Insert Code Here ####### Day 2
           
            # Get dictionary of ArUco markers being used
            arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
            # Define ArUco detection parameters
            arucoParams = cv2.aruco.DetectorParameters_create()
            # Perform ArUco marker detection
            (corners, ids, rejected) = cv2.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)
            
            ################################

            # If markers detected...
            if len(corners) > 0:
                print("Detected arUco")

                # flatten the ArUco IDs list
                ids = ids.flatten()

                # loop over the detected ArUCo corners
                for (markerCorner, markerID) in zip(corners, ids):
                    
                    # extract the marker corners (which are always returned in
                    # top-left, top-right, bottom-right, and bottom-left order)
                    corners = markerCorner.reshape((4, 2))
                    (topLeft, topRight, bottomRight, bottomLeft) = corners.astype(int)

                    # compute the center (x, y)-coordinates of the ArUco marker
                    centerX = (topLeft[0] + bottomRight[0]) // 2
                    centerY = (topLeft[1] + bottomRight[1]) // 2
                    
                    # compute the top-middle (x, y)-coordinates of the ArUco marker
                    forwardX = (topLeft[0] + topRight[0]) // 2
                    forwardY = (topLeft[1] + topRight[1]) // 2

                    ### Visualization ####

                    # draw the bounding box of the ArUCo detection
                    cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
                    cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
                    cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
                    cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
                    
                    # visualize center of ArUco marker
                    cv2.circle(image, (centerX, centerY), 4, (0, 0, 255), -1)
                    
                    # visualize top of ArUco marker
                    cv2.circle(image, (forwardX, forwardY), 4, (0, 0, 255), -1)
                    
                    # draw the ArUco marker ID on the image
                    cv2.putText(image, str(markerID), (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    print("[INFO] ArUco marker ID: {}".format(markerID))   

                    # Get x and y reference vectors
                    orie_x = int((topLeft[0]+topRight[0])/2.0 - (bottomLeft[0] + bottomRight[0])/2.0)
                    orie_y = int((topLeft[1]+topRight[1])/2.0 - (bottomLeft[1] + bottomRight[1])/2.0)

                    # If marker is for obstacle (box)
                    if markerID == 1:

                        # Get world coordinates of box
                        world_x, world_y, world_z, yaw = pixel2world(camera_info, centerX, centerY, orie_x, orie_y, CAMERA_TO_BOX_TOP)

                        if world_x == None:
                            return None

                        # Convert to quaternion
                        q = quaternion_from_euler(0, 0, yaw)

                        # Create pose messsage
                        pose = Pose()
                        pose.position.x = world_x
                        pose.position.y = world_y
                        pose.position.z = world_z
                        pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

                        pose = transform_pose(pose, source_frame, "map")
                        box_poses.append(pose)

                    # If marker is for cane
                    elif markerID == 0 and is_cane_detected == False:
                        
                        is_cane_detected = True

                        # Get world coordinates of cane
                        world_x, world_y, world_z, yaw = pixel2world(camera_info, centerX, centerY, orie_x, orie_y, CAMERA_TO_CANE_TOP)

                        if world_x == None:
                            return None

                        q = quaternion_from_euler(0, 0, yaw)

                        cane_pose.position.x = world_x
                        cane_pose.position.y = world_y
                        cane_pose.position.z = world_z
                        cane_pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

                        cane_pose = transform_pose(cane_pose, source_frame, "map")

        im_v_1 = cv2.vconcat([cv2_img1, cv2_img3])
        im_v_2 = cv2.vconcat([cv2_img2, cv2_img4])
        cv_image = cv2.hconcat([im_v_1, im_v_2])
        # show the output image

        if is_cane_detected:
            cane_state_publisher.publish(cane_pose)
            if visualize_detected_objects:
                world_visualizer.visualize_cane(cane_pose)

        box_pose_array = PoseArray()
        box_pose_array.poses = box_poses

        boxes_state_publisher.publish(box_pose_array)    

        if len(box_poses) != 0 and visualize_detected_objects:
            world_visualizer.visualize_boxes(box_poses)

        # cv2.imshow("Image", cv_image)
        # cv2.waitKey(10)

        stiched_color_image_publisher.publish(bridge.cv2_to_imgmsg(cv_image, "bgr8"))

def main():
    
    """
    [Day 1] TODO 1: Write code for the following
        - Initialize a ROS node called "perceive_world"
        - Define variables for the following topics:
            - "/camera/color/image_raw"
            - "/camera/color/camera_info"
    """
    ####### Insert Code Here #######

    # Initialize node
    rospy.init_node('perceive_world')

    image_sub1 = message_filters.Subscriber("/camera1/color/image_raw", Image, queue_size = 1,  buff_size = 65536*100)
    info_sub1 = message_filters.Subscriber("/camera1/color/camera_info", CameraInfo, queue_size = 1,  buff_size = 65536*100)
    image_sub2 = message_filters.Subscriber("/camera2/color/image_raw", Image, queue_size = 1,  buff_size = 65536*100)
    info_sub2 = message_filters.Subscriber("/camera2/color/camera_info", CameraInfo, queue_size = 1,  buff_size = 65536*100)
    image_sub3 = message_filters.Subscriber("/camera3/color/image_raw", Image, queue_size = 1,  buff_size = 65536*100)
    info_sub3 = message_filters.Subscriber("/camera3/color/camera_info", CameraInfo, queue_size = 1,  buff_size = 65536*100)
    image_sub4 = message_filters.Subscriber("/camera4/color/image_raw", Image, queue_size = 1,  buff_size = 65536*100)
    info_sub4 = message_filters.Subscriber("/camera4/color/camera_info", CameraInfo, queue_size = 1,  buff_size = 65536*100)
    ts = message_filters.ApproximateTimeSynchronizer([image_sub1, image_sub2, image_sub3, image_sub4, info_sub1, info_sub2, info_sub3, info_sub4], 1, 0.2)
    ts.registerCallback(image_callback)

    global world_visualizer
    world_visualizer = WorldVisualizer()

    global boxes_state_publisher
    """
    [Day 1] TODO 2: Create boxes state publisher
        - Create a publisher with the following parameters
            - Topic: "boxes_state"
            - Message type: PoseArray
            - Queue size: 10
    """
    ####### Insert Code Here #######

    boxes_state_publisher = rospy.Publisher('boxes_state', PoseArray, queue_size=10)

    ################################

    global cane_state_publisher
    """
    [Day 1] TODO 3: Create cane state publisher
        - Create a publisher with the following parameters
            - Topic: "cane_state"
            - Message type: Pose
            - Queue size: 10
    """
    ####### Insert Code Here #######
    
    cane_state_publisher = rospy.Publisher('cane_state', Pose, queue_size=10)
    
    ################################

    global stiched_color_image_publisher
    stiched_color_image_publisher = rospy.Publisher("camera_stitch/color/image_raw", Image, queue_size=10)


    rospy.spin()

if __name__ == '__main__':
    main()