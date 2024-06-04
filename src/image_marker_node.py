#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from aruco_msgs.msg import MarkerArray

class ImageMarkerNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/image_rect_color", Image, self.image_callback)
        self.markers_sub = rospy.Subscriber("/aruco_marker_publisher/markers", MarkerArray, self.markers_callback)
        self.image_pub = rospy.Publisher("/camera/image_markers", Image, queue_size=10)
        self.markers = []

    def markers_callback(self, data):
        self.markers = data.markers

    def image_callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        for marker in self.markers:
            corners = marker.corners
            for i in range(len(corners)):
                cv2.line(cv_image, tuple(corners[i]), tuple(corners[(i+1)%4]), (0, 255, 0), 2)
            cv2.putText(cv_image, str(marker.id), tuple(corners[0]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        image_message = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        self.image_pub.publish(image_message)

if __name__ == "__main__":
    rospy.init_node('image_marker_node', anonymous=True)
    ImageMarkerNode()
    rospy.spin()
