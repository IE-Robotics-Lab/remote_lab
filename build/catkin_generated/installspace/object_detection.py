import rospy
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
import numpy as np

class ArucoTagDetection:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/image_rect_color", Image, self.image_callback)
        self.image_pub = rospy.Publisher("/image_with_aruco", Image, queue_size=10)
        self.dictionary = cv.aruco.Dictionary_get(cv.aruco.DICT_4X4_250)
        self.parameters = cv.aruco.DetectorParameters_create()
        self.marker_publishers = {}

    def create_marker_publisher(self, marker_id):
        topic_name = f"/aruco_marker_{marker_id}"
        if topic_name not in self.marker_publishers:
            self.marker_publishers[topic_name] = rospy.Publisher(topic_name, Marker, queue_size=10)

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # Detect ArUco markers in the image
        markerCorners, markerIds, rejectedCandidates = cv.aruco.detectMarkers(cv_image, self.dictionary, parameters=self.parameters)

        if markerIds is not None:
            # Draw rectangles around the detected markers
            cv.aruco.drawDetectedMarkers(cv_image, markerCorners, markerIds)

            for marker_id, corners in zip(markerIds, markerCorners):
                self.create_marker_publisher(marker_id)
                marker = Marker()
                marker.header.frame_id = "camera_frame"  # Set the appropriate frame
                marker.header.stamp = rospy.Time.now()
                marker.ns = "aruco_markers"
                marker.id = int(marker_id)
                marker.type = Marker.LINE_STRIP
                marker.action = Marker.ADD
                marker.scale.x = 0.01  # Line width
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0

                # Add the corners to the marker
                for corner in corners[0]:
                    point = Point()
                    point.x = corner[0]
                    point.y = corner[1]
                    point.z = 0  # Assuming the marker is in a 2D plane
                    marker.points.append(point)

                # Close the loop by adding the first point again
                point = Point()
                point.x = corners[0][0]
                point.y = corners[0][1]
                point.z = 0
                marker.points.append(point)

                # Publish the marker
                self.marker_publishers[f"/aruco_marker_{marker_id}"].publish(marker)

        try:
            image_with_aruco = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.image_pub.publish(image_with_aruco)
        except CvBridgeError as e:
            rospy.logerr(e)

if __name__ == '__main__':
    rospy.init_node('aruco_tag_detection', anonymous=True)
    detector = ArucoTag
