import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from remote_lab.msg import Marker  # Import the custom message
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
import numpy as np
from threading import Lock

class ArucoTagDetection:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image_rect_color", Image, self.image_callback)
        self.image_pub = rospy.Publisher("/image_with_aruco", Image, queue_size=10)
        self.marker_pub = rospy.Publisher("/aruco_marker_positions", Marker, queue_size=10)  # Publisher for marker positions

        self.dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_ARUCO_ORIGINAL)
        self.parameters = cv.aruco.DetectorParameters()
        self.detector = cv.aruco.ArucoDetector(self.dictionary, self.parameters)

        self.camera_matrix = np.array([[836.527947, 0, 808.422471], [0, 839.354724, 588.0755], [0, 0, 1]])  # Replace with your calibration values
        self.dist_coeffs = np.array([-0.262186, 0.048066, 0.001499, -0.000339, 0.000000])  # Replace with your distortion coefficients

        self.marker_publishers = {}
        self.lock = Lock()

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
        markerCorners, markerIds, rejectedCandidates = self.detector.detectMarkers(cv_image)

        if markerIds is not None:
            # Draw rectangles around the detected markers
            cv.aruco.drawDetectedMarkers(cv_image, markerCorners, markerIds)

            for marker_id, corners in zip(markerIds.flatten(), markerCorners):
                self.create_marker_publisher(marker_id)
                marker = Marker()
                marker.header = Header()
                marker.header.frame_id = "camera_frame"  # Set the appropriate frame
                marker.header.stamp = rospy.Time.now()
                marker.id = int(marker_id)
                marker.position = Point(corners[0][0][0], corners[0][0][1], 0)  # Use the first corner for the position

                # Publish the marker
                self.marker_publishers[f"/aruco_marker_{marker_id}"].publish(marker)

        try:
            image_with_aruco = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.image_pub.publish(image_with_aruco)
        except CvBridgeError as e:
            rospy.logerr(e)

if __name__ == '__main__':
    rospy.init_node('aruco_tag_detection', anonymous=True)
    detector = ArucoTagDetection()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
