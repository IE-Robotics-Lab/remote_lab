import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
import numpy as np
from time import sleep

class ArucoTagDetection:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image_rect_color", Image, self.image_callback)
        self.image_pub = rospy.Publisher("/image_with_aruco", Image, queue_size=10)
        self.dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_ARUCO_ORIGINAL)
        self.parameters = cv.aruco.DetectorParameters()
        self.detector = cv.aruco.ArucoDetector(self.dictionary, self.parameters)

        # Camera calibration parameters
        self.camera_matrix = np.array([[836.527947, 0, 808.422471], [0, 839.354724, 588.0755], [0, 0, 1]])  # Replace fx, fy, cx, cy with your calibration values
        self.dist_coeffs = np.array([-0.262186, 0.048066, 0.001499, -0.000339, 0.000000])  # Replace k1, k2, p1, p2, k3 with your distortion coefficients

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

            # Estimate pose of each marker
            rvecs, tvecs, _ = cv.aruco.estimatePoseSingleMarkers(markerCorners, 0.14, self.camera_matrix, self.dist_coeffs)

            # Draw axis for each marker
            for i in range(len(markerIds)):
                cv.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.05)
        else:
            rospy.loginfo("No markers detected")
            sleep(6)

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
