import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
import numpy as np
from threading import Thread, Lock
import queue

class ArucoTagDetection:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image_rect_color", Image, self.image_callback)
        self.image_pub = rospy.Publisher("/image_with_aruco", Image, queue_size=10)
        self.dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_ARUCO_ORIGINAL)
        self.parameters = cv.aruco.DetectorParameters()
        self.detector = cv.aruco.ArucoDetector(self.dictionary, self.parameters)

        self.camera_matrix = np.array([[836.527947, 0, 808.422471], [0, 839.354724, 588.0755], [0, 0, 1]])  # Replace with your calibration values
        self.dist_coeffs = np.array([-0.262186, 0.048066, 0.001499, -0.000339, 0.000000])  # Replace with your distortion coefficients

        self.image_queue = queue.Queue(maxsize=1)
        self.lock = Lock()

        self.processing_thread = Thread(target=self.process_images)
        self.processing_thread.daemon = True
        self.processing_thread.start()

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        with self.lock:
            if not self.image_queue.full():
                self.image_queue.put(cv_image)

    def process_images(self):
        while not rospy.is_shutdown():
            if not self.image_queue.empty():
                with self.lock:
                    cv_image = self.image_queue.get()

                markerCorners, markerIds, rejectedCandidates = self.detector.detectMarkers(cv_image)

                if markerIds is not None:
                    cv.aruco.drawDetectedMarkers(cv_image, markerCorners, markerIds)
                    rvecs, tvecs, _ = cv.aruco.estimatePoseSingleMarkers(markerCorners, 0.14, self.camera_matrix, self.dist_coeffs)

                    for i in range(len(markerIds)):
                        cv.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.05)
                else:
                    rospy.loginfo("No markers detected")

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
