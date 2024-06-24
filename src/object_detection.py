import rospy
import tf
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
import numpy as np

class ArucoTagDetection:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image_rect_color", Image, self.image_callback)
        self.image_pub = rospy.Publisher("/image_with_aruco", Image, queue_size=10)
        self.pose_pub = rospy.Publisher("/aruco_marker_pose", PoseStamped, queue_size=10)  # Publisher for marker poses

        self.dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_ARUCO_ORIGINAL)
        self.parameters = cv.aruco.DetectorParameters()
        self.detector = cv.aruco.ArucoDetector(self.dictionary, self.parameters)

        self.camera_matrix = np.array([[836.527947, 0, 808.422471], [0, 839.354724, 588.0755], [0, 0, 1]])  # Replace with your calibration values
        self.dist_coeffs = np.array([-0.262186, 0.048066, 0.001499, -0.000339, 0.000000])  # Replace with your distortion coefficients

        self.tf_broadcaster = tf.TransformBroadcaster()

        # Define the list of marker IDs you want to detect
        self.allowed_marker_ids = [582]  # Replace with your desired marker IDs

        self.cv_image = None

    def image_callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

    def process_images(self):
        if self.cv_image is not None:
            cv_image = self.cv_image.copy()
            markerCorners, markerIds, rejectedCandidates = self.detector.detectMarkers(cv_image)

            if markerIds is not None:
                valid_indices = [i for i, marker_id in enumerate(markerIds) if marker_id[0] in self.allowed_marker_ids]
                markerCorners = [markerCorners[i] for i in valid_indices]
                markerIds = np.array([markerIds[i] for i in valid_indices])

                if markerIds:
                    cv.aruco.drawDetectedMarkers(cv_image, markerCorners, markerIds)
                    rvecs, tvecs, _ = cv.aruco.estimatePoseSingleMarkers(markerCorners, 0.14, self.camera_matrix, self.dist_coeffs)

                    for i in range(len(markerIds)):
                        cv.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.05)

                        # Publish marker pose
                        pose_msg = PoseStamped()
                        pose_msg.header.stamp = rospy.Time.now()
                        pose_msg.header.frame_id = "camera_frame"  # Ensure this frame ID matches the static transform
                        pose_msg.pose.position = Point(tvecs[i][0][0], tvecs[i][0][1], tvecs[i][0][2])
                        rotation_matrix, _ = cv.Rodrigues(rvecs[i])
                        quaternion = tf.transformations.quaternion_from_matrix(np.vstack((np.hstack((rotation_matrix, [[0], [0], [0]])), [0, 0, 0, 1])))
                        pose_msg.pose.orientation = Quaternion(*quaternion)

                        rospy.loginfo(f'Publishing marker pose: {pose_msg}') #log coords
                        self.pose_pub.publish(pose_msg)

                        # Broadcast the transform
                        self.broadcast_transform(tvecs[i], rvecs[i], markerIds[i])
                else:
                    rospy.loginfo("No valid markers detected")

            try:
                image_with_aruco = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
                self.image_pub.publish(image_with_aruco)
            except CvBridgeError as e:
                rospy.logerr(e)

    def broadcast_transform(self, tvec, rvec, marker_id):
        # Convert rotation vector to quaternion
        rotation_matrix, _ = cv.Rodrigues(rvec)
        quaternion = tf.transformations.quaternion_from_matrix(np.vstack((np.hstack((rotation_matrix, [[0], [0], [0]])), [0, 0, 0, 1])))

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(
            (tvec[0][0], tvec[0][1], tvec[0][2]),
            quaternion,
            rospy.Time.now(),
            f"marker_{marker_id}",
            "camera_frame"  # Parent frame
        )

if __name__ == '__main__':
    rospy.init_node('aruco_tag_detection', anonymous=True)
    detector = ArucoTagDetection()
    rate = rospy.Rate(50)  # Adjust the rate as necessary
    try:
        while not rospy.is_shutdown():
            detector.process_images()
            rate.sleep()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
