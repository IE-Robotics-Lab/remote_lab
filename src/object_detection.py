import rospy
import tf
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
import numpy as np
from collections import deque

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

        # Buffer for smoothing
        self.pose_buffer = {marker_id: deque(maxlen=5) for marker_id in self.allowed_marker_ids}

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

                        marker_id = markerIds[i][0]

                        # Apply smoothing
                        self.pose_buffer[marker_id].append((tvecs[i], rvecs[i]))
                        tvecs_smoothed, rvecs_smoothed = self.apply_smoothing(marker_id)

                        # Publish marker pose
                        pose_msg = PoseStamped()
                        pose_msg.header.stamp = rospy.Time.now()
                        pose_msg.header.frame_id = "camera_frame"  # Ensure this frame ID matches the static transform
                        pose_msg.pose.position = Point(tvecs_smoothed[0][0], tvecs_smoothed[0][1], tvecs_smoothed[0][2])
                        rotation_matrix, _ = cv.Rodrigues(rvecs_smoothed)
                        quaternion = self.rotation_matrix_to_quaternion(rotation_matrix)
                        pose_msg.pose.orientation = Quaternion(*quaternion)

                        self.pose_pub.publish(pose_msg)

                        # Broadcast the transform
                        self.broadcast_transform(tvecs_smoothed, quaternion, marker_id)
                else:
                    rospy.loginfo("No valid markers detected")

            try:
                image_with_aruco = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
                self.image_pub.publish(image_with_aruco)
            except CvBridgeError as e:
                rospy.logerr(e)

    def apply_smoothing(self, marker_id):
        tvecs = np.mean([pose[0] for pose in self.pose_buffer[marker_id]], axis=0)
        rvecs = np.mean([pose[1] for pose in self.pose_buffer[marker_id]], axis=0)
        return tvecs, rvecs

    def rotation_matrix_to_quaternion(self, rotation_matrix):
        m = np.eye(4)
        m[:3, :3] = rotation_matrix
        return tf.transformations.quaternion_from_matrix(m)

    def broadcast_transform(self, tvec, quaternion, marker_id):
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
    rate = rospy.Rate(10)  # Adjust the rate as necessary
    try:
        while not rospy.is_shutdown():
            detector.process_images()
            rate.sleep()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
