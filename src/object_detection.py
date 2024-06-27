import rospy
import tf
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
import numpy as np
from collections import deque
import yaml
import os

def load_params():
    # Get the directory of the current file
    current_dir = os.path.dirname(__file__)
    # Construct the full path to the YAML file
    yaml_file = os.path.join(current_dir, '..', 'config', 'params.yaml')

    with open(yaml_file, 'r') as file:
        params = yaml.safe_load(file)
    return params

class ArucoTagDetection:
    def __init__(self):
        params = load_params()
        rospy.loginfo(f"parameters loaded are: {params}")

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(params['image_sub'], Image, self.image_callback) #Camera image feed to take from
        self.image_pub = rospy.Publisher("/image_with_aruco", Image, queue_size=10) # Image topic with marker detection
        self.pose_pub = rospy.Publisher("/aruco_marker_pose", PoseStamped, queue_size=10) # Publisher for marker poses

        self.dictionary = cv.aruco.getPredefinedDictionary(eval(f"cv.aruco.{params['aruco_dict']}")) # Aruco Dictionary
        self.allowed_marker_ids = params['allowed_marker_ids'] # Allowed Markers IDs
        self.marker_length = params['marker_length'] # Marker side length

        self.parameters = cv.aruco.DetectorParameters()
        self.detector = cv.aruco.ArucoDetector(self.dictionary, self.parameters)

        self.camera_matrix = np.array(params['camera_matrix'])  # Replace with your calibration values
        self.dist_coeffs = np.array(params['distortion_coefficients'])  # Replace with your distortion coefficients

        self.tf_broadcaster = tf.TransformBroadcaster()
        self.ros_rate = params['rate'] # Broadcast rate

        # Define the list of marker IDs you want to detect
        # Replace with your desired marker IDs

        self.cv_image = None

        # Buffer for smoothing, increase deque max length for more smoothing
        self.pose_buffer = {marker_id: deque(maxlen=5) for marker_id in self.allowed_marker_ids}

    def image_callback(self, data): # Image callback function to covert from a ROS image to an OpenCV image
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)


    def ensure_z_axis_up(self, rvec): #Helper function to fix Z axis as always facing the camera on RViz
        # Convert the rotation vector to a rotation matrix
        rotation_matrix, _ = cv.Rodrigues(rvec)

        # Define the desired z-axis direction (pointing up)
        desired_z = np.array([0, 0, 1])

        # Overwrite the current z-axis with the desired z-axis
        rotation_matrix[:, 2] = desired_z

        # Ensure the rotation matrix remains orthogonal
        rotation_matrix[:, 0] = np.cross(rotation_matrix[:, 1], rotation_matrix[:, 2])
        rotation_matrix[:, 0] /= np.linalg.norm(rotation_matrix[:, 0])
        rotation_matrix[:, 1] = np.cross(rotation_matrix[:, 2], rotation_matrix[:, 0])
        rotation_matrix[:, 1] /= np.linalg.norm(rotation_matrix[:, 1])

        # Convert the adjusted rotation matrix back to a rotation vector
        rvec, _ = cv.Rodrigues(rotation_matrix)

        return rvec

    def apply_smoothing(self, marker_id): # Applying smoothing by taking the mean of the last 5 vectors to remove any jittering
        tvecs = np.mean([pose[0] for pose in self.pose_buffer[marker_id]], axis=0)
        rvecs = np.mean([pose[1] for pose in self.pose_buffer[marker_id]], axis=0)
        return tvecs, rvecs

    def rotation_matrix_to_quaternion(self, rotation_matrix): # Converting rvecs into quaternions for RViz
        # Create a 4x4 identity matrix
        m = np.eye(4)
        # Insert 3x3 rotation matrix into top left corner of identity matrix
        m[:3, :3] = rotation_matrix
        #4x4 matrix to quaternion
        return tf.transformations.quaternion_from_matrix(m)

    def broadcast_transform(self, tvec, quaternion, marker_id):
        # Broadcast the transform
        self.tf_broadcaster.sendTransform(
            (tvec[0][0], tvec[0][1], tvec[0][2]),
            quaternion,
            rospy.Time.now(),
            f"marker_{marker_id}", #frame for each marker
            "camera_frame"  # Parent frame, ensure this frame ID matches the static transform
        )

    def process_images(self):
        if self.cv_image is not None:
            cv_image = self.cv_image.copy()
            # Detect Markers in the image
            markerCorners, markerIds, rejectedCandidates = self.detector.detectMarkers(cv_image)

            if markerIds is not None:
                # Filter out markers not in the allowed list
                valid_indices = [i for i, marker_id in enumerate(markerIds) if marker_id[0] in self.allowed_marker_ids]
                markerCorners = [markerCorners[i] for i in valid_indices]
                markerIds = np.array([markerIds[i] for i in valid_indices])

                if markerIds:
                    # Draw detected markers on the image topic
                    cv.aruco.drawDetectedMarkers(cv_image, markerCorners, markerIds)
                    try:
                        # Estimate the pose of the detected markers
                        rvecs, tvecs, _ = cv.aruco.estimatePoseSingleMarkers(markerCorners, self.marker_length, self.camera_matrix, self.dist_coeffs)
                    except Exception as e:
                        rospy.logerr("Pose estimation error: %s", e)
                        return

                    for i in range(len(markerIds)):
                        try:
                            # Draw coordinate axes on the markers
                            cv.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.05)
                        except Exception as e:
                            rospy.logerr("Error drawing axes: %s", e)
                            continue

                        marker_id = markerIds[i][0]

                        # Ensure z axis is always facing up on RViz
                        rvecs[i] = self.ensure_z_axis_up(rvecs[i]).reshape((3,))

                        #Adjusting translation vectors based on camera position
                        tvecs[i][0][0] += 2.8
                        tvecs[i][0][1] += 0.95
                        tvecs[i][0][2] = 2.7 - tvecs[i][0][2]
                        if tvecs[i][0][2] < 0:
                            tvecs[i][0][2] = 0

                        # Apply smoothing
                        self.pose_buffer[marker_id].append((tvecs[i], rvecs[i]))
                        tvecs_smoothed, rvecs_smoothed = self.apply_smoothing(marker_id)

                        # Publish PoseStamped message for the marker
                        pose_msg = PoseStamped()
                        pose_msg.header.stamp = rospy.Time.now()
                        pose_msg.header.frame_id = "camera_frame"  # Ensure this frame ID matches the static transform
                        pose_msg.pose.position = Point(tvecs_smoothed[0][0], tvecs_smoothed[0][1], tvecs_smoothed[0][2])
                        rotation_matrix, _ = cv.Rodrigues(rvecs_smoothed)
                        quaternion = self.rotation_matrix_to_quaternion(rotation_matrix)
                        pose_msg.pose.orientation = Quaternion(*quaternion)

                        self.pose_pub.publish(pose_msg) #Publish pose message into a topic

                        # Broadcast the transform
                        self.broadcast_transform(tvecs_smoothed, quaternion, marker_id)
                else:
                    rospy.loginfo("No valid markers detected")

            try:
                image_with_aruco = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
                self.image_pub.publish(image_with_aruco)
            except CvBridgeError as e:
                rospy.logerr(e)


if __name__ == '__main__':
    rospy.init_node('aruco_tag_detection', anonymous=True)
    detector = ArucoTagDetection()
    rate = rospy.Rate(detector.ros_rate)  # Adjust the rate as necessary
    try:
        while not rospy.is_shutdown():
            detector.process_images()
            rate.sleep()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
