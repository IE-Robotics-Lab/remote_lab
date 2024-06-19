import rospy
import cv2
import cv2.aruco as aruco
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import UInt32MultiArray
import image_transport

class Tracker:
    def __init__(self):
        self.nh = rospy.NodeHandle()
        self.bridge = CvBridge()
        self.image_sub = None
        self.pub = rospy.Publisher("tracker/positions_stamped", UInt32MultiArray, queue_size=20)

        # Set aruco dictionary to use for marker detection
        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters_create()

        # Subscribe to camera
        crop = rospy.get_param("tracker/crop", False)
        if crop:
            self.image_sub = image_transport.ImageTransport(self.nh).subscribe("image_cropped", 20, self.imageCb)
        else:
            self.image_sub = image_transport.ImageTransport(self.nh).subscribe("camera/image_rect_color", 20, self.imageCb)

    def imageCb(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridgeError: {}".format(e))
            return

        # Get time stamp
        stamp = msg.header.stamp

        # List that will store detection results
        corners, ids, _ = aruco.detectMarkers(cv_image, self.dictionary, parameters=self.parameters)

        # Add time stamp and all marker positions to list
        positions_stamped = UInt32MultiArray()
        positions_stamped.data.append(stamp.secs)
        positions_stamped.data.append(stamp.nsecs)

        if ids is not None:
            for i in range(len(ids)):
                positions_stamped.data.append(ids[i][0])
                for corner in corners[i]:
                    positions_stamped.data.append(int(corner[0]))
                    positions_stamped.data.append(int(corner[1]))

        # Publish
        self.pub.publish(positions_stamped)

if __name__ == '__main__':
    rospy.init_node('tracker', anonymous=True)
    tracker_object = Tracker()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
    cv2.destroyAllWindows()
