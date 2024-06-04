#!/usr/bin/env python3

import rospy
import tf

def static_transform_broadcaster():
    rospy.init_node('static_transform_broadcaster', anonymous=True)
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        # Transform from map to world
        br.sendTransform((0.0, 0.0, 0.0),  # Translation (x, y, z)
                         (0.0, 0.0, 0.0, 1.0),  # Rotation as quaternion (x, y, z, w)
                         rospy.Time.now(),
                         "world",  # Child frame
                         "map")  # Parent frame

        # Transform from world to camera_frame
        br.sendTransform((0.0, 0.0, 0.0),  # Translation (x, y, z)
                         (0.0, 0.0, 0.0, 1.0),  # Rotation as quaternion (x, y, z, w)
                         rospy.Time.now(),
                         "camera_frame",  # Child frame
                         "world")  # Parent frame

        rate.sleep()

if __name__ == '__main__':
    try:
        static_transform_broadcaster()
    except rospy.ROSInterruptException:
        pass
