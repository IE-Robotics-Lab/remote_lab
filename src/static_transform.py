#!/usr/bin/env python

import rospy
import tf
import os
import yaml


def load_params():
    # Get the directory of the current file
    current_dir = os.path.dirname(__file__)
    # Construct the full path to the YAML file
    yaml_file = os.path.join(current_dir, '..', 'config', 'params.yaml')

    with open(yaml_file, 'r') as file:
        params = yaml.safe_load(file)
    return params


def static_transform_broadcaster():
    params = load_params()
    rospy.init_node('static_transform_broadcaster', anonymous=True)
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(params['rate'])

    while not rospy.is_shutdown():
        # Transform from map to world
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