
# remote_lab

## Overview

`remote_lab` is a ROS package designed for ArUco marker detection, OptiTrack integration, and camera handling using the AVT Vimba camera driver for use at the IE Robotics Lab. The package detects ArUco Markers, publishes their poses with TF so they can be visualized with RViz, and aligns the camera frame with the OptiTrack's world frame.

## Installation

### Prerequisites

Before installing this package, make sure you have the following dependencies installed:

- ROS (Robot Operating System)
- OpenCV
- Vimba SDK for AVT Vimba camera ([refer to this guide](https://github.com/astuff/avt_vimba_camera))
- Python packages: `rospy`, `cv_bridge`, `tf`, `sensor_msgs`, `std_msgs`, `geometry_msgs`, `image_transport`, `message_generation`
- ROS packages: `vrpn_client_ros`, `rosbridge_server`, `rosapi`, `avt_vimba_camera`

### Installing Dependencies

```bash
# Update and install dependencies
sudo apt-get update
sudo apt-get install ros-noetic-cv-bridge ros-noetic-tf ros-noetic-geometry-msgs ros-noetic-image-transport ros-noetic-message-generation ros-noetic-vrpn-client-ros ros-noetic-rosbridge-server ros-noetic-rosapi

# Install Vimba SDK (follow AVT Vimba camera's official installation guide)
```

Replace `noetic` with your ROS distribution

### Creating a Catkin Workspace

If you don't already have a catkin workspace, you can create one by following these steps:

```bash
# Create the workspace directory
mkdir -p ~/catkin_ws/src

# Navigate to the workspace directory
cd ~/catkin_ws/

# Initialize the workspace
catkin_make

# Source the workspace
source devel/setup.bash
```

### Cloning the Repository

1. Clone the repository into your catkin workspace:

```bash
cd ~/catkin_ws/src
git clone https://github.com/IE-Robotics-Lab/remote_lab.git
```

2. Build the workspace:

```bash
cd ~/catkin_ws
catkin_make
```

3. Source the workspace:

```bash
source devel/setup.bash
```

## Usage

### Running the Nodes

#### 1. AVT Vimba Camera and Rosbridge

The `start_camera.launch` launch file starts the AVT Vimba camera and the rosbridge server in the same node.

```bash
roslaunch remote_lab start_camera.launch
```

#### 2. OptiTrack Client

The `optitrack.launch` launch file starts the OptiTrack node which publishes the positions of the markers and the world frame, you can find more info [here](https://github.com/IE-Robotics-Lab/optitrack-ros-data-publisher).

```bash
roslaunch remote_lab optitrack.launch
```

#### 3. ArUco Marker Detection

The `aruco_detection.launch` launch file starts the ArUco detecion node which detects ArUco markers in the camera feed and publishes their poses to TF.

```bash
roslaunch remote_lab aruco_detection.launch
```

### Configurations

The `camera_calibration.yaml` file contains the camera calibration data which is obtained by following [this guide](https://github.com/IE-Robotics-Lab/camera_calibration_guide). You will need to calibrate the camera again using the rectified image feed instead of the raw feed to obtain the camera matrix and distortion coefficients needed to track the aruco markers properly.

The `remote_lab/config/params.yaml` file contains the parameters needed for the ArUco tag detection and other configurable settings:

```yaml
# Camera Matrix and Distortion Coefficients here are obtained by calibrating the rectified image view
camera_matrix: [[540.734833, 0.000000, 808.435922], [0.000000, 665.360297, 596.629675], [0.000000, 0.000000, 1.000000]]
distortion_coefficients: [-0.040286, 0.011029, 0.004940, -0.001410, 0.000000]

image_sub: "/camera/image_rect_color" # Rectified image topic
rate: 50 # ROS publish rate

aruco_dict: "DICT_ARUCO_ORIGINAL" # ArUco marker dictionary
allowed_marker_ids: [582] # Allowed ArUco marker IDs
marker_length: 0.14 # Marker side length in meters

# Coordinates are obtained by measuring distance in meters from optitrack origin to the location of the camera in the real world
camera_location:
  x_coord: 2.8
  y_coord: 0.95
  z_coord: 2.7
```

## Contributing

Contributions are welcome! Please submit a pull request or open an issue to discuss changes.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

