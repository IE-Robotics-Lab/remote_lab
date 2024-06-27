
# remote_lab

## Overview

`remote_lab` is a ROS package designed for ArUco marker detection, VRPN client integration, and camera handling using the AVT Vimba camera driver. The package includes nodes for detecting ArUco markers, transforming coordinates, and integrating with an OptiTrack system via VRPN.

## Installation

### Prerequisites

Before installing this package, make sure you have the following dependencies installed:

- ROS (Robot Operating System)
- OpenCV
- Vimba SDK (for AVT Vimba camera)
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
git clone <repository-url> remote_lab
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

#### 3. AVT Vimba Camera and Rosbridge

This launch file starts the AVT Vimba camera and the rosbridge server.

```bash
roslaunch remote_lab start_camera.launch
```

#### 2. VRPN Client

This node integrates with an OptiTrack system via VRPN.

```bash
roslaunch remote_lab optitrack.launch
```

#### 1. ArUco Marker Detection

This node detects ArUco markers in the camera feed.

```bash
roslaunch remote_lab aruco_detection.launch
```

### Configurations

The `remote_lab/config/camera_params.yaml` file contains the camera calibration parameters and other configurable settings:

```yaml
camera_matrix: [[540.734833, 0.000000, 808.435922], [0.000000, 665.360297, 596.629675], [0.000000, 0.000000, 1.000000]]
distortion_coefficients: [-0.040286, 0.011029, 0.004940, -0.001410, 0.000000]
marker_length: 0.14
allowed_marker_ids: [1, 2, 3, 4]
image_sub: "/camera/image_raw"
rate: 10
aruco_dict: "DICT_6X6_250"
tvecs_adjustment: 
  x: 2.8
  y: 0.95
  z: 2.7
```

### Launch Files

- **start_camera.launch**: Includes the AVT Vimba camera launch file and starts the rosbridge server.
- **optitrack.launch**: Launches the VRPN client node for OptiTrack integration.
- **aruco_detection.launch**: Launches the ArUco marker detection node and a static transform publisher.



## Contributing

Contributions are welcome! Please submit a pull request or open an issue to discuss changes.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

