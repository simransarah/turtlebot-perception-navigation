# TurtleBot3 Object Detection

This ROS package enables a TurtleBot3 Waffle to perform real-time object detection using the YOLOv8 model. 

## Prerequisites

- Ubuntu 20.04  
- ROS Noetic  
- Git and Git LFS  
- Python 3.8+ and Pip  

## Setup Instructions

### 1. Clone the Repository

Navigate to your workspace's `src` directory and clone the repository:

```bash
cd ~/catkin_ws/src/
git clone https://github.com/simransarah/turtlebot-perception-navigation.git
```

### 3. Install Git LFS and Pull Model Files

```bash
cd turtlebot-perception-navigation
git lfs install
git lfs pull
```

### 4. Create and Install Python Dependencies

Install dependencies:

```bash
cd ~/catkin_ws/
pip3 install -r src/turtlebot-perception-navigation/requirements.txt
```

### 5. Install ROS Dependencies

```bash
cd ~/catkin_ws/
rosdep install --from-paths src --ignore-src -r -y
```

### 6. Build the Workspace

```bash
cd ~/catkin_ws/
catkin_make
```

## Usage

### 1. Launch the System

To run the object detection node, use the launch file that starts the Gazebo simulation and the detection node

```bash
cd ~/catkin_ws/
source devel/setup.bash
roslaunch turtlebot-perception-navigation turtlebot_object_detection.launch
```

### 2. Verify the Output

To verify that the node is working:

- **Visualisation**: View the camera feed with detection boxes:

```bash
rqt_image_view /annotated_image
```

