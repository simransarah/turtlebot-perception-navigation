# TurtleBot Perception and Navigation

This ROS package enables a TurtleBot3 Waffle to perform real-time object detection using a YOLOv11 model and to be controlled via natural language voice commands processed by Azure Cognitive Services.

The system can:

  * Detect and locate objects in 3D space.
  * Understand commands like "go to the chair" or "look to the left of the bottle".
  * Navigate towards objects while performing basic obstacle avoidance.
  * Execute simple movements like "go forward" or "turn left".

## Prerequisites

  - Ubuntu 20.04
  - ROS Noetic
  - Git and Git LFS
  - Python 3.8+ and Pip
  - An Azure account with **Speech** and **Conversational Language Understanding (CLU)** services set up.

## Setup Instructions

### 1\. Clone the Repository

Navigate to your catkin workspace's `src` directory and clone the repository:

```bash
cd ~/catkin_ws/src/
git clone https://github.com/simransarah/turtlebot-perception-navigation.git
```

### 2\. Install Git LFS and Pull Model Files

The YOLO model is stored using Git LFS.

```bash
cd turtlebot-perception-navigation
git lfs install
git lfs pull
```

### 3\. Configure Azure Credentials

This project uses Azure for voice recognition. You must set the following environment variables. It's best to add them to your `~/.bashrc` file.

```bash
export SPEECH_KEY="<Your-Azure-Speech-Service-Key>"
export SPEECH_REGION="<Your-Azure-Speech-Service-Region>"
export CLU_ENDPOINT="<Your-Azure-CLU-Endpoint>"
export CLU_KEY="<Your-Azure-CLU-Key>"
export CLU_PROJECT_NAME="<Your-CLU-Project-Name>"
export CLU_DEPLOYMENT_NAME="<Your-CLU-Deployment-Name>"
```

Remember to run `source ~/.bashrc` after adding them.

### 4\. Install Python Dependencies

Install all necessary Python packages, including the Azure SDKs and Ultralytics.

```bash
cd ~/catkin_ws/
pip3 install -r src/turtlebot-perception-navigation/requirements.txt
```

### 5\. Install ROS Dependencies

This will install any missing ROS packages that your project depends on.

```bash
cd ~/catkin_ws/
rosdep install --from-paths src --ignore-src -r -y
```

### 6\. Build the Workspace

Compile the ROS package.

```bash
cd ~/catkin_ws/
catkin_make
```

## Usage

### 1\. Launch the System

Before launching, make sure to source your workspace. Then, use the provided launch file to start the Gazebo simulation and all the required nodes.

```bash
cd ~/catkin_ws/
source devel/setup.bash
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot-perception-navigation turtlebot.launch
```

### 2\. Verify the Output

You can check that the different parts of the system are working correctly with the following commands:

  * **View Annotated Camera Feed**: See what the robot sees, with bounding boxes around detected objects.

    ```bash
    rqt_image_view /annotated_image
    ```

  * **Check Parsed Voice Commands**: See the JSON output from the voice command node after you speak. This is very useful for debugging.

    ```bash
    rostopic echo /robot_command
    ```

### 3\. Voice Commands

The robot understands a variety of natural language commands. Here are some examples of what you can say:

  * **Simple Navigation**:

      * "Go forward"
      * "Turn left" / "Turn right"
      * "Go backwards"
      * "Stop"

  * **Object-Based Navigation**:

      * "Go to the chair"
      * "Navigate to the bottle"

  * **Centering on an Object**:

      * "Centre on the person"
      * "Face the tv"

  * **Relative Positioning**:

      * "Look to the left of the potted plant"
      * "Adjust view to the right of the cup"