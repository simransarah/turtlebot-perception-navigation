# TurtleBot Perception & Navigation

This ROS package allows a TurtleBot to perform visually-guided navigation. It uses a YOLOv4 model to detect objects in its environment and can move towards a specified object.

## Features

-   Real-time object detection using YOLOv4.
-   3D object localisation using synchronised RGB and Depth camera data.
-   Publishes TF2 transforms for the real-world position of detected objects.
-   A movement node that subscribes to commands and navigates the robot to a target object's transform.

## Prerequisites

-   Ubuntu 20.04 with ROS Noetic
-   Python 3.8+
-   Git
-   Git LFS 

## Installation

1.  **Clone the Repository**

    Clone this specific branch into the `src` directory of your Catkin workspace.

2.  **Install Git LFS and Pull Model Files**

    This project uses Git LFS for the large YOLO weights file.

3.  **Install Dependencies**

    Navigate to the root of your workspace and use `rosdep` to install any missing dependencies.

4.  **Build the Workspace**

## Configuration

The YOLO model files (`yolov4.weights`, `yolov4.cfg`, `coco.names`) are included in this repository (via Git LFS). Their paths are configured in the launch file, so you shouldn't need any more configuration if the default setup is used.

## Usage

To run the full detection and navigation system, use the  launch file!
