#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import tf2_ros
import message_filters
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import geometry_msgs.msg
from ultralytics import YOLO
import ros_numpy
import tf.transformations

class ObjectDetectionNode:
    def __init__(self):
        rospy.init_node('object_detection_node')
        
        # Tools for ROS to OpenCV conversion and TF broadcasting
        self.bridge = CvBridge()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Loads parameters from the launch file
        try:
            model_path = rospy.get_param('~yolo_model_path')
            self.model = YOLO(model_path)
            self.confidence_threshold = rospy.get_param('~confidence_threshold', 0.25)
            rospy.loginfo("YOLOv11 model loaded successfully!")
        except Exception as e:
            rospy.logerr(f"Failed to load YOLOv11 model: {str(e)}")
            rospy.signal_shutdown("Failed to load YOLOv11 model")
            return

        # Publisher for the annotated image (for visualisation)
        self.annotated_image_pub = rospy.Publisher('/annotated_image', Image, queue_size=1)
        
        # Synchronises the camera, point cloud, and camera info topics
        image_sub = message_filters.Subscriber('/camera/rgb/image_raw', Image)
        pc_sub = message_filters.Subscriber('/camera/depth/points', PointCloud2)
        info_sub = message_filters.Subscriber('/camera/rgb/camera_info', CameraInfo)

        ts = message_filters.ApproximateTimeSynchronizer(
            [image_sub, pc_sub, info_sub], queue_size=10, slop=0.1
        )
        ts.registerCallback(self.data_callback)
        
        rospy.loginfo("Object detection node started with point cloud integration.")

    def data_callback(self, rgb_msg, pc_msg, camera_info_msg):
        # Converts ROS Image message to OpenCV image
        try:
            colour_image = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"Conversion error: {str(e)}")
            return
        
        # Uses the model to track objects in the image
        results = self.model.track(colour_image, persist=True, conf=self.confidence_threshold, verbose=False)

        for result in results:
            for box in result.boxes:
                class_id = int(box.cls[0])
                label = self.model.names[class_id]
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)

                # Gets the 3D position and publish the transform
                self.process_detection(label, x1, y1, x2, y2, pc_msg, camera_info_msg, rgb_msg.header)

                # Draws bounding box and label on the image for visualisation
                colour = (0, 255, 0)
                cv2.rectangle(colour_image, (x1, y1), (x2, y2), colour, 2)
                text = f"{label}: {float(box.conf[0]):.2f}"
                cv2.putText(colour_image, text, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 2)

        # Publishes the annotated image
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(colour_image, "bgr8")
            self.annotated_image_pub.publish(annotated_msg)
        except CvBridgeError as e:
            rospy.logerr(f"Failed to publish annotated image: {str(e)}")

    def process_detection(self, object_name, x1, y1, x2, y2, pc_msg, camera_info, header):
        # Gets camera intrinsic parameters
        fx, fy, cx, cy = camera_info.K[0], camera_info.K[4], camera_info.K[2], camera_info.K[5]

        # Converts the PointCloud2 message to a NumPy array
        points_3d = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pc_msg)
        
        # Filters out any points that are too close (invalid)
        valid_indices = points_3d[:, 2] > 0.01 
        points_3d = points_3d[valid_indices]
        
        if points_3d.shape[0] == 0:
            rospy.logwarn("No valid 3D points found.")
            return

        # Projects the 3D points back to 2D pixel coordinates
        projected_x = (points_3d[:, 0] * fx / points_3d[:, 2]) + cx
        projected_y = (points_3d[:, 1] * fy / points_3d[:, 2]) + cy
        
        # Finds which of the 3D points fall inside the bounding box
        in_box_mask = (projected_x >= x1) & (projected_x <= x2) & (projected_y >= y1) & (projected_y <= y2)
        points_in_box = points_3d[in_box_mask]
        
        # Calculates the average 3D point within the box to find the object's centre
        if len(points_in_box) > 0:
            x_3d, y_3d, z_3d = np.nanmean(points_in_box, axis=0)
            
            if np.isfinite(x_3d) and np.isfinite(y_3d) and np.isfinite(z_3d):
                self.publish_transform(x_3d, y_3d, z_3d, object_name, header)
            else:
                rospy.logwarn(f"Invalid centroid for {object_name}.")
        else:
            rospy.logwarn(f"No point cloud data found within bounding box for {object_name}.")

    def publish_transform(self, x, y, z, object_name, header):
        # Creates and broadcasts the transform for the detected object
        t = geometry_msgs.msg.TransformStamped()
        
        t.header.stamp = header.stamp
        t.header.frame_id = header.frame_id
        t.child_frame_id = f"object_{object_name.strip()}"

        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        
        # Sets a default orientation (no rotation)
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)
        rospy.loginfo(f"Published transform for {object_name} at ({x:.2f}, {y:.2f}, {z:.2f})")

def main():
    try:
        ObjectDetectionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main() 