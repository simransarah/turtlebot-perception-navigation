#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import tf2_ros
import tf.transformations
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import geometry_msgs.msg
from ultralytics import YOLO

class ObjectDetectionNode:
    def __init__(self):
        # Tools for converting ROS messages and broadcasting transforms
        self.bridge = CvBridge()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Load configurable parameters from the launch file
        model_path = rospy.get_param('~yolo_model_path')
        self.confidence_threshold = rospy.get_param('~confidence_threshold', 0.25)
        self.tf_prefix = rospy.get_param('~tf_prefix', 'object_')

        # Load the YOLOv8 model
        try:
            self.model = YOLO(model_path)
            rospy.loginfo("YOLOv8 model loaded successfully!")
        except Exception as e:
            rospy.logerr(f"Failed to load YOLOv8 model: {str(e)}")
            rospy.signal_shutdown("Failed to load YOLOv8 model")
            return
        
        # Publisher for the annotated image (for visualisation)
        self.annotated_image_pub = rospy.Publisher('/annotated_image', Image, queue_size=1)
        
        # Set up message_filters to synchronise camera topics
        image_sub = message_filters.Subscriber('/camera/rgb/image_raw', Image)
        depth_sub = message_filters.Subscriber('/camera/depth/image_raw', Image)
        info_sub = message_filters.Subscriber('/camera/rgb/camera_info', CameraInfo)

        ts = message_filters.ApproximateTimeSynchronizer(
            [image_sub, depth_sub, info_sub], queue_size=10, slop=0.1
        )
        ts.registerCallback(self.image_processing_callback)
        
        rospy.loginfo("Object detection node started")

    def image_processing_callback(self, rgb_msg, depth_msg, camera_info_msg):
        # Convert the incoming ROS messages to OpenCV format
        try:
            colour_image = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
        except CvBridgeError as e:
            rospy.logerr(f"Image conversion error: {str(e)}")
            return
        
        # Use the object tracker for stable detections
        results = self.model.track(colour_image, persist=True, conf=self.confidence_threshold, verbose=False)

        # Loop through the results from the model
        for result in results:
            for box in result.boxes:
                # Extract data from the bounding box
                class_id = int(box.cls[0])
                label = self.model.names[class_id]
                x_center, y_center, w, h = box.xywh[0].cpu().numpy().astype(int)
                x1 = x_center - w // 2
                y1 = y_center - h // 2

                # Calculate 3D position and publish the TF transform
                self.process_detection(label, x1, y1, w, h, depth_image, camera_info_msg)

                # Draw visualisation on the image
                colour = (0, 255, 0)
                cv2.rectangle(colour_image, (x1, y1), (x1 + w, y1 + h), colour, 2)
                
                # Create the label text, including the track ID if available
                if box.id is not None: 
                    track_id = int(box.id[0])
                    text = f"ID {track_id}: {label}: {float(box.conf[0]):.2f}"
                else:
                    text = f"{label}: {float(box.conf[0]):.2f}"

                cv2.putText(colour_image, text, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 2)

        # Publish the annotated image
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(colour_image, "bgr8")
            self.annotated_image_pub.publish(annotated_msg)
        except CvBridgeError as e:
            rospy.logerr(f"Failed to publish annotated image: {str(e)}")

    def process_detection(self, object_name, x, y, w, h, depth_image, camera_info):
        # Calculate the centre point of the bounding box
        centroid_x = x + w // 2
        centroid_y = y + h // 2

        # Make sure the centroid is within the image boundaries
        if not (0 <= centroid_y < depth_image.shape[0] and 0 <= centroid_x < depth_image.shape[1]):
            rospy.logwarn("Centroid out of image bounds")
            return
        
        # Get the depth value at the centroid
        depth = depth_image[centroid_y, centroid_x]

        # Check for invalid depth values
        if not np.isfinite(depth) or depth <= 0:
            rospy.logwarn(f"Invalid depth for {object_name}: {depth}")
            return
        
        # Get camera intrinsics from the CameraInfo message
        fx = camera_info.K[0]
        fy = camera_info.K[4]
        cx = camera_info.K[2]
        cy = camera_info.K[5]

        # Project 2D pixel to 3D point
        z_3d = float(depth)
        x_3d = (centroid_x - cx) * z_3d / fx
        y_3d = (centroid_y - cy) * z_3d / fy

        # Pass all necessary data to the transform publisher
        self.publish_transform(x_3d, y_3d, z_3d, object_name, camera_info.header)

    def publish_transform(self, x, y, z, object_name, header):
        # Create a TransformStamped message to broadcast
        t = geometry_msgs.msg.TransformStamped()
        
        t.header.stamp = header.stamp
        t.header.frame_id = header.frame_id
        t.child_frame_id = f"{self.tf_prefix}{object_name.strip()}"

        # Set the 3D position of the object
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        
        # Set the orientation to a default (no rotation)
        q = tf.transformations.quaternion_from_euler(0, 0, 0)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Broadcast the transform to the TF tree
        self.tf_broadcaster.sendTransform(t)
        rospy.loginfo(f"Published transform for {object_name}")

def main():
    rospy.init_node('object_detection_node', anonymous=True)
    ObjectDetectionNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down...")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()