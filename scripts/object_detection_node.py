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

class ObjectDetectionNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.colour_image = None
        self.depth_image = None
        self.camera_info = None

        weights_path = rospy.get_param('~yolo_weights_path')
        config_path = rospy.get_param('~yolo_config_path')
        names_path = rospy.get_param('~yolo_names_path')
        self.confidence_threshold = rospy.get_param('~confidence_threshold', 0.5)

        self.labels = open(names_path).read().strip().split('\n')
        try:
            self.net = cv2.dnn.readNet(weights_path, config_path)
            rospy.loginfo("YOLOv4 model loaded successfully!")
        except cv2.error as e:
            rospy.logerr(f"Failed to load YOLOv4 network :/): {str(e)}")
            rospy.signal_shutdown("Failed to load YOLOv4 model")
            return
        
        ln = self.net.getLayerNames()
        try:
            self.output_layers = [ln[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]
        except IndexError:
            self.output_layers = [ln[i - 1] for i in self.net.getUnconnectedOutLayers()]
        
        self.annotated_image_pub = rospy.Publisher('/annotated_image', Image, queue_size=1)
        
        image_sub = message_filters.Subscriber('/camera/rgb/image_raw', Image)
        depth_sub = message_filters.Subscriber('/camera/depth/image_raw', Image)
        info_sub = message_filters.Subscriber('/camera/rgb/camera_info', CameraInfo)

        ts = message_filters.ApproximateTimeSynchronizer(
            [image_sub, depth_sub, info_sub], queue_size=10, slop=0.1
        )
        ts.registerCallback(self.image_processing_callback)
        
        rospy.loginfo("Object detection node started")

    def image_processing_callback(self, rgb_msg, depth_msg, camera_info_msg):
        try:
            self.colour_image = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            self.depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
            self.camera_info = camera_info_msg
        except CvBridgeError as e:
            rospy.logerr(f"Image conversion error: {str(e)}")
            return
        
        height, width = self.colour_image.shape[:2]

        blob = cv2.dnn.blobFromImage(self.colour_image, 1/255.0, (416, 416), swapRB=True, crop=False)
        self.net.setInput(blob)
        detections = self.net.forward(self.output_layers)

        boxes = []
        confidences = []
        class_ids = []

        for output in detections:
            for detection in output:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]

                if confidence > self.confidence_threshold:
                    centre_x = int(detection[0] * width)
                    centre_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)
                    x = int(centre_x - w / 2)
                    y = int(centre_y - h / 2)
                    
                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)
        
        indices = cv2.dnn.NMSBoxes(boxes, confidences, self.confidence_threshold, 0.4)

        if len(indices) > 0:
            for i in indices.flatten():
                (x, y, w, h) = boxes[i]
                self.process_detection(class_ids[i], x, y, w, h)

                colour = (0, 255, 0)
                cv2.rectangle(self.colour_image, (x, y), (x + w, y + h), colour, 2)
                text = f"{self.labels[class_ids[i]]}: {confidences[i]:.2f}"
                cv2.putText(self.colour_image, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 2)

        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(self.colour_image, "bgr8")
            self.annotated_image_pub.publish(annotated_msg)
        except CvBridgeError as e:
            rospy.logerr(f"Failed to publish annotated image: {str(e)}")

    def process_detection(self, class_id, x, y, w, h):
        centroid_x = x + w // 2
        centroid_y = y + h // 2

        if not (0 <= centroid_y < self.depth_image.shape[0] and 0 <= centroid_x < self.depth_image.shape[1]):
            rospy.logwarn("Centroid out of image bounds")
            return
        
        depth = self.depth_image[centroid_y, centroid_x]

        if not np.isfinite(depth) or depth <= 0:
            rospy.logwarn("Invalid depth value")
            return
        
        fx = self.camera_info.K[0]
        fy = self.camera_info.K[4]
        cx = self.camera_info.K[2]
        cy = self.camera_info.K[5]

        z_3d = float(depth)
        x_3d = (centroid_x - cx) * z_3d / fx
        y_3d = (centroid_y - cy) * z_3d / fy

        object_name = self.labels[class_id]
        self.publish_transform(x_3d, y_3d, z_3d, object_name)

    def publish_transform(self, x, y, z, object_name):
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = self.camera_info.header.stamp
        t.header.frame_id = self.camera_info.header.frame_id
        t.child_frame_id = f"object_{object_name.strip()}"

        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        
        q = tf.transformations.quaternion_from_euler(0, 0, 0)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

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