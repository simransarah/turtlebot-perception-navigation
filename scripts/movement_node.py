#!/usr/bin/env python3
import rospy
import tf2_ros
import math
import json
from std_msgs.msg import String
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import LaserScan
import numpy as np

class MovementNode:
    def __init__(self):
        rospy.init_node('movement_node')
        # Sets up the listener to get transform data
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()


        self.current_action = None
        self.target_frame = None
        self.is_navigating = False
        self.move_cmd = Twist()
        self.virtual_target_transform = None
        
        # Loads our configurable parameters from the launch file
        self.base_frame = rospy.get_param('~base_footprint', 'base_footprint')
        self.stop_distance = rospy.get_param('~stop_distance', 0.8)
        self.max_linear_speed = rospy.get_param('~max_linear_speed', 0.15)
        self.max_angular_speed = rospy.get_param('~max_angular_speed', 0.8)
        self.obstacle_threshold = 0.4 # a sensible default
        
        # Publishers and subscribers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.command_sub = rospy.Subscriber('/robot_command', String, self.command_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.latest_scan = None

        # Starts the main control loop timer
        self.timer = rospy.Timer(rospy.Duration(0.1), self.control_loop)
        rospy.loginfo("Movement node started with robust state management.")

    def scan_callback(self, msg):
        # Stores the latest laser scan data to avoid obstacles
        self.latest_scan = msg

    def reset_state(self):
        self.is_navigating = False
        self.current_action = None
        self.target_frame = None
        self.virtual_target_transform = None
        self.move_cmd = Twist() 
        self.cmd_vel_pub.publish(self.move_cmd)
        rospy.loginfo("Robot state reset and motion stopped.")

    def command_callback(self, msg):
        self.reset_state() 

        try:
            command = json.loads(msg.data)
            action = command.get("action")

            if action == "navigate":
                self.handle_navigate(command)
            elif action == "centre":
                self.handle_centre(command)
            elif action == "adjust_view":
                self.handle_adjust_view(command)
            elif action == "stop":
                pass
            else:
                rospy.logwarn(f"Unknown action received: {action}")

        except (json.JSONDecodeError, KeyError) as e:
            rospy.logerr(f"Failed to parse command '{msg.data}': {e}")
    
    def handle_navigate(self, command):
        target = command.get("target")
        direction = command.get("direction")

        # Prioritises navigating to a target object
        if target:
            self.target_frame = f"object_{target}"
            self.current_action = "go_to"
            self.is_navigating = True
            rospy.loginfo(f"Navigating to target: '{self.target_frame}'")
        elif direction:
            if direction == "left": self.move_cmd.angular.z = 0.5
            elif direction == "right": self.move_cmd.angular.z = -0.5
            elif direction == "forward" or direction == "forwards": self.move_cmd.linear.x = 0.2
            elif direction == "backward" or direction == "backwards" or direction == "go back": self.move_cmd.linear.x = -0.2
            rospy.loginfo(f"Executing persistent move: '{direction}'")

    def handle_centre(self, command):
        target = command.get("target")
        if target:
            self.target_frame = f"object_{target}"
            self.current_action = "centre"
            self.is_navigating = True
            rospy.loginfo(f"Centering on target: '{self.target_frame}'")
        else:
            rospy.logwarn("Centre command received without a target.")
            
    def handle_adjust_view(self, command):
        target = command.get("target")
        relation = command.get("relation")

        if not (target and relation):
            rospy.logwarn("Adjust view command is missing target or relation.")
            return

        real_target_frame = f"object_{target}"
        fixed_frame = "odom"

        try:
            # Get the positions of the robot and the object in the odom frame
            object_transform_odom = self.tf_buffer.lookup_transform(fixed_frame, real_target_frame, rospy.Time(0), rospy.Duration(1.0))
            robot_transform_odom = self.tf_buffer.lookup_transform(fixed_frame, self.base_frame, rospy.Time(0), rospy.Duration(1.0))

            obj_x = object_transform_odom.transform.translation.x
            obj_y = object_transform_odom.transform.translation.y
            robot_x = robot_transform_odom.transform.translation.x
            robot_y = robot_transform_odom.transform.translation.y

            # Calculates the angle from the robot to the real object
            angle_to_real_object = math.atan2(obj_y - robot_y, obj_x - robot_x)
            glance_angle_rad = math.pi / 12 # 15 degrees

            # Calculates the final angle for the virtual target
            if "left of" in relation:
                final_angle = angle_to_real_object + glance_angle_rad
            elif "right of" in relation:
                final_angle = angle_to_real_object - glance_angle_rad
            else:
                rospy.logwarn(f"Relation '{relation}' not implemented.")
                return

            # Creates a new virtual TF frame to aim at
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = fixed_frame
            t.child_frame_id = "virtual_target"

            # Places the virtual target 1m away from the robot at the desired angle
            virtual_dist = 1.0
            t.transform.translation.x = robot_x + virtual_dist * math.cos(final_angle)
            t.transform.translation.y = robot_y + virtual_dist * math.sin(final_angle)
            t.transform.rotation.w = 1.0
            
            self.virtual_target_transform = t
            self.tf_broadcaster.sendTransform(t)
            
            # Waits for the new TF frame to be available
            rospy.loginfo("Waiting for virtual_target to become available in TF tree...")
            self.tf_buffer.can_transform(self.base_frame, "virtual_target", rospy.Time(0), rospy.Duration(1.0))
            
            # Sets the robot's goal to centre on the new virtual target
            self.target_frame = "virtual_target"
            self.current_action = "centre"
            self.is_navigating = True
            rospy.loginfo(f"Adjusting view to be '{relation}' the '{target}'")

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Cannot perform relative positioning. TF error: {e}. Command aborted.")

    def control_loop(self, event):
        # This function runs at 10 hertz to keep controlling the robot
        if self.is_navigating:
            if self.target_frame == "virtual_target" and self.virtual_target_transform:
                self.virtual_target_transform.header.stamp = rospy.Time.now()
                self.tf_broadcaster.sendTransform(self.virtual_target_transform)

            if not self.target_frame or not self.latest_scan:
                return

            try:
                # Get the transform from the robot's base to the target frame
                transform = self.tf_buffer.lookup_transform(self.base_frame, self.target_frame, rospy.Time(0))
                
                distance = math.sqrt(transform.transform.translation.x**2 + transform.transform.translation.y**2)
                angle_to_target = math.atan2(transform.transform.translation.y, transform.transform.translation.x)

                twist = Twist()

                # Obstacle avoidance logic
                if self.current_action == "go_to":
                    ranges = np.array(self.latest_scan.ranges)
                    front_ranges = np.concatenate((ranges[0:30], ranges[330:360]))
                    if np.nanmin(front_ranges) < self.obstacle_threshold:
                        rospy.logwarn("Obstacle detected! Avoiding...")
                        twist.linear.x = 0.0 
                        twist.angular.z = 0.5 
                        self.cmd_vel_pub.publish(twist)
                        return 

                if self.current_action == "centre":
                    if abs(angle_to_target) > 0.05:
                        twist.angular.z = np.clip(1.5 * angle_to_target, -self.max_angular_speed, self.max_angular_speed)
                    else:
                        rospy.loginfo(f"Centering complete for '{self.target_frame}'.")
                        self.reset_state()
                
                elif self.current_action == "go_to":
                    if distance > self.stop_distance:
                        twist.linear.x = np.clip(0.5 * distance, -self.max_linear_speed, self.max_linear_speed)
                        twist.angular.z = np.clip(1.5 * angle_to_target, -self.max_angular_speed, self.max_angular_speed)
                    else:
                        rospy.loginfo(f"Reached target '{self.target_frame}'.")
                        self.reset_state()

                self.cmd_vel_pub.publish(twist)

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                # In case we can't find the object, spin around to try and find it
                rospy.logwarn_throttle(2.0, f"Cannot find transform for '{self.target_frame}', searching...")
                search_cmd = Twist()
                search_cmd.angular.z = 0.4
                self.cmd_vel_pub.publish(search_cmd)
        else:
            self.cmd_vel_pub.publish(self.move_cmd)

def main():
    try:
        MovementNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Movement node terminated.")

if __name__ == '__main__':
    main()