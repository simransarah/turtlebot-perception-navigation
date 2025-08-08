#!/usr/bin/env python3
import rospy
import tf2_ros
import math
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class MovementNode:
    def __init__(self):
        # Set up the listener to get transform (TF) data between coordinate frames
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # State variables to manage what the robot is doing
        self.target_object_name = None
        self.is_navigating = False  
        self.move_cmd = Twist()     

        # Load configurable parameters from the launch file, with default values
        self.base_frame = rospy.get_param('~base_footprint', 'base_link')
        self.stop_distance = rospy.get_param('~stop_distance', 0.5)
        self.max_linear_speed = rospy.get_param('~max_linear_speed', 0.3)
        self.max_angular_speed = rospy.get_param('~max_angular_speed', 0.8)

        # Set up ROS publishers and subscribers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.command_sub = rospy.Subscriber('/voice_command', String, self.command_callback)

        # Start a timer to run the control_loop method 10 times a second 
        self.timer = rospy.Timer(rospy.Duration(0.1), self.control_loop)
        rospy.loginfo("Movement node started")

    def command_callback(self, msg):
        # This function is called every time a message is received on /voice_command
        command = msg.data.lower().strip()
        
        # Any new command should cancel the "go to object" navigation mode
        self.is_navigating = False
        
        # Handle simple, direct movement commands
        if command == "go forward":
            self.move_cmd.linear.x = 0.2
            self.move_cmd.angular.z = 0.0
        elif command == "go back":
            self.move_cmd.linear.x = -0.2
            self.move_cmd.angular.z = 0.0
        elif command == "turn left":
            self.move_cmd.linear.x = 0.0
            self.move_cmd.angular.z = 0.4
        elif command == "turn right":
            self.move_cmd.linear.x = 0.0
            self.move_cmd.angular.z = -0.4
        elif command == "stop":
            # Reset the Twist message to zeros
            self.move_cmd = Twist()
        # Handle the more complex 'go to the' command
        elif command.startswith("go to the"):
            self.move_cmd = Twist()  # Stop any simple movement
            self.target_object_name = command.split("go to the")[1].strip()
            self.is_navigating = True 
            rospy.loginfo(f"Targeting object: '{self.target_object_name}'")
        else:
            rospy.logwarn(f"Unknown command: '{command}'")

    def control_loop(self, event):
        # This function runs at 10Hz to continuously control the robot's movement
        if self.is_navigating:
            # Logic for when the robot is navigating to a specific object
            if self.target_object_name is None:
                return
            
            target_tf_frame = f"object_{self.target_object_name}"
            try:
                # Get the latest transform between the robot's base and the object
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame, target_tf_frame, rospy.Time(0), rospy.Duration(1.0)
                )
                
                # Calculate distance and angle to the target
                x = transform.transform.translation.x
                y = transform.transform.translation.y
                distance = math.sqrt(x**2 + y**2)
                angle_to_target = math.atan2(y, x)

                twist = Twist()
                if distance > self.stop_distance:
                    # If we are too far, move towards the object
                    twist.linear.x = min(0.5 * distance, self.max_linear_speed)
                    twist.angular.z = max(min(1.0 * angle_to_target, self.max_angular_speed), -self.max_angular_speed)
                else:
                    # If we have arrived, stop and exit navigation mode
                    rospy.loginfo(f"Reached '{self.target_object_name}', stopping.")
                    self.is_navigating = False
                
                self.cmd_vel_pub.publish(twist)
            
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                # If we can't find the object's transform, log a warning and stop
                rospy.logwarn(f"Transform error for '{self.target_object_name}': {e}")
                self.cmd_vel_pub.publish(Twist()) 
        else:
            # If not navigating, just publish the velocity from the simple commands
            self.cmd_vel_pub.publish(self.move_cmd)

def main():
    # Initialise the ROS node
    rospy.init_node('movement_node', anonymous=True)
    # Create an instance of the MovementNode class
    MovementNode()
    try:
        # Keep the node running until it's stopped
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Movement node terminated.")

if __name__ == '__main__':
    main()
