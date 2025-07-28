#!/usr/bin/env python3
import rospy
import tf2_ros
import math
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class MovementNode:
    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.target_object_name = None
        self.is_navigating = False
        self.move_cmd = Twist()

        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.stop_distance = rospy.get_param('~stop_distance', 0.5)
        self.max_linear_speed = rospy.get_param('~max_linear_speed', 0.3)
        self.max_angular_speed = rospy.get_param('~max_angular_speed', 0.8)

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.command_sub = rospy.Subscriber('/voice_command', String, self.command_callback)

        self.timer = rospy.Timer(rospy.Duration(0.1), self.control_loop)
        rospy.loginfo("Movement node started")

    def command_callback(self, msg):
        command = msg.data.lower().strip()
        self.is_navigating = False
        
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
            self.move_cmd = Twist()
        elif command.startswith("go to the"):
            self.move_cmd = Twist()
            self.target_object_name = command.split("go to the")[1].strip()
            self.is_navigating = True 
            rospy.loginfo(f"Targeting object: '{self.target_object_name}'")
        else:
            rospy.logwarn(f"Unknown command: '{command}'")

    def control_loop(self, event):
        if self.is_navigating:
            if self.target_object_name is None:
                return
            
            target_tf_frame = f"object_{self.target_object_name}"
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame, target_tf_frame, rospy.Time(0), rospy.Duration(1.0)
                )
                
                x = transform.transform.translation.x
                y = transform.transform.translation.y
                distance = math.sqrt(x**2 + y**2)
                angle_to_target = math.atan2(y, x)

                twist = Twist()
                if distance > self.stop_distance:
                    twist.linear.x = min(0.5 * distance, self.max_linear_speed)
                    twist.angular.z = max(min(1.0 * angle_to_target, self.max_angular_speed), -self.max_angular_speed)
                else:
                    rospy.loginfo(f"Reached '{self.target_object_name}', stopping.")
                    self.is_navigating = False
                
                self.cmd_vel_pub.publish(twist)
            
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logwarn(f"Transform error for '{self.target_object_name}': {e}")
                self.cmd_vel_pub.publish(Twist()) 
        else:
            self.cmd_vel_pub.publish(self.move_cmd)

def main():
    rospy.init_node('movement_node', anonymous=True)
    MovementNode()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Movement node terminated.")

if __name__ == '__main__':
    main()