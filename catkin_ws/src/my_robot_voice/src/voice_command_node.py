#!/usr/bin/env python3

import os
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import azure.cognitiveservices.speech as speechsdk

class VoiceCommandNode:
    def __init__(self):
        # ROS node setup
        rospy.init_node('voice_command_node')
        self.cmd_pub = rospy.Publisher('/voice_commands', String, queue_size=10)
        # subscribe to the cmd vel of the robot
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # setup Azure Speech SDK
        # this needs to be sourced (see README)
        self.speech_config = speechsdk.SpeechConfig(subscription=os.environ.get('SPEECH_KEY'), endpoint=os.environ.get('ENDPOINT'))
        # configure the language
        self.speech_config.speech_recognition_language = "en-US"
        # define input and initialise SpeechRecogniser
        self.audio_config = speechsdk.audio.AudioConfig(use_default_microphone=True)
        self.speech_recognizer = speechsdk.SpeechRecognizer(speech_config=self.speech_config, audio_config=self.audio_config)

        # connect the speech together
        self.speech_recognizer.recognized.connect(self.recognized_cb)
        self.speech_recognizer.canceled.connect(self.stop_cb)
        self.speech_recognizer.session_stopped.connect(self.stop_cb)

        rospy.loginfo("Voice command node started. Listening for commands...")
        # will continue to recognise the speech until cancelled
        self.speech_recognizer.start_continuous_recognition()


    def recognized_cb(self, evt):
        # speech has been recognised
        if evt.result.reason == speechsdk.ResultReason.RecognizedSpeech:
            recognized_text = evt.result.text.strip().lower() # get the dictation from the speech
            rospy.loginfo(f"Recognised: {recognized_text}")

            twist = Twist()
            # rate = rospy.rate(10)

            # program the commands for moving
            # need the full stop for some reason
            # TODO add more semantics, so if the phrase is similar to this 

            if recognized_text == 'move forward.':
                twist.linear.x = 2.0
            elif recognized_text == 'turn left.':
                twist.angular.z = 2.0
            elif recognized_text == 'turn right.':
                twist.angular.z = -2.0
            elif recognized_text == 'stop.':
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            elif recognized_text == 'circle.':
                while not rospy.is_shutdown():
                    twist.linear.x = 0.5
                    twist.angular.z = 0.2
                    self.vel_pub.publish(twist)
                    # rate.sleep()
            else:
                rospy.loginfo("Command not recognised as an action")
                return
            self.vel_pub.publish(twist)
            rospy.loginfo(f"Published velocity command: {twist}")


    def stop_cb(self, evt):
        rospy.loginfo(f"Session stopped or cancelled: {evt}")
        self.speech_recognizer.stop_continuous_recognition()
        rospy.signal_shutdown("Speech recognition stopped")

if __name__ == '__main__':
    try:
        # runs the node
        node = VoiceCommandNode()
        rospy.spin()
    except rospy.ROSInternalException:
        pass