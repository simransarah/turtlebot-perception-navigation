#!/usr/bin/env python3

import os
import rospy
from std_msgs.msg import String
import azure.cognitiveservices.speech as speechsdk

class VoiceCommandNode:
    def __init__(self):
        # ROS node setup
        rospy.init_node('voice_command_node')
        self.cmd_pub = rospy.Publisher('/voice_commands', String, queue_size=10)

        # setup Azure Speech SDK
        self.speech_config = speechsdk.SpeechConfig(subscription=os.environ.get('SPEECH_KEY'), endpoint=os.environ.get('ENDPOINT'))
        self.speech_config.speech_recognition_language = "en-US"
        # define input and initialise SpeechRecogniser
        self.audio_config = speechsdk.audio.AudioConfig(use_default_microphone=True)
        self.speech_recognizer = speechsdk.SpeechRecognizer(speech_config=self.speech_config, audio_config=self.audio_config)

        self.speech_recognizer.recognized.connect(self.recognized_cb)
        self.speech_recognizer.canceled.connect(self.stop_cb)
        self.speech_recognizer.session_stopped.connect(self.stop_cb)

        rospy.loginfo("Voice command node started. Listening for commands...")
        self.speech_recognizer.start_continuous_recognition()

    def recognized_cb(self, evt):
        recognized_text = evt.result.text.strip().lower()
        rospy.loginfo(f"Recognised: {recognized_text}")

        if recognized_text in ['turn left', 'turn right', 'move forward', 'stop']:
            self.cmd_pub.publish(recognized_text)
            rospy.loginfo(f"Published voice command: {recognized_text}")
        else:
            rospy.loginfo("Command not recognised as an action")

    def stop_cb(self, evt):
        rospy.loginfo(f"Session stopped or cancelled: {evt}")
        self.speech_recognizer.stop_continuous_recognition()
        rospy.signal_shutdown("Speech recognition stopped")

if __name__ == '__main__':
    try:
        node = VoiceCommandNode()
        rospy.spin()
    except rospy.ROSInternalException:
        pass