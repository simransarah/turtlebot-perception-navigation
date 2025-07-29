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

            message = ''

            # sees if commands are found
            # not case sensitive
            if 'move forward' in recognized_text.lower():
                message = 'move forward'
            elif 'turn left' in recognized_text.lower():
                message = 'turn left'
            elif 'turn right' in recognized_text.lower():
                message = 'turn right'
            elif 'stop' in recognized_text.lower():
                message = 'stop'
            else:
                rospy.loginfo("Command not recognised as an action")
                return
            self.cmd_pub.publish(message)
            rospy.loginfo(f"Published velocity command: {message}")


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
