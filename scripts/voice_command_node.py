#!/usr/bin/env python3
import os
import rospy
import json
import azure.cognitiveservices.speech as speechsdk
from azure.core.credentials import AzureKeyCredential
from azure.ai.language.conversations import ConversationAnalysisClient
from std_msgs.msg import String

class VoiceCommandNode:
    def __init__(self):
        # Publisher for the JSON commands
        self.cmd_pub = rospy.Publisher('/robot_command', String, queue_size=10)

       
        try:
            # Gets credentials from environment variables (needs to be sourced)
            self.speech_key = os.environ.get('SPEECH_KEY')
            speech_region = os.environ.get('SPEECH_REGION')
            self.clu_endpoint = os.environ.get('CLU_ENDPOINT')
            clu_key = os.environ.get('CLU_KEY')
            
            self.clu_project = os.environ.get('CLU_PROJECT_NAME')
            self.clu_deployment = os.environ.get('CLU_DEPLOYMENT_NAME')

            # Checks if all required environment variables are set
            if not all([self.speech_key, speech_region, self.clu_endpoint, clu_key, self.clu_project, self.clu_deployment]):
                raise ValueError("One or more Azure environment variables are not set.")

            # Configures the Speech to Text 
            self.speech_config = speechsdk.SpeechConfig(subscription=self.speech_key, region=speech_region)
            self.audio_config = speechsdk.audio.AudioConfig(use_default_microphone=True)
            self.speech_recognizer = speechsdk.SpeechRecognizer(
                speech_config=self.speech_config, audio_config=self.audio_config
            )

            # Configures the CLU for language understanding
            credential = AzureKeyCredential(clu_key)
            self.clu_client = ConversationAnalysisClient(endpoint=self.clu_endpoint, credential=credential)

        except Exception as e:
            rospy.logerr(f"Failed to initialise Azure clients: {e}")
            rospy.signal_shutdown("Azure Initialisation Error")
            return

        # Connects callbacks for the speech recogniser
        self.speech_recognizer.recognized.connect(self.recognized_callback)
        self.speech_recognizer.canceled.connect(self.stop_callback)
        self.speech_recognizer.session_stopped.connect(self.stop_callback)
        
        # Starts listening for audio
        self.speech_recognizer.start_continuous_recognition()
        rospy.loginfo("Voice command node started with CLU integration.")

    def query_clu(self, text):
        # Sends the recognised text to CLU and returns the result
        clu_request = {
            "kind": "Conversation",
            "analysisInput": {"conversationItem": {"id": "1", "participantId": "1", "text": text}},
            "parameters": {"projectName": self.clu_project, "deploymentName": self.clu_deployment}
        }
        return self.clu_client.analyze_conversation(clu_request)

    def process_clu_result(self, intent, entities):
        # Converts a CLU intent and entities into the command structure
        command = {"action": None}

        # Maps the CLU intent directly to our action name
        if intent == "Navigate":
            command["action"] = "navigate"
        elif intent == "AdjustView":
            command["action"] = "adjust_view"
        elif intent == "CentreObject":
            command["action"] = "centre"
        elif intent == "Stop": 
            command["action"] = "stop"
        else:
            rospy.logwarn(f"Unknown intent received: {intent}")
            return

        # Process entities and add them as key-value pairs
        for entity in entities:
            entity_name = entity['category']
            entity_value = entity.get('text', '').strip().strip('.')
            
            # Maps entity categories to the JSON keys
            if entity_name == "object_name":
                command["target"] = entity_value
            elif entity_name == "direction":
                command["direction"] = entity_value
            elif entity_name == "spatial_relation":
                command["relation"] = entity_value

        # Validation
        action = command.get("action")
        if action == "adjust_view" and ("target" not in command or "relation" not in command):
            rospy.logwarn(f"Command '{action}' requires a target and relation, but CLU did not identify them.")
            return

        if action in ["centre", "navigate"] and "target" not in command and "direction" not in command:
            rospy.logwarn(f"Command '{action}' requires a target or direction, but CLU did not identify one.")
            return

        if command["action"]:
            self.cmd_pub.publish(json.dumps(command))
            rospy.loginfo(f"Published command: '{json.dumps(command)}'")

    def recognized_callback(self, evt):
        if evt.result.reason != speechsdk.ResultReason.RecognizedSpeech:
            return

        recognized_text = evt.result.text.strip()
        if not recognized_text:
            return
            
        rospy.loginfo(f"Recognised: '{recognized_text}'")
        
        try:
            # Sends the text to CLU for understanding
            clu_result = self.query_clu(recognized_text)
            prediction = clu_result['result']['prediction']
            
            top_intent = prediction['topIntent']
            entities = prediction['entities']
            rospy.loginfo(f"CLU Result -> Intent: '{top_intent}', Entities: {entities}")

            # Processes the intent and entities to form our command
            self.process_clu_result(top_intent, entities)

        except Exception as e:
            rospy.logerr(f"Error processing CLU result: {e}")
            
    def stop_callback(self, evt):
        """Callback for session stop or cancellation."""
        rospy.loginfo(f"Stopping recognition: {evt}")
        self.speech_recognizer.stop_continuous_recognition()
        rospy.signal_shutdown("Speech recognition stopped.")

def main():
    rospy.init_node('voice_command_node')
    VoiceCommandNode()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Voice command node terminated.")

if __name__ == '__main__':
    main()