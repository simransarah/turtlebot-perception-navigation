# turtlebot-perception-navigation

Libraries required
- Speech SDK

```
pip install azure-cognitiveservices-speech
```
Also need to get API key and Endpoint, then source the environment variables in the .bashrc file
```
export SPEECH_KEY=your-key
export ENDPOINT=your-endpoint
```
The transcribing is done through accessing the Microsoft Azure Speech API

To run the files

1. Start roscore
```
roscore
```
2. Then in a separate terminal, start the gazebo to run the tutlebot in the simulation
```
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```
3. Then in another terminal, start the node to move the robot
```
rosrun my_robot_voice voice_command_node.py
```

The commands that work are
- move forward
- turn left
- turn right
- stop