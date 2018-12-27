Team1
ws://127.0.0.1:9090

# list all topic
rostopic list

#
rostopic info <topic-name>

# Topic type of topic
rostopic type <topic-name>

# echo
rostopic echo /publisher_node/topic

# RUN for test
roslaunch lane_detect test.launch test:=image server_on:=0
roslaunch lane_detect test.launch test:=video server_on:=0
roslaunch lane_detect test.launch test:=save_video server_on:=1

# RUN
roslaunch lane_detect lane_detect.launch

[=] Prepare data:
  

[=] Module:
  Lane Detector:
  Sign Detector
  Controller: process for calculate
