#!/usr/bin/env python
import rospy 

# from <package name> import util
from example import util
from std_msgs.msg import String 

# Initialize the node with rospy 
rospy.init_node('publisher_node')

# create publisher 
publisher = rospy.Publisher('~topic', String, queue_size = 1)

# define timer callback 
def callback(e):
	msg = String()
	msg.data = '%s is %s!' % (util.getName(), util.getStatus())
	publisher.publish(msg)

# Read parameter 
pub_period = rospy.get_param('~pub_period', 1.0)

# Create timer 
rospy.Timer(rospy.Duration.from_sec(pub_period), callback)

# spin to keep the scipt for exiting 
rospy.spin()