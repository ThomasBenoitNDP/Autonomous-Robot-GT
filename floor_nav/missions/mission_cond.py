#!/usr/bin/python
# ROS specific imports
import roslib; roslib.load_manifest('floor_nav')
import rospy
from math import *
from task_manager_lib.TaskClient import *

rospy.init_node('task_client')
server_node = rospy.get_param("~server","/task_server")
default_period = rospy.get_param("~period",0.2)
tc = TaskClient(server_node,default_period)


while True:
	
    # Start the wait for face task in the background
	w4face = tc.WaitForFace(foreground=False)
    # Prepare a condition so that the following gets executed only until the 
    # Region of Interest is found
	tc.addCondition(ConditionIsCompleted("Face detector",tc,w4face))
	try:
		tc.Wander(max_linear_speed=0.2)
        # Clear the conditions if we reach this point
		tc.clearConditions()
	except TaskConditionException, e:
		tc.Wait(duration=3.)
		tc.StareAtFace(relative=True)
		tc.Wait(duration=3.)
		tc.SetHeading(target = 45, relative = True, max_angular_velocity=0.2)
		tc.Wait(duration=3.)
		rospy.loginfo("Path following interrupted on condition: %s" % \
		" or ".join([str(c) for c in e.conditions]))
        # This means the conditions were triggered. We need to react to it
        # Conditions are cleared on trigger
        #tc.ReachAngle(target=pi/2)



rospy.loginfo("Mission completed")


