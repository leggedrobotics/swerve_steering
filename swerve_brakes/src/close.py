#!/usr/bin/env python3
import rospy
from swerve_msgs.srv import TriggerBrakes, TriggerBrakesRequest, TriggerBrakesResponse

rospy.init_node("close_brakes")

rospy.loginfo("Waiting for service...")
client = rospy.wait_for_service("/brakes")
rospy.loginfo("... service is available")
try:
    trigger_brakes = rospy.ServiceProxy('/brakes', TriggerBrakes)
    request = TriggerBrakesRequest()
    request.front_left = True
    request.front_right = True
    request.back_left = True
    request.back_right = True
    response : TriggerBrakesResponse = trigger_brakes(request)
    print(f"success: {response.success}")
except rospy.ServiceException as e:
    print("Service call failed: %s"%e)