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
    request.front_left = False
    request.front_right = False
    request.back_left = False
    request.back_right = False
    response : TriggerBrakesResponse = trigger_brakes(request)
    print(f"success: {response.success}")
except rospy.ServiceException as e:
    print("Service call failed: %s"%e)