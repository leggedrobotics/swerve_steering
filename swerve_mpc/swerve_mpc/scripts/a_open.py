#!/usr/bin/env python3
import rospy
from swerve_msgs.srv import BrakesService, BrakesServiceRequest, BrakesServiceResponse

rospy.init_node("mpc_brake_commands")

rospy.loginfo("Waiting for service...")
client = rospy.wait_for_service("/swerve_base/brakes_service")
rospy.loginfo("... service is available")
try:
    trigger_brakes = rospy.ServiceProxy('/swerve_base/brakes_service', BrakesService)
    request = BrakesServiceRequest()
    request.lf_angle = 0.5
    request.rf_angle = -0.5
    request.lb_angle = 0.72
    request.rb_angle = -0.72
    request.lf_brake_active = False
    request.rf_brake_active = False
    request.lb_brake_active = False
    request.rb_brake_active = False
    response : BrakesServiceResponse = trigger_brakes(request)
    print(f"success: {response.result}")
except rospy.ServiceException as e:
    print("Service call failed: %s"%e)