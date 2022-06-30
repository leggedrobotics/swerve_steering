#!/usr/bin/env python3
import rospy
from time import time, sleep
from rosgraph_msgs.msg import Clock

rospy.init_node("clock_publisher")
starting_time = time()
clock_publisher = rospy.Publisher("/clock", Clock, tcp_nodelay=True, queue_size=1)
real_time_factor = 0.1
publishing_rate = 1000.0

while not rospy.is_shutdown():
    now = time()
    clock_msgs = Clock()
    clock_msgs.clock = rospy.Time.from_sec((now-starting_time) * real_time_factor)
    clock_publisher.publish(clock_msgs)
    sleep(1.0 / publishing_rate)