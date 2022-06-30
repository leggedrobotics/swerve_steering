#!/usr/bin/env python3
from copy import deepcopy
import rospy
import tf2_ros
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import ros_numpy
from geometry_msgs.msg import Twist, TransformStamped, PoseStamped, Point, Quaternion
from nav_msgs.msg import Path
import numpy as np

pub : rospy.Publisher = None
tfBuffer = tf2_ros.Buffer()
last_received : rospy.Time = None
time_horizon = 10.0
num_poses = 100

zero_sent = False
path : Path = None

def callback(twist : Twist):
    global pub, path, zero_sent
    
    linear_velocity : np.ndarray = ros_numpy.numpify(twist.linear)
    angular_velocity : np.ndarray = ros_numpy.numpify(twist.angular)

    if np.all(np.abs(linear_velocity) < 0.001) and np.all(np.abs(angular_velocity) < 0.001):
        path = None
        if zero_sent == True:
            return
        else:
            zero_sent = True
    else:
        zero_sent = False

    trans : TransformStamped = tfBuffer.lookup_transform("odom", 'base_link', rospy.Time(0.0))

    if path is None:
        path = Path()
        current_pose = PoseStamped()
        current_pose.header = trans.header
        current_pose.pose.position = trans.transform.translation
        current_pose.pose.orientation = trans.transform.rotation
        path.poses.append(current_pose)
    else:
        while len(path.poses) > 0:
            if (path.poses[-1].header.stamp > trans.header.stamp):
                path.poses.pop()
            else:
                break
        current_pose = path.poses[-1]

    current_position = ros_numpy.numpify(current_pose.pose.position)
    current_orientation = ros_numpy.numpify(current_pose.pose.orientation)
    current_yaw = euler_from_quaternion(current_orientation.tolist())[2]

    for i in range(num_poses-1):
        trans_np = ros_numpy.numpify(path.poses[-1].pose)
        # transform linear velocity from base to odom frame
        linear_velocity_base_frame = np.matmul(trans_np[:3, :3], linear_velocity)

        goal_position = trans_np[:3, 3] + linear_velocity_base_frame * time_horizon / num_poses
        goal_pose = deepcopy(current_pose)
        goal_pose.header.stamp = path.poses[-1].header.stamp + rospy.Duration.from_sec(time_horizon / num_poses)
        goal_pose.pose.position = ros_numpy.msgify(Point, goal_position)

        current_orientation = ros_numpy.numpify(path.poses[-1].pose.orientation)
        current_yaw = euler_from_quaternion(current_orientation.tolist())[2]
        goal_yaw = current_yaw + angular_velocity[2] * time_horizon / num_poses
        goal_pose.pose.orientation = ros_numpy.msgify(Quaternion, np.array(quaternion_from_euler(0.0, 0.0, goal_yaw)))
        
        path.poses.append(goal_pose)

    path.header = trans.header
    pub.publish(path)



if __name__ == '__main__':
    rospy.init_node("joy_reference")
    listener = tf2_ros.TransformListener(tfBuffer)
    pub = rospy.Publisher("command_path", Path, queue_size=1)
    sub = rospy.Subscriber("cmd_vel", Twist, callback, queue_size=1)
    rospy.spin()