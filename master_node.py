#!/usr/bin/env python3

import rospy
import math
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def cycloidal_trajectory():
    rospy.init_node('master_controller', anonymous=True)
    pub = rospy.Publisher('/open_manipulator/goal_joint_space_path', JointTrajectory, queue_size=10)
    rate = rospy.Rate(50)  # 50 Hz for smooth motion

    # Trajectory parameters
    freq = 0.2  # Frequency in Hz
    duration = 5.0  # Duration in seconds
    start_time = rospy.Time.now().to_sec()
    joint_start = [0.0, 0.0, 0.0, 0.0]  # Initial joint positions
    joint_end = [1.0, -1.0, 0.5, 0.2]   # Final joint positions

    while not rospy.is_shutdown():
        # Compute elapsed time
        current_time = rospy.Time.now().to_sec() - start_time
        t_normalized = (current_time % duration) / duration  # Normalize time to [0, 1]

        # Compute cycloidal positions for each joint
        joint_positions = [
            start + (end - start) * (t_normalized - math.sin(2 * math.pi * freq * t_normalized) / (2 * math.pi * freq))
            for start, end in zip(joint_start, joint_end)
        ]

        # Create and publish the trajectory message
        trajectory = JointTrajectory()
        trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start = rospy.Duration(0.02)  # 20 ms (50 Hz)

        trajectory.points.append(point)
        pub.publish(trajectory)

        rate.sleep()

if __name__ == "__main__":
    try:
        cycloidal_trajectory()
    except rospy.ROSInterruptException:
        pass
