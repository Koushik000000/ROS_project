#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Publisher for slave manipulator
slave_pub = None

def slave_controller(master_joint_states):
    """
    Callback to mimic master's joint states for the slave manipulator.
    """
    global slave_pub

    # Create a trajectory message for the slave manipulator
    trajectory = JointTrajectory()
    trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

    # Create a trajectory point with the master's joint positions
    point = JointTrajectoryPoint()
    point.positions = master_joint_states.position
    point.time_from_start = rospy.Duration(0.002)  # Mimic master's update frequency (20ms)

    trajectory.points.append(point)

    # Publish to the slave manipulator's goal trajectory topic
    rospy.loginfo("Slave: Mimicking Master's Joint Positions: %s", master_joint_states.position)
    slave_pub.publish(trajectory)

def listener():
    """
    Slave node initialization and subscription to the master's joint states.
    """
    global slave_pub

    # Initialize the ROS node
    rospy.init_node('slave_listener', anonymous=True)

    # Publisher for the slave manipulator's goal trajectory
    slave_pub = rospy.Publisher('/slave_manipulator/goal_joint_space_path', JointTrajectory, queue_size=10)

    # Subscriber to the master's joint states topic
    rospy.Subscriber('/open_manipulator/joint_states', JointState, slave_controller)

    # Keep the node alive
    rospy.spin()

if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
