#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TwistWithCovarianceStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from robot_localization.srv import SetPose

"""
    We create messages which are valid so that we do not get any warnings
    because the quaternions, covariances or frames are not correct.
"""

pose_template = PoseWithCovarianceStamped()
pose_template.header.frame_id = "base_link"
pose_template.pose.pose.position.x = 1
pose_template.pose.pose.orientation.w = 1
for i in range(6):
    pose_template.pose.covariance[i*6 + i] = 1

twist_template = TwistWithCovarianceStamped()
twist_template.header.frame_id = "base_link"
for i in range(6):
    twist_template.twist.covariance[i*6 + i] = 1

odom_template = Odometry()
odom_template.header.frame_id = "odom"
odom_template.child_frame_id = "base_link"
odom_template.pose = pose_template.pose
odom_template.twist = twist_template.twist

imu_template = Imu()
imu_template.header.frame_id = "base_link"
imu_template.orientation.w = 1
for i in range(3):
    imu_template.orientation_covariance[i * 3 + i] = 1
    imu_template.angular_velocity_covariance[i * 3 + i] = 1
    imu_template.linear_acceleration_covariance[i * 3 + i] = 1


def update_timestamps(timestamp):
    odom_template.header.stamp = timestamp
    pose_template.header.stamp = timestamp
    twist_template.header.stamp = timestamp
    imu_template.header.stamp = timestamp


def set_pose(pose):
    rospy.wait_for_service('set_pose')
    try:
        pose_setter = rospy.ServiceProxy('set_pose', SetPose)
        result = pose_setter(pose)
        return result
    except rospy.ServiceException, e:
        print("Service call failed: %s".format(e))

if __name__ == '__main__':
    rospy.init_node('timestamp_diagnostic_tester', anonymous=True)
    hz = 10.0

    publisher_odom = rospy.Publisher("example/odom",
                                     Odometry, queue_size=10)
    publisher_pose = rospy.Publisher("example/pose",
                                     PoseWithCovarianceStamped, queue_size=10)
    publisher_twist = rospy.Publisher("example/twist",
                                      TwistWithCovarianceStamped, queue_size=10)
    publisher_imu = rospy.Publisher("example/imu/data",
                                    Imu, queue_size=10)

    def publish_messages():
        publisher_odom.publish(odom_template)
        publisher_pose.publish(pose_template)
        publisher_twist.publish(twist_template)
        publisher_imu.publish(imu_template)

    def idle(duration):
        current_time = rospy.Time.now()
        while (rospy.Time.now() - current_time) < rospy.Duration(duration):
            rospy.sleep(1/hz)
            update_timestamps(rospy.get_rostime())
            publish_messages()

    idle(5)

    # test empty / timestamp before last reset
    update_timestamps(rospy.Time.from_sec(0))
    publish_messages()

    idle(3)

    # reset the filter with the set_pose service.
    update_timestamps(rospy.get_rostime())
    set_pose(pose_template)

    # send messages from before the set_pose timestamp.
    update_timestamps(rospy.Time.from_sec(rospy.get_rostime().to_sec() - 10))
    publish_messages()

    idle(3)

    # send messages prior to the last messages
    update_timestamps(rospy.Time.from_sec(rospy.get_rostime().to_sec() - 2))
    publish_messages()

    idle(3)

    rospy.signal_shutdown("Finished testing")
