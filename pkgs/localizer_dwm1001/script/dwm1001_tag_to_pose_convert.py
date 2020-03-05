#!/usr/bin/env python
import rospy
from localizer_dwm1001.msg import Tag
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu 

p = PoseWithCovarianceStamped()
pub = rospy.Publisher('/dwm1001/tagPose',
                      PoseWithCovarianceStamped, queue_size=10)


def callback(data):
    p.pose.pose.position.x = data.x
    p.pose.pose.position.y = data.y
    p.pose.pose.position.z = data.z
    pub.publish(p)


def imu_callback(data):
    p.pose.pose.orientation = data.orientation

if __name__ == '__main__':
    try:
        rospy.init_node('Tag_pose_converter', anonymous=True)
        rospy.Subscriber("/dwm1001/tag", Tag, callback)
        rospy.Subscriber("imu/data", Imu, imu_callback)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
