#!/usr/bin/env python
import rospy
from localizer_dwm1001.msg import Tag
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu 

odom = Odometry()
pub = rospy.Publisher('/dwm1001/tag_odom',
                      Odometry, queue_size=10)


def callback(data):
    odom.header.frame_id =  "world"
    odom.child_frame_id =  data.header.frame_id
    odom.pose.pose.position.x = data.x
    odom.pose.pose.position.y = data.y
    odom.pose.pose.position.z = data.z
    pub.publish(odom)


def imu_callback(data):
    odom.pose.pose.orientation = data.orientation

if __name__ == '__main__':
    try:
        rospy.init_node('Tag_pose_converter', anonymous=True)
        rospy.Subscriber("/dwm1001/tag", Tag, callback)
        rospy.Subscriber("imu/data", Imu, imu_callback)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
