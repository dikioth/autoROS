#!/usr/bin/env python


'''
Simulate Anchor and Tag
Author: ELvis Rodas @Uppsala university.
'''

import rospy
from localizer_dwm1001.msg import Tag
from noise import pnoise2


def main():
    rospy.init_node('talker', anonymous=True)
    pub = rospy.Publisher('/dwm1001/tag_simulation', String, queue_size=10)
    rate = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
