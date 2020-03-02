import rospy
import noise
from localizer_dwm1001.msg import Tag


class TagSimulation:

    def start():
        anchor = Tag("TagSimulation",
        # publish each anchor, add anchor number to the topic, so we can pubblish multiple anchors
        # example /dwm1001/anchor0, the last digit is taken from AN0 and so on
        pub_tag_simulator=rospy.Publisher(
            '/dwm1001/TagSimulator', Tag, queue_size=1)
       rate=10;
       dt=0

        while not rospy.is_shutdown():

            pub_tag_simulator.publish()

            delay(100)


def main():
    tagSimulator=TagSimulation()
    tagSimulator.start()


if __name__ == '__main__':
    try:
        main()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass



#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

def talker():
    pub=rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate=rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        hello_str="hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
