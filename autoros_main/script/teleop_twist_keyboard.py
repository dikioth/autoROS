#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String


msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""


class Teleop:
    
    def __init__(self):

        self.key = "k"

        self.moveBindings = {
            'i': (1, 0, 0, 0),
            'o': (1, 0, 0, -1),
            'j': (0, 0, 0, 1),
            'l': (0, 0, 0, -1),
            'u': (1, 0, 0, 1),
            ',': (-1, 0, 0, 0),
            '.': (-1, 0, 0, 1),
            'm': (-1, 0, 0, -1),
            'O': (1, -1, 0, 0),
            'I': (1, 0, 0, 0),
            'J': (0, 1, 0, 0),
            'L': (0, -1, 0, 0),
            'U': (1, 1, 0, 0),
            '<': (-1, 0, 0, 0),
            '>': (-1, -1, 0, 0),
            'M': (-1, 1, 0, 0),
            't': (0, 0, 1, 0),
            'b': (0, 0, -1, 0),
        }

        self.speedBindings = {
            'q': (1.1, 1.1),
            'z': (.9, .9),
            'w': (1.1, 1),
            'x': (.9, 1),
            'e': (1, 1.1),
            'c': (1, .9),
        }

        rospy.init_node('autoros_teleop')
        rospy.Subscriber('/autoros/teleop_string',
                         String, self.handleKeyString)
        self.pub = rospy.Publisher(
            '/autoros/teleop_twist', Twist, queue_size=1)
        self.rate = rospy.Rate(10)  # 10hz

    def handleKeyString(self, data):
        self.key = data.data

    def vels(self, speed, turn):
        return "currently:\tspeed %s\tturn %s " % (speed, turn)

    def main(self):

        speed = rospy.get_param("~speed", 0.5)
        turn = rospy.get_param("~turn", 1.0)
        x = 0
        y = 0
        z = 0
        th = 0
        status = 0

        try:
            while not rospy.is_shutdown():
                rospy.loginfo("VEL: " + self.vels(speed, turn))
                rospy.loginfo(self.key)
                if self.key in self.moveBindings.keys():
                    x = self.moveBindings[self.key][0]
                    y = self.moveBindings[self.key][1]
                    z = self.moveBindings[self.key][2]
                    th = self.moveBindings[self.key][3]
                elif self.key in self.speedBindings.keys():
                    speed = speed * self.speedBindings[self.key][0]
                    turn = turn * self.speedBindings[self.key][1]

                    
                else:
                    x = 0
                    y = 0
                    z = 0
                    th = 0
                    
                twist = Twist()
                twist.linear.x = x*speed
                twist.linear.y = y*speed
                twist.linear.z = z*speed
                twist.angular.x = 0
                twist.angular.y = 0
                twist.angular.z = th*turn
                self.pub.publish(twist)
                self.rate.sleep()

        except Exception as e:
            print(e)

        finally:
            twist = Twist()
            twist.linear.x = 0
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = 0
            self.pub.publish(twist)


if __name__ == "__main__":

    tel = Teleop()
    tel.main()
