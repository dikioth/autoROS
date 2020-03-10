#!/usr/bin/env python  

''' 
This packages subscribe to Tag & Anchorarrays and broadcasts the transformations.
'''



import rospy
import tf
from localizer_dwm1001.msg import Tag
from localizer_dwm1001.msg import AnchorArray
from sensor_msgs.msg import Imu


class Broadcaster:
    '''
    This class broadcast the transform of the tags position and orientation.
    Author: Elvis Rodas
    '''
    def __init__(self):
        self.orient = (0,0,0,0)
        self.br = tf.TransformBroadcaster()

    def start(self):
        rospy.init_node('tag_anchors_broadcaster', anonymous=True)
        rospy.Subscriber("/dwm1001/tag", Tag, self.tag_callback)
        rospy.Subscriber("/imu/data", Imu, self.imu_callback)
        rospy.loginfo("Running ...")
        rospy.spin()

    def imu_callback(self, data):
        self.orient = (data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)

    def tag_callback(self, data):
        self.br.sendTransform((data.x, data.y, data.z),
                        self.orient,
                        rospy.Time.now(),
                        data.header.frame_id,
                        "world")
        
    def anchors_callback(self, data):
        br = tf.TransformBroadcaster()

        for anchor in data.anchors:
            br.sendTransform((anchor.x, anchor.y, anchor.z),
                            tf.transformations.quaternion_from_euler(0, 0, 0),
                            rospy.Time.now(),
                            anchor.header.frame_id,
                            "world")

if __name__ == '__main__':
    bc = Broadcaster();
    bc.start()
