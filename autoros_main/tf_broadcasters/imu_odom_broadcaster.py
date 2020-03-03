#!/usr/bin/env python  
#import roslib
#roslib.load_manifest('learning_tf')

''' 
This packages subscribe to Tag & Anchorarrays and broadcasts the transformations.
'''

import rospy
from localizer_dwm1001.msg import Tag
from localizer_dwm1001.msg import AnchorArray


def tag_callback(data):
    br = tf.TransformBroadcaster()
    br.sendTransform((data.x, data.y, data.z),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     data.header.frame_id,
                     "world")
    
def anchors_callback(data):
    br = tf.TransformBroadcaster()

    for anchor in data.anchors:
        br.sendTransform((anchor.x, anchor.y, anchor.z),
                        tf.transformations.quaternion_from_euler(0, 0, 0),
                        rospy.Time.now(),
                        anchor.header.frame_id,
                        "world")

def listener():

    # Subscribe to Anchors and tags topics
    rospy.init_node('tag_anchors_broadcaster', anonymous=True)
    rospy.Subscriber("/dwm1001/tag", Tag, tag_callback)
    rospy.Subscriber("/dwm1001/anchors", AnchorArray, anchors_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()