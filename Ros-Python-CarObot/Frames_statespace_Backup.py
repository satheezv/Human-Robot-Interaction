#!/usr/bin/env python

import roslib
#roslib.load_manifest('nav_goal_publisher')

import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('fixed_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br.sendTransform((-0.75, -0.82, 0.0),
                    (0.0, 0.0, 0.0, 1.0),
                    rospy.Time.now(),
                    "state_1",
                    "map_corrected")

        br.sendTransform((0.35, 0.35, 0.0),
                    (0.0, 0.0, 0.0, 1.0),
                    rospy.Time.now(),
                    "reference ",
                    "state_1")

        br.sendTransform((0.0, 0.7, 0.0),
                    (0.0, 0.0, 0.0, 1.0),
                    rospy.Time.now(),
                    "state_2",
                    "state_1")
        br.sendTransform((0.0, 1.4, 0.0),
                    (0.0, 0.0, 0.0, 1.0),
                    rospy.Time.now(),
                    "state_3",
                    "state_1")
        br.sendTransform((0.7, 0.0, 0.0),
                    (0.0, 0.0, 0.0, 1.0),
                    rospy.Time.now(),
                    "state_4",
                    "state_1")
        br.sendTransform((0.7, 0.7, 0.0),
                    (0.0, 0.0, 0.0, 1.0),
                    rospy.Time.now(),
                    "state_5",
                    "state_1")
        br.sendTransform((0.7, 1.4, 0.0),
                    (0.0, 0.0, 0.0, 1.0),
                    rospy.Time.now(),
                    "state_6",
                    "state_1")
        br.sendTransform((1.4, 0.0, 0.0),
                    (0.0, 0.0, 0.0, 1.0),
                    rospy.Time.now(),
                    "state_7",
                    "state_1")
        br.sendTransform((1.4, 0.7, 0.0),
                    (0.0, 0.0, 0.0, 1.0),
                    rospy.Time.now(),
                    "state_8",
                    "state_1")
        br.sendTransform((1.4, 1.4, 0.0),
                    (0.0, 0.0, 0.0, 1.0),
                    rospy.Time.now(),
                    "state_9",
                    "state_1")
        br.sendTransform((2.1, 0.0, 0.0),
                    (0.0, 0.0, 0.0, 1.0),
                    rospy.Time.now(),
                    "state_10",
                    "state_1")
        br.sendTransform((2.1, 0.7, 0.0),
                    (0.0, 0.0, 0.0, 1.0),
                    rospy.Time.now(),
                    "state_11",
                    "state_1")
        br.sendTransform((2.1, 1.4, 0.0),
                    (0.0, 0.0, 0.0, 1.0),
                    rospy.Time.now(),
                    "state_12",
                    "state_1")
        br.sendTransform((2.8, 0.0, 0.0),
                    (0.0, 0.0, 0.0, 1.0),
                    rospy.Time.now(),
                    "state_13",
                    "state_1")
        br.sendTransform((2.8, 0.7, 0.0),
                    (0.0, 0.0, 0.0, 1.0),
                    rospy.Time.now(),
                    "state_14",
                    "state_1")
        br.sendTransform((2.8, 1.4, 0.0),
                    (0.0, 0.0, 0.0, 1.0),
                    rospy.Time.now(),
                    "state_15",
                    "state_1")
        br.sendTransform((3.5, 0.0, 0.0),
                    (0.0, 0.0, 0.0, 1.0),
                    rospy.Time.now(),
                    "state_16",
                    "state_1")
        br.sendTransform((3.5, 0.7, 0.0),
                    (0.0, 0.0, 0.0, 1.0),
                    rospy.Time.now(),
                    "state_17",
                    "state_1")
        br.sendTransform((3.5, 1.4, 0.0),
                    (0.0, 0.0, 0.0, 1.0),
                    rospy.Time.now(),
                    "state_18",
                    "state_1")
        rate.sleep()