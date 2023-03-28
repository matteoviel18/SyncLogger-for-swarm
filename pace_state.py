#! /usr/bin/env python

# pace that triggers the publication of the state

import rospy
from std_msgs.msg import Empty

if __name__ == '__main__':

    rospy.init_node('pace_state')
    pub = rospy.Publisher('/pace_state', Empty, queue_size=1)

    rate = rospy.Rate(1)  # pace
    pace = Empty()

    while not rospy.is_shutdown():
        pub.publish(pace)
        rate.sleep()
