#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import UInt8
 
def fiber_sel():
    pub = rospy.Publisher('/agiltron/read_fiber', UInt8, queue_size=10)
    rospy.init_node('fiber_sel', anonymous=True)
    rate = rospy.Rate(.1) # 110hz
    fiber_num = 0
    while not rospy.is_shutdown():
        fiber_num += 1
        fiber_num  %= 5
        if fiber_num == 0:
            fiber_num = 1
        rospy.loginfo(fiber_num)
        pub.publish(fiber_num)
        rate.sleep()
 
if __name__ == '__main__':
    try:
        fiber_sel()
    except rospy.ROSInterruptException:
        pass