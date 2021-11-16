#!/usr/bin/env python

import rospy
from baxter_core_msgs.msg import JointCommand

cmd = JointCommand()
cmd.mode = JointCommand.POSITION_MODE
cmd.names = ["right_s0", "right_s1", "right_e0", "right_e1", "right_w0", "right_w1", "right_w2"]
cmd.command = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

def talker():
    rospy.init_node('talker', anonymous = 'True')
    pub = rospy.Publisher('/robot/limb/right/joint_command', JointCommand, queue_size = 10)   
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish(cmd)
        rate.sleep()     

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
