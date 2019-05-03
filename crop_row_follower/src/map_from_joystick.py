#!/usr/bin/env python

#In the name of Allah
import rospy
from geometry_msgs.msg import Twist
move_cmd = Twist()
def callback(data):
    move_cmd = data

def talker():
    rospy.init_node('map_from_joystick', anonymous=True)
    rospy.Subscriber('/joy_teleop/cmd_vel', Twist, callback)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    pub.publish(move_cmd)
    rospy.loginfo("ya Allah")
    rospy.loginfo(move_cmd)
    
    
    
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pub.publish(move_cmd)
        rate.sleep()
    rospy.spin()
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass


