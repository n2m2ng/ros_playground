#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy

def callback(data):
    rospy .loginfo ("*")
    print 'buttons:["%s %s %s %s %s %s %s %s %s %s %s %s %s"]' % (data.buttons[0],data.buttons[1],data.buttons[2],data.buttons[3],data.buttons[4],data.buttons[5],data.buttons[6],data.buttons[7],data.buttons[8],data.buttons[9],data.buttons[10],data.buttons[11],data.buttons[12])
    print 'axes: ["%s %s %s %s %s %s"]' % (data.axes[0],data.axes[1],data.axes[2],data.axes[3],data.axes[4],data.axes[5])

    pan_joint = data.axes[0]*1024+1024
    tilt_joint = data.axes[1]*1024+1024
    pan_joint_int = int(pan_joint)
    tilt_joint_int = int(tilt_joint)
    print pan_joint_int
    print tilt_joint_int
    print type(pan_joint_int)
    print type(tilt_joint_int)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy. Subscriber("joy" , Joy , callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
