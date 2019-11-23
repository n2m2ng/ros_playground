#!/usr/bin/env python
# coding: UTF-8
import rospy
import sys
sys.path.append('/user/catkin_ws/dynamixel-workbench-msgs')

from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy
from dynamixel_workbench_msgs.msg import DynamixelStateList
from dynamixel_workbench_msgs.srv import DynamixelCommand, DynamixelCommandResponse

SERVO_ID_1 = 1  # サーボID 1
SERVO_ID_2 = 2  # サーボID 2
SERVO_ID_3 = 3  # サーボID 3
SERVO_ID_4 = 4  # サーボID 4

def callback(data):
    rospy.loginfo("*")
    print 'buttons: ["%s %s %s %s %s %s %s %s %s %s %s %s %s"]' % (data.buttons[0], data.buttons[1], data.buttons[2], data.buttons[3], data.buttons[4], data.buttons[5], data.buttons[6], data.buttons[7], data.buttons[8], data.buttons[9], data.buttons[10], data.buttons[11], data.buttons[12])
    print 'axes: ["%s %s %s %s %s %s"]' % (data.axes[0], data.axes[1], data.axes[2], data.axes[3], data.axes[4], data.axes[5])

    # dynamixel_commandサービスが公開されるのを待つ
    rospy.wait_for_service('dynamixel_workbench/dynamixel_command')

    try:
        # ローカルプロキシにサービス名(dynamixel_command)と型(DynamixelCommand)を設定
        service = rospy.ServiceProxy('dynamixel_workbench/dynamixel_command', DynamixelCommand)
        id1_joint = data.axes[0] * 1024 + 1024
        id2_joint = data.axes[1] * 1024 + 1024
        id3_joint = data.axes[2] * 1024 + 1024
        id4_joint = data.axes[3] * 1024 + 1024
        id1_joint_int = int(id1_joint)
        id2_joint_int = int(id2_joint)
        id3_joint_int = int(id3_joint)
        id4_joint_int = int(id4_joint)
        print id1_joint_int
        print id2_joint_int
        print id3_joint_int
        print id4_joint_int
        response = service('', SERVO_ID_1, 'Goal_Position', id1_joint_int)
        response = service('', SERVO_ID_2, 'Goal_Position', id2_joint_int)
        response = service('', SERVO_ID_3, 'Goal_Position', id3_joint_int)
        response = service('', SERVO_ID_4, 'Goal_Position', id4_joint_int)
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e
    return DynamixelCommandResponse


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("joy" , Joy, callback)
    rospy.wait_for_service("/dynamixel_workbench/dynamixel_command")
    rospy.spin()


if __name__ == '__main__':
    listener()
