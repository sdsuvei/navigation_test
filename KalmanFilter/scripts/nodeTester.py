#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion

def talker():
    #pub = rospy.Publisher('chatter', String, queue_size=10)
    pubImu = rospy.Publisher('imuData', Imu, queue_size=10)
    
    msgImu = Imu()
    msgImuVector = Vector3()
    msgImuVector = [0.1, 0.2, 0.3 ]
    #msgImuVector.x = [0.1]
    #msgImuVector.y = [0.2]
    #msgImuVector.z = [0.3]

    msgQuaternion = Quaternion()
    msgQuaternion.x = 0.1
    msgQuaternion.y = 0.2
    msgQuaternion.z = 0.3
    msgQuaternion.w = 0.4
    msgImu.orientation = msgQuaternion

    #msgImu.orientation = msgImuVector2
    #msgImu.orientation_covariance = msgImuVector
    #msgImu.angular_velocity_covariance = msgImuVector
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    #for n in range(0,10):
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        #pub.publish(hello_str)
        pubImu.publish(msgImu)
        rate.sleep()


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("chatter", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass