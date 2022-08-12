#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, Vector3, Quaternion


class transformerNode():
    def __init__(self):
        rospy.init_node("poseTransform", anonymous=True)
        self.sub = rospy.Subscriber("vectornav/IMU", Imu, self.transformCallback)
        self.pub = rospy.Publisher("poseTransformed", PoseStamped, queue_size=100)
        rospy.spin()

    def transformCallback(self, msgIn):
        msgOut = PoseStamped()
        msgOut.header = msgIn.header
        msgOut.header.frame_id = "map"    
        msgOut.pose.position = Vector3(0,0,0)
        msgOut.pose.orientation = Quaternion(msgIn.orientation.w, msgIn.orientation.x, msgIn.orientation.y, msgIn.orientation.z)
        self.pub.publish(msgOut)


if __name__ == '__main__':
    try:
        node = transformerNode()
        rospy.spin

    except rospy.ROSInterruptException:
        pass
