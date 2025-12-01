#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud, PointCloud2
import sensor_msgs.point_cloud2 as pc2

def callback(msg):
    # Convert PointCloud -> PointCloud2
    points = [(p.x, p.y, p.z) for p in msg.points]
    header = msg.header
    pc2_msg = pc2.create_cloud_xyz32(header, points)
    pc2_msg.header.frame_id = "front_sonar"
    pub.publish(pc2_msg)

if __name__ == '__main__':
    rospy.init_node('sonar_pc1_to_pc2')
    pub = rospy.Publisher('/RosAria/sonar_pc2', PointCloud2, queue_size=10)
    sub = rospy.Subscriber('/RosAria/sonar', PointCloud, callback)
    rospy.spin()
