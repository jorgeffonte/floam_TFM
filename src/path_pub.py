#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class OdometryToPath:
    def __init__(self):
        rospy.init_node('odometry_to_path')

        self.odom_sub = rospy.Subscriber('/Odometry', Odometry, self.odom_callback)
        self.path_pub = rospy.Publisher('/floam/path', Path, queue_size=10)

        self.path = Path()

    def odom_callback(self, msg):
        # Crear un nuevo PoseStamped con la posición actual
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose

        # Agregar la posición al Path
        self.path.poses.append(pose_stamped)
        self.path.header.stamp=msg.header.stamp
        self.path.header.frame_id=msg.header.frame_id
        # Publicar el Path
        self.path_pub.publish(self.path)

if __name__ == '__main__':
    try:
        odometry_to_path = OdometryToPath()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
