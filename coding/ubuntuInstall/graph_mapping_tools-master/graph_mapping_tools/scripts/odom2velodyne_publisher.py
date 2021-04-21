#!/usr/bin/python
import tf
import rospy
from geometry_msgs.msg import *
from nav_msgs.msg import *


class Map2OdomPublisher:
	def __init__(self):
		self.broadcaster = tf.TransformBroadcaster()
		self.subscriber = rospy.Subscriber('/aft_mapped_to_init_10', Odometry, self.callback)

	def callback(self, odom_msg):
		self.odom_msg = odom_msg

	def spin(self):
		if not hasattr(self, 'odom_msg'):
			self.broadcaster.sendTransform((0, 0, 0), (0, 0, 0, 1), rospy.Time.now(), 'odom_graph_tools', 'velodyne_graph_tools')
			return

		pose = self.odom_msg.pose
		pos = (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
		quat = (pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w)

		map_frame_id = self.odom_msg.header.frame_id
		odom_frame_id = self.odom_msg.child_frame_id

                self.broadcaster.sendTransform(pos, quat, rospy.Time.now(), odom_frame_id, map_frame_id)


def main():
	rospy.init_node('odom2velodyne_publisher')
	node = Map2OdomPublisher()

        rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		node.spin()
		rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except:
        print 'shutdown'

