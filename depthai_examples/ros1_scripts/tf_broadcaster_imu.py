#!/usr/bin/env python3

from cmath import pi
import rospy
import tf_conversions # because of transformations
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
import tf2_ros
import geometry_msgs.msg
from sensor_msgs.msg import Imu

def broadcast_cam_pose(msg):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "oak-d-base-frame"
    t.child_frame_id = "oak-d_frame"
    t.transform.translation.x = 0
    t.transform.translation.y = 0
    t.transform.translation.z = 0
    orientation_q = msg.orientation

    q_imu = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]

    # Origin quat
    q1_inv = [0, 0, 0, -1]

    # Relative rot quat from origin to q_imu
    q_rel = quaternion_multiply(q_imu, q1_inv)

    # Rotate that quat with -90 deg around y-axis
    q_rot = quaternion_from_euler(0, -1.5708, 0)
    # Locally rotate it (depends on order of multiplication)
    q_new = quaternion_multiply(q_rel, q_rot)

    t.transform.rotation.x = q_new[0]
    t.transform.rotation.y = q_new[1]
    t.transform.rotation.z = q_new[2]
    t.transform.rotation.w = q_new[3]
    
    br.sendTransform(t)
    
if __name__ == '__main__':
      rospy.init_node('tf_broadcaster_imu')
      rospy.Subscriber('/stereo_inertial_publisher/imu/data',Imu, broadcast_cam_pose)
      rospy.spin()