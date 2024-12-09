import rclpy
import sys
import math
import numpy as np
from rclpy.clock import Clock
from rclpy.node import Node
from rclpy.qos import QoSProfile
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from scipy.spatial.transform import Rotation as R


from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

class Quaternion:
    w: float
    x: float
    y: float
    z: float

def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr

    return q 


class OdomToTf(Node):
  def __init__(self, pub_odom_tf_):
    super().__init__('odom_to_tf')

    self.pub_odom_tf = pub_odom_tf_
    print("[tm_tfpublisher] create odometry subscriber")
    self.sub_odom = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
    if self.pub_odom_tf:
      print("[tm_tfpublisher] transform publisher")
      self.pub_tf   = self.create_publisher(TFMessage, "/tf", 10)
      # self.pub_tf = TransformBroadcaster(self)

    print("[tm_tfpublisher] static transform publisher")
    # self.pub_stf  = self.create_publisher(TFMessage, "/tf_static", 10)
    self.pub_stf = StaticTransformBroadcaster(self)


  def odom_callback(self, msg):
    print(f"update tf2 {msg.header.stamp}")
    p_x = msg.pose.pose.position.x
    p_y = msg.pose.pose.position.y
    p_z = msg.pose.pose.position.z
    q_x = msg.pose.pose.orientation.x
    q_y = msg.pose.pose.orientation.y
    q_z = msg.pose.pose.orientation.z
    q_w = msg.pose.pose.orientation.w
    q = [q_x, q_y, q_z, q_w]
    r = R.from_quat(q)
    euler_angles = r.as_euler('xyz', degrees=False)

    print(f"body_angle : {euler_angles[2]}")
    if self.pub_odom_tf:
      # print("publish odom_tf")
      odom_tf = TransformStamped()
      odom_tf.header.frame_id = "/odom"
      odom_tf.header.stamp = msg.header.stamp
      odom_tf.child_frame_id  = "/base_footprint"
      odom_tf.transform.translation.x = p_x
      odom_tf.transform.translation.y = p_y
      odom_tf.transform.translation.z = p_z
      odom_tf.transform.rotation.x    = q_x
      odom_tf.transform.rotation.y    = q_y
      odom_tf.transform.rotation.z    = q_z
      odom_tf.transform.rotation.w    = q_w
      odom_tfmsg = TFMessage()
      odom_tfmsg.transforms = [odom_tf]
      self.pub_tf.publish(odom_tfmsg)
      # self.pub_tf.sendTransform(odom_tf)
      print(f"publish tf of odom to base_footprint pos: {p_x,p_y,p_z} q: {q_x,q_y,q_z,q_w}")

    # publish base_link
    stf0 = TransformStamped()
    stf0.header.frame_id = "/base_footprint"
    stf0.header.stamp = msg.header.stamp
    stf0.child_frame_id = "/base_link"
    stf0.transform.translation.x = 0.0
    stf0.transform.translation.y = 0.0
    stf0.transform.translation.z = 0.036
    angle = 0
    q = quaternion_from_euler(0,0,angle)
    stf0.transform.rotation.x    = q.x
    stf0.transform.rotation.y    = q.y
    stf0.transform.rotation.z    = q.z
    stf0.transform.rotation.w    = q.w
    print(f"publish tf of base_footprint to base_link {q.x, q.y, q.z, q.w}")

    # publish scan_link
    stf1 = TransformStamped()
    stf1.header.frame_id = "/base_link"
    stf1.header.stamp = msg.header.stamp
    stf1.child_frame_id = "/scan_link"
    stf1.transform.translation.x = 0.0
    stf1.transform.translation.y = 0.0
    stf1.transform.translation.z = 0.050
    angle = math.pi/2
    q = quaternion_from_euler(0,0,angle)
    stf1.transform.rotation.x    = q.x
    stf1.transform.rotation.y    = q.y
    stf1.transform.rotation.z    = q.z
    stf1.transform.rotation.w    = q.w
    print(f"publish tf of base_link to scan_link {q.x, q.y, q.z, q.w}")

    # publish imu_link
    stf2 = TransformStamped()
    stf2.header.frame_id = "/base_link"
    stf2.header.stamp = msg.header.stamp
    stf2.child_frame_id = "/imu_link"
    stf2.transform.translation.x = 0.0
    stf2.transform.translation.y = 0.0
    stf2.transform.translation.z = 0.036
    angle = 0
    q = quaternion_from_euler(0,0,angle)
    stf2.transform.rotation.x    = q.x
    stf2.transform.rotation.y    = q.y
    stf2.transform.rotation.z    = q.z
    stf2.transform.rotation.w    = q.w
    print(f"publish tf of base_link to imu_link {q.x, q.y, q.z, q.w}")
    
    # print("publish static_tf")
    self.pub_stf.sendTransform(stf0)
    self.pub_stf.sendTransform(stf1)
    self.pub_stf.sendTransform(stf2)

def main(args=None):
  pub_odom_tf = False
  for s in sys.argv:
    if 'odom_tf' in s and (('True' in s or 'true' in s)):
      pub_odom_tf = True

  rclpy.init(args=args)
  node = OdomToTf(pub_odom_tf)

  rclpy.spin(node)

  node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()

