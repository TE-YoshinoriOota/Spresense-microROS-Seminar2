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
from geometry_msgs.msg import TransformStamped

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

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
    # print("update odom")
    if self.pub_odom_tf:
      odom_tf = TransformStamped()
      odom_tf.header.frame_id = "/odom"
      odom_tf.header.stamp = msg.header.stamp
      odom_tf.child_frame_id  = "/base_footprint"
      odom_tf.transform.translation.x = msg.pose.pose.position.x
      odom_tf.transform.translation.y = msg.pose.pose.position.y
      odom_tf.transform.translation.z = msg.pose.pose.position.z
      odom_tf.transform.rotation.x    = msg.pose.pose.orientation.x
      odom_tf.transform.rotation.y    = msg.pose.pose.orientation.y
      odom_tf.transform.rotation.z    = msg.pose.pose.orientation.z
      odom_tf.transform.rotation.w    = msg.pose.pose.orientation.w
      odom_tfmsg = TFMessage()
      odom_tfmsg.transforms = [odom_tf]
      # print("publish odom_tf")
      self.pub_tf.publish(odom_tfmsg)
      # self.pub_tf.sendTransform(odom_tf)


    # publish base_link
    stf0 = TransformStamped()
    stf0.header.frame_id = "/base_footprint"
    stf0.header.stamp = msg.header.stamp
    stf0.child_frame_id = "/base_link"
    stf0.transform.translation.x = 0.0
    stf0.transform.translation.y = 0.0
    stf0.transform.translation.z = 0.036
    # angle = math.pi/2
    angle = 0
    q = quaternion_from_euler(0,0,angle)
    stf0.transform.rotation.x    = q[0]
    stf0.transform.rotation.y    = q[1]
    stf0.transform.rotation.z    = q[2]
    stf0.transform.rotation.w    = q[3]

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
    stf1.transform.rotation.x    = q[0]
    stf1.transform.rotation.y    = q[1]
    stf1.transform.rotation.z    = q[2]
    stf1.transform.rotation.w    = q[3]

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
    stf2.transform.rotation.x    = q[0]
    stf2.transform.rotation.y    = q[1]
    stf2.transform.rotation.z    = q[2]
    stf2.transform.rotation.w    = q[3]
    
    # static_tfmsg = TFMessage()
    # static_tfmsg.transforms = [stf0, stf1]
    # print("publish static_tf")
    # self.pub_stf.publish(static_tfmsg)
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

