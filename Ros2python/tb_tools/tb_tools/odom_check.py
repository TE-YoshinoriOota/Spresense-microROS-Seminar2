import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy as np

class OdometryListener(Node):
    def __init__(self):
        super().__init__('odometry_listener')
        self.x_data = [0]
        self.y_data = [0]
        self.subscription = self.create_subscription(
              Odometry, '/odom', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning
        try:
            plt.ion()
            fig,ax=plt.subplots()
            ax.set_aspect('equal')
            ax.set_xlim(-0.5, 0.5)
            ax.set_ylim(-0.2, 0.5)
            ax.xaxis.set_major_locator(ticker.MultipleLocator(0.1))
            ax.yaxis.set_major_locator(ticker.MultipleLocator(0.1))
            for spine in ['left', 'bottom']:
                ax.spines[spine].set_linewidth(1)
            ax.spines['left'].set_position('zero')
            ax.spines['bottom'].set_position('zero')
            plt.grid(True)
            plt.show()
        except KeyboardInterrupt:
            plt.close()

    def listener_callback(self, msg):
        p = msg.pose.pose.position
        print(f'Position: x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}')
        q = msg.pose.pose.orientation
        qt = [q.x, q.y, q.z, q.w]
        r = R.from_quat(qt)
        el = r.as_euler('xyz', degrees=True)
        print(f'Degree  : x={el[0]:.3f}, y={el[1]:.3f}, z={el[2]:.3f}')
        self.x_data.append(p.x)
        self.y_data.append(-p.y)
        try:
            plt.plot(self.y_data, self.x_data, color='blue', marker="o")
            r = 0.03
            ex = p.x + r*np.cos(np.radians(el[2])) 
            ey = p.y + r*np.sin(np.radians(el[2]))
            rx = [ p.x,  ex]
            ry = [-p.y, -ey]
            plt.plot(ry, rx, color='red')
            plt.pause(0.01)
            plt.draw()
        except KeyboardInterrupt:
            plt.savefig("graph.png")
            plt.close()

def main(args=None):
    rclpy.init(args=args)
    odometry_listener = OdometryListener()
    rclpy.spin(odometry_listener)
    odometry_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

