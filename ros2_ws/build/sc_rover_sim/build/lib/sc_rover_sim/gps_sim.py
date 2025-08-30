import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Header
import time, math

class GPSSim(Node):
    def __init__(self):
        super().__init__('gps_sim')
        self.pub = self.create_publisher(NavSatFix, '/fix', 10)
        self.timer = self.create_timer(1.0, self.tick)
        self.t0 = time.time()

    def tick(self):
        msg = NavSatFix()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        # produce small oscillation around a nominal location
        t = time.time() - self.t0
        msg.latitude = 37.0 + 0.0001 * math.sin(t)
        msg.longitude = -122.0 + 0.0001 * math.cos(t)
        msg.altitude = 10.0
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = GPSSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
