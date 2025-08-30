import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import numpy as np, time, math

class CameraSim(Node):
    def __init__(self):
        super().__init__('camera_sim')
        self.pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.timer = self.create_timer(0.1, self.tick)
        self.t0 = time.time()

    def tick(self):
        w,h = 320,240
        arr = np.zeros((h,w,3), dtype=np.uint8)
        t = time.time()-self.t0
        arr[:,:,0] = (np.linspace(0,255,w)[None,:]).astype(np.uint8)
        arr[:,:,1] = int((math.sin(t)+1)*127)
        msg = Image()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height, msg.width = h,w
        msg.encoding = 'rgb8'
        msg.step = w*3
        msg.data = arr.tobytes()
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = CameraSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
