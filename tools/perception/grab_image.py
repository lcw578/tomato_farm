import rclpy, sys
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2, numpy as np

class Grab(Node):
    def __init__(self):
        super().__init__('grab_one')
        self.saved = False
        self.sub = self.create_subscription(Image, '/verifycam/cam/image_raw', self.cb, 10)
    def cb(self, msg):
        if self.saved: return
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)
        if msg.encoding in ('rgb8', 'rgba8'):
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR if msg.encoding == 'rgb8' else cv2.COLOR_RGBA2BGR)
        cv2.imwrite(sys.argv[1], img)
        self.saved = True
        print('saved', sys.argv[1], img.shape)

rclpy.init()
n = Grab()
end = n.get_clock().now().to_msg().sec + 20
while rclpy.ok() and not n.saved:
    rclpy.spin_once(n, timeout_sec=0.5)
    if n.get_clock().now().to_msg().sec > end:
        print('TIMEOUT no image'); break
