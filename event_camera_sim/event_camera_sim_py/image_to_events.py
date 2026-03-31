import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge
import numpy as np

from event_camera_sim.msg import Event, EventArray  # Custom messages

class EventCameraSimulator(Node):
    def __init__(self):
        super().__init__('event_camera_simulator')
        self.bridge = CvBridge()
        self.prev_log_image = None

        self.subscription = self.create_subscription(
            Image,
            '/camera/infrared1/image_raw',  # Use your actual grayscale topic
            self.image_callback,
            10)

        self.publisher = self.create_publisher(EventArray, '/event_camera/events', 10)

    def image_callback(self, msg):
        curr_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8').astype(np.float32)
        log_img = np.log1p(curr_img)

        if self.prev_log_image is not None:
            diff = log_img - self.prev_log_image
            events = []
            threshold = 0.2  # Adjust this for sensitivity

            ts = msg.header.stamp
            for y in range(diff.shape[0]):
                for x in range(diff.shape[1]):
                    d = diff[y, x]
                    if abs(d) >= threshold:
                        event = Event()
                        event.x = x
                        event.y = y
                        event.timestamp = ts
                        event.polarity = d > 0
                        events.append(event)

            if events:
                event_array = EventArray()
                event_array.header = Header()
                event_array.header.stamp = ts
                event_array.events = events
                self.publisher.publish(event_array)

        self.prev_log_image = log_img.copy()

def main(args=None):
    rclpy.init(args=args)
    node = EventCameraSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

