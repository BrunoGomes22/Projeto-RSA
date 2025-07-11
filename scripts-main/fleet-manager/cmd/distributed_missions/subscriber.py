import argparse

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

topic_name = ''


class MinimalSubscriber(Node):
    global topic_name

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            topic_name,
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        print(msg.data)

def main(args=None):
    global topic_name

    parser = argparse.ArgumentParser()
    parser.add_argument('-t', '--topic', required=True)
    cli_args = parser.parse_args()

    topic_name = cli_args.topic

    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
