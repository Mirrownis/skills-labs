import sys

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class NodeUIOutput(Node):
    """
    User information output node of the ui node cluster
    ---
    Intended to run on its own independently of the other nodes,
    to allow for information display without full access to node services.
    """

    def __init__(self):
        """
        Initialize the node and its attributes.
        ---
        :param self.subscription: topic "user_information" subscriber, receives node information,
                          stores up to 100 messages in queue to be displayed as fast as possible
        """

        super().__init__('node_ui_output')
        self.subscription = self.create_subscription(
            String,
            'user_information',
            self.listener_callback,
            100)

    def listener_callback(self, msg):
        """
        Callback function of the subscriber.
        Displays all topic messages to the command interface with a time stamp.
        """
        self.get_logger().info('I heard: "%s"' % msg.data)


def main():
    """
    entry point to the ui communicator node
    """

    """ initialise ros client library """
    rclpy.init()
    """ start the UI Output node """
    rclpy.spin(NodeUIOutput())
    """ close the node once it terminates """
    rclpy.shutdown()


if __name__ == '__main__':
    """
    block 1 execution to start main() when invoked
    """

    main()
