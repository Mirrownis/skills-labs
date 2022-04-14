import sys

from interfaces.srv import Item
from interfaces.srv import Plan

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class NodeNavCore(Node):
    """
    ???
    ---
    ???
    """

    def __init__(self):
        """
        ???
        ---
        :param ???: ???
        """

        super().__init__('node_nav_core')
        self.srv_execute_plan = self.create_service(Plan, 'execute_plan', self.callback_execute_plan)

    def callback_execute_plan(self, request, response):
        """
        callback for executing a plan as provided
        :param request: service request containing ???
        :param response: service response acknowledging the task
        :return: acknowledgement of the task
        """

        return response


def main():
    """
    entry point to the nav core node
    """

    """ initialise ros client library """
    rclpy.init()
    """ start the Nav Core node """
    rclpy.spin(NodeNavCore())
    """ close the node once it terminates """
    rclpy.shutdown()


if __name__ == '__main__':
    """
    block 1 execution to start main() when invoked
    """

    main()
