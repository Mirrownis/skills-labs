import sys

from interfaces.srv import MakeDB
from interfaces.srv import CreateItem
from interfaces.srv import DeleteItem
from interfaces.srv import CreateGoal
from interfaces.srv import DeleteGoal

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class NodeUICore(Node):
    def __init__(self):
        super().__init__('node_ui_core')
        self.node_comm = NodeUICommunicator()
        self.node_comm.make_log_db("log")
        self.node_comm.make_plan_db("plan")
        rclpy.spin_once(self.node_comm)
        self.service_choice = ""
        while True:
            self.service_choice = input("Select a service: \n"
                                        "show_item\n"
                                        "create_item\n"
                                        "delete_item\n"
                                        "show_goal\n"
                                        "create_goal\n"
                                        "delete_goal\n"
                                        "show_device_status\n"
                                        "close_ui\n"
                                        "---\n")
            if self.service_choice == "close_ui":
                break
            else:
                try:
                    getattr(self.node_comm, self.service_choice)()
                except AttributeError:
                    print("Service unknown!\n\n")


class NodeUICommunicator(Node):

    def __init__(self):
        super().__init__('node_ui_communicator')
        self.cli = None
        self.req = None
        self.future = None

    def make_log_db(self, db_name):
        self.cli = self.create_client(MakeDB, 'make_log_db')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = MakeDB.Request()
        self.req.db_name = db_name
        self.future = self.cli.call_async(self.req)

    def make_plan_db(self, db_name):
        self.cli = self.create_client(MakeDB, 'make_plan_db')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = MakeDB.Request()
        self.req.db_name = db_name
        self.future = self.cli.call_async(self.req)

    def create_item(self):
        self.cli = self.create_client(CreateItem, 'create_item')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = CreateItem.Request()
        self.req.item_desc = input("Item Description: ")
        self.future = self.cli.call_async(self.req)

    def delete_item(self):
        self.cli = self.create_client(DeleteItem, 'delete_item')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = DeleteItem.Request()
        self.req.item_id = int(input("Item ID: "))
        self.future = self.cli.call_async(self.req)

    def create_goal(self):
        self.cli = self.create_client(CreateGoal, 'create_goal')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = CreateGoal.Request()
        self.req.goal_desc = input("Goal Description: ")
        self.future = self.cli.call_async(self.req)

    def delete_goal(self):
        self.cli = self.create_client(DeleteGoal, 'delete_goal')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = DeleteGoal.Request()
        self.req.goal_id = int(input("Goal ID: "))
        self.future = self.cli.call_async(self.req)


def main():
    rclpy.init()
    rclpy.spin_once(NodeUICore())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
