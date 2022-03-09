import sys

from interfaces.srv import MakeDB
from interfaces.srv import Item
from interfaces.srv import Goal
from interfaces.srv import Plan

import rclpy
from rclpy.node import Node


class NodeUICore(Node):
    """
    Core node of the ui node cluster
    ---
    After start-up, it connects to the logistics and planning nodes and takes user input.
    """

    def __init__(self):
        """
        Initialize the node and its attributes, then runs start-up actions.
        ---
        :param self.node_comm: defines the communicator node to be used to make service calls
        :param self.service_choice: holds the last user choice to let the communicator node execute it
        """
        super().__init__('node_ui_core')
        self.node_comm = NodeUICommunicator()
        self.service_choice = ""

        """ instruct the logistics node to make (or connect to an existing) logistics database """
        self.node_comm.make_log_db()
        """ instruct the planning node to make (or connect to an existing) planning database """
        self.node_comm.make_plan_db()
        rclpy.spin_until_future_complete(self.node_comm, self.node_comm.future)
        """ show the user their options, and take their input via command line input """
        self.select_service()

    def select_service(self):
        """ show the user their options, and take their input via command line input """
        while True:
            self.service_choice = input("\n---\n"
                                        "Select a service: \n"
                                        "show_item\n"
                                        "show_goal\n"
                                        "show_plan\n"
                                        "show_device_status\n"
                                        "create_item\n"
                                        "create_goal\n"
                                        "create_plan\n"
                                        "edit_item\n"
                                        "edit_goal\n"
                                        "edit_plan\n"
                                        "delete_item\n"
                                        "delete_goal\n"
                                        "delete_plan\n"
                                        "close_ui\n"
                                        "---\n")
            """ exit the service, or call a service of the communicator node """
            if self.service_choice == "close_ui":
                break
            else:
                try:
                    getattr(self.node_comm, self.service_choice)()
                except AttributeError:
                    print("Service unknown!\n\n")


class NodeUICommunicator(Node):
    """
    Communicator node of the ui node cluster
    ---
    Is not intended to run continuously, instead just to send service requests to other node clusters.
    Can be called multiple times in parallel by the Core UI Node to allow for parallel execution
    """

    def __init__(self):
        """
        Initialize the node and its attributes.
        ---
        :param self.cli: holds the client that calls services of other nodes
        :param self.req: holds the request for services of other nodes
        :param self.future: holds the response to service calls
        """
        
        super().__init__('node_ui_communicator')
        self.cli = None
        self.req = None
        self.future = None

    def make_log_db(self):
        """
        Calls the logistics node to make or connect to its database.
        ---
        Note: Doesn't need any additional input, as the database is set.
        """

        """ create service client for MakeDB interface under the name 'make_log_db' """
        self.cli = self.create_client(MakeDB, 'make_log_db')
        """ wait for the server to be available """
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        """ define the request for the MakeDB interface """
        self.req = MakeDB.Request()
        """ call the service as defined above """
        self.future = self.cli.call_async(self.req)

    def make_plan_db(self):
        """
        Calls the planning node to make or connect to its database.
        ---
        Note: Doesn't need any additional input, as the database is set.
        """

        """ create service client for MakeDB interface under the name 'make_plan_db' """
        self.cli = self.create_client(MakeDB, 'make_plan_db')
        """ wait for the server to be available """
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        """ define the request for the MakeDB interface """
        self.req = MakeDB.Request()
        """ call the service as defined above """
        self.future = self.cli.call_async(self.req)

    def create_item(self):
        """
        Calls the logistics node to create a new item in its database.
        ---
        Asks for an item description, which the planning node looks for when reserving items.
        Asks for a position of the item to be used by the navigation node to find it.
        """

        """ create service client for Item interface under the name 'create_item' """
        self.cli = self.create_client(Item, 'create_item')
        """ wait for the server to be available """
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        """ define the request for the Item interface """
        self.req = Item.Request()
        """ take input for the parameters of the service interface """
        self.req.item_kind = input("Item Description: ")
        item_position = "[" + input("Item position x: ") + " " + input("Item position y: ") + "]"
        self.req.position = item_position
        """ call the service as defined above """
        self.future = self.cli.call_async(self.req)

    def edit_item(self):
        """
        Calls the logistics node to edit an existing item description manually.
        ---
        Asks for a new item description, which the planning node looks for when reserving items.
        """

        """ create service client for Item interface under the name 'edit_item' """
        self.cli = self.create_client(Item, 'edit_item')
        """ wait for the server to be available """
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        """ define the request for the Item interface """
        self.req = Item.Request()
        """ take input for the parameters of the service interface """
        self.req.id = int(input("ID of item to change: "))
        self.req.item_kind = input("New item description: ")
        """ call the service as defined above """
        self.future = self.cli.call_async(self.req)

    def delete_item(self):
        """
        Calls the logistics node to delete an existing item.
        ---
        Asks for the id of the item to be deleted.
        """

        """ create service client for Item interface under the name 'delete_item' """
        self.cli = self.create_client(Item, 'delete_item')
        """ wait for the server to be available """
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        """ define the request for the Item interface """
        self.req = Item.Request()
        """ take input for the parameters of the service interface """
        self.req.id = int(input("Item ID: "))
        """ call the service as defined above """
        self.future = self.cli.call_async(self.req)

    def show_item(self):
        """
        Calls the logistics node to show an item.
        ---
        Asks for the id of the item to be shown.
        ---
        Note: The item data will be sent to the user info node.
        """

        """ create service client for Item interface under the name 'show_item' """
        self.cli = self.create_client(Item, 'show_item')
        """ wait for the server to be available """
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        """ define the request for the Item interface """
        self.req = Item.Request()
        """ take input for the parameters of the service interface """
        self.req.id = int(input("Item ID: "))
        """ call the service as defined above """
        self.future = self.cli.call_async(self.req)

    def create_goal(self):
        """
        Calls the planning node to create a new goal in its database.
        ---
        Asks for a goal description that lays out the actions to be executed by the navigation node.
        """

        """ create service client for Goal interface under the name 'create_goal' """
        self.cli = self.create_client(Goal, 'create_goal')
        """ wait for the server to be available """
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        """ define the request for the Goal interface """
        self.req = Goal.Request()
        """ take input for the parameters of the service interface """
        self.req.goal_desc = input("Goal Description: ")
        self.req.instructions = input("Machine Instructions: ")
        self.req.items = input("Items: ")
        self.req.room_kind = input("Kind of Room: ")
        """ call the service as defined above """
        self.future = self.cli.call_async(self.req)

    def edit_goal(self):
        """
        Calls the planning node to edit an existing goal description manually.
        ---
        Asks for the id of the goal to be changed.
        Asks for a new goal description that lays out the actions to be executed by the navigation node.
        """

        """ create service client for Goal interface under the name 'edit_goal' """
        self.cli = self.create_client(Goal, 'edit_goal')
        """ wait for the server to be available """
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        """ define the request for the Goal interface """
        self.req = Goal.Request()
        """ take input for the parameters of the service interface """
        self.req.goal_id = int(input("ID of goal to change: "))
        self.req.goal_desc = input("Goal Description: ")
        self.req.instructions = input("Machine Instructions: ")
        self.req.items = input("Items: ")
        self.req.room_kind = input("Kind of Room: ")
        """ call the service as defined above """
        self.future = self.cli.call_async(self.req)

    def delete_goal(self):
        """
        Calls the planning node to delete an existing goal.
        ---
        Asks for the id of the goal to be deleted.
        """

        """ create service client for Goal interface under the name 'delete_goal' """
        self.cli = self.create_client(Goal, 'delete_goal')
        """ wait for the server to be available """
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        """ define the request for the Goal interface """
        self.req = Goal.Request()
        """ take input for the parameters of the service interface """
        self.req.goal_id = int(input("Goal ID: "))
        """ call the service as defined above """
        self.future = self.cli.call_async(self.req)

    def show_goal(self):
        """
        Calls the planning node to show an existing goal.
        ---
        Asks for the id of the goal to be shown.
        ---
        Note: The item data will be sent to the user info node.
        """

        """ create service client for Goal interface under the name 'show_goal' """
        self.cli = self.create_client(Goal, 'show_goal')
        """ wait for the server to be available """
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        """ define the request for the Goal interface """
        self.req = Goal.Request()
        """ take input for the parameters of the service interface """
        self.req.goal_id = int(input("Goal ID: "))
        """ call the service as defined above """
        self.future = self.cli.call_async(self.req)

    def create_plan(self):
        """
        Calls the planning node to edit a plans info manually.
        ---
        Asks for a new goal id, which holds the plan description.
        Asks for the used item kinds, which are then looked for in the logistics database.
        Asks for the beginning and end date of the plan, so that the plan can be scheduled
        and the items can be reserved for the time.
        """

        """ create service client for Plan interface under the name 'create_plan' """
        self.cli = self.create_client(Plan, 'create_plan')
        """ wait for the server to be available """
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        """ define the request for the CreatePlan interface """
        self.req = Plan.Request()
        """ take input for the parameters of the service interface """
        self.req.goal_id = int(input("ID of goal to achieve: "))
        self.req.begin_date = int(input("Starting date of plan (UNIX): "))
        self.req.end_date = int(input("Ending date of plan (UNIX): "))
        """ call the service as defined above """
        self.future = self.cli.call_async(self.req)

    def edit_plan(self):
        """
        Calls the planning node to edit a plan's info manually.
        ---
        Asks for a new goal id, the used item kinds, the beginning and end date of the plan.
        """

        """ create service client for Plan interface under the name 'edit_plan' """
        self.cli = self.create_client(Plan, 'edit_plan')
        """ wait for the server to be available """
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        """ define the request for the EditPlan interface """
        self.req = Plan.Request()
        """ take input for the parameters of the service interface """
        self.req.plan_id = int(input("ID of plan to change: "))
        self.req.goal_id = int(input("New id of goal to achieve: "))
        self.req.begin_date = int(input("New starting date of plan (UNIX): "))
        self.req.end_date = int(input("New ending date of plan (UNIX): "))
        """ call the service as defined above """
        self.future = self.cli.call_async(self.req)

    def delete_plan(self):
        """
        Calls the planning node to delete a plan.
        ---
        Asks for the id of the plan to be deleted.
        """

        """ create service client for Plan interface under the name 'delete_plan' """
        self.cli = self.create_client(Plan, 'delete_plan')
        """ wait for the server to be available """
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        """ define the request for the DeletePlan interface """
        self.req = Plan.Request()
        """ take input for the parameters of the service interface """
        self.req.plan_id = int(input("Plan ID: "))
        """ call the service as defined above """
        self.future = self.cli.call_async(self.req)

    def show_plan(self):
        """
        Calls the planning node to show an existing plan.
        ---
        Asks for the id of the plan to be shown.
        ---
        Note: The item data will be sent to the user info node.
        """

        """ create service client for Plan interface under the name 'show_plan' """
        self.cli = self.create_client(Plan, 'show_plan')
        """ wait for the server to be available """
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        """ define the request for the Plan interface """
        self.req = Plan.Request()
        """ take input for the parameters of the service interface """
        self.req.plan_id = int(input("Plan ID: "))
        """ call the service as defined above """
        self.future = self.cli.call_async(self.req)


def main():
    """
    entry point to the ui communicator node
    """

    """ initialise ros client library """
    rclpy.init()
    """ start the UI Output node """
    rclpy.spin(NodeUICore())
    """ close the node once it terminates """
    rclpy.shutdown()


if __name__ == '__main__':
    """
    block 1 execution to start main() when invoked
    """

    main()
