from interfaces.srv import Goal
from interfaces.srv import Plan

import rclpy
from rclpy.node import Node
import sqlite3
from sqlite3 import Error
from std_msgs.msg import String
import time
import threading


class NodePlanScheduler(Node):
    """
    User information output node of the plan scheduler node
    ---
    Intended to run on its own independently of the other nodes,
    to allow for information display without full access to node services.
    """

    def __init__(self):
        """
        Initialize the node and its attributes.
        ---
        :param self.wall_time: current UNIX time
        :param self.ticker: time intervall in seconds between schedule updates
        :param self.scheduled_plans: array of the scheduled plans
        """

        super().__init__('node_ui_output')
        self.cli = None
        self.req = None
        self.future = None

        self.wall_time = 0
        self.ticker = threading.Event()
        self.scheduled_plans = []
        self.conn = None
        self.db_file = r"sqlite/plan.db"
        self.db_make_connection()
        """ fetch existing plans """
        self.get_backlog()
        """ start scheduling service """
        self.scheduler_loop()

    def get_backlog(self):
        """
        Fetches plans that are already created,
        and adds those in the future to the schedule
        """

        """ update time """
        self.wall_time = int(time.time())

        """ create service client for Item interface under the name 'reserve_item' """
        self.cli = self.create_client(Plan, 'get_backlog')
        """ wait for the server to be available """
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        """ define the request for the Plan interface """
        self.req = Plan.Request()
        """ set the parameters of the service interface """
        self.req.begin_date = self.wall_time
        """ call the service as defined above """
        self.future = self.cli.call_async(self.req)

        """ save the results """
        backlog_result_ids = self.node_comm.future.result().plan_id
        backlog_result_begin_date = self.node_comm.future.result().begin_date
        """ arrange the backlog in the schedule """
        self.scheduled_plans = zip(backlog_result_ids, backlog_result_begin_date)

        """ sort backlog """
        self.sort_schedule()

    def sort_schedule(self):
        """
        sorts the scheduled plans by begin_date
        """

        self.scheduled_plans.sort(key=lambda tup: tup[1])

    def scheduler_loop(self):
        """
        checks if current
        """

        while True:
            self.wall_time = int(time.time())
            next_plan = self.scheduled_plans[0]
            if next_plan(1) >= self.wall_time:
                """ execute plan """
                self.executed_plan()
                """ ... """
            else:
                """ wait a second """

    def execute_plan(self):
        """
        Calls the planning core node to execute a plan.
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
        self.req.items = input("New array of used items: ")
        self.req.begin_time = int(input("New starting time of plan (UNIX): "))
        self.req.end_time = int(input("New ending time of plan (UNIX): "))
        """ call the service as defined above """
        self.future = self.cli.call_async(self.req)

    def check_schedule(self):
        while not self.ticker.wait(self.loop_intervall):
            self.wall_time = int(time.time())


def main():
    """
    entry point to the plan scheduler node
    """

    """ initialise ros client library """
    rclpy.init()
    """ start the Plan Scheduler node """
    rclpy.spin(NodePlanScheduler())
    """ close the node once it terminates """
    rclpy.shutdown()


if __name__ == '__main__':
    """
    block 1 execution to start main() when invoked
    """

    main()