from interfaces.srv import Plan

import rclpy
from rclpy.node import Node
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
        self.msg = String()

        self.wall_time = 0
        self.ticker = threading.Event()
        self.scheduled_plans = []
        self.publisher_ = self.create_publisher(String, 'user_information', 10)

        """ fetch existing plans """
        self.get_backlog()

    def get_backlog(self):
        """
        Fetches plans that are already created,
        and adds those in the future to the schedule
        """

        """ import NodePlanCore to access database """
        from .node_plan_core import NodePlanCore

        """ update time """
        self.wall_time = int(time.time())

        """ get backlog from NodePlanCore """
        self.scheduled_plans = NodePlanCore.db_get_backlog(NodePlanCore(), self.wall_time)

        """ sort schedule """
        self.sort_schedule()

        """ inform user about action and send requested details """
        self.msg.data = 'PlanScheduler: Scheduled backlog %s!' % str(self.scheduled_plans)
        self.publisher_.publish(self.msg)
        self.get_logger().info('Publishing: %s' % self.msg.data)

    def add_to_schedule(self, plan_id, begin_date):
        """
        add a plan to the schedule, called when a new plan is made or an existing one is changed
        in the plan core node
        :param plan_id: the id of the plan
        :param begin_date: the begin date of the plan
        """

        """ insert plan into schedule """
        schedule_entry = (plan_id, begin_date)
        self.scheduled_plans.append(schedule_entry)

        """ sort schedule """
        self.sort_schedule()

    def remove_from_schedule(self, plan_id):
        """
        remove a plan from the schedule, called when a plan is deleted or changed
        :param plan_id: id of the plan to be removed
        """

        """ remove plan from schedule """
        self.scheduled_plans = list(filter(lambda x: x[0] != plan_id, self.scheduled_plans))

    def sort_schedule(self):
        """
        sorts the scheduled plans by begin_date
        """

        self.scheduled_plans.sort(key=lambda tup: tup[1])


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