from interfaces.srv import MakeDB
from interfaces.srv import Goal
from interfaces.srv import Plan
from interfaces.srv import Item

import rclpy
from rclpy.node import Node
import sqlite3
from sqlite3 import Error
from std_msgs.msg import String


class NodePlanCore(Node):
    """
    Core node of the planning node cluster
    ---
    After start-up, it creates all services that can be called by clients,
    and announces its presence to the user info node.
    """

    def __init__(self):
        """
        initialize the node, its attributes and create services to be called
        ---
        :param self.conn: database connection
        :param self.db_file: name of the database to be used
        :param self.msg: string to be published to the user info to inform about actions
        :param self.node_comm: defines the communicator node to be used to make service calls
        :param self.srv_make_plan_db: service interface to connect to the planning database or make a new one
        :param self.srv_create_goal: service interface to create a new goal in the database
        :param self.srv_edit_goal: service interface to edit an existing goal in the database
        :param self.srv_show_goal: service interface to show an existing goal in the database
        :param self.srv_delete_goal: service interface to delete a goal from the database
        :param self.srv_create_plan: service interface to create a new goal in the database
        :param self.srv_edit_plan: service interface to edit a plan in the database
        :param self.srv_delete_plan: service interface to delete a plan from the database
        :param self.srv_get_backlog: service interface to find all already scheduled plans on the database
        :param self.publisher_: publisher for user info node (see self.msg)
        """

        super().__init__('node_plan_core')
        self.conn = None
        self.db_file = r"sqlite/plan.db"
        self.msg = String()
        self.node_comm = NodePlanCommunicator()

        self.srv_make_plan_db = self.create_service(MakeDB, 'make_plan_db', self.callback_make_plan_db)
        self.srv_create_goal = self.create_service(Goal, 'create_goal', self.callback_create_goal)
        self.srv_edit_goal = self.create_service(Goal, 'edit_goal', self.callback_edit_goal)
        self.srv_show_goal = self.create_service(Goal, 'show_goal', self.callback_show_goal)
        self.srv_delete_goal = self.create_service(Goal, 'delete_goal', self.callback_delete_goal)
        self.srv_create_plan = self.create_service(Plan, 'create_plan', self.callback_create_plan)
        self.srv_edit_plan = self.create_service(Plan, 'edit_plan', self.callback_edit_plan)
        self.srv_delete_plan = self.create_service(Plan, 'delete_plan', self.callback_delete_plan)
        self.srv_get_backlog = self.create_service(Plan, 'get_backlog', self.callback_get_backlog)
        self.publisher_ = self.create_publisher(String, 'user_information', 10)

        """ sends a message to user info to make sure connections are working """
        self.msg.data = 'Good Morning from PlanCore!'
        self.publisher_.publish(self.msg)
        self.get_logger().info('Publishing: %s' % self.msg.data)

    def callback_make_plan_db(self, request, response):
        """
        callback for making the 'plan' database and its tables 'goals' and 'goal_planning'
        :param request: service request
        :param response: service response acknowledging the task
        :return: acknowledgement of the task
        """

        """
        create table that holds the goals in the planning database:
        -- id: unique identifier of each individual goal
        -- goal: description of the task for users
        -- instructions: machine instructions for the planning node
        -- items: the kind of items needed
        -- room_kind: the kind of room to be reserved for the goal
        """
        sql_create_goals_table = """ CREATE TABLE IF NOT EXISTS goals (
                                     id integer PRIMARY KEY,
                                     goal text NOT NULL,
                                     instructions text NOT NULL,
                                     items text,
                                     room_kind text NOT NULL
                                     ); """

        """
        create table that holds the rooms in the planning database:
        -- id: unique identifier of each individual room
        -- room_kind: description of the room for generating a plan
        """
        sql_create_rooms_table = """ CREATE TABLE IF NOT EXISTS goals (
                                     id integer PRIMARY KEY,
                                     room_kind text NOT NULL
                                     ); """

        """
        create table that holds the scheduled plans:
        -- id: unique identifier of the scheduled plan
        -- goal_id: number of the goal planned, non-unique in goal_planning
        -- items: list of the needed items (FORMAT: thing1 thing2 thing3)
        -- begin_date: starting date of the plan
        -- end_date: ending date of the plan
        """
        sql_create_goal_planning_table = """ CREATE TABLE IF NOT EXISTS goal_planning (
                                              id integer PRIMARY KEY,
                                              goal_id integer NOT NULL,
                                              room_id integer NOT NULL,
                                              item_ids text NOT NULL,
                                              begin_date integer NOT NULL,
                                              end_date integer NOT NULL,
                                              FOREIGN KEY (goal_id) REFERENCES goals (id)
                                              ); """
        """ create a database connection """
        self.db_make_connection()
        """ create tables """
        if self.conn is not None:
            """ create "goals" table """
            self.db_make_table(sql_create_goals_table)
            """ create "rooms" table """
            self.db_make_table(sql_create_rooms_table)
            """ create "goal_planning" table """
            self.db_make_table(sql_create_goal_planning_table)
            """ inform user about action """
            self.msg.data = 'PlanCore: made connection to database'
            self.publisher_.publish(self.msg)
            self.get_logger().info('Publishing: %s' % self.msg.data)
            """ send acknowledgement to indicate successful task """
            response.ack = True
        else:
            """ output an error message """
            self.get_logger().info("Error! cannot create the database connection.")
            """ create false acknowledgement to indicate failed task """
            response.ack = False
        return response

    def callback_create_goal(self, request, response):
        """
        callback for creating a goal in the goals table
        :param request: service request containing the goal description
        :param response: service response acknowledging the task
        :return: acknowledgement of the task
        """

        """ create a database connection """
        self.db_make_connection()
        """ insert new goal into table """
        if self.conn is not None:
            """ create goal in table """
            self.db_create_goal(request.goal_desc)
            """ inform user about action """
            self.msg.data = 'PlanCore: Created goal "%s"!' % request.goal_desc
            self.publisher_.publish(self.msg)
            self.get_logger().info('Publishing: %s' % self.msg.data)
            """ send acknowledgement to indicate successful task """
            response.ack = True
        else:
            """ output an error message """
            self.get_logger().info("Error! cannot create the database connection.")
            """ create false acknowledgement to indicate failed task """
            response.ack = False
        return response

    def callback_edit_goal(self, request, response):
        """
        callback for editing a goal in the goals table
        :param request: service request containing the goal id and description
        :param response: service response acknowledging the task
        :return: acknowledgement of the task
        """

        """ create a database connection """
        self.db_make_connection()
        """ insert new goal description into table """
        if self.conn is not None:
            """ edit goal in table """
            self.db_edit_goal([request.goal_desc, request.goal_id])
            """ inform user about action """
            self.msg.data = 'PlanCore: Edited goal %d to "%s"!' % (request.goal_id, request.goal_desc)
            self.publisher_.publish(self.msg)
            self.get_logger().info('Publishing: %s' % self.msg.data)
            """ send acknowledgement to indicate successful task """
            response.ack = True
        else:
            """ output an error message """
            self.get_logger().info("Error! cannot create the database connection.")
            """ create false acknowledgement to indicate failed task """
            response.ack = False
        return response

    def callback_delete_goal(self, request, response):
        """
        callback for deleting a goal from the goals table
        :param request: service request containing the goal id
        :param response: service response acknowledging the task
        :return: acknowledgement of the task
        """

        """ create a database connection """
        self.db_make_connection()
        """ insert new goal into table """
        if self.conn is not None:
            """ delete goal from table """
            self.db_delete_goal(request.goal_id)
            """ inform user about action """
            self.msg.data = 'PlanCore: Deleted goal %d!' % request.goal_id
            self.publisher_.publish(self.msg)
            self.get_logger().info('Publishing: %s' % self.msg.data)
            """ send acknowledgement to indicate successful task """
            response.ack = True
        else:
            """ output an error message """
            self.get_logger().info("Error! cannot create the database connection.")
            """ create false acknowledgement to indicate failed task """
            response.ack = False
        return response

    def callback_show_goal(self, request, response):
        """
        callback for deleting a goal from the goals table
        :param request: service request containing the goal id
        :param response: service response acknowledging the task
        :return: acknowledgement of the task
        """

        """ create a database connection """
        self.db_make_connection()
        """ insert new goal into table """
        if self.conn is not None:
            """ read the description of the specified goal from the table """
            goal_data = self.db_show_goal(request.goal_id)[0]
            goal_id = goal_data[0]
            goal_description = goal_data[1]
            """ inform user about action and send requested details """
            self.msg.data = 'PlanCore: Item %d reads "%s"!' % (goal_id, goal_description)
            self.publisher_.publish(self.msg)
            self.get_logger().info('Publishing: %s' % self.msg.data)
            """ send acknowledgement to indicate successful task """
            response.ack = True
        else:
            """ output an error message """
            self.get_logger().info("Error! cannot create the database connection.")
            """ create false acknowledgement to indicate failed task """
            response.ack = False
        return response

    def callback_create_plan(self, request, response):
        """
        callback for creating a plan in the goal_planning table
        :param request: service request containing the plan details,
                        including needed items and scheduled date
        :param response: service response acknowledging the task
        :return: acknowledgement of the task
        """

        """ create a database connection """
        self.db_make_connection()
        """ insert new plan into table """
        if self.conn is not None:
            """ reserve the needed items from the log core node """
            self.node_comm.reserve_item(request.items, request.begin_date, request.end_date)
            rclpy.spin_once(self.node_comm)
            reserve_result = self.node_comm.future.result()
            """ if the log core node can reserve the needed items, proceed """
            if reserve_result.reserved_ids is not None:
                """ save the plan in the db to execute """
                self.db_create_plan([request.goal_id, str(reserve_result.reserved_ids),
                                     request.begin_date, request.end_date])
                """ inform user about action """
                self.msg.data = 'PlanCore: Scheduled plan %d from %d to %d, using items %s!' \
                                % (request.goal_id, request.begin_date, request.end_date,
                                   str(reserve_result.reserved_ids))
                self.publisher_.publish(self.msg)
                self.get_logger().info('Publishing: %s' % self.msg.data)
                """ send acknowledgement to indicate successful task """
                response.ack = True
        else:
            """ output an error message """
            self.get_logger().info("Error! cannot create the database connection.")
            """ create false acknowledgement to indicate failed task """
            response.ack = False
        return response

    def callback_edit_plan(self, request, response):
        """
        callback for editing a plan in the goal_planning table
        :param request: service request containing the id and updated plan description
        :param response: service response acknowledging the task
        :return: acknowledgement of the task
        """

        """ create a database connection """
        self.db_make_connection()
        """ change details of plan in table """
        if self.conn is not None:
            self.db_edit_plan(request.plan_id, request.goal_id, request.items, request.begin_date, request.end_date)
            """ inform user about action """
            self.msg.data = 'PlanCore: Edited plan %d to goal %d from %d to %d using items %s!' \
                            % (request.plan_id, request.goal_id, request.begin_date, request.end_date, request.items)
            self.publisher_.publish(self.msg)
            self.get_logger().info('Publishing: %s' % self.msg.data)
            """ send acknowledgement to indicate successful task """
            response.ack = True
        else:
            """ output an error message """
            self.get_logger().info("Error! cannot create the database connection.")
            """ create false acknowledgement to indicate failed task """
            response.ack = False
        return response

    def callback_delete_plan(self, request, response):
        """
        callback for deleting a plan from the goal_planning table
        :param request: service request containing the plan id
        :param response: service response acknowledging the task
        :return: acknowledgement of the task
        """

        """ create a database connection """
        self.db_make_connection()
        """ delete plan from database """
        if self.conn is not None:
            self.db_delete_plan(request.plan_id)
            """ inform user about action """
            self.msg.data = 'PlanCore: Deleted plan %d!' % request.plan_id
            self.publisher_.publish(self.msg)
            self.get_logger().info('Publishing: %s' % self.msg.data)
            """ send acknowledgement to indicate successful task """
            response.ack = True
        else:
            """ output an error message """
            self.get_logger().info("Error! cannot create the database connection.")
            """ create false acknowledgement to indicate failed task """
            response.ack = False
        return response

    def callback_get_backlog(self, request, response):
        """
        callback for finding all already scheduled plans on the database
        :param request: service request
        :param response: service response acknowledging the task and sending the found database entries
        :return: response
        """

        """ create a database connection """
        self.db_make_connection()
        """ get plans from database """
        if self.conn is not None:
            backlog = self.db_get_backlog(request.begin_date)
            """ inform user about action """
            self.msg.data = 'PlanCore: Getting backlog of scheduled plans after %d!' % request.plan_id
            self.publisher_.publish(self.msg)
            self.get_logger().info('Publishing: %s' % self.msg.data)
            """ send acknowledgement to indicate successful task """
            response.ack = True
            response.plan_ids = [entries[0] for entries in backlog]
            response.begin_dates = [entries[1] for entries in backlog]
        else:
            """ output an error message """
            self.get_logger().info("Error! cannot create the database connection.")
            """ create false acknowledgement to indicate failed task """
            response.ack = False
        return response

    def db_make_connection(self):
        """
        create a database connection to the SQLite database specified by self.db_file
        """

        try:
            """ make connection to existing database or create new one, if none exists yet """
            self.conn = sqlite3.connect(self.db_file)
        except Error as e:
            """ output an error message """
            self.get_logger().info(e)

    def db_make_table(self, create_table_sql):
        """
        create a table from the create_table_sql statement
        :param create_table_sql: a CREATE TABLE statement
        """

        try:
            """ make cursor to interact with the table """
            c = self.conn.cursor()
            """ execute the sql statement """
            c.execute(create_table_sql)
        except Error as e:
            """ output an error message """
            self.get_logger().info(e)

    def db_create_goal(self, goal):
        """
        Create a new goal in the 'goals' table with a description
        :param goal: description of the kind of goal
        """

        """ build an INSERT INTO sql statement from the provided description """
        sql = ''' INSERT INTO goals(goal) VALUES(?) '''
        """ make cursor to interact with the table """
        c = self.conn.cursor()
        """ execute the sql statement with the given description """
        c.execute(sql, [goal])
        self.conn.commit()

    def db_edit_goal(self, goal):
        """
        Edit an existing goal in the 'goals' table
        :param goal: id and the new description of the goal
        """

        """ build an UPDATE sql statement from the provided parameters """
        sql = ''' UPDATE goals
                  SET goal = ?
                  WHERE id = ?'''
        """ make cursor to interact with the table """
        c = self.conn.cursor()
        """ execute the sql statement with the given id and new goal description """
        c.execute(sql, goal)
        self.conn.commit()

    def db_delete_goal(self, goal_id):
        """
        Delete a goal from the 'goals' table by id
        :param goal_id: id of the goal
        """

        """ build a DELETE FROM sql statement from the provided id """
        sql = 'DELETE FROM goals WHERE id=?'
        """ make cursor to interact with the table """
        c = self.conn.cursor()
        """ execute the sql statement with the given id """
        c.execute(sql, (goal_id,))
        self.conn.commit()

    def db_show_goal(self, goal_id):
        """
        Show goal with the provided id
        :param goal_id: id of the goal to be shown
        :return: goal data from table
        """

        """ build a SELECT FROM sql statement from the provided id """
        sql = "SELECT * FROM goals WHERE id=?"
        """ make cursor to interact with the table """
        c = self.conn.cursor()
        """ execute the sql statement with the given id """
        c.execute(sql, (goal_id,))
        """ return the goal description to the callback function """
        return c.fetchall()

    def db_create_plan(self, plan):
        """
        Create a new plan in the 'goal_planning' table with an associated goal, starting and end date
        :param plan: data of the plan as a tuple of goal_id, begin_date and end_date
        """

        """ build an INSERT INTO sql statement from the provided parameters """
        sql = ''' INSERT INTO goal_planning(goal_id, items, begin_date, end_date) VALUES(?,?,?,?) '''
        """ make cursor to interact with the table """
        c = self.conn.cursor()
        """ execute the sql statement with the given plan """
        c.execute(sql, plan)
        self.conn.commit()

    def db_edit_plan(self, plan_id, goal_id, items, begin_date, end_date):
        """
        Edit an existing plan in the 'goal_planning' table
        :param plan_id: id number of plan to be changed
        :param goal_id: new id of the associated goal to be implemented
        :param items: new list of items to be used
        :param begin_date: new starting date of the scheduled plan
        :param end_date: new end date of the scheduled plan
        """

        """ build an UPDATE sql statement from the provided parameters """
        sql = ''' UPDATE goal_planning
                  SET goal_id = ?,
                      items = ?,
                      begin_date = ?,
                      end_date = ?
                  WHERE id = ?'''
        """ make cursor to interact with the table """
        c = self.conn.cursor()
        """ execute the sql statement with the given parameters """
        c.execute(sql, (goal_id, items, begin_date, end_date, plan_id))
        self.conn.commit()

    def db_delete_plan(self, plan_id):
        """
        Delete a goal from the 'goals' table by id
        :param plan_id: id number of the plan to be deleted
        """

        """ build a DELETE FROM sql statement from the provided parameters """
        sql = 'DELETE FROM goal_planning WHERE id=?'
        """ make cursor to interact with the table """
        c = self.conn.cursor()
        """ execute the sql statement with the given id """
        c.execute(sql, (plan_id,))
        self.conn.commit()

    def db_get_backlog(self, time):
        """
        Get all plans that are already scheduled from the database
        :param time: current date, fetches all entries that begin after this
        :return: plan ids and begin dates from table
        """

        """ build a SELECT FROM sql statement from the provided id """
        sql = "SELECT plan_id, begin_date FROM goal_planning WHERE begin_date >= ?"
        """ make cursor to interact with the table """
        c = self.conn.cursor()
        """ execute the sql statement with the given time """
        c.execute(sql, (time,))
        """ return the goal description to the callback function """
        return c.fetchall()


class NodePlanCommunicator(Node):
    """
    Communicator node of the planning node cluster
    ---
    Is not intended to run continuously, instead just to send service requests to other node clusters.
    Can be called multiple times in parallel by the Core Plan Node to allow for parallel execution
    """

    def __init__(self):
        """
        Initialize the node and its attributes.
        ---
        :param self.cli: holds the client that calls services of other nodes
        :param self.req: holds the request for services of other nodes
        :param self.future: holds the response to service calls
        """
        super().__init__('node_plan_communicator')
        self.cli = None
        self.req = None
        self.future = None

    def reserve_item(self, item_kind, begin_date, end_date):
        """
        Calls the logistics node to reserve an item for a plan
        ---
        :param item_kind: the list of item kinds needed for the plan
        :param begin_date: starting date of the plan
        :param end_date: end date of the plan
        """

        """ create service client for Item interface under the name 'reserve_item' """
        self.cli = self.create_client(Item, 'reserve_item')
        """ wait for the server to be available """
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        """ define the request for the Item interface """
        self.req = Item.Request()
        """ set the parameters of the service interface """
        self.req.item_kind = item_kind
        self.req.begin_date = begin_date
        self.req.end_date = end_date
        """ call the service as defined above """
        self.future = self.cli.call_async(self.req)


def main():
    """
    entry point to the planning core node
    """

    """ initialise ros client library """
    rclpy.init()
    """ start the Log Core node """
    rclpy.spin(NodePlanCore())
    """ close the node once it terminates """
    rclpy.shutdown()


if __name__ == '__main__':
    """
    block 1 execution to start main() when invoked
    """

    main()
