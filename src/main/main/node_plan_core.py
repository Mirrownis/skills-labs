from interfaces.srv import MakeDB
from interfaces.srv import Goal
from interfaces.srv import CreatePlan
from interfaces.srv import EditPlan
from interfaces.srv import DeletePlan

import rclpy
from rclpy.node import Node
import sqlite3
from sqlite3 import Error
from std_msgs.msg import String


class NodePlanCore(Node):

    def __init__(self):
        """
        initialize the node, its attributes and create services to be called

        conn: database connection
        db_file: name of the database to be used
        srv_make_plan_db: service interface to connect to a planning database or make a new one
        srv_create_goal: service interface to create a new goal in the database
        srv_delete_goal: service interface to delete a goal in the database
        """

        super().__init__('node_plan_core')
        self.conn = None
        self.db_file = None
        self.msg = String()
        self.srv_make_plan_db = self.create_service(MakeDB, 'make_plan_db', self.callback_make_plan_db)
        self.srv_create_goal = self.create_service(Goal, 'create_goal', self.callback_create_goal)
        self.srv_edit_goal = self.create_service(Goal, 'edit_goal', self.callback_edit_goal)
        self.srv_show_goal = self.create_service(Goal, 'show_goal', self.callback_show_goal)
        self.srv_delete_goal = self.create_service(Goal, 'delete_goal', self.callback_delete_goal)
        self.srv_create_plan = self.create_service(CreatePlan, 'create_plan', self.callback_create_plan)
        self.srv_edit_plan = self.create_service(EditPlan, 'edit_plan', self.callback_edit_plan)
        self.srv_delete_plan = self.create_service(DeletePlan, 'delete_plan', self.callback_delete_plan)
        self.publisher_ = self.create_publisher(String, 'user_information', 10)
        self.msg.data = 'Good Morning from PlanCore!'
        self.publisher_.publish(self.msg)
        self.get_logger().info('Publishing: %s' % self.msg.data)

    def callback_make_plan_db(self, request, response):
        """
        callback for making the 'plan' database and its tables 'goals' and 'goal_planning'
        :param request: service request containing the database name
        :param response: service response acknowledging the task
        :return: updated response
        """

        """ create a database connection to a SQLite database """
        self.db_file = r"sqlite/%s.db" % request.db_name
        """ create two tables that hold the goals in planning and their availability """
        sql_create_goals_table = """ CREATE TABLE IF NOT EXISTS goals (
                                     id integer PRIMARY KEY,
                                     goal text NOT NULL
                                     ); """
        sql_create_goal_planning_table = """ CREATE TABLE IF NOT EXISTS goal_planning (
                                              id integer PRIMARY KEY,
                                              goal_id integer NOT NULL,
                                              items text NOT NULL,
                                              begin_date integer NOT NULL,
                                              end_date integer NOT NULL,
                                              FOREIGN KEY (goal_id) REFERENCES goals (id)
                                              ); """
        """ create a database connection """
        self.db_make_connection()
        """ create tables """
        if self.conn is not None:
            """ create goals table """
            self.db_make_table(sql_create_goals_table)
            """ create goal_planning table """
            self.db_make_table(sql_create_goal_planning_table)
            self.msg.data = 'PlanCore: made connection to database'
            self.publisher_.publish(self.msg)
            self.get_logger().info('Publishing: %s' % self.msg.data)
            """ create response """
            response.ack = True
        else:
            print("Error! cannot create the database connection.")
            response.ack = False
        return response

    def callback_create_goal(self, request, response):
        """
        callback for creating a goal in the goals table
        :param request: service request containing the goal description
        :param response: service response acknowledging the task
        :return: updated response
        """

        """ create a database connection """
        self.db_make_connection()
        """ insert new goal into table """
        if self.conn is not None:
            self.db_create_goal(request.goal_desc)
            self.msg.data = 'PlanCore: Created goal %s!' % request.goal_desc
            self.publisher_.publish(self.msg)
            self.get_logger().info('Publishing: %s' % self.msg.data)
            response.ack = True
        else:
            print("Error! cannot create the database connection.")
            response.ack = False
        return response

    def callback_edit_goal(self, request, response):
        """
        callback for editing a goal in the goals table
        :param request: service request containing the id and goal description
        :param response: service response acknowledging the task
        :return: updated response
        """

        """ create a database connection """
        self.db_make_connection()
        """ insert new goal description into table """
        if self.conn is not None:
            self.db_edit_goal([request.goal_desc, request.goal_id])
            self.msg.data = 'PlanCore: Edited goal %d to %s!' % (request.goal_id, request.goal_desc)
            self.publisher_.publish(self.msg)
            self.get_logger().info('Publishing: %s' % self.msg.data)
            response.ack = True
        else:
            print("Error! cannot create the database connection.")
            response.ack = False
        return response

    def callback_delete_goal(self, request, response):
        """
        callback for deleting a goal from the goals table
        :param request: service request containing the goal id
        :param response: service response acknowledging the task
        :return: updated response
        """

        """ create a database connection """
        self.db_make_connection()
        """ insert new goal into table """
        if self.conn is not None:
            self.db_delete_goal(request.goal_id)
            self.msg.data = 'PlanCore: Deleted goal %d!' % request.goal_id
            self.publisher_.publish(self.msg)
            self.get_logger().info('Publishing: %s' % self.msg.data)
            response.ack = True
        else:
            print("Error! cannot create the database connection.")
            response.ack = False
        return response

    def callback_show_goal(self, request, response):
        """
        callback for deleting a goal from the goals table
        :param request: service request containing the goal id
        :param response: service response acknowledging the task
        :return: updated response
        """

        """ create a database connection """
        self.db_make_connection()
        """ insert new goal into table """
        if self.conn is not None:
            goal_data = self.db_show_goal(request.goal_id)[0]
            goal_id = goal_data[0]
            goal_description = goal_data[1]
            self.msg.data = 'PlanCore: Item %d reads "%s"!' % (goal_id, goal_description)
            self.publisher_.publish(self.msg)
            self.get_logger().info('Publishing: %s' % self.msg.data)
            response.ack = True
        else:
            print("Error! cannot create the database connection.")
            response.ack = False
        return response

    def callback_create_plan(self, request, response):
        """
        callback for creating a plan in the goal_planning table
        :param request: service request containing the plan description
        :param response: service response acknowledging the task
        :return: updated response
        """

        """ create a database connection """
        self.db_make_connection()
        """ insert new plan into table """
        if self.conn is not None:
            self.db_create_plan([request.goal_id, request.items, request.begin_time, request.end_time])
            self.msg.data = 'PlanCore: Scheduled plan %d from %d to %d, using items %s!'\
                            % (request.goal_id, request.begin_time, request.end_time, request.items)
            self.publisher_.publish(self.msg)
            self.get_logger().info('Publishing: %s' % self.msg.data)
            response.ack = True
        else:
            print("Error! cannot create the database connection.")
            response.ack = False
        return response

    def callback_edit_plan(self, request, response):
        """
        callback for editing a plan in the goal_planning table
        :param request: service request containing the id and updated plan description
        :param response: service response acknowledging the task
        :return: updated response
        """

        """ create a database connection """
        self.db_make_connection()
        """ insert new plan into table """
        if self.conn is not None:
            self.db_edit_plan(request.id, request.goal_id, request.items, request.begin_time, request.end_time)
            self.msg.data = 'PlanCore: Edited plan %d to goal %d from %d to %d using items %s!'\
                            % (request.id, request.goal_id, request.begin_time, request.end_time, request.items)
            self.publisher_.publish(self.msg)
            self.get_logger().info('Publishing: %s' % self.msg.data)
            response.ack = True
        else:
            print("Error! cannot create the database connection.")
            response.ack = False
        return response

    def callback_delete_plan(self, request, response):
        """
        callback for deleting a plan from the goal_planning table
        :param request: service request containing the plan id
        :param response: service response acknowledging the task
        :return: updated response
        """

        """ create a database connection """
        self.db_make_connection()
        """ insert new goal into table """
        if self.conn is not None:
            self.db_delete_plan(request.plan_id)
            self.msg.data = 'PlanCore: Deleted plan %d!' % request.plan_id
            self.publisher_.publish(self.msg)
            self.get_logger().info('Publishing: %s' % self.msg.data)
            response.ack = True
        else:
            print("Error! cannot create the database connection.")
            response.ack = False
        return response

    def db_make_connection(self):
        """
        create a database connection to the SQLite database
        specified by db_file
        """

        try:
            self.conn = sqlite3.connect(self.db_file)
        except Error as e:
            print(e)

    def db_make_table(self, create_table_sql):
        """
        create a table from the create_table_sql statement
        :param create_table_sql: a CREATE TABLE statement
        """

        try:
            c = self.conn.cursor()
            c.execute(create_table_sql)
        except Error as e:
            print(e)

    def db_create_goal(self, goal):
        """
        Create a new goal into the 'goals' table with a description
        :param goal: description of the kind of goal
        """

        sql = ''' INSERT INTO goals(goal) VALUES(?) '''
        c = self.conn.cursor()
        c.execute(sql, [goal])
        self.conn.commit()

    def db_edit_goal(self, goal):
        """
        Edit an existing goal in the 'goals' table
        :param goal: id and the new description of the goal
        """

        sql = ''' UPDATE goals
                  SET goal = ?
                  WHERE id = ?'''
        c = self.conn.cursor()
        c.execute(sql, goal)
        self.conn.commit()

    def db_delete_goal(self, goal_id):
        """
        Delete a goal from the 'goals' table by id
        :param goal_id: id of the goal
        """

        sql = 'DELETE FROM goals WHERE id=?'
        c = self.conn.cursor()
        c.execute(sql, (goal_id,))
        self.conn.commit()

    def db_show_goal(self, goal_id):
        """
        Query tasks by priority
        :param goal_id: id of the goal to be selected
        :return:
        """

        sql = "SELECT * FROM goals WHERE id=?"
        c = self.conn.cursor()
        c.execute(sql, (goal_id,))

        return c.fetchall()

    def db_create_plan(self, plan):
        """
        Create a new plan in the 'goal_planning' table with an associated goal, starting and end time
        :param plan: description of the kind of plan
        """

        sql = ''' INSERT INTO goal_planning(goal_id, items, begin_date, end_date) VALUES(?,?,?,?) '''
        c = self.conn.cursor()
        c.execute(sql, plan)
        self.conn.commit()

    def db_edit_plan(self, plan_id, goal_id, items, begin_time, end_time):
        """
        Edit an existing plan in the 'goal_planning' table
        :param plan_id:
        :param end_time:
        :param begin_time:
        :param items:
        :param goal_id:
        """

        plan = (plan_id, goal_id, items, begin_time, end_time)
        sql = ''' UPDATE goal_planning
                  SET goal_id = ?,
                      items = ?,
                      begin_date = ?,
                      end_date = ?
                  WHERE id = ?'''
        c = self.conn.cursor()
        c.execute(sql, plan)
        self.conn.commit()

    def db_delete_plan(self, plan_id):
        """
        Delete a goal from the 'goals' table by id
        :param plan_id: id of the plan
        """

        sql = 'DELETE FROM goal_planning WHERE id=?'
        c = self.conn.cursor()
        c.execute(sql, (plan_id,))
        self.conn.commit()


def main():
    rclpy.init()
    plan_core = NodePlanCore()
    rclpy.spin(plan_core)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
