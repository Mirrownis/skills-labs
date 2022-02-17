from interfaces.srv import MakeDB
from interfaces.srv import CreateGoal
from interfaces.srv import DeleteGoal

import rclpy
from rclpy.node import Node
import sqlite3
from sqlite3 import Error


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
        self.srv_make_plan_db = self.create_service(MakeDB, 'make_plan_db', self.callback_make_plan_db)
        self.srv_create_goal = self.create_service(CreateGoal, 'create_goal', self.callback_create_goal)
        self.srv_delete_goal = self.create_service(DeleteGoal, 'delete_goal', self.callback_delete_goal)

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
                                              begin_date text NOT NULL,
                                              end_date text NOT NULL,
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
            print(request.goal_desc)
            self.db_create_goal(request.goal_desc)
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
            print(request.goal_id)
            self.db_delete_goal(request.goal_id)
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
        print("Created goal " + goal)

    def db_delete_goal(self, goal_id):
        """
        Delete a goal from the 'goals' table by id
        :param goal_id: id of the goal
        """

        sql = 'DELETE FROM goals WHERE id=?'
        c = self.conn.cursor()
        c.execute(sql, (goal_id,))
        self.conn.commit()
        print("Deleted goal " + goal_id)


def main():
    rclpy.init()
    plan_core = NodePlanCore()
    rclpy.spin(plan_core)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
