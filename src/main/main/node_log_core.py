from interfaces.srv import MakeDB
from interfaces.srv import Item

import rclpy
from rclpy.node import Node
import sqlite3
from sqlite3 import Error
from std_msgs.msg import String


class NodeLogCore(Node):

    def __init__(self):
        """
        initialize the node, its attributes and create services to be called

        conn: database connection
        db_file: name of the database to be used
        srv_make_log_db: service interface to connect to a database or make a new one
        srv_create_item: service interface to create a new item in the database
        srv_delete_item: service interface to delete an item in the database
        """

        super().__init__('node_log_core')
        self.conn = None
        self.db_file = None
        self.msg = String()

        self.srv_make_log_db = self.create_service(MakeDB, 'make_log_db', self.callback_make_log_db)
        self.srv_create_item = self.create_service(Item, 'create_item', self.callback_create_item)
        self.srv_edit_item = self.create_service(Item, 'edit_item', self.callback_edit_item)
        self.srv_delete_item = self.create_service(Item, 'delete_item', self.callback_delete_item)
        self.srv_show_item = self.create_service(Item, 'show_item', self.callback_show_item)
        self.publisher_ = self.create_publisher(String, 'user_information', 10)

        self.msg.data = 'Good Morning from LogCore!'
        self.publisher_.publish(self.msg)
        self.get_logger().info('Publishing: "%s"' % self.msg.data)

    def callback_make_log_db(self, request, response):
        """
        callback for making the 'log' database and its tables 'items' and 'item_logistics'
        :param request: service request containing the database name
        :param response: service response acknowledging the task
        :return: updated response
        """

        """ create a database connection to a SQLite database """
        self.db_file = r"sqlite/%s.db" % request.db_name
        """ create two tables that hold the items in logistics and their availability """
        sql_create_items_table = """ CREATE TABLE IF NOT EXISTS items (
                                     id integer PRIMARY KEY,
                                     position text NOT NULL,
                                     item_kind text NOT NULL
                                     ); """
        sql_create_item_logistics_table = """ CREATE TABLE IF NOT EXISTS item_logistics (
                                              id integer PRIMARY KEY,
                                              item_id integer NOT NULL,
                                              begin_date text NOT NULL,
                                              end_date text NOT NULL,
                                              FOREIGN KEY (item_id) REFERENCES items (id)
                                              ); """
        """ create a database connection """
        self.db_make_connection()
        """ create tables """
        if self.conn is not None:
            """ create items table """
            self.db_make_table(sql_create_items_table)
            """ create item_logistics table """
            self.db_make_table(sql_create_item_logistics_table)
            self.msg.data = 'LogCore: made connection to database'
            self.publisher_.publish(self.msg)
            self.get_logger().info('Publishing: %s' % self.msg.data)
            """ create response """
            response.ack = True
        else:
            print("Error! cannot create the database connection.")
            response.ack = False
        return response

    def callback_create_item(self, request, response):
        """
        callback for creating an item in the items table
        :param request: service request containing the item description
        :param response: service response acknowledging the task
        :return: updated response
        """

        """ create a database connection """
        self.db_make_connection()
        """ insert new item into table """
        if self.conn is not None:
            print(request.item_desc)
            print(request.position)
            self.db_create_item(request.item_desc, request.position)
            self.msg.data = 'LogCore: Created item %s at %s!' % (request.item_desc, str(request.position))
            self.publisher_.publish(self.msg)
            self.get_logger().info('Publishing: %s' % self.msg.data)
            response.ack = True
        else:
            print("Error! cannot create the database connection.")
            response.ack = False
        return response

    def callback_edit_item(self, request, response):
        """
        callback for editing an item in the items table
        :param request: service request containing the id and item description
        :param response: service response acknowledging the task
        :return: updated response
        """

        """ create a database connection """
        self.db_make_connection()
        """ insert new item description into table """
        if self.conn is not None:
            self.db_edit_item(request.item_desc, request.id)
            self.msg.data = 'LogCore: Edited item %d to %s!' % (request.id, request.item_desc)
            self.publisher_.publish(self.msg)
            self.get_logger().info('Publishing: %s' % self.msg.data)
            response.ack = True
        else:
            print("Error! cannot create the database connection.")
            response.ack = False
        return response

    def callback_delete_item(self, request, response):
        """
        callback for deleting an item from the items table
        :param request: service request containing the item id
        :param response: service response acknowledging the task
        :return: updated response
        """

        """ create a database connection """
        self.db_make_connection()
        """ insert new item into table """
        if self.conn is not None:
            print(request.id)
            self.db_delete_item(request.id)
            self.msg.data = 'LogCore: Deleted item %d!' % request.id
            self.publisher_.publish(self.msg)
            self.get_logger().info('Publishing: %s' % self.msg.data)
            response.ack = True
        else:
            print("Error! cannot create the database connection.")
            response.ack = False
        return response

    def callback_show_item(self, request, response):
        """
        callback for deleting an item from the items table
        :param request: service request containing the item id
        :param response: service response acknowledging the task
        :return: updated response
        """

        """ create a database connection """
        self.db_make_connection()
        """ insert new item into table """
        if self.conn is not None:
            item_data = self.db_show_item(request.id)[0]
            item_position = item_data[1]
            item_description = item_data[2]
            self.msg.data = 'LogCore: Item reads "%s" and lies at position %s!' % (item_description, item_position)
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

    def db_create_item(self, item_desc, position):
        """
        Create a new item into the 'items' table with a description
        :param item_desc: description of the kind of item
        :param position: location of the item
        """

        sql = ''' INSERT INTO items(item_kind, position) VALUES(?,?) '''
        c = self.conn.cursor()
        c.execute(sql, [item_desc, position])
        self.conn.commit()

    def db_edit_item(self, item_desc, id):
        """
        Edit an existing item in the 'items' table
        :param item: id and the new description of the item
        """

        sql = ''' UPDATE items
                  SET item_kind = ?
                  WHERE id = ?'''
        c = self.conn.cursor()
        c.execute(sql, (item_desc, id))
        self.conn.commit()

    def db_delete_item(self, item_id):
        """
        Delete an item from the 'items' table by id
        :param item_id: id of the item
        """

        sql = 'DELETE FROM items WHERE id=?'
        c = self.conn.cursor()
        c.execute(sql, (item_id,))
        self.conn.commit()

    def db_show_item(self, item_id):
        """
        Query tasks by priority
        :param item_id: id of the item to be selected
        :return:
        """

        sql = "SELECT * FROM items WHERE id=?"
        c = self.conn.cursor()
        c.execute(sql, (item_id,))

        return c.fetchall()


def main():
    rclpy.init()
    log_core = NodeLogCore()
    rclpy.spin(log_core)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
