from interfaces.srv import MakeDB
from interfaces.srv import CreateItem
from interfaces.srv import DeleteItem

import rclpy
from rclpy.node import Node
import sqlite3
from sqlite3 import Error


class NodeLogCore(Node):

    def __init__(self):
        """
        initialize the node, its attributes and create services to be called

        conn: database connection
        db_file: name of the database to be used
        srv_make_db: service interface to connect to a database or make a new one
        srv_create_item: service interface to create a new item in the database
        srv_delete_item: service interface to delete an item in the database
        """

        super().__init__('node_log_core')
        self.conn = None
        self.db_file = None
        self.srv_make_db = self.create_service(MakeDB, 'make_db', self.callback_make_db)
        self.srv_create_item = self.create_service(CreateItem, 'create_item', self.callback_create_item)
        self.srv_delete_item = self.create_service(DeleteItem, 'delete_item', self.callback_delete_item)

    def callback_make_db(self, request, response):
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
                                     item_kind text NOT NULL
                                     ); """
        sql_create_item_logistics_table = """ CREATE TABLE IF NOT EXISTS item_logistics (
                                              id integer PRIMARY KEY,
                                              item_id integer NOT NULL,
                                              position text NOT NULL,
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
            self.db_create_item(request.item_desc)
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
            print(request.item_id)
            self.db_delete_item(request.item_id)
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

    def db_create_item(self, item):
        """
        Create a new item into the 'items' table with a description
        :param item: description of the kind of item
        """

        sql = ''' INSERT INTO items(item_kind) VALUES(?) '''
        c = self.conn.cursor()
        c.execute(sql, [item])
        self.conn.commit()
        print("Created item " + item)

    def db_delete_item(self, item_id):
        """
        Delete an item from the 'items' table by id
        :param item_id: id of the item
        """

        sql = 'DELETE FROM items WHERE id=?'
        c = self.conn.cursor()
        c.execute(sql, (item_id,))
        self.conn.commit()
        print("Deleted item " + item_id)


def main():
    rclpy.init()
    log_core = NodeLogCore()
    rclpy.spin(log_core)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
