from interfaces.srv import MakeDB
from interfaces.srv import Item

import rclpy
from rclpy.node import Node
import sqlite3
from sqlite3 import Error
from std_msgs.msg import String


class NodeLogCore(Node):
    """
    Core node of the logistics node cluster
    ---
    After start-up, it creates all services that can be called by clients,
    and announces its presence to the user info node.
    """

    def __init__(self):
        """
        Initializes the node and its attributes, services and publisher.
        Also sends a message to announce initialisation.
        ---
        :param self.conn: database connection
        :param self.db_file: name of the database to be used
        :param self.msg: string to be published to the user info to inform about actions
        :param self.srv_make_log_db: service interface to connect to a database or make a new one
        :param self.srv_create_item: service interface to create a new item in the database
        :param self.srv_edit_item: service interface to edit an existing item in the database
        :param self.srv_delete_item: service interface to delete an item in the database
        :param self.srv_show_item: service interface to show an item in the database
        :param self.srv_reserve_item: service interface to reserve an item for a plan
        :param self.publisher_: publisher for user info node (see self.msg)
        """

        super().__init__('node_log_core')
        self.conn = None
        self.db_file = r"sqlite/log.db"
        self.msg = String()

        self.srv_make_log_db = self.create_service(MakeDB, 'make_log_db', self.callback_make_log_db)
        self.srv_create_item = self.create_service(Item, 'create_item', self.callback_create_item)
        self.srv_edit_item = self.create_service(Item, 'edit_item', self.callback_edit_item)
        self.srv_delete_item = self.create_service(Item, 'delete_item', self.callback_delete_item)
        self.srv_show_item = self.create_service(Item, 'show_item', self.callback_show_item)
        self.srv_reserve_item = self.create_service(Item, 'reserve_item', self.callback_reserve_item)
        self.publisher_ = self.create_publisher(String, 'user_information', 10)

        """ sends a message to user info to make sure connections are working """
        self.msg.data = 'Good Morning from LogCore!'
        self.publisher_.publish(self.msg)
        self.get_logger().info('Publishing: "%s"' % self.msg.data)

    def callback_make_log_db(self, request, response):
        """
        callback for making the 'log' database and its tables 'items' and 'item_logistics'
        :param request: service request containing the database name
        :param response: service response acknowledging the task
        :return: acknowledgement of the task
        """

        """
        create table that holds the items in logistics:
        -- id: unique identifier of each individual item
        -- position: physical location of the item on a 2d grid
        -- item_kind: non-unique name of the item to search for it
        """
        sql_create_items_table = """ CREATE TABLE IF NOT EXISTS items (
                                     id integer PRIMARY KEY,
                                     position text NOT NULL,
                                     item_kind text NOT NULL
                                     ); """
        """
        create table that holds the item reservations:
        id: unique identifier of the item reservation
        item_id: number of the item reserved, non-unique in reservations
        begin_date: starting time of the reservation
        end_date: ending time of the reservation
        """
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
            """ create "items" table """
            self.db_make_table(sql_create_items_table)
            """ create "item_logistics" table """
            self.db_make_table(sql_create_item_logistics_table)
            """ inform user about action """
            self.msg.data = 'LogCore: made connection to database'
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

    def callback_create_item(self, request, response):
        """
        callback for creating an item in the items table
        :param request: service request containing the item description and position
        :param response: service response acknowledging the task
        :return: acknowledgement of the task
        """

        """ create a database connection """
        self.db_make_connection()
        """ insert new item into table """
        if self.conn is not None:
            """ create item in table """
            self.db_create_item(request.item_kind, request.position)
            """ inform user about action """
            self.msg.data = 'LogCore: Created item %s at %s!' % (request.item_kind, str(request.position))
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

    def callback_edit_item(self, request, response):
        """
        callback for editing an item in the items table
        :param request: service request containing the id and item description
        :param response: service response acknowledging the task
        :return: acknowledgement of the task
        """

        """ create a database connection """
        self.db_make_connection()
        """ insert new item description into table """
        if self.conn is not None:
            """ edit item in the table """
            self.db_edit_item(request.item_kind, request.id)
            """ inform user about action """
            self.msg.data = 'LogCore: Edited item %d to %s!' % (request.id, request.item_kind)
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

    def callback_delete_item(self, request, response):
        """
        callback for deleting an item from the items table
        :param request: service request containing the item id
        :param response: service response acknowledging the task
        :return: acknowledgement of the task
        """

        """ create a database connection """
        self.db_make_connection()
        """ insert new item into table """
        if self.conn is not None:
            """ delete item from table """
            self.db_delete_item(request.id)
            """ inform user about action """
            self.msg.data = 'LogCore: Deleted item %d!' % request.id
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

    def callback_show_item(self, request, response):
        """
        callback for deleting an item from the items table
        :param request: service request containing the item id
        :param response: service response acknowledging the task
        :return: acknowledgement of the task
        """

        """ create a database connection """
        self.db_make_connection()
        """ insert new item into table """
        if self.conn is not None:
            """ read the position and description data of the specified item from the table """
            item_data = self.db_show_item(request.id)[0]
            item_position = item_data[1]
            item_description = item_data[2]
            """ inform user about action and send requested details """
            self.msg.data = 'LogCore: Item reads "%s" and lies at position %s!' % (item_description, item_position)
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

    def callback_reserve_item(self, request, response):
        """
        callback for reserving an item in the item_logistics table
        :param request: service request containing the item kinds needed, the beginning and end date of the reservation
        :param response: service response acknowledging the task
        :return: acknowledgement of the task
        """

        """ create a database connection """
        self.db_make_connection()
        if self.conn is not None:
            """ reserve the requested items, if able """
            item_ids = self.db_reserve_item(request.item_kind, request.begin_date, request.end_date)
            """ transform tuple into a list type for sending in the response """
            item_ids = list(item_ids)
            response.reserved_ids = item_ids
            """ send acknowledgement to indicate successful task """
            response.ack = True
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
        create a table from the provided create_table_sql statement
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

    def db_create_item(self, item_kind, position):
        """
        Create a new item into the 'items' table with a description
        :param item_kind: description of the kind of item
        :param position: location of the item
        """

        """ build an INSERT INTO sql statement from the provided parameters """
        sql = ''' INSERT INTO items(item_kind, position) VALUES(?,?) '''
        try:
            """ make cursor to interact with the table """
            c = self.conn.cursor()
            """ execute the sql statement with the given item kind and position """
            c.execute(sql, [item_kind, position])
            self.conn.commit()
        except Error as e:
            """ output an error message """
            self.get_logger().info(e)

    def db_edit_item(self, item_kind, id):
        """
        Edit an existing item in the 'items' table
        :param id: identifier number of the item
        :param item_kind: new description of the item
        """

        """ build an UPDATE sql statement from the provided parameters """
        sql = ''' UPDATE items SET item_kind = ? WHERE id = ?'''
        """ make cursor to interact with the table """
        c = self.conn.cursor()
        """ execute the sql statement with the given id and new item kind """
        c.execute(sql, (item_kind, id))
        self.conn.commit()

    def db_delete_item(self, item_id):
        """
        Delete an item from the 'items' table by id
        :param item_id: id of the item
        """

        """ build a DELETE FROM sql statement from the provided id """
        sql = 'DELETE FROM items WHERE id=?'
        """ make cursor to interact with the table """
        c = self.conn.cursor()
        """ execute the sql statement with the given item id """
        c.execute(sql, (item_id,))
        self.conn.commit()

    def db_show_item(self, item_id):
        """
        Show item with the provided id
        :param item_id: id of the item to be shown
        :return: item data from table
        """

        """ build a SELECT FROM sql statement from the provided id """
        sql = "SELECT * FROM items WHERE id=?"
        """ make cursor to interact with the table """
        c = self.conn.cursor()
        """ execute the sql statement with the given item id """
        c.execute(sql, (item_id,))
        """ return the item data to the callback function """
        return c.fetchall()

    def db_reserve_item(self, item_kind, begin_date, end_date):
        """
        Checks if items of a certain kind can be reserved, and do so if able
        :param item_kind: list of kinds of item to be reserved
        :param begin_date: when the item needs to be reserved
        :param end_date: when the item is free again
        :return: id of reserved item, or None of no item was found
        """

        """ converts the items from a string into a processable list """
        item_kind_arr = item_kind.split()
        """ create a list to be filled with items that can be reserved """
        reserved_inventory = []
        """ for every item in the given "item_kind" array, do: """
        for item in item_kind_arr:
            """ build a SELECT FROM sql statement from the provided parameters """
            sql = "SELECT id FROM items WHERE item_kind=?"
            """ make cursor to interact with the table """
            c = self.conn.cursor()
            """ execute the sql statement with the given item """
            c.execute(sql, (item,))
            """ safe the items found in a list """
            possible_items = c.fetchall()
            possible_items = list(possible_items)
            """
            for every found item that fits the looked for description,
            find those that can actually be reserved
            """
            for possible_item in possible_items:
                """ build a SELECT FROM sql statement from the provided parameters """
                sql = "SELECT id FROM item_logistics WHERE " \
                      "item_id= ? AND " \
                      "((begin_date > ? AND begin_date > ?) OR" \
                      "(end_date < ? AND end_date < ?))"
                """ make cursor to interact with the table """
                c = self.conn.cursor()
                """ execute the sql statement with the given item, the starting and end time """
                c.execute(sql, (possible_item[0], begin_date, end_date, begin_date, end_date))
                """ check if at least one item could be reserved """
                row = c.fetchone()
                if row is None:
                    """ add the reservable item to the shopping list """
                    reserved_inventory.append(possible_item[0])
        """ delete duplicate items from the list of reservable items """
        reserved_inventory = set(reserved_inventory)
        """ if enough items of the requested kinds are available, reserve them """
        if len(reserved_inventory) == len(item_kind_arr):
            """ for every item to be reserved, write an entry into the logistics table """
            for item in reserved_inventory:
                """ build an INSERT INTO sql statement from the provided parameters """
                sql = "INSERT INTO item_logistics " \
                      "(item_id, begin_date, end_date) VALUES(:id, :begin_date, :end_date)"
                """ create a dictionary of the values to be added """
                dict = {'id': item, 'begin_date': begin_date, 'end_date': end_date}
                """ make cursor to interact with the table """
                c = self.conn.cursor()
                """ execute the sql statement with the dictionary """
                c.execute(sql, dict)
                self.conn.commit()
            """ return the items that have been reserved """
            return reserved_inventory
        """ return none if not all items needed could be reserved """
        return None


def main():
    """
    entry point to the logistics core node
    """

    """ initialise ros client library """
    rclpy.init()
    """ start the Log Core node """
    rclpy.spin(NodeLogCore())
    """ close the node once it terminates """
    rclpy.shutdown()


if __name__ == '__main__':
    """
    block 1 execution to start main() when invoked
    """

    main()
