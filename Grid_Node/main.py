#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()


# Write your program here.
ev3.speaker.beep()


import copy
# import numpy as np


class Node:
    """
    This class represents a Node
    """

    def __init__(self, idx: str):
        """
        init a Node object

        :param idx: the id of the node
        """

        self.__idx = idx
        self.__x1 = -1 # np.nan  # x1 and x2: refer to the location of the node in the graph
        self.__x2 = - 1 # np.nan  # x1 and x2: refer to the location of the node in the graph
        self.__graph = None  # the graph to which the node belongs
        self.__neighbors = dict()  # list of node's neighbors
        self.__distances = dict()  # list of distances to node's neighbors
        self.__init_neighbors()

    """
    Getters and Setters of node's attributes
    """
    @property
    def idx(self):
        return self.__idx

    @property
    def location(self):
        return self.__x1, self.__x2

    @location.setter
    def location(self, loc: tuple):
        self.__x1 = loc[0]
        self.__x2 = loc[1]

    @property
    def neighbors(self):
        return copy.deepcopy(self.__neighbors)

    @property
    def distances(self):
        return copy.deepcopy(self.__distances)

    @property
    def graph(self):
        return self.__graph

    @graph.setter
    def graph(self, graph):
        if isinstance(graph, Grid):
            if self.__graph is None:
                self.__graph = graph
            else:
                raise RuntimeError("Node is already in a graph, must be removed first")
        else:
            raise TypeError('Parameter graph must be of <class "Grid">')

    """
    Methods: add or remove a neighbor
    """

    def add_neighbor(self, node, distance: float, direction: str):
        """
        add a node the to neighbors list

        :param node: the node to add to the neighbors list
        :param distance: the distance to the node
        :param direction: the direction to the node
        :return: True if the node was added to the neighbors list; otherwise it raise an exception
        """

        if node.graph == self.graph:
            if node not in self.__neighbors.values() and not self.has_neighbor(direction):
                self.__neighbors[direction] = node
                self.__distances[direction] = distance
                return True
            else:
                raise ValueError("Cannot connect to node=", node, " as node may be already in neighbors "
                                 "or direction=", direction, " may be already full")
        raise ValueError("Cannot connect to node=", node, " as not in the same graph")

    def remove_neighbor(self, value):
        """
        remove a node from the neighbors list

        :param value: the idx of the node or the node to remove
        :return: True if the node was removed or False if not or it may raise an exception
        """

        if isinstance(value, str):
            if value in self.__neighbors.keys():
                if self.has_neighbor(value):
                    return self.__neighbors[value], self.__remove_from_neighbors(value)
                return None
            raise KeyError("Unknown key=",value, " for parameter 'direction'")
        elif isinstance(value, Node):
            if self.is_neighbor(value):
                direction = self.direction_to_neighbor(value)
                return direction, self.__remove_from_neighbors(direction)
        return False

    def __remove_from_neighbors(self, direction: str):
        """
        remove a node from the neighbors list

        :param direction: the direction where the node to remove is located
        """

        self.__neighbors[direction] = None
        self.__distances[direction] = -1 # np.nan
        return self.__distances[direction]

    """
    Methods: remove from graph and check if in graph
    """

    def disconnect(self):
        """
        A method used to remove a node from the graph. All neighbors must be removed before calling this method;
        otherwise an exception is raised.
        """

        if len(list(filter(lambda n: isinstance(n, Node), self.__neighbors.values()))) == 0:
            self.__graph = None
            self.__init_neighbors()
        else:
            raise RuntimeError("Node cannot be disconnected as it is still connected to other nodes in the graph")

    def is_in_graph(self, graph):
        """
        Check if the node belongs to the graph

        :param graph: the graph
        :return: True if the node belongs to this graph; otherwise, False.
        """

        return self.__graph == graph

    def has_graph(self):
        """
        Check if the node belongs to a graph

        :return: True if the node is in a graph; otherwise, False.
        """

        return self.__graph is not None

    """"
    Methods: general methods to check and search a neighbor node using direction or a node object
    """
    def neighbor(self, direction):
        if direction in self.__neighbors.keys():
            return self.__neighbors[direction]
        raise KeyError("Unknown key=", direction, " for parameter 'direction'")

    def is_neighbor(self, node):
        return node in self.__neighbors.values()

    def has_neighbor(self, direction):
        return self.neighbor(direction) is not None

    """"
    Methods: general methods to get distance or direction to neighbor
    """
    def distance_to_neighbor(self, node):
        if self.is_neighbor(node):
            key = [k for k, n in self.__neighbors.items() if n == node][0]
            return self.__distances[key]
        return-1 #  np.nan

    def direction_to_neighbor(self, node):
        if self.is_neighbor(node):
            return [k for k, n in self.__neighbors.items() if n == node][0]
        return None

    def distance_to(self, direction: str):
        return self.__distances[direction]

    """"
    Methods: to get or check the left neighbor and get the distance if it exists
    """
    @property
    def neighbor_on_left(self):
        return self.__neighbors['left']

    @property
    def distance_to_left(self):
        return self.__distances['left']

    def is_on_left(self, node):
        return self.__neighbors['left'] == node

    def is_neighbor_on_left(self):
        return self.__neighbors['left'] is not None

    """"
    Methods: to get or check the right neighbor and get the distance if it exists
    """
    @property
    def neighbor_on_right(self):
        return self.__neighbors['right']

    @property
    def distance_to_right(self):
        return self.__distances['right']

    def is_on_right(self, node):
        return self.__neighbors['right'] == node

    def is_neighbor_on_right(self):
        return self.__neighbors['right'] is not None

    """"
    Methods: to get or check the above neighbor and get the distance if it exists
    """
    @property
    def neighbor_above(self):
        return self.__neighbors['up']

    @property
    def distance_to_up(self):
        return self.__distances['up']

    def is_above(self, node):
        return self.__neighbors['up'] == node

    def is_neighbor_above(self):
        return self.__neighbors['up'] is not None

    """"
    Methods: to get or check the below neighbor and get the distance if it exists
    """
    @property
    def neighbor_below(self):
        return self.__neighbors['down']

    @property
    def distance_to_down(self):
        return self.__distances['down']

    def is_below(self, node):
        return self.__neighbors['down'] == node

    def is_neighbor_below(self):
        return self.__neighbors['down'] is not None

    def surroundings(self):
        """
        Combine distances list and neighbors list into a dictionary with the direction as key

        :return: a dictionary of the surrounding nodes with distance
        """
        # return {k: (n.idx if isinstance(n, Node) else n, self.__distances[k]) for k, n in self.__neighbors.items()}
        return dict(zip(self.__neighbors.keys(), zip(self.__neighbors.values(), self.__distances.values())))

    def __str__(self):
        return "Node [idx=", self.idx, ", location=", self.location, "]"

    def __repr__(self):
        return "Node [idx=", self.idx, ", location=", self.location, "]"

    def __init_neighbors(self):
        """
        Private method used to init distances list and neighbors list
        """
        self.__neighbors['left'], self.__distances['left'] = None, -1 # np.nan
        self.__neighbors['right'], self.__distances['right'] = None, -1 # np.nan
        self.__neighbors['up'], self.__distances['up'] = None, -1 # np.nan
        self.__neighbors['down'], self.__distances['down'] = None, -1 # np.nan


class Grid:
    """
    This class represents a Grid of Nodes
    """

    def __init__(self, row, col):
        """
        init a Grid object

        :param row: number of rows
        :param col: number of columns
        """

        self.__col = col
        self.__row = row
        self.__nodes = dict()  # dict() object for nodes

    @property
    def shape(self):
        """
        Return the shape of the grid

        :return: shape of grid
        """

        return self.__col, self.__row

    @property
    def size(self):
        """
        Retunr the current number of nodes

        :return: number of nodes in the grid
        """

        return len(self.__nodes)

    @property
    def max_size(self):
        """
        Computes the max size of the grid: row x col

        :return: max size of the grid
        """

        return self.__col * self.__row

    @property
    def nodes(self):
        return copy.copy(self.__nodes)

    @property
    def nodes_indexes(self):
        """"
        Return a list of nodes indexes: [n.idx if isinstance(n, Node) else None for n in self.__nodes.values()]
        """
        return list(self.__nodes.keys())

    def contains(self, node: Node):
        """
        Check if the node is in the grid

        :param node: the node
        :return: True if the node belongs to the grid; otherwise, False
        """

        return node in self.__nodes.values()

    def has_node(self, idx):
        """
        Check if the grid contains a node with idx

        :param idx: the idx of the node
        :return: True if the node with idx belongs to the grid; otherwise, False
        """

        return idx in self.__nodes.keys()

    def is_complete(self):
        """
        Check if the grid is full of nodes; no free places: n_free - n_nodes = 0

        :return: True if complete; otherwise, False
        """

        return len(self.__nodes) == self.max_size

    def is_empty(self):
        """
        Check if the grid is empty; no node in the grid: n_nodes = 0

        :return: True if empty; otherwise, False
        """

        return len(self.__nodes) == 0

    def is_free(self):
        """
        Check if the grid has free places: 0 <= len(self.__nodes) < self.max_size

        :return: True if free; otherwise, False
        """

        # return 0 <= len(self.__nodes) < self.max_size
        return self.n_free > 0

    @property
    def n_free(self):
        """
        Computes free places in the gird

        :return: number of free places
        """

        return self.max_size - len(self.__nodes)

    def node_of(self, idx):
        """
        Search for node with idx in the nodes dict

        :param idx: idx of the node
        :return: Node if it belongs to graph; otherwise, None
        """

        try:
            return self.__nodes[idx]
        except KeyError:
            return None

    def node(self, row, col):
        """
        Search for node in the given row and col

        :param row: row in the grid
        :param col: col in the grid
        :return: Node if exists in the given location; otherwise, None
        """

        if not row == -1 and not col == -1:
            for node in self.__nodes.values():
                if node.location[0] == row and node.location[1] == col:
                    return node
        return None

    def add_node(self, node: Node, row=-1, col=-1):
        """
        Add a node to the graph

        :param node: node to add
        :param row: row where to add the node
        :param col: column where to add the node
        :return: True if node is added; otherwise, False. It may raise an exception if grid is full or node is not of
        type <class Node>
        """

        if not self.is_free():
            raise IndexError("Grid of size=", self.size, " is full(free=", self.n_free, "), cannot add new node=", node)

        if not isinstance(node, Node):
            raise TypeError("Node=", node, " must be of <class Node>")

        if self.contains(node):
            return False

        node.graph = self
        node.location = (row, col)
        self.__nodes[node.idx] = node
        return True

    def remove_node(self, node):
        """
        Remove a node from the graph

        :param node: node to remove
        :return: True if node is removed; otherwise, False.
        """

        if not isinstance(node, Node):
            node = self.node_of(node)

        if self.contains(node):
            del self.__nodes[node.idx]
            node.disconnect()
            return True
        return False

    def clear(self):
        """
        Remove all nodes from the graph
        """

        for node in self.__nodes.values():
            node.disconnect()
        self.__nodes.clear()

    def add_edge(self, n1, n2, distance: float, direction: str):
        """
        Connect two nodes; create edge between n1 and n2

        :param n1: first node
        :param n2: second node
        :param distance: distance between n1 and n2
        :param direction: direction from n1 to n2 and vice versa.
        :return: (True, True) if an edge is created; otherwise, a tuple of two values where only one or both are False.
        """

        if not isinstance(n1, Node):
            n1 = self.node_of(n1)

        if not isinstance(n2, Node):
            n2 = self.node_of(n2)

        if self.contains(n1) and self.contains(n2):
            if direction == "from-left-to-right":
                return n1.add_neighbor(n2, distance=distance, direction='right'), \
                    n2.add_neighbor(n1, distance=distance, direction='left')
            elif direction == "from-right-to-left":
                return n1.add_neighbor(n2, distance=distance, direction='left'), \
                    n2.add_neighbor(n1, distance=distance, direction='right')
            elif direction == "from-up-to-down":
                return n1.add_neighbor(n2, distance=distance, direction='down'), \
                    n2.add_neighbor(n1, distance=distance, direction='up')
            elif direction == "from-down-to-up":
                return n1.add_neighbor(n2, distance=distance, direction='up'), \
                    n2.add_neighbor(n1, distance=distance, direction='down')
            else:
                raise KeyError("Unknown key=", direction, " for parameter 'direction'")
        return False, False

    def remove_edge(self, n1, n2):
        """
        Disconnect two ndes; remove edge between n1 and n2

        :param n1: first node
        :param n2: second node
        :return: True if edge is removed; otherwise, False
        """

        if not isinstance(n1, Node):
            n1 = self.node_of(n1)

        if not isinstance(n2, Node):
            n2 = self.node_of(n2)

        if self.contains(n1) and self.contains(n2):
            if n1.is_neighbor(n2):
                n1.remove_neighbor(n2)
                n2.remove_neighbor(n1)
                return True
        return False


def set_neighbors_of(grid, node_id):
    """
    Search for node's neighbors then connect them to the node, i.e., add them to the neighbors list

    :param grid: the grid object
    :param node_id: the node idx
    """

    node = grid.node_of(node_id)  # get the node object
    x, y = node.location  # the location of node in the grid

    #  check if neighbor is below the node
    if x - 1 >= 0 and grid.node(x - 1, y) is not None:
        try:
            grid.add_edge(node, grid.node(x - 1, y), distance=10, direction='from-down-to-up')
        except ValueError:
            pass

    #  check if neighbor is above the node
    if x + 1 < grid.shape[0] and grid.node(x + 1, y) is not None:
        try:
            grid.add_edge(node, grid.node(x + 1, y), distance=10, direction='from-up-to-down')
        except ValueError:
            pass

    #  check if neighbor is on the left the node
    if y - 1 >= 0 and grid.node(x, y - 1) is not None:
        try:
            grid.add_edge(node, grid.node(x, y - 1), distance=10, direction='from-right-to-left')
        except ValueError:
            pass

    #  check if neighbor is on the right the node
    if y + 1 < grid.shape[1] and grid.node(x, y + 1) is not None:
        try:
            grid.add_edge(node, grid.node(x, y + 1), distance=10, direction='from-left-to-right')
        except ValueError:
            pass


##########################################################################################
# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()
# Initialize the color sensor.
line_sensor = ColorSensor(Port.S1)

# Write your program here.
ev3.speaker.beep()

# Initialize the motors.
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)

ev3.speaker.beep()
  
BLACK = 9
WHITE = 85
threshold = (BLACK + WHITE) / 2
PROPORTIONAL_GAIN = 1.2

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    # dimension of the grid
    n = 2
    m = 2

    g = Grid(n, m)  # create a grid object

    # add nodes with idx from A to I to the grid
    for i, idx in enumerate(range(ord('A'), ord('J'))):
        pass
        # ord: returns the integer Unicode code point value of a single Unicode character
        # chr: does the opposite operation of ord()

        # n = Node(chr(idx))  # create a node object
        # g.add_node(n, i // 3, i % 3)  # add the node at the specified location
        # g.add_node(n)
        # set_neighbors_of(g, chr(idx))  # search for node's neighbors and add them to the neighbors list of the node
    
    a = Node('A')
    b = Node('B')
    c = Node('C')
    d = Node('D')
    
    g.add_node(a)
    g.add_node(b)
    g.add_node(c)
    g.add_node(d)

    g.add_edge(a, b, 290, 'from-left-to-right')
    g.add_edge(a, d, 290, 'from-down-to-up')
    g.add_edge(b, c, 290, 'from-down-to-up')
    g.add_edge(c, d, 290, 'from-right-to-left')

    # Creer le chemin : list path 

    path =

# Creer la boucle for 
    for idx, node in enumerate (path[1:]):
        
