from typing import Tuple


class Node:

    def __init__(self, x: int, y: int):
        self._x = x
        self._y = y

    def getx(self):
        return self._x

    def gety(self):
        return self._y

    def getscore(self):
        return self._score

    def __str__(self):
        return f'({self._x}, {self._y})'

    def __eq__(self, other):
        return self._x == other.getx() and self._y == other.gety()

    def __hash__(self):
        return hash((self._x, self._y))

    def __lt__(self, other):
        return self._x < other.getx() and self._y < other.gety()


class Edge:

    def __init__(self, nodes: Tuple[Node, Node], weigth: int = 1):
        self._node1, self._node2 = nodes
        self._weigth = weigth

    def getnode1(self):
        return self._node1

    def getnode2(self):
        return self._node2

    def getweight(self):
        return self._weigth

    def __str__(self):
        return f'{self._node1}, {self._node2}'


class Graph:

    def __init__(self):
        self._nodes: list[Node] = []
        self._edges: list[Edge] = []

    def insert_node(self, node: Node):
        self._nodes.append(node)

    def insert_nodes(self, *nodes: Node):
        nodes = list(nodes)
        for node in nodes:
            self.insert_node(node)

    def insert_edge(self, edge: Edge):
        self._edges.append(edge)

    def insert_edges(self, *edges: Edge):
        edges = list(edges)
        for edge in edges:
            self.insert_edge(edge)

    def is_adjacent(self, nodes: Tuple[Node, Node]):
        """Zwei Knoten (Vertices) sind adjazent, wenn eine direkte Kante (Edge) zwischen ihnen existiert"""
        node1, node2 = nodes
        for edge in self._edges:
            if (edge.getnode1() == node1 or edge.getnode2() == node1) and (
                    edge.getnode1() == node2 or edge.getnode2() == node2):
                return True

        return False

    def get_incident(self, node: Node):
        """Ein Edge ist incident, wenn er an einem Node ein oder ausgeht"""
        return [edge for edge in self._edges if edge.getnode1() == node or edge.getnode2() == node]

    def get_opposite(self, node: Node, edge: Edge):
        return edge.getnode1() if edge.getnode1() != node else edge.getnode2()

    def getnodes(self):
        return self._nodes

    def getedges(self):
        return self._edges