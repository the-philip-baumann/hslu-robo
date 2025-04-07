from collections import deque
from enum import Enum
from heapq import heapify, heappop, heappush
from typing import Dict

from graph import Graph, Node


class Instruction(Enum):
    STRAIGHT_FORWARD = 's'
    RIGHT = 'r'
    LEFT = 'l'


class Heading(Enum):
    X_POS = 0
    Y_POS = 1
    X_NEG = 2
    Y_NEG = 3


def is_heading_y_pos(heading: Heading):
    return heading == Heading.Y_POS


def is_heading_x_neg(heading: Heading):
    return heading == Heading.X_NEG


def is_heading_x_pos(heading: Heading):
    return heading == Heading.X_POS


def is_heading_y_neg(heading: Heading):
    return heading == Heading.Y_NEG


class Dijkstra:

    def __init__(self, graph: Graph, start: Node, destination: Node):
        self._graph = graph
        self._start = start
        self._destination = destination
        self._current_heading = Heading.Y_POS
        self._shortest_path = deque()

    def generate_distances_for_shortest_path(self) -> Dict[Node, float]:
        visited_nodes = set()
        distances = {node: float("inf") for node in self._graph.getnodes()}
        distances[self._start] = 0

        priority_queue = [(0, self._start)]
        heapify(priority_queue)

        while priority_queue:
            current_distance, current_node = heappop(priority_queue)

            if current_node in visited_nodes:
                continue

            visited_nodes.add(current_node)

            for edge in self._graph.get_incident(current_node):
                neighbor_node = self._graph.get_opposite(current_node, edge)
                distance = edge.getweight() + current_distance
                if distance < distances[neighbor_node]:
                    distances[neighbor_node] = distance
                    heappush(priority_queue, (distance, neighbor_node))

        return distances

    def explore_path_from_node(self, distances: Dict[Node, float], current_node: Node):
        if current_node is self._start:
            self._shortest_path.appendleft(current_node)
            return

        for edge in self._graph.get_incident(current_node):
            opposite = self._graph.get_opposite(current_node, edge)
            if distances[opposite] == distances[current_node] - edge.getweight():
                self._shortest_path.appendleft(current_node)
                return self.explore_path_from_node(distances, opposite)

    def map_path_to_instruction(self, start: Node, destination: Node) -> Instruction:
        if is_heading_y_pos(self._current_heading):
            if start.getx() < destination.getx():
                self._current_heading = Heading.X_POS
                return Instruction.RIGHT

            if start.getx() > destination.getx():
                self._current_heading = Heading.X_NEG
                return Instruction.LEFT

            return Instruction.STRAIGHT_FORWARD

        elif is_heading_y_neg(self._current_heading):
            if start.getx() < destination.getx():
                self._current_heading = Heading.X_NEG
                return Instruction.LEFT

            if start.getx() > destination.getx():
                self._current_heading = Heading.X_POS
                return Instruction.RIGHT

            return Instruction.STRAIGHT_FORWARD

        elif is_heading_x_neg(self._current_heading):
            if start.gety() < destination.gety():
                self._current_heading = Heading.Y_POS
                return Instruction.RIGHT

            if start.gety() > destination.gety():
                self._current_heading = Heading.Y_NEG
                return Instruction.LEFT

            return Instruction.STRAIGHT_FORWARD

        elif is_heading_x_pos(self._current_heading):
            if start.gety() < destination.gety():
                self._current_heading = Heading.Y_POS
                return Instruction.LEFT

            if start.gety() > destination.gety():
                self._current_heading = Heading.Y_NEG
                return Instruction.RIGHT

            return Instruction.STRAIGHT_FORWARD

        return Instruction.STRAIGHT_FORWARD

    def generate_instructions_for_shortest_path(self):
        distances: Dict[Node, float] = self.generate_distances_for_shortest_path()
        self.explore_path_from_node(distances, self._destination)
        return self.translate_path_to_instruction_set()

    def translate_path_to_instruction_set(self):
        instructionset = list()
        for i in range(0, len(self._shortest_path) - 1):
            instructionset.append(self.map_path_to_instruction(self._shortest_path[i], self._shortest_path[i + 1]))

        return instructionset
