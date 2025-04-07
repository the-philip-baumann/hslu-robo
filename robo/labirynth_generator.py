from graph import Graph, Edge, Node
from typing import Tuple


def generateLabirynth() -> Tuple[Graph, Node, Node]:
    """ Generiert einen Graph mit den Kreuzungen als Nodes und die Strassen als Edges. 
        Die Nodes im Graphen sind in einem 4x4 Raster gestaltet. 
        Nummerierung ist aufsteigend horizontal"""

    starting_node = Node(0, 0)
    node10 = Node(1, 0)
    node20 = Node(2, 0)
    node30 = Node(3, 0)
    node40 = Node(4, 0)
    node01 = Node(0, 1)
    node11 = Node(1, 1)
    node21 = Node(2, 1)
    node31 = Node(3, 1)
    node41 = Node(4, 1)
    node02 = Node(0, 2)
    node12 = Node(1, 2)
    node22 = Node(2, 2)
    node32 = Node(3, 2)
    node42 = Node(4, 2)
    node03 = Node(0, 3)
    node13 = Node(1, 3)
    node23 = Node(2, 3)
    node33 = Node(3, 3)
    node43 = Node(4, 3)
    node04 = Node(0, 4)
    node14 = Node(1, 4)
    node24 = Node(2, 4)
    node34 = Node(3, 4)
    destination_node = Node(4, 4)

    # First horizontal
    edge_10_20 = Edge((node10, node20))
    edge_20_30 = Edge((node20, node30))
    edge_30_40 = Edge((node30, node40))

    #First Vertical
    edge_00_01 = Edge((starting_node, node01))
    edge_10_11 = Edge((node10, node11))
    edge_30_31 = Edge((node30, node31))
    edge_40_41 = Edge((node40, node41))

    # Second horizontal
    edge_01_11 = Edge((node01, node11))
    edge_11_21 = Edge((node11, node21))
    edge_21_31 = Edge((node21, node31))

    #Second Vertical
    edge_01_02 = Edge((node01, node02))
    edge_31_32 = Edge((node31, node32))

    #Third horizontal
    edge_02_12 = Edge((node02, node12))
    edge_12_22 = Edge((node12, node22))
    edge_22_32 = Edge((node22, node32))
    edge_32_42 = Edge((node32, node42))

    # Third Vertical
    edge_12_13 = Edge((node12, node13))
    edge_22_23 = Edge((node22, node23))
    edge_42_43 = Edge((node42, node43))

    # Fourth Horizontal
    edge_03_13 = Edge((node03, node13))
    edge_13_23 = Edge((node13, node23))
    edge_23_33 = Edge((node23, node33))
    edge_33_43 = Edge((node33, node43))

    # Fourth Vertical
    edge_03_04 = Edge((node03, node04))
    edge_13_14 = Edge((node13, node14))
    edge_23_24 = Edge((node23, node24))
    edge_33_34 = Edge((node33, node34))

    # Fifth Horizontal
    edge_04_14 = Edge((node04, node14))
    edge_34_44 = Edge((node34, destination_node))

    graph: Graph = Graph()
    graph.insert_nodes(
        starting_node,
        node10,
        node20,
        node30,
        node40,
        node01,
        node11,
        node21,
        node31,
        node41,
        node02,
        node12,
        node22,
        node32,
        node42,
        node03,
        node13,
        node23,
        node33,
        node43,
        node04,
        node14,
        node24,
        node34,
        destination_node)

    graph.insert_edges(
        edge_10_20,
        edge_20_30,
        edge_30_40,
        edge_00_01,
        edge_10_11,
        edge_30_31,
        edge_40_41,
        edge_01_11,
        edge_11_21,
        edge_21_31,
        edge_01_02,
        edge_31_32,
        edge_02_12,
        edge_12_22,
        edge_22_32,
        edge_32_42,
        edge_12_13,
        edge_22_23,
        edge_42_43,
        edge_03_13,
        edge_13_23,
        edge_23_33,
        edge_33_43,
        edge_03_04,
        edge_13_14,
        edge_23_24,
        edge_33_34,
        edge_04_14,
        edge_34_44
    )

    return graph, starting_node, destination_node
