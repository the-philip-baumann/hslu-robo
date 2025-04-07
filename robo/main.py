from dijkstra import Dijkstra
from labirynth_generator import generateLabirynth

if __name__ == '__main__':
    graph, start, destination = generateLabirynth()
    dijstra = Dijkstra(graph, start, destination)
    instructions = dijstra.generate_instructions_for_shortest_path()
    print(instructions)
