from robo.Dijkstra import Dijkstra
from robo.LaborynthGenerator import generateLaborynth

if __name__ == '__main__':
    graph, start, destination = generateLaborynth()
    dijstra = Dijkstra(graph, start, destination)
    instructions = dijstra.generate_instructions_for_shortest_path()
    print(instructions)
