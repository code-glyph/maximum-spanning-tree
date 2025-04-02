from graph import OUTPUT_FILE
from collections import deque
import networkx as nx

def read_trees(file_name):
    lines = deque([line.strip() for line in open(file_name).readlines()])
    graphs = []
    if len(lines) > 0:
        number_of_graphs = int(lines.popleft())
    else:
        return []
    for _ in range(number_of_graphs):
        edges = []
        vertexedge = lines.popleft().split()
        NODES = int(vertexedge[0])
        number_of_edges = int(vertexedge[1])
        for _ in range(number_of_edges):
            edge_ends = lines.popleft().split()
            u = int(edge_ends[0])
            v = int(edge_ends[1])
            edges.append((u, v))

        graph = nx.Graph()
        graph.add_edges_from(edges)
        graphs.append(graph)
    return graphs

def get_all_vertices(edges):
    all_vertices = set()
    for pair in edges:
        all_vertices.update(pair)
    return all_vertices

graphs = read_trees(OUTPUT_FILE)
for number, graph in enumerate(graphs):
    if len(graph) != 0:
        if nx.is_tree(graph) == False:
            print(graph.nodes())
            print(f"{number + 1} case failed")
        else:
            leaves = [node for node, degree in graph.degree() if degree == 1]
            print(number, len(leaves))
    else:
        print(graph.nodes())
        print(f"{number + 1} case failed")