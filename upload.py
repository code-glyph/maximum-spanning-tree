
"""
    Team: Blue (Durva Dev, Varad Kshemkalyani, Vedant Manelkar)
    For this project, we have used 2 algorithms & we output the best algorithm
    --> Weighted BFS. Constructs a spanning tree by starting from the vertex with the highest degree and prioritizing edges to
                    high-degree vertices using a max-heap. Graph is explored just like BFS - adding unvisited vertices and their
                    connecting edges to the spanning tree
    --> Randomized. Randomly selects edges from the graph using Union Find (like Kruskal's algorithm) to make sure that no cycles
                    are formed. Algorithm stops when |V| - 1 vertices are added to the tree
"""

INPUT_FILE = 'all-hard.in'
OUTPUT_FILE = 'all-hard.out'
MAXIMUM_NODES = 100
MAXIMUM_EDGES = 2000
RANDOMIZED_ITERATIONS = 1000
NODES = 0

def weighted_bfs(graph):
   # Starting from the vertex with max degree
   start = graph.max_degree_vertex()
   # Initialize a new graph for the spanning tree
   tree = Graph(graph.vertices)
   Q = []  # Priority queue
   visited = set()  # Set to track visited nodes

   # Start with the max-degree vertex, negative to find the maximum spanning tree
   heapq.heappush(Q, (-graph.degrees[start], start))
   visited.add(start)

   while Q:
       # Pop vertex with highest priority
       _, u = heapq.heappop(Q)
       for v in graph.neighbors[u]:
           if v not in visited:
               # Add the edge (u, v) to the spanning tree
               tree.add_edge(Edge(u, v))  # Adding the edge to the spanning tree
               visited.add(v)  # Mark vertex v as visited
               heapq.heappush(Q, (-graph.degrees[v], v))  # Push with new priority
   return tree

def randomized(graph):
   # Create a UnionFind structure to help with cycle detection
   max_leaves = 0
   best_tree = Graph(graph.vertices) # graph.vertices contain the number of vertices in graph

    # while generating the all-hard.out file it was set to 1000
   for _ in range(RANDOMIZED_ITERATIONS):
       nodes = extract_nodes(graph)  # list of vertices
       node_set = UnionFind() # UnionFind is used to find cycles in the graph
       node_set.insert_objects(nodes)

       spanning_tree = Graph(graph.vertices) # creating a spanning tree with equal vertices
       edges = extract_edges(graph) # getting list of all edges
       number_of_edges = 0 # keeping track of number of edges added to the tree

       while edges:
           # Randomly select an edge
           edge = random.choice(edges)
           u, v = edge.u, edge.v
           if node_set.find(u) != node_set.find(v): # if the edge does not form a cycle, add it to the graph
               node_set.union(u, v)
               spanning_tree.add_edge(edge)
               edges.remove(edge) # remove it from the edge list
               number_of_edges += 1

           if number_of_edges == len(nodes) - 1:  # Check leaves when tree is complete, |E| = |V| - 1
               number_of_leaves = len(extract_leaf_nodes(spanning_tree))

               if number_of_leaves >= max_leaves: # Update best_tree if better number_of_leaves
                   max_leaves = number_of_leaves
                   best_tree = spanning_tree
               break
   return best_tree

def is_valid_MST(graph):
    # isolated vertex is already an MST
   if graph.vertices == 1:
       return True

   # MST should not contain any cycles
   graph.search()
   if graph.cycle == True:
       return False

   # checking for |E| = |V| - 1
   if len(extract_edges(graph)) != len(extract_nodes(graph)) - 1:
       return False

   # all nodes need to be present in the graph; extract_nodes function returns a list of all nodes
   ideal_vertices = set(range(len(extract_nodes(graph))))
   graph_vertices = set(extract_nodes(graph))
   if ideal_vertices != graph_vertices:
       return False
   return True

def find_maximum_spanning_tree(graphs):
   listOfMST = []
   for i in range(len(graphs)):
       listOfMST.append(findMSTforGraph(i, graphs[i]))
   return listOfMST

def findMSTforGraph(itr, graph):
   ALGORITHMS = [('RANDOM', randomized), ('WEIGHTED', weighted_bfs)]  # type: ignore
   optimal_mst = None
   best_case_leaves = 0
   alg = 'MST'
   # if graph is already a valid MST, we just find the leaves. No need to run the algorithms.
   if is_valid_MST(graph):
       graph.search()
       best_case_leaves = graph.leaves
       optimal_mst = graph
   else:
       for algorithm_name, algorithm in ALGORITHMS:
           NODES = len(extract_nodes(graph))
           tree = algorithm(graph)
           tree.search()
           if is_valid_MST(tree):
               if tree.leaves >= best_case_leaves:
                   optimal_mst = tree
                   best_case_leaves = tree.leaves
                   alg = algorithm_name
   return optimal_mst

if __name__=="__main__":
   graphs = read_graphs_from_input(INPUT_FILE)
   trees = find_maximum_spanning_tree(graphs)
   write_trees_to_file(trees, OUTPUT_FILE)


