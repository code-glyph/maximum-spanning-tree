import heapq, random
from collections import deque

INPUT_FILE = 'all-hard.in'
OUTPUT_FILE = 'all-hard.out'
MAXIMUM_NODES = 100
MAXIMUM_EDGES = 2000

RANDOMIZED_ITERATIONS = 1000
NODES = 0

class Edge:
    def __init__(self, u, v):
        # order edges in specific order
        self.u = min(u,v)
        self.v = max(u,v)

    def __hash__(self):
        return self.u * MAXIMUM_NODES + self.v

    def __cmp__(self):
        return object.__cmp__(self)

    def __eq__(self, edge2):
        return self.u == edge2.u and self.v == edge2.v

    def check(self):
        assert self.u < 0 or self.v < 0 or self.u >= MAXIMUM_NODES or self.v >= MAXIMUM_NODES
        assert self.u == self.v

class Graph:
    def __init__(self, number_of_nodes):
        self.adjacent_nodes = [[] for i in range(number_of_nodes)]
        self.components = 0
        self.vertices = number_of_nodes
        self.traversed_nodes = 0
        self.leaves = 0
        self.cycle = False
        self.degrees = [0 for _ in range(number_of_nodes)]

    def add_edge(self, e):
        u = e.u
        v = e.v
        self.adjacent_nodes[u].append(v)
        self.adjacent_nodes[v].append(u)
        self.degrees[u] += 1
        self.degrees[v] += 1

    def get_degree(self, u):
        return self.degrees[u]

    def max_degree_vertex(self):
        return max(range(len(self.degrees)), key=lambda i: self.degrees[i])

    def perform_dfs(self):
        visited = [False for i in range(self.vertices)]
        self.traversed_nodes = 0
        self.leaves = 0
        self.components = 0
        self.cycle = False

        def dfs(node, parent):
            visited[node] = True
            for u in self.adjacent_nodes[node]:
                if u != parent:
                    if not visited[u]:
                        dfs(u, node)
                    else:
                        self.cycle = True

        for i in range(len(self.adjacent_nodes)):
            self.traversed_nodes += 1
            if len(self.adjacent_nodes[i]) == 1:
                self.leaves += 1
            if self.vertices == 1 and len(self.adjacent_nodes[i]) == 0:
                self.leaves += 1
            if not visited[i]:
                self.components += 1
                dfs(i, -1)

class UnionFind:
    def __init__(self, objects):
        self.component_sizes = {}  # Tracks the size of each component
        self.parent_links = {}  # Tracks the parent of each node
        self.index_to_object = {}  # Maps index to the original object
        self.object_to_index = {}  # Maps the original object to its index
        self.__repr__ = self.__str__
        for obj in objects:
            self.find(obj)

    def find(self, obj):
        # If the object is not in the mapping, initialize it
        if obj not in self.object_to_index:
            obj_index = len(self.object_to_index)
            self.component_sizes[obj_index] = 1
            self.object_to_index[obj] = obj_index
            self.index_to_object[obj_index] = obj
            self.parent_links[obj_index] = obj_index
            return obj

        # Path compression: Traverse and update parent links to point directly to root
        path = [self.object_to_index[obj]]
        root = self.parent_links[path[-1]]
        while root != path[-1]:
            path.append(root)
            root = self.parent_links[root]
        for index in path:
            self.parent_links[index] = root
        return self.index_to_object[root]

    def merge(self, object1, object2):
        # Find the roots of both objects
        root1 = self.find(object1)
        root2 = self.find(object2)

        if root1 != root2:  # If they are in different components
            index1 = self.object_to_index[root1]
            index2 = self.object_to_index[root2]
            size1 = self.component_sizes[index1]
            size2 = self.component_sizes[index2]

            # Union by size: Attach the smaller tree under the larger one
            if size1 < size2:
                root1, root2, index1, index2, size1, size2 = root2, root1, index2, index1, size2, size1
            self.component_sizes[index1] = size1 + size2
            del self.component_sizes[index2]
            self.parent_links[index2] = index1

    def __str__(self):
        # Group objects by their root
        components = {}
        for i in range(len(self.object_to_index)):
            components[i] = []
        for obj in self.object_to_index:
            root = self.find(obj)
            components[self.object_to_index[root]].append(obj)

        # Create a string representation of the components
        output = []
        for group in components.values():
            if group:
                output.append(repr(group))
        return ', '.join(output)


def make_graph(NODES, edge_set):
    G = Graph(NODES)
    for e in edge_set:
        G.add_edge(e)
    return G

def read_graphs_from_input(file_name):
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
            edges.append(Edge(u, v))

        graph = make_graph(NODES, edges)
        graphs.append(graph)
    return graphs

def write_trees_to_file(trees, FILE_NAME):
    output_file = open(FILE_NAME, 'w') # write # of instances
    output_file.write(str(len(trees)) + '\n')
    for tree in trees:
        edges = extract_edges(tree)
        number_of_edges = len(edges)
        if tree != None:
            output_file.write(str(tree.leaves) + ' ' + str(number_of_edges) + '\n')
            # Output all the edges in this tree to file
            for edge in sorted(edges, key=lambda edge: edge.u):
                u, v = edge.u, edge.v
                output_file.write(str(u) + ' ' + str(v) + '\n')
        else:
            # algorithms were not able to find a solution
            output_file.write(str(0) + ' ' + str(0) + '\n')
    output_file.close()

def extract_edges(graph):
    edge_set = set()
    for current_node in range(0, graph.vertices): # Iterate over each neighbor of the current node
        for adjacent_node in graph.adjacent_nodes[current_node]:
            edge_set.add(Edge(current_node, adjacent_node))

    # Convert the set of edges to a list before returning
    return list(edge_set)

def extract_nodes(graph):
    nodes = set()
    edges = extract_edges(graph)

    # If there are edges, add nodes from each edge
    if edges:
        for edge in edges:
            nodes.add(edge.u)
            nodes.add(edge.v)
    else:
        # If no edges exist, add all vertices
        # Since we are only concerned with connected graphs, the total number of components is always singular
        for itr in range(graph.vertices):
            nodes.add(itr)

    # Convert the set of nodes into a list before returning
    return list(nodes)

def extract_leaf_nodes(tree):
    leaves = []
    for node in range(0, len(tree.adjacent_nodes)):
        if len(tree.adjacent_nodes[node]) == 1:
            leaves.append(node)
    return leaves

def weighted_bfs(graph):
    # Starting from the vertex with max degree
    start = graph.max_degree_vertex()
    # Initialize a new graph for the spanning tree
    tree = Graph(graph.vertices)
    Q = []  # Priority queue
    visited = set()  # Set to track visited nodes

    heapq.heappush(Q, (-graph.degrees[start], start))  # Start with the max-degree vertex
    visited.add(start)

    while Q:
        _, u = heapq.heappop(Q)  # Pop vertex with highest priority
        for v in graph.adjacent_nodes[u]:
            if v not in visited:
                # Add the edge (u, v) to the spanning tree
                tree.add_edge(Edge(u, v))  # Adding the edge to the spanning tree
                visited.add(v)  # Mark vertex v as visited
                # priority = graph.degrees[v] - graph.degrees[u]  # Priority based on degree difference
                heapq.heappush(Q, (-graph.degrees[v], v))  # Push with new priority
    return tree

def randomized(graph):
   # Create a UnionFind structure to help with cycle detection
   max_leaves = 0
   optimal_mst = Graph(graph.vertices) # graph.vertices contain the number of vertices in graph

    # while generating the all-hard.out file it was set to 1000
   for _ in range(RANDOMIZED_ITERATIONS):
       nodes = extract_nodes(graph)  # list of vertices
       node_set = UnionFind(nodes) # UnionFind is used to find cycles in the graph
       # node_set.add()

       spanning_tree = Graph(graph.vertices) # creating a spanning tree with equal vertices
       edges = extract_edges(graph) # getting list of all edges
       number_of_edges = 0 # keeping track of number of edges added to the tree

       while edges:
           # Randomly select an edge
           edge = random.choice(edges)
           u, v = edge.u, edge.v
           if node_set.find(u) != node_set.find(v): # if the edge does not form a cycle, add it to the graph
               node_set.merge(u, v)
               spanning_tree.add_edge(edge)
               edges.remove(edge) # remove it from the edge list
               number_of_edges += 1

           if number_of_edges == len(nodes) - 1:  # Check leaves when tree is complete, |E| = |V| - 1
               number_of_leaves = len(extract_leaf_nodes(spanning_tree))
               if number_of_leaves > max_leaves: # Update optimal_mst if better number_of_leaves
                   max_leaves = number_of_leaves
                   optimal_mst = spanning_tree
               break
   return optimal_mst

def is_valid_MST(graph):
    if graph.vertices == 1:
        return True

    # MST should not contain any cycles
    graph.perform_dfs()
    if graph.cycle == True:
        return False

    # checking for |E| = |V| - 1
    if len(extract_edges(graph)) != len(extract_nodes(graph)) - 1:
        return False

    # ideally, all nodes need to be present in the graph
    ideal_vertices = set(range(len(extract_nodes(graph))))
    graph_vertices = set(extract_nodes(graph))
    if ideal_vertices != graph_vertices:
        return False
    return True

def find_maximum_spanning_tree(graphs):
    listOfMST = []
    # print("Instance\tVertices\tEdges\tLeaves\tAlgorithm")
    for i in range(len(graphs)):
        listOfMST.append(findMSTforGraph(i, graphs[i]))
    return listOfMST

def findMSTforGraph(itr, graph):
    ALGORITHMS = [ ('WEIGHTED', weighted_bfs), ('randomized', randomized)]  # type: ignore
    optimal_mst = None
    best_case_leaves = 0
    alg = 'MST'
    if is_valid_MST(graph):
        graph.perform_dfs()
        best_case_leaves = graph.leaves
        optimal_mst = graph
    else:
        NODES = graph.vertices
        tree = weighted_bfs(graph)
        if is_valid_MST(tree):
            if tree.leaves >= best_case_leaves:
                optimal_mst = tree
                best_case_leaves = tree.leaves
        tree = randomized(graph)
        if is_valid_MST(tree):
            if tree.leaves >= best_case_leaves:
                optimal_mst = tree
                best_case_leaves = tree.leaves
    return optimal_mst

if __name__=="__main__":
    graphs = read_graphs_from_input(INPUT_FILE)
    trees = find_maximum_spanning_tree(graphs)
    write_trees_to_file(trees, OUTPUT_FILE)