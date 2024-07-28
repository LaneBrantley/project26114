import heapq

def read_graph_from_file(filename): #helper function to read the txt file to graph
    with open(filename, 'r') as file:
        lines = file.readlines()

    num_vertices, num_edges, graph_type = lines[0].strip().split() #assigns values to vertices, edges and type
    num_vertices, num_edges = int(num_vertices), int(num_edges) #changes to int
    is_directed = (graph_type == 'D') #gives is_directed bool value

    edges = []
    for line in lines[1:num_edges + 1]:
        u, v, weight = line.strip().split() #gets weight and gives int value
        weight = int(weight)
        edges.append((u, v, weight))

    source = None
    if len(lines) > num_edges + 1:
        source = lines[num_edges + 1].strip()

    return num_vertices, num_edges, is_directed, edges, source

def dijkstra(graph, start_vertex): #dijkstra algorithm
    D = {vertex: float('inf') for vertex in graph}
    D[start_vertex] = 0

    priority_queue = [(0, start_vertex)]
    while priority_queue:
        (dist, current_vertex) = heapq.heappop(priority_queue)

        for neighbor, weight in graph[current_vertex].items():
            distance = dist + weight

            if distance < D[neighbor]:
                D[neighbor] = distance
                heapq.heappush(priority_queue, (distance, neighbor))

    return D

class DisjointSet: #class to find/union on vertices
    def __init__(self, vertices):
        self.parent = {v: v for v in vertices}
        self.rank = {v: 0 for v in vertices}

    def find(self, item):
        if self.parent[item] == item:
            return item
        else:
            self.parent[item] = self.find(self.parent[item])
            return self.parent[item]

    def union(self, set1, set2):
        root1 = self.find(set1)
        root2 = self.find(set2)

        if root1 != root2:
            if self.rank[root1] > self.rank[root2]:
                self.parent[root2] = root1
            else:
                self.parent[root1] = root2
                if self.rank[root1] == self.rank[root2]:
                    self.rank[root2] += 1

def kruskal(vertices, edges):
    mst = []
    total_weight = 0

    disjoint_set = DisjointSet(vertices)

    sorted_edges = sorted(edges, key=lambda x: x[2])

    for edge in sorted_edges:
        u, v, weight = edge
        if disjoint_set.find(u) != disjoint_set.find(v):
            disjoint_set.union(u, v)
            mst.append(edge)
            total_weight += weight

    return mst, total_weight

def main(filename):
    num_vertices, num_edges, is_directed, edges, source = read_graph_from_file(filename)

    vertices = [chr(ord('A') + i) for i in range(num_vertices)]
    graph = {v: {} for v in vertices}

    for u, v, weight in edges:
        graph[u][v] = weight
        if not is_directed:
            graph[v][u] = weight

    if source:
        shortest_paths = dijkstra(graph, source)
        print(f"Shortest paths from {source}:")
        for vertex, distance in shortest_paths.items():
            print(f"Distance to {vertex}: {distance}")

    if not is_directed:
        mst, total_weight = kruskal(vertices, edges)
        print("Minimum Spanning Tree (MST):")
        for edge in mst:
            print(edge)
        print(f"Total weight of MST: {total_weight}")

if __name__ == "__main__":
    main('undirectedGraph2.txt')