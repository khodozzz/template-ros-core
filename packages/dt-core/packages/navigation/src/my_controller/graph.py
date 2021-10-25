from queue import PriorityQueue


class Graph:
    class Edge:
        def __init__(self, source, target, weight, dir_source, dir_target):
            self.source = source
            self.target = target
            self.weight = weight

            self.dir_source = dir_source
            self.dir_target = dir_target

        def __repr__(self):
            return f'({self.source} -> {self.target} <{self.dir_source}, {self.dir_target}>)'

    def __init__(self):
        self._nodes = set()
        self._edges = dict()

    def add_node(self, node_id):
        self._nodes.add(node_id)

    def add_edge(self, source, target, weight=1, dir_source=0, dir_target=0, bidirectional=True):
        self.add_node(source)
        self.add_node(target)

        edges = self._edges.get(source, set())
        edges.add(self.Edge(source, target, weight, dir_source, dir_target))
        self._edges[source] = edges

        if bidirectional:
            edges = self._edges.get(target, set())
            edges.add(self.Edge(target, source, weight, dir_target - 180, dir_source - 180))
            self._edges[target] = edges

    def shortest_path(self, start, goal, start_angle=0):
        visited = set()
        queue = PriorityQueue()
        queue.put((0, [start], [], []))

        while queue:
            (dist, path, edges, turns) = queue.get()
            node = path[-1]

            if node == goal:
                return dist, path, edges, turns

            if node not in visited:
                for edge in self._edges[node]:
                    new_path = list(path)
                    new_path.append(edge.target)

                    new_edges = list(edges)
                    new_edges.append(edge)

                    new_turns = list(turns)
                    if len(edges) > 0:
                        angle = edge.dir_source - edges[-1].dir_target
                    else:
                        angle = edge.dir_source - start_angle
                    new_turns.append((360 + angle) * (angle < 0) + angle * (angle > 0))

                    new_dist = dist + edge.weight
                    queue.put((new_dist, new_path, new_edges, new_turns))

            visited.add(node)

        return None


if __name__ == '__main__':
    g = Graph()
    g.add_edge(1, 2, 5, 0, 0, bidirectional=True)
    g.add_edge(2, 3, 7, 0, 0, bidirectional=True)
    g.add_edge(3, 4, 15, 270, 180, bidirectional=True)

    print(g.shortest_path(4, 1))
