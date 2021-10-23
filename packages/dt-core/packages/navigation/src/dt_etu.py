from graph import Graph


def build_graph():
    g = Graph()
    g.add_edge(1, 2, 5, 0, 0, bidirectional=True)
    g.add_edge(2, 3, 7, 0, 0, bidirectional=True)
    g.add_edge(1, 4, 8, 270, 0)
    g.add_edge(2, 4, 10, 270, 270)
    g.add_edge(3, 4, 15, 270, 180, bidirectional=True)
    return g


def start_pos():
    return 4


def target_pos():
    return 1
