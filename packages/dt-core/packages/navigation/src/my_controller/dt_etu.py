from my_controller.graph import Graph


def build_graph():
    g = Graph()

    g.add_edge(1, 2, 5, 0, 0)
    g.add_edge(2, 3, 5, 0, 0)

    g.add_edge(2, 4, 15, 270, 0)
    g.add_edge(3, 4, 15, 270, 180)

    g.add_edge(1, 5, 20, 180, 270)
    g.add_edge(1, 6, 15, 270, 270)
    g.add_edge(3, 7, 25, 0, 270)

    g.add_edge(4, 8, 5, 270, 270)

    g.add_edge(5, 9, 7, 270, 270)
    g.add_edge(6, 10, 7, 270, 270)

    g.add_edge(9, 11, 25, 270, 0)
    g.add_edge(8, 11, 13, 270, 270)

    return g


def start_pos():
    return 11


def target_pos():
    return 1


if __name__ == '__main__':
    print(build_graph().shortest_path(11, 1))
    print(build_graph().shortest_path(1, 11))
