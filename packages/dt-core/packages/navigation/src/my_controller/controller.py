from my_controller import dt_etu


class NavigationController:
    def __init__(self, graph, start_pos):
        self.graph = graph
        self.current_pos = start_pos

        self._dist = self._path = self._edges = self._turns = self._turns_it = self._edges_it = None

    def plan_path(self, target):
        self._dist, self._path, self._edges, self._turns = self.graph.shortest_path(self.current_pos, target)
        self._turns_it = iter(self._turns)
        self._edges_it = iter(self._edges)

    def next_turn(self):
        turn_deg = next(self._turns_it, None)
        if turn_deg == 90:
            return 0
        elif turn_deg == 0:
            return 1
        elif turn_deg == 270:
            return 2

    def next_road(self):
        return next(self._edges_it)


if __name__ == '__main__':
    controller = NavigationController(dt_etu.build_graph(), dt_etu.start_pos())
    controller.plan_path(dt_etu.target_pos())
    while True:
        turn = controller.next_turn()
        if turn is None:
            break
        print(turn)
