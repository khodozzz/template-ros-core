from my_controller import dt_etu


class NavigationController:
    def __init__(self, graph):
        self.graph = graph

        self._dist = self._path = self._edges = self._turns = self._turns_it = None

    def plan_path(self, source, target):
        self._dist, self._path, self._edges, self._turns = self.graph.shortest_path(source, target)
        self._turns_it = iter(self._turns)

    def next_turn(self):
        turn_deg = next(self._turns_it, None)
        if turn_deg == 90:
            return 0, 'left'
        if turn_deg == 0:
            return 1, 'straight'
        if turn_deg == 270:
            return 2, 'right'
        return None


if __name__ == '__main__':
    controller = NavigationController(dt_etu.build_graph())
    controller.plan_path(dt_etu.start_pos(), dt_etu.target_pos())
    while True:
        turn = controller.next_turn()
        if turn is None:
            break
        print(turn)
