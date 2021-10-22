class NavigationController:
    def __init__(self, graph, start_pos):
        self.graph = graph
        self.current_pos = start_pos

        self._turns_it = None

    def plan_path(self, target):
        path, turns, dist = self.graph.shortest_path(self.current_pos, target)
        self._turns_it = iter(turns)

        self.path = path

    def next_turn(self):
        return next(self._turns_it)
