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
        return next(self._turns_it)

    def next_road(self):
        return next(self._edges_it)
