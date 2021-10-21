class RoadMap:
    class Location:
        pass

    class Crossroad(Location):
        def __init__(self, cr_id):
            self.id = cr_id

    class Road(Location):
        def __init__(self, cr1, cr2, length):
            self.cr1 = cr1
            self.cr2 = cr2
            self.length = length

    def __init__(self):
        self._crossroads = []
        self._roads = []

    def add_crossroad(self, cr_id):
        self._crossroads.append(self.Crossroad(cr_id))

    def add_road(self, cr1, cr2, length):
        self._roads.append(self.Road(cr1, cr2, length))

    @staticmethod
    def dt_eltech_map(self):
        roadmap = RoadMap()

        for i in range(11):
            roadmap.add_crossroad(i)

        roadmap.add_road(1, 2, 5)
        roadmap.add_road(2, 3, 5)
        # TODO

        return roadmap
