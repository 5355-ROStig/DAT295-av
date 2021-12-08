from coordination_strategies.strategy import CoordinationStrategy


class GiveWayRoad(CoordinationStrategy):
    def __init__(self, coordinator):
        super().__init__("Give Way", coordinator)

    def has_priority(self):
        return False
