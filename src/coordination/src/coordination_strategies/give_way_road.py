from coordination_strategies.strategy import CoordinationStrategy

class GiveWayRoad(CoordinationStrategy):
    def __init__(self):
        super().__init__("Give Way")

    def has_priority(self):
        return False
