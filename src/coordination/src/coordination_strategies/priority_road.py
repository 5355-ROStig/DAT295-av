from coordination_strategies.strategy import CoordinationStrategy


class PriorityRoad(CoordinationStrategy):
    def __init__(self, coordinator):
        super().__init__("Priority", coordinator)

    def has_priority(self):
        return True
