from coordination_strategies.strategy import CoordinationStrategy

class PriorityRoad(CoordinationStrategy):
    def __init__(self):
        super().__init__("Priority")

    def has_priority(self):
        return True
