from coordination_strategies.strategy import CoordinationStrategy


class StopRoad(CoordinationStrategy):
    def __init__(self, coordinator):
        super().__init__("Stop-sign", coordinator)

    def has_priority(self) -> bool:
        ...
