from coordination_strategies.strategy import CoordinationStrategy


class TrafficLight(CoordinationStrategy):
    def __init__(self, coordinator):
        super().__init__("Traffic light", coordinator)

    def has_priority(self) -> bool:
        ...
