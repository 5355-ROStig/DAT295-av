from typing import Type, Mapping

from mapdata.msg import RoadSection

from coordination_strategies.give_way_road import GiveWayRoad
from coordination_strategies.priority_road import PriorityRoad
from coordination_strategies.stop_road import StopRoad
from coordination_strategies.strategy import CoordinationStrategy
from coordination_strategies.traffic_light import TrafficLight

COORDINATION_STRATEGIES: Mapping[int, Type[CoordinationStrategy]] = {
    RoadSection.PRIORITY_ROAD: PriorityRoad,
    RoadSection.GIVE_WAY: GiveWayRoad,
    RoadSection.STOP_SIGN: StopRoad,
    RoadSection.TRAFFIC_LIGHT: TrafficLight
}
