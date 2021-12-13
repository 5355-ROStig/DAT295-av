from typing import Dict, List, Type

from phases.approach_phase import ApproachPhase
from phases.crossing_phase import CrossingPhase
from phases.leave_phase import LeavePhase
from phases.phase import Phase
from phases.stop_phase import StopAtIntersection
from phases.stopsign_stop_phase import StopSignAtIntersection

MISSIONS: Dict[str, List[Type[Phase]]] = {
    'scenario1': [
        ApproachPhase,
        StopAtIntersection,
        CrossingPhase,
        LeavePhase
    ],
    'scenario2': [
        ApproachPhase,
        StopSignAtIntersection,
        CrossingPhase,
        LeavePhase
    ],
    'scenario3': [
        ApproachPhase,
        StopAtIntersection,
        CrossingPhase,
        LeavePhase
    ]
}

