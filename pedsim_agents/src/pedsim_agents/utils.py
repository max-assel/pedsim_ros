import dataclasses
import enum
from typing import Any, Dict, List, Optional, Tuple, Type, TypeVar
import typing

import numpy as np

import pedsim_msgs.msg as pedsim_msgs
import std_msgs.msg as std_msgs
import geometry_msgs.msg as geometry_msgs

T = TypeVar("T")

def NList(l: Optional[List[T]]) -> List[T]:
    return [] if l is None else l

# INPUT

InMsg = pedsim_msgs.PedsimAgentsDataframe

@dataclasses.dataclass
class InData:
    header: std_msgs.Header
    agents: List[pedsim_msgs.AgentState]
    robots: List[pedsim_msgs.RobotState]
    groups: List[pedsim_msgs.AgentGroup]
    waypoints: List[pedsim_msgs.Waypoint]
    walls: List[pedsim_msgs.Wall]
    obstacles: List[pedsim_msgs.Obstacle]

@dataclasses.dataclass
class OutDatum:
    id: np.string_
    force: np.ndarray
    social_state: np.string_

OutMsg = pedsim_msgs.AgentFeedbacks
    
class WorkData:

    @classmethod
    def construct(cls, in_data: InData) -> "WorkData":
        data = WorkData(
            n_agents=len(in_data.agents)
        )
        
        for i, agent in enumerate(in_data.agents):
            data.id.append(agent.id)
            data.force[i] = agent.forces.force.x, agent.forces.force.y, agent.forces.force.z
            data.social_state[i] = 0

        return data

    agents: np.recarray

    # _storage: np.ndarray

    id: list
    force: np.ndarray
    social_state: np.ndarray

    def __init__(self, n_agents: int):
        self.id = list()
        # self._storage = np.zeros((n_agents, 4))
        # self.force = self._storage[:,0:3]
        # self.social_state = self._storage[:,3]

        self.force = np.zeros((n_agents, 3), dtype=np.double)
        self.social_state = np.zeros((n_agents,), dtype=np.uint8)

    def msg(self, header: typing.Optional[std_msgs.Header] = None) -> OutMsg:

        return OutMsg(**dict(
            agents = [
                pedsim_msgs.AgentFeedback(
                    id = id,
                    force = geometry_msgs.Vector3(*force),
                    social_state = 1
                )
                for id, force, social_state
                in zip(self.id, self.force.tolist(), self.social_state.tolist())
            ],
            **(dict(header=header) if header is not None else dict())
        ))



# SEMANTIC 

@enum.unique
class PedType(enum.IntEnum):
    NONE = 0
    adult = enum.auto()

@enum.unique
class SemanticAttribute(enum.Enum):
    IS_PEDESTRIAN = "pedestrian"
    IS_PEDESTRIAN_MOVING = "pedestrian_moving"
    PEDESTRIAN_VEL_X = "pedestrian_vel_x"
    PEDESTRIAN_VEL_Y = "pedestrian_vel_y"
    PEDESTRIAN_TYPE = "pedestrian_type"

SemanticData = Dict[SemanticAttribute, List[Tuple[geometry_msgs.Point, float]]]
SemanticMsg = pedsim_msgs.SemanticData

