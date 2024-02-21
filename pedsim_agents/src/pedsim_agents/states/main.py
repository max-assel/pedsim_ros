

import typing
from pedsim_agents.states import Pedestrian
from pedsim_agents.states.human.adult.adult import Adult
from pedsim_agents.utils import InData, WorkData
import pedsim_msgs.msg as pedsim_msgs

class PedsimStates:

    _agents: typing.Dict[str, Pedestrian]


    def __init__(self):
        self._agents = dict()

    def reset(self):
        self._agents.clear()        

    def pre(self, in_data: InData, work_data: WorkData):
        for i, ped in enumerate(in_data.agents):
            if ped.id not in self._agents:
                self._agents[ped.id] = Adult(ped.id, dict()).setup()
            machine = self._agents[ped.id]
            
            machine.pre(in_data, work_data, i)

    def post(self, in_data: InData, work_data: WorkData):
        for i, ped in enumerate(in_data.agents):
            machine = self._agents[ped.id]
            machine.post(in_data, work_data, i)
            
    def semantic(self) -> typing.Collection[typing.Dict[str, float]]:
        return [machine.semantic() for machine in self._agents.values()]