

import typing
from pedsim_agents.states import Pedestrian
from pedsim_agents.states.human.adult.adult import Adult
from pedsim_agents.utils import InData, WorkData


class PedsimStates:

    _peds: typing.Dict[str, Pedestrian]


    def __init__(self):
        self._peds = dict()

    def reset(self):
        self._peds.clear()        

    def pre(self, in_data: InData, work_data: WorkData):

        for ped in in_data.agents:
            if ped.id not in self._peds:
                self._peds[ped.id] = Adult(ped.id, dict()).setup()
            machine = self._peds[ped.id]
            
            machine.tick()

        return in_data

    def post(self, in_data: InData, work_data: WorkData):
        for i, ped in enumerate(in_data.agents):
            machine = self._peds[ped.id]
            machine.post(in_data, work_data, i)
            
            # import rospy
            # rospy.logwarn(machine._state)

        return in_data