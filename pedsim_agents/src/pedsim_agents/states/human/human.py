import enum
import os
from typing import Dict
import typing

from .. import Pedestrian, StatechartProvider

import pedsim_msgs.msg as pedsim_msgs

@StatechartProvider.load(os.path.join(os.path.dirname(__file__), "human.yaml"))
class Human(Pedestrian):

    class Animation(str, enum.Enum):
        IDLE        = "Idle"
        WALKING     = "Walking"
        RUNNING     = "Running"
        INTERACTING = "Interacting"
        TALKING     = "Talking"
        PHONE       = "Phone"

    def __init__(self, id: str, config: Dict):
        super().__init__(id, config)

        self._config = dict(
            **self._config,

            min_time_in_state = 2.0, # seconds

            stress_lo = 0.20,
            stress_hi = 0.80,

            energy_lo = 0.20,
            energy_hi = 0.80,

            social_lo = 0.20,
            social_hi = 0.80,

            # drift
            d_stress = -0.01,
            d_energy = +0.01,
            d_social = +0.01,

            # idle state [rest]
            idle_d_stress = -0.05,
            idle_d_energy = +0.15,
            idle_d_social = +0.05,

            # walking state [rest]
            walking_d_stress = +0.00,
            walking_d_energy = +0.00,
            walking_d_social = +0.00,

            # running state
            running_d_stress = +0.00,
            running_d_energy = -0.05,
            running_d_social = +0.00,

            # talking state
            talking_d_stress = +0.00,
            talking_d_energy = +0.00,
            talking_d_social = -0.05,

            # phone state
            phone_d_stress = +0.00,
            phone_d_energy = +0.00,
            phone_d_social = -0.05,

            # interacting state
            interacting_d_stress = +0.00,
            interacting_d_energy = -0.05,
            interacting_d_social = +0.05,
        )

        class _State(dict):

            def __init__(
                self,
                stress,
                energy,
                social
            ):
                self["stress"] = stress
                self["energy"] = energy
                self["social"] = social

            def __setitem__(self, __key, __value):
                super().__setitem__(__key, min(1, max(0, __value)))


        self._state = _State(
            energy = .50,
            stress = .00,
            social = .50
        )

    def event_handler(self, event):
        if event.name == "animation":
            self._animation = self.Animation[event.data["animation"]].value
        else:
            super().event_handler(event)

    def post(self, in_data, work_data, i):
        super().post(in_data, work_data, i)

        if self._animation == self.Animation.RUNNING: work_data.vmax[i] = 2.
        if self._animation == self.Animation.PHONE: work_data.vmax[i] = .5