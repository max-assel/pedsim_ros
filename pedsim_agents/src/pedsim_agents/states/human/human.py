import enum
import os
from typing import Dict

from .. import Pedestrian, StatechartProvider

import pedsim_msgs.msg as pedsim_msgs

@StatechartProvider.load(os.path.join(os.path.dirname(__file__), "human.yaml"))
class Human(Pedestrian):

    class Animation(str, enum.Enum):
        IDLE        = pedsim_msgs.AgentState.IDLE
        WALKING     = pedsim_msgs.AgentState.WALKING
        RUNNING     = pedsim_msgs.AgentState.RUNNING
        INTERACTING = pedsim_msgs.AgentState.INTERACTING
        TALKING     = pedsim_msgs.AgentState.TALKING
        PHONE       = pedsim_msgs.AgentState.PHONE

    def __init__(self, id: str, config: Dict):
        super().__init__(id, config)

        self._config = dict(
            **self._config,

            min_time_in_state = 2.0, # seconds

            stress_init = 0.0,
            stress_min = 0.01,
            stress_lo = 0.20,
            stress_hi = 0.80,
            stress_max = 0.99,

            energy_init = 0.50,
            energy_min = 0.01,
            energy_lo = 0.20,
            energy_hi = 0.80,
            energy_max = 0.99,

            social_init = 0.55,
            social_min = 0.01,
            social_lo = 0.20,
            social_hi = 0.80,
            social_max = 0.99,

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
            running_d_energy = -0.10,
            running_d_social = +0.0,

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

    def event_handler(self, event):
        if event.name == "animation":
            self.animation = self.Animation[event.data["animation"]].value
        else:
            super().event_handler(event)