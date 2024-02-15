import enum
from typing import Dict

from .. import Pedestrian, StatechartProvider

@StatechartProvider.load("human.yaml")
class Human(Pedestrian):

    class Animation(str, enum.Enum):
        IDLE = "idle"
        WALKING = "walking"
        RUNNING = "running"
        INTERACTING = "interacting"
        TALKING = "talking"
        PHONE = "phone"

    def __init__(self, id: str, config: Dict):
        super().__init__(id, config)

        self.config = dict(

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
        if event.name == "":
            pass
        else:
            super().event_handler(event)