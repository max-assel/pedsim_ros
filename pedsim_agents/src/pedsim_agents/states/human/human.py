import enum
import functools
import itertools
import os
import typing

import sismic.model
import numpy as np

from pedsim_agents import utils

from .. import Pedestrian, StatechartProvider


class _States:
    STRESS = utils.SemanticAttribute.STATE_STRESS.value
    ENERGY = utils.SemanticAttribute.STATE_ENERGY.value
    SOCIAL = utils.SemanticAttribute.STATE_SOCIAL.value

@StatechartProvider.load(os.path.join(os.path.dirname(__file__), "human.yaml"))
class Human(Pedestrian):

    _tracked_agents: typing.Set[str]
    _tracked_obstacles: typing.Set[str]
    _seek: typing.Optional[str]

    class Animation(str, enum.Enum):
        IDLE        = "Idle"
        WALKING     = "Walking"
        RUNNING     = "Running"
        INTERACTING = "Interacting"
        TALKING     = "Talking"
        PHONE       = "Phone"

    def __init__(self, id: str, config: typing.Dict):
        super().__init__(id, config)

        self._tracked_agents = set()
        self._tracked_obstacles = set()
        self._seek = None

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

            # vision
            vision_range = 5,
            vision_angle = 45,

            # fears and norms
            personal_space_radius = 1.0,
            fear_robot = +0.02,
            fear_robot_speed_tolerance = 0.7, #m/s = 2.5km/h
        )

        class _State(dict):

            def __init__(
                self,
                **kwargs
            ):
                for k,v in kwargs.items():
                    self.__setitem__(k,v)

            def __setitem__(self, __key, __value):
                super().__setitem__(__key, min(1, max(0, __value)))


        self._state = _State(**{
            _States.ENERGY: .50,
            _States.STRESS: .00,
            _States.SOCIAL: .50
        })

    def event_handler(self, event):
        if event.name == "animation":
            self._animation = self.Animation[event.data["animation"]].value
        
        elif event.name == "seek":
            self._seek = event.data.get("id", None)
            if self._seek is None: self._destination = None

        else:
            super().event_handler(event)

    def pre(self, in_data, work_data, i, events: typing.Collection[sismic.model.Event] = ()):
        new_events: typing.Collection[sismic.model.Event] = []

        my_position = utils.msg_to_vec(in_data.agents[i].pose.position)

        # I shall fear no robot
        for robot in in_data.robots:
            if np.linalg.norm(utils.msg_to_vec(robot.pose.position) - my_position) < self._config["personal_space_radius"]:
                self._state[_States.STRESS] += self._config["fear_robot"]
                self._state[_States.STRESS] += max(0, np.linalg.norm(utils.msg_to_vec(robot.twist.linear)) - self._config["fear_robot_speed_tolerance"])
            

        # who do I see?
        i_see = lambda z: np.logical_or(
            np.logical_and(np.abs(np.angle(z)) < np.deg2rad(self._config["vision_angle"]), np.abs(z) < self._config["vision_range"]), # vision
            np.abs(z) < 1.1 * self._config["personal_space_radius"] # perception
        )
        
        p = in_data.agents[i].pose.position
        d_p = lambda v: (v.x-p.x, v.y-p.y)

        seen_agents = set(
            in_data.agents[a].id
            for a
            in np.where(
                i_see(
                    np.array([d_p(ag.pose.position) for ag in in_data.agents]).view(dtype=np.complex128)
                )
            )[0].tolist()
            if a != i
        ) 

        seen_obstacles = set(
            in_data.obstacles[o].name
            for o
            in np.where(
                i_see(
                    np.array([d_p(obstacle.pose.position) for obstacle in in_data.obstacles]).view(dtype=np.complex128)
                )
            )[0].tolist()
        ) 

        for added in seen_agents.difference(self._tracked_agents):
            new_events.append(sismic.model.Event("agent", id=added))
        for lost in self._tracked_agents.difference(seen_agents):
            if lost == self._seek:
                new_events.append(sismic.model.Event("lost"))
        self._tracked_agents = seen_agents

        for added in seen_obstacles.difference(self._tracked_obstacles):
            new_events.append(sismic.model.Event("obstacle", id=added))
        for lost in self._tracked_obstacles.difference(seen_obstacles):
            if lost == self._seek:
                new_events.append(sismic.model.Event("lost"))
        self._tracked_obstacles = seen_obstacles

        # time to seek
        if self._seek is not None:
            hit = next(
                itertools.chain(
                    (agent for agent in in_data.agents if agent.id == self._seek),
                    (obstacle for obstacle in in_data.obstacles if obstacle.name == self._seek),
                ),
                None
            )
            if hit is not None:
                p_seek = utils.msg_to_vec(hit.pose.position)
                p_my = my_position.copy()
                
                p_my -= p_seek
                p_my /= np.linalg.norm(p_my)
                p_my *= self._config["personal_space_radius"]
                p_my += p_seek

                self._destination = p_my.tolist()


        super().pre(in_data, work_data, i, events=(*events, *new_events))



    def post(self, in_data, work_data, i):
        if self._animation == self.Animation.RUNNING: work_data.vmax[i] = 2.
        if self._animation == self.Animation.PHONE: work_data.vmax[i] = .5

        super().post(in_data, work_data, i)