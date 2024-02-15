import numpy as np
from pedsim_agents.forces import Forcemodel, Forcemodels

@Forcemodels.register(Forcemodel.Name.SPINNY)
class Plugin_Spinny(Forcemodel):

    offset = np.deg2rad(15)

    def __init__(self):
        ...

    def compute(self, in_data, work_data):
        work_data.force = (np.array([
            [np.cos(self.offset), -np.sin(self.offset), 0],
            [np.sin(self.offset), np.cos(self.offset), 0],
            [0, 0, 1]
        ]) @ work_data.force.T).T

        work_data.force[np.linalg.norm(work_data.force, axis=1) == 0] = np.array([1,0,0]) 