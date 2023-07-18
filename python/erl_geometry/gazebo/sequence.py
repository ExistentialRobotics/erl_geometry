import os.path
from typing import List
from typing import Union
from typing import overload

import numpy as np
from erl_geometry import Lidar2DFrame
from scipy.io.matlab import loadmat


class GazeboSequence:
    def __init__(self) -> None:
        self._data = loadmat(os.path.join(os.path.dirname(__file__), "gazebo1.mat"))

        sensor_offset = np.array([[0.08], [0]])

        # R^w_r [R^r_s x^s + t^r_s] + t^w_r = R^w_r x^s + (R^w_r t^r_s + t^w_r)
        sensor_pose_relative_to_robot = np.array(
            [
                [1.0, 0.0, sensor_offset[0, 0]],
                [0.0, 1.0, sensor_offset[1, 0]],
                [0.0, 0.0, 1.0],
            ]
        )

        # robot pose, we need to transform it to sensor pose
        x, y, theta = self._data["poses"].T  # each row is (x, y, theta)
        s = np.sin(theta)
        c = np.cos(theta)
        zeros = np.zeros_like(s)
        ones = np.ones_like(s)
        robot_poses = np.array([[c, -s, x], [s, c, y], [zeros, zeros, ones]]).transpose([2, 0, 1])
        sensor_pose = robot_poses @ sensor_pose_relative_to_robot
        x = sensor_pose[:, 0, 2]
        y = sensor_pose[:, 1, 2]
        theta = np.arctan2(sensor_pose[:, 1, 0], sensor_pose[:, 0, 0])
        self.path: np.ndarray = np.array([x, y, theta]).T
        self.angles = self._data["thetas"][0].astype(np.float64)
        self.ranges = self._data["ranges"].astype(np.float64)

    def __len__(self) -> int:
        return self.path.shape[0]

    @overload
    def __getitem__(self, item: int) -> Lidar2DFrame:
        ...

    @overload
    def __getitem__(self, item: slice) -> List[Lidar2DFrame]:
        ...

    def __getitem__(self, item: Union[slice, int]) -> Union[List[Lidar2DFrame], Lidar2DFrame]:
        if isinstance(item, slice):
            return [self.__getitem__(i) for i in list(range(len(self)))[item]]
        else:
            if item < -self.path.shape[0] or item >= self.path.shape[0]:
                raise IndexError
            x, y, theta = self.path[item]
            return Lidar2DFrame(x, y, theta, self.angles, self.ranges[item])
