from typing import List
from typing import Union
from typing import overload

import numpy as np
import numpy.typing as npt

from erl_geometry.pyerl_geometry import Lidar2D
from erl_geometry.pyerl_geometry import LidarFrame2D
from . import HouseExpoMap

__all__ = ["load_trajectory", "HouseExpoSequence"]


def load_trajectory(path_file: str) -> npt.NDArray[np.float64]:
    print(f"Loading trajectory from {path_file}")
    return np.loadtxt(path_file, dtype=float, delimiter=",")


class HouseExpoSequence:
    def __init__(
        self, json_file: str, path_file: str, wall_thickness: float = None, lidar_mode: str = "kSddfV2"
    ) -> None:
        if wall_thickness is None:
            self.map = HouseExpoMap(json_file)
        else:
            self.map = HouseExpoMap(json_file, wall_thickness)
        lidar_setting = Lidar2D.Setting()
        lidar_setting.min_angle = -np.pi
        lidar_setting.max_angle = np.pi
        lidar_setting.num_lines = 360
        lidar_setting.sign_method = self.map.meter_space.SignMethod.kPolygon
        lidar_setting.mode = Lidar2D.Mode(lidar_mode)
        self.lidar = Lidar2D(lidar_setting, self.map.meter_space)
        self.path = load_trajectory(path_file)
        self.parallel = True

    def __len__(self) -> int:
        return self.path.shape[0]

    @overload
    def __getitem__(self, item: int) -> LidarFrame2D:
        ...

    @overload
    def __getitem__(self, item: slice) -> List[LidarFrame2D]:
        ...

    def __getitem__(self, item: Union[slice, int]) -> Union[List[LidarFrame2D], LidarFrame2D]:
        if isinstance(item, slice):
            return [self.__getitem__(i) for i in list(range(len(self)))[item]]
        else:
            if item < -self.path.shape[0] or item >= self.path.shape[0]:
                raise IndexError
            rotation_angle = self.path[item, 2]
            rotation = np.array(
                [[np.cos(rotation_angle), -np.sin(rotation_angle)], [np.sin(rotation_angle), np.cos(rotation_angle)]]
            )
            translation = self.path[item, :2]
            frame_setting = LidarFrame2D.Setting()
            frame_setting.valid_angle_min = self.lidar.setting.min_angle
            frame_setting.valid_angle_max = self.lidar.setting.max_angle
            frame = LidarFrame2D(frame_setting)
            frame.update(
                rotation,
                translation,
                self.lidar.angles,
                self.lidar.scan(rotation, translation, self.parallel),
            )
            return frame
