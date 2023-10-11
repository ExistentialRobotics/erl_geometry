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
        self.lidar = Lidar2D(self.map.meter_space)
        self.lidar.sign_method = self.map.meter_space.SignMethod.kPolygon
        if lidar_mode == "kDdf":
            self.lidar.mode = self.lidar.Mode.kDdf
        elif lidar_mode == "kSddfV1":
            self.lidar.mode = self.lidar.Mode.kSddfV1
        elif lidar_mode == "kSddfV2":
            self.lidar.mode = self.lidar.Mode.kSddfV2
        else:
            raise ValueError(f"Unknown lidar_mode: {lidar_mode}.")
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
            self.set_lidar_to_frame_index(item)
            frame_setting = LidarFrame2D.Setting()
            frame_setting.valid_angle_min = self.lidar.min_angle
            frame_setting.valid_angle_max = self.lidar.max_angle
            frame = LidarFrame2D(frame_setting)
            frame.update(self.lidar.rotation, self.lidar.translation, self.lidar.angles, self.lidar.scan(self.parallel))
            return frame

    def set_lidar_to_frame_index(self, frame_index: int) -> None:
        self.lidar.translation = self.path[frame_index, :2]
        self.lidar.set_rotation(self.path[frame_index, 2])

    def get_rays_of_frames(self, frame_indices: Union[range, List[int]]) -> npt.NDArray[np.float64]:
        frame_indices = list(frame_indices)
        assert len(frame_indices) > 0
        if len(frame_indices) == 1:
            self.set_lidar_to_frame_index(frame_indices[0])
            return self.lidar.get_rays()
        else:
            xs = self.path[frame_indices, 0]
            ys = self.path[frame_indices, 1]
            thetas = self.path[frame_indices, 2]
            return self.lidar.get_rays_of_multi_poses(xs, ys, thetas, self.parallel)
