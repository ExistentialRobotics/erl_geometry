import os.path
from typing import Generator
import numpy as np
import numpy.typing as npt

from erl_common.storage import GridMapDrawer2D
from erl_common.storage import GridMapInfo2D
from erl_geometry.house_expo.sequence import HouseExpoSequence


class HouseExpoSequenceVisualizer:
    def __init__(
        self,
        map_file: str,
        path_file: str,
        meter2pixel: int,
        padding: int,
        path_skip: int = 1,
        draw_trajectory: bool = True,
        lidar_mode: str = "kSddfV2",
    ):
        map_file = os.path.expanduser(map_file)
        path_file = os.path.expanduser(path_file)
        self.sequence = HouseExpoSequence(map_file, path_file, 0.0, lidar_mode)
        self.path_skip = path_skip
        vertices = self.sequence.map.meter_space.surface.vertices
        mins = np.min(vertices, axis=1)
        maxs = np.max(vertices, axis=1)
        self.grid_map_info = GridMapInfo2D(mins, maxs, np.array([1 / meter2pixel] * 2), np.array([padding] * 2))
        self.visualizer = GridMapDrawer2D(self.grid_map_info)
        image = np.dstack([self.sequence.map.meter_space.generate_map_image(self.grid_map_info)] * 3).copy()
        self.visualizer.image = image
        self.draw_trajectory = draw_trajectory
        self.ray_color = np.array([0, 255, 0, 255], dtype=np.float64)
        self.ray_thickness = 2
        self.traj_color = np.array([255, 0, 0, 255], dtype=np.float64)
        self.traj_thickness = 2

    def __len__(self) -> int:
        return len(self.sequence)

    def __getitem__(self, frame_index: int) -> npt.NDArray[np.uint8]:
        if frame_index < -len(self) or frame_index >= len(self):
            raise IndexError

        frame = self.sequence[frame_index]
        img = self.visualizer.draw_rays(
            self.visualizer.image,
            self.ray_color,
            self.ray_thickness,
            frame.translation,
            frame.ray_end_points_in_world_frame,
        )
        if self.draw_trajectory and frame_index > 0:
            img = self.visualizer.draw_polyline(
                img, self.traj_color, self.traj_thickness, False, self.sequence.path[:frame_index, :2].T
            )
        return img

    def __iter__(self) -> Generator[npt.NDArray[np.uint8], None, None]:
        for i in range(0, len(self), self.path_skip):
            yield self.__getitem__(i)
