import argparse
from typing import List
from typing import Optional

import cv2
import numpy as np
import screeninfo
import vedo
import vedo.pyplot
from erl_common.storage import GridMapInfo2D
from erl_common.vedo_ext.lines import lidar_rays
from erl_common.vedo_ext.picture import image_mesh
from tqdm import tqdm

from erl_geometry import LogOddMap2D
from .list_data import get_map_and_traj_files
from .sequence import HouseExpoSequence


class App:
    def __init__(self) -> None:
        # parse arguments
        parser = argparse.ArgumentParser()
        parser.add_argument("--house-expo-index", default=0, type=int)
        parser.add_argument("--wall-thickness", default=None, type=float)
        parser.add_argument("--skip", default=10, type=int)
        parser.add_argument("--meter2pixel", default=50, type=int)
        parser.add_argument("--num-lines", default=360, type=int)
        parser.add_argument("--padding", default=5, type=int)
        parser.add_argument("--sensor-max-range", default=30, type=float)
        parser.add_argument("--measurement-certainty", default=0.99, type=float)
        parser.add_argument("--max-log-odd", default=100, type=float)
        parser.add_argument("--min-log-odd", default=-8, type=float)
        parser.add_argument("--threshold-occupied", default=0.5, type=float)
        parser.add_argument("--threshold-free", default=0.5, type=float)
        parser.add_argument("--hold", action="store_true")
        self.args = parser.parse_args()

        self.frame_index = 0

        self._init_data()
        self._init_gui()

    def _init_data(self) -> None:
        json_file, path_file = get_map_and_traj_files()[self.args.house_expo_index]
        self.sequence = HouseExpoSequence(
            json_file, path_file, wall_thickness=self.args.wall_thickness, lidar_mode="kDdf"
        )

        vertices = self.sequence.map.meter_space.surface.vertices
        self.grid_map_info = GridMapInfo2D(
            np.min(vertices, axis=1),
            np.max(vertices, axis=1),
            np.array([1 / self.args.meter2pixel] * 2),
            np.array([self.args.padding] * 2),
        )
        self.extent = [
            self.grid_map_info.min[0],
            self.grid_map_info.max[0],
            self.grid_map_info.min[1],
            self.grid_map_info.max[1],
        ]

        self.sequence.lidar.num_lines = self.args.num_lines

        log_odd_map_setting = LogOddMap2D.Setting()
        log_odd_map_setting.sensor_max_range = self.args.sensor_max_range
        log_odd_map_setting.measurement_certainty = self.args.measurement_certainty
        log_odd_map_setting.max_log_odd = self.args.max_log_odd
        log_odd_map_setting.min_log_odd = self.args.min_log_odd
        log_odd_map_setting.threshold_occupied = self.args.threshold_occupied
        log_odd_map_setting.threshold_free = self.args.threshold_free
        self.log_odd_map = LogOddMap2D(log_odd_map_setting, self.grid_map_info)

    def _init_gui(self) -> None:
        self.dx = 0.01
        self.ux = (1 - 4 * self.dx) / 4  # unit width
        self.dy = self.dx
        self.uy = (1 - 4 * self.dy) / 3  # unit height
        # layout
        # | 0: The full window
        # |--------------------------|--------------------|--------------------|
        # |                          |     2: log_map     | 3: unexplored_mask |
        # |                          |--------------------|--------------------|
        # |          1: env          | 4: possibility_map |  5: occupied_mask  |
        # |                          |--------------------|--------------------|
        # |                          |  6: occupancy_map  |    7: free_mask    |
        # |--------------------------|--------------------|--------------------|
        self.rn_env_map = 1
        self.rn_log_map = 2
        self.rn_unexplored_mask = 3
        self.rn_possibility_map = 4
        self.rn_occupied_mask = 5
        self.rn_occupancy_map = 6
        self.rn_free_mask = 7
        shape = [
            dict(bottomleft=(0, 0), topright=(1, 1), bg="k1"),  # 0: the full empty window
            dict(
                bottomleft=(self.dx, self.dy),
                topright=(self.dx + 2 * self.ux, 1 - self.dy),
                bg="w",
            ),  # 1
            dict(
                bottomleft=(2 * self.dx + 2 * self.ux, 1 - self.dy - self.uy),
                topright=(2 * self.dx + 3 * self.ux, 1 - self.dy),
                bg="w",
            ),  # 2
            dict(
                bottomleft=(1 - self.dx - self.ux, 1 - self.dy - self.uy),
                topright=(1 - self.dx, 1 - self.dy),
                bg="w",
            ),  # 3
            dict(
                bottomleft=(2 * self.dx + 2 * self.ux, 2 * self.dy + self.uy),
                topright=(2 * self.dx + 3 * self.ux, 1 - 2 * self.dy - self.uy),
                bg="w",
            ),  # 4
            dict(
                bottomleft=(1 - self.dx - self.ux, 2 * self.dy + self.uy),
                topright=(1 - self.dx, 1 - 2 * self.dy - self.uy),
                bg="w",
            ),  # 5
            dict(
                bottomleft=(2 * self.dx + 2 * self.ux, self.dy),
                topright=(2 * self.dx + 3 * self.ux, self.dy + self.uy),
                bg="w",
            ),  # 6
            dict(
                bottomleft=(1 - self.dx - self.ux, self.dy),
                topright=(1 - self.dx, self.dy + self.uy),
                bg="w",
            ),  # 7
        ]
        # window top-left position on screen
        pos = None
        for monitor in screeninfo.get_monitors():
            if monitor.height > monitor.width:
                # portrait mode
                continue
            pos = [monitor.width // 2 - 200, monitor.height // 2 - 100]
        # size of the rendering window
        size = (2560, 1280)

        self.plt = vedo.Plotter(shape=shape, sharecam=False, size=size, pos=pos)

        self.vedo_lidar_rays: Optional[vedo.Lines] = None
        self.vedo_curve_path: Optional[vedo.Lines] = None
        self.vedo_point_frontiers: Optional[List[vedo.Points]] = None

        self.vedo_mesh_env = self._get_env_map()
        self.plt.at(self.rn_env_map).show(self.vedo_mesh_env, axes=1, zoom="tight")

        self.plt.at(self.rn_log_map).add(vedo.Text2D("Log Odd Map", pos="top-middle", c="red"))
        self.plt.at(self.rn_possibility_map).add(vedo.Text2D("Possibility Map", pos="top-middle", c="red"))
        self.plt.at(self.rn_occupancy_map).add(vedo.Text2D("Occupancy Map", pos="top-middle", c="red"))
        self.plt.at(self.rn_unexplored_mask).add(vedo.Text2D("Unexplored Mask", pos="top-middle", c="red"))
        self.plt.at(self.rn_occupied_mask).add(vedo.Text2D("Occupied Mask", pos="top-middle", c="red"))
        self.plt.at(self.rn_free_mask).add(vedo.Text2D("Free Mask", pos="top-middle", c="red"))

        self.vedo_picture_log_map = None
        self.vedo_picture_possibility_map = None
        self.vedo_picture_occupancy_map = None
        self.vedo_picture_unexplored_mask = None
        self.vedo_picture_occupied_mask = None
        self.vedo_picture_free_mask = None

    def run(self) -> None:
        self.plt.show()

        if self.args.hold:
            input("Press Enter to continue...")

        cnt = 0
        for frame_index in tqdm(range(0, len(self.sequence), self.args.skip), ncols=80):
            self.frame_index = frame_index
            frame = self.sequence[self.frame_index]
            self.log_odd_map.update(frame.translation, frame.theta, frame.angles, frame.ranges)
            # self.log_odd_map.compute_statistics_of_lidar_frame(
            #     frame.translation, frame.theta, frame.angles, frame.ranges, clip_ranges=True
            # )

            self._update_renderer_env_map()
            self._update_renderer_log_map()
            self._update_renderer_possibility_map()
            self._update_renderer_occupancy_map()
            self._update_renderer_unexplored_mask()
            self._update_renderer_occupied_mask()
            self._update_renderer_free_mask()

            cnt += 1
            if cnt % 10 == 0:
                # cannot call this event loop too frequently
                self.plt.interactor.ProcessEvents()

        self.plt.at(0).interactive().close()

    def _update_renderer_env_map(self) -> None:
        self.plt.at(self.rn_env_map)
        if self.vedo_lidar_rays is not None:
            self.plt.remove(self.vedo_lidar_rays)
        if self.vedo_curve_path is not None:
            self.plt.remove(self.vedo_curve_path)

        frame = self.sequence[self.frame_index]

        self.vedo_lidar_rays = lidar_rays(
            frame.translation,
            frame.oriented_ray_directions,
            frame.ranges,
            color="lime",
            alpha=0.7,
        )

        path = self.sequence.path[: self.frame_index, :2]
        if path.shape[1] > 0:
            self.vedo_curve_path = vedo.Lines(path[:-1], path[1:], c="red", lw=2)
            self.plt.add(self.vedo_curve_path, render=False)

        if self.vedo_point_frontiers is not None:
            self.plt.remove(*self.vedo_point_frontiers)
        frontiers = self.log_odd_map.get_frontiers(approx_iters=10)
        self.vedo_point_frontiers = [
            vedo.Points(
                self.grid_map_info.grid_to_meter_for_points(frontier.astype(float)).T,
                r=4,
                c="blue",
            )
            for frontier in frontiers
        ]

        self.plt.add(self.vedo_lidar_rays, *self.vedo_point_frontiers)

    def _update_renderer_log_map(self) -> None:
        self.plt.at(self.rn_log_map)
        if self.vedo_picture_log_map is not None:
            self.plt.remove(self.vedo_picture_log_map)
        m = self.log_odd_map.log_map.T[::-1]
        m = ((m - np.min(m)) / (np.max(m) - np.min(m)) * 255).astype(np.uint8)
        self.vedo_picture_log_map = vedo.Picture(cv2.applyColorMap(m, cv2.COLORMAP_SUMMER))
        self.plt.show(self.vedo_picture_log_map, axes=1, zoom="tight")

    def _update_renderer_possibility_map(self) -> None:
        self.plt.at(self.rn_possibility_map)
        if self.vedo_picture_possibility_map is not None:
            self.plt.remove(self.vedo_picture_possibility_map)
        m = self.log_odd_map.possibility_map.T[::-1]
        m = (m * 255).astype(np.uint8)
        self.vedo_picture_possibility_map = vedo.Picture(cv2.applyColorMap(m, cv2.COLORMAP_RAINBOW))
        self.plt.show(self.vedo_picture_possibility_map, axes=1, zoom="tight")

    def _update_renderer_occupancy_map(self) -> None:
        self.plt.at(self.rn_occupancy_map)
        if self.vedo_picture_occupancy_map is not None:
            self.plt.remove(self.vedo_picture_occupancy_map)
        m = self.log_odd_map.occupancy_map.T
        self.vedo_picture_occupancy_map = vedo.Picture(m)
        self.plt.show(self.vedo_picture_occupancy_map, axes=1, zoom="tight")

    def _update_renderer_unexplored_mask(self) -> None:
        self.plt.at(self.rn_unexplored_mask)
        if self.vedo_picture_unexplored_mask is not None:
            self.plt.remove(self.vedo_picture_unexplored_mask)
        mask = self.log_odd_map.unexplored_mask.T * 255
        self.vedo_picture_unexplored_mask = vedo.Picture(mask)
        self.plt.show(self.vedo_picture_unexplored_mask, axes=1, zoom="tight")

    def _update_renderer_occupied_mask(self) -> None:
        self.plt.at(self.rn_occupied_mask)
        if self.vedo_picture_occupied_mask is not None:
            self.plt.remove(self.vedo_picture_occupied_mask)
        mask = self.log_odd_map.occupied_mask.T * 255
        self.vedo_picture_occupied_mask = vedo.Picture(mask)
        self.plt.show(self.vedo_picture_occupied_mask, axes=1, zoom="tight")

    def _update_renderer_free_mask(self) -> None:
        self.plt.at(self.rn_free_mask)
        if self.vedo_picture_free_mask is not None:
            self.plt.remove(self.vedo_picture_free_mask)
        mask = self.log_odd_map.free_mask.T * 255
        self.vedo_picture_free_mask = vedo.Picture(mask)
        self.plt.show(self.vedo_picture_free_mask, axes=1, zoom="tight")

    def _get_env_map(self) -> vedo.Grid:
        image = self.sequence.map.meter_space.generate_map_image(self.grid_map_info)[::-1]
        image = np.dstack([image] * 3).copy()
        return image_mesh(
            vedo.Picture(image), self.grid_map_info.get_dim_lin_space(0), self.grid_map_info.get_dim_lin_space(1)[::-1]
        )


def main() -> None:
    app = App()
    app.run()


if __name__ == "__main__":
    main()
