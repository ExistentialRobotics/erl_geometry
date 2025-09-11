import argparse
import os
from typing import Dict

import cv2
import numpy as np
import vedo
from erl_common.storage import GridMapInfo2D
from vedo import Plotter

from . import HouseExpoMap
from .list_data import get_map_and_traj_files
from .list_data import get_map_dir
from .list_data import get_traj_dir
from .sequence import load_trajectory


class App:
    def __init__(self) -> None:
        parser = argparse.ArgumentParser()
        parser.add_argument("--init-house-expo-index", type=int, default=0)
        args = parser.parse_args()

        self.plt = Plotter()

        map_traj_pairs = get_map_and_traj_files()
        self.map_files = [x[0] for x in map_traj_pairs]
        self.map_dir = get_map_dir()
        self.traj_dir = get_traj_dir()
        self.configs: Dict[str, int] = dict()

        self.index = args.init_house_expo_index
        self.text_title = vedo.Text2D(f"HouseExpo Room {self.index}")

        self.button_next = self.plt.add_button(self._callback_button_next, ["  Next Room  ", ""], pos=[0.85, 0.01])
        self.button_previous = self.plt.add_button(
            self._callback_button_previous, ["Previous Room", ""], pos=[0.6, 0.01]
        )

        self.picture = self._get_picture()
        self.plt.add(self.text_title, self.picture, render=False)

    def _get_picture(self) -> vedo.Picture:
        map_filename = self.map_files[self.index]
        print(map_filename)
        map_file = os.path.join(self.map_dir, map_filename)
        house_expo_map = HouseExpoMap(map_file)
        token = os.path.splitext(map_filename)[0]
        traj_file = os.path.join(self.traj_dir, f"{token}.csv")

        width = 1000
        vertices = house_expo_map.meter_space.surface.vertices
        xmin = np.min(vertices[0])
        xmax = np.max(vertices[0])
        ymin = np.min(vertices[1])
        ymax = np.max(vertices[1])
        height = int(np.round(width * (ymax - ymin) / (xmax - xmin)))
        if height > width:
            r = height / 1000
            height = int(height / r)
            width = int(width / r)
        resolutions = np.array([(xmax - xmin) / width, (ymax - ymin) / height])
        padding = np.array([2, 2])
        grid_map_info = GridMapInfo2D(np.array([xmin, ymin]), np.array([xmax, ymax]), resolutions, padding)
        img = house_expo_map.meter_space.generate_map_image(grid_map_info)
        img = np.dstack([img] * 3).copy()

        if os.path.exists(traj_file):
            path = grid_map_info.meter_to_pixel_for_points(load_trajectory(traj_file)[:, :2].T).T.astype(
                np.int32
            )  # must be int32
            is_closed = False
            color = (255, 0, 0)
            thickness = 2
            cv2.polylines(img, [path], is_closed, color, thickness)

        return vedo.Picture(img)

    def _callback_button_next(self) -> None:
        if self.index == len(self.map_files) - 1:
            return
        self.index += 1
        self._update_picture()

    def _callback_button_previous(self) -> None:
        if self.index == 0:
            return
        self.index -= 1
        self._update_picture()

    def _update_picture(self) -> None:
        if self.picture is not None:
            self.plt.remove(self.picture)
        self.picture = self._get_picture()
        self.plt.add(self.picture, render=False)

        self.text_title.text(f"HouseExpo Room {self.index}")
        self.plt.render()

    def run(self) -> None:
        self.plt.show().interactive().close()


def main() -> None:
    app = App()
    app.run()


if __name__ == "__main__":
    main()
