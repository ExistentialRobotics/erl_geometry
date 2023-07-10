`erl_geometry.house_expo`
=========================

# Dependencies

- [diff_info_gathering](https://github.com/daizhirui/diff_info_gathering.git)
- [bc_exploration](https://github.com/daizhirui/bc_exploration.git)

# An Example Of Using This Dataset

```python
from erl_geometry.house_expo.list_data import get_map_and_traj_files
from erl_geometry.house_expo.sequence import HouseExpoSequence, Lidar2dFrame

samples = get_map_and_traj_files()
sequence = HouseExpoSequence(samples[0][0], samples[0][1], lidar_mode="kDdf")
for frame in sequence:
    frame: Lidar2dFrame
    # do your processing here
    pass
```

# SurfaceData Distribution and Loading

- `bad_json_files.txt`: list of json files that define bad room layouts. e.g.
  inaccessible room, too narrow path
- `uncompress_data.py`: uncompress `json.tar.gz` and `traj.tar.gz` into
  `$HOME/.cache/erl_geometry/house_expo/data`.
    - You can call `erl-geometry-house-expo-uncompress-data` in command line.
- `list_data.py`: list available .json files, .csv files, etc.
    - You can call `erl-geometry-house-expo-list-data` in command line.
- `sequence.py`: load a pair of .json file and .csv file

# Trajectory Generation

- `generate_png.py`
- `generate_trajectory.py`
    - You can call `erl-geometry-house-expo-generate-trajectory` in command line.
- `generate_all_trajectories.py`
    - You can call `erl-geometry-house-expo-generate-all-trajectories` in command
      line.
- `verify_data.py`: verify .csv files, results are stored in verify_cache.pkl
    - You can call `erl-geometry-house-expo-verify-data` in command line.
- `merge_verify_cache.py`: collect `verify_cache.pkl` files and merge them into
  `verify_result.pkl`.
- `print_verify_result.py`: dump the results from the `verify_result.pkl` file.
- `collect_csv_files.py`: collect `PASS` .csv files into a folder.

# Visualization

- `explorer.py`: a simple explorer of viewing HouseExpoMap
- `play_trajectory.py`
- `visualizer.py`: used by `play_trajectory.py`
