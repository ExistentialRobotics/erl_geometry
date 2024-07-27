# import os.path
from erl_geometry.pyerl_geometry import HouseExpoMap
from . import list_data
from . import sequence
from . import uncompress_data

__all__ = [
    "HouseExpoMap",
    "list_data",
    "sequence",
    "uncompress_data",
]

# if not os.path.exists(list_data.get_map_dir()):
#     uncompress_data.uncompress_json_data()
#
# if not os.path.exists(list_data.get_traj_dir()):
#     uncompress_data.uncompress_traj_data()
