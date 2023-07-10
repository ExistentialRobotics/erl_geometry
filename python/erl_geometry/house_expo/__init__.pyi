from typing import overload

from . import list_data
from . import sequence
from . import uncompress_data
from .. import Space2D

__all__ = [
    "HouseExpoMap",
    "list_data",
    "sequence",
    "uncompress_data",
]

class HouseExpoMap:
    @overload
    def __init__(self: HouseExpoMap, file: str) -> None: ...
    @overload
    def __init__(self: HouseExpoMap, file: str, wall_thickness: float) -> None: ...
    @property
    def file(self: HouseExpoMap) -> str: ...
    @property
    def room_id(self: HouseExpoMap) -> str: ...
    @property
    def meter_space(self: HouseExpoMap) -> Space2D: ...
