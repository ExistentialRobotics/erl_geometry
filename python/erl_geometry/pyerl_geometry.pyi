import flags
import numpy
import torch
import typing
from typing import Callable, ClassVar, Iterator, overload

class Aabb2Dd:
    class CornerType:
        """Type of corners.

        Members:

          kBottomLeft

          kBottomRight

          kTopLeft

          kTopRight"""
        __members__: ClassVar[dict] = ...  # read-only
        __entries: ClassVar[dict] = ...
        kBottomLeft: ClassVar[Aabb2Dd.CornerType] = ...
        kBottomRight: ClassVar[Aabb2Dd.CornerType] = ...
        kTopLeft: ClassVar[Aabb2Dd.CornerType] = ...
        kTopRight: ClassVar[Aabb2Dd.CornerType] = ...
        def __init__(self, value: int) -> None:
            """__init__(self: erl_geometry.pyerl_geometry.Aabb2Dd.CornerType, value: int) -> None"""
        def __and__(self, other: object) -> object:
            """__and__(self: object, other: object) -> object"""
        def __eq__(self, other: object) -> bool:
            """__eq__(self: object, other: object) -> bool"""
        def __ge__(self, other: object) -> bool:
            """__ge__(self: object, other: object) -> bool"""
        def __gt__(self, other: object) -> bool:
            """__gt__(self: object, other: object) -> bool"""
        def __hash__(self) -> int:
            """__hash__(self: object) -> int"""
        def __index__(self) -> int:
            """__index__(self: erl_geometry.pyerl_geometry.Aabb2Dd.CornerType) -> int"""
        def __int__(self) -> int:
            """__int__(self: erl_geometry.pyerl_geometry.Aabb2Dd.CornerType) -> int"""
        def __invert__(self) -> object:
            """__invert__(self: object) -> object"""
        def __le__(self, other: object) -> bool:
            """__le__(self: object, other: object) -> bool"""
        def __lt__(self, other: object) -> bool:
            """__lt__(self: object, other: object) -> bool"""
        def __ne__(self, other: object) -> bool:
            """__ne__(self: object, other: object) -> bool"""
        def __or__(self, other: object) -> object:
            """__or__(self: object, other: object) -> object"""
        def __rand__(self, other: object) -> object:
            """__rand__(self: object, other: object) -> object"""
        def __ror__(self, other: object) -> object:
            """__ror__(self: object, other: object) -> object"""
        def __rxor__(self, other: object) -> object:
            """__rxor__(self: object, other: object) -> object"""
        def __xor__(self, other: object) -> object:
            """__xor__(self: object, other: object) -> object"""
        @property
        def name(self) -> str:
            """name(self: object) -> str

            name(self: object) -> str
            """
        @property
        def value(self) -> int:
            """(arg0: erl_geometry.pyerl_geometry.Aabb2Dd.CornerType) -> int"""
    kBottomLeft: ClassVar[Aabb2Dd.CornerType] = ...
    kBottomRight: ClassVar[Aabb2Dd.CornerType] = ...
    kTopLeft: ClassVar[Aabb2Dd.CornerType] = ...
    kTopRight: ClassVar[Aabb2Dd.CornerType] = ...
    @overload
    def __init__(self, center: numpy.ndarray[numpy.float64[2, 1]], half_size: float) -> None:
        """__init__(*args, **kwargs)
        Overloaded function.

        1. __init__(self: erl_geometry.pyerl_geometry.Aabb2Dd, center: numpy.ndarray[numpy.float64[2, 1]], half_size: float) -> None

        2. __init__(self: erl_geometry.pyerl_geometry.Aabb2Dd, min: numpy.ndarray[numpy.float64[2, 1]], max: numpy.ndarray[numpy.float64[2, 1]]) -> None
        """
    @overload
    def __init__(self, min: numpy.ndarray[numpy.float64[2, 1]], max: numpy.ndarray[numpy.float64[2, 1]]) -> None:
        """__init__(*args, **kwargs)
        Overloaded function.

        1. __init__(self: erl_geometry.pyerl_geometry.Aabb2Dd, center: numpy.ndarray[numpy.float64[2, 1]], half_size: float) -> None

        2. __init__(self: erl_geometry.pyerl_geometry.Aabb2Dd, min: numpy.ndarray[numpy.float64[2, 1]], max: numpy.ndarray[numpy.float64[2, 1]]) -> None
        """
    def corner(self, corner_type: Aabb2Dd.CornerType) -> numpy.ndarray[numpy.float64[2, 1]]:
        """corner(self: erl_geometry.pyerl_geometry.Aabb2Dd, corner_type: erl_geometry.pyerl_geometry.Aabb2Dd.CornerType) -> numpy.ndarray[numpy.float64[2, 1]]"""
    def intersects(self, *args, **kwargs):
        """intersects(self: erl_geometry.pyerl_geometry.Aabb2Dd, another_aabb: Eigen::AlignedBox<double, 2>) -> bool"""
    @overload
    def padding(self, padding: numpy.ndarray[numpy.float64[2, 1]]) -> Aabb2Dd:
        """padding(*args, **kwargs)
        Overloaded function.

        1. padding(self: erl_geometry.pyerl_geometry.Aabb2Dd, padding: numpy.ndarray[numpy.float64[2, 1]]) -> erl_geometry.pyerl_geometry.Aabb2Dd

        2. padding(self: erl_geometry.pyerl_geometry.Aabb2Dd, padding: float) -> erl_geometry.pyerl_geometry.Aabb2Dd
        """
    @overload
    def padding(self, padding: float) -> Aabb2Dd:
        """padding(*args, **kwargs)
        Overloaded function.

        1. padding(self: erl_geometry.pyerl_geometry.Aabb2Dd, padding: numpy.ndarray[numpy.float64[2, 1]]) -> erl_geometry.pyerl_geometry.Aabb2Dd

        2. padding(self: erl_geometry.pyerl_geometry.Aabb2Dd, padding: float) -> erl_geometry.pyerl_geometry.Aabb2Dd
        """
    @overload
    def __contains__(self, point: numpy.ndarray[numpy.float64[2, 1]]) -> bool:
        """__contains__(*args, **kwargs)
        Overloaded function.

        1. __contains__(self: erl_geometry.pyerl_geometry.Aabb2Dd, point: numpy.ndarray[numpy.float64[2, 1]]) -> bool

        2. __contains__(self: erl_geometry.pyerl_geometry.Aabb2Dd, another_aabb: erl_geometry.pyerl_geometry.Aabb2Dd) -> bool
        """
    @overload
    def __contains__(self, another_aabb: Aabb2Dd) -> bool:
        """__contains__(*args, **kwargs)
        Overloaded function.

        1. __contains__(self: erl_geometry.pyerl_geometry.Aabb2Dd, point: numpy.ndarray[numpy.float64[2, 1]]) -> bool

        2. __contains__(self: erl_geometry.pyerl_geometry.Aabb2Dd, another_aabb: erl_geometry.pyerl_geometry.Aabb2Dd) -> bool
        """
    @property
    def center(self) -> numpy.ndarray[numpy.float64[2, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.Aabb2Dd) -> numpy.ndarray[numpy.float64[2, 1]]"""
    @property
    def half_sizes(self) -> numpy.ndarray[numpy.float64[2, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.Aabb2Dd) -> numpy.ndarray[numpy.float64[2, 1]]"""
    @property
    def max(self) -> numpy.ndarray[numpy.float64[2, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.Aabb2Dd) -> numpy.ndarray[numpy.float64[2, 1]]"""
    @property
    def min(self) -> numpy.ndarray[numpy.float64[2, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.Aabb2Dd) -> numpy.ndarray[numpy.float64[2, 1]]"""

class Aabb2Df:
    class CornerType:
        """Type of corners.

        Members:

          kBottomLeft

          kBottomRight

          kTopLeft

          kTopRight"""
        __members__: ClassVar[dict] = ...  # read-only
        __entries: ClassVar[dict] = ...
        kBottomLeft: ClassVar[Aabb2Df.CornerType] = ...
        kBottomRight: ClassVar[Aabb2Df.CornerType] = ...
        kTopLeft: ClassVar[Aabb2Df.CornerType] = ...
        kTopRight: ClassVar[Aabb2Df.CornerType] = ...
        def __init__(self, value: int) -> None:
            """__init__(self: erl_geometry.pyerl_geometry.Aabb2Df.CornerType, value: int) -> None"""
        def __and__(self, other: object) -> object:
            """__and__(self: object, other: object) -> object"""
        def __eq__(self, other: object) -> bool:
            """__eq__(self: object, other: object) -> bool"""
        def __ge__(self, other: object) -> bool:
            """__ge__(self: object, other: object) -> bool"""
        def __gt__(self, other: object) -> bool:
            """__gt__(self: object, other: object) -> bool"""
        def __hash__(self) -> int:
            """__hash__(self: object) -> int"""
        def __index__(self) -> int:
            """__index__(self: erl_geometry.pyerl_geometry.Aabb2Df.CornerType) -> int"""
        def __int__(self) -> int:
            """__int__(self: erl_geometry.pyerl_geometry.Aabb2Df.CornerType) -> int"""
        def __invert__(self) -> object:
            """__invert__(self: object) -> object"""
        def __le__(self, other: object) -> bool:
            """__le__(self: object, other: object) -> bool"""
        def __lt__(self, other: object) -> bool:
            """__lt__(self: object, other: object) -> bool"""
        def __ne__(self, other: object) -> bool:
            """__ne__(self: object, other: object) -> bool"""
        def __or__(self, other: object) -> object:
            """__or__(self: object, other: object) -> object"""
        def __rand__(self, other: object) -> object:
            """__rand__(self: object, other: object) -> object"""
        def __ror__(self, other: object) -> object:
            """__ror__(self: object, other: object) -> object"""
        def __rxor__(self, other: object) -> object:
            """__rxor__(self: object, other: object) -> object"""
        def __xor__(self, other: object) -> object:
            """__xor__(self: object, other: object) -> object"""
        @property
        def name(self) -> str:
            """name(self: object) -> str

            name(self: object) -> str
            """
        @property
        def value(self) -> int:
            """(arg0: erl_geometry.pyerl_geometry.Aabb2Df.CornerType) -> int"""
    kBottomLeft: ClassVar[Aabb2Df.CornerType] = ...
    kBottomRight: ClassVar[Aabb2Df.CornerType] = ...
    kTopLeft: ClassVar[Aabb2Df.CornerType] = ...
    kTopRight: ClassVar[Aabb2Df.CornerType] = ...
    @overload
    def __init__(self, center: numpy.ndarray[numpy.float32[2, 1]], half_size: float) -> None:
        """__init__(*args, **kwargs)
        Overloaded function.

        1. __init__(self: erl_geometry.pyerl_geometry.Aabb2Df, center: numpy.ndarray[numpy.float32[2, 1]], half_size: float) -> None

        2. __init__(self: erl_geometry.pyerl_geometry.Aabb2Df, min: numpy.ndarray[numpy.float32[2, 1]], max: numpy.ndarray[numpy.float32[2, 1]]) -> None
        """
    @overload
    def __init__(self, min: numpy.ndarray[numpy.float32[2, 1]], max: numpy.ndarray[numpy.float32[2, 1]]) -> None:
        """__init__(*args, **kwargs)
        Overloaded function.

        1. __init__(self: erl_geometry.pyerl_geometry.Aabb2Df, center: numpy.ndarray[numpy.float32[2, 1]], half_size: float) -> None

        2. __init__(self: erl_geometry.pyerl_geometry.Aabb2Df, min: numpy.ndarray[numpy.float32[2, 1]], max: numpy.ndarray[numpy.float32[2, 1]]) -> None
        """
    def corner(self, corner_type: Aabb2Df.CornerType) -> numpy.ndarray[numpy.float32[2, 1]]:
        """corner(self: erl_geometry.pyerl_geometry.Aabb2Df, corner_type: erl_geometry.pyerl_geometry.Aabb2Df.CornerType) -> numpy.ndarray[numpy.float32[2, 1]]"""
    def intersects(self, *args, **kwargs):
        """intersects(self: erl_geometry.pyerl_geometry.Aabb2Df, another_aabb: Eigen::AlignedBox<float, 2>) -> bool"""
    @overload
    def padding(self, padding: numpy.ndarray[numpy.float32[2, 1]]) -> Aabb2Df:
        """padding(*args, **kwargs)
        Overloaded function.

        1. padding(self: erl_geometry.pyerl_geometry.Aabb2Df, padding: numpy.ndarray[numpy.float32[2, 1]]) -> erl_geometry.pyerl_geometry.Aabb2Df

        2. padding(self: erl_geometry.pyerl_geometry.Aabb2Df, padding: float) -> erl_geometry.pyerl_geometry.Aabb2Df
        """
    @overload
    def padding(self, padding: float) -> Aabb2Df:
        """padding(*args, **kwargs)
        Overloaded function.

        1. padding(self: erl_geometry.pyerl_geometry.Aabb2Df, padding: numpy.ndarray[numpy.float32[2, 1]]) -> erl_geometry.pyerl_geometry.Aabb2Df

        2. padding(self: erl_geometry.pyerl_geometry.Aabb2Df, padding: float) -> erl_geometry.pyerl_geometry.Aabb2Df
        """
    @overload
    def __contains__(self, point: numpy.ndarray[numpy.float32[2, 1]]) -> bool:
        """__contains__(*args, **kwargs)
        Overloaded function.

        1. __contains__(self: erl_geometry.pyerl_geometry.Aabb2Df, point: numpy.ndarray[numpy.float32[2, 1]]) -> bool

        2. __contains__(self: erl_geometry.pyerl_geometry.Aabb2Df, another_aabb: erl_geometry.pyerl_geometry.Aabb2Df) -> bool
        """
    @overload
    def __contains__(self, another_aabb: Aabb2Df) -> bool:
        """__contains__(*args, **kwargs)
        Overloaded function.

        1. __contains__(self: erl_geometry.pyerl_geometry.Aabb2Df, point: numpy.ndarray[numpy.float32[2, 1]]) -> bool

        2. __contains__(self: erl_geometry.pyerl_geometry.Aabb2Df, another_aabb: erl_geometry.pyerl_geometry.Aabb2Df) -> bool
        """
    @property
    def center(self) -> numpy.ndarray[numpy.float32[2, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.Aabb2Df) -> numpy.ndarray[numpy.float32[2, 1]]"""
    @property
    def half_sizes(self) -> numpy.ndarray[numpy.float32[2, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.Aabb2Df) -> numpy.ndarray[numpy.float32[2, 1]]"""
    @property
    def max(self) -> numpy.ndarray[numpy.float32[2, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.Aabb2Df) -> numpy.ndarray[numpy.float32[2, 1]]"""
    @property
    def min(self) -> numpy.ndarray[numpy.float32[2, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.Aabb2Df) -> numpy.ndarray[numpy.float32[2, 1]]"""

class Aabb3Dd:
    class CornerType:
        """Type of corners.

        Members:

          kBottomLeftFloor

          kBottomRightFloor

          kTopLeftFloor

          kTopRightFloor

          kBottomLeftCeil

          kBottomRightCeil

          kTopLeftCeil

          kTopRightCeil"""
        __members__: ClassVar[dict] = ...  # read-only
        __entries: ClassVar[dict] = ...
        kBottomLeftCeil: ClassVar[Aabb3Dd.CornerType] = ...
        kBottomLeftFloor: ClassVar[Aabb3Dd.CornerType] = ...
        kBottomRightCeil: ClassVar[Aabb3Dd.CornerType] = ...
        kBottomRightFloor: ClassVar[Aabb3Dd.CornerType] = ...
        kTopLeftCeil: ClassVar[Aabb3Dd.CornerType] = ...
        kTopLeftFloor: ClassVar[Aabb3Dd.CornerType] = ...
        kTopRightCeil: ClassVar[Aabb3Dd.CornerType] = ...
        kTopRightFloor: ClassVar[Aabb3Dd.CornerType] = ...
        def __init__(self, value: int) -> None:
            """__init__(self: erl_geometry.pyerl_geometry.Aabb3Dd.CornerType, value: int) -> None"""
        def __and__(self, other: object) -> object:
            """__and__(self: object, other: object) -> object"""
        def __eq__(self, other: object) -> bool:
            """__eq__(self: object, other: object) -> bool"""
        def __ge__(self, other: object) -> bool:
            """__ge__(self: object, other: object) -> bool"""
        def __gt__(self, other: object) -> bool:
            """__gt__(self: object, other: object) -> bool"""
        def __hash__(self) -> int:
            """__hash__(self: object) -> int"""
        def __index__(self) -> int:
            """__index__(self: erl_geometry.pyerl_geometry.Aabb3Dd.CornerType) -> int"""
        def __int__(self) -> int:
            """__int__(self: erl_geometry.pyerl_geometry.Aabb3Dd.CornerType) -> int"""
        def __invert__(self) -> object:
            """__invert__(self: object) -> object"""
        def __le__(self, other: object) -> bool:
            """__le__(self: object, other: object) -> bool"""
        def __lt__(self, other: object) -> bool:
            """__lt__(self: object, other: object) -> bool"""
        def __ne__(self, other: object) -> bool:
            """__ne__(self: object, other: object) -> bool"""
        def __or__(self, other: object) -> object:
            """__or__(self: object, other: object) -> object"""
        def __rand__(self, other: object) -> object:
            """__rand__(self: object, other: object) -> object"""
        def __ror__(self, other: object) -> object:
            """__ror__(self: object, other: object) -> object"""
        def __rxor__(self, other: object) -> object:
            """__rxor__(self: object, other: object) -> object"""
        def __xor__(self, other: object) -> object:
            """__xor__(self: object, other: object) -> object"""
        @property
        def name(self) -> str:
            """name(self: object) -> str

            name(self: object) -> str
            """
        @property
        def value(self) -> int:
            """(arg0: erl_geometry.pyerl_geometry.Aabb3Dd.CornerType) -> int"""
    kBottomLeftCeil: ClassVar[Aabb3Dd.CornerType] = ...
    kBottomLeftFloor: ClassVar[Aabb3Dd.CornerType] = ...
    kBottomRightCeil: ClassVar[Aabb3Dd.CornerType] = ...
    kBottomRightFloor: ClassVar[Aabb3Dd.CornerType] = ...
    kTopLeftCeil: ClassVar[Aabb3Dd.CornerType] = ...
    kTopLeftFloor: ClassVar[Aabb3Dd.CornerType] = ...
    kTopRightCeil: ClassVar[Aabb3Dd.CornerType] = ...
    kTopRightFloor: ClassVar[Aabb3Dd.CornerType] = ...
    @overload
    def __init__(self, center: numpy.ndarray[numpy.float64[3, 1]], half_size: float) -> None:
        """__init__(*args, **kwargs)
        Overloaded function.

        1. __init__(self: erl_geometry.pyerl_geometry.Aabb3Dd, center: numpy.ndarray[numpy.float64[3, 1]], half_size: float) -> None

        2. __init__(self: erl_geometry.pyerl_geometry.Aabb3Dd, min: numpy.ndarray[numpy.float64[3, 1]], max: numpy.ndarray[numpy.float64[3, 1]]) -> None
        """
    @overload
    def __init__(self, min: numpy.ndarray[numpy.float64[3, 1]], max: numpy.ndarray[numpy.float64[3, 1]]) -> None:
        """__init__(*args, **kwargs)
        Overloaded function.

        1. __init__(self: erl_geometry.pyerl_geometry.Aabb3Dd, center: numpy.ndarray[numpy.float64[3, 1]], half_size: float) -> None

        2. __init__(self: erl_geometry.pyerl_geometry.Aabb3Dd, min: numpy.ndarray[numpy.float64[3, 1]], max: numpy.ndarray[numpy.float64[3, 1]]) -> None
        """
    def corner(self, corner_type: Aabb3Dd.CornerType) -> numpy.ndarray[numpy.float64[3, 1]]:
        """corner(self: erl_geometry.pyerl_geometry.Aabb3Dd, corner_type: erl_geometry.pyerl_geometry.Aabb3Dd.CornerType) -> numpy.ndarray[numpy.float64[3, 1]]"""
    def intersects(self, *args, **kwargs):
        """intersects(self: erl_geometry.pyerl_geometry.Aabb3Dd, another_aabb: Eigen::AlignedBox<double, 3>) -> bool"""
    @overload
    def padding(self, padding: numpy.ndarray[numpy.float64[3, 1]]) -> Aabb3Dd:
        """padding(*args, **kwargs)
        Overloaded function.

        1. padding(self: erl_geometry.pyerl_geometry.Aabb3Dd, padding: numpy.ndarray[numpy.float64[3, 1]]) -> erl_geometry.pyerl_geometry.Aabb3Dd

        2. padding(self: erl_geometry.pyerl_geometry.Aabb3Dd, padding: float) -> erl_geometry.pyerl_geometry.Aabb3Dd
        """
    @overload
    def padding(self, padding: float) -> Aabb3Dd:
        """padding(*args, **kwargs)
        Overloaded function.

        1. padding(self: erl_geometry.pyerl_geometry.Aabb3Dd, padding: numpy.ndarray[numpy.float64[3, 1]]) -> erl_geometry.pyerl_geometry.Aabb3Dd

        2. padding(self: erl_geometry.pyerl_geometry.Aabb3Dd, padding: float) -> erl_geometry.pyerl_geometry.Aabb3Dd
        """
    @overload
    def __contains__(self, point: numpy.ndarray[numpy.float64[3, 1]]) -> bool:
        """__contains__(*args, **kwargs)
        Overloaded function.

        1. __contains__(self: erl_geometry.pyerl_geometry.Aabb3Dd, point: numpy.ndarray[numpy.float64[3, 1]]) -> bool

        2. __contains__(self: erl_geometry.pyerl_geometry.Aabb3Dd, another_aabb: erl_geometry.pyerl_geometry.Aabb3Dd) -> bool
        """
    @overload
    def __contains__(self, another_aabb: Aabb3Dd) -> bool:
        """__contains__(*args, **kwargs)
        Overloaded function.

        1. __contains__(self: erl_geometry.pyerl_geometry.Aabb3Dd, point: numpy.ndarray[numpy.float64[3, 1]]) -> bool

        2. __contains__(self: erl_geometry.pyerl_geometry.Aabb3Dd, another_aabb: erl_geometry.pyerl_geometry.Aabb3Dd) -> bool
        """
    @property
    def center(self) -> numpy.ndarray[numpy.float64[3, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.Aabb3Dd) -> numpy.ndarray[numpy.float64[3, 1]]"""
    @property
    def half_sizes(self) -> numpy.ndarray[numpy.float64[3, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.Aabb3Dd) -> numpy.ndarray[numpy.float64[3, 1]]"""
    @property
    def max(self) -> numpy.ndarray[numpy.float64[3, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.Aabb3Dd) -> numpy.ndarray[numpy.float64[3, 1]]"""
    @property
    def min(self) -> numpy.ndarray[numpy.float64[3, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.Aabb3Dd) -> numpy.ndarray[numpy.float64[3, 1]]"""

class Aabb3Df:
    class CornerType:
        """Type of corners.

        Members:

          kBottomLeftFloor

          kBottomRightFloor

          kTopLeftFloor

          kTopRightFloor

          kBottomLeftCeil

          kBottomRightCeil

          kTopLeftCeil

          kTopRightCeil"""
        __members__: ClassVar[dict] = ...  # read-only
        __entries: ClassVar[dict] = ...
        kBottomLeftCeil: ClassVar[Aabb3Df.CornerType] = ...
        kBottomLeftFloor: ClassVar[Aabb3Df.CornerType] = ...
        kBottomRightCeil: ClassVar[Aabb3Df.CornerType] = ...
        kBottomRightFloor: ClassVar[Aabb3Df.CornerType] = ...
        kTopLeftCeil: ClassVar[Aabb3Df.CornerType] = ...
        kTopLeftFloor: ClassVar[Aabb3Df.CornerType] = ...
        kTopRightCeil: ClassVar[Aabb3Df.CornerType] = ...
        kTopRightFloor: ClassVar[Aabb3Df.CornerType] = ...
        def __init__(self, value: int) -> None:
            """__init__(self: erl_geometry.pyerl_geometry.Aabb3Df.CornerType, value: int) -> None"""
        def __and__(self, other: object) -> object:
            """__and__(self: object, other: object) -> object"""
        def __eq__(self, other: object) -> bool:
            """__eq__(self: object, other: object) -> bool"""
        def __ge__(self, other: object) -> bool:
            """__ge__(self: object, other: object) -> bool"""
        def __gt__(self, other: object) -> bool:
            """__gt__(self: object, other: object) -> bool"""
        def __hash__(self) -> int:
            """__hash__(self: object) -> int"""
        def __index__(self) -> int:
            """__index__(self: erl_geometry.pyerl_geometry.Aabb3Df.CornerType) -> int"""
        def __int__(self) -> int:
            """__int__(self: erl_geometry.pyerl_geometry.Aabb3Df.CornerType) -> int"""
        def __invert__(self) -> object:
            """__invert__(self: object) -> object"""
        def __le__(self, other: object) -> bool:
            """__le__(self: object, other: object) -> bool"""
        def __lt__(self, other: object) -> bool:
            """__lt__(self: object, other: object) -> bool"""
        def __ne__(self, other: object) -> bool:
            """__ne__(self: object, other: object) -> bool"""
        def __or__(self, other: object) -> object:
            """__or__(self: object, other: object) -> object"""
        def __rand__(self, other: object) -> object:
            """__rand__(self: object, other: object) -> object"""
        def __ror__(self, other: object) -> object:
            """__ror__(self: object, other: object) -> object"""
        def __rxor__(self, other: object) -> object:
            """__rxor__(self: object, other: object) -> object"""
        def __xor__(self, other: object) -> object:
            """__xor__(self: object, other: object) -> object"""
        @property
        def name(self) -> str:
            """name(self: object) -> str

            name(self: object) -> str
            """
        @property
        def value(self) -> int:
            """(arg0: erl_geometry.pyerl_geometry.Aabb3Df.CornerType) -> int"""
    kBottomLeftCeil: ClassVar[Aabb3Df.CornerType] = ...
    kBottomLeftFloor: ClassVar[Aabb3Df.CornerType] = ...
    kBottomRightCeil: ClassVar[Aabb3Df.CornerType] = ...
    kBottomRightFloor: ClassVar[Aabb3Df.CornerType] = ...
    kTopLeftCeil: ClassVar[Aabb3Df.CornerType] = ...
    kTopLeftFloor: ClassVar[Aabb3Df.CornerType] = ...
    kTopRightCeil: ClassVar[Aabb3Df.CornerType] = ...
    kTopRightFloor: ClassVar[Aabb3Df.CornerType] = ...
    @overload
    def __init__(self, center: numpy.ndarray[numpy.float32[3, 1]], half_size: float) -> None:
        """__init__(*args, **kwargs)
        Overloaded function.

        1. __init__(self: erl_geometry.pyerl_geometry.Aabb3Df, center: numpy.ndarray[numpy.float32[3, 1]], half_size: float) -> None

        2. __init__(self: erl_geometry.pyerl_geometry.Aabb3Df, min: numpy.ndarray[numpy.float32[3, 1]], max: numpy.ndarray[numpy.float32[3, 1]]) -> None
        """
    @overload
    def __init__(self, min: numpy.ndarray[numpy.float32[3, 1]], max: numpy.ndarray[numpy.float32[3, 1]]) -> None:
        """__init__(*args, **kwargs)
        Overloaded function.

        1. __init__(self: erl_geometry.pyerl_geometry.Aabb3Df, center: numpy.ndarray[numpy.float32[3, 1]], half_size: float) -> None

        2. __init__(self: erl_geometry.pyerl_geometry.Aabb3Df, min: numpy.ndarray[numpy.float32[3, 1]], max: numpy.ndarray[numpy.float32[3, 1]]) -> None
        """
    def corner(self, corner_type: Aabb3Df.CornerType) -> numpy.ndarray[numpy.float32[3, 1]]:
        """corner(self: erl_geometry.pyerl_geometry.Aabb3Df, corner_type: erl_geometry.pyerl_geometry.Aabb3Df.CornerType) -> numpy.ndarray[numpy.float32[3, 1]]"""
    def intersects(self, *args, **kwargs):
        """intersects(self: erl_geometry.pyerl_geometry.Aabb3Df, another_aabb: Eigen::AlignedBox<float, 3>) -> bool"""
    @overload
    def padding(self, padding: numpy.ndarray[numpy.float32[3, 1]]) -> Aabb3Df:
        """padding(*args, **kwargs)
        Overloaded function.

        1. padding(self: erl_geometry.pyerl_geometry.Aabb3Df, padding: numpy.ndarray[numpy.float32[3, 1]]) -> erl_geometry.pyerl_geometry.Aabb3Df

        2. padding(self: erl_geometry.pyerl_geometry.Aabb3Df, padding: float) -> erl_geometry.pyerl_geometry.Aabb3Df
        """
    @overload
    def padding(self, padding: float) -> Aabb3Df:
        """padding(*args, **kwargs)
        Overloaded function.

        1. padding(self: erl_geometry.pyerl_geometry.Aabb3Df, padding: numpy.ndarray[numpy.float32[3, 1]]) -> erl_geometry.pyerl_geometry.Aabb3Df

        2. padding(self: erl_geometry.pyerl_geometry.Aabb3Df, padding: float) -> erl_geometry.pyerl_geometry.Aabb3Df
        """
    @overload
    def __contains__(self, point: numpy.ndarray[numpy.float32[3, 1]]) -> bool:
        """__contains__(*args, **kwargs)
        Overloaded function.

        1. __contains__(self: erl_geometry.pyerl_geometry.Aabb3Df, point: numpy.ndarray[numpy.float32[3, 1]]) -> bool

        2. __contains__(self: erl_geometry.pyerl_geometry.Aabb3Df, another_aabb: erl_geometry.pyerl_geometry.Aabb3Df) -> bool
        """
    @overload
    def __contains__(self, another_aabb: Aabb3Df) -> bool:
        """__contains__(*args, **kwargs)
        Overloaded function.

        1. __contains__(self: erl_geometry.pyerl_geometry.Aabb3Df, point: numpy.ndarray[numpy.float32[3, 1]]) -> bool

        2. __contains__(self: erl_geometry.pyerl_geometry.Aabb3Df, another_aabb: erl_geometry.pyerl_geometry.Aabb3Df) -> bool
        """
    @property
    def center(self) -> numpy.ndarray[numpy.float32[3, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.Aabb3Df) -> numpy.ndarray[numpy.float32[3, 1]]"""
    @property
    def half_sizes(self) -> numpy.ndarray[numpy.float32[3, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.Aabb3Df) -> numpy.ndarray[numpy.float32[3, 1]]"""
    @property
    def max(self) -> numpy.ndarray[numpy.float32[3, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.Aabb3Df) -> numpy.ndarray[numpy.float32[3, 1]]"""
    @property
    def min(self) -> numpy.ndarray[numpy.float32[3, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.Aabb3Df) -> numpy.ndarray[numpy.float32[3, 1]]"""

class AbstractOccupancyOctreeD(AbstractOctreeD):
    def __init__(self, *args, **kwargs) -> None:
        """Initialize self.  See help(type(self)) for accurate signature."""
    def get_hit_occupied_node(self, px: float, py: float, pz: float, vx: float, vy: float, vz: float, ignore_unknown: bool, max_range: float) -> dict:
        """get_hit_occupied_node(self: erl_geometry.pyerl_geometry.AbstractOccupancyOctreeD, px: float, py: float, pz: float, vx: float, vy: float, vz: float, ignore_unknown: bool, max_range: float) -> dict"""
    def is_node_at_threshold(self, node: OccupancyOctreeNode) -> bool:
        """is_node_at_threshold(self: erl_geometry.pyerl_geometry.AbstractOccupancyOctreeD, node: erl_geometry.pyerl_geometry.OccupancyOctreeNode) -> bool"""
    def is_node_occupied(self, node: OccupancyOctreeNode) -> bool:
        """is_node_occupied(self: erl_geometry.pyerl_geometry.AbstractOccupancyOctreeD, node: erl_geometry.pyerl_geometry.OccupancyOctreeNode) -> bool"""
    def read_binary(self, filename: str) -> bool:
        """read_binary(self: erl_geometry.pyerl_geometry.AbstractOccupancyOctreeD, filename: str) -> bool"""
    def write_binary(self, filename: str, prune_at_first: bool) -> bool:
        """write_binary(self: erl_geometry.pyerl_geometry.AbstractOccupancyOctreeD, filename: str, prune_at_first: bool) -> bool"""

class AbstractOccupancyOctreeF(AbstractOctreeF):
    def __init__(self, *args, **kwargs) -> None:
        """Initialize self.  See help(type(self)) for accurate signature."""
    def get_hit_occupied_node(self, px: float, py: float, pz: float, vx: float, vy: float, vz: float, ignore_unknown: bool, max_range: float) -> dict:
        """get_hit_occupied_node(self: erl_geometry.pyerl_geometry.AbstractOccupancyOctreeF, px: float, py: float, pz: float, vx: float, vy: float, vz: float, ignore_unknown: bool, max_range: float) -> dict"""
    def is_node_at_threshold(self, node: OccupancyOctreeNode) -> bool:
        """is_node_at_threshold(self: erl_geometry.pyerl_geometry.AbstractOccupancyOctreeF, node: erl_geometry.pyerl_geometry.OccupancyOctreeNode) -> bool"""
    def is_node_occupied(self, node: OccupancyOctreeNode) -> bool:
        """is_node_occupied(self: erl_geometry.pyerl_geometry.AbstractOccupancyOctreeF, node: erl_geometry.pyerl_geometry.OccupancyOctreeNode) -> bool"""
    def read_binary(self, filename: str) -> bool:
        """read_binary(self: erl_geometry.pyerl_geometry.AbstractOccupancyOctreeF, filename: str) -> bool"""
    def write_binary(self, filename: str, prune_at_first: bool) -> bool:
        """write_binary(self: erl_geometry.pyerl_geometry.AbstractOccupancyOctreeF, filename: str, prune_at_first: bool) -> bool"""

class AbstractOccupancyQuadtreeD(AbstractQuadtreeD):
    def __init__(self, *args, **kwargs) -> None:
        """Initialize self.  See help(type(self)) for accurate signature."""
    def get_hit_occupied_node(self, px: float, py: float, vx: float, vy: float, ignore_unknown: bool, max_range: float) -> dict:
        """get_hit_occupied_node(self: erl_geometry.pyerl_geometry.AbstractOccupancyQuadtreeD, px: float, py: float, vx: float, vy: float, ignore_unknown: bool, max_range: float) -> dict"""
    def is_node_at_threshold(self, node: OccupancyQuadtreeNode) -> bool:
        """is_node_at_threshold(self: erl_geometry.pyerl_geometry.AbstractOccupancyQuadtreeD, node: erl_geometry.pyerl_geometry.OccupancyQuadtreeNode) -> bool"""
    def is_node_occupied(self, node: OccupancyQuadtreeNode) -> bool:
        """is_node_occupied(self: erl_geometry.pyerl_geometry.AbstractOccupancyQuadtreeD, node: erl_geometry.pyerl_geometry.OccupancyQuadtreeNode) -> bool"""
    def read_binary(self, filename: str) -> bool:
        """read_binary(self: erl_geometry.pyerl_geometry.AbstractOccupancyQuadtreeD, filename: str) -> bool"""
    def write_binary(self, filename: str, prune_at_first: bool) -> bool:
        """write_binary(self: erl_geometry.pyerl_geometry.AbstractOccupancyQuadtreeD, filename: str, prune_at_first: bool) -> bool"""

class AbstractOccupancyQuadtreeF(AbstractQuadtreeF):
    def __init__(self, *args, **kwargs) -> None:
        """Initialize self.  See help(type(self)) for accurate signature."""
    def get_hit_occupied_node(self, px: float, py: float, vx: float, vy: float, ignore_unknown: bool, max_range: float) -> dict:
        """get_hit_occupied_node(self: erl_geometry.pyerl_geometry.AbstractOccupancyQuadtreeF, px: float, py: float, vx: float, vy: float, ignore_unknown: bool, max_range: float) -> dict"""
    def is_node_at_threshold(self, node: OccupancyQuadtreeNode) -> bool:
        """is_node_at_threshold(self: erl_geometry.pyerl_geometry.AbstractOccupancyQuadtreeF, node: erl_geometry.pyerl_geometry.OccupancyQuadtreeNode) -> bool"""
    def is_node_occupied(self, node: OccupancyQuadtreeNode) -> bool:
        """is_node_occupied(self: erl_geometry.pyerl_geometry.AbstractOccupancyQuadtreeF, node: erl_geometry.pyerl_geometry.OccupancyQuadtreeNode) -> bool"""
    def read_binary(self, filename: str) -> bool:
        """read_binary(self: erl_geometry.pyerl_geometry.AbstractOccupancyQuadtreeF, filename: str) -> bool"""
    def write_binary(self, filename: str, prune_at_first: bool) -> bool:
        """write_binary(self: erl_geometry.pyerl_geometry.AbstractOccupancyQuadtreeF, filename: str, prune_at_first: bool) -> bool"""

class AbstractOctreeD:
    class OctreeNodeIterator:
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""
        def get_index_key(self) -> OctreeKey:
            """get_index_key(self: erl_geometry.pyerl_geometry.AbstractOctreeD.OctreeNodeIterator) -> erl_geometry.pyerl_geometry.OctreeKey"""
        def get_key(self) -> OctreeKey:
            """get_key(self: erl_geometry.pyerl_geometry.AbstractOctreeD.OctreeNodeIterator) -> erl_geometry.pyerl_geometry.OctreeKey"""
        def get_node(self) -> AbstractOctreeNode:
            """get_node(self: erl_geometry.pyerl_geometry.AbstractOctreeD.OctreeNodeIterator) -> erl_geometry.pyerl_geometry.AbstractOctreeNode"""
        def next(self) -> None:
            """next(self: erl_geometry.pyerl_geometry.AbstractOctreeD.OctreeNodeIterator) -> None"""
        @property
        def depth(self) -> int:
            """(arg0: erl_geometry.pyerl_geometry.AbstractOctreeD.OctreeNodeIterator) -> int"""
        @property
        def is_valid(self) -> bool:
            """(arg0: erl_geometry.pyerl_geometry.AbstractOctreeD.OctreeNodeIterator) -> bool"""
        @property
        def node_size(self) -> float:
            """(arg0: erl_geometry.pyerl_geometry.AbstractOctreeD.OctreeNodeIterator) -> float"""
        @property
        def x(self) -> float:
            """(arg0: erl_geometry.pyerl_geometry.AbstractOctreeD.OctreeNodeIterator) -> float"""
        @property
        def y(self) -> float:
            """(arg0: erl_geometry.pyerl_geometry.AbstractOctreeD.OctreeNodeIterator) -> float"""
        @property
        def z(self) -> float:
            """(arg0: erl_geometry.pyerl_geometry.AbstractOctreeD.OctreeNodeIterator) -> float"""
    def __init__(self, *args, **kwargs) -> None:
        """Initialize self.  See help(type(self)) for accurate signature."""
    def apply_setting(self) -> None:
        """apply_setting(self: erl_geometry.pyerl_geometry.AbstractOctreeD) -> None"""
    def read(self, filename: str) -> bool:
        """read(self: erl_geometry.pyerl_geometry.AbstractOctreeD, filename: str) -> bool"""
    def read_setting(self, arg0) -> bool:
        """read_setting(self: erl_geometry.pyerl_geometry.AbstractOctreeD, arg0: std::istream) -> bool"""
    @overload
    def search_node(self, x: float, y: float, z: float, max_depth: int) -> AbstractOctreeNode:
        """search_node(*args, **kwargs)
        Overloaded function.

        1. search_node(self: erl_geometry.pyerl_geometry.AbstractOctreeD, x: float, y: float, z: float, max_depth: int) -> erl_geometry.pyerl_geometry.AbstractOctreeNode

        2. search_node(self: erl_geometry.pyerl_geometry.AbstractOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, max_depth: int) -> erl_geometry.pyerl_geometry.AbstractOctreeNode
        """
    @overload
    def search_node(self, key: OctreeKey, max_depth: int) -> AbstractOctreeNode:
        """search_node(*args, **kwargs)
        Overloaded function.

        1. search_node(self: erl_geometry.pyerl_geometry.AbstractOctreeD, x: float, y: float, z: float, max_depth: int) -> erl_geometry.pyerl_geometry.AbstractOctreeNode

        2. search_node(self: erl_geometry.pyerl_geometry.AbstractOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, max_depth: int) -> erl_geometry.pyerl_geometry.AbstractOctreeNode
        """
    def write(self, filename: str) -> bool:
        """write(self: erl_geometry.pyerl_geometry.AbstractOctreeD, filename: str) -> bool"""
    def write_setting(self, arg0) -> None:
        """write_setting(self: erl_geometry.pyerl_geometry.AbstractOctreeD, arg0: std::ostream) -> None"""
    @property
    def tree_type(self) -> str:
        """(arg0: erl_geometry.pyerl_geometry.AbstractOctreeD) -> str"""

class AbstractOctreeF:
    class OctreeNodeIterator:
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""
        def get_index_key(self) -> OctreeKey:
            """get_index_key(self: erl_geometry.pyerl_geometry.AbstractOctreeF.OctreeNodeIterator) -> erl_geometry.pyerl_geometry.OctreeKey"""
        def get_key(self) -> OctreeKey:
            """get_key(self: erl_geometry.pyerl_geometry.AbstractOctreeF.OctreeNodeIterator) -> erl_geometry.pyerl_geometry.OctreeKey"""
        def get_node(self) -> AbstractOctreeNode:
            """get_node(self: erl_geometry.pyerl_geometry.AbstractOctreeF.OctreeNodeIterator) -> erl_geometry.pyerl_geometry.AbstractOctreeNode"""
        def next(self) -> None:
            """next(self: erl_geometry.pyerl_geometry.AbstractOctreeF.OctreeNodeIterator) -> None"""
        @property
        def depth(self) -> int:
            """(arg0: erl_geometry.pyerl_geometry.AbstractOctreeF.OctreeNodeIterator) -> int"""
        @property
        def is_valid(self) -> bool:
            """(arg0: erl_geometry.pyerl_geometry.AbstractOctreeF.OctreeNodeIterator) -> bool"""
        @property
        def node_size(self) -> float:
            """(arg0: erl_geometry.pyerl_geometry.AbstractOctreeF.OctreeNodeIterator) -> float"""
        @property
        def x(self) -> float:
            """(arg0: erl_geometry.pyerl_geometry.AbstractOctreeF.OctreeNodeIterator) -> float"""
        @property
        def y(self) -> float:
            """(arg0: erl_geometry.pyerl_geometry.AbstractOctreeF.OctreeNodeIterator) -> float"""
        @property
        def z(self) -> float:
            """(arg0: erl_geometry.pyerl_geometry.AbstractOctreeF.OctreeNodeIterator) -> float"""
    def __init__(self, *args, **kwargs) -> None:
        """Initialize self.  See help(type(self)) for accurate signature."""
    def apply_setting(self) -> None:
        """apply_setting(self: erl_geometry.pyerl_geometry.AbstractOctreeF) -> None"""
    def read(self, filename: str) -> bool:
        """read(self: erl_geometry.pyerl_geometry.AbstractOctreeF, filename: str) -> bool"""
    def read_setting(self, arg0) -> bool:
        """read_setting(self: erl_geometry.pyerl_geometry.AbstractOctreeF, arg0: std::istream) -> bool"""
    @overload
    def search_node(self, x: float, y: float, z: float, max_depth: int) -> AbstractOctreeNode:
        """search_node(*args, **kwargs)
        Overloaded function.

        1. search_node(self: erl_geometry.pyerl_geometry.AbstractOctreeF, x: float, y: float, z: float, max_depth: int) -> erl_geometry.pyerl_geometry.AbstractOctreeNode

        2. search_node(self: erl_geometry.pyerl_geometry.AbstractOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, max_depth: int) -> erl_geometry.pyerl_geometry.AbstractOctreeNode
        """
    @overload
    def search_node(self, key: OctreeKey, max_depth: int) -> AbstractOctreeNode:
        """search_node(*args, **kwargs)
        Overloaded function.

        1. search_node(self: erl_geometry.pyerl_geometry.AbstractOctreeF, x: float, y: float, z: float, max_depth: int) -> erl_geometry.pyerl_geometry.AbstractOctreeNode

        2. search_node(self: erl_geometry.pyerl_geometry.AbstractOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, max_depth: int) -> erl_geometry.pyerl_geometry.AbstractOctreeNode
        """
    def write(self, filename: str) -> bool:
        """write(self: erl_geometry.pyerl_geometry.AbstractOctreeF, filename: str) -> bool"""
    def write_setting(self, arg0) -> None:
        """write_setting(self: erl_geometry.pyerl_geometry.AbstractOctreeF, arg0: std::ostream) -> None"""
    @property
    def tree_type(self) -> str:
        """(arg0: erl_geometry.pyerl_geometry.AbstractOctreeF) -> str"""

class AbstractOctreeNode:
    def __init__(self, *args, **kwargs) -> None:
        """Initialize self.  See help(type(self)) for accurate signature."""
    def has_child(self, child_idx: int) -> bool:
        """has_child(self: erl_geometry.pyerl_geometry.AbstractOctreeNode, child_idx: int) -> bool"""
    @property
    def child_index(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.AbstractOctreeNode) -> int"""
    @property
    def depth(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.AbstractOctreeNode) -> int"""
    @property
    def has_any_child(self) -> bool:
        """(arg0: erl_geometry.pyerl_geometry.AbstractOctreeNode) -> bool"""
    @property
    def node_type(self) -> str:
        """(arg0: erl_geometry.pyerl_geometry.AbstractOctreeNode) -> str"""
    @property
    def num_children(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.AbstractOctreeNode) -> int"""

class AbstractQuadtreeD:
    class QuadtreeNodeIterator:
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""
        def get_index_key(self) -> QuadtreeKey:
            """get_index_key(self: erl_geometry.pyerl_geometry.AbstractQuadtreeD.QuadtreeNodeIterator) -> erl_geometry.pyerl_geometry.QuadtreeKey"""
        def get_key(self) -> QuadtreeKey:
            """get_key(self: erl_geometry.pyerl_geometry.AbstractQuadtreeD.QuadtreeNodeIterator) -> erl_geometry.pyerl_geometry.QuadtreeKey"""
        def get_node(self) -> AbstractQuadtreeNode:
            """get_node(self: erl_geometry.pyerl_geometry.AbstractQuadtreeD.QuadtreeNodeIterator) -> erl_geometry.pyerl_geometry.AbstractQuadtreeNode"""
        def next(self) -> None:
            """next(self: erl_geometry.pyerl_geometry.AbstractQuadtreeD.QuadtreeNodeIterator) -> None"""
        @property
        def depth(self) -> int:
            """(arg0: erl_geometry.pyerl_geometry.AbstractQuadtreeD.QuadtreeNodeIterator) -> int"""
        @property
        def is_valid(self) -> bool:
            """(arg0: erl_geometry.pyerl_geometry.AbstractQuadtreeD.QuadtreeNodeIterator) -> bool"""
        @property
        def node_size(self) -> float:
            """(arg0: erl_geometry.pyerl_geometry.AbstractQuadtreeD.QuadtreeNodeIterator) -> float"""
        @property
        def x(self) -> float:
            """(arg0: erl_geometry.pyerl_geometry.AbstractQuadtreeD.QuadtreeNodeIterator) -> float"""
        @property
        def y(self) -> float:
            """(arg0: erl_geometry.pyerl_geometry.AbstractQuadtreeD.QuadtreeNodeIterator) -> float"""
    def __init__(self, *args, **kwargs) -> None:
        """Initialize self.  See help(type(self)) for accurate signature."""
    def apply_setting(self) -> None:
        """apply_setting(self: erl_geometry.pyerl_geometry.AbstractQuadtreeD) -> None"""
    def read(self, filename: str) -> bool:
        """read(self: erl_geometry.pyerl_geometry.AbstractQuadtreeD, filename: str) -> bool"""
    def read_setting(self, arg0) -> bool:
        """read_setting(self: erl_geometry.pyerl_geometry.AbstractQuadtreeD, arg0: std::istream) -> bool"""
    @overload
    def search_node(self, x: float, y: float, max_depth: int) -> AbstractQuadtreeNode:
        """search_node(*args, **kwargs)
        Overloaded function.

        1. search_node(self: erl_geometry.pyerl_geometry.AbstractQuadtreeD, x: float, y: float, max_depth: int) -> erl_geometry.pyerl_geometry.AbstractQuadtreeNode

        2. search_node(self: erl_geometry.pyerl_geometry.AbstractQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, max_depth: int) -> erl_geometry.pyerl_geometry.AbstractQuadtreeNode
        """
    @overload
    def search_node(self, key: QuadtreeKey, max_depth: int) -> AbstractQuadtreeNode:
        """search_node(*args, **kwargs)
        Overloaded function.

        1. search_node(self: erl_geometry.pyerl_geometry.AbstractQuadtreeD, x: float, y: float, max_depth: int) -> erl_geometry.pyerl_geometry.AbstractQuadtreeNode

        2. search_node(self: erl_geometry.pyerl_geometry.AbstractQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, max_depth: int) -> erl_geometry.pyerl_geometry.AbstractQuadtreeNode
        """
    def write(self, filename: str) -> bool:
        """write(self: erl_geometry.pyerl_geometry.AbstractQuadtreeD, filename: str) -> bool"""
    def write_setting(self, arg0) -> None:
        """write_setting(self: erl_geometry.pyerl_geometry.AbstractQuadtreeD, arg0: std::ostream) -> None"""
    @property
    def tree_type(self) -> str:
        """(arg0: erl_geometry.pyerl_geometry.AbstractQuadtreeD) -> str"""

class AbstractQuadtreeF:
    class QuadtreeNodeIterator:
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""
        def get_index_key(self) -> QuadtreeKey:
            """get_index_key(self: erl_geometry.pyerl_geometry.AbstractQuadtreeF.QuadtreeNodeIterator) -> erl_geometry.pyerl_geometry.QuadtreeKey"""
        def get_key(self) -> QuadtreeKey:
            """get_key(self: erl_geometry.pyerl_geometry.AbstractQuadtreeF.QuadtreeNodeIterator) -> erl_geometry.pyerl_geometry.QuadtreeKey"""
        def get_node(self) -> AbstractQuadtreeNode:
            """get_node(self: erl_geometry.pyerl_geometry.AbstractQuadtreeF.QuadtreeNodeIterator) -> erl_geometry.pyerl_geometry.AbstractQuadtreeNode"""
        def next(self) -> None:
            """next(self: erl_geometry.pyerl_geometry.AbstractQuadtreeF.QuadtreeNodeIterator) -> None"""
        @property
        def depth(self) -> int:
            """(arg0: erl_geometry.pyerl_geometry.AbstractQuadtreeF.QuadtreeNodeIterator) -> int"""
        @property
        def is_valid(self) -> bool:
            """(arg0: erl_geometry.pyerl_geometry.AbstractQuadtreeF.QuadtreeNodeIterator) -> bool"""
        @property
        def node_size(self) -> float:
            """(arg0: erl_geometry.pyerl_geometry.AbstractQuadtreeF.QuadtreeNodeIterator) -> float"""
        @property
        def x(self) -> float:
            """(arg0: erl_geometry.pyerl_geometry.AbstractQuadtreeF.QuadtreeNodeIterator) -> float"""
        @property
        def y(self) -> float:
            """(arg0: erl_geometry.pyerl_geometry.AbstractQuadtreeF.QuadtreeNodeIterator) -> float"""
    def __init__(self, *args, **kwargs) -> None:
        """Initialize self.  See help(type(self)) for accurate signature."""
    def apply_setting(self) -> None:
        """apply_setting(self: erl_geometry.pyerl_geometry.AbstractQuadtreeF) -> None"""
    def read(self, filename: str) -> bool:
        """read(self: erl_geometry.pyerl_geometry.AbstractQuadtreeF, filename: str) -> bool"""
    def read_setting(self, arg0) -> bool:
        """read_setting(self: erl_geometry.pyerl_geometry.AbstractQuadtreeF, arg0: std::istream) -> bool"""
    @overload
    def search_node(self, x: float, y: float, max_depth: int) -> AbstractQuadtreeNode:
        """search_node(*args, **kwargs)
        Overloaded function.

        1. search_node(self: erl_geometry.pyerl_geometry.AbstractQuadtreeF, x: float, y: float, max_depth: int) -> erl_geometry.pyerl_geometry.AbstractQuadtreeNode

        2. search_node(self: erl_geometry.pyerl_geometry.AbstractQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, max_depth: int) -> erl_geometry.pyerl_geometry.AbstractQuadtreeNode
        """
    @overload
    def search_node(self, key: QuadtreeKey, max_depth: int) -> AbstractQuadtreeNode:
        """search_node(*args, **kwargs)
        Overloaded function.

        1. search_node(self: erl_geometry.pyerl_geometry.AbstractQuadtreeF, x: float, y: float, max_depth: int) -> erl_geometry.pyerl_geometry.AbstractQuadtreeNode

        2. search_node(self: erl_geometry.pyerl_geometry.AbstractQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, max_depth: int) -> erl_geometry.pyerl_geometry.AbstractQuadtreeNode
        """
    def write(self, filename: str) -> bool:
        """write(self: erl_geometry.pyerl_geometry.AbstractQuadtreeF, filename: str) -> bool"""
    def write_setting(self, arg0) -> None:
        """write_setting(self: erl_geometry.pyerl_geometry.AbstractQuadtreeF, arg0: std::ostream) -> None"""
    @property
    def tree_type(self) -> str:
        """(arg0: erl_geometry.pyerl_geometry.AbstractQuadtreeF) -> str"""

class AbstractQuadtreeNode:
    def __init__(self, *args, **kwargs) -> None:
        """Initialize self.  See help(type(self)) for accurate signature."""
    def has_child(self, child_idx: int) -> bool:
        """has_child(self: erl_geometry.pyerl_geometry.AbstractQuadtreeNode, child_idx: int) -> bool"""
    @property
    def child_index(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.AbstractQuadtreeNode) -> int"""
    @property
    def depth(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.AbstractQuadtreeNode) -> int"""
    @property
    def has_any_child(self) -> bool:
        """(arg0: erl_geometry.pyerl_geometry.AbstractQuadtreeNode) -> bool"""
    @property
    def node_type(self) -> str:
        """(arg0: erl_geometry.pyerl_geometry.AbstractQuadtreeNode) -> str"""
    @property
    def num_children(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.AbstractQuadtreeNode) -> int"""

class AxisAlignedRectangle2D(Primitive2D, Aabb2Dd):
    def __init__(self, id: int, center: numpy.ndarray[numpy.float64[2, 1]], half_sizes: numpy.ndarray[numpy.float64[2, 1]]) -> None:
        """__init__(self: erl_geometry.pyerl_geometry.AxisAlignedRectangle2D, id: int, center: numpy.ndarray[numpy.float64[2, 1]], half_sizes: numpy.ndarray[numpy.float64[2, 1]]) -> None"""

class Box(Primitive3D):
    center: numpy.ndarray[numpy.float64[3, 1]]
    rotation_matrix: numpy.ndarray[numpy.float64[3, 3]]
    def __init__(self, id: int, center: numpy.ndarray[numpy.float64[3, 1]], half_sizes: numpy.ndarray[numpy.float64[3, 1]], rotation: numpy.ndarray[numpy.float64[3, 3]]) -> None:
        """__init__(self: erl_geometry.pyerl_geometry.Box, id: int, center: numpy.ndarray[numpy.float64[3, 1]], half_sizes: numpy.ndarray[numpy.float64[3, 1]], rotation: numpy.ndarray[numpy.float64[3, 3]]) -> None"""
    def rotate(self, rotation: numpy.ndarray[numpy.float64[3, 3]]) -> Box:
        """rotate(self: erl_geometry.pyerl_geometry.Box, rotation: numpy.ndarray[numpy.float64[3, 3]]) -> erl_geometry.pyerl_geometry.Box"""
    def translate(self, translation: numpy.ndarray[numpy.float64[3, 1]]) -> Box:
        """translate(self: erl_geometry.pyerl_geometry.Box, translation: numpy.ndarray[numpy.float64[3, 1]]) -> erl_geometry.pyerl_geometry.Box"""
    @property
    def half_sizes(self) -> numpy.ndarray[numpy.float64[3, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.Box) -> numpy.ndarray[numpy.float64[3, 1]]"""

class CameraBase3Dd:
    cTo: ClassVar[numpy.ndarray[numpy.float64[4, 4]]] = ...  # read-only
    oTc: ClassVar[numpy.ndarray[numpy.float64[4, 4]]] = ...  # read-only
    def __init__(self, *args, **kwargs) -> None:
        """Initialize self.  See help(type(self)) for accurate signature."""
    @staticmethod
    def compute_camera_pose(orientation: numpy.ndarray[numpy.float64[3, 3], flags.f_contiguous], translation: numpy.ndarray[numpy.float64[3, 1]]) -> tuple[numpy.ndarray[numpy.float64[3, 3]], numpy.ndarray[numpy.float64[3, 1]]]:
        """compute_camera_pose(orientation: numpy.ndarray[numpy.float64[3, 3], flags.f_contiguous], translation: numpy.ndarray[numpy.float64[3, 1]]) -> tuple[numpy.ndarray[numpy.float64[3, 3]], numpy.ndarray[numpy.float64[3, 1]]]"""
    @staticmethod
    def compute_extrinsic(camera_orientation: numpy.ndarray[numpy.float64[3, 3], flags.f_contiguous], camera_translation: numpy.ndarray[numpy.float64[3, 1]]) -> numpy.ndarray[numpy.float64[4, 4]]:
        """compute_extrinsic(camera_orientation: numpy.ndarray[numpy.float64[3, 3], flags.f_contiguous], camera_translation: numpy.ndarray[numpy.float64[3, 1]]) -> numpy.ndarray[numpy.float64[4, 4]]"""
    @staticmethod
    def compute_optical_pose(orientation: numpy.ndarray[numpy.float64[3, 3], flags.f_contiguous], translation: numpy.ndarray[numpy.float64[3, 1]]) -> tuple[numpy.ndarray[numpy.float64[3, 3]], numpy.ndarray[numpy.float64[3, 1]]]:
        """compute_optical_pose(orientation: numpy.ndarray[numpy.float64[3, 3], flags.f_contiguous], translation: numpy.ndarray[numpy.float64[3, 1]]) -> tuple[numpy.ndarray[numpy.float64[3, 3]], numpy.ndarray[numpy.float64[3, 1]]]"""

class CameraBase3Df:
    cTo: ClassVar[numpy.ndarray[numpy.float32[4, 4]]] = ...  # read-only
    oTc: ClassVar[numpy.ndarray[numpy.float32[4, 4]]] = ...  # read-only
    def __init__(self, *args, **kwargs) -> None:
        """Initialize self.  See help(type(self)) for accurate signature."""
    @staticmethod
    def compute_camera_pose(orientation: numpy.ndarray[numpy.float32[3, 3], flags.f_contiguous], translation: numpy.ndarray[numpy.float32[3, 1]]) -> tuple[numpy.ndarray[numpy.float32[3, 3]], numpy.ndarray[numpy.float32[3, 1]]]:
        """compute_camera_pose(orientation: numpy.ndarray[numpy.float32[3, 3], flags.f_contiguous], translation: numpy.ndarray[numpy.float32[3, 1]]) -> tuple[numpy.ndarray[numpy.float32[3, 3]], numpy.ndarray[numpy.float32[3, 1]]]"""
    @staticmethod
    def compute_extrinsic(camera_orientation: numpy.ndarray[numpy.float32[3, 3], flags.f_contiguous], camera_translation: numpy.ndarray[numpy.float32[3, 1]]) -> numpy.ndarray[numpy.float32[4, 4]]:
        """compute_extrinsic(camera_orientation: numpy.ndarray[numpy.float32[3, 3], flags.f_contiguous], camera_translation: numpy.ndarray[numpy.float32[3, 1]]) -> numpy.ndarray[numpy.float32[4, 4]]"""
    @staticmethod
    def compute_optical_pose(orientation: numpy.ndarray[numpy.float32[3, 3], flags.f_contiguous], translation: numpy.ndarray[numpy.float32[3, 1]]) -> tuple[numpy.ndarray[numpy.float32[3, 3]], numpy.ndarray[numpy.float32[3, 1]]]:
        """compute_optical_pose(orientation: numpy.ndarray[numpy.float32[3, 3], flags.f_contiguous], translation: numpy.ndarray[numpy.float32[3, 1]]) -> tuple[numpy.ndarray[numpy.float32[3, 3]], numpy.ndarray[numpy.float32[3, 1]]]"""

class CameraIntrinsicD(YamlableBase):
    camera_cx: float
    camera_cy: float
    camera_fx: float
    camera_fy: float
    image_height: int
    image_width: int
    def __init__(self) -> None:
        """__init__(self: erl_geometry.pyerl_geometry.CameraIntrinsicD) -> None"""
    def compute_frame_direction(self, u: int, v: int) -> tuple[float, float, float]:
        """compute_frame_direction(self: erl_geometry.pyerl_geometry.CameraIntrinsicD, u: int, v: int) -> tuple[float, float, float]"""
    def compute_frame_directions(self) -> numpy.ndarray[numpy.float64]:
        """compute_frame_directions(self: erl_geometry.pyerl_geometry.CameraIntrinsicD) -> numpy.ndarray[numpy.float64]"""
    def convert_depth_to_distance(self, depth: numpy.ndarray[numpy.float64[m, n]], rgb: Mat, optical_pose: numpy.ndarray[numpy.float64[4, 4]] | None = ...) -> dict:
        """convert_depth_to_distance(self: erl_geometry.pyerl_geometry.CameraIntrinsicD, depth: numpy.ndarray[numpy.float64[m, n]], rgb: Mat, optical_pose: Optional[numpy.ndarray[numpy.float64[4, 4]]] = None) -> dict"""
    def resize(self, factor: float) -> tuple[int, int]:
        """resize(self: erl_geometry.pyerl_geometry.CameraIntrinsicD, factor: float) -> tuple[int, int]"""
    @property
    def matrix(self) -> numpy.ndarray[numpy.float64[3, 3]]:
        """(arg0: erl_geometry.pyerl_geometry.CameraIntrinsicD) -> numpy.ndarray[numpy.float64[3, 3]]"""

class CameraIntrinsicF(YamlableBase):
    camera_cx: float
    camera_cy: float
    camera_fx: float
    camera_fy: float
    image_height: int
    image_width: int
    def __init__(self) -> None:
        """__init__(self: erl_geometry.pyerl_geometry.CameraIntrinsicF) -> None"""
    def compute_frame_direction(self, u: int, v: int) -> tuple[float, float, float]:
        """compute_frame_direction(self: erl_geometry.pyerl_geometry.CameraIntrinsicF, u: int, v: int) -> tuple[float, float, float]"""
    def compute_frame_directions(self) -> numpy.ndarray[numpy.float32]:
        """compute_frame_directions(self: erl_geometry.pyerl_geometry.CameraIntrinsicF) -> numpy.ndarray[numpy.float32]"""
    def convert_depth_to_distance(self, depth: numpy.ndarray[numpy.float32[m, n]], rgb: Mat, optical_pose: numpy.ndarray[numpy.float32[4, 4]] | None = ...) -> dict:
        """convert_depth_to_distance(self: erl_geometry.pyerl_geometry.CameraIntrinsicF, depth: numpy.ndarray[numpy.float32[m, n]], rgb: Mat, optical_pose: Optional[numpy.ndarray[numpy.float32[4, 4]]] = None) -> dict"""
    def resize(self, factor: float) -> tuple[int, int]:
        """resize(self: erl_geometry.pyerl_geometry.CameraIntrinsicF, factor: float) -> tuple[int, int]"""
    @property
    def matrix(self) -> numpy.ndarray[numpy.float32[3, 3]]:
        """(arg0: erl_geometry.pyerl_geometry.CameraIntrinsicF) -> numpy.ndarray[numpy.float32[3, 3]]"""

class CityStreetMap:
    class Scene:
        bucket: int
        goal_x: int
        goal_y: int
        map: str
        map_height: int
        map_width: int
        optimal_length: float
        start_x: int
        start_y: int
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""
    kFree: ClassVar[int] = ...  # read-only
    kObstacle: ClassVar[int] = ...  # read-only
    kOutOfBoundAt: ClassVar[str] = ...  # read-only
    kOutOfBoundO: ClassVar[str] = ...  # read-only
    kPassableDot: ClassVar[str] = ...  # read-only
    kPassableG: ClassVar[str] = ...  # read-only
    kSwamp: ClassVar[str] = ...  # read-only
    kTree: ClassVar[str] = ...  # read-only
    kWater: ClassVar[str] = ...  # read-only
    def __init__(self, *args, **kwargs) -> None:
        """Initialize self.  See help(type(self)) for accurate signature."""
    @staticmethod
    def load_map(filename: str) -> Mat:
        """load_map(filename: str) -> Mat"""
    @staticmethod
    def load_scenes(*args, **kwargs):
        """load_scenes(filename: str) -> list[erl::geometry::CityStreetMap::Scene]"""

class DepthCamera3Dd(CameraBase3Dd, RangeSensor3Dd):
    def __init__(self, setting: CameraIntrinsicD) -> None:
        """__init__(self: erl_geometry.pyerl_geometry.DepthCamera3Dd, setting: erl_geometry.pyerl_geometry.CameraIntrinsicD) -> None"""
    def Setting(self) -> CameraIntrinsicD:
        """Setting() -> erl_geometry.pyerl_geometry.CameraIntrinsicD"""
    @property
    def setting(self) -> CameraIntrinsicD:
        """(arg0: erl_geometry.pyerl_geometry.DepthCamera3Dd) -> erl_geometry.pyerl_geometry.CameraIntrinsicD"""

class DepthCamera3Df(CameraBase3Df, RangeSensor3Df):
    def __init__(self, setting: CameraIntrinsicF) -> None:
        """__init__(self: erl_geometry.pyerl_geometry.DepthCamera3Df, setting: erl_geometry.pyerl_geometry.CameraIntrinsicF) -> None"""
    def Setting(self) -> CameraIntrinsicF:
        """Setting() -> erl_geometry.pyerl_geometry.CameraIntrinsicF"""
    @property
    def setting(self) -> CameraIntrinsicF:
        """(arg0: erl_geometry.pyerl_geometry.DepthCamera3Df) -> erl_geometry.pyerl_geometry.CameraIntrinsicF"""

class DepthFrame3Dd(RangeSensorFrame3Dd):
    class Setting(RangeSensorFrame3Dd.Setting):
        camera_intrinsic: CameraIntrinsicD
        def __init__(self) -> None:
            """__init__(self: erl_geometry.pyerl_geometry.DepthFrame3Dd.Setting) -> None"""
    def __init__(self, setting: DepthFrame3Dd.Setting) -> None:
        """__init__(self: erl_geometry.pyerl_geometry.DepthFrame3Dd, setting: erl_geometry.pyerl_geometry.DepthFrame3Dd.Setting) -> None"""
    @staticmethod
    def depth_image_to_depth(depth_image: numpy.ndarray[numpy.float64[m, n]], depth_scale: float) -> numpy.ndarray[numpy.float64[m, n]]:
        """depth_image_to_depth(depth_image: numpy.ndarray[numpy.float64[m, n]], depth_scale: float) -> numpy.ndarray[numpy.float64[m, n]]"""
    @staticmethod
    def depth_to_depth_image(depth: numpy.ndarray[numpy.float64[m, n]], depth_scale: float) -> numpy.ndarray[numpy.float64[m, n]]:
        """depth_to_depth_image(depth: numpy.ndarray[numpy.float64[m, n]], depth_scale: float) -> numpy.ndarray[numpy.float64[m, n]]"""
    def reset(self) -> None:
        """reset(self: erl_geometry.pyerl_geometry.DepthFrame3Dd) -> None"""
    @overload
    def update_ranges(self, rotation: numpy.ndarray[numpy.float64[3, 3], flags.f_contiguous], translation: numpy.ndarray[numpy.float64[3, 1]], depth: numpy.ndarray[numpy.float64[m, n]]) -> None:
        """update_ranges(*args, **kwargs)
        Overloaded function.

        1. update_ranges(self: erl_geometry.pyerl_geometry.DepthFrame3Dd, rotation: numpy.ndarray[numpy.float64[3, 3], flags.f_contiguous], translation: numpy.ndarray[numpy.float64[3, 1]], depth: numpy.ndarray[numpy.float64[m, n]]) -> None

        2. update_ranges(self: erl_geometry.pyerl_geometry.DepthFrame3Dd, rotation: numpy.ndarray[numpy.float64[3, 3], flags.f_contiguous], translation: numpy.ndarray[numpy.float64[3, 1]], depth_file: str, depth_scale: float) -> None
        """
    @overload
    def update_ranges(self, rotation: numpy.ndarray[numpy.float64[3, 3], flags.f_contiguous], translation: numpy.ndarray[numpy.float64[3, 1]], depth_file: str, depth_scale: float) -> None:
        """update_ranges(*args, **kwargs)
        Overloaded function.

        1. update_ranges(self: erl_geometry.pyerl_geometry.DepthFrame3Dd, rotation: numpy.ndarray[numpy.float64[3, 3], flags.f_contiguous], translation: numpy.ndarray[numpy.float64[3, 1]], depth: numpy.ndarray[numpy.float64[m, n]]) -> None

        2. update_ranges(self: erl_geometry.pyerl_geometry.DepthFrame3Dd, rotation: numpy.ndarray[numpy.float64[3, 3], flags.f_contiguous], translation: numpy.ndarray[numpy.float64[3, 1]], depth_file: str, depth_scale: float) -> None
        """
    @property
    def image_height(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.DepthFrame3Dd) -> int"""
    @property
    def image_width(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.DepthFrame3Dd) -> int"""
    @property
    def setting(self) -> DepthFrame3Dd.Setting:
        """(arg0: erl_geometry.pyerl_geometry.DepthFrame3Dd) -> erl_geometry.pyerl_geometry.DepthFrame3Dd.Setting"""

class DepthFrame3Df(RangeSensorFrame3Df):
    class Setting(RangeSensorFrame3Df.Setting):
        camera_intrinsic: CameraIntrinsicF
        def __init__(self) -> None:
            """__init__(self: erl_geometry.pyerl_geometry.DepthFrame3Df.Setting) -> None"""
    def __init__(self, setting: DepthFrame3Df.Setting) -> None:
        """__init__(self: erl_geometry.pyerl_geometry.DepthFrame3Df, setting: erl_geometry.pyerl_geometry.DepthFrame3Df.Setting) -> None"""
    @staticmethod
    def depth_image_to_depth(depth_image: numpy.ndarray[numpy.float32[m, n]], depth_scale: float) -> numpy.ndarray[numpy.float32[m, n]]:
        """depth_image_to_depth(depth_image: numpy.ndarray[numpy.float32[m, n]], depth_scale: float) -> numpy.ndarray[numpy.float32[m, n]]"""
    @staticmethod
    def depth_to_depth_image(depth: numpy.ndarray[numpy.float32[m, n]], depth_scale: float) -> numpy.ndarray[numpy.float32[m, n]]:
        """depth_to_depth_image(depth: numpy.ndarray[numpy.float32[m, n]], depth_scale: float) -> numpy.ndarray[numpy.float32[m, n]]"""
    def reset(self) -> None:
        """reset(self: erl_geometry.pyerl_geometry.DepthFrame3Df) -> None"""
    @overload
    def update_ranges(self, rotation: numpy.ndarray[numpy.float32[3, 3], flags.f_contiguous], translation: numpy.ndarray[numpy.float32[3, 1]], depth: numpy.ndarray[numpy.float32[m, n]]) -> None:
        """update_ranges(*args, **kwargs)
        Overloaded function.

        1. update_ranges(self: erl_geometry.pyerl_geometry.DepthFrame3Df, rotation: numpy.ndarray[numpy.float32[3, 3], flags.f_contiguous], translation: numpy.ndarray[numpy.float32[3, 1]], depth: numpy.ndarray[numpy.float32[m, n]]) -> None

        2. update_ranges(self: erl_geometry.pyerl_geometry.DepthFrame3Df, rotation: numpy.ndarray[numpy.float32[3, 3], flags.f_contiguous], translation: numpy.ndarray[numpy.float32[3, 1]], depth_file: str, depth_scale: float) -> None
        """
    @overload
    def update_ranges(self, rotation: numpy.ndarray[numpy.float32[3, 3], flags.f_contiguous], translation: numpy.ndarray[numpy.float32[3, 1]], depth_file: str, depth_scale: float) -> None:
        """update_ranges(*args, **kwargs)
        Overloaded function.

        1. update_ranges(self: erl_geometry.pyerl_geometry.DepthFrame3Df, rotation: numpy.ndarray[numpy.float32[3, 3], flags.f_contiguous], translation: numpy.ndarray[numpy.float32[3, 1]], depth: numpy.ndarray[numpy.float32[m, n]]) -> None

        2. update_ranges(self: erl_geometry.pyerl_geometry.DepthFrame3Df, rotation: numpy.ndarray[numpy.float32[3, 3], flags.f_contiguous], translation: numpy.ndarray[numpy.float32[3, 1]], depth_file: str, depth_scale: float) -> None
        """
    @property
    def image_height(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.DepthFrame3Df) -> int"""
    @property
    def image_width(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.DepthFrame3Df) -> int"""
    @property
    def setting(self) -> DepthFrame3Df.Setting:
        """(arg0: erl_geometry.pyerl_geometry.DepthFrame3Df) -> erl_geometry.pyerl_geometry.DepthFrame3Df.Setting"""

class Ellipse2D(Primitive2D):
    center: numpy.ndarray[numpy.float64[2, 1]]
    orientation_angle: float
    def __init__(self, id: int, center: numpy.ndarray[numpy.float64[2, 1]], a: float, b: float, angle: float) -> None:
        """__init__(self: erl_geometry.pyerl_geometry.Ellipse2D, id: int, center: numpy.ndarray[numpy.float64[2, 1]], a: float, b: float, angle: float) -> None"""
    def compute_points_on_boundary(self, num_points: int, start_angle: float, end_angle: float) -> list[numpy.ndarray[numpy.float64[2, 1]]]:
        """compute_points_on_boundary(self: erl_geometry.pyerl_geometry.Ellipse2D, num_points: int, start_angle: float, end_angle: float) -> list[numpy.ndarray[numpy.float64[2, 1]]]"""
    def rotate(self, angle: float) -> Ellipse2D:
        """rotate(self: erl_geometry.pyerl_geometry.Ellipse2D, angle: float) -> erl_geometry.pyerl_geometry.Ellipse2D"""
    def translate(self, translation: numpy.ndarray[numpy.float64[2, 1]]) -> Ellipse2D:
        """translate(self: erl_geometry.pyerl_geometry.Ellipse2D, translation: numpy.ndarray[numpy.float64[2, 1]]) -> erl_geometry.pyerl_geometry.Ellipse2D"""
    @property
    def radii(self) -> numpy.ndarray[numpy.float64[2, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.Ellipse2D) -> numpy.ndarray[numpy.float64[2, 1]]"""
    @property
    def rotation_matrix(self) -> numpy.ndarray[numpy.float64[2, 2]]:
        """(arg0: erl_geometry.pyerl_geometry.Ellipse2D) -> numpy.ndarray[numpy.float64[2, 2]]"""

class Ellipsoid(Primitive3D):
    center: numpy.ndarray[numpy.float64[3, 1]]
    rotation_matrix: numpy.ndarray[numpy.float64[3, 3]]
    def __init__(self, id: int, center: numpy.ndarray[numpy.float64[3, 1]], radius: numpy.ndarray[numpy.float64[3, 1]], rotation: numpy.ndarray[numpy.float64[3, 3]]) -> None:
        """__init__(self: erl_geometry.pyerl_geometry.Ellipsoid, id: int, center: numpy.ndarray[numpy.float64[3, 1]], radius: numpy.ndarray[numpy.float64[3, 1]], rotation: numpy.ndarray[numpy.float64[3, 3]]) -> None"""
    def rotate(self, rotation: numpy.ndarray[numpy.float64[3, 3]]) -> Ellipsoid:
        """rotate(self: erl_geometry.pyerl_geometry.Ellipsoid, rotation: numpy.ndarray[numpy.float64[3, 3]]) -> erl_geometry.pyerl_geometry.Ellipsoid"""
    def translate(self, translation: numpy.ndarray[numpy.float64[3, 1]]) -> Ellipsoid:
        """translate(self: erl_geometry.pyerl_geometry.Ellipsoid, translation: numpy.ndarray[numpy.float64[3, 1]]) -> erl_geometry.pyerl_geometry.Ellipsoid"""
    @property
    def radii(self) -> numpy.ndarray[numpy.float64[3, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.Ellipsoid) -> numpy.ndarray[numpy.float64[3, 1]]"""

class HouseExpoMap:
    @overload
    def __init__(self, file: str) -> None:
        """__init__(*args, **kwargs)
        Overloaded function.

        1. __init__(self: erl_geometry.pyerl_geometry.HouseExpoMap, file: str) -> None

        2. __init__(self: erl_geometry.pyerl_geometry.HouseExpoMap, file: str, wall_thickness: float) -> None
        """
    @overload
    def __init__(self, file: str, wall_thickness: float) -> None:
        """__init__(*args, **kwargs)
        Overloaded function.

        1. __init__(self: erl_geometry.pyerl_geometry.HouseExpoMap, file: str) -> None

        2. __init__(self: erl_geometry.pyerl_geometry.HouseExpoMap, file: str, wall_thickness: float) -> None
        """
    def extrude_to_3d(self, room_height: float, filename: str) -> None:
        """extrude_to_3d(self: erl_geometry.pyerl_geometry.HouseExpoMap, room_height: float, filename: str) -> None"""
    def to_json(self) -> str:
        """to_json(self: erl_geometry.pyerl_geometry.HouseExpoMap) -> str"""
    @property
    def file(self) -> str:
        """(arg0: erl_geometry.pyerl_geometry.HouseExpoMap) -> str"""
    @property
    def meter_space(self) -> Space2D:
        """(arg0: erl_geometry.pyerl_geometry.HouseExpoMap) -> erl_geometry.pyerl_geometry.Space2D"""
    @property
    def room_id(self) -> str:
        """(arg0: erl_geometry.pyerl_geometry.HouseExpoMap) -> str"""

class Lidar2D:
    class Mode:
        """Mode of directed distance.

        Members:

          kDdf : Compute unsigned directed distance.

          kSddfV1 : Compute signed directed distance, version 1.

          kSddfV2 : Compute signed directed distance, version 2."""
        __members__: ClassVar[dict] = ...  # read-only
        __entries: ClassVar[dict] = ...
        kDdf: ClassVar[Lidar2D.Mode] = ...
        kSddfV1: ClassVar[Lidar2D.Mode] = ...
        kSddfV2: ClassVar[Lidar2D.Mode] = ...
        def __init__(self, value: int) -> None:
            """__init__(self: erl_geometry.pyerl_geometry.Lidar2D.Mode, value: int) -> None"""
        def __eq__(self, other: object) -> bool:
            """__eq__(self: object, other: object) -> bool"""
        def __ge__(self, other: object) -> bool:
            """__ge__(self: object, other: object) -> bool"""
        def __gt__(self, other: object) -> bool:
            """__gt__(self: object, other: object) -> bool"""
        def __hash__(self) -> int:
            """__hash__(self: object) -> int"""
        def __index__(self) -> int:
            """__index__(self: erl_geometry.pyerl_geometry.Lidar2D.Mode) -> int"""
        def __int__(self) -> int:
            """__int__(self: erl_geometry.pyerl_geometry.Lidar2D.Mode) -> int"""
        def __le__(self, other: object) -> bool:
            """__le__(self: object, other: object) -> bool"""
        def __lt__(self, other: object) -> bool:
            """__lt__(self: object, other: object) -> bool"""
        def __ne__(self, other: object) -> bool:
            """__ne__(self: object, other: object) -> bool"""
        @property
        def name(self) -> str:
            """name(self: object) -> str

            name(self: object) -> str
            """
        @property
        def value(self) -> int:
            """(arg0: erl_geometry.pyerl_geometry.Lidar2D.Mode) -> int"""

    class Setting(YamlableBase):
        max_angle: float
        min_angle: float
        mode: Lidar2D.Mode
        num_lines: int
        sign_method: Space2D.SignMethod
        def __init__(self) -> None:
            """__init__(self: erl_geometry.pyerl_geometry.Lidar2D.Setting) -> None"""
    kDdf: ClassVar[Lidar2D.Mode] = ...
    kSddfV1: ClassVar[Lidar2D.Mode] = ...
    kSddfV2: ClassVar[Lidar2D.Mode] = ...
    def __init__(self, setting: Lidar2D.Setting, space2d: Space2D) -> None:
        """__init__(self: erl_geometry.pyerl_geometry.Lidar2D, setting: erl_geometry.pyerl_geometry.Lidar2D.Setting, space2d: erl_geometry.pyerl_geometry.Space2D) -> None"""
    @overload
    def scan(self, rotation_angle: float, translation: numpy.ndarray[numpy.float64[2, 1]], parallel: bool = ...) -> numpy.ndarray[numpy.float64[m, 1]]:
        """scan(*args, **kwargs)
        Overloaded function.

        1. scan(self: erl_geometry.pyerl_geometry.Lidar2D, rotation_angle: float, translation: numpy.ndarray[numpy.float64[2, 1]], parallel: bool = False) -> numpy.ndarray[numpy.float64[m, 1]]

        2. scan(self: erl_geometry.pyerl_geometry.Lidar2D, rotation: numpy.ndarray[numpy.float64[2, 2], flags.f_contiguous], translation: numpy.ndarray[numpy.float64[2, 1]], parallel: bool = False) -> numpy.ndarray[numpy.float64[m, 1]]
        """
    @overload
    def scan(self, rotation: numpy.ndarray[numpy.float64[2, 2], flags.f_contiguous], translation: numpy.ndarray[numpy.float64[2, 1]], parallel: bool = ...) -> numpy.ndarray[numpy.float64[m, 1]]:
        """scan(*args, **kwargs)
        Overloaded function.

        1. scan(self: erl_geometry.pyerl_geometry.Lidar2D, rotation_angle: float, translation: numpy.ndarray[numpy.float64[2, 1]], parallel: bool = False) -> numpy.ndarray[numpy.float64[m, 1]]

        2. scan(self: erl_geometry.pyerl_geometry.Lidar2D, rotation: numpy.ndarray[numpy.float64[2, 2], flags.f_contiguous], translation: numpy.ndarray[numpy.float64[2, 1]], parallel: bool = False) -> numpy.ndarray[numpy.float64[m, 1]]
        """
    def scan_multi_poses(self, poses: list[numpy.ndarray[numpy.float64[3, 3]]], parallel: bool = ...) -> list[numpy.ndarray[numpy.float64[m, 1]]]:
        """scan_multi_poses(self: erl_geometry.pyerl_geometry.Lidar2D, poses: list[numpy.ndarray[numpy.float64[3, 3]]], parallel: bool = False) -> list[numpy.ndarray[numpy.float64[m, 1]]]"""
    @property
    def angles(self) -> numpy.ndarray[numpy.float64[m, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.Lidar2D) -> numpy.ndarray[numpy.float64[m, 1]]"""
    @property
    def ray_directions_in_frame(self) -> numpy.ndarray[numpy.float64[2, n]]:
        """(arg0: erl_geometry.pyerl_geometry.Lidar2D) -> numpy.ndarray[numpy.float64[2, n]]"""
    @property
    def setting(self) -> Lidar2D.Setting:
        """(arg0: erl_geometry.pyerl_geometry.Lidar2D) -> erl_geometry.pyerl_geometry.Lidar2D.Setting"""

class Lidar3Dd(RangeSensor3Dd):
    class Setting(YamlableBase):
        azimuth_max: float
        azimuth_min: float
        elevation_max: float
        elevation_min: float
        num_azimuth_lines: int
        num_elevation_lines: int
        def __init__(self) -> None:
            """__init__(self: erl_geometry.pyerl_geometry.Lidar3Dd.Setting) -> None"""
    def __init__(self, setting: Lidar3Dd.Setting) -> None:
        """__init__(self: erl_geometry.pyerl_geometry.Lidar3Dd, setting: erl_geometry.pyerl_geometry.Lidar3Dd.Setting) -> None"""
    @property
    def azimuth_angles(self) -> numpy.ndarray[numpy.float64[m, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.Lidar3Dd) -> numpy.ndarray[numpy.float64[m, 1]]"""
    @property
    def elevation_angles(self) -> numpy.ndarray[numpy.float64[m, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.Lidar3Dd) -> numpy.ndarray[numpy.float64[m, 1]]"""
    @property
    def setting(self) -> Lidar3Dd.Setting:
        """(arg0: erl_geometry.pyerl_geometry.Lidar3Dd) -> erl_geometry.pyerl_geometry.Lidar3Dd.Setting"""

class Lidar3Df(RangeSensor3Df):
    class Setting(YamlableBase):
        azimuth_max: float
        azimuth_min: float
        elevation_max: float
        elevation_min: float
        num_azimuth_lines: int
        num_elevation_lines: int
        def __init__(self) -> None:
            """__init__(self: erl_geometry.pyerl_geometry.Lidar3Df.Setting) -> None"""
    def __init__(self, setting: Lidar3Df.Setting) -> None:
        """__init__(self: erl_geometry.pyerl_geometry.Lidar3Df, setting: erl_geometry.pyerl_geometry.Lidar3Df.Setting) -> None"""
    @property
    def azimuth_angles(self) -> numpy.ndarray[numpy.float32[m, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.Lidar3Df) -> numpy.ndarray[numpy.float32[m, 1]]"""
    @property
    def elevation_angles(self) -> numpy.ndarray[numpy.float32[m, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.Lidar3Df) -> numpy.ndarray[numpy.float32[m, 1]]"""
    @property
    def setting(self) -> Lidar3Df.Setting:
        """(arg0: erl_geometry.pyerl_geometry.Lidar3Df) -> erl_geometry.pyerl_geometry.Lidar3Df.Setting"""

class LidarFrame2Dd:
    class Partition:
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""
        def angle_in_partition(self, angle_world: float) -> bool:
            """angle_in_partition(self: erl_geometry.pyerl_geometry.LidarFrame2Dd.Partition, angle_world: float) -> bool"""
        @property
        def index_begin(self) -> int:
            """(arg0: erl_geometry.pyerl_geometry.LidarFrame2Dd.Partition) -> int"""
        @property
        def index_end(self) -> int:
            """(arg0: erl_geometry.pyerl_geometry.LidarFrame2Dd.Partition) -> int"""

    class Setting(YamlableBase):
        angle_max: float
        angle_min: float
        discontinuity_factor: float
        min_partition_size: int
        num_rays: int
        rolling_diff_discount: float
        valid_range_max: float
        valid_range_min: float
        def __init__(self) -> None:
            """__init__(self: erl_geometry.pyerl_geometry.LidarFrame2Dd.Setting) -> None"""
    def __init__(self, setting: LidarFrame2Dd.Setting) -> None:
        """__init__(self: erl_geometry.pyerl_geometry.LidarFrame2Dd, setting: erl_geometry.pyerl_geometry.LidarFrame2Dd.Setting) -> None"""
    def compute_closest_end_point(self, position: numpy.ndarray[numpy.float64[2, 1]]) -> dict:
        """compute_closest_end_point(self: erl_geometry.pyerl_geometry.LidarFrame2Dd, position: numpy.ndarray[numpy.float64[2, 1]]) -> dict"""
    def compute_rays_at(self, position_world: numpy.ndarray[numpy.float64[2, 1]]) -> dict:
        """compute_rays_at(self: erl_geometry.pyerl_geometry.LidarFrame2Dd, position_world: numpy.ndarray[numpy.float64[2, 1]]) -> dict"""
    def coords_is_in_frame(self, angle_frame: float) -> bool:
        """coords_is_in_frame(self: erl_geometry.pyerl_geometry.LidarFrame2Dd, angle_frame: float) -> bool"""
    def dir_frame_to_world(self, dir_frame: numpy.ndarray[numpy.float64[2, 1]]) -> numpy.ndarray[numpy.float64[2, 1]]:
        """dir_frame_to_world(self: erl_geometry.pyerl_geometry.LidarFrame2Dd, dir_frame: numpy.ndarray[numpy.float64[2, 1]]) -> numpy.ndarray[numpy.float64[2, 1]]"""
    def dir_world_to_frame(self, dir_world: numpy.ndarray[numpy.float64[2, 1]]) -> numpy.ndarray[numpy.float64[2, 1]]:
        """dir_world_to_frame(self: erl_geometry.pyerl_geometry.LidarFrame2Dd, dir_world: numpy.ndarray[numpy.float64[2, 1]]) -> numpy.ndarray[numpy.float64[2, 1]]"""
    def pos_frame_to_world(self, xy_frame: numpy.ndarray[numpy.float64[2, 1]]) -> numpy.ndarray[numpy.float64[2, 1]]:
        """pos_frame_to_world(self: erl_geometry.pyerl_geometry.LidarFrame2Dd, xy_frame: numpy.ndarray[numpy.float64[2, 1]]) -> numpy.ndarray[numpy.float64[2, 1]]"""
    def pos_world_to_frame(self, xy_world: numpy.ndarray[numpy.float64[2, 1]]) -> numpy.ndarray[numpy.float64[2, 1]]:
        """pos_world_to_frame(self: erl_geometry.pyerl_geometry.LidarFrame2Dd, xy_world: numpy.ndarray[numpy.float64[2, 1]]) -> numpy.ndarray[numpy.float64[2, 1]]"""
    def position_is_in_frame(self, xy_frame: numpy.ndarray[numpy.float64[2, 1]]) -> bool:
        """position_is_in_frame(self: erl_geometry.pyerl_geometry.LidarFrame2Dd, xy_frame: numpy.ndarray[numpy.float64[2, 1]]) -> bool"""
    @overload
    def sample_along_rays(self, num_samples_per_ray: int, max_in_obstacle_dist: float, sampled_rays_ratio: float) -> dict:
        """sample_along_rays(*args, **kwargs)
        Overloaded function.

        1. sample_along_rays(self: erl_geometry.pyerl_geometry.LidarFrame2Dd, num_samples_per_ray: int, max_in_obstacle_dist: float, sampled_rays_ratio: float) -> dict

        2. sample_along_rays(self: erl_geometry.pyerl_geometry.LidarFrame2Dd, range_step: float, max_in_obstacle_dist: float, sampled_rays_ratio: float) -> dict
        """
    @overload
    def sample_along_rays(self, range_step: float, max_in_obstacle_dist: float, sampled_rays_ratio: float) -> dict:
        """sample_along_rays(*args, **kwargs)
        Overloaded function.

        1. sample_along_rays(self: erl_geometry.pyerl_geometry.LidarFrame2Dd, num_samples_per_ray: int, max_in_obstacle_dist: float, sampled_rays_ratio: float) -> dict

        2. sample_along_rays(self: erl_geometry.pyerl_geometry.LidarFrame2Dd, range_step: float, max_in_obstacle_dist: float, sampled_rays_ratio: float) -> dict
        """
    def sample_in_region(self, num_positions: int, num_along_ray_samples_per_ray: int, num_near_surface_samples_per_ray: int, max_in_obstacle_dist: float) -> dict:
        """sample_in_region(self: erl_geometry.pyerl_geometry.LidarFrame2Dd, num_positions: int, num_along_ray_samples_per_ray: int, num_near_surface_samples_per_ray: int, max_in_obstacle_dist: float) -> dict"""
    def sample_near_surface(self, num_samples_per_ray: int, max_offset: float, sampled_rays_ratio: float) -> dict:
        """sample_near_surface(self: erl_geometry.pyerl_geometry.LidarFrame2Dd, num_samples_per_ray: int, max_offset: float, sampled_rays_ratio: float) -> dict"""
    def update_ranges(self, rotation: numpy.ndarray[numpy.float64[2, 2], flags.f_contiguous], translation: numpy.ndarray[numpy.float64[2, 1]], ranges: numpy.ndarray[numpy.float64[m, 1]]) -> None:
        """update_ranges(self: erl_geometry.pyerl_geometry.LidarFrame2Dd, rotation: numpy.ndarray[numpy.float64[2, 2], flags.f_contiguous], translation: numpy.ndarray[numpy.float64[2, 1]], ranges: numpy.ndarray[numpy.float64[m, 1]]) -> None"""
    @property
    def angles_in_frame(self) -> numpy.ndarray[numpy.float64[m, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.LidarFrame2Dd) -> numpy.ndarray[numpy.float64[m, 1]]"""
    @property
    def angles_in_world(self) -> numpy.ndarray[numpy.float64[m, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.LidarFrame2Dd) -> numpy.ndarray[numpy.float64[m, 1]]"""
    @property
    def end_points_in_frame(self) -> list[numpy.ndarray[numpy.float64[2, 1]]]:
        """(arg0: erl_geometry.pyerl_geometry.LidarFrame2Dd) -> list[numpy.ndarray[numpy.float64[2, 1]]]"""
    @property
    def end_points_in_world(self) -> list[numpy.ndarray[numpy.float64[2, 1]]]:
        """(arg0: erl_geometry.pyerl_geometry.LidarFrame2Dd) -> list[numpy.ndarray[numpy.float64[2, 1]]]"""
    @property
    def hit_points_world(self) -> list[numpy.ndarray[numpy.float64[2, 1]]]:
        """(arg0: erl_geometry.pyerl_geometry.LidarFrame2Dd) -> list[numpy.ndarray[numpy.float64[2, 1]]]"""
    @property
    def hit_ray_indices(self) -> list[int]:
        """(arg0: erl_geometry.pyerl_geometry.LidarFrame2Dd) -> list[int]"""
    @property
    def is_partitioned(self) -> bool:
        """(arg0: erl_geometry.pyerl_geometry.LidarFrame2Dd) -> bool"""
    @property
    def is_valid(self) -> bool:
        """(arg0: erl_geometry.pyerl_geometry.LidarFrame2Dd) -> bool"""
    @property
    def max_valid_range(self) -> float:
        """(arg0: erl_geometry.pyerl_geometry.LidarFrame2Dd) -> float"""
    @property
    def min_valid_range(self) -> float:
        """(arg0: erl_geometry.pyerl_geometry.LidarFrame2Dd) -> float"""
    @property
    def num_hit_rays(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.LidarFrame2Dd) -> int"""
    @property
    def num_rays(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.LidarFrame2Dd) -> int"""
    @property
    def partitions(self) -> list[LidarFrame2Dd.Partition]:
        """(arg0: erl_geometry.pyerl_geometry.LidarFrame2Dd) -> list[erl_geometry.pyerl_geometry.LidarFrame2Dd.Partition]"""
    @property
    def pose_matrix(self) -> numpy.ndarray[numpy.float64[3, 3]]:
        """(arg0: erl_geometry.pyerl_geometry.LidarFrame2Dd) -> numpy.ndarray[numpy.float64[3, 3]]"""
    @property
    def ranges(self) -> numpy.ndarray[numpy.float64[m, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.LidarFrame2Dd) -> numpy.ndarray[numpy.float64[m, 1]]"""
    @property
    def ray_directions_in_frame(self) -> list[numpy.ndarray[numpy.float64[2, 1]]]:
        """(arg0: erl_geometry.pyerl_geometry.LidarFrame2Dd) -> list[numpy.ndarray[numpy.float64[2, 1]]]"""
    @property
    def ray_directions_in_world(self) -> list[numpy.ndarray[numpy.float64[2, 1]]]:
        """(arg0: erl_geometry.pyerl_geometry.LidarFrame2Dd) -> list[numpy.ndarray[numpy.float64[2, 1]]]"""
    @property
    def rotation_angle(self) -> float:
        """(arg0: erl_geometry.pyerl_geometry.LidarFrame2Dd) -> float"""
    @property
    def rotation_matrix(self) -> numpy.ndarray[numpy.float64[2, 2]]:
        """(arg0: erl_geometry.pyerl_geometry.LidarFrame2Dd) -> numpy.ndarray[numpy.float64[2, 2]]"""
    @property
    def setting(self) -> LidarFrame2Dd.Setting:
        """(arg0: erl_geometry.pyerl_geometry.LidarFrame2Dd) -> erl_geometry.pyerl_geometry.LidarFrame2Dd.Setting"""
    @property
    def translation_vector(self) -> numpy.ndarray[numpy.float64[2, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.LidarFrame2Dd) -> numpy.ndarray[numpy.float64[2, 1]]"""

class LidarFrame2Df:
    class Partition:
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""
        def angle_in_partition(self, angle_world: float) -> bool:
            """angle_in_partition(self: erl_geometry.pyerl_geometry.LidarFrame2Df.Partition, angle_world: float) -> bool"""
        @property
        def index_begin(self) -> int:
            """(arg0: erl_geometry.pyerl_geometry.LidarFrame2Df.Partition) -> int"""
        @property
        def index_end(self) -> int:
            """(arg0: erl_geometry.pyerl_geometry.LidarFrame2Df.Partition) -> int"""

    class Setting(YamlableBase):
        angle_max: float
        angle_min: float
        discontinuity_factor: float
        min_partition_size: int
        num_rays: int
        rolling_diff_discount: float
        valid_range_max: float
        valid_range_min: float
        def __init__(self) -> None:
            """__init__(self: erl_geometry.pyerl_geometry.LidarFrame2Df.Setting) -> None"""
    def __init__(self, setting: LidarFrame2Df.Setting) -> None:
        """__init__(self: erl_geometry.pyerl_geometry.LidarFrame2Df, setting: erl_geometry.pyerl_geometry.LidarFrame2Df.Setting) -> None"""
    def compute_closest_end_point(self, position: numpy.ndarray[numpy.float32[2, 1]]) -> dict:
        """compute_closest_end_point(self: erl_geometry.pyerl_geometry.LidarFrame2Df, position: numpy.ndarray[numpy.float32[2, 1]]) -> dict"""
    def compute_rays_at(self, position_world: numpy.ndarray[numpy.float32[2, 1]]) -> dict:
        """compute_rays_at(self: erl_geometry.pyerl_geometry.LidarFrame2Df, position_world: numpy.ndarray[numpy.float32[2, 1]]) -> dict"""
    def coords_is_in_frame(self, angle_frame: float) -> bool:
        """coords_is_in_frame(self: erl_geometry.pyerl_geometry.LidarFrame2Df, angle_frame: float) -> bool"""
    def dir_frame_to_world(self, dir_frame: numpy.ndarray[numpy.float32[2, 1]]) -> numpy.ndarray[numpy.float32[2, 1]]:
        """dir_frame_to_world(self: erl_geometry.pyerl_geometry.LidarFrame2Df, dir_frame: numpy.ndarray[numpy.float32[2, 1]]) -> numpy.ndarray[numpy.float32[2, 1]]"""
    def dir_world_to_frame(self, dir_world: numpy.ndarray[numpy.float32[2, 1]]) -> numpy.ndarray[numpy.float32[2, 1]]:
        """dir_world_to_frame(self: erl_geometry.pyerl_geometry.LidarFrame2Df, dir_world: numpy.ndarray[numpy.float32[2, 1]]) -> numpy.ndarray[numpy.float32[2, 1]]"""
    def pos_frame_to_world(self, xy_frame: numpy.ndarray[numpy.float32[2, 1]]) -> numpy.ndarray[numpy.float32[2, 1]]:
        """pos_frame_to_world(self: erl_geometry.pyerl_geometry.LidarFrame2Df, xy_frame: numpy.ndarray[numpy.float32[2, 1]]) -> numpy.ndarray[numpy.float32[2, 1]]"""
    def pos_world_to_frame(self, xy_world: numpy.ndarray[numpy.float32[2, 1]]) -> numpy.ndarray[numpy.float32[2, 1]]:
        """pos_world_to_frame(self: erl_geometry.pyerl_geometry.LidarFrame2Df, xy_world: numpy.ndarray[numpy.float32[2, 1]]) -> numpy.ndarray[numpy.float32[2, 1]]"""
    def position_is_in_frame(self, xy_frame: numpy.ndarray[numpy.float32[2, 1]]) -> bool:
        """position_is_in_frame(self: erl_geometry.pyerl_geometry.LidarFrame2Df, xy_frame: numpy.ndarray[numpy.float32[2, 1]]) -> bool"""
    @overload
    def sample_along_rays(self, num_samples_per_ray: int, max_in_obstacle_dist: float, sampled_rays_ratio: float) -> dict:
        """sample_along_rays(*args, **kwargs)
        Overloaded function.

        1. sample_along_rays(self: erl_geometry.pyerl_geometry.LidarFrame2Df, num_samples_per_ray: int, max_in_obstacle_dist: float, sampled_rays_ratio: float) -> dict

        2. sample_along_rays(self: erl_geometry.pyerl_geometry.LidarFrame2Df, range_step: float, max_in_obstacle_dist: float, sampled_rays_ratio: float) -> dict
        """
    @overload
    def sample_along_rays(self, range_step: float, max_in_obstacle_dist: float, sampled_rays_ratio: float) -> dict:
        """sample_along_rays(*args, **kwargs)
        Overloaded function.

        1. sample_along_rays(self: erl_geometry.pyerl_geometry.LidarFrame2Df, num_samples_per_ray: int, max_in_obstacle_dist: float, sampled_rays_ratio: float) -> dict

        2. sample_along_rays(self: erl_geometry.pyerl_geometry.LidarFrame2Df, range_step: float, max_in_obstacle_dist: float, sampled_rays_ratio: float) -> dict
        """
    def sample_in_region(self, num_positions: int, num_along_ray_samples_per_ray: int, num_near_surface_samples_per_ray: int, max_in_obstacle_dist: float) -> dict:
        """sample_in_region(self: erl_geometry.pyerl_geometry.LidarFrame2Df, num_positions: int, num_along_ray_samples_per_ray: int, num_near_surface_samples_per_ray: int, max_in_obstacle_dist: float) -> dict"""
    def sample_near_surface(self, num_samples_per_ray: int, max_offset: float, sampled_rays_ratio: float) -> dict:
        """sample_near_surface(self: erl_geometry.pyerl_geometry.LidarFrame2Df, num_samples_per_ray: int, max_offset: float, sampled_rays_ratio: float) -> dict"""
    def update_ranges(self, rotation: numpy.ndarray[numpy.float32[2, 2], flags.f_contiguous], translation: numpy.ndarray[numpy.float32[2, 1]], ranges: numpy.ndarray[numpy.float32[m, 1]]) -> None:
        """update_ranges(self: erl_geometry.pyerl_geometry.LidarFrame2Df, rotation: numpy.ndarray[numpy.float32[2, 2], flags.f_contiguous], translation: numpy.ndarray[numpy.float32[2, 1]], ranges: numpy.ndarray[numpy.float32[m, 1]]) -> None"""
    @property
    def angles_in_frame(self) -> numpy.ndarray[numpy.float32[m, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.LidarFrame2Df) -> numpy.ndarray[numpy.float32[m, 1]]"""
    @property
    def angles_in_world(self) -> numpy.ndarray[numpy.float32[m, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.LidarFrame2Df) -> numpy.ndarray[numpy.float32[m, 1]]"""
    @property
    def end_points_in_frame(self) -> list[numpy.ndarray[numpy.float32[2, 1]]]:
        """(arg0: erl_geometry.pyerl_geometry.LidarFrame2Df) -> list[numpy.ndarray[numpy.float32[2, 1]]]"""
    @property
    def end_points_in_world(self) -> list[numpy.ndarray[numpy.float32[2, 1]]]:
        """(arg0: erl_geometry.pyerl_geometry.LidarFrame2Df) -> list[numpy.ndarray[numpy.float32[2, 1]]]"""
    @property
    def hit_points_world(self) -> list[numpy.ndarray[numpy.float32[2, 1]]]:
        """(arg0: erl_geometry.pyerl_geometry.LidarFrame2Df) -> list[numpy.ndarray[numpy.float32[2, 1]]]"""
    @property
    def hit_ray_indices(self) -> list[int]:
        """(arg0: erl_geometry.pyerl_geometry.LidarFrame2Df) -> list[int]"""
    @property
    def is_partitioned(self) -> bool:
        """(arg0: erl_geometry.pyerl_geometry.LidarFrame2Df) -> bool"""
    @property
    def is_valid(self) -> bool:
        """(arg0: erl_geometry.pyerl_geometry.LidarFrame2Df) -> bool"""
    @property
    def max_valid_range(self) -> float:
        """(arg0: erl_geometry.pyerl_geometry.LidarFrame2Df) -> float"""
    @property
    def min_valid_range(self) -> float:
        """(arg0: erl_geometry.pyerl_geometry.LidarFrame2Df) -> float"""
    @property
    def num_hit_rays(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.LidarFrame2Df) -> int"""
    @property
    def num_rays(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.LidarFrame2Df) -> int"""
    @property
    def partitions(self) -> list[LidarFrame2Df.Partition]:
        """(arg0: erl_geometry.pyerl_geometry.LidarFrame2Df) -> list[erl_geometry.pyerl_geometry.LidarFrame2Df.Partition]"""
    @property
    def pose_matrix(self) -> numpy.ndarray[numpy.float32[3, 3]]:
        """(arg0: erl_geometry.pyerl_geometry.LidarFrame2Df) -> numpy.ndarray[numpy.float32[3, 3]]"""
    @property
    def ranges(self) -> numpy.ndarray[numpy.float32[m, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.LidarFrame2Df) -> numpy.ndarray[numpy.float32[m, 1]]"""
    @property
    def ray_directions_in_frame(self) -> list[numpy.ndarray[numpy.float32[2, 1]]]:
        """(arg0: erl_geometry.pyerl_geometry.LidarFrame2Df) -> list[numpy.ndarray[numpy.float32[2, 1]]]"""
    @property
    def ray_directions_in_world(self) -> list[numpy.ndarray[numpy.float32[2, 1]]]:
        """(arg0: erl_geometry.pyerl_geometry.LidarFrame2Df) -> list[numpy.ndarray[numpy.float32[2, 1]]]"""
    @property
    def rotation_angle(self) -> float:
        """(arg0: erl_geometry.pyerl_geometry.LidarFrame2Df) -> float"""
    @property
    def rotation_matrix(self) -> numpy.ndarray[numpy.float32[2, 2]]:
        """(arg0: erl_geometry.pyerl_geometry.LidarFrame2Df) -> numpy.ndarray[numpy.float32[2, 2]]"""
    @property
    def setting(self) -> LidarFrame2Df.Setting:
        """(arg0: erl_geometry.pyerl_geometry.LidarFrame2Df) -> erl_geometry.pyerl_geometry.LidarFrame2Df.Setting"""
    @property
    def translation_vector(self) -> numpy.ndarray[numpy.float32[2, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.LidarFrame2Df) -> numpy.ndarray[numpy.float32[2, 1]]"""

class LidarFrame3Dd(RangeSensorFrame3Dd):
    class Setting(RangeSensorFrame3Dd.Setting):
        azimuth_max: float
        azimuth_min: float
        elevation_max: float
        elevation_min: float
        num_azimuth_lines: int
        num_elevation_lines: int
        def __init__(self) -> None:
            """__init__(self: erl_geometry.pyerl_geometry.LidarFrame3Dd.Setting) -> None"""
    def __init__(self, setting: LidarFrame3Dd.Setting) -> None:
        """__init__(self: erl_geometry.pyerl_geometry.LidarFrame3Dd, setting: erl_geometry.pyerl_geometry.LidarFrame3Dd.Setting) -> None"""
    def reset(self) -> None:
        """reset(self: erl_geometry.pyerl_geometry.LidarFrame3Dd) -> None"""
    def update_ranges(self, rotation: numpy.ndarray[numpy.float64[3, 3], flags.f_contiguous], translation: numpy.ndarray[numpy.float64[3, 1]], ranges: numpy.ndarray[numpy.float64[m, n]]) -> None:
        """update_ranges(self: erl_geometry.pyerl_geometry.LidarFrame3Dd, rotation: numpy.ndarray[numpy.float64[3, 3], flags.f_contiguous], translation: numpy.ndarray[numpy.float64[3, 1]], ranges: numpy.ndarray[numpy.float64[m, n]]) -> None"""
    @property
    def num_azimuth_lines(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.LidarFrame3Dd) -> int"""
    @property
    def num_elevation_lines(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.LidarFrame3Dd) -> int"""
    @property
    def setting(self) -> LidarFrame3Dd.Setting:
        """(arg0: erl_geometry.pyerl_geometry.LidarFrame3Dd) -> erl_geometry.pyerl_geometry.LidarFrame3Dd.Setting"""

class LidarFrame3Df(RangeSensorFrame3Df):
    class Setting(RangeSensorFrame3Df.Setting):
        azimuth_max: float
        azimuth_min: float
        elevation_max: float
        elevation_min: float
        num_azimuth_lines: int
        num_elevation_lines: int
        def __init__(self) -> None:
            """__init__(self: erl_geometry.pyerl_geometry.LidarFrame3Df.Setting) -> None"""
    def __init__(self, setting: LidarFrame3Df.Setting) -> None:
        """__init__(self: erl_geometry.pyerl_geometry.LidarFrame3Df, setting: erl_geometry.pyerl_geometry.LidarFrame3Df.Setting) -> None"""
    def reset(self) -> None:
        """reset(self: erl_geometry.pyerl_geometry.LidarFrame3Df) -> None"""
    def update_ranges(self, rotation: numpy.ndarray[numpy.float32[3, 3], flags.f_contiguous], translation: numpy.ndarray[numpy.float32[3, 1]], ranges: numpy.ndarray[numpy.float32[m, n]]) -> None:
        """update_ranges(self: erl_geometry.pyerl_geometry.LidarFrame3Df, rotation: numpy.ndarray[numpy.float32[3, 3], flags.f_contiguous], translation: numpy.ndarray[numpy.float32[3, 1]], ranges: numpy.ndarray[numpy.float32[m, n]]) -> None"""
    @property
    def num_azimuth_lines(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.LidarFrame3Df) -> int"""
    @property
    def num_elevation_lines(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.LidarFrame3Df) -> int"""
    @property
    def setting(self) -> LidarFrame3Df.Setting:
        """(arg0: erl_geometry.pyerl_geometry.LidarFrame3Df) -> erl_geometry.pyerl_geometry.LidarFrame3Df.Setting"""

class Line2D(Primitive2D):
    p0: numpy.ndarray[numpy.float64[2, 1]]
    p1: numpy.ndarray[numpy.float64[2, 1]]
    def __init__(self, id: int, p0: numpy.ndarray[numpy.float64[2, 1]], p1: numpy.ndarray[numpy.float64[2, 1]]) -> None:
        """__init__(self: erl_geometry.pyerl_geometry.Line2D, id: int, p0: numpy.ndarray[numpy.float64[2, 1]], p1: numpy.ndarray[numpy.float64[2, 1]]) -> None"""

class LogOddMap:
    class CellType:
        """Type of grid cell.

        Members:

          kOccupied

          kUnexplored

          kFree"""
        __members__: ClassVar[dict] = ...  # read-only
        __entries: ClassVar[dict] = ...
        kFree: ClassVar[LogOddMap.CellType] = ...
        kOccupied: ClassVar[LogOddMap.CellType] = ...
        kUnexplored: ClassVar[LogOddMap.CellType] = ...
        def __init__(self, value: int) -> None:
            """__init__(self: erl_geometry.pyerl_geometry.LogOddMap.CellType, value: int) -> None"""
        def __and__(self, other: object) -> object:
            """__and__(self: object, other: object) -> object"""
        def __eq__(self, other: object) -> bool:
            """__eq__(self: object, other: object) -> bool"""
        def __ge__(self, other: object) -> bool:
            """__ge__(self: object, other: object) -> bool"""
        def __gt__(self, other: object) -> bool:
            """__gt__(self: object, other: object) -> bool"""
        def __hash__(self) -> int:
            """__hash__(self: object) -> int"""
        def __index__(self) -> int:
            """__index__(self: erl_geometry.pyerl_geometry.LogOddMap.CellType) -> int"""
        def __int__(self) -> int:
            """__int__(self: erl_geometry.pyerl_geometry.LogOddMap.CellType) -> int"""
        def __invert__(self) -> object:
            """__invert__(self: object) -> object"""
        def __le__(self, other: object) -> bool:
            """__le__(self: object, other: object) -> bool"""
        def __lt__(self, other: object) -> bool:
            """__lt__(self: object, other: object) -> bool"""
        def __ne__(self, other: object) -> bool:
            """__ne__(self: object, other: object) -> bool"""
        def __or__(self, other: object) -> object:
            """__or__(self: object, other: object) -> object"""
        def __rand__(self, other: object) -> object:
            """__rand__(self: object, other: object) -> object"""
        def __ror__(self, other: object) -> object:
            """__ror__(self: object, other: object) -> object"""
        def __rxor__(self, other: object) -> object:
            """__rxor__(self: object, other: object) -> object"""
        def __xor__(self, other: object) -> object:
            """__xor__(self: object, other: object) -> object"""
        @property
        def name(self) -> str:
            """name(self: object) -> str

            name(self: object) -> str
            """
        @property
        def value(self) -> int:
            """(arg0: erl_geometry.pyerl_geometry.LogOddMap.CellType) -> int"""
    kFree: ClassVar[LogOddMap.CellType] = ...
    kOccupied: ClassVar[LogOddMap.CellType] = ...
    kUnexplored: ClassVar[LogOddMap.CellType] = ...
    def __init__(self, *args, **kwargs) -> None:
        """Initialize self.  See help(type(self)) for accurate signature."""

class LogOddMap2Dd:
    class Setting(YamlableBase):
        filter_obstacles_in_cleaned_mask: bool
        max_log_odd: float
        measurement_certainty: float
        min_log_odd: float
        num_iters_for_cleaned_mask: int
        sensor_max_range: float
        sensor_min_range: float
        threshold_free: float
        threshold_occupied: float
        use_cross_kernel: bool
        def __init__(self) -> None:
            """__init__(self: erl_geometry.pyerl_geometry.LogOddMap2Dd.Setting) -> None"""
    def __init__(self, *args, **kwargs) -> None:
        """__init__(*args, **kwargs)
        Overloaded function.

        1. __init__(self: erl_geometry.pyerl_geometry.LogOddMap2Dd, setting: erl_geometry.pyerl_geometry.LogOddMap2Dd.Setting, grid_map_info: erl::common::GridMapInfo<double, 2>) -> None

        2. __init__(self: erl_geometry.pyerl_geometry.LogOddMap2Dd, setting: erl_geometry.pyerl_geometry.LogOddMap2Dd.Setting, grid_map_info: erl::common::GridMapInfo<double, 2>, shape_vertices: numpy.ndarray[numpy.float64[2, n], flags.f_contiguous]) -> None
        """
    def compute_statistics_of_lidar_frame(self, position: numpy.ndarray[numpy.float64[2, 1]], theta: float, angles_body: numpy.ndarray[numpy.float64[m, 1]], ranges: numpy.ndarray[numpy.float64[m, 1]], clip_ranges: bool) -> tuple[int, int, int, int]:
        """compute_statistics_of_lidar_frame(self: erl_geometry.pyerl_geometry.LogOddMap2Dd, position: numpy.ndarray[numpy.float64[2, 1]], theta: float, angles_body: numpy.ndarray[numpy.float64[m, 1]], ranges: numpy.ndarray[numpy.float64[m, 1]], clip_ranges: bool) -> tuple[int, int, int, int]"""
    @staticmethod
    def get_cell_type_from_name(cell_type_name: str) -> LogOddMap.CellType:
        """get_cell_type_from_name(cell_type_name: str) -> erl_geometry.pyerl_geometry.LogOddMap.CellType"""
    @staticmethod
    def get_cell_type_name(cell_type: LogOddMap.CellType) -> str:
        """get_cell_type_name(cell_type: erl_geometry.pyerl_geometry.LogOddMap.CellType) -> str"""
    def get_frontiers(self, clean_at_first: bool = ..., approx_iters: int = ...) -> list[numpy.ndarray[numpy.int32[2, n]]]:
        """get_frontiers(self: erl_geometry.pyerl_geometry.LogOddMap2Dd, clean_at_first: bool = True, approx_iters: int = 4) -> list[numpy.ndarray[numpy.int32[2, n]]]"""
    def load_external_possibility_map(self, position: numpy.ndarray[numpy.float64[2, 1]], theta: float, possibility_map: numpy.ndarray[numpy.int32[m, n], flags.f_contiguous]) -> None:
        """load_external_possibility_map(self: erl_geometry.pyerl_geometry.LogOddMap2Dd, position: numpy.ndarray[numpy.float64[2, 1]], theta: float, possibility_map: numpy.ndarray[numpy.int32[m, n], flags.f_contiguous]) -> None"""
    def update(self, position: numpy.ndarray[numpy.float64[2, 1]], theta: float, angles_body: numpy.ndarray[numpy.float64[m, 1]], ranges: numpy.ndarray[numpy.float64[m, 1]]) -> None:
        """update(self: erl_geometry.pyerl_geometry.LogOddMap2Dd, position: numpy.ndarray[numpy.float64[2, 1]], theta: float, angles_body: numpy.ndarray[numpy.float64[m, 1]], ranges: numpy.ndarray[numpy.float64[m, 1]]) -> None"""
    @property
    def cleaned_free_mask(self) -> numpy.ndarray[numpy.uint8[m, n]]:
        """(arg0: erl_geometry.pyerl_geometry.LogOddMap2Dd) -> numpy.ndarray[numpy.uint8[m, n]]"""
    @property
    def cleaned_occupied_mask(self) -> numpy.ndarray[numpy.uint8[m, n]]:
        """(arg0: erl_geometry.pyerl_geometry.LogOddMap2Dd) -> numpy.ndarray[numpy.uint8[m, n]]"""
    @property
    def cleaned_unexplored_mask(self) -> numpy.ndarray[numpy.uint8[m, n]]:
        """(arg0: erl_geometry.pyerl_geometry.LogOddMap2Dd) -> numpy.ndarray[numpy.uint8[m, n]]"""
    @property
    def free_mask(self) -> numpy.ndarray[numpy.uint8[m, n]]:
        """(arg0: erl_geometry.pyerl_geometry.LogOddMap2Dd) -> numpy.ndarray[numpy.uint8[m, n]]"""
    @property
    def log_map(self) -> numpy.ndarray[numpy.float64[m, n]]:
        """(arg0: erl_geometry.pyerl_geometry.LogOddMap2Dd) -> numpy.ndarray[numpy.float64[m, n]]"""
    @property
    def num_free_cells(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.LogOddMap2Dd) -> int"""
    @property
    def num_occupied_cells(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.LogOddMap2Dd) -> int"""
    @property
    def num_unexplored_cells(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.LogOddMap2Dd) -> int"""
    @property
    def occupancy_map(self) -> numpy.ndarray[numpy.uint8[m, n]]:
        """(arg0: erl_geometry.pyerl_geometry.LogOddMap2Dd) -> numpy.ndarray[numpy.uint8[m, n]]"""
    @property
    def occupied_mask(self) -> numpy.ndarray[numpy.uint8[m, n]]:
        """(arg0: erl_geometry.pyerl_geometry.LogOddMap2Dd) -> numpy.ndarray[numpy.uint8[m, n]]"""
    @property
    def possibility_map(self) -> numpy.ndarray[numpy.float64[m, n]]:
        """(arg0: erl_geometry.pyerl_geometry.LogOddMap2Dd) -> numpy.ndarray[numpy.float64[m, n]]"""
    @property
    def setting(self) -> LogOddMap2Dd.Setting:
        """(arg0: erl_geometry.pyerl_geometry.LogOddMap2Dd) -> erl_geometry.pyerl_geometry.LogOddMap2Dd.Setting"""
    @property
    def unexplored_mask(self) -> numpy.ndarray[numpy.uint8[m, n]]:
        """(arg0: erl_geometry.pyerl_geometry.LogOddMap2Dd) -> numpy.ndarray[numpy.uint8[m, n]]"""

class LogOddMap2Df:
    class Setting(YamlableBase):
        filter_obstacles_in_cleaned_mask: bool
        max_log_odd: float
        measurement_certainty: float
        min_log_odd: float
        num_iters_for_cleaned_mask: int
        sensor_max_range: float
        sensor_min_range: float
        threshold_free: float
        threshold_occupied: float
        use_cross_kernel: bool
        def __init__(self) -> None:
            """__init__(self: erl_geometry.pyerl_geometry.LogOddMap2Df.Setting) -> None"""
    def __init__(self, *args, **kwargs) -> None:
        """__init__(*args, **kwargs)
        Overloaded function.

        1. __init__(self: erl_geometry.pyerl_geometry.LogOddMap2Df, setting: erl_geometry.pyerl_geometry.LogOddMap2Df.Setting, grid_map_info: erl::common::GridMapInfo<float, 2>) -> None

        2. __init__(self: erl_geometry.pyerl_geometry.LogOddMap2Df, setting: erl_geometry.pyerl_geometry.LogOddMap2Df.Setting, grid_map_info: erl::common::GridMapInfo<float, 2>, shape_vertices: numpy.ndarray[numpy.float32[2, n], flags.f_contiguous]) -> None
        """
    def compute_statistics_of_lidar_frame(self, position: numpy.ndarray[numpy.float32[2, 1]], theta: float, angles_body: numpy.ndarray[numpy.float32[m, 1]], ranges: numpy.ndarray[numpy.float32[m, 1]], clip_ranges: bool) -> tuple[int, int, int, int]:
        """compute_statistics_of_lidar_frame(self: erl_geometry.pyerl_geometry.LogOddMap2Df, position: numpy.ndarray[numpy.float32[2, 1]], theta: float, angles_body: numpy.ndarray[numpy.float32[m, 1]], ranges: numpy.ndarray[numpy.float32[m, 1]], clip_ranges: bool) -> tuple[int, int, int, int]"""
    @staticmethod
    def get_cell_type_from_name(cell_type_name: str) -> LogOddMap.CellType:
        """get_cell_type_from_name(cell_type_name: str) -> erl_geometry.pyerl_geometry.LogOddMap.CellType"""
    @staticmethod
    def get_cell_type_name(cell_type: LogOddMap.CellType) -> str:
        """get_cell_type_name(cell_type: erl_geometry.pyerl_geometry.LogOddMap.CellType) -> str"""
    def get_frontiers(self, clean_at_first: bool = ..., approx_iters: int = ...) -> list[numpy.ndarray[numpy.int32[2, n]]]:
        """get_frontiers(self: erl_geometry.pyerl_geometry.LogOddMap2Df, clean_at_first: bool = True, approx_iters: int = 4) -> list[numpy.ndarray[numpy.int32[2, n]]]"""
    def load_external_possibility_map(self, position: numpy.ndarray[numpy.float32[2, 1]], theta: float, possibility_map: numpy.ndarray[numpy.int32[m, n], flags.f_contiguous]) -> None:
        """load_external_possibility_map(self: erl_geometry.pyerl_geometry.LogOddMap2Df, position: numpy.ndarray[numpy.float32[2, 1]], theta: float, possibility_map: numpy.ndarray[numpy.int32[m, n], flags.f_contiguous]) -> None"""
    def update(self, position: numpy.ndarray[numpy.float32[2, 1]], theta: float, angles_body: numpy.ndarray[numpy.float32[m, 1]], ranges: numpy.ndarray[numpy.float32[m, 1]]) -> None:
        """update(self: erl_geometry.pyerl_geometry.LogOddMap2Df, position: numpy.ndarray[numpy.float32[2, 1]], theta: float, angles_body: numpy.ndarray[numpy.float32[m, 1]], ranges: numpy.ndarray[numpy.float32[m, 1]]) -> None"""
    @property
    def cleaned_free_mask(self) -> numpy.ndarray[numpy.uint8[m, n]]:
        """(arg0: erl_geometry.pyerl_geometry.LogOddMap2Df) -> numpy.ndarray[numpy.uint8[m, n]]"""
    @property
    def cleaned_occupied_mask(self) -> numpy.ndarray[numpy.uint8[m, n]]:
        """(arg0: erl_geometry.pyerl_geometry.LogOddMap2Df) -> numpy.ndarray[numpy.uint8[m, n]]"""
    @property
    def cleaned_unexplored_mask(self) -> numpy.ndarray[numpy.uint8[m, n]]:
        """(arg0: erl_geometry.pyerl_geometry.LogOddMap2Df) -> numpy.ndarray[numpy.uint8[m, n]]"""
    @property
    def free_mask(self) -> numpy.ndarray[numpy.uint8[m, n]]:
        """(arg0: erl_geometry.pyerl_geometry.LogOddMap2Df) -> numpy.ndarray[numpy.uint8[m, n]]"""
    @property
    def log_map(self) -> numpy.ndarray[numpy.float32[m, n]]:
        """(arg0: erl_geometry.pyerl_geometry.LogOddMap2Df) -> numpy.ndarray[numpy.float32[m, n]]"""
    @property
    def num_free_cells(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.LogOddMap2Df) -> int"""
    @property
    def num_occupied_cells(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.LogOddMap2Df) -> int"""
    @property
    def num_unexplored_cells(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.LogOddMap2Df) -> int"""
    @property
    def occupancy_map(self) -> numpy.ndarray[numpy.uint8[m, n]]:
        """(arg0: erl_geometry.pyerl_geometry.LogOddMap2Df) -> numpy.ndarray[numpy.uint8[m, n]]"""
    @property
    def occupied_mask(self) -> numpy.ndarray[numpy.uint8[m, n]]:
        """(arg0: erl_geometry.pyerl_geometry.LogOddMap2Df) -> numpy.ndarray[numpy.uint8[m, n]]"""
    @property
    def possibility_map(self) -> numpy.ndarray[numpy.float32[m, n]]:
        """(arg0: erl_geometry.pyerl_geometry.LogOddMap2Df) -> numpy.ndarray[numpy.float32[m, n]]"""
    @property
    def setting(self) -> LogOddMap2Df.Setting:
        """(arg0: erl_geometry.pyerl_geometry.LogOddMap2Df) -> erl_geometry.pyerl_geometry.LogOddMap2Df.Setting"""
    @property
    def unexplored_mask(self) -> numpy.ndarray[numpy.uint8[m, n]]:
        """(arg0: erl_geometry.pyerl_geometry.LogOddMap2Df) -> numpy.ndarray[numpy.uint8[m, n]]"""

class NdTreeSetting(YamlableBase):
    resolution: float
    tree_depth: int
    def __init__(self) -> None:
        """__init__(self: erl_geometry.pyerl_geometry.NdTreeSetting) -> None"""
    def __eq__(self, other: NdTreeSetting) -> bool:
        """__eq__(self: erl_geometry.pyerl_geometry.NdTreeSetting, other: erl_geometry.pyerl_geometry.NdTreeSetting) -> bool"""
    def __ne__(self, other: NdTreeSetting) -> bool:
        """__ne__(self: erl_geometry.pyerl_geometry.NdTreeSetting, other: erl_geometry.pyerl_geometry.NdTreeSetting) -> bool"""

class OccupancyNdTreeSetting(NdTreeSetting):
    log_odd_hit: float
    log_odd_max: float
    log_odd_min: float
    log_odd_miss: float
    log_odd_occ_threshold: float
    probability_hit: float
    probability_miss: float
    probability_occupied_threshold: float
    def __init__(self) -> None:
        """__init__(self: erl_geometry.pyerl_geometry.OccupancyNdTreeSetting) -> None"""

class OccupancyOctreeBaseSetting(OccupancyNdTreeSetting):
    aabb: Aabb3Dd
    use_aabb_limit: bool
    use_change_detection: bool
    def __init__(self) -> None:
        """__init__(self: erl_geometry.pyerl_geometry.OccupancyOctreeBaseSetting) -> None"""

class OccupancyOctreeD(AbstractOccupancyOctreeD):
    class BatchRayCaster:
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""
        def step(self, max_depth: numpy.ndarray[bool[m, 1]] = ...) -> OccupancyOctreeD.BatchRayCaster:
            """step(self: erl_geometry.pyerl_geometry.OccupancyOctreeD.BatchRayCaster, max_depth: numpy.ndarray[bool[m, 1]] = 0) -> erl_geometry.pyerl_geometry.OccupancyOctreeD.BatchRayCaster"""
        @property
        def ever_hit_flags(self) -> numpy.ndarray[bool[m, 1]]:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeD.BatchRayCaster) -> numpy.ndarray[bool[m, 1]]"""
        @property
        def frontier_keys(self) -> list[OctreeKey]:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeD.BatchRayCaster) -> list[erl_geometry.pyerl_geometry.OctreeKey]"""
        @property
        def frontier_nodes(self) -> list[OccupancyOctreeNode]:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeD.BatchRayCaster) -> list[erl_geometry.pyerl_geometry.OccupancyOctreeNode]"""
        @property
        def frontier_ray_indices(self) -> list[list[int]]:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeD.BatchRayCaster) -> list[list[int]]"""
        @property
        def hit_distances(self) -> numpy.ndarray[numpy.float64[m, 1]]:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeD.BatchRayCaster) -> numpy.ndarray[numpy.float64[m, 1]]"""
        @property
        def hit_flags(self) -> numpy.ndarray[bool[m, 1]]:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeD.BatchRayCaster) -> numpy.ndarray[bool[m, 1]]"""
        @property
        def hit_nodes(self) -> list[OccupancyOctreeNode]:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeD.BatchRayCaster) -> list[erl_geometry.pyerl_geometry.OccupancyOctreeNode]"""
        @property
        def hit_positions(self) -> list[numpy.ndarray[numpy.float64[3, 1]]]:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeD.BatchRayCaster) -> list[numpy.ndarray[numpy.float64[3, 1]]]"""
        @property
        def num_rays(self) -> int:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeD.BatchRayCaster) -> int"""
        @property
        def ray_directions(self) -> numpy.ndarray[numpy.float64[3, n]]:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeD.BatchRayCaster) -> numpy.ndarray[numpy.float64[3, n]]"""
        @property
        def ray_origins(self) -> numpy.ndarray[numpy.float64[3, n]]:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeD.BatchRayCaster) -> numpy.ndarray[numpy.float64[3, n]]"""

    class BottomLeafNeighborIterator(OccupancyOctreeD.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class Drawer:
        def __init__(self, setting: OccupancyOctreeDrawerSetting, octree: OccupancyOctreeD = ...) -> None:
            """__init__(self: erl_geometry.pyerl_geometry.OccupancyOctreeD.Drawer, setting: erl_geometry.pyerl_geometry.OccupancyOctreeDrawerSetting, octree: erl_geometry.pyerl_geometry.OccupancyOctreeD = None) -> None"""
        @staticmethod
        def Setting() -> OccupancyOctreeDrawerSetting:
            """Setting() -> erl_geometry.pyerl_geometry.OccupancyOctreeDrawerSetting"""
        def draw_leaves(self, filename: str) -> None:
            """draw_leaves(*args, **kwargs)
            Overloaded function.

            1. draw_leaves(self: erl_geometry.pyerl_geometry.OccupancyOctreeD.Drawer) -> list[open3d::geometry::Geometry]

            2. draw_leaves(self: erl_geometry.pyerl_geometry.OccupancyOctreeD.Drawer, filename: str) -> None
            """
        def draw_tree(self, filename: str) -> None:
            """draw_tree(*args, **kwargs)
            Overloaded function.

            1. draw_tree(self: erl_geometry.pyerl_geometry.OccupancyOctreeD.Drawer) -> list[open3d::geometry::Geometry]

            2. draw_tree(self: erl_geometry.pyerl_geometry.OccupancyOctreeD.Drawer, filename: str) -> None
            """
        def set_draw_leaf_callback(self, callback) -> None:
            """set_draw_leaf_callback(self: erl_geometry.pyerl_geometry.OccupancyOctreeD.Drawer, callback: Callable[[erl_geometry.pyerl_geometry.OccupancyOctreeD.Drawer, list[open3d::geometry::Geometry], erl_geometry.pyerl_geometry.OccupancyOctreeD.LeafInAabbIterator], None]) -> None"""
        def set_draw_tree_callback(self, callback) -> None:
            """set_draw_tree_callback(self: erl_geometry.pyerl_geometry.OccupancyOctreeD.Drawer, callback: Callable[[erl_geometry.pyerl_geometry.OccupancyOctreeD.Drawer, list[open3d::geometry::Geometry], erl_geometry.pyerl_geometry.OccupancyOctreeD.TreeInAabbIterator], None]) -> None"""
        @property
        def setting(self) -> OccupancyOctreeDrawerSetting:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeD.Drawer) -> erl_geometry.pyerl_geometry.OccupancyOctreeDrawerSetting"""

    class EastLeafNeighborIterator(OccupancyOctreeD.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class IteratorBase(AbstractOctreeD.OctreeNodeIterator):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""
        def __eq__(self, arg0: OccupancyOctreeD.IteratorBase) -> bool:
            """__eq__(self: erl_geometry.pyerl_geometry.OccupancyOctreeD.IteratorBase, arg0: erl_geometry.pyerl_geometry.OccupancyOctreeD.IteratorBase) -> bool"""
        def __ne__(self, arg0: OccupancyOctreeD.IteratorBase) -> bool:
            """__ne__(self: erl_geometry.pyerl_geometry.OccupancyOctreeD.IteratorBase, arg0: erl_geometry.pyerl_geometry.OccupancyOctreeD.IteratorBase) -> bool"""
        @property
        def index_key(self) -> OctreeKey:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeD.IteratorBase) -> erl_geometry.pyerl_geometry.OctreeKey"""
        @property
        def key(self) -> OctreeKey:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeD.IteratorBase) -> erl_geometry.pyerl_geometry.OctreeKey"""
        @property
        def node(self) -> OccupancyOctreeNode:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeD.IteratorBase) -> erl_geometry.pyerl_geometry.OccupancyOctreeNode"""
        @property
        def node_aabb(self) -> Aabb3Dd:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeD.IteratorBase) -> erl_geometry.pyerl_geometry.Aabb3Dd"""

    class LeafInAabbIterator(OccupancyOctreeD.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class LeafIterator(OccupancyOctreeD.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class LeafOfNodeIterator(OccupancyOctreeD.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class NodeOnRayIterator(OccupancyOctreeD.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""
        @property
        def distance(self) -> float:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeD.NodeOnRayIterator) -> float"""

    class NorthLeafNeighborIterator(OccupancyOctreeD.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class SouthLeafNeighborIterator(OccupancyOctreeD.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class TopLeafNeighborIterator(OccupancyOctreeD.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class TreeInAabbIterator(OccupancyOctreeD.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class TreeIterator(OccupancyOctreeD.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class WestLeafNeighborIterator(OccupancyOctreeD.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""
    @overload
    def __init__(self) -> None:
        """__init__(*args, **kwargs)
        Overloaded function.

        1. __init__(self: erl_geometry.pyerl_geometry.OccupancyOctreeD) -> None

        2. __init__(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, setting: erl_geometry.pyerl_geometry.OccupancyOctreeBaseSetting) -> None

        3. __init__(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, filename: str) -> None
        """
    @overload
    def __init__(self, setting: OccupancyOctreeBaseSetting) -> None:
        """__init__(*args, **kwargs)
        Overloaded function.

        1. __init__(self: erl_geometry.pyerl_geometry.OccupancyOctreeD) -> None

        2. __init__(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, setting: erl_geometry.pyerl_geometry.OccupancyOctreeBaseSetting) -> None

        3. __init__(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, filename: str) -> None
        """
    @overload
    def __init__(self, filename: str) -> None:
        """__init__(*args, **kwargs)
        Overloaded function.

        1. __init__(self: erl_geometry.pyerl_geometry.OccupancyOctreeD) -> None

        2. __init__(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, setting: erl_geometry.pyerl_geometry.OccupancyOctreeBaseSetting) -> None

        3. __init__(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, filename: str) -> None
        """
    def Setting(self) -> OccupancyOctreeBaseSetting:
        """Setting() -> erl_geometry.pyerl_geometry.OccupancyOctreeBaseSetting"""
    @overload
    def adjust_key_to_depth(self, key: int, depth: int) -> int:
        """adjust_key_to_depth(*args, **kwargs)
        Overloaded function.

        1. adjust_key_to_depth(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, key: int, depth: int) -> int

        2. adjust_key_to_depth(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> erl_geometry.pyerl_geometry.OctreeKey
        """
    @overload
    def adjust_key_to_depth(self, key: OctreeKey, depth: int) -> OctreeKey:
        """adjust_key_to_depth(*args, **kwargs)
        Overloaded function.

        1. adjust_key_to_depth(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, key: int, depth: int) -> int

        2. adjust_key_to_depth(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> erl_geometry.pyerl_geometry.OctreeKey
        """
    def cast_ray(self, px: float, py: float, pz: float, vx: float, vy: float, vz: float, ignore_unknown: bool, max_range: float) -> dict:
        """cast_ray(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, px: float, py: float, pz: float, vx: float, vy: float, vz: float, ignore_unknown: bool, max_range: float) -> dict"""
    @overload
    def cast_rays(self, position: numpy.ndarray[numpy.float64[3, 1]], rotation: numpy.ndarray[numpy.float64[3, 3], flags.f_contiguous], azimuth_angles: numpy.ndarray[numpy.float64[m, 1]], elevation_angles: numpy.ndarray[numpy.float64[m, 1]], ignore_unknown: bool, max_range: float, prune_rays: bool, parallel: bool) -> dict:
        """cast_rays(*args, **kwargs)
        Overloaded function.

        1. cast_rays(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, position: numpy.ndarray[numpy.float64[3, 1]], rotation: numpy.ndarray[numpy.float64[3, 3], flags.f_contiguous], azimuth_angles: numpy.ndarray[numpy.float64[m, 1]], elevation_angles: numpy.ndarray[numpy.float64[m, 1]], ignore_unknown: bool, max_range: float, prune_rays: bool, parallel: bool) -> dict

        2. cast_rays(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, positions: numpy.ndarray[numpy.float64[3, n], flags.f_contiguous], directions: numpy.ndarray[numpy.float64[3, n], flags.f_contiguous], ignore_unknown: bool, max_range: float, prune_rays: bool, parallel: bool) -> dict
        """
    @overload
    def cast_rays(self, positions: numpy.ndarray[numpy.float64[3, n], flags.f_contiguous], directions: numpy.ndarray[numpy.float64[3, n], flags.f_contiguous], ignore_unknown: bool, max_range: float, prune_rays: bool, parallel: bool) -> dict:
        """cast_rays(*args, **kwargs)
        Overloaded function.

        1. cast_rays(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, position: numpy.ndarray[numpy.float64[3, 1]], rotation: numpy.ndarray[numpy.float64[3, 3], flags.f_contiguous], azimuth_angles: numpy.ndarray[numpy.float64[m, 1]], elevation_angles: numpy.ndarray[numpy.float64[m, 1]], ignore_unknown: bool, max_range: float, prune_rays: bool, parallel: bool) -> dict

        2. cast_rays(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, positions: numpy.ndarray[numpy.float64[3, n], flags.f_contiguous], directions: numpy.ndarray[numpy.float64[3, n], flags.f_contiguous], ignore_unknown: bool, max_range: float, prune_rays: bool, parallel: bool) -> dict
        """
    def clear(self) -> None:
        """clear(self: erl_geometry.pyerl_geometry.OccupancyOctreeD) -> None"""
    def compute_bottom_neighbor_key(self, key: OctreeKey, depth: int) -> OctreeKey | None:
        """compute_bottom_neighbor_key(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]"""
    def compute_common_ancestor_key(self, arg0: OctreeKey, arg1: OctreeKey) -> tuple[OctreeKey, int]:
        """compute_common_ancestor_key(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, arg0: erl_geometry.pyerl_geometry.OctreeKey, arg1: erl_geometry.pyerl_geometry.OctreeKey) -> tuple[erl_geometry.pyerl_geometry.OctreeKey, int]"""
    def compute_east_neighbor_key(self, key: OctreeKey, depth: int) -> OctreeKey | None:
        """compute_east_neighbor_key(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]"""
    def compute_north_neighbor_key(self, key: OctreeKey, depth: int) -> OctreeKey | None:
        """compute_north_neighbor_key(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]"""
    def compute_ray_coords(self, sx: float, sy: float, sz: float, ex: float, ey: float, ez: float) -> list[numpy.ndarray[numpy.float64[3, 1]]] | None:
        """compute_ray_coords(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, sx: float, sy: float, sz: float, ex: float, ey: float, ez: float) -> Optional[list[numpy.ndarray[numpy.float64[3, 1]]]]"""
    def compute_ray_keys(self, sx: float, sy: float, sz: float, ex: float, ey: float, ez: float) -> list[OctreeKey] | None:
        """compute_ray_keys(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, sx: float, sy: float, sz: float, ex: float, ey: float, ez: float) -> Optional[list[erl_geometry.pyerl_geometry.OctreeKey]]"""
    def compute_south_neighbor_key(self, key: OctreeKey, depth: int) -> OctreeKey | None:
        """compute_south_neighbor_key(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]"""
    def compute_top_neighbor_key(self, key: OctreeKey, depth: int) -> OctreeKey | None:
        """compute_top_neighbor_key(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]"""
    def compute_west_neighbor_key(self, key: OctreeKey, depth: int) -> OctreeKey | None:
        """compute_west_neighbor_key(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]"""
    @overload
    def coord_to_key(self, coordinate: float) -> int:
        """coord_to_key(*args, **kwargs)
        Overloaded function.

        1. coord_to_key(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, coordinate: float) -> int

        2. coord_to_key(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, coordinate: float, depth: int) -> int

        3. coord_to_key(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, x: float, y: float, z: float) -> erl_geometry.pyerl_geometry.OctreeKey

        4. coord_to_key(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, x: float, y: float, z: float, depth: int) -> erl_geometry.pyerl_geometry.OctreeKey
        """
    @overload
    def coord_to_key(self, coordinate: float, depth: int) -> int:
        """coord_to_key(*args, **kwargs)
        Overloaded function.

        1. coord_to_key(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, coordinate: float) -> int

        2. coord_to_key(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, coordinate: float, depth: int) -> int

        3. coord_to_key(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, x: float, y: float, z: float) -> erl_geometry.pyerl_geometry.OctreeKey

        4. coord_to_key(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, x: float, y: float, z: float, depth: int) -> erl_geometry.pyerl_geometry.OctreeKey
        """
    @overload
    def coord_to_key(self, x: float, y: float, z: float) -> OctreeKey:
        """coord_to_key(*args, **kwargs)
        Overloaded function.

        1. coord_to_key(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, coordinate: float) -> int

        2. coord_to_key(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, coordinate: float, depth: int) -> int

        3. coord_to_key(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, x: float, y: float, z: float) -> erl_geometry.pyerl_geometry.OctreeKey

        4. coord_to_key(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, x: float, y: float, z: float, depth: int) -> erl_geometry.pyerl_geometry.OctreeKey
        """
    @overload
    def coord_to_key(self, x: float, y: float, z: float, depth: int) -> OctreeKey:
        """coord_to_key(*args, **kwargs)
        Overloaded function.

        1. coord_to_key(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, coordinate: float) -> int

        2. coord_to_key(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, coordinate: float, depth: int) -> int

        3. coord_to_key(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, x: float, y: float, z: float) -> erl_geometry.pyerl_geometry.OctreeKey

        4. coord_to_key(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, x: float, y: float, z: float, depth: int) -> erl_geometry.pyerl_geometry.OctreeKey
        """
    @overload
    def coord_to_key_checked(self, coordinate: float) -> int | None:
        """coord_to_key_checked(*args, **kwargs)
        Overloaded function.

        1. coord_to_key_checked(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, coordinate: float) -> Optional[int]

        2. coord_to_key_checked(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, coordinate: float, depth: int) -> Optional[int]

        3. coord_to_key_checked(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, x: float, y: float, z: float) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]

        4. coord_to_key_checked(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, x: float, y: float, z: float, depth: int) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]
        """
    @overload
    def coord_to_key_checked(self, coordinate: float, depth: int) -> int | None:
        """coord_to_key_checked(*args, **kwargs)
        Overloaded function.

        1. coord_to_key_checked(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, coordinate: float) -> Optional[int]

        2. coord_to_key_checked(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, coordinate: float, depth: int) -> Optional[int]

        3. coord_to_key_checked(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, x: float, y: float, z: float) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]

        4. coord_to_key_checked(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, x: float, y: float, z: float, depth: int) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]
        """
    @overload
    def coord_to_key_checked(self, x: float, y: float, z: float) -> OctreeKey | None:
        """coord_to_key_checked(*args, **kwargs)
        Overloaded function.

        1. coord_to_key_checked(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, coordinate: float) -> Optional[int]

        2. coord_to_key_checked(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, coordinate: float, depth: int) -> Optional[int]

        3. coord_to_key_checked(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, x: float, y: float, z: float) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]

        4. coord_to_key_checked(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, x: float, y: float, z: float, depth: int) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]
        """
    @overload
    def coord_to_key_checked(self, x: float, y: float, z: float, depth: int) -> OctreeKey | None:
        """coord_to_key_checked(*args, **kwargs)
        Overloaded function.

        1. coord_to_key_checked(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, coordinate: float) -> Optional[int]

        2. coord_to_key_checked(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, coordinate: float, depth: int) -> Optional[int]

        3. coord_to_key_checked(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, x: float, y: float, z: float) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]

        4. coord_to_key_checked(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, x: float, y: float, z: float, depth: int) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]
        """
    def create_node_child(self, node: OccupancyOctreeNode, child_idx: int) -> OccupancyOctreeNode:
        """create_node_child(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, node: erl_geometry.pyerl_geometry.OccupancyOctreeNode, child_idx: int) -> erl_geometry.pyerl_geometry.OccupancyOctreeNode"""
    @overload
    def delete_node(self, x: float, y: float, z: float, depth: int) -> int:
        """delete_node(*args, **kwargs)
        Overloaded function.

        1. delete_node(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, x: float, y: float, z: float, depth: int) -> int

        2. delete_node(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> None
        """
    @overload
    def delete_node(self, key: OctreeKey, depth: int) -> None:
        """delete_node(*args, **kwargs)
        Overloaded function.

        1. delete_node(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, x: float, y: float, z: float, depth: int) -> int

        2. delete_node(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> None
        """
    def delete_node_child(self, node: OccupancyOctreeNode, child_idx: int, key: OctreeKey) -> int:
        """delete_node_child(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, node: erl_geometry.pyerl_geometry.OccupancyOctreeNode, child_idx: int, key: erl_geometry.pyerl_geometry.OctreeKey) -> int"""
    def expand(self) -> None:
        """expand(self: erl_geometry.pyerl_geometry.OccupancyOctreeD) -> None"""
    def expand_node(self, node: OccupancyOctreeNode) -> None:
        """expand_node(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, node: erl_geometry.pyerl_geometry.OccupancyOctreeNode) -> None"""
    def get_batch_ray_caster(self, *args, **kwargs):
        """get_batch_ray_caster(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, origins: numpy.ndarray[numpy.float64[3, n]], directions: numpy.ndarray[numpy.float64[3, n]], max_ranges: numpy.ndarray[numpy.float64[m, 1]] = array([], dtype=float64), node_paddings: numpy.ndarray[numpy.float64[m, 1]] = array([], dtype=float64), bidirectional_flags: numpy.ndarray[bool[m, 1]] = array([], dtype=bool), leaf_only_flags: numpy.ndarray[bool[m, 1]] = array([], dtype=bool), min_node_depths: numpy.ndarray[numpy.int32[m, 1]] = array([], dtype=int32), max_node_depths: numpy.ndarray[numpy.int32[m, 1]] = array([], dtype=int32)) -> erl::geometry::OccupancyNdTreeBatchRayCaster<erl::geometry::OccupancyOctreeBase<double, erl::geometry::OccupancyOctreeNode, erl::geometry::OccupancyOctreeBaseSetting>, 3>"""
    def get_node_child(self, node: OccupancyOctreeNode, child_idx: int) -> OccupancyOctreeNode:
        """get_node_child(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, node: erl_geometry.pyerl_geometry.OccupancyOctreeNode, child_idx: int) -> erl_geometry.pyerl_geometry.OccupancyOctreeNode"""
    def get_node_size(self, depth: int) -> float:
        """get_node_size(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, depth: int) -> float"""
    @overload
    def insert_node(self, x: float, y: float, z: float, depth: int) -> OccupancyOctreeNode:
        """insert_node(*args, **kwargs)
        Overloaded function.

        1. insert_node(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, x: float, y: float, z: float, depth: int) -> erl_geometry.pyerl_geometry.OccupancyOctreeNode

        2. insert_node(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> erl_geometry.pyerl_geometry.OccupancyOctreeNode
        """
    @overload
    def insert_node(self, key: OctreeKey, depth: int) -> OccupancyOctreeNode:
        """insert_node(*args, **kwargs)
        Overloaded function.

        1. insert_node(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, x: float, y: float, z: float, depth: int) -> erl_geometry.pyerl_geometry.OccupancyOctreeNode

        2. insert_node(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> erl_geometry.pyerl_geometry.OccupancyOctreeNode
        """
    def insert_point_cloud(self, points: numpy.ndarray[numpy.float64[3, n], flags.f_contiguous], sensor_origin: numpy.ndarray[numpy.float64[3, 1]], min_range: float, max_range: float, with_count: bool, parallel: bool, lazy_eval: bool, discrete: bool) -> None:
        """insert_point_cloud(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, points: numpy.ndarray[numpy.float64[3, n], flags.f_contiguous], sensor_origin: numpy.ndarray[numpy.float64[3, 1]], min_range: float, max_range: float, with_count: bool, parallel: bool, lazy_eval: bool, discrete: bool) -> None"""
    def insert_point_cloud_rays(self, points: numpy.ndarray[numpy.float64[3, n], flags.f_contiguous], sensor_origin: numpy.ndarray[numpy.float64[3, 1]], min_range: float, max_range: float, parallel: bool, lazy_eval: bool) -> None:
        """insert_point_cloud_rays(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, points: numpy.ndarray[numpy.float64[3, n], flags.f_contiguous], sensor_origin: numpy.ndarray[numpy.float64[3, 1]], min_range: float, max_range: float, parallel: bool, lazy_eval: bool) -> None"""
    def insert_ray(self, sx: float, sy: float, sz: float, ex: float, ey: float, ez: float, min_range: float, max_range: float, lazy_eval: bool) -> bool:
        """insert_ray(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, sx: float, sy: float, sz: float, ex: float, ey: float, ez: float, min_range: float, max_range: float, lazy_eval: bool) -> bool"""
    def is_node_collapsible(self, node: OccupancyOctreeNode) -> bool:
        """is_node_collapsible(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, node: erl_geometry.pyerl_geometry.OccupancyOctreeNode) -> bool"""
    @overload
    def iter_bottom_leaf_neighbor(self, x: float, y: float, z: float, max_leaf_depth: int = ...) -> Iterator[OccupancyOctreeD.BottomLeafNeighborIterator]:
        """iter_bottom_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_bottom_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeD.BottomLeafNeighborIterator]

        2. iter_bottom_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeD.BottomLeafNeighborIterator]
        """
    @overload
    def iter_bottom_leaf_neighbor(self, key: OctreeKey, key_depth: int, max_leaf_depth: int = ...) -> Iterator[OccupancyOctreeD.BottomLeafNeighborIterator]:
        """iter_bottom_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_bottom_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeD.BottomLeafNeighborIterator]

        2. iter_bottom_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeD.BottomLeafNeighborIterator]
        """
    @overload
    def iter_east_leaf_neighbor(self, x: float, y: float, z: float, max_leaf_depth: int = ...) -> Iterator[OccupancyOctreeD.EastLeafNeighborIterator]:
        """iter_east_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_east_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeD.EastLeafNeighborIterator]

        2. iter_east_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeD.EastLeafNeighborIterator]
        """
    @overload
    def iter_east_leaf_neighbor(self, key: OctreeKey, key_depth: int, max_leaf_depth: int = ...) -> Iterator[OccupancyOctreeD.EastLeafNeighborIterator]:
        """iter_east_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_east_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeD.EastLeafNeighborIterator]

        2. iter_east_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeD.EastLeafNeighborIterator]
        """
    def iter_leaf(self, max_depth: int = ...) -> Iterator[OccupancyOctreeD.LeafIterator]:
        """iter_leaf(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeD.LeafIterator]"""
    @overload
    def iter_leaf_in_aabb(self, aabb_min_x: float, aabb_min_y: float, aabb_min_z: float, aabb_max_x: float, aabb_max_y: float, aabb_max_z: float, max_depth: int = ...) -> Iterator[OccupancyOctreeD.LeafInAabbIterator]:
        """iter_leaf_in_aabb(*args, **kwargs)
        Overloaded function.

        1. iter_leaf_in_aabb(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, aabb_min_x: float, aabb_min_y: float, aabb_min_z: float, aabb_max_x: float, aabb_max_y: float, aabb_max_z: float, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeD.LeafInAabbIterator]

        2. iter_leaf_in_aabb(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, aabb_min_key: erl_geometry.pyerl_geometry.OctreeKey, aabb_max_key: erl_geometry.pyerl_geometry.OctreeKey, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeD.LeafInAabbIterator]
        """
    @overload
    def iter_leaf_in_aabb(self, aabb_min_key: OctreeKey, aabb_max_key: OctreeKey, max_depth: int = ...) -> Iterator[OccupancyOctreeD.LeafInAabbIterator]:
        """iter_leaf_in_aabb(*args, **kwargs)
        Overloaded function.

        1. iter_leaf_in_aabb(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, aabb_min_x: float, aabb_min_y: float, aabb_min_z: float, aabb_max_x: float, aabb_max_y: float, aabb_max_z: float, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeD.LeafInAabbIterator]

        2. iter_leaf_in_aabb(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, aabb_min_key: erl_geometry.pyerl_geometry.OctreeKey, aabb_max_key: erl_geometry.pyerl_geometry.OctreeKey, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeD.LeafInAabbIterator]
        """
    def iter_leaf_of_node(self, node_key: OctreeKey, node_depth: int, max_depth: int = ...) -> Iterator[OccupancyOctreeD.LeafOfNodeIterator]:
        """iter_leaf_of_node(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, node_key: erl_geometry.pyerl_geometry.OctreeKey, node_depth: int, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeD.LeafOfNodeIterator]"""
    def iter_node(self, max_depth: int = ...) -> Iterator[OccupancyOctreeD.TreeIterator]:
        """iter_node(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeD.TreeIterator]"""
    @overload
    def iter_node_in_aabb(self, aabb_min_x: float, aabb_min_y: float, aabb_min_z: float, aabb_max_x: float, aabb_max_y: float, aabb_max_z: float, max_depth: int = ...) -> Iterator[OccupancyOctreeD.TreeInAabbIterator]:
        """iter_node_in_aabb(*args, **kwargs)
        Overloaded function.

        1. iter_node_in_aabb(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, aabb_min_x: float, aabb_min_y: float, aabb_min_z: float, aabb_max_x: float, aabb_max_y: float, aabb_max_z: float, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeD.TreeInAabbIterator]

        2. iter_node_in_aabb(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, aabb_min_key: erl_geometry.pyerl_geometry.OctreeKey, aabb_max_key: erl_geometry.pyerl_geometry.OctreeKey, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeD.TreeInAabbIterator]
        """
    @overload
    def iter_node_in_aabb(self, aabb_min_key: OctreeKey, aabb_max_key: OctreeKey, max_depth: int = ...) -> Iterator[OccupancyOctreeD.TreeInAabbIterator]:
        """iter_node_in_aabb(*args, **kwargs)
        Overloaded function.

        1. iter_node_in_aabb(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, aabb_min_x: float, aabb_min_y: float, aabb_min_z: float, aabb_max_x: float, aabb_max_y: float, aabb_max_z: float, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeD.TreeInAabbIterator]

        2. iter_node_in_aabb(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, aabb_min_key: erl_geometry.pyerl_geometry.OctreeKey, aabb_max_key: erl_geometry.pyerl_geometry.OctreeKey, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeD.TreeInAabbIterator]
        """
    def iter_node_on_ray(self, px: float, py: float, pz: float, vx: float, vy: float, vz: float, max_range: float = ..., node_padding: float = ..., bidirectional: bool = ..., leaf_only: bool = ..., min_node_depth: int = ..., max_node_depth: int = ...) -> Iterator[OccupancyOctreeD.NodeOnRayIterator]:
        """iter_node_on_ray(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, px: float, py: float, pz: float, vx: float, vy: float, vz: float, max_range: float = -1, node_padding: float = 0, bidirectional: bool = False, leaf_only: bool = True, min_node_depth: int = 0, max_node_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeD.NodeOnRayIterator]"""
    @overload
    def iter_north_leaf_neighbor(self, x: float, y: float, z: float, max_leaf_depth: int = ...) -> Iterator[OccupancyOctreeD.NorthLeafNeighborIterator]:
        """iter_north_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_north_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeD.NorthLeafNeighborIterator]

        2. iter_north_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeD.NorthLeafNeighborIterator]
        """
    @overload
    def iter_north_leaf_neighbor(self, key: OctreeKey, key_depth: int, max_leaf_depth: int = ...) -> Iterator[OccupancyOctreeD.NorthLeafNeighborIterator]:
        """iter_north_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_north_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeD.NorthLeafNeighborIterator]

        2. iter_north_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeD.NorthLeafNeighborIterator]
        """
    @overload
    def iter_south_leaf_neighbor(self, x: float, y: float, z: float, max_leaf_depth: int = ...) -> Iterator[OccupancyOctreeD.SouthLeafNeighborIterator]:
        """iter_south_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_south_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeD.SouthLeafNeighborIterator]

        2. iter_south_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeD.SouthLeafNeighborIterator]
        """
    @overload
    def iter_south_leaf_neighbor(self, key: OctreeKey, key_depth: int, max_leaf_depth: int = ...) -> Iterator[OccupancyOctreeD.SouthLeafNeighborIterator]:
        """iter_south_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_south_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeD.SouthLeafNeighborIterator]

        2. iter_south_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeD.SouthLeafNeighborIterator]
        """
    @overload
    def iter_top_leaf_neighbor(self, x: float, y: float, z: float, max_leaf_depth: int = ...) -> Iterator[OccupancyOctreeD.TopLeafNeighborIterator]:
        """iter_top_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_top_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeD.TopLeafNeighborIterator]

        2. iter_top_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeD.TopLeafNeighborIterator]
        """
    @overload
    def iter_top_leaf_neighbor(self, key: OctreeKey, key_depth: int, max_leaf_depth: int = ...) -> Iterator[OccupancyOctreeD.TopLeafNeighborIterator]:
        """iter_top_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_top_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeD.TopLeafNeighborIterator]

        2. iter_top_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeD.TopLeafNeighborIterator]
        """
    @overload
    def iter_west_leaf_neighbor(self, x: float, y: float, z: float, max_leaf_depth: int = ...) -> Iterator[OccupancyOctreeD.WestLeafNeighborIterator]:
        """iter_west_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_west_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeD.WestLeafNeighborIterator]

        2. iter_west_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeD.WestLeafNeighborIterator]
        """
    @overload
    def iter_west_leaf_neighbor(self, key: OctreeKey, key_depth: int, max_leaf_depth: int = ...) -> Iterator[OccupancyOctreeD.WestLeafNeighborIterator]:
        """iter_west_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_west_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeD.WestLeafNeighborIterator]

        2. iter_west_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeD.WestLeafNeighborIterator]
        """
    @overload
    def key_to_coord(self, key: int) -> float:
        """key_to_coord(*args, **kwargs)
        Overloaded function.

        1. key_to_coord(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, key: int) -> float

        2. key_to_coord(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, key: int, depth: int) -> float

        3. key_to_coord(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey) -> tuple[float, float, float]

        4. key_to_coord(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> tuple[float, float, float]
        """
    @overload
    def key_to_coord(self, key: int, depth: int) -> float:
        """key_to_coord(*args, **kwargs)
        Overloaded function.

        1. key_to_coord(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, key: int) -> float

        2. key_to_coord(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, key: int, depth: int) -> float

        3. key_to_coord(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey) -> tuple[float, float, float]

        4. key_to_coord(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> tuple[float, float, float]
        """
    @overload
    def key_to_coord(self, key: OctreeKey) -> tuple[float, float, float]:
        """key_to_coord(*args, **kwargs)
        Overloaded function.

        1. key_to_coord(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, key: int) -> float

        2. key_to_coord(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, key: int, depth: int) -> float

        3. key_to_coord(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey) -> tuple[float, float, float]

        4. key_to_coord(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> tuple[float, float, float]
        """
    @overload
    def key_to_coord(self, key: OctreeKey, depth: int) -> tuple[float, float, float]:
        """key_to_coord(*args, **kwargs)
        Overloaded function.

        1. key_to_coord(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, key: int) -> float

        2. key_to_coord(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, key: int, depth: int) -> float

        3. key_to_coord(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey) -> tuple[float, float, float]

        4. key_to_coord(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> tuple[float, float, float]
        """
    def prune(self) -> None:
        """prune(self: erl_geometry.pyerl_geometry.OccupancyOctreeD) -> None"""
    def prune_node(self, node: OccupancyOctreeNode) -> bool:
        """prune_node(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, node: erl_geometry.pyerl_geometry.OccupancyOctreeNode) -> bool"""
    def sample_positions(self, num_positions: int) -> list[numpy.ndarray[numpy.float64[3, 1]]]:
        """sample_positions(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, num_positions: int) -> list[numpy.ndarray[numpy.float64[3, 1]]]"""
    @overload
    def search(self, x: float, y: float, z: float, max_depth: int = ...) -> OccupancyOctreeNode:
        """search(*args, **kwargs)
        Overloaded function.

        1. search(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, x: float, y: float, z: float, max_depth: int = 0) -> erl_geometry.pyerl_geometry.OccupancyOctreeNode

        2. search(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, max_depth: int = 0) -> erl_geometry.pyerl_geometry.OccupancyOctreeNode
        """
    @overload
    def search(self, key: OctreeKey, max_depth: int = ...) -> OccupancyOctreeNode:
        """search(*args, **kwargs)
        Overloaded function.

        1. search(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, x: float, y: float, z: float, max_depth: int = 0) -> erl_geometry.pyerl_geometry.OccupancyOctreeNode

        2. search(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, max_depth: int = 0) -> erl_geometry.pyerl_geometry.OccupancyOctreeNode
        """
    def to_max_likelihood(self) -> None:
        """to_max_likelihood(self: erl_geometry.pyerl_geometry.OccupancyOctreeD) -> None"""
    def update_inner_occupancy(self) -> None:
        """update_inner_occupancy(self: erl_geometry.pyerl_geometry.OccupancyOctreeD) -> None"""
    @overload
    def update_node(self, x: float, y: float, z: float, occupied: bool, lazy_eval: bool) -> OccupancyOctreeNode:
        """update_node(*args, **kwargs)
        Overloaded function.

        1. update_node(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, x: float, y: float, z: float, occupied: bool, lazy_eval: bool) -> erl_geometry.pyerl_geometry.OccupancyOctreeNode

        2. update_node(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, node_key: erl_geometry.pyerl_geometry.OctreeKey, occupied: bool, lazy_eval: bool) -> erl_geometry.pyerl_geometry.OccupancyOctreeNode

        3. update_node(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, x: float, y: float, z: float, log_odds_delta: float, lazy_eval: bool) -> erl_geometry.pyerl_geometry.OccupancyOctreeNode

        4. update_node(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, node_key: erl_geometry.pyerl_geometry.OctreeKey, log_odds_delta: float, lazy_eval: bool) -> erl_geometry.pyerl_geometry.OccupancyOctreeNode
        """
    @overload
    def update_node(self, node_key: OctreeKey, occupied: bool, lazy_eval: bool) -> OccupancyOctreeNode:
        """update_node(*args, **kwargs)
        Overloaded function.

        1. update_node(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, x: float, y: float, z: float, occupied: bool, lazy_eval: bool) -> erl_geometry.pyerl_geometry.OccupancyOctreeNode

        2. update_node(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, node_key: erl_geometry.pyerl_geometry.OctreeKey, occupied: bool, lazy_eval: bool) -> erl_geometry.pyerl_geometry.OccupancyOctreeNode

        3. update_node(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, x: float, y: float, z: float, log_odds_delta: float, lazy_eval: bool) -> erl_geometry.pyerl_geometry.OccupancyOctreeNode

        4. update_node(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, node_key: erl_geometry.pyerl_geometry.OctreeKey, log_odds_delta: float, lazy_eval: bool) -> erl_geometry.pyerl_geometry.OccupancyOctreeNode
        """
    @overload
    def update_node(self, x: float, y: float, z: float, log_odds_delta: float, lazy_eval: bool) -> OccupancyOctreeNode:
        """update_node(*args, **kwargs)
        Overloaded function.

        1. update_node(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, x: float, y: float, z: float, occupied: bool, lazy_eval: bool) -> erl_geometry.pyerl_geometry.OccupancyOctreeNode

        2. update_node(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, node_key: erl_geometry.pyerl_geometry.OctreeKey, occupied: bool, lazy_eval: bool) -> erl_geometry.pyerl_geometry.OccupancyOctreeNode

        3. update_node(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, x: float, y: float, z: float, log_odds_delta: float, lazy_eval: bool) -> erl_geometry.pyerl_geometry.OccupancyOctreeNode

        4. update_node(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, node_key: erl_geometry.pyerl_geometry.OctreeKey, log_odds_delta: float, lazy_eval: bool) -> erl_geometry.pyerl_geometry.OccupancyOctreeNode
        """
    @overload
    def update_node(self, node_key: OctreeKey, log_odds_delta: float, lazy_eval: bool) -> OccupancyOctreeNode:
        """update_node(*args, **kwargs)
        Overloaded function.

        1. update_node(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, x: float, y: float, z: float, occupied: bool, lazy_eval: bool) -> erl_geometry.pyerl_geometry.OccupancyOctreeNode

        2. update_node(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, node_key: erl_geometry.pyerl_geometry.OctreeKey, occupied: bool, lazy_eval: bool) -> erl_geometry.pyerl_geometry.OccupancyOctreeNode

        3. update_node(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, x: float, y: float, z: float, log_odds_delta: float, lazy_eval: bool) -> erl_geometry.pyerl_geometry.OccupancyOctreeNode

        4. update_node(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, node_key: erl_geometry.pyerl_geometry.OctreeKey, log_odds_delta: float, lazy_eval: bool) -> erl_geometry.pyerl_geometry.OccupancyOctreeNode
        """
    def visualize(self, leaf_only: bool = ..., scaling: float = ..., area_min: numpy.ndarray[numpy.float64[3, 1]] = ..., area_max: numpy.ndarray[numpy.float64[3, 1]] = ..., border_color: numpy.ndarray[numpy.float64[3, 1]] = ..., occupied_color: numpy.ndarray[numpy.float64[3, 1]] = ..., occupied_only: bool = ..., draw_node_boxes: bool = ..., draw_node_borders: bool = ..., window_width: int = ..., window_height: int = ..., window_left: int = ..., window_top: int = ...) -> None:
        """visualize(self: erl_geometry.pyerl_geometry.OccupancyOctreeD, leaf_only: bool = False, scaling: float = 1.0, area_min: numpy.ndarray[numpy.float64[3, 1]] = array([-1., -1., -1.]), area_max: numpy.ndarray[numpy.float64[3, 1]] = array([1., 1., 1.]), border_color: numpy.ndarray[numpy.float64[3, 1]] = array([0., 0., 0.]), occupied_color: numpy.ndarray[numpy.float64[3, 1]] = array([0.5, 0.5, 0.5]), occupied_only: bool = False, draw_node_boxes: bool = True, draw_node_borders: bool = True, window_width: int = 1920, window_height: int = 1080, window_left: int = 50, window_top: int = 50) -> None"""
    @property
    def memory_usage(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeD) -> int"""
    @property
    def memory_usage_per_node(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeD) -> int"""
    @property
    def metric_aabb(self) -> Aabb3Dd:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeD) -> erl_geometry.pyerl_geometry.Aabb3Dd"""
    @property
    def metric_max(self) -> numpy.ndarray[numpy.float64[3, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeD) -> numpy.ndarray[numpy.float64[3, 1]]"""
    @property
    def metric_min(self) -> numpy.ndarray[numpy.float64[3, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeD) -> numpy.ndarray[numpy.float64[3, 1]]"""
    @property
    def metric_min_max(self) -> tuple[numpy.ndarray[numpy.float64[3, 1]], numpy.ndarray[numpy.float64[3, 1]]]:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeD) -> tuple[numpy.ndarray[numpy.float64[3, 1]], numpy.ndarray[numpy.float64[3, 1]]]"""
    @property
    def metric_size(self) -> numpy.ndarray[numpy.float64[3, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeD) -> numpy.ndarray[numpy.float64[3, 1]]"""
    @property
    def number_of_leaf_nodes(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeD) -> int"""
    @property
    def number_of_nodes(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeD) -> int"""
    @property
    def resolution(self) -> float:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeD) -> float"""
    @property
    def root(self) -> OccupancyOctreeNode:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeD) -> erl_geometry.pyerl_geometry.OccupancyOctreeNode"""
    @property
    def setting(self) -> OccupancyOctreeBaseSetting:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeD) -> erl_geometry.pyerl_geometry.OccupancyOctreeBaseSetting"""
    @property
    def tree_center(self) -> numpy.ndarray[numpy.float64[3, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeD) -> numpy.ndarray[numpy.float64[3, 1]]"""
    @property
    def tree_center_key(self) -> OctreeKey:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeD) -> erl_geometry.pyerl_geometry.OctreeKey"""
    @property
    def tree_depth(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeD) -> int"""
    @property
    def tree_max_half_size(self) -> numpy.ndarray[numpy.float64[3, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeD) -> numpy.ndarray[numpy.float64[3, 1]]"""

class OccupancyOctreeDrawerSetting(YamlableBase):
    area_max: numpy.ndarray[numpy.float64[3, 1]]
    area_min: numpy.ndarray[numpy.float64[3, 1]]
    border_color: numpy.ndarray[numpy.float64[3, 1]]
    draw_node_borders: bool
    draw_node_boxes: bool
    occupied_color: numpy.ndarray[numpy.float64[3, 1]]
    occupied_only: bool
    def __init__(self) -> None:
        """__init__(self: erl_geometry.pyerl_geometry.OccupancyOctreeDrawerSetting) -> None"""

class OccupancyOctreeF(AbstractOccupancyOctreeF):
    class BatchRayCaster:
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""
        def step(self, max_depth: numpy.ndarray[bool[m, 1]] = ...) -> OccupancyOctreeF.BatchRayCaster:
            """step(self: erl_geometry.pyerl_geometry.OccupancyOctreeF.BatchRayCaster, max_depth: numpy.ndarray[bool[m, 1]] = 0) -> erl_geometry.pyerl_geometry.OccupancyOctreeF.BatchRayCaster"""
        @property
        def ever_hit_flags(self) -> numpy.ndarray[bool[m, 1]]:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeF.BatchRayCaster) -> numpy.ndarray[bool[m, 1]]"""
        @property
        def frontier_keys(self) -> list[OctreeKey]:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeF.BatchRayCaster) -> list[erl_geometry.pyerl_geometry.OctreeKey]"""
        @property
        def frontier_nodes(self) -> list[OccupancyOctreeNode]:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeF.BatchRayCaster) -> list[erl_geometry.pyerl_geometry.OccupancyOctreeNode]"""
        @property
        def frontier_ray_indices(self) -> list[list[int]]:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeF.BatchRayCaster) -> list[list[int]]"""
        @property
        def hit_distances(self) -> numpy.ndarray[numpy.float32[m, 1]]:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeF.BatchRayCaster) -> numpy.ndarray[numpy.float32[m, 1]]"""
        @property
        def hit_flags(self) -> numpy.ndarray[bool[m, 1]]:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeF.BatchRayCaster) -> numpy.ndarray[bool[m, 1]]"""
        @property
        def hit_nodes(self) -> list[OccupancyOctreeNode]:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeF.BatchRayCaster) -> list[erl_geometry.pyerl_geometry.OccupancyOctreeNode]"""
        @property
        def hit_positions(self) -> list[numpy.ndarray[numpy.float32[3, 1]]]:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeF.BatchRayCaster) -> list[numpy.ndarray[numpy.float32[3, 1]]]"""
        @property
        def num_rays(self) -> int:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeF.BatchRayCaster) -> int"""
        @property
        def ray_directions(self) -> numpy.ndarray[numpy.float32[3, n]]:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeF.BatchRayCaster) -> numpy.ndarray[numpy.float32[3, n]]"""
        @property
        def ray_origins(self) -> numpy.ndarray[numpy.float32[3, n]]:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeF.BatchRayCaster) -> numpy.ndarray[numpy.float32[3, n]]"""

    class BottomLeafNeighborIterator(OccupancyOctreeF.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class Drawer:
        def __init__(self, setting: OccupancyOctreeDrawerSetting, octree: OccupancyOctreeF = ...) -> None:
            """__init__(self: erl_geometry.pyerl_geometry.OccupancyOctreeF.Drawer, setting: erl_geometry.pyerl_geometry.OccupancyOctreeDrawerSetting, octree: erl_geometry.pyerl_geometry.OccupancyOctreeF = None) -> None"""
        @staticmethod
        def Setting() -> OccupancyOctreeDrawerSetting:
            """Setting() -> erl_geometry.pyerl_geometry.OccupancyOctreeDrawerSetting"""
        def draw_leaves(self, filename: str) -> None:
            """draw_leaves(*args, **kwargs)
            Overloaded function.

            1. draw_leaves(self: erl_geometry.pyerl_geometry.OccupancyOctreeF.Drawer) -> list[open3d::geometry::Geometry]

            2. draw_leaves(self: erl_geometry.pyerl_geometry.OccupancyOctreeF.Drawer, filename: str) -> None
            """
        def draw_tree(self, filename: str) -> None:
            """draw_tree(*args, **kwargs)
            Overloaded function.

            1. draw_tree(self: erl_geometry.pyerl_geometry.OccupancyOctreeF.Drawer) -> list[open3d::geometry::Geometry]

            2. draw_tree(self: erl_geometry.pyerl_geometry.OccupancyOctreeF.Drawer, filename: str) -> None
            """
        def set_draw_leaf_callback(self, callback) -> None:
            """set_draw_leaf_callback(self: erl_geometry.pyerl_geometry.OccupancyOctreeF.Drawer, callback: Callable[[erl_geometry.pyerl_geometry.OccupancyOctreeF.Drawer, list[open3d::geometry::Geometry], erl_geometry.pyerl_geometry.OccupancyOctreeF.LeafInAabbIterator], None]) -> None"""
        def set_draw_tree_callback(self, callback) -> None:
            """set_draw_tree_callback(self: erl_geometry.pyerl_geometry.OccupancyOctreeF.Drawer, callback: Callable[[erl_geometry.pyerl_geometry.OccupancyOctreeF.Drawer, list[open3d::geometry::Geometry], erl_geometry.pyerl_geometry.OccupancyOctreeF.TreeInAabbIterator], None]) -> None"""
        @property
        def setting(self) -> OccupancyOctreeDrawerSetting:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeF.Drawer) -> erl_geometry.pyerl_geometry.OccupancyOctreeDrawerSetting"""

    class EastLeafNeighborIterator(OccupancyOctreeF.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class IteratorBase(AbstractOctreeF.OctreeNodeIterator):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""
        def __eq__(self, arg0: OccupancyOctreeF.IteratorBase) -> bool:
            """__eq__(self: erl_geometry.pyerl_geometry.OccupancyOctreeF.IteratorBase, arg0: erl_geometry.pyerl_geometry.OccupancyOctreeF.IteratorBase) -> bool"""
        def __ne__(self, arg0: OccupancyOctreeF.IteratorBase) -> bool:
            """__ne__(self: erl_geometry.pyerl_geometry.OccupancyOctreeF.IteratorBase, arg0: erl_geometry.pyerl_geometry.OccupancyOctreeF.IteratorBase) -> bool"""
        @property
        def index_key(self) -> OctreeKey:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeF.IteratorBase) -> erl_geometry.pyerl_geometry.OctreeKey"""
        @property
        def key(self) -> OctreeKey:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeF.IteratorBase) -> erl_geometry.pyerl_geometry.OctreeKey"""
        @property
        def node(self) -> OccupancyOctreeNode:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeF.IteratorBase) -> erl_geometry.pyerl_geometry.OccupancyOctreeNode"""
        @property
        def node_aabb(self) -> Aabb3Df:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeF.IteratorBase) -> erl_geometry.pyerl_geometry.Aabb3Df"""

    class LeafInAabbIterator(OccupancyOctreeF.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class LeafIterator(OccupancyOctreeF.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class LeafOfNodeIterator(OccupancyOctreeF.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class NodeOnRayIterator(OccupancyOctreeF.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""
        @property
        def distance(self) -> float:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeF.NodeOnRayIterator) -> float"""

    class NorthLeafNeighborIterator(OccupancyOctreeF.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class SouthLeafNeighborIterator(OccupancyOctreeF.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class TopLeafNeighborIterator(OccupancyOctreeF.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class TreeInAabbIterator(OccupancyOctreeF.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class TreeIterator(OccupancyOctreeF.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class WestLeafNeighborIterator(OccupancyOctreeF.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""
    @overload
    def __init__(self) -> None:
        """__init__(*args, **kwargs)
        Overloaded function.

        1. __init__(self: erl_geometry.pyerl_geometry.OccupancyOctreeF) -> None

        2. __init__(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, setting: erl_geometry.pyerl_geometry.OccupancyOctreeBaseSetting) -> None

        3. __init__(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, filename: str) -> None
        """
    @overload
    def __init__(self, setting: OccupancyOctreeBaseSetting) -> None:
        """__init__(*args, **kwargs)
        Overloaded function.

        1. __init__(self: erl_geometry.pyerl_geometry.OccupancyOctreeF) -> None

        2. __init__(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, setting: erl_geometry.pyerl_geometry.OccupancyOctreeBaseSetting) -> None

        3. __init__(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, filename: str) -> None
        """
    @overload
    def __init__(self, filename: str) -> None:
        """__init__(*args, **kwargs)
        Overloaded function.

        1. __init__(self: erl_geometry.pyerl_geometry.OccupancyOctreeF) -> None

        2. __init__(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, setting: erl_geometry.pyerl_geometry.OccupancyOctreeBaseSetting) -> None

        3. __init__(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, filename: str) -> None
        """
    def Setting(self) -> OccupancyOctreeBaseSetting:
        """Setting() -> erl_geometry.pyerl_geometry.OccupancyOctreeBaseSetting"""
    @overload
    def adjust_key_to_depth(self, key: int, depth: int) -> int:
        """adjust_key_to_depth(*args, **kwargs)
        Overloaded function.

        1. adjust_key_to_depth(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, key: int, depth: int) -> int

        2. adjust_key_to_depth(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> erl_geometry.pyerl_geometry.OctreeKey
        """
    @overload
    def adjust_key_to_depth(self, key: OctreeKey, depth: int) -> OctreeKey:
        """adjust_key_to_depth(*args, **kwargs)
        Overloaded function.

        1. adjust_key_to_depth(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, key: int, depth: int) -> int

        2. adjust_key_to_depth(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> erl_geometry.pyerl_geometry.OctreeKey
        """
    def cast_ray(self, px: float, py: float, pz: float, vx: float, vy: float, vz: float, ignore_unknown: bool, max_range: float) -> dict:
        """cast_ray(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, px: float, py: float, pz: float, vx: float, vy: float, vz: float, ignore_unknown: bool, max_range: float) -> dict"""
    @overload
    def cast_rays(self, position: numpy.ndarray[numpy.float32[3, 1]], rotation: numpy.ndarray[numpy.float32[3, 3], flags.f_contiguous], azimuth_angles: numpy.ndarray[numpy.float32[m, 1]], elevation_angles: numpy.ndarray[numpy.float32[m, 1]], ignore_unknown: bool, max_range: float, prune_rays: bool, parallel: bool) -> dict:
        """cast_rays(*args, **kwargs)
        Overloaded function.

        1. cast_rays(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, position: numpy.ndarray[numpy.float32[3, 1]], rotation: numpy.ndarray[numpy.float32[3, 3], flags.f_contiguous], azimuth_angles: numpy.ndarray[numpy.float32[m, 1]], elevation_angles: numpy.ndarray[numpy.float32[m, 1]], ignore_unknown: bool, max_range: float, prune_rays: bool, parallel: bool) -> dict

        2. cast_rays(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, positions: numpy.ndarray[numpy.float32[3, n], flags.f_contiguous], directions: numpy.ndarray[numpy.float32[3, n], flags.f_contiguous], ignore_unknown: bool, max_range: float, prune_rays: bool, parallel: bool) -> dict
        """
    @overload
    def cast_rays(self, positions: numpy.ndarray[numpy.float32[3, n], flags.f_contiguous], directions: numpy.ndarray[numpy.float32[3, n], flags.f_contiguous], ignore_unknown: bool, max_range: float, prune_rays: bool, parallel: bool) -> dict:
        """cast_rays(*args, **kwargs)
        Overloaded function.

        1. cast_rays(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, position: numpy.ndarray[numpy.float32[3, 1]], rotation: numpy.ndarray[numpy.float32[3, 3], flags.f_contiguous], azimuth_angles: numpy.ndarray[numpy.float32[m, 1]], elevation_angles: numpy.ndarray[numpy.float32[m, 1]], ignore_unknown: bool, max_range: float, prune_rays: bool, parallel: bool) -> dict

        2. cast_rays(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, positions: numpy.ndarray[numpy.float32[3, n], flags.f_contiguous], directions: numpy.ndarray[numpy.float32[3, n], flags.f_contiguous], ignore_unknown: bool, max_range: float, prune_rays: bool, parallel: bool) -> dict
        """
    def clear(self) -> None:
        """clear(self: erl_geometry.pyerl_geometry.OccupancyOctreeF) -> None"""
    def compute_bottom_neighbor_key(self, key: OctreeKey, depth: int) -> OctreeKey | None:
        """compute_bottom_neighbor_key(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]"""
    def compute_common_ancestor_key(self, arg0: OctreeKey, arg1: OctreeKey) -> tuple[OctreeKey, int]:
        """compute_common_ancestor_key(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, arg0: erl_geometry.pyerl_geometry.OctreeKey, arg1: erl_geometry.pyerl_geometry.OctreeKey) -> tuple[erl_geometry.pyerl_geometry.OctreeKey, int]"""
    def compute_east_neighbor_key(self, key: OctreeKey, depth: int) -> OctreeKey | None:
        """compute_east_neighbor_key(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]"""
    def compute_north_neighbor_key(self, key: OctreeKey, depth: int) -> OctreeKey | None:
        """compute_north_neighbor_key(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]"""
    def compute_ray_coords(self, sx: float, sy: float, sz: float, ex: float, ey: float, ez: float) -> list[numpy.ndarray[numpy.float32[3, 1]]] | None:
        """compute_ray_coords(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, sx: float, sy: float, sz: float, ex: float, ey: float, ez: float) -> Optional[list[numpy.ndarray[numpy.float32[3, 1]]]]"""
    def compute_ray_keys(self, sx: float, sy: float, sz: float, ex: float, ey: float, ez: float) -> list[OctreeKey] | None:
        """compute_ray_keys(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, sx: float, sy: float, sz: float, ex: float, ey: float, ez: float) -> Optional[list[erl_geometry.pyerl_geometry.OctreeKey]]"""
    def compute_south_neighbor_key(self, key: OctreeKey, depth: int) -> OctreeKey | None:
        """compute_south_neighbor_key(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]"""
    def compute_top_neighbor_key(self, key: OctreeKey, depth: int) -> OctreeKey | None:
        """compute_top_neighbor_key(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]"""
    def compute_west_neighbor_key(self, key: OctreeKey, depth: int) -> OctreeKey | None:
        """compute_west_neighbor_key(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]"""
    @overload
    def coord_to_key(self, coordinate: float) -> int:
        """coord_to_key(*args, **kwargs)
        Overloaded function.

        1. coord_to_key(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, coordinate: float) -> int

        2. coord_to_key(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, coordinate: float, depth: int) -> int

        3. coord_to_key(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, x: float, y: float, z: float) -> erl_geometry.pyerl_geometry.OctreeKey

        4. coord_to_key(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, x: float, y: float, z: float, depth: int) -> erl_geometry.pyerl_geometry.OctreeKey
        """
    @overload
    def coord_to_key(self, coordinate: float, depth: int) -> int:
        """coord_to_key(*args, **kwargs)
        Overloaded function.

        1. coord_to_key(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, coordinate: float) -> int

        2. coord_to_key(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, coordinate: float, depth: int) -> int

        3. coord_to_key(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, x: float, y: float, z: float) -> erl_geometry.pyerl_geometry.OctreeKey

        4. coord_to_key(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, x: float, y: float, z: float, depth: int) -> erl_geometry.pyerl_geometry.OctreeKey
        """
    @overload
    def coord_to_key(self, x: float, y: float, z: float) -> OctreeKey:
        """coord_to_key(*args, **kwargs)
        Overloaded function.

        1. coord_to_key(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, coordinate: float) -> int

        2. coord_to_key(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, coordinate: float, depth: int) -> int

        3. coord_to_key(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, x: float, y: float, z: float) -> erl_geometry.pyerl_geometry.OctreeKey

        4. coord_to_key(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, x: float, y: float, z: float, depth: int) -> erl_geometry.pyerl_geometry.OctreeKey
        """
    @overload
    def coord_to_key(self, x: float, y: float, z: float, depth: int) -> OctreeKey:
        """coord_to_key(*args, **kwargs)
        Overloaded function.

        1. coord_to_key(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, coordinate: float) -> int

        2. coord_to_key(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, coordinate: float, depth: int) -> int

        3. coord_to_key(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, x: float, y: float, z: float) -> erl_geometry.pyerl_geometry.OctreeKey

        4. coord_to_key(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, x: float, y: float, z: float, depth: int) -> erl_geometry.pyerl_geometry.OctreeKey
        """
    @overload
    def coord_to_key_checked(self, coordinate: float) -> int | None:
        """coord_to_key_checked(*args, **kwargs)
        Overloaded function.

        1. coord_to_key_checked(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, coordinate: float) -> Optional[int]

        2. coord_to_key_checked(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, coordinate: float, depth: int) -> Optional[int]

        3. coord_to_key_checked(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, x: float, y: float, z: float) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]

        4. coord_to_key_checked(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, x: float, y: float, z: float, depth: int) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]
        """
    @overload
    def coord_to_key_checked(self, coordinate: float, depth: int) -> int | None:
        """coord_to_key_checked(*args, **kwargs)
        Overloaded function.

        1. coord_to_key_checked(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, coordinate: float) -> Optional[int]

        2. coord_to_key_checked(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, coordinate: float, depth: int) -> Optional[int]

        3. coord_to_key_checked(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, x: float, y: float, z: float) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]

        4. coord_to_key_checked(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, x: float, y: float, z: float, depth: int) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]
        """
    @overload
    def coord_to_key_checked(self, x: float, y: float, z: float) -> OctreeKey | None:
        """coord_to_key_checked(*args, **kwargs)
        Overloaded function.

        1. coord_to_key_checked(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, coordinate: float) -> Optional[int]

        2. coord_to_key_checked(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, coordinate: float, depth: int) -> Optional[int]

        3. coord_to_key_checked(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, x: float, y: float, z: float) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]

        4. coord_to_key_checked(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, x: float, y: float, z: float, depth: int) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]
        """
    @overload
    def coord_to_key_checked(self, x: float, y: float, z: float, depth: int) -> OctreeKey | None:
        """coord_to_key_checked(*args, **kwargs)
        Overloaded function.

        1. coord_to_key_checked(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, coordinate: float) -> Optional[int]

        2. coord_to_key_checked(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, coordinate: float, depth: int) -> Optional[int]

        3. coord_to_key_checked(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, x: float, y: float, z: float) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]

        4. coord_to_key_checked(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, x: float, y: float, z: float, depth: int) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]
        """
    def create_node_child(self, node: OccupancyOctreeNode, child_idx: int) -> OccupancyOctreeNode:
        """create_node_child(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, node: erl_geometry.pyerl_geometry.OccupancyOctreeNode, child_idx: int) -> erl_geometry.pyerl_geometry.OccupancyOctreeNode"""
    @overload
    def delete_node(self, x: float, y: float, z: float, depth: int) -> int:
        """delete_node(*args, **kwargs)
        Overloaded function.

        1. delete_node(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, x: float, y: float, z: float, depth: int) -> int

        2. delete_node(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> None
        """
    @overload
    def delete_node(self, key: OctreeKey, depth: int) -> None:
        """delete_node(*args, **kwargs)
        Overloaded function.

        1. delete_node(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, x: float, y: float, z: float, depth: int) -> int

        2. delete_node(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> None
        """
    def delete_node_child(self, node: OccupancyOctreeNode, child_idx: int, key: OctreeKey) -> int:
        """delete_node_child(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, node: erl_geometry.pyerl_geometry.OccupancyOctreeNode, child_idx: int, key: erl_geometry.pyerl_geometry.OctreeKey) -> int"""
    def expand(self) -> None:
        """expand(self: erl_geometry.pyerl_geometry.OccupancyOctreeF) -> None"""
    def expand_node(self, node: OccupancyOctreeNode) -> None:
        """expand_node(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, node: erl_geometry.pyerl_geometry.OccupancyOctreeNode) -> None"""
    def get_batch_ray_caster(self, *args, **kwargs):
        """get_batch_ray_caster(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, origins: numpy.ndarray[numpy.float32[3, n]], directions: numpy.ndarray[numpy.float32[3, n]], max_ranges: numpy.ndarray[numpy.float32[m, 1]] = array([], dtype=float32), node_paddings: numpy.ndarray[numpy.float32[m, 1]] = array([], dtype=float32), bidirectional_flags: numpy.ndarray[bool[m, 1]] = array([], dtype=bool), leaf_only_flags: numpy.ndarray[bool[m, 1]] = array([], dtype=bool), min_node_depths: numpy.ndarray[numpy.int32[m, 1]] = array([], dtype=int32), max_node_depths: numpy.ndarray[numpy.int32[m, 1]] = array([], dtype=int32)) -> erl::geometry::OccupancyNdTreeBatchRayCaster<erl::geometry::OccupancyOctreeBase<float, erl::geometry::OccupancyOctreeNode, erl::geometry::OccupancyOctreeBaseSetting>, 3>"""
    def get_node_child(self, node: OccupancyOctreeNode, child_idx: int) -> OccupancyOctreeNode:
        """get_node_child(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, node: erl_geometry.pyerl_geometry.OccupancyOctreeNode, child_idx: int) -> erl_geometry.pyerl_geometry.OccupancyOctreeNode"""
    def get_node_size(self, depth: int) -> float:
        """get_node_size(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, depth: int) -> float"""
    @overload
    def insert_node(self, x: float, y: float, z: float, depth: int) -> OccupancyOctreeNode:
        """insert_node(*args, **kwargs)
        Overloaded function.

        1. insert_node(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, x: float, y: float, z: float, depth: int) -> erl_geometry.pyerl_geometry.OccupancyOctreeNode

        2. insert_node(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> erl_geometry.pyerl_geometry.OccupancyOctreeNode
        """
    @overload
    def insert_node(self, key: OctreeKey, depth: int) -> OccupancyOctreeNode:
        """insert_node(*args, **kwargs)
        Overloaded function.

        1. insert_node(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, x: float, y: float, z: float, depth: int) -> erl_geometry.pyerl_geometry.OccupancyOctreeNode

        2. insert_node(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> erl_geometry.pyerl_geometry.OccupancyOctreeNode
        """
    def insert_point_cloud(self, points: numpy.ndarray[numpy.float32[3, n], flags.f_contiguous], sensor_origin: numpy.ndarray[numpy.float32[3, 1]], min_range: float, max_range: float, with_count: bool, parallel: bool, lazy_eval: bool, discrete: bool) -> None:
        """insert_point_cloud(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, points: numpy.ndarray[numpy.float32[3, n], flags.f_contiguous], sensor_origin: numpy.ndarray[numpy.float32[3, 1]], min_range: float, max_range: float, with_count: bool, parallel: bool, lazy_eval: bool, discrete: bool) -> None"""
    def insert_point_cloud_rays(self, points: numpy.ndarray[numpy.float32[3, n], flags.f_contiguous], sensor_origin: numpy.ndarray[numpy.float32[3, 1]], min_range: float, max_range: float, parallel: bool, lazy_eval: bool) -> None:
        """insert_point_cloud_rays(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, points: numpy.ndarray[numpy.float32[3, n], flags.f_contiguous], sensor_origin: numpy.ndarray[numpy.float32[3, 1]], min_range: float, max_range: float, parallel: bool, lazy_eval: bool) -> None"""
    def insert_ray(self, sx: float, sy: float, sz: float, ex: float, ey: float, ez: float, min_range: float, max_range: float, lazy_eval: bool) -> bool:
        """insert_ray(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, sx: float, sy: float, sz: float, ex: float, ey: float, ez: float, min_range: float, max_range: float, lazy_eval: bool) -> bool"""
    def is_node_collapsible(self, node: OccupancyOctreeNode) -> bool:
        """is_node_collapsible(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, node: erl_geometry.pyerl_geometry.OccupancyOctreeNode) -> bool"""
    @overload
    def iter_bottom_leaf_neighbor(self, x: float, y: float, z: float, max_leaf_depth: int = ...) -> Iterator[OccupancyOctreeF.BottomLeafNeighborIterator]:
        """iter_bottom_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_bottom_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeF.BottomLeafNeighborIterator]

        2. iter_bottom_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeF.BottomLeafNeighborIterator]
        """
    @overload
    def iter_bottom_leaf_neighbor(self, key: OctreeKey, key_depth: int, max_leaf_depth: int = ...) -> Iterator[OccupancyOctreeF.BottomLeafNeighborIterator]:
        """iter_bottom_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_bottom_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeF.BottomLeafNeighborIterator]

        2. iter_bottom_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeF.BottomLeafNeighborIterator]
        """
    @overload
    def iter_east_leaf_neighbor(self, x: float, y: float, z: float, max_leaf_depth: int = ...) -> Iterator[OccupancyOctreeF.EastLeafNeighborIterator]:
        """iter_east_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_east_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeF.EastLeafNeighborIterator]

        2. iter_east_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeF.EastLeafNeighborIterator]
        """
    @overload
    def iter_east_leaf_neighbor(self, key: OctreeKey, key_depth: int, max_leaf_depth: int = ...) -> Iterator[OccupancyOctreeF.EastLeafNeighborIterator]:
        """iter_east_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_east_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeF.EastLeafNeighborIterator]

        2. iter_east_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeF.EastLeafNeighborIterator]
        """
    def iter_leaf(self, max_depth: int = ...) -> Iterator[OccupancyOctreeF.LeafIterator]:
        """iter_leaf(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeF.LeafIterator]"""
    @overload
    def iter_leaf_in_aabb(self, aabb_min_x: float, aabb_min_y: float, aabb_min_z: float, aabb_max_x: float, aabb_max_y: float, aabb_max_z: float, max_depth: int = ...) -> Iterator[OccupancyOctreeF.LeafInAabbIterator]:
        """iter_leaf_in_aabb(*args, **kwargs)
        Overloaded function.

        1. iter_leaf_in_aabb(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, aabb_min_x: float, aabb_min_y: float, aabb_min_z: float, aabb_max_x: float, aabb_max_y: float, aabb_max_z: float, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeF.LeafInAabbIterator]

        2. iter_leaf_in_aabb(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, aabb_min_key: erl_geometry.pyerl_geometry.OctreeKey, aabb_max_key: erl_geometry.pyerl_geometry.OctreeKey, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeF.LeafInAabbIterator]
        """
    @overload
    def iter_leaf_in_aabb(self, aabb_min_key: OctreeKey, aabb_max_key: OctreeKey, max_depth: int = ...) -> Iterator[OccupancyOctreeF.LeafInAabbIterator]:
        """iter_leaf_in_aabb(*args, **kwargs)
        Overloaded function.

        1. iter_leaf_in_aabb(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, aabb_min_x: float, aabb_min_y: float, aabb_min_z: float, aabb_max_x: float, aabb_max_y: float, aabb_max_z: float, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeF.LeafInAabbIterator]

        2. iter_leaf_in_aabb(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, aabb_min_key: erl_geometry.pyerl_geometry.OctreeKey, aabb_max_key: erl_geometry.pyerl_geometry.OctreeKey, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeF.LeafInAabbIterator]
        """
    def iter_leaf_of_node(self, node_key: OctreeKey, node_depth: int, max_depth: int = ...) -> Iterator[OccupancyOctreeF.LeafOfNodeIterator]:
        """iter_leaf_of_node(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, node_key: erl_geometry.pyerl_geometry.OctreeKey, node_depth: int, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeF.LeafOfNodeIterator]"""
    def iter_node(self, max_depth: int = ...) -> Iterator[OccupancyOctreeF.TreeIterator]:
        """iter_node(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeF.TreeIterator]"""
    @overload
    def iter_node_in_aabb(self, aabb_min_x: float, aabb_min_y: float, aabb_min_z: float, aabb_max_x: float, aabb_max_y: float, aabb_max_z: float, max_depth: int = ...) -> Iterator[OccupancyOctreeF.TreeInAabbIterator]:
        """iter_node_in_aabb(*args, **kwargs)
        Overloaded function.

        1. iter_node_in_aabb(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, aabb_min_x: float, aabb_min_y: float, aabb_min_z: float, aabb_max_x: float, aabb_max_y: float, aabb_max_z: float, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeF.TreeInAabbIterator]

        2. iter_node_in_aabb(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, aabb_min_key: erl_geometry.pyerl_geometry.OctreeKey, aabb_max_key: erl_geometry.pyerl_geometry.OctreeKey, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeF.TreeInAabbIterator]
        """
    @overload
    def iter_node_in_aabb(self, aabb_min_key: OctreeKey, aabb_max_key: OctreeKey, max_depth: int = ...) -> Iterator[OccupancyOctreeF.TreeInAabbIterator]:
        """iter_node_in_aabb(*args, **kwargs)
        Overloaded function.

        1. iter_node_in_aabb(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, aabb_min_x: float, aabb_min_y: float, aabb_min_z: float, aabb_max_x: float, aabb_max_y: float, aabb_max_z: float, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeF.TreeInAabbIterator]

        2. iter_node_in_aabb(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, aabb_min_key: erl_geometry.pyerl_geometry.OctreeKey, aabb_max_key: erl_geometry.pyerl_geometry.OctreeKey, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeF.TreeInAabbIterator]
        """
    def iter_node_on_ray(self, px: float, py: float, pz: float, vx: float, vy: float, vz: float, max_range: float = ..., node_padding: float = ..., bidirectional: bool = ..., leaf_only: bool = ..., min_node_depth: int = ..., max_node_depth: int = ...) -> Iterator[OccupancyOctreeF.NodeOnRayIterator]:
        """iter_node_on_ray(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, px: float, py: float, pz: float, vx: float, vy: float, vz: float, max_range: float = -1, node_padding: float = 0, bidirectional: bool = False, leaf_only: bool = True, min_node_depth: int = 0, max_node_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeF.NodeOnRayIterator]"""
    @overload
    def iter_north_leaf_neighbor(self, x: float, y: float, z: float, max_leaf_depth: int = ...) -> Iterator[OccupancyOctreeF.NorthLeafNeighborIterator]:
        """iter_north_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_north_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeF.NorthLeafNeighborIterator]

        2. iter_north_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeF.NorthLeafNeighborIterator]
        """
    @overload
    def iter_north_leaf_neighbor(self, key: OctreeKey, key_depth: int, max_leaf_depth: int = ...) -> Iterator[OccupancyOctreeF.NorthLeafNeighborIterator]:
        """iter_north_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_north_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeF.NorthLeafNeighborIterator]

        2. iter_north_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeF.NorthLeafNeighborIterator]
        """
    @overload
    def iter_south_leaf_neighbor(self, x: float, y: float, z: float, max_leaf_depth: int = ...) -> Iterator[OccupancyOctreeF.SouthLeafNeighborIterator]:
        """iter_south_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_south_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeF.SouthLeafNeighborIterator]

        2. iter_south_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeF.SouthLeafNeighborIterator]
        """
    @overload
    def iter_south_leaf_neighbor(self, key: OctreeKey, key_depth: int, max_leaf_depth: int = ...) -> Iterator[OccupancyOctreeF.SouthLeafNeighborIterator]:
        """iter_south_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_south_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeF.SouthLeafNeighborIterator]

        2. iter_south_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeF.SouthLeafNeighborIterator]
        """
    @overload
    def iter_top_leaf_neighbor(self, x: float, y: float, z: float, max_leaf_depth: int = ...) -> Iterator[OccupancyOctreeF.TopLeafNeighborIterator]:
        """iter_top_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_top_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeF.TopLeafNeighborIterator]

        2. iter_top_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeF.TopLeafNeighborIterator]
        """
    @overload
    def iter_top_leaf_neighbor(self, key: OctreeKey, key_depth: int, max_leaf_depth: int = ...) -> Iterator[OccupancyOctreeF.TopLeafNeighborIterator]:
        """iter_top_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_top_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeF.TopLeafNeighborIterator]

        2. iter_top_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeF.TopLeafNeighborIterator]
        """
    @overload
    def iter_west_leaf_neighbor(self, x: float, y: float, z: float, max_leaf_depth: int = ...) -> Iterator[OccupancyOctreeF.WestLeafNeighborIterator]:
        """iter_west_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_west_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeF.WestLeafNeighborIterator]

        2. iter_west_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeF.WestLeafNeighborIterator]
        """
    @overload
    def iter_west_leaf_neighbor(self, key: OctreeKey, key_depth: int, max_leaf_depth: int = ...) -> Iterator[OccupancyOctreeF.WestLeafNeighborIterator]:
        """iter_west_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_west_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeF.WestLeafNeighborIterator]

        2. iter_west_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyOctreeF.WestLeafNeighborIterator]
        """
    @overload
    def key_to_coord(self, key: int) -> float:
        """key_to_coord(*args, **kwargs)
        Overloaded function.

        1. key_to_coord(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, key: int) -> float

        2. key_to_coord(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, key: int, depth: int) -> float

        3. key_to_coord(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey) -> tuple[float, float, float]

        4. key_to_coord(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> tuple[float, float, float]
        """
    @overload
    def key_to_coord(self, key: int, depth: int) -> float:
        """key_to_coord(*args, **kwargs)
        Overloaded function.

        1. key_to_coord(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, key: int) -> float

        2. key_to_coord(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, key: int, depth: int) -> float

        3. key_to_coord(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey) -> tuple[float, float, float]

        4. key_to_coord(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> tuple[float, float, float]
        """
    @overload
    def key_to_coord(self, key: OctreeKey) -> tuple[float, float, float]:
        """key_to_coord(*args, **kwargs)
        Overloaded function.

        1. key_to_coord(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, key: int) -> float

        2. key_to_coord(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, key: int, depth: int) -> float

        3. key_to_coord(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey) -> tuple[float, float, float]

        4. key_to_coord(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> tuple[float, float, float]
        """
    @overload
    def key_to_coord(self, key: OctreeKey, depth: int) -> tuple[float, float, float]:
        """key_to_coord(*args, **kwargs)
        Overloaded function.

        1. key_to_coord(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, key: int) -> float

        2. key_to_coord(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, key: int, depth: int) -> float

        3. key_to_coord(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey) -> tuple[float, float, float]

        4. key_to_coord(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> tuple[float, float, float]
        """
    def prune(self) -> None:
        """prune(self: erl_geometry.pyerl_geometry.OccupancyOctreeF) -> None"""
    def prune_node(self, node: OccupancyOctreeNode) -> bool:
        """prune_node(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, node: erl_geometry.pyerl_geometry.OccupancyOctreeNode) -> bool"""
    def sample_positions(self, num_positions: int) -> list[numpy.ndarray[numpy.float32[3, 1]]]:
        """sample_positions(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, num_positions: int) -> list[numpy.ndarray[numpy.float32[3, 1]]]"""
    @overload
    def search(self, x: float, y: float, z: float, max_depth: int = ...) -> OccupancyOctreeNode:
        """search(*args, **kwargs)
        Overloaded function.

        1. search(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, x: float, y: float, z: float, max_depth: int = 0) -> erl_geometry.pyerl_geometry.OccupancyOctreeNode

        2. search(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, max_depth: int = 0) -> erl_geometry.pyerl_geometry.OccupancyOctreeNode
        """
    @overload
    def search(self, key: OctreeKey, max_depth: int = ...) -> OccupancyOctreeNode:
        """search(*args, **kwargs)
        Overloaded function.

        1. search(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, x: float, y: float, z: float, max_depth: int = 0) -> erl_geometry.pyerl_geometry.OccupancyOctreeNode

        2. search(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, max_depth: int = 0) -> erl_geometry.pyerl_geometry.OccupancyOctreeNode
        """
    def to_max_likelihood(self) -> None:
        """to_max_likelihood(self: erl_geometry.pyerl_geometry.OccupancyOctreeF) -> None"""
    def update_inner_occupancy(self) -> None:
        """update_inner_occupancy(self: erl_geometry.pyerl_geometry.OccupancyOctreeF) -> None"""
    @overload
    def update_node(self, x: float, y: float, z: float, occupied: bool, lazy_eval: bool) -> OccupancyOctreeNode:
        """update_node(*args, **kwargs)
        Overloaded function.

        1. update_node(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, x: float, y: float, z: float, occupied: bool, lazy_eval: bool) -> erl_geometry.pyerl_geometry.OccupancyOctreeNode

        2. update_node(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, node_key: erl_geometry.pyerl_geometry.OctreeKey, occupied: bool, lazy_eval: bool) -> erl_geometry.pyerl_geometry.OccupancyOctreeNode

        3. update_node(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, x: float, y: float, z: float, log_odds_delta: float, lazy_eval: bool) -> erl_geometry.pyerl_geometry.OccupancyOctreeNode

        4. update_node(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, node_key: erl_geometry.pyerl_geometry.OctreeKey, log_odds_delta: float, lazy_eval: bool) -> erl_geometry.pyerl_geometry.OccupancyOctreeNode
        """
    @overload
    def update_node(self, node_key: OctreeKey, occupied: bool, lazy_eval: bool) -> OccupancyOctreeNode:
        """update_node(*args, **kwargs)
        Overloaded function.

        1. update_node(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, x: float, y: float, z: float, occupied: bool, lazy_eval: bool) -> erl_geometry.pyerl_geometry.OccupancyOctreeNode

        2. update_node(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, node_key: erl_geometry.pyerl_geometry.OctreeKey, occupied: bool, lazy_eval: bool) -> erl_geometry.pyerl_geometry.OccupancyOctreeNode

        3. update_node(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, x: float, y: float, z: float, log_odds_delta: float, lazy_eval: bool) -> erl_geometry.pyerl_geometry.OccupancyOctreeNode

        4. update_node(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, node_key: erl_geometry.pyerl_geometry.OctreeKey, log_odds_delta: float, lazy_eval: bool) -> erl_geometry.pyerl_geometry.OccupancyOctreeNode
        """
    @overload
    def update_node(self, x: float, y: float, z: float, log_odds_delta: float, lazy_eval: bool) -> OccupancyOctreeNode:
        """update_node(*args, **kwargs)
        Overloaded function.

        1. update_node(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, x: float, y: float, z: float, occupied: bool, lazy_eval: bool) -> erl_geometry.pyerl_geometry.OccupancyOctreeNode

        2. update_node(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, node_key: erl_geometry.pyerl_geometry.OctreeKey, occupied: bool, lazy_eval: bool) -> erl_geometry.pyerl_geometry.OccupancyOctreeNode

        3. update_node(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, x: float, y: float, z: float, log_odds_delta: float, lazy_eval: bool) -> erl_geometry.pyerl_geometry.OccupancyOctreeNode

        4. update_node(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, node_key: erl_geometry.pyerl_geometry.OctreeKey, log_odds_delta: float, lazy_eval: bool) -> erl_geometry.pyerl_geometry.OccupancyOctreeNode
        """
    @overload
    def update_node(self, node_key: OctreeKey, log_odds_delta: float, lazy_eval: bool) -> OccupancyOctreeNode:
        """update_node(*args, **kwargs)
        Overloaded function.

        1. update_node(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, x: float, y: float, z: float, occupied: bool, lazy_eval: bool) -> erl_geometry.pyerl_geometry.OccupancyOctreeNode

        2. update_node(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, node_key: erl_geometry.pyerl_geometry.OctreeKey, occupied: bool, lazy_eval: bool) -> erl_geometry.pyerl_geometry.OccupancyOctreeNode

        3. update_node(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, x: float, y: float, z: float, log_odds_delta: float, lazy_eval: bool) -> erl_geometry.pyerl_geometry.OccupancyOctreeNode

        4. update_node(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, node_key: erl_geometry.pyerl_geometry.OctreeKey, log_odds_delta: float, lazy_eval: bool) -> erl_geometry.pyerl_geometry.OccupancyOctreeNode
        """
    def visualize(self, leaf_only: bool = ..., scaling: float = ..., area_min: numpy.ndarray[numpy.float64[3, 1]] = ..., area_max: numpy.ndarray[numpy.float64[3, 1]] = ..., border_color: numpy.ndarray[numpy.float64[3, 1]] = ..., occupied_color: numpy.ndarray[numpy.float64[3, 1]] = ..., occupied_only: bool = ..., draw_node_boxes: bool = ..., draw_node_borders: bool = ..., window_width: int = ..., window_height: int = ..., window_left: int = ..., window_top: int = ...) -> None:
        """visualize(self: erl_geometry.pyerl_geometry.OccupancyOctreeF, leaf_only: bool = False, scaling: float = 1.0, area_min: numpy.ndarray[numpy.float64[3, 1]] = array([-1., -1., -1.], dtype=float32), area_max: numpy.ndarray[numpy.float64[3, 1]] = array([1., 1., 1.], dtype=float32), border_color: numpy.ndarray[numpy.float64[3, 1]] = array([0., 0., 0.], dtype=float32), occupied_color: numpy.ndarray[numpy.float64[3, 1]] = array([0.5, 0.5, 0.5], dtype=float32), occupied_only: bool = False, draw_node_boxes: bool = True, draw_node_borders: bool = True, window_width: int = 1920, window_height: int = 1080, window_left: int = 50, window_top: int = 50) -> None"""
    @property
    def memory_usage(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeF) -> int"""
    @property
    def memory_usage_per_node(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeF) -> int"""
    @property
    def metric_aabb(self) -> Aabb3Df:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeF) -> erl_geometry.pyerl_geometry.Aabb3Df"""
    @property
    def metric_max(self) -> numpy.ndarray[numpy.float32[3, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeF) -> numpy.ndarray[numpy.float32[3, 1]]"""
    @property
    def metric_min(self) -> numpy.ndarray[numpy.float32[3, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeF) -> numpy.ndarray[numpy.float32[3, 1]]"""
    @property
    def metric_min_max(self) -> tuple[numpy.ndarray[numpy.float32[3, 1]], numpy.ndarray[numpy.float32[3, 1]]]:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeF) -> tuple[numpy.ndarray[numpy.float32[3, 1]], numpy.ndarray[numpy.float32[3, 1]]]"""
    @property
    def metric_size(self) -> numpy.ndarray[numpy.float32[3, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeF) -> numpy.ndarray[numpy.float32[3, 1]]"""
    @property
    def number_of_leaf_nodes(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeF) -> int"""
    @property
    def number_of_nodes(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeF) -> int"""
    @property
    def resolution(self) -> float:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeF) -> float"""
    @property
    def root(self) -> OccupancyOctreeNode:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeF) -> erl_geometry.pyerl_geometry.OccupancyOctreeNode"""
    @property
    def setting(self) -> OccupancyOctreeBaseSetting:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeF) -> erl_geometry.pyerl_geometry.OccupancyOctreeBaseSetting"""
    @property
    def tree_center(self) -> numpy.ndarray[numpy.float32[3, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeF) -> numpy.ndarray[numpy.float32[3, 1]]"""
    @property
    def tree_center_key(self) -> OctreeKey:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeF) -> erl_geometry.pyerl_geometry.OctreeKey"""
    @property
    def tree_depth(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeF) -> int"""
    @property
    def tree_max_half_size(self) -> numpy.ndarray[numpy.float32[3, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeF) -> numpy.ndarray[numpy.float32[3, 1]]"""

class OccupancyOctreeNode(AbstractOctreeNode):
    def __init__(self, *args, **kwargs) -> None:
        """Initialize self.  See help(type(self)) for accurate signature."""
    def add_log_odds(self, log_odds: float) -> None:
        """add_log_odds(self: erl_geometry.pyerl_geometry.OccupancyOctreeNode, log_odds: float) -> None"""
    def allow_update_log_odds(self, delta: float) -> bool:
        """allow_update_log_odds(self: erl_geometry.pyerl_geometry.OccupancyOctreeNode, delta: float) -> bool"""
    def get_child(self, child_idx: int) -> OccupancyOctreeNode:
        """get_child(self: erl_geometry.pyerl_geometry.OccupancyOctreeNode, child_idx: int) -> erl_geometry.pyerl_geometry.OccupancyOctreeNode"""
    @property
    def log_odds(self) -> float:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeNode) -> float"""
    @property
    def max_child_log_odds(self) -> float:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeNode) -> float"""
    @property
    def mean_child_log_odds(self) -> float:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeNode) -> float"""
    @property
    def occupancy(self) -> float:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyOctreeNode) -> float"""

class OccupancyQuadtreeBaseSetting(OccupancyNdTreeSetting):
    aabb: Aabb2Dd
    use_aabb_limit: bool
    use_change_detection: bool
    def __init__(self) -> None:
        """__init__(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeBaseSetting) -> None"""

class OccupancyQuadtreeD(AbstractOccupancyQuadtreeD):
    class BatchRayCaster:
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""
        def step(self, mask: numpy.ndarray[bool[m, 1]] = ...) -> OccupancyQuadtreeD.BatchRayCaster:
            """step(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD.BatchRayCaster, mask: numpy.ndarray[bool[m, 1]] = array([], dtype=bool)) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeD.BatchRayCaster"""
        @property
        def ever_hit_flags(self) -> numpy.ndarray[bool[m, 1]]:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeD.BatchRayCaster) -> numpy.ndarray[bool[m, 1]]"""
        @property
        def frontier_keys(self) -> list[QuadtreeKey]:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeD.BatchRayCaster) -> list[erl_geometry.pyerl_geometry.QuadtreeKey]"""
        @property
        def frontier_nodes(self) -> list[OccupancyQuadtreeNode]:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeD.BatchRayCaster) -> list[erl_geometry.pyerl_geometry.OccupancyQuadtreeNode]"""
        @property
        def frontier_ray_indices(self) -> list[list[int]]:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeD.BatchRayCaster) -> list[list[int]]"""
        @property
        def hit_distances(self) -> numpy.ndarray[numpy.float64[m, 1]]:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeD.BatchRayCaster) -> numpy.ndarray[numpy.float64[m, 1]]"""
        @property
        def hit_flags(self) -> numpy.ndarray[bool[m, 1]]:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeD.BatchRayCaster) -> numpy.ndarray[bool[m, 1]]"""
        @property
        def hit_nodes(self) -> list[OccupancyQuadtreeNode]:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeD.BatchRayCaster) -> list[erl_geometry.pyerl_geometry.OccupancyQuadtreeNode]"""
        @property
        def hit_positions(self) -> list[numpy.ndarray[numpy.float64[2, 1]]]:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeD.BatchRayCaster) -> list[numpy.ndarray[numpy.float64[2, 1]]]"""
        @property
        def num_rays(self) -> int:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeD.BatchRayCaster) -> int"""
        @property
        def ray_directions(self) -> numpy.ndarray[numpy.float64[2, n]]:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeD.BatchRayCaster) -> numpy.ndarray[numpy.float64[2, n]]"""
        @property
        def ray_origins(self) -> numpy.ndarray[numpy.float64[2, n]]:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeD.BatchRayCaster) -> numpy.ndarray[numpy.float64[2, n]]"""

    class Drawer:
        def __init__(self, setting: OccupancyQuadtreeDrawerSettingD, quadtree: OccupancyQuadtreeD = ...) -> None:
            """__init__(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD.Drawer, setting: erl_geometry.pyerl_geometry.OccupancyQuadtreeDrawerSettingD, quadtree: erl_geometry.pyerl_geometry.OccupancyQuadtreeD = None) -> None"""
        @staticmethod
        def Setting() -> OccupancyQuadtreeDrawerSettingD:
            """Setting() -> erl_geometry.pyerl_geometry.OccupancyQuadtreeDrawerSettingD"""
        @overload
        def draw_leaves(self) -> Mat:
            """draw_leaves(*args, **kwargs)
            Overloaded function.

            1. draw_leaves(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD.Drawer) -> Mat

            2. draw_leaves(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD.Drawer, filename: str) -> None
            """
        @overload
        def draw_leaves(self, filename: str) -> None:
            """draw_leaves(*args, **kwargs)
            Overloaded function.

            1. draw_leaves(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD.Drawer) -> Mat

            2. draw_leaves(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD.Drawer, filename: str) -> None
            """
        @overload
        def draw_tree(self) -> Mat:
            """draw_tree(*args, **kwargs)
            Overloaded function.

            1. draw_tree(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD.Drawer) -> Mat

            2. draw_tree(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD.Drawer, filename: str) -> None
            """
        @overload
        def draw_tree(self, filename: str) -> None:
            """draw_tree(*args, **kwargs)
            Overloaded function.

            1. draw_tree(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD.Drawer) -> Mat

            2. draw_tree(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD.Drawer, filename: str) -> None
            """
        def set_draw_leaf_callback(self, callback: Callable[[OccupancyQuadtreeD.Drawer, Mat, OccupancyQuadtreeD.LeafIterator], None]) -> None:
            """set_draw_leaf_callback(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD.Drawer, callback: Callable[[erl_geometry.pyerl_geometry.OccupancyQuadtreeD.Drawer, Mat, erl_geometry.pyerl_geometry.OccupancyQuadtreeD.LeafIterator], None]) -> None"""
        def set_draw_tree_callback(self, callback: Callable[[OccupancyQuadtreeD.Drawer, Mat, OccupancyQuadtreeD.TreeIterator], None]) -> None:
            """set_draw_tree_callback(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD.Drawer, callback: Callable[[erl_geometry.pyerl_geometry.OccupancyQuadtreeD.Drawer, Mat, erl_geometry.pyerl_geometry.OccupancyQuadtreeD.TreeIterator], None]) -> None"""
        @property
        def grid_map_info(self):
            """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeD.Drawer) -> erl::common::GridMapInfo<float, 2>"""
        @property
        def setting(self) -> OccupancyQuadtreeDrawerSettingD:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeD.Drawer) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeDrawerSettingD"""

    class EastLeafNeighborIterator(OccupancyQuadtreeD.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class IteratorBase(AbstractQuadtreeD.QuadtreeNodeIterator):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""
        def __eq__(self, arg0: OccupancyQuadtreeD.IteratorBase) -> bool:
            """__eq__(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD.IteratorBase, arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeD.IteratorBase) -> bool"""
        def __ne__(self, arg0: OccupancyQuadtreeD.IteratorBase) -> bool:
            """__ne__(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD.IteratorBase, arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeD.IteratorBase) -> bool"""
        @property
        def index_key(self) -> QuadtreeKey:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeD.IteratorBase) -> erl_geometry.pyerl_geometry.QuadtreeKey"""
        @property
        def key(self) -> QuadtreeKey:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeD.IteratorBase) -> erl_geometry.pyerl_geometry.QuadtreeKey"""
        @property
        def node(self) -> OccupancyQuadtreeNode:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeD.IteratorBase) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeNode"""
        @property
        def node_aabb(self) -> Aabb2Dd:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeD.IteratorBase) -> erl_geometry.pyerl_geometry.Aabb2Dd"""

    class LeafInAabbIterator(OccupancyQuadtreeD.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class LeafIterator(OccupancyQuadtreeD.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class LeafOfNodeIterator(OccupancyQuadtreeD.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class NodeOnRayIterator(OccupancyQuadtreeD.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""
        @property
        def distance(self) -> float:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeD.NodeOnRayIterator) -> float"""

    class NorthLeafNeighborIterator(OccupancyQuadtreeD.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class SouthLeafNeighborIterator(OccupancyQuadtreeD.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class TreeInAabbIterator(OccupancyQuadtreeD.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class TreeIterator(OccupancyQuadtreeD.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class WestLeafNeighborIterator(OccupancyQuadtreeD.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""
    @overload
    def __init__(self) -> None:
        """__init__(*args, **kwargs)
        Overloaded function.

        1. __init__(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD) -> None

        2. __init__(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, setting: erl_geometry.pyerl_geometry.OccupancyQuadtreeBaseSetting) -> None

        3. __init__(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, filename: str) -> None

        4. __init__(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, map_info: erl::common::GridMapInfo<double, 2>, image_map: Mat, occupied_threshold: float, padding: int = 0) -> None
        """
    @overload
    def __init__(self, setting: OccupancyQuadtreeBaseSetting) -> None:
        """__init__(*args, **kwargs)
        Overloaded function.

        1. __init__(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD) -> None

        2. __init__(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, setting: erl_geometry.pyerl_geometry.OccupancyQuadtreeBaseSetting) -> None

        3. __init__(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, filename: str) -> None

        4. __init__(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, map_info: erl::common::GridMapInfo<double, 2>, image_map: Mat, occupied_threshold: float, padding: int = 0) -> None
        """
    @overload
    def __init__(self, filename: str) -> None:
        """__init__(*args, **kwargs)
        Overloaded function.

        1. __init__(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD) -> None

        2. __init__(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, setting: erl_geometry.pyerl_geometry.OccupancyQuadtreeBaseSetting) -> None

        3. __init__(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, filename: str) -> None

        4. __init__(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, map_info: erl::common::GridMapInfo<double, 2>, image_map: Mat, occupied_threshold: float, padding: int = 0) -> None
        """
    def Setting(self) -> OccupancyQuadtreeBaseSetting:
        """Setting() -> erl_geometry.pyerl_geometry.OccupancyQuadtreeBaseSetting"""
    @overload
    def adjust_key_to_depth(self, key: int, depth: int) -> int:
        """adjust_key_to_depth(*args, **kwargs)
        Overloaded function.

        1. adjust_key_to_depth(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, key: int, depth: int) -> int

        2. adjust_key_to_depth(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> erl_geometry.pyerl_geometry.QuadtreeKey
        """
    @overload
    def adjust_key_to_depth(self, key: QuadtreeKey, depth: int) -> QuadtreeKey:
        """adjust_key_to_depth(*args, **kwargs)
        Overloaded function.

        1. adjust_key_to_depth(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, key: int, depth: int) -> int

        2. adjust_key_to_depth(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> erl_geometry.pyerl_geometry.QuadtreeKey
        """
    def cast_ray(self, px: float, py: float, vx: float, vy: float, ignore_unknown: bool, max_range: float) -> dict:
        """cast_ray(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, px: float, py: float, vx: float, vy: float, ignore_unknown: bool, max_range: float) -> dict"""
    @overload
    def cast_rays(self, position: numpy.ndarray[numpy.float64[2, 1]], rotation: numpy.ndarray[numpy.float64[2, 2], flags.f_contiguous], angles: numpy.ndarray[numpy.float64[m, 1]], ignore_unknown: bool, max_range: float, prune_rays: bool, parallel: bool) -> dict:
        """cast_rays(*args, **kwargs)
        Overloaded function.

        1. cast_rays(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, position: numpy.ndarray[numpy.float64[2, 1]], rotation: numpy.ndarray[numpy.float64[2, 2], flags.f_contiguous], angles: numpy.ndarray[numpy.float64[m, 1]], ignore_unknown: bool, max_range: float, prune_rays: bool, parallel: bool) -> dict

        2. cast_rays(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, positions: numpy.ndarray[numpy.float64[2, n], flags.f_contiguous], directions: numpy.ndarray[numpy.float64[2, n], flags.f_contiguous], ignore_unknown: bool, max_range: float, prune_rays: bool, parallel: bool) -> dict
        """
    @overload
    def cast_rays(self, positions: numpy.ndarray[numpy.float64[2, n], flags.f_contiguous], directions: numpy.ndarray[numpy.float64[2, n], flags.f_contiguous], ignore_unknown: bool, max_range: float, prune_rays: bool, parallel: bool) -> dict:
        """cast_rays(*args, **kwargs)
        Overloaded function.

        1. cast_rays(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, position: numpy.ndarray[numpy.float64[2, 1]], rotation: numpy.ndarray[numpy.float64[2, 2], flags.f_contiguous], angles: numpy.ndarray[numpy.float64[m, 1]], ignore_unknown: bool, max_range: float, prune_rays: bool, parallel: bool) -> dict

        2. cast_rays(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, positions: numpy.ndarray[numpy.float64[2, n], flags.f_contiguous], directions: numpy.ndarray[numpy.float64[2, n], flags.f_contiguous], ignore_unknown: bool, max_range: float, prune_rays: bool, parallel: bool) -> dict
        """
    def clear(self) -> None:
        """clear(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD) -> None"""
    def compute_common_ancestor_key(self, arg0: QuadtreeKey, arg1: QuadtreeKey) -> tuple[QuadtreeKey, int]:
        """compute_common_ancestor_key(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, arg0: erl_geometry.pyerl_geometry.QuadtreeKey, arg1: erl_geometry.pyerl_geometry.QuadtreeKey) -> tuple[erl_geometry.pyerl_geometry.QuadtreeKey, int]"""
    def compute_east_neighbor_key(self, key: QuadtreeKey, depth: int) -> QuadtreeKey | None:
        """compute_east_neighbor_key(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]"""
    def compute_north_neighbor_key(self, key: QuadtreeKey, depth: int) -> QuadtreeKey | None:
        """compute_north_neighbor_key(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]"""
    def compute_ray_coords(self, sx: float, sy: float, ex: float, ey: float) -> list[numpy.ndarray[numpy.float64[2, 1]]] | None:
        """compute_ray_coords(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, sx: float, sy: float, ex: float, ey: float) -> Optional[list[numpy.ndarray[numpy.float64[2, 1]]]]"""
    def compute_ray_keys(self, sx: float, sy: float, ex: float, ey: float) -> list[QuadtreeKey] | None:
        """compute_ray_keys(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, sx: float, sy: float, ex: float, ey: float) -> Optional[list[erl_geometry.pyerl_geometry.QuadtreeKey]]"""
    def compute_south_neighbor_key(self, key: QuadtreeKey, depth: int) -> QuadtreeKey | None:
        """compute_south_neighbor_key(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]"""
    def compute_west_neighbor_key(self, key: QuadtreeKey, depth: int) -> QuadtreeKey | None:
        """compute_west_neighbor_key(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]"""
    @overload
    def coord_to_key(self, coordinate: float) -> int:
        """coord_to_key(*args, **kwargs)
        Overloaded function.

        1. coord_to_key(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, coordinate: float) -> int

        2. coord_to_key(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, coordinate: float, depth: int) -> int

        3. coord_to_key(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, x: float, y: float) -> erl_geometry.pyerl_geometry.QuadtreeKey

        4. coord_to_key(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, x: float, y: float, depth: int) -> erl_geometry.pyerl_geometry.QuadtreeKey
        """
    @overload
    def coord_to_key(self, coordinate: float, depth: int) -> int:
        """coord_to_key(*args, **kwargs)
        Overloaded function.

        1. coord_to_key(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, coordinate: float) -> int

        2. coord_to_key(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, coordinate: float, depth: int) -> int

        3. coord_to_key(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, x: float, y: float) -> erl_geometry.pyerl_geometry.QuadtreeKey

        4. coord_to_key(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, x: float, y: float, depth: int) -> erl_geometry.pyerl_geometry.QuadtreeKey
        """
    @overload
    def coord_to_key(self, x: float, y: float) -> QuadtreeKey:
        """coord_to_key(*args, **kwargs)
        Overloaded function.

        1. coord_to_key(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, coordinate: float) -> int

        2. coord_to_key(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, coordinate: float, depth: int) -> int

        3. coord_to_key(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, x: float, y: float) -> erl_geometry.pyerl_geometry.QuadtreeKey

        4. coord_to_key(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, x: float, y: float, depth: int) -> erl_geometry.pyerl_geometry.QuadtreeKey
        """
    @overload
    def coord_to_key(self, x: float, y: float, depth: int) -> QuadtreeKey:
        """coord_to_key(*args, **kwargs)
        Overloaded function.

        1. coord_to_key(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, coordinate: float) -> int

        2. coord_to_key(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, coordinate: float, depth: int) -> int

        3. coord_to_key(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, x: float, y: float) -> erl_geometry.pyerl_geometry.QuadtreeKey

        4. coord_to_key(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, x: float, y: float, depth: int) -> erl_geometry.pyerl_geometry.QuadtreeKey
        """
    @overload
    def coord_to_key_checked(self, coordinate: float) -> int | None:
        """coord_to_key_checked(*args, **kwargs)
        Overloaded function.

        1. coord_to_key_checked(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, coordinate: float) -> Optional[int]

        2. coord_to_key_checked(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, coordinate: float, depth: int) -> Optional[int]

        3. coord_to_key_checked(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, x: float, y: float) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]

        4. coord_to_key_checked(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, x: float, y: float, depth: int) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]
        """
    @overload
    def coord_to_key_checked(self, coordinate: float, depth: int) -> int | None:
        """coord_to_key_checked(*args, **kwargs)
        Overloaded function.

        1. coord_to_key_checked(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, coordinate: float) -> Optional[int]

        2. coord_to_key_checked(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, coordinate: float, depth: int) -> Optional[int]

        3. coord_to_key_checked(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, x: float, y: float) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]

        4. coord_to_key_checked(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, x: float, y: float, depth: int) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]
        """
    @overload
    def coord_to_key_checked(self, x: float, y: float) -> QuadtreeKey | None:
        """coord_to_key_checked(*args, **kwargs)
        Overloaded function.

        1. coord_to_key_checked(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, coordinate: float) -> Optional[int]

        2. coord_to_key_checked(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, coordinate: float, depth: int) -> Optional[int]

        3. coord_to_key_checked(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, x: float, y: float) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]

        4. coord_to_key_checked(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, x: float, y: float, depth: int) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]
        """
    @overload
    def coord_to_key_checked(self, x: float, y: float, depth: int) -> QuadtreeKey | None:
        """coord_to_key_checked(*args, **kwargs)
        Overloaded function.

        1. coord_to_key_checked(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, coordinate: float) -> Optional[int]

        2. coord_to_key_checked(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, coordinate: float, depth: int) -> Optional[int]

        3. coord_to_key_checked(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, x: float, y: float) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]

        4. coord_to_key_checked(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, x: float, y: float, depth: int) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]
        """
    def create_node_child(self, node: OccupancyQuadtreeNode, child_idx: int) -> OccupancyQuadtreeNode:
        """create_node_child(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, node: erl_geometry.pyerl_geometry.OccupancyQuadtreeNode, child_idx: int) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeNode"""
    @overload
    def delete_node(self, x: float, y: float, depth: int) -> int:
        """delete_node(*args, **kwargs)
        Overloaded function.

        1. delete_node(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, x: float, y: float, depth: int) -> int

        2. delete_node(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> None
        """
    @overload
    def delete_node(self, key: QuadtreeKey, depth: int) -> None:
        """delete_node(*args, **kwargs)
        Overloaded function.

        1. delete_node(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, x: float, y: float, depth: int) -> int

        2. delete_node(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> None
        """
    def delete_node_child(self, node: OccupancyQuadtreeNode, child_idx: int, key: QuadtreeKey) -> int:
        """delete_node_child(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, node: erl_geometry.pyerl_geometry.OccupancyQuadtreeNode, child_idx: int, key: erl_geometry.pyerl_geometry.QuadtreeKey) -> int"""
    def expand(self) -> None:
        """expand(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD) -> None"""
    def expand_node(self, node: OccupancyQuadtreeNode) -> None:
        """expand_node(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, node: erl_geometry.pyerl_geometry.OccupancyQuadtreeNode) -> None"""
    def get_batch_ray_caster(self, *args, **kwargs):
        """get_batch_ray_caster(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, origins: numpy.ndarray[numpy.float64[2, n]], directions: numpy.ndarray[numpy.float64[2, n]], max_ranges: numpy.ndarray[numpy.float64[m, 1]] = array([], dtype=float64), node_paddings: numpy.ndarray[numpy.float64[m, 1]] = array([], dtype=float64), bidirectional_flags: numpy.ndarray[bool[m, 1]] = array([], dtype=bool), leaf_only_flags: numpy.ndarray[bool[m, 1]] = array([], dtype=bool), min_node_depths: numpy.ndarray[numpy.int32[m, 1]] = array([], dtype=int32), max_node_depths: numpy.ndarray[numpy.int32[m, 1]] = array([], dtype=int32)) -> erl::geometry::OccupancyNdTreeBatchRayCaster<erl::geometry::OccupancyQuadtreeBase<double, erl::geometry::OccupancyQuadtreeNode, erl::geometry::OccupancyQuadtreeBaseSetting>, 2>"""
    def get_node_child(self, node: OccupancyQuadtreeNode, child_idx: int) -> OccupancyQuadtreeNode:
        """get_node_child(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, node: erl_geometry.pyerl_geometry.OccupancyQuadtreeNode, child_idx: int) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeNode"""
    def get_node_size(self, depth: int) -> float:
        """get_node_size(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, depth: int) -> float"""
    @overload
    def insert_node(self, x: float, y: float, depth: int) -> OccupancyQuadtreeNode:
        """insert_node(*args, **kwargs)
        Overloaded function.

        1. insert_node(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, x: float, y: float, depth: int) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeNode

        2. insert_node(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeNode
        """
    @overload
    def insert_node(self, key: QuadtreeKey, depth: int) -> OccupancyQuadtreeNode:
        """insert_node(*args, **kwargs)
        Overloaded function.

        1. insert_node(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, x: float, y: float, depth: int) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeNode

        2. insert_node(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeNode
        """
    def insert_point_cloud(self, points: numpy.ndarray[numpy.float64[2, n], flags.f_contiguous], sensor_origin: numpy.ndarray[numpy.float64[2, 1]], min_range: float, max_range: float, with_count: bool, parallel: bool, lazy_eval: bool, discrete: bool) -> None:
        """insert_point_cloud(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, points: numpy.ndarray[numpy.float64[2, n], flags.f_contiguous], sensor_origin: numpy.ndarray[numpy.float64[2, 1]], min_range: float, max_range: float, with_count: bool, parallel: bool, lazy_eval: bool, discrete: bool) -> None"""
    def insert_point_cloud_rays(self, points: numpy.ndarray[numpy.float64[2, n], flags.f_contiguous], sensor_origin: numpy.ndarray[numpy.float64[2, 1]], min_range: float, max_range: float, parallel: bool, lazy_eval: bool) -> None:
        """insert_point_cloud_rays(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, points: numpy.ndarray[numpy.float64[2, n], flags.f_contiguous], sensor_origin: numpy.ndarray[numpy.float64[2, 1]], min_range: float, max_range: float, parallel: bool, lazy_eval: bool) -> None"""
    def insert_ray(self, sx: float, sy: float, ex: float, ey: float, min_range: float, max_range: float, lazy_eval: bool) -> bool:
        """insert_ray(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, sx: float, sy: float, ex: float, ey: float, min_range: float, max_range: float, lazy_eval: bool) -> bool"""
    def is_node_collapsible(self, node: OccupancyQuadtreeNode) -> bool:
        """is_node_collapsible(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, node: erl_geometry.pyerl_geometry.OccupancyQuadtreeNode) -> bool"""
    @overload
    def iter_east_leaf_neighbor(self, x: float, y: float, max_leaf_depth: int = ...) -> Iterator[OccupancyQuadtreeD.EastLeafNeighborIterator]:
        """iter_east_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_east_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, x: float, y: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyQuadtreeD.EastLeafNeighborIterator]

        2. iter_east_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyQuadtreeD.EastLeafNeighborIterator]
        """
    @overload
    def iter_east_leaf_neighbor(self, key: QuadtreeKey, key_depth: int, max_leaf_depth: int = ...) -> Iterator[OccupancyQuadtreeD.EastLeafNeighborIterator]:
        """iter_east_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_east_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, x: float, y: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyQuadtreeD.EastLeafNeighborIterator]

        2. iter_east_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyQuadtreeD.EastLeafNeighborIterator]
        """
    def iter_leaf(self, max_depth: int = ...) -> Iterator[OccupancyQuadtreeD.LeafIterator]:
        """iter_leaf(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyQuadtreeD.LeafIterator]"""
    @overload
    def iter_leaf_in_aabb(self, aabb_min_x: float, aabb_min_y: float, aabb_max_x: float, aabb_max_y: float, max_depth: int = ...) -> Iterator[OccupancyQuadtreeD.LeafInAabbIterator]:
        """iter_leaf_in_aabb(*args, **kwargs)
        Overloaded function.

        1. iter_leaf_in_aabb(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, aabb_min_x: float, aabb_min_y: float, aabb_max_x: float, aabb_max_y: float, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyQuadtreeD.LeafInAabbIterator]

        2. iter_leaf_in_aabb(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, aabb_min_key: erl_geometry.pyerl_geometry.QuadtreeKey, aabb_max_key: erl_geometry.pyerl_geometry.QuadtreeKey, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyQuadtreeD.LeafInAabbIterator]
        """
    @overload
    def iter_leaf_in_aabb(self, aabb_min_key: QuadtreeKey, aabb_max_key: QuadtreeKey, max_depth: int = ...) -> Iterator[OccupancyQuadtreeD.LeafInAabbIterator]:
        """iter_leaf_in_aabb(*args, **kwargs)
        Overloaded function.

        1. iter_leaf_in_aabb(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, aabb_min_x: float, aabb_min_y: float, aabb_max_x: float, aabb_max_y: float, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyQuadtreeD.LeafInAabbIterator]

        2. iter_leaf_in_aabb(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, aabb_min_key: erl_geometry.pyerl_geometry.QuadtreeKey, aabb_max_key: erl_geometry.pyerl_geometry.QuadtreeKey, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyQuadtreeD.LeafInAabbIterator]
        """
    def iter_leaf_of_node(self, node_key: QuadtreeKey, node_depth: int, max_depth: int = ...) -> Iterator[OccupancyQuadtreeD.LeafOfNodeIterator]:
        """iter_leaf_of_node(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, node_key: erl_geometry.pyerl_geometry.QuadtreeKey, node_depth: int, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyQuadtreeD.LeafOfNodeIterator]"""
    def iter_node(self, max_depth: int = ...) -> Iterator[OccupancyQuadtreeD.TreeIterator]:
        """iter_node(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyQuadtreeD.TreeIterator]"""
    @overload
    def iter_node_in_aabb(self, aabb_min_x: float, aabb_min_y: float, aabb_max_x: float, aabb_max_y: float, max_depth: int = ...) -> Iterator[OccupancyQuadtreeD.TreeInAabbIterator]:
        """iter_node_in_aabb(*args, **kwargs)
        Overloaded function.

        1. iter_node_in_aabb(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, aabb_min_x: float, aabb_min_y: float, aabb_max_x: float, aabb_max_y: float, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyQuadtreeD.TreeInAabbIterator]

        2. iter_node_in_aabb(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, aabb_min_key: erl_geometry.pyerl_geometry.QuadtreeKey, aabb_max_key: erl_geometry.pyerl_geometry.QuadtreeKey, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyQuadtreeD.TreeInAabbIterator]
        """
    @overload
    def iter_node_in_aabb(self, aabb_min_key: QuadtreeKey, aabb_max_key: QuadtreeKey, max_depth: int = ...) -> Iterator[OccupancyQuadtreeD.TreeInAabbIterator]:
        """iter_node_in_aabb(*args, **kwargs)
        Overloaded function.

        1. iter_node_in_aabb(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, aabb_min_x: float, aabb_min_y: float, aabb_max_x: float, aabb_max_y: float, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyQuadtreeD.TreeInAabbIterator]

        2. iter_node_in_aabb(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, aabb_min_key: erl_geometry.pyerl_geometry.QuadtreeKey, aabb_max_key: erl_geometry.pyerl_geometry.QuadtreeKey, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyQuadtreeD.TreeInAabbIterator]
        """
    def iter_node_on_ray(self, px: float, py: float, vx: float, vy: float, max_range: float = ..., node_padding: float = ..., bidirectional: bool = ..., leaf_only: bool = ..., min_node_depth: int = ..., max_node_depth: int = ...) -> Iterator[OccupancyQuadtreeD.NodeOnRayIterator]:
        """iter_node_on_ray(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, px: float, py: float, vx: float, vy: float, max_range: float = -1, node_padding: float = 0, bidirectional: bool = False, leaf_only: bool = False, min_node_depth: int = 0, max_node_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyQuadtreeD.NodeOnRayIterator]"""
    @overload
    def iter_north_leaf_neighbor(self, x: float, y: float, max_leaf_depth: int = ...) -> Iterator[OccupancyQuadtreeD.NorthLeafNeighborIterator]:
        """iter_north_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_north_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, x: float, y: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyQuadtreeD.NorthLeafNeighborIterator]

        2. iter_north_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyQuadtreeD.NorthLeafNeighborIterator]
        """
    @overload
    def iter_north_leaf_neighbor(self, key: QuadtreeKey, key_depth: int, max_leaf_depth: int = ...) -> Iterator[OccupancyQuadtreeD.NorthLeafNeighborIterator]:
        """iter_north_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_north_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, x: float, y: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyQuadtreeD.NorthLeafNeighborIterator]

        2. iter_north_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyQuadtreeD.NorthLeafNeighborIterator]
        """
    @overload
    def iter_south_leaf_neighbor(self, x: float, y: float, max_leaf_depth: int = ...) -> Iterator[OccupancyQuadtreeD.SouthLeafNeighborIterator]:
        """iter_south_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_south_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, x: float, y: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyQuadtreeD.SouthLeafNeighborIterator]

        2. iter_south_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyQuadtreeD.SouthLeafNeighborIterator]
        """
    @overload
    def iter_south_leaf_neighbor(self, key: QuadtreeKey, key_depth: int, max_leaf_depth: int = ...) -> Iterator[OccupancyQuadtreeD.SouthLeafNeighborIterator]:
        """iter_south_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_south_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, x: float, y: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyQuadtreeD.SouthLeafNeighborIterator]

        2. iter_south_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyQuadtreeD.SouthLeafNeighborIterator]
        """
    @overload
    def iter_west_leaf_neighbor(self, x: float, y: float, max_leaf_depth: int = ...) -> Iterator[OccupancyQuadtreeD.WestLeafNeighborIterator]:
        """iter_west_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_west_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, x: float, y: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyQuadtreeD.WestLeafNeighborIterator]

        2. iter_west_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyQuadtreeD.WestLeafNeighborIterator]
        """
    @overload
    def iter_west_leaf_neighbor(self, key: QuadtreeKey, key_depth: int, max_leaf_depth: int = ...) -> Iterator[OccupancyQuadtreeD.WestLeafNeighborIterator]:
        """iter_west_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_west_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, x: float, y: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyQuadtreeD.WestLeafNeighborIterator]

        2. iter_west_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyQuadtreeD.WestLeafNeighborIterator]
        """
    @overload
    def key_to_coord(self, key: int) -> float:
        """key_to_coord(*args, **kwargs)
        Overloaded function.

        1. key_to_coord(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, key: int) -> float

        2. key_to_coord(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, key: int, depth: int) -> float

        3. key_to_coord(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey) -> tuple[float, float]

        4. key_to_coord(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> tuple[float, float]
        """
    @overload
    def key_to_coord(self, key: int, depth: int) -> float:
        """key_to_coord(*args, **kwargs)
        Overloaded function.

        1. key_to_coord(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, key: int) -> float

        2. key_to_coord(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, key: int, depth: int) -> float

        3. key_to_coord(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey) -> tuple[float, float]

        4. key_to_coord(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> tuple[float, float]
        """
    @overload
    def key_to_coord(self, key: QuadtreeKey) -> tuple[float, float]:
        """key_to_coord(*args, **kwargs)
        Overloaded function.

        1. key_to_coord(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, key: int) -> float

        2. key_to_coord(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, key: int, depth: int) -> float

        3. key_to_coord(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey) -> tuple[float, float]

        4. key_to_coord(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> tuple[float, float]
        """
    @overload
    def key_to_coord(self, key: QuadtreeKey, depth: int) -> tuple[float, float]:
        """key_to_coord(*args, **kwargs)
        Overloaded function.

        1. key_to_coord(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, key: int) -> float

        2. key_to_coord(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, key: int, depth: int) -> float

        3. key_to_coord(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey) -> tuple[float, float]

        4. key_to_coord(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> tuple[float, float]
        """
    def prune(self) -> None:
        """prune(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD) -> None"""
    def prune_node(self, node: OccupancyQuadtreeNode) -> bool:
        """prune_node(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, node: erl_geometry.pyerl_geometry.OccupancyQuadtreeNode) -> bool"""
    def sample_positions(self, num_positions: int) -> list[numpy.ndarray[numpy.float64[2, 1]]]:
        """sample_positions(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, num_positions: int) -> list[numpy.ndarray[numpy.float64[2, 1]]]"""
    @overload
    def search(self, x: float, y: float, max_depth: int = ...) -> OccupancyQuadtreeNode:
        """search(*args, **kwargs)
        Overloaded function.

        1. search(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, x: float, y: float, max_depth: int = 0) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeNode

        2. search(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, max_depth: int = 0) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeNode
        """
    @overload
    def search(self, key: QuadtreeKey, max_depth: int = ...) -> OccupancyQuadtreeNode:
        """search(*args, **kwargs)
        Overloaded function.

        1. search(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, x: float, y: float, max_depth: int = 0) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeNode

        2. search(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, max_depth: int = 0) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeNode
        """
    def to_max_likelihood(self) -> None:
        """to_max_likelihood(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD) -> None"""
    def update_inner_occupancy(self) -> None:
        """update_inner_occupancy(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD) -> None"""
    @overload
    def update_node(self, x: float, y: float, occupied: bool, lazy_eval: bool) -> OccupancyQuadtreeNode:
        """update_node(*args, **kwargs)
        Overloaded function.

        1. update_node(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, x: float, y: float, occupied: bool, lazy_eval: bool) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeNode

        2. update_node(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, node_key: erl_geometry.pyerl_geometry.QuadtreeKey, occupied: bool, lazy_eval: bool) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeNode

        3. update_node(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, x: float, y: float, log_odds_delta: float, lazy_eval: bool) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeNode

        4. update_node(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, node_key: erl_geometry.pyerl_geometry.QuadtreeKey, log_odds_delta: float, lazy_eval: bool) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeNode
        """
    @overload
    def update_node(self, node_key: QuadtreeKey, occupied: bool, lazy_eval: bool) -> OccupancyQuadtreeNode:
        """update_node(*args, **kwargs)
        Overloaded function.

        1. update_node(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, x: float, y: float, occupied: bool, lazy_eval: bool) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeNode

        2. update_node(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, node_key: erl_geometry.pyerl_geometry.QuadtreeKey, occupied: bool, lazy_eval: bool) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeNode

        3. update_node(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, x: float, y: float, log_odds_delta: float, lazy_eval: bool) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeNode

        4. update_node(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, node_key: erl_geometry.pyerl_geometry.QuadtreeKey, log_odds_delta: float, lazy_eval: bool) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeNode
        """
    @overload
    def update_node(self, x: float, y: float, log_odds_delta: float, lazy_eval: bool) -> OccupancyQuadtreeNode:
        """update_node(*args, **kwargs)
        Overloaded function.

        1. update_node(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, x: float, y: float, occupied: bool, lazy_eval: bool) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeNode

        2. update_node(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, node_key: erl_geometry.pyerl_geometry.QuadtreeKey, occupied: bool, lazy_eval: bool) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeNode

        3. update_node(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, x: float, y: float, log_odds_delta: float, lazy_eval: bool) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeNode

        4. update_node(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, node_key: erl_geometry.pyerl_geometry.QuadtreeKey, log_odds_delta: float, lazy_eval: bool) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeNode
        """
    @overload
    def update_node(self, node_key: QuadtreeKey, log_odds_delta: float, lazy_eval: bool) -> OccupancyQuadtreeNode:
        """update_node(*args, **kwargs)
        Overloaded function.

        1. update_node(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, x: float, y: float, occupied: bool, lazy_eval: bool) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeNode

        2. update_node(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, node_key: erl_geometry.pyerl_geometry.QuadtreeKey, occupied: bool, lazy_eval: bool) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeNode

        3. update_node(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, x: float, y: float, log_odds_delta: float, lazy_eval: bool) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeNode

        4. update_node(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, node_key: erl_geometry.pyerl_geometry.QuadtreeKey, log_odds_delta: float, lazy_eval: bool) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeNode
        """
    def visualize(self, leaf_only: bool = ..., area_min: numpy.ndarray[numpy.float32[2, 1]] | None = ..., area_max: numpy.ndarray[numpy.float32[2, 1]] | None = ..., resolution: float = ..., padding: int = ..., bg_color: numpy.ndarray[numpy.int32[4, 1]] = ..., fg_color: numpy.ndarray[numpy.int32[4, 1]] = ..., occupied_color: numpy.ndarray[numpy.int32[4, 1]] = ..., free_color: numpy.ndarray[numpy.int32[4, 1]] = ..., border_color: numpy.ndarray[numpy.int32[4, 1]] = ..., border_thickness: int = ...) -> numpy.ndarray[numpy.uint8[m, n]]:
        """visualize(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeD, leaf_only: bool = False, area_min: Optional[numpy.ndarray[numpy.float32[2, 1]]] = None, area_max: Optional[numpy.ndarray[numpy.float32[2, 1]]] = None, resolution: float = 0.1, padding: int = 1, bg_color: numpy.ndarray[numpy.int32[4, 1]] = array([128, 128, 128, 255], dtype=int32), fg_color: numpy.ndarray[numpy.int32[4, 1]] = array([255, 255, 255, 255], dtype=int32), occupied_color: numpy.ndarray[numpy.int32[4, 1]] = array([ 0, 0, 0, 255], dtype=int32), free_color: numpy.ndarray[numpy.int32[4, 1]] = array([255, 255, 255, 255], dtype=int32), border_color: numpy.ndarray[numpy.int32[4, 1]] = array([ 0, 0, 0, 255], dtype=int32), border_thickness: int = 1) -> numpy.ndarray[numpy.uint8[m, n]]"""
    @property
    def memory_usage(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeD) -> int"""
    @property
    def memory_usage_per_node(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeD) -> int"""
    @property
    def metric_aabb(self) -> Aabb2Dd:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeD) -> erl_geometry.pyerl_geometry.Aabb2Dd"""
    @property
    def metric_max(self) -> numpy.ndarray[numpy.float64[2, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeD) -> numpy.ndarray[numpy.float64[2, 1]]"""
    @property
    def metric_min(self) -> numpy.ndarray[numpy.float64[2, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeD) -> numpy.ndarray[numpy.float64[2, 1]]"""
    @property
    def metric_min_max(self) -> tuple[numpy.ndarray[numpy.float64[2, 1]], numpy.ndarray[numpy.float64[2, 1]]]:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeD) -> tuple[numpy.ndarray[numpy.float64[2, 1]], numpy.ndarray[numpy.float64[2, 1]]]"""
    @property
    def metric_size(self) -> numpy.ndarray[numpy.float64[2, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeD) -> numpy.ndarray[numpy.float64[2, 1]]"""
    @property
    def number_of_leaf_nodes(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeD) -> int"""
    @property
    def number_of_nodes(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeD) -> int"""
    @property
    def resolution(self) -> float:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeD) -> float"""
    @property
    def root(self) -> OccupancyQuadtreeNode:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeD) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeNode"""
    @property
    def setting(self) -> OccupancyQuadtreeBaseSetting:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeD) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeBaseSetting"""
    @property
    def tree_center(self) -> numpy.ndarray[numpy.float64[2, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeD) -> numpy.ndarray[numpy.float64[2, 1]]"""
    @property
    def tree_center_key(self) -> QuadtreeKey:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeD) -> erl_geometry.pyerl_geometry.QuadtreeKey"""
    @property
    def tree_depth(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeD) -> int"""
    @property
    def tree_max_half_size(self) -> numpy.ndarray[numpy.float64[2, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeD) -> numpy.ndarray[numpy.float64[2, 1]]"""

class OccupancyQuadtreeDrawerSettingD(YamlableBase):
    area_max: numpy.ndarray[numpy.float32[2, 1]]
    area_min: numpy.ndarray[numpy.float32[2, 1]]
    bg_color: Scalar
    border_color: Scalar
    border_thickness: int
    fg_color: Scalar
    free_color: Scalar
    occupied_color: Scalar
    padding: int
    resolution: float
    scaling: float
    def __init__(self) -> None:
        """__init__(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeDrawerSettingD) -> None"""

class OccupancyQuadtreeDrawerSettingF(YamlableBase):
    area_max: numpy.ndarray[numpy.float32[2, 1]]
    area_min: numpy.ndarray[numpy.float32[2, 1]]
    bg_color: Scalar
    border_color: Scalar
    border_thickness: int
    fg_color: Scalar
    free_color: Scalar
    occupied_color: Scalar
    padding: int
    resolution: float
    scaling: float
    def __init__(self) -> None:
        """__init__(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeDrawerSettingF) -> None"""

class OccupancyQuadtreeF(AbstractOccupancyQuadtreeF):
    class BatchRayCaster:
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""
        def step(self, mask: numpy.ndarray[bool[m, 1]] = ...) -> OccupancyQuadtreeF.BatchRayCaster:
            """step(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF.BatchRayCaster, mask: numpy.ndarray[bool[m, 1]] = array([], dtype=bool)) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeF.BatchRayCaster"""
        @property
        def ever_hit_flags(self) -> numpy.ndarray[bool[m, 1]]:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeF.BatchRayCaster) -> numpy.ndarray[bool[m, 1]]"""
        @property
        def frontier_keys(self) -> list[QuadtreeKey]:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeF.BatchRayCaster) -> list[erl_geometry.pyerl_geometry.QuadtreeKey]"""
        @property
        def frontier_nodes(self) -> list[OccupancyQuadtreeNode]:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeF.BatchRayCaster) -> list[erl_geometry.pyerl_geometry.OccupancyQuadtreeNode]"""
        @property
        def frontier_ray_indices(self) -> list[list[int]]:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeF.BatchRayCaster) -> list[list[int]]"""
        @property
        def hit_distances(self) -> numpy.ndarray[numpy.float32[m, 1]]:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeF.BatchRayCaster) -> numpy.ndarray[numpy.float32[m, 1]]"""
        @property
        def hit_flags(self) -> numpy.ndarray[bool[m, 1]]:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeF.BatchRayCaster) -> numpy.ndarray[bool[m, 1]]"""
        @property
        def hit_nodes(self) -> list[OccupancyQuadtreeNode]:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeF.BatchRayCaster) -> list[erl_geometry.pyerl_geometry.OccupancyQuadtreeNode]"""
        @property
        def hit_positions(self) -> list[numpy.ndarray[numpy.float32[2, 1]]]:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeF.BatchRayCaster) -> list[numpy.ndarray[numpy.float32[2, 1]]]"""
        @property
        def num_rays(self) -> int:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeF.BatchRayCaster) -> int"""
        @property
        def ray_directions(self) -> numpy.ndarray[numpy.float32[2, n]]:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeF.BatchRayCaster) -> numpy.ndarray[numpy.float32[2, n]]"""
        @property
        def ray_origins(self) -> numpy.ndarray[numpy.float32[2, n]]:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeF.BatchRayCaster) -> numpy.ndarray[numpy.float32[2, n]]"""

    class Drawer:
        def __init__(self, setting: OccupancyQuadtreeDrawerSettingF, quadtree: OccupancyQuadtreeF = ...) -> None:
            """__init__(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF.Drawer, setting: erl_geometry.pyerl_geometry.OccupancyQuadtreeDrawerSettingF, quadtree: erl_geometry.pyerl_geometry.OccupancyQuadtreeF = None) -> None"""
        @staticmethod
        def Setting() -> OccupancyQuadtreeDrawerSettingF:
            """Setting() -> erl_geometry.pyerl_geometry.OccupancyQuadtreeDrawerSettingF"""
        @overload
        def draw_leaves(self) -> Mat:
            """draw_leaves(*args, **kwargs)
            Overloaded function.

            1. draw_leaves(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF.Drawer) -> Mat

            2. draw_leaves(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF.Drawer, filename: str) -> None
            """
        @overload
        def draw_leaves(self, filename: str) -> None:
            """draw_leaves(*args, **kwargs)
            Overloaded function.

            1. draw_leaves(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF.Drawer) -> Mat

            2. draw_leaves(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF.Drawer, filename: str) -> None
            """
        @overload
        def draw_tree(self) -> Mat:
            """draw_tree(*args, **kwargs)
            Overloaded function.

            1. draw_tree(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF.Drawer) -> Mat

            2. draw_tree(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF.Drawer, filename: str) -> None
            """
        @overload
        def draw_tree(self, filename: str) -> None:
            """draw_tree(*args, **kwargs)
            Overloaded function.

            1. draw_tree(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF.Drawer) -> Mat

            2. draw_tree(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF.Drawer, filename: str) -> None
            """
        def set_draw_leaf_callback(self, callback: Callable[[OccupancyQuadtreeF.Drawer, Mat, OccupancyQuadtreeF.LeafIterator], None]) -> None:
            """set_draw_leaf_callback(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF.Drawer, callback: Callable[[erl_geometry.pyerl_geometry.OccupancyQuadtreeF.Drawer, Mat, erl_geometry.pyerl_geometry.OccupancyQuadtreeF.LeafIterator], None]) -> None"""
        def set_draw_tree_callback(self, callback: Callable[[OccupancyQuadtreeF.Drawer, Mat, OccupancyQuadtreeF.TreeIterator], None]) -> None:
            """set_draw_tree_callback(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF.Drawer, callback: Callable[[erl_geometry.pyerl_geometry.OccupancyQuadtreeF.Drawer, Mat, erl_geometry.pyerl_geometry.OccupancyQuadtreeF.TreeIterator], None]) -> None"""
        @property
        def grid_map_info(self):
            """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeF.Drawer) -> erl::common::GridMapInfo<float, 2>"""
        @property
        def setting(self) -> OccupancyQuadtreeDrawerSettingF:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeF.Drawer) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeDrawerSettingF"""

    class EastLeafNeighborIterator(OccupancyQuadtreeF.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class IteratorBase(AbstractQuadtreeF.QuadtreeNodeIterator):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""
        def __eq__(self, arg0: OccupancyQuadtreeF.IteratorBase) -> bool:
            """__eq__(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF.IteratorBase, arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeF.IteratorBase) -> bool"""
        def __ne__(self, arg0: OccupancyQuadtreeF.IteratorBase) -> bool:
            """__ne__(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF.IteratorBase, arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeF.IteratorBase) -> bool"""
        @property
        def index_key(self) -> QuadtreeKey:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeF.IteratorBase) -> erl_geometry.pyerl_geometry.QuadtreeKey"""
        @property
        def key(self) -> QuadtreeKey:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeF.IteratorBase) -> erl_geometry.pyerl_geometry.QuadtreeKey"""
        @property
        def node(self) -> OccupancyQuadtreeNode:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeF.IteratorBase) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeNode"""
        @property
        def node_aabb(self) -> Aabb2Df:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeF.IteratorBase) -> erl_geometry.pyerl_geometry.Aabb2Df"""

    class LeafInAabbIterator(OccupancyQuadtreeF.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class LeafIterator(OccupancyQuadtreeF.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class LeafOfNodeIterator(OccupancyQuadtreeF.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class NodeOnRayIterator(OccupancyQuadtreeF.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""
        @property
        def distance(self) -> float:
            """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeF.NodeOnRayIterator) -> float"""

    class NorthLeafNeighborIterator(OccupancyQuadtreeF.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class SouthLeafNeighborIterator(OccupancyQuadtreeF.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class TreeInAabbIterator(OccupancyQuadtreeF.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class TreeIterator(OccupancyQuadtreeF.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class WestLeafNeighborIterator(OccupancyQuadtreeF.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""
    @overload
    def __init__(self) -> None:
        """__init__(*args, **kwargs)
        Overloaded function.

        1. __init__(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF) -> None

        2. __init__(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, setting: erl_geometry.pyerl_geometry.OccupancyQuadtreeBaseSetting) -> None

        3. __init__(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, filename: str) -> None

        4. __init__(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, map_info: erl::common::GridMapInfo<float, 2>, image_map: Mat, occupied_threshold: float, padding: int = 0) -> None
        """
    @overload
    def __init__(self, setting: OccupancyQuadtreeBaseSetting) -> None:
        """__init__(*args, **kwargs)
        Overloaded function.

        1. __init__(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF) -> None

        2. __init__(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, setting: erl_geometry.pyerl_geometry.OccupancyQuadtreeBaseSetting) -> None

        3. __init__(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, filename: str) -> None

        4. __init__(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, map_info: erl::common::GridMapInfo<float, 2>, image_map: Mat, occupied_threshold: float, padding: int = 0) -> None
        """
    @overload
    def __init__(self, filename: str) -> None:
        """__init__(*args, **kwargs)
        Overloaded function.

        1. __init__(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF) -> None

        2. __init__(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, setting: erl_geometry.pyerl_geometry.OccupancyQuadtreeBaseSetting) -> None

        3. __init__(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, filename: str) -> None

        4. __init__(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, map_info: erl::common::GridMapInfo<float, 2>, image_map: Mat, occupied_threshold: float, padding: int = 0) -> None
        """
    def Setting(self) -> OccupancyQuadtreeBaseSetting:
        """Setting() -> erl_geometry.pyerl_geometry.OccupancyQuadtreeBaseSetting"""
    @overload
    def adjust_key_to_depth(self, key: int, depth: int) -> int:
        """adjust_key_to_depth(*args, **kwargs)
        Overloaded function.

        1. adjust_key_to_depth(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, key: int, depth: int) -> int

        2. adjust_key_to_depth(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> erl_geometry.pyerl_geometry.QuadtreeKey
        """
    @overload
    def adjust_key_to_depth(self, key: QuadtreeKey, depth: int) -> QuadtreeKey:
        """adjust_key_to_depth(*args, **kwargs)
        Overloaded function.

        1. adjust_key_to_depth(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, key: int, depth: int) -> int

        2. adjust_key_to_depth(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> erl_geometry.pyerl_geometry.QuadtreeKey
        """
    def cast_ray(self, px: float, py: float, vx: float, vy: float, ignore_unknown: bool, max_range: float) -> dict:
        """cast_ray(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, px: float, py: float, vx: float, vy: float, ignore_unknown: bool, max_range: float) -> dict"""
    @overload
    def cast_rays(self, position: numpy.ndarray[numpy.float32[2, 1]], rotation: numpy.ndarray[numpy.float32[2, 2], flags.f_contiguous], angles: numpy.ndarray[numpy.float32[m, 1]], ignore_unknown: bool, max_range: float, prune_rays: bool, parallel: bool) -> dict:
        """cast_rays(*args, **kwargs)
        Overloaded function.

        1. cast_rays(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, position: numpy.ndarray[numpy.float32[2, 1]], rotation: numpy.ndarray[numpy.float32[2, 2], flags.f_contiguous], angles: numpy.ndarray[numpy.float32[m, 1]], ignore_unknown: bool, max_range: float, prune_rays: bool, parallel: bool) -> dict

        2. cast_rays(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, positions: numpy.ndarray[numpy.float32[2, n], flags.f_contiguous], directions: numpy.ndarray[numpy.float32[2, n], flags.f_contiguous], ignore_unknown: bool, max_range: float, prune_rays: bool, parallel: bool) -> dict
        """
    @overload
    def cast_rays(self, positions: numpy.ndarray[numpy.float32[2, n], flags.f_contiguous], directions: numpy.ndarray[numpy.float32[2, n], flags.f_contiguous], ignore_unknown: bool, max_range: float, prune_rays: bool, parallel: bool) -> dict:
        """cast_rays(*args, **kwargs)
        Overloaded function.

        1. cast_rays(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, position: numpy.ndarray[numpy.float32[2, 1]], rotation: numpy.ndarray[numpy.float32[2, 2], flags.f_contiguous], angles: numpy.ndarray[numpy.float32[m, 1]], ignore_unknown: bool, max_range: float, prune_rays: bool, parallel: bool) -> dict

        2. cast_rays(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, positions: numpy.ndarray[numpy.float32[2, n], flags.f_contiguous], directions: numpy.ndarray[numpy.float32[2, n], flags.f_contiguous], ignore_unknown: bool, max_range: float, prune_rays: bool, parallel: bool) -> dict
        """
    def clear(self) -> None:
        """clear(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF) -> None"""
    def compute_common_ancestor_key(self, arg0: QuadtreeKey, arg1: QuadtreeKey) -> tuple[QuadtreeKey, int]:
        """compute_common_ancestor_key(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, arg0: erl_geometry.pyerl_geometry.QuadtreeKey, arg1: erl_geometry.pyerl_geometry.QuadtreeKey) -> tuple[erl_geometry.pyerl_geometry.QuadtreeKey, int]"""
    def compute_east_neighbor_key(self, key: QuadtreeKey, depth: int) -> QuadtreeKey | None:
        """compute_east_neighbor_key(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]"""
    def compute_north_neighbor_key(self, key: QuadtreeKey, depth: int) -> QuadtreeKey | None:
        """compute_north_neighbor_key(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]"""
    def compute_ray_coords(self, sx: float, sy: float, ex: float, ey: float) -> list[numpy.ndarray[numpy.float32[2, 1]]] | None:
        """compute_ray_coords(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, sx: float, sy: float, ex: float, ey: float) -> Optional[list[numpy.ndarray[numpy.float32[2, 1]]]]"""
    def compute_ray_keys(self, sx: float, sy: float, ex: float, ey: float) -> list[QuadtreeKey] | None:
        """compute_ray_keys(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, sx: float, sy: float, ex: float, ey: float) -> Optional[list[erl_geometry.pyerl_geometry.QuadtreeKey]]"""
    def compute_south_neighbor_key(self, key: QuadtreeKey, depth: int) -> QuadtreeKey | None:
        """compute_south_neighbor_key(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]"""
    def compute_west_neighbor_key(self, key: QuadtreeKey, depth: int) -> QuadtreeKey | None:
        """compute_west_neighbor_key(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]"""
    @overload
    def coord_to_key(self, coordinate: float) -> int:
        """coord_to_key(*args, **kwargs)
        Overloaded function.

        1. coord_to_key(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, coordinate: float) -> int

        2. coord_to_key(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, coordinate: float, depth: int) -> int

        3. coord_to_key(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, x: float, y: float) -> erl_geometry.pyerl_geometry.QuadtreeKey

        4. coord_to_key(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, x: float, y: float, depth: int) -> erl_geometry.pyerl_geometry.QuadtreeKey
        """
    @overload
    def coord_to_key(self, coordinate: float, depth: int) -> int:
        """coord_to_key(*args, **kwargs)
        Overloaded function.

        1. coord_to_key(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, coordinate: float) -> int

        2. coord_to_key(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, coordinate: float, depth: int) -> int

        3. coord_to_key(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, x: float, y: float) -> erl_geometry.pyerl_geometry.QuadtreeKey

        4. coord_to_key(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, x: float, y: float, depth: int) -> erl_geometry.pyerl_geometry.QuadtreeKey
        """
    @overload
    def coord_to_key(self, x: float, y: float) -> QuadtreeKey:
        """coord_to_key(*args, **kwargs)
        Overloaded function.

        1. coord_to_key(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, coordinate: float) -> int

        2. coord_to_key(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, coordinate: float, depth: int) -> int

        3. coord_to_key(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, x: float, y: float) -> erl_geometry.pyerl_geometry.QuadtreeKey

        4. coord_to_key(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, x: float, y: float, depth: int) -> erl_geometry.pyerl_geometry.QuadtreeKey
        """
    @overload
    def coord_to_key(self, x: float, y: float, depth: int) -> QuadtreeKey:
        """coord_to_key(*args, **kwargs)
        Overloaded function.

        1. coord_to_key(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, coordinate: float) -> int

        2. coord_to_key(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, coordinate: float, depth: int) -> int

        3. coord_to_key(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, x: float, y: float) -> erl_geometry.pyerl_geometry.QuadtreeKey

        4. coord_to_key(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, x: float, y: float, depth: int) -> erl_geometry.pyerl_geometry.QuadtreeKey
        """
    @overload
    def coord_to_key_checked(self, coordinate: float) -> int | None:
        """coord_to_key_checked(*args, **kwargs)
        Overloaded function.

        1. coord_to_key_checked(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, coordinate: float) -> Optional[int]

        2. coord_to_key_checked(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, coordinate: float, depth: int) -> Optional[int]

        3. coord_to_key_checked(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, x: float, y: float) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]

        4. coord_to_key_checked(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, x: float, y: float, depth: int) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]
        """
    @overload
    def coord_to_key_checked(self, coordinate: float, depth: int) -> int | None:
        """coord_to_key_checked(*args, **kwargs)
        Overloaded function.

        1. coord_to_key_checked(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, coordinate: float) -> Optional[int]

        2. coord_to_key_checked(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, coordinate: float, depth: int) -> Optional[int]

        3. coord_to_key_checked(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, x: float, y: float) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]

        4. coord_to_key_checked(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, x: float, y: float, depth: int) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]
        """
    @overload
    def coord_to_key_checked(self, x: float, y: float) -> QuadtreeKey | None:
        """coord_to_key_checked(*args, **kwargs)
        Overloaded function.

        1. coord_to_key_checked(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, coordinate: float) -> Optional[int]

        2. coord_to_key_checked(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, coordinate: float, depth: int) -> Optional[int]

        3. coord_to_key_checked(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, x: float, y: float) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]

        4. coord_to_key_checked(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, x: float, y: float, depth: int) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]
        """
    @overload
    def coord_to_key_checked(self, x: float, y: float, depth: int) -> QuadtreeKey | None:
        """coord_to_key_checked(*args, **kwargs)
        Overloaded function.

        1. coord_to_key_checked(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, coordinate: float) -> Optional[int]

        2. coord_to_key_checked(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, coordinate: float, depth: int) -> Optional[int]

        3. coord_to_key_checked(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, x: float, y: float) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]

        4. coord_to_key_checked(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, x: float, y: float, depth: int) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]
        """
    def create_node_child(self, node: OccupancyQuadtreeNode, child_idx: int) -> OccupancyQuadtreeNode:
        """create_node_child(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, node: erl_geometry.pyerl_geometry.OccupancyQuadtreeNode, child_idx: int) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeNode"""
    @overload
    def delete_node(self, x: float, y: float, depth: int) -> int:
        """delete_node(*args, **kwargs)
        Overloaded function.

        1. delete_node(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, x: float, y: float, depth: int) -> int

        2. delete_node(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> None
        """
    @overload
    def delete_node(self, key: QuadtreeKey, depth: int) -> None:
        """delete_node(*args, **kwargs)
        Overloaded function.

        1. delete_node(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, x: float, y: float, depth: int) -> int

        2. delete_node(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> None
        """
    def delete_node_child(self, node: OccupancyQuadtreeNode, child_idx: int, key: QuadtreeKey) -> int:
        """delete_node_child(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, node: erl_geometry.pyerl_geometry.OccupancyQuadtreeNode, child_idx: int, key: erl_geometry.pyerl_geometry.QuadtreeKey) -> int"""
    def expand(self) -> None:
        """expand(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF) -> None"""
    def expand_node(self, node: OccupancyQuadtreeNode) -> None:
        """expand_node(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, node: erl_geometry.pyerl_geometry.OccupancyQuadtreeNode) -> None"""
    def get_batch_ray_caster(self, *args, **kwargs):
        """get_batch_ray_caster(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, origins: numpy.ndarray[numpy.float32[2, n]], directions: numpy.ndarray[numpy.float32[2, n]], max_ranges: numpy.ndarray[numpy.float32[m, 1]] = array([], dtype=float32), node_paddings: numpy.ndarray[numpy.float32[m, 1]] = array([], dtype=float32), bidirectional_flags: numpy.ndarray[bool[m, 1]] = array([], dtype=bool), leaf_only_flags: numpy.ndarray[bool[m, 1]] = array([], dtype=bool), min_node_depths: numpy.ndarray[numpy.int32[m, 1]] = array([], dtype=int32), max_node_depths: numpy.ndarray[numpy.int32[m, 1]] = array([], dtype=int32)) -> erl::geometry::OccupancyNdTreeBatchRayCaster<erl::geometry::OccupancyQuadtreeBase<float, erl::geometry::OccupancyQuadtreeNode, erl::geometry::OccupancyQuadtreeBaseSetting>, 2>"""
    def get_node_child(self, node: OccupancyQuadtreeNode, child_idx: int) -> OccupancyQuadtreeNode:
        """get_node_child(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, node: erl_geometry.pyerl_geometry.OccupancyQuadtreeNode, child_idx: int) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeNode"""
    def get_node_size(self, depth: int) -> float:
        """get_node_size(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, depth: int) -> float"""
    @overload
    def insert_node(self, x: float, y: float, depth: int) -> OccupancyQuadtreeNode:
        """insert_node(*args, **kwargs)
        Overloaded function.

        1. insert_node(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, x: float, y: float, depth: int) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeNode

        2. insert_node(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeNode
        """
    @overload
    def insert_node(self, key: QuadtreeKey, depth: int) -> OccupancyQuadtreeNode:
        """insert_node(*args, **kwargs)
        Overloaded function.

        1. insert_node(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, x: float, y: float, depth: int) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeNode

        2. insert_node(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeNode
        """
    def insert_point_cloud(self, points: numpy.ndarray[numpy.float32[2, n], flags.f_contiguous], sensor_origin: numpy.ndarray[numpy.float32[2, 1]], min_range: float, max_range: float, with_count: bool, parallel: bool, lazy_eval: bool, discrete: bool) -> None:
        """insert_point_cloud(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, points: numpy.ndarray[numpy.float32[2, n], flags.f_contiguous], sensor_origin: numpy.ndarray[numpy.float32[2, 1]], min_range: float, max_range: float, with_count: bool, parallel: bool, lazy_eval: bool, discrete: bool) -> None"""
    def insert_point_cloud_rays(self, points: numpy.ndarray[numpy.float32[2, n], flags.f_contiguous], sensor_origin: numpy.ndarray[numpy.float32[2, 1]], min_range: float, max_range: float, parallel: bool, lazy_eval: bool) -> None:
        """insert_point_cloud_rays(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, points: numpy.ndarray[numpy.float32[2, n], flags.f_contiguous], sensor_origin: numpy.ndarray[numpy.float32[2, 1]], min_range: float, max_range: float, parallel: bool, lazy_eval: bool) -> None"""
    def insert_ray(self, sx: float, sy: float, ex: float, ey: float, min_range: float, max_range: float, lazy_eval: bool) -> bool:
        """insert_ray(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, sx: float, sy: float, ex: float, ey: float, min_range: float, max_range: float, lazy_eval: bool) -> bool"""
    def is_node_collapsible(self, node: OccupancyQuadtreeNode) -> bool:
        """is_node_collapsible(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, node: erl_geometry.pyerl_geometry.OccupancyQuadtreeNode) -> bool"""
    @overload
    def iter_east_leaf_neighbor(self, x: float, y: float, max_leaf_depth: int = ...) -> Iterator[OccupancyQuadtreeF.EastLeafNeighborIterator]:
        """iter_east_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_east_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, x: float, y: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyQuadtreeF.EastLeafNeighborIterator]

        2. iter_east_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyQuadtreeF.EastLeafNeighborIterator]
        """
    @overload
    def iter_east_leaf_neighbor(self, key: QuadtreeKey, key_depth: int, max_leaf_depth: int = ...) -> Iterator[OccupancyQuadtreeF.EastLeafNeighborIterator]:
        """iter_east_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_east_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, x: float, y: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyQuadtreeF.EastLeafNeighborIterator]

        2. iter_east_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyQuadtreeF.EastLeafNeighborIterator]
        """
    def iter_leaf(self, max_depth: int = ...) -> Iterator[OccupancyQuadtreeF.LeafIterator]:
        """iter_leaf(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyQuadtreeF.LeafIterator]"""
    @overload
    def iter_leaf_in_aabb(self, aabb_min_x: float, aabb_min_y: float, aabb_max_x: float, aabb_max_y: float, max_depth: int = ...) -> Iterator[OccupancyQuadtreeF.LeafInAabbIterator]:
        """iter_leaf_in_aabb(*args, **kwargs)
        Overloaded function.

        1. iter_leaf_in_aabb(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, aabb_min_x: float, aabb_min_y: float, aabb_max_x: float, aabb_max_y: float, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyQuadtreeF.LeafInAabbIterator]

        2. iter_leaf_in_aabb(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, aabb_min_key: erl_geometry.pyerl_geometry.QuadtreeKey, aabb_max_key: erl_geometry.pyerl_geometry.QuadtreeKey, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyQuadtreeF.LeafInAabbIterator]
        """
    @overload
    def iter_leaf_in_aabb(self, aabb_min_key: QuadtreeKey, aabb_max_key: QuadtreeKey, max_depth: int = ...) -> Iterator[OccupancyQuadtreeF.LeafInAabbIterator]:
        """iter_leaf_in_aabb(*args, **kwargs)
        Overloaded function.

        1. iter_leaf_in_aabb(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, aabb_min_x: float, aabb_min_y: float, aabb_max_x: float, aabb_max_y: float, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyQuadtreeF.LeafInAabbIterator]

        2. iter_leaf_in_aabb(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, aabb_min_key: erl_geometry.pyerl_geometry.QuadtreeKey, aabb_max_key: erl_geometry.pyerl_geometry.QuadtreeKey, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyQuadtreeF.LeafInAabbIterator]
        """
    def iter_leaf_of_node(self, node_key: QuadtreeKey, node_depth: int, max_depth: int = ...) -> Iterator[OccupancyQuadtreeF.LeafOfNodeIterator]:
        """iter_leaf_of_node(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, node_key: erl_geometry.pyerl_geometry.QuadtreeKey, node_depth: int, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyQuadtreeF.LeafOfNodeIterator]"""
    def iter_node(self, max_depth: int = ...) -> Iterator[OccupancyQuadtreeF.TreeIterator]:
        """iter_node(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyQuadtreeF.TreeIterator]"""
    @overload
    def iter_node_in_aabb(self, aabb_min_x: float, aabb_min_y: float, aabb_max_x: float, aabb_max_y: float, max_depth: int = ...) -> Iterator[OccupancyQuadtreeF.TreeInAabbIterator]:
        """iter_node_in_aabb(*args, **kwargs)
        Overloaded function.

        1. iter_node_in_aabb(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, aabb_min_x: float, aabb_min_y: float, aabb_max_x: float, aabb_max_y: float, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyQuadtreeF.TreeInAabbIterator]

        2. iter_node_in_aabb(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, aabb_min_key: erl_geometry.pyerl_geometry.QuadtreeKey, aabb_max_key: erl_geometry.pyerl_geometry.QuadtreeKey, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyQuadtreeF.TreeInAabbIterator]
        """
    @overload
    def iter_node_in_aabb(self, aabb_min_key: QuadtreeKey, aabb_max_key: QuadtreeKey, max_depth: int = ...) -> Iterator[OccupancyQuadtreeF.TreeInAabbIterator]:
        """iter_node_in_aabb(*args, **kwargs)
        Overloaded function.

        1. iter_node_in_aabb(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, aabb_min_x: float, aabb_min_y: float, aabb_max_x: float, aabb_max_y: float, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyQuadtreeF.TreeInAabbIterator]

        2. iter_node_in_aabb(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, aabb_min_key: erl_geometry.pyerl_geometry.QuadtreeKey, aabb_max_key: erl_geometry.pyerl_geometry.QuadtreeKey, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyQuadtreeF.TreeInAabbIterator]
        """
    def iter_node_on_ray(self, px: float, py: float, vx: float, vy: float, max_range: float = ..., node_padding: float = ..., bidirectional: bool = ..., leaf_only: bool = ..., min_node_depth: int = ..., max_node_depth: int = ...) -> Iterator[OccupancyQuadtreeF.NodeOnRayIterator]:
        """iter_node_on_ray(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, px: float, py: float, vx: float, vy: float, max_range: float = -1, node_padding: float = 0, bidirectional: bool = False, leaf_only: bool = False, min_node_depth: int = 0, max_node_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyQuadtreeF.NodeOnRayIterator]"""
    @overload
    def iter_north_leaf_neighbor(self, x: float, y: float, max_leaf_depth: int = ...) -> Iterator[OccupancyQuadtreeF.NorthLeafNeighborIterator]:
        """iter_north_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_north_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, x: float, y: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyQuadtreeF.NorthLeafNeighborIterator]

        2. iter_north_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyQuadtreeF.NorthLeafNeighborIterator]
        """
    @overload
    def iter_north_leaf_neighbor(self, key: QuadtreeKey, key_depth: int, max_leaf_depth: int = ...) -> Iterator[OccupancyQuadtreeF.NorthLeafNeighborIterator]:
        """iter_north_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_north_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, x: float, y: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyQuadtreeF.NorthLeafNeighborIterator]

        2. iter_north_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyQuadtreeF.NorthLeafNeighborIterator]
        """
    @overload
    def iter_south_leaf_neighbor(self, x: float, y: float, max_leaf_depth: int = ...) -> Iterator[OccupancyQuadtreeF.SouthLeafNeighborIterator]:
        """iter_south_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_south_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, x: float, y: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyQuadtreeF.SouthLeafNeighborIterator]

        2. iter_south_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyQuadtreeF.SouthLeafNeighborIterator]
        """
    @overload
    def iter_south_leaf_neighbor(self, key: QuadtreeKey, key_depth: int, max_leaf_depth: int = ...) -> Iterator[OccupancyQuadtreeF.SouthLeafNeighborIterator]:
        """iter_south_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_south_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, x: float, y: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyQuadtreeF.SouthLeafNeighborIterator]

        2. iter_south_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyQuadtreeF.SouthLeafNeighborIterator]
        """
    @overload
    def iter_west_leaf_neighbor(self, x: float, y: float, max_leaf_depth: int = ...) -> Iterator[OccupancyQuadtreeF.WestLeafNeighborIterator]:
        """iter_west_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_west_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, x: float, y: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyQuadtreeF.WestLeafNeighborIterator]

        2. iter_west_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyQuadtreeF.WestLeafNeighborIterator]
        """
    @overload
    def iter_west_leaf_neighbor(self, key: QuadtreeKey, key_depth: int, max_leaf_depth: int = ...) -> Iterator[OccupancyQuadtreeF.WestLeafNeighborIterator]:
        """iter_west_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_west_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, x: float, y: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyQuadtreeF.WestLeafNeighborIterator]

        2. iter_west_leaf_neighbor(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.OccupancyQuadtreeF.WestLeafNeighborIterator]
        """
    @overload
    def key_to_coord(self, key: int) -> float:
        """key_to_coord(*args, **kwargs)
        Overloaded function.

        1. key_to_coord(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, key: int) -> float

        2. key_to_coord(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, key: int, depth: int) -> float

        3. key_to_coord(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey) -> tuple[float, float]

        4. key_to_coord(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> tuple[float, float]
        """
    @overload
    def key_to_coord(self, key: int, depth: int) -> float:
        """key_to_coord(*args, **kwargs)
        Overloaded function.

        1. key_to_coord(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, key: int) -> float

        2. key_to_coord(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, key: int, depth: int) -> float

        3. key_to_coord(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey) -> tuple[float, float]

        4. key_to_coord(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> tuple[float, float]
        """
    @overload
    def key_to_coord(self, key: QuadtreeKey) -> tuple[float, float]:
        """key_to_coord(*args, **kwargs)
        Overloaded function.

        1. key_to_coord(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, key: int) -> float

        2. key_to_coord(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, key: int, depth: int) -> float

        3. key_to_coord(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey) -> tuple[float, float]

        4. key_to_coord(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> tuple[float, float]
        """
    @overload
    def key_to_coord(self, key: QuadtreeKey, depth: int) -> tuple[float, float]:
        """key_to_coord(*args, **kwargs)
        Overloaded function.

        1. key_to_coord(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, key: int) -> float

        2. key_to_coord(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, key: int, depth: int) -> float

        3. key_to_coord(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey) -> tuple[float, float]

        4. key_to_coord(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> tuple[float, float]
        """
    def prune(self) -> None:
        """prune(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF) -> None"""
    def prune_node(self, node: OccupancyQuadtreeNode) -> bool:
        """prune_node(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, node: erl_geometry.pyerl_geometry.OccupancyQuadtreeNode) -> bool"""
    def sample_positions(self, num_positions: int) -> list[numpy.ndarray[numpy.float32[2, 1]]]:
        """sample_positions(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, num_positions: int) -> list[numpy.ndarray[numpy.float32[2, 1]]]"""
    @overload
    def search(self, x: float, y: float, max_depth: int = ...) -> OccupancyQuadtreeNode:
        """search(*args, **kwargs)
        Overloaded function.

        1. search(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, x: float, y: float, max_depth: int = 0) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeNode

        2. search(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, max_depth: int = 0) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeNode
        """
    @overload
    def search(self, key: QuadtreeKey, max_depth: int = ...) -> OccupancyQuadtreeNode:
        """search(*args, **kwargs)
        Overloaded function.

        1. search(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, x: float, y: float, max_depth: int = 0) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeNode

        2. search(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, max_depth: int = 0) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeNode
        """
    def to_max_likelihood(self) -> None:
        """to_max_likelihood(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF) -> None"""
    def update_inner_occupancy(self) -> None:
        """update_inner_occupancy(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF) -> None"""
    @overload
    def update_node(self, x: float, y: float, occupied: bool, lazy_eval: bool) -> OccupancyQuadtreeNode:
        """update_node(*args, **kwargs)
        Overloaded function.

        1. update_node(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, x: float, y: float, occupied: bool, lazy_eval: bool) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeNode

        2. update_node(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, node_key: erl_geometry.pyerl_geometry.QuadtreeKey, occupied: bool, lazy_eval: bool) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeNode

        3. update_node(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, x: float, y: float, log_odds_delta: float, lazy_eval: bool) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeNode

        4. update_node(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, node_key: erl_geometry.pyerl_geometry.QuadtreeKey, log_odds_delta: float, lazy_eval: bool) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeNode
        """
    @overload
    def update_node(self, node_key: QuadtreeKey, occupied: bool, lazy_eval: bool) -> OccupancyQuadtreeNode:
        """update_node(*args, **kwargs)
        Overloaded function.

        1. update_node(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, x: float, y: float, occupied: bool, lazy_eval: bool) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeNode

        2. update_node(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, node_key: erl_geometry.pyerl_geometry.QuadtreeKey, occupied: bool, lazy_eval: bool) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeNode

        3. update_node(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, x: float, y: float, log_odds_delta: float, lazy_eval: bool) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeNode

        4. update_node(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, node_key: erl_geometry.pyerl_geometry.QuadtreeKey, log_odds_delta: float, lazy_eval: bool) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeNode
        """
    @overload
    def update_node(self, x: float, y: float, log_odds_delta: float, lazy_eval: bool) -> OccupancyQuadtreeNode:
        """update_node(*args, **kwargs)
        Overloaded function.

        1. update_node(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, x: float, y: float, occupied: bool, lazy_eval: bool) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeNode

        2. update_node(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, node_key: erl_geometry.pyerl_geometry.QuadtreeKey, occupied: bool, lazy_eval: bool) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeNode

        3. update_node(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, x: float, y: float, log_odds_delta: float, lazy_eval: bool) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeNode

        4. update_node(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, node_key: erl_geometry.pyerl_geometry.QuadtreeKey, log_odds_delta: float, lazy_eval: bool) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeNode
        """
    @overload
    def update_node(self, node_key: QuadtreeKey, log_odds_delta: float, lazy_eval: bool) -> OccupancyQuadtreeNode:
        """update_node(*args, **kwargs)
        Overloaded function.

        1. update_node(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, x: float, y: float, occupied: bool, lazy_eval: bool) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeNode

        2. update_node(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, node_key: erl_geometry.pyerl_geometry.QuadtreeKey, occupied: bool, lazy_eval: bool) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeNode

        3. update_node(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, x: float, y: float, log_odds_delta: float, lazy_eval: bool) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeNode

        4. update_node(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, node_key: erl_geometry.pyerl_geometry.QuadtreeKey, log_odds_delta: float, lazy_eval: bool) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeNode
        """
    def visualize(self, leaf_only: bool = ..., area_min: numpy.ndarray[numpy.float32[2, 1]] | None = ..., area_max: numpy.ndarray[numpy.float32[2, 1]] | None = ..., resolution: float = ..., padding: int = ..., bg_color: numpy.ndarray[numpy.int32[4, 1]] = ..., fg_color: numpy.ndarray[numpy.int32[4, 1]] = ..., occupied_color: numpy.ndarray[numpy.int32[4, 1]] = ..., free_color: numpy.ndarray[numpy.int32[4, 1]] = ..., border_color: numpy.ndarray[numpy.int32[4, 1]] = ..., border_thickness: int = ...) -> numpy.ndarray[numpy.uint8[m, n]]:
        """visualize(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeF, leaf_only: bool = False, area_min: Optional[numpy.ndarray[numpy.float32[2, 1]]] = None, area_max: Optional[numpy.ndarray[numpy.float32[2, 1]]] = None, resolution: float = 0.1, padding: int = 1, bg_color: numpy.ndarray[numpy.int32[4, 1]] = array([128, 128, 128, 255], dtype=int32), fg_color: numpy.ndarray[numpy.int32[4, 1]] = array([255, 255, 255, 255], dtype=int32), occupied_color: numpy.ndarray[numpy.int32[4, 1]] = array([ 0, 0, 0, 255], dtype=int32), free_color: numpy.ndarray[numpy.int32[4, 1]] = array([255, 255, 255, 255], dtype=int32), border_color: numpy.ndarray[numpy.int32[4, 1]] = array([ 0, 0, 0, 255], dtype=int32), border_thickness: int = 1) -> numpy.ndarray[numpy.uint8[m, n]]"""
    @property
    def memory_usage(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeF) -> int"""
    @property
    def memory_usage_per_node(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeF) -> int"""
    @property
    def metric_aabb(self) -> Aabb2Df:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeF) -> erl_geometry.pyerl_geometry.Aabb2Df"""
    @property
    def metric_max(self) -> numpy.ndarray[numpy.float32[2, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeF) -> numpy.ndarray[numpy.float32[2, 1]]"""
    @property
    def metric_min(self) -> numpy.ndarray[numpy.float32[2, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeF) -> numpy.ndarray[numpy.float32[2, 1]]"""
    @property
    def metric_min_max(self) -> tuple[numpy.ndarray[numpy.float32[2, 1]], numpy.ndarray[numpy.float32[2, 1]]]:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeF) -> tuple[numpy.ndarray[numpy.float32[2, 1]], numpy.ndarray[numpy.float32[2, 1]]]"""
    @property
    def metric_size(self) -> numpy.ndarray[numpy.float32[2, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeF) -> numpy.ndarray[numpy.float32[2, 1]]"""
    @property
    def number_of_leaf_nodes(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeF) -> int"""
    @property
    def number_of_nodes(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeF) -> int"""
    @property
    def resolution(self) -> float:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeF) -> float"""
    @property
    def root(self) -> OccupancyQuadtreeNode:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeF) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeNode"""
    @property
    def setting(self) -> OccupancyQuadtreeBaseSetting:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeF) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeBaseSetting"""
    @property
    def tree_center(self) -> numpy.ndarray[numpy.float32[2, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeF) -> numpy.ndarray[numpy.float32[2, 1]]"""
    @property
    def tree_center_key(self) -> QuadtreeKey:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeF) -> erl_geometry.pyerl_geometry.QuadtreeKey"""
    @property
    def tree_depth(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeF) -> int"""
    @property
    def tree_max_half_size(self) -> numpy.ndarray[numpy.float32[2, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeF) -> numpy.ndarray[numpy.float32[2, 1]]"""

class OccupancyQuadtreeNode(AbstractQuadtreeNode):
    def __init__(self, *args, **kwargs) -> None:
        """Initialize self.  See help(type(self)) for accurate signature."""
    def add_log_odds(self, log_odds: float) -> None:
        """add_log_odds(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeNode, log_odds: float) -> None"""
    def allow_update_log_odds(self, delta: float) -> bool:
        """allow_update_log_odds(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeNode, delta: float) -> bool"""
    def get_child(self, child_idx: int) -> OccupancyQuadtreeNode:
        """get_child(self: erl_geometry.pyerl_geometry.OccupancyQuadtreeNode, child_idx: int) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeNode"""
    @property
    def log_odds(self) -> float:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeNode) -> float"""
    @property
    def max_child_log_odds(self) -> float:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeNode) -> float"""
    @property
    def mean_child_log_odds(self) -> float:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeNode) -> float"""
    @property
    def occupancy(self) -> float:
        """(arg0: erl_geometry.pyerl_geometry.OccupancyQuadtreeNode) -> float"""

class OctreeKey:
    @overload
    def __init__(self) -> None:
        """__init__(*args, **kwargs)
        Overloaded function.

        1. __init__(self: erl_geometry.pyerl_geometry.OctreeKey) -> None

        2. __init__(self: erl_geometry.pyerl_geometry.OctreeKey, a: int, b: int, c: int) -> None
        """
    @overload
    def __init__(self, a: int, b: int, c: int) -> None:
        """__init__(*args, **kwargs)
        Overloaded function.

        1. __init__(self: erl_geometry.pyerl_geometry.OctreeKey) -> None

        2. __init__(self: erl_geometry.pyerl_geometry.OctreeKey, a: int, b: int, c: int) -> None
        """
    def to_list(self) -> list:
        """to_list(self: erl_geometry.pyerl_geometry.OctreeKey) -> list"""
    def __eq__(self, arg0: OctreeKey) -> bool:
        """__eq__(self: erl_geometry.pyerl_geometry.OctreeKey, arg0: erl_geometry.pyerl_geometry.OctreeKey) -> bool"""
    def __getitem__(self, arg0: int) -> int:
        """__getitem__(self: erl_geometry.pyerl_geometry.OctreeKey, arg0: int) -> int"""
    def __iter__(self) -> typing.Iterator[int]:
        """def __iter__(self) -> typing.Iterator[int]"""
    def __hash__(self) -> int:
        """__hash__(self: erl_geometry.pyerl_geometry.OctreeKey) -> int"""
    def __ne__(self, arg0: OctreeKey) -> bool:
        """__ne__(self: erl_geometry.pyerl_geometry.OctreeKey, arg0: erl_geometry.pyerl_geometry.OctreeKey) -> bool"""

class Primitive2D:
    class Type:
        """Members:

          kLine2D

          kSegment2D

          kRay2D

          kAxisAlignedRectangle

          kRectangle

          kEllipse"""
        __members__: ClassVar[dict] = ...  # read-only
        __entries: ClassVar[dict] = ...
        kAxisAlignedRectangle: ClassVar[Primitive2D.Type] = ...
        kEllipse: ClassVar[Primitive2D.Type] = ...
        kLine2D: ClassVar[Primitive2D.Type] = ...
        kRay2D: ClassVar[Primitive2D.Type] = ...
        kRectangle: ClassVar[Primitive2D.Type] = ...
        kSegment2D: ClassVar[Primitive2D.Type] = ...
        def __init__(self, value: int) -> None:
            """__init__(self: erl_geometry.pyerl_geometry.Primitive2D.Type, value: int) -> None"""
        def __eq__(self, other: object) -> bool:
            """__eq__(self: object, other: object) -> bool"""
        def __hash__(self) -> int:
            """__hash__(self: object) -> int"""
        def __index__(self) -> int:
            """__index__(self: erl_geometry.pyerl_geometry.Primitive2D.Type) -> int"""
        def __int__(self) -> int:
            """__int__(self: erl_geometry.pyerl_geometry.Primitive2D.Type) -> int"""
        def __ne__(self, other: object) -> bool:
            """__ne__(self: object, other: object) -> bool"""
        @property
        def name(self) -> str:
            """name(self: object) -> str

            name(self: object) -> str
            """
        @property
        def value(self) -> int:
            """(arg0: erl_geometry.pyerl_geometry.Primitive2D.Type) -> int"""
    kAxisAlignedRectangle: ClassVar[Primitive2D.Type] = ...
    kEllipse: ClassVar[Primitive2D.Type] = ...
    kLine2D: ClassVar[Primitive2D.Type] = ...
    kRay2D: ClassVar[Primitive2D.Type] = ...
    kRectangle: ClassVar[Primitive2D.Type] = ...
    kSegment2D: ClassVar[Primitive2D.Type] = ...
    id: int
    def __init__(self, *args, **kwargs) -> None:
        """Initialize self.  See help(type(self)) for accurate signature."""
    @overload
    def compute_intersections(self, line) -> list[numpy.ndarray[numpy.float64[2, 1]]]:
        """compute_intersections(*args, **kwargs)
        Overloaded function.

        1. compute_intersections(self: erl_geometry.pyerl_geometry.Primitive2D, line: erl::geometry::Line2D) -> list[numpy.ndarray[numpy.float64[2, 1]]]

        2. compute_intersections(self: erl_geometry.pyerl_geometry.Primitive2D, segment: erl::geometry::Segment2D) -> list[numpy.ndarray[numpy.float64[2, 1]]]

        3. compute_intersections(self: erl_geometry.pyerl_geometry.Primitive2D, ray: erl::geometry::Ray2D) -> list[numpy.ndarray[numpy.float64[2, 1]]]
        """
    @overload
    def compute_intersections(self, segment) -> list[numpy.ndarray[numpy.float64[2, 1]]]:
        """compute_intersections(*args, **kwargs)
        Overloaded function.

        1. compute_intersections(self: erl_geometry.pyerl_geometry.Primitive2D, line: erl::geometry::Line2D) -> list[numpy.ndarray[numpy.float64[2, 1]]]

        2. compute_intersections(self: erl_geometry.pyerl_geometry.Primitive2D, segment: erl::geometry::Segment2D) -> list[numpy.ndarray[numpy.float64[2, 1]]]

        3. compute_intersections(self: erl_geometry.pyerl_geometry.Primitive2D, ray: erl::geometry::Ray2D) -> list[numpy.ndarray[numpy.float64[2, 1]]]
        """
    @overload
    def compute_intersections(self, ray) -> list[numpy.ndarray[numpy.float64[2, 1]]]:
        """compute_intersections(*args, **kwargs)
        Overloaded function.

        1. compute_intersections(self: erl_geometry.pyerl_geometry.Primitive2D, line: erl::geometry::Line2D) -> list[numpy.ndarray[numpy.float64[2, 1]]]

        2. compute_intersections(self: erl_geometry.pyerl_geometry.Primitive2D, segment: erl::geometry::Segment2D) -> list[numpy.ndarray[numpy.float64[2, 1]]]

        3. compute_intersections(self: erl_geometry.pyerl_geometry.Primitive2D, ray: erl::geometry::Ray2D) -> list[numpy.ndarray[numpy.float64[2, 1]]]
        """
    def is_inside(self, point: numpy.ndarray[numpy.float64[2, 1]]) -> bool:
        """is_inside(self: erl_geometry.pyerl_geometry.Primitive2D, point: numpy.ndarray[numpy.float64[2, 1]]) -> bool"""
    def is_on_boundary(self, point: numpy.ndarray[numpy.float64[2, 1]]) -> bool:
        """is_on_boundary(self: erl_geometry.pyerl_geometry.Primitive2D, point: numpy.ndarray[numpy.float64[2, 1]]) -> bool"""
    @property
    def orientation_angle(self) -> float:
        """(arg0: erl_geometry.pyerl_geometry.Primitive2D) -> float"""
    @property
    def type(self) -> Primitive2D.Type:
        """(arg0: erl_geometry.pyerl_geometry.Primitive2D) -> erl_geometry.pyerl_geometry.Primitive2D.Type"""

class Primitive3D:
    class Type:
        """Members:

          kLine3D

          kSegment3D

          kRay3D

          kPlane

          kTriangle

          kAxisAlignedBox

          kBox

          kEllipsoid"""
        __members__: ClassVar[dict] = ...  # read-only
        __entries: ClassVar[dict] = ...
        kAxisAlignedBox: ClassVar[Primitive3D.Type] = ...
        kBox: ClassVar[Primitive3D.Type] = ...
        kEllipsoid: ClassVar[Primitive3D.Type] = ...
        kLine3D: ClassVar[Primitive3D.Type] = ...
        kPlane: ClassVar[Primitive3D.Type] = ...
        kRay3D: ClassVar[Primitive3D.Type] = ...
        kSegment3D: ClassVar[Primitive3D.Type] = ...
        kTriangle: ClassVar[Primitive3D.Type] = ...
        def __init__(self, value: int) -> None:
            """__init__(self: erl_geometry.pyerl_geometry.Primitive3D.Type, value: int) -> None"""
        def __eq__(self, other: object) -> bool:
            """__eq__(self: object, other: object) -> bool"""
        def __hash__(self) -> int:
            """__hash__(self: object) -> int"""
        def __index__(self) -> int:
            """__index__(self: erl_geometry.pyerl_geometry.Primitive3D.Type) -> int"""
        def __int__(self) -> int:
            """__int__(self: erl_geometry.pyerl_geometry.Primitive3D.Type) -> int"""
        def __ne__(self, other: object) -> bool:
            """__ne__(self: object, other: object) -> bool"""
        @property
        def name(self) -> str:
            """name(self: object) -> str

            name(self: object) -> str
            """
        @property
        def value(self) -> int:
            """(arg0: erl_geometry.pyerl_geometry.Primitive3D.Type) -> int"""
    id: int
    def __init__(self, *args, **kwargs) -> None:
        """Initialize self.  See help(type(self)) for accurate signature."""
    def is_inside(self, point: numpy.ndarray[numpy.float64[3, 1]]) -> bool:
        """is_inside(self: erl_geometry.pyerl_geometry.Primitive3D, point: numpy.ndarray[numpy.float64[3, 1]]) -> bool"""
    @property
    def type(self) -> Primitive3D.Type:
        """(arg0: erl_geometry.pyerl_geometry.Primitive3D) -> erl_geometry.pyerl_geometry.Primitive3D.Type"""

class PyObjectOccupancyOctreeD(AbstractOccupancyOctreeD):
    class BatchRayCaster:
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""
        def step(self, max_depth: numpy.ndarray[bool[m, 1]] = ...) -> PyObjectOccupancyOctreeD.BatchRayCaster:
            """step(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.BatchRayCaster, max_depth: numpy.ndarray[bool[m, 1]] = 0) -> erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.BatchRayCaster"""
        @property
        def ever_hit_flags(self) -> numpy.ndarray[bool[m, 1]]:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.BatchRayCaster) -> numpy.ndarray[bool[m, 1]]"""
        @property
        def frontier_keys(self) -> list[OctreeKey]:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.BatchRayCaster) -> list[erl_geometry.pyerl_geometry.OctreeKey]"""
        @property
        def frontier_nodes(self) -> list[PyObjectOccupancyOctreeNode]:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.BatchRayCaster) -> list[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode]"""
        @property
        def frontier_ray_indices(self) -> list[list[int]]:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.BatchRayCaster) -> list[list[int]]"""
        @property
        def hit_distances(self) -> numpy.ndarray[numpy.float64[m, 1]]:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.BatchRayCaster) -> numpy.ndarray[numpy.float64[m, 1]]"""
        @property
        def hit_flags(self) -> numpy.ndarray[bool[m, 1]]:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.BatchRayCaster) -> numpy.ndarray[bool[m, 1]]"""
        @property
        def hit_nodes(self) -> list[PyObjectOccupancyOctreeNode]:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.BatchRayCaster) -> list[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode]"""
        @property
        def hit_positions(self) -> list[numpy.ndarray[numpy.float64[3, 1]]]:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.BatchRayCaster) -> list[numpy.ndarray[numpy.float64[3, 1]]]"""
        @property
        def num_rays(self) -> int:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.BatchRayCaster) -> int"""
        @property
        def ray_directions(self) -> numpy.ndarray[numpy.float64[3, n]]:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.BatchRayCaster) -> numpy.ndarray[numpy.float64[3, n]]"""
        @property
        def ray_origins(self) -> numpy.ndarray[numpy.float64[3, n]]:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.BatchRayCaster) -> numpy.ndarray[numpy.float64[3, n]]"""

    class BottomLeafNeighborIterator(PyObjectOccupancyOctreeD.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class Drawer:
        def __init__(self, setting: OccupancyOctreeDrawerSetting, octree: PyObjectOccupancyOctreeD = ...) -> None:
            """__init__(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.Drawer, setting: erl_geometry.pyerl_geometry.OccupancyOctreeDrawerSetting, octree: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD = None) -> None"""
        @staticmethod
        def Setting() -> OccupancyOctreeDrawerSetting:
            """Setting() -> erl_geometry.pyerl_geometry.OccupancyOctreeDrawerSetting"""
        def draw_leaves(self, filename: str) -> None:
            """draw_leaves(*args, **kwargs)
            Overloaded function.

            1. draw_leaves(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.Drawer) -> list[open3d::geometry::Geometry]

            2. draw_leaves(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.Drawer, filename: str) -> None
            """
        def draw_tree(self, filename: str) -> None:
            """draw_tree(*args, **kwargs)
            Overloaded function.

            1. draw_tree(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.Drawer) -> list[open3d::geometry::Geometry]

            2. draw_tree(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.Drawer, filename: str) -> None
            """
        def set_draw_leaf_callback(self, callback) -> None:
            """set_draw_leaf_callback(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.Drawer, callback: Callable[[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.Drawer, list[open3d::geometry::Geometry], erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.LeafInAabbIterator], None]) -> None"""
        def set_draw_tree_callback(self, callback) -> None:
            """set_draw_tree_callback(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.Drawer, callback: Callable[[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.Drawer, list[open3d::geometry::Geometry], erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.TreeInAabbIterator], None]) -> None"""
        @property
        def setting(self) -> OccupancyOctreeDrawerSetting:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.Drawer) -> erl_geometry.pyerl_geometry.OccupancyOctreeDrawerSetting"""

    class EastLeafNeighborIterator(PyObjectOccupancyOctreeD.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class IteratorBase(AbstractOctreeD.OctreeNodeIterator):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""
        def __eq__(self, arg0: PyObjectOccupancyOctreeD.IteratorBase) -> bool:
            """__eq__(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.IteratorBase, arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.IteratorBase) -> bool"""
        def __ne__(self, arg0: PyObjectOccupancyOctreeD.IteratorBase) -> bool:
            """__ne__(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.IteratorBase, arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.IteratorBase) -> bool"""
        @property
        def index_key(self) -> OctreeKey:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.IteratorBase) -> erl_geometry.pyerl_geometry.OctreeKey"""
        @property
        def key(self) -> OctreeKey:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.IteratorBase) -> erl_geometry.pyerl_geometry.OctreeKey"""
        @property
        def node(self) -> PyObjectOccupancyOctreeNode:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.IteratorBase) -> erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode"""
        @property
        def node_aabb(self) -> Aabb3Dd:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.IteratorBase) -> erl_geometry.pyerl_geometry.Aabb3Dd"""

    class LeafInAabbIterator(PyObjectOccupancyOctreeD.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class LeafIterator(PyObjectOccupancyOctreeD.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class LeafOfNodeIterator(PyObjectOccupancyOctreeD.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class NodeOnRayIterator(PyObjectOccupancyOctreeD.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""
        @property
        def distance(self) -> float:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.NodeOnRayIterator) -> float"""

    class NorthLeafNeighborIterator(PyObjectOccupancyOctreeD.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class SouthLeafNeighborIterator(PyObjectOccupancyOctreeD.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class TopLeafNeighborIterator(PyObjectOccupancyOctreeD.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class TreeInAabbIterator(PyObjectOccupancyOctreeD.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class TreeIterator(PyObjectOccupancyOctreeD.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class WestLeafNeighborIterator(PyObjectOccupancyOctreeD.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""
    @overload
    def __init__(self) -> None:
        """__init__(*args, **kwargs)
        Overloaded function.

        1. __init__(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD) -> None

        2. __init__(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, setting: erl_geometry.pyerl_geometry.OccupancyOctreeBaseSetting) -> None

        3. __init__(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, filename: str) -> None
        """
    @overload
    def __init__(self, setting: OccupancyOctreeBaseSetting) -> None:
        """__init__(*args, **kwargs)
        Overloaded function.

        1. __init__(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD) -> None

        2. __init__(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, setting: erl_geometry.pyerl_geometry.OccupancyOctreeBaseSetting) -> None

        3. __init__(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, filename: str) -> None
        """
    @overload
    def __init__(self, filename: str) -> None:
        """__init__(*args, **kwargs)
        Overloaded function.

        1. __init__(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD) -> None

        2. __init__(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, setting: erl_geometry.pyerl_geometry.OccupancyOctreeBaseSetting) -> None

        3. __init__(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, filename: str) -> None
        """
    def Setting(self) -> OccupancyOctreeBaseSetting:
        """Setting() -> erl_geometry.pyerl_geometry.OccupancyOctreeBaseSetting"""
    @overload
    def adjust_key_to_depth(self, key: int, depth: int) -> int:
        """adjust_key_to_depth(*args, **kwargs)
        Overloaded function.

        1. adjust_key_to_depth(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, key: int, depth: int) -> int

        2. adjust_key_to_depth(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> erl_geometry.pyerl_geometry.OctreeKey
        """
    @overload
    def adjust_key_to_depth(self, key: OctreeKey, depth: int) -> OctreeKey:
        """adjust_key_to_depth(*args, **kwargs)
        Overloaded function.

        1. adjust_key_to_depth(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, key: int, depth: int) -> int

        2. adjust_key_to_depth(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> erl_geometry.pyerl_geometry.OctreeKey
        """
    def cast_ray(self, px: float, py: float, pz: float, vx: float, vy: float, vz: float, ignore_unknown: bool, max_range: float) -> dict:
        """cast_ray(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, px: float, py: float, pz: float, vx: float, vy: float, vz: float, ignore_unknown: bool, max_range: float) -> dict"""
    @overload
    def cast_rays(self, position: numpy.ndarray[numpy.float64[3, 1]], rotation: numpy.ndarray[numpy.float64[3, 3], flags.f_contiguous], azimuth_angles: numpy.ndarray[numpy.float64[m, 1]], elevation_angles: numpy.ndarray[numpy.float64[m, 1]], ignore_unknown: bool, max_range: float, prune_rays: bool, parallel: bool) -> dict:
        """cast_rays(*args, **kwargs)
        Overloaded function.

        1. cast_rays(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, position: numpy.ndarray[numpy.float64[3, 1]], rotation: numpy.ndarray[numpy.float64[3, 3], flags.f_contiguous], azimuth_angles: numpy.ndarray[numpy.float64[m, 1]], elevation_angles: numpy.ndarray[numpy.float64[m, 1]], ignore_unknown: bool, max_range: float, prune_rays: bool, parallel: bool) -> dict

        2. cast_rays(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, positions: numpy.ndarray[numpy.float64[3, n], flags.f_contiguous], directions: numpy.ndarray[numpy.float64[3, n], flags.f_contiguous], ignore_unknown: bool, max_range: float, prune_rays: bool, parallel: bool) -> dict
        """
    @overload
    def cast_rays(self, positions: numpy.ndarray[numpy.float64[3, n], flags.f_contiguous], directions: numpy.ndarray[numpy.float64[3, n], flags.f_contiguous], ignore_unknown: bool, max_range: float, prune_rays: bool, parallel: bool) -> dict:
        """cast_rays(*args, **kwargs)
        Overloaded function.

        1. cast_rays(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, position: numpy.ndarray[numpy.float64[3, 1]], rotation: numpy.ndarray[numpy.float64[3, 3], flags.f_contiguous], azimuth_angles: numpy.ndarray[numpy.float64[m, 1]], elevation_angles: numpy.ndarray[numpy.float64[m, 1]], ignore_unknown: bool, max_range: float, prune_rays: bool, parallel: bool) -> dict

        2. cast_rays(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, positions: numpy.ndarray[numpy.float64[3, n], flags.f_contiguous], directions: numpy.ndarray[numpy.float64[3, n], flags.f_contiguous], ignore_unknown: bool, max_range: float, prune_rays: bool, parallel: bool) -> dict
        """
    def clear(self) -> None:
        """clear(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD) -> None"""
    def compute_bottom_neighbor_key(self, key: OctreeKey, depth: int) -> OctreeKey | None:
        """compute_bottom_neighbor_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]"""
    def compute_common_ancestor_key(self, arg0: OctreeKey, arg1: OctreeKey) -> tuple[OctreeKey, int]:
        """compute_common_ancestor_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, arg0: erl_geometry.pyerl_geometry.OctreeKey, arg1: erl_geometry.pyerl_geometry.OctreeKey) -> tuple[erl_geometry.pyerl_geometry.OctreeKey, int]"""
    def compute_east_neighbor_key(self, key: OctreeKey, depth: int) -> OctreeKey | None:
        """compute_east_neighbor_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]"""
    def compute_north_neighbor_key(self, key: OctreeKey, depth: int) -> OctreeKey | None:
        """compute_north_neighbor_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]"""
    def compute_ray_coords(self, sx: float, sy: float, sz: float, ex: float, ey: float, ez: float) -> list[numpy.ndarray[numpy.float64[3, 1]]] | None:
        """compute_ray_coords(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, sx: float, sy: float, sz: float, ex: float, ey: float, ez: float) -> Optional[list[numpy.ndarray[numpy.float64[3, 1]]]]"""
    def compute_ray_keys(self, sx: float, sy: float, sz: float, ex: float, ey: float, ez: float) -> list[OctreeKey] | None:
        """compute_ray_keys(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, sx: float, sy: float, sz: float, ex: float, ey: float, ez: float) -> Optional[list[erl_geometry.pyerl_geometry.OctreeKey]]"""
    def compute_south_neighbor_key(self, key: OctreeKey, depth: int) -> OctreeKey | None:
        """compute_south_neighbor_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]"""
    def compute_top_neighbor_key(self, key: OctreeKey, depth: int) -> OctreeKey | None:
        """compute_top_neighbor_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]"""
    def compute_west_neighbor_key(self, key: OctreeKey, depth: int) -> OctreeKey | None:
        """compute_west_neighbor_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]"""
    @overload
    def coord_to_key(self, coordinate: float) -> int:
        """coord_to_key(*args, **kwargs)
        Overloaded function.

        1. coord_to_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, coordinate: float) -> int

        2. coord_to_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, coordinate: float, depth: int) -> int

        3. coord_to_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, x: float, y: float, z: float) -> erl_geometry.pyerl_geometry.OctreeKey

        4. coord_to_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, x: float, y: float, z: float, depth: int) -> erl_geometry.pyerl_geometry.OctreeKey
        """
    @overload
    def coord_to_key(self, coordinate: float, depth: int) -> int:
        """coord_to_key(*args, **kwargs)
        Overloaded function.

        1. coord_to_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, coordinate: float) -> int

        2. coord_to_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, coordinate: float, depth: int) -> int

        3. coord_to_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, x: float, y: float, z: float) -> erl_geometry.pyerl_geometry.OctreeKey

        4. coord_to_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, x: float, y: float, z: float, depth: int) -> erl_geometry.pyerl_geometry.OctreeKey
        """
    @overload
    def coord_to_key(self, x: float, y: float, z: float) -> OctreeKey:
        """coord_to_key(*args, **kwargs)
        Overloaded function.

        1. coord_to_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, coordinate: float) -> int

        2. coord_to_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, coordinate: float, depth: int) -> int

        3. coord_to_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, x: float, y: float, z: float) -> erl_geometry.pyerl_geometry.OctreeKey

        4. coord_to_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, x: float, y: float, z: float, depth: int) -> erl_geometry.pyerl_geometry.OctreeKey
        """
    @overload
    def coord_to_key(self, x: float, y: float, z: float, depth: int) -> OctreeKey:
        """coord_to_key(*args, **kwargs)
        Overloaded function.

        1. coord_to_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, coordinate: float) -> int

        2. coord_to_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, coordinate: float, depth: int) -> int

        3. coord_to_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, x: float, y: float, z: float) -> erl_geometry.pyerl_geometry.OctreeKey

        4. coord_to_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, x: float, y: float, z: float, depth: int) -> erl_geometry.pyerl_geometry.OctreeKey
        """
    @overload
    def coord_to_key_checked(self, coordinate: float) -> int | None:
        """coord_to_key_checked(*args, **kwargs)
        Overloaded function.

        1. coord_to_key_checked(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, coordinate: float) -> Optional[int]

        2. coord_to_key_checked(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, coordinate: float, depth: int) -> Optional[int]

        3. coord_to_key_checked(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, x: float, y: float, z: float) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]

        4. coord_to_key_checked(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, x: float, y: float, z: float, depth: int) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]
        """
    @overload
    def coord_to_key_checked(self, coordinate: float, depth: int) -> int | None:
        """coord_to_key_checked(*args, **kwargs)
        Overloaded function.

        1. coord_to_key_checked(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, coordinate: float) -> Optional[int]

        2. coord_to_key_checked(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, coordinate: float, depth: int) -> Optional[int]

        3. coord_to_key_checked(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, x: float, y: float, z: float) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]

        4. coord_to_key_checked(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, x: float, y: float, z: float, depth: int) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]
        """
    @overload
    def coord_to_key_checked(self, x: float, y: float, z: float) -> OctreeKey | None:
        """coord_to_key_checked(*args, **kwargs)
        Overloaded function.

        1. coord_to_key_checked(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, coordinate: float) -> Optional[int]

        2. coord_to_key_checked(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, coordinate: float, depth: int) -> Optional[int]

        3. coord_to_key_checked(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, x: float, y: float, z: float) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]

        4. coord_to_key_checked(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, x: float, y: float, z: float, depth: int) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]
        """
    @overload
    def coord_to_key_checked(self, x: float, y: float, z: float, depth: int) -> OctreeKey | None:
        """coord_to_key_checked(*args, **kwargs)
        Overloaded function.

        1. coord_to_key_checked(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, coordinate: float) -> Optional[int]

        2. coord_to_key_checked(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, coordinate: float, depth: int) -> Optional[int]

        3. coord_to_key_checked(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, x: float, y: float, z: float) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]

        4. coord_to_key_checked(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, x: float, y: float, z: float, depth: int) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]
        """
    def create_node_child(self, node: PyObjectOccupancyOctreeNode, child_idx: int) -> PyObjectOccupancyOctreeNode:
        """create_node_child(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, node: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode, child_idx: int) -> erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode"""
    @overload
    def delete_node(self, x: float, y: float, z: float, depth: int) -> int:
        """delete_node(*args, **kwargs)
        Overloaded function.

        1. delete_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, x: float, y: float, z: float, depth: int) -> int

        2. delete_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> None
        """
    @overload
    def delete_node(self, key: OctreeKey, depth: int) -> None:
        """delete_node(*args, **kwargs)
        Overloaded function.

        1. delete_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, x: float, y: float, z: float, depth: int) -> int

        2. delete_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> None
        """
    def delete_node_child(self, node: PyObjectOccupancyOctreeNode, child_idx: int, key: OctreeKey) -> int:
        """delete_node_child(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, node: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode, child_idx: int, key: erl_geometry.pyerl_geometry.OctreeKey) -> int"""
    def expand(self) -> None:
        """expand(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD) -> None"""
    def expand_node(self, node: PyObjectOccupancyOctreeNode) -> None:
        """expand_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, node: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode) -> None"""
    def get_batch_ray_caster(self, *args, **kwargs):
        """get_batch_ray_caster(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, origins: numpy.ndarray[numpy.float64[3, n]], directions: numpy.ndarray[numpy.float64[3, n]], max_ranges: numpy.ndarray[numpy.float64[m, 1]] = array([], dtype=float64), node_paddings: numpy.ndarray[numpy.float64[m, 1]] = array([], dtype=float64), bidirectional_flags: numpy.ndarray[bool[m, 1]] = array([], dtype=bool), leaf_only_flags: numpy.ndarray[bool[m, 1]] = array([], dtype=bool), min_node_depths: numpy.ndarray[numpy.int32[m, 1]] = array([], dtype=int32), max_node_depths: numpy.ndarray[numpy.int32[m, 1]] = array([], dtype=int32)) -> erl::geometry::OccupancyNdTreeBatchRayCaster<erl::geometry::OccupancyOctreeBase<double, erl::geometry::PyObjectOccupancyOctreeNode, erl::geometry::OccupancyOctreeBaseSetting>, 3>"""
    def get_node_child(self, node: PyObjectOccupancyOctreeNode, child_idx: int) -> PyObjectOccupancyOctreeNode:
        """get_node_child(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, node: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode, child_idx: int) -> erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode"""
    def get_node_size(self, depth: int) -> float:
        """get_node_size(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, depth: int) -> float"""
    @overload
    def insert_node(self, x: float, y: float, z: float, depth: int) -> PyObjectOccupancyOctreeNode:
        """insert_node(*args, **kwargs)
        Overloaded function.

        1. insert_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, x: float, y: float, z: float, depth: int) -> erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode

        2. insert_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode
        """
    @overload
    def insert_node(self, key: OctreeKey, depth: int) -> PyObjectOccupancyOctreeNode:
        """insert_node(*args, **kwargs)
        Overloaded function.

        1. insert_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, x: float, y: float, z: float, depth: int) -> erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode

        2. insert_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode
        """
    def insert_point_cloud(self, points: numpy.ndarray[numpy.float64[3, n], flags.f_contiguous], sensor_origin: numpy.ndarray[numpy.float64[3, 1]], min_range: float, max_range: float, with_count: bool, parallel: bool, lazy_eval: bool, discrete: bool) -> None:
        """insert_point_cloud(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, points: numpy.ndarray[numpy.float64[3, n], flags.f_contiguous], sensor_origin: numpy.ndarray[numpy.float64[3, 1]], min_range: float, max_range: float, with_count: bool, parallel: bool, lazy_eval: bool, discrete: bool) -> None"""
    def insert_point_cloud_rays(self, points: numpy.ndarray[numpy.float64[3, n], flags.f_contiguous], sensor_origin: numpy.ndarray[numpy.float64[3, 1]], min_range: float, max_range: float, parallel: bool, lazy_eval: bool) -> None:
        """insert_point_cloud_rays(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, points: numpy.ndarray[numpy.float64[3, n], flags.f_contiguous], sensor_origin: numpy.ndarray[numpy.float64[3, 1]], min_range: float, max_range: float, parallel: bool, lazy_eval: bool) -> None"""
    def insert_ray(self, sx: float, sy: float, sz: float, ex: float, ey: float, ez: float, min_range: float, max_range: float, lazy_eval: bool) -> bool:
        """insert_ray(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, sx: float, sy: float, sz: float, ex: float, ey: float, ez: float, min_range: float, max_range: float, lazy_eval: bool) -> bool"""
    def is_node_collapsible(self, node: PyObjectOccupancyOctreeNode) -> bool:
        """is_node_collapsible(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, node: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode) -> bool"""
    @overload
    def iter_bottom_leaf_neighbor(self, x: float, y: float, z: float, max_leaf_depth: int = ...) -> Iterator[PyObjectOccupancyOctreeD.BottomLeafNeighborIterator]:
        """iter_bottom_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_bottom_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.BottomLeafNeighborIterator]

        2. iter_bottom_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.BottomLeafNeighborIterator]
        """
    @overload
    def iter_bottom_leaf_neighbor(self, key: OctreeKey, key_depth: int, max_leaf_depth: int = ...) -> Iterator[PyObjectOccupancyOctreeD.BottomLeafNeighborIterator]:
        """iter_bottom_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_bottom_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.BottomLeafNeighborIterator]

        2. iter_bottom_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.BottomLeafNeighborIterator]
        """
    @overload
    def iter_east_leaf_neighbor(self, x: float, y: float, z: float, max_leaf_depth: int = ...) -> Iterator[PyObjectOccupancyOctreeD.EastLeafNeighborIterator]:
        """iter_east_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_east_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.EastLeafNeighborIterator]

        2. iter_east_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.EastLeafNeighborIterator]
        """
    @overload
    def iter_east_leaf_neighbor(self, key: OctreeKey, key_depth: int, max_leaf_depth: int = ...) -> Iterator[PyObjectOccupancyOctreeD.EastLeafNeighborIterator]:
        """iter_east_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_east_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.EastLeafNeighborIterator]

        2. iter_east_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.EastLeafNeighborIterator]
        """
    def iter_leaf(self, max_depth: int = ...) -> Iterator[PyObjectOccupancyOctreeD.LeafIterator]:
        """iter_leaf(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.LeafIterator]"""
    @overload
    def iter_leaf_in_aabb(self, aabb_min_x: float, aabb_min_y: float, aabb_min_z: float, aabb_max_x: float, aabb_max_y: float, aabb_max_z: float, max_depth: int = ...) -> Iterator[PyObjectOccupancyOctreeD.LeafInAabbIterator]:
        """iter_leaf_in_aabb(*args, **kwargs)
        Overloaded function.

        1. iter_leaf_in_aabb(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, aabb_min_x: float, aabb_min_y: float, aabb_min_z: float, aabb_max_x: float, aabb_max_y: float, aabb_max_z: float, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.LeafInAabbIterator]

        2. iter_leaf_in_aabb(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, aabb_min_key: erl_geometry.pyerl_geometry.OctreeKey, aabb_max_key: erl_geometry.pyerl_geometry.OctreeKey, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.LeafInAabbIterator]
        """
    @overload
    def iter_leaf_in_aabb(self, aabb_min_key: OctreeKey, aabb_max_key: OctreeKey, max_depth: int = ...) -> Iterator[PyObjectOccupancyOctreeD.LeafInAabbIterator]:
        """iter_leaf_in_aabb(*args, **kwargs)
        Overloaded function.

        1. iter_leaf_in_aabb(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, aabb_min_x: float, aabb_min_y: float, aabb_min_z: float, aabb_max_x: float, aabb_max_y: float, aabb_max_z: float, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.LeafInAabbIterator]

        2. iter_leaf_in_aabb(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, aabb_min_key: erl_geometry.pyerl_geometry.OctreeKey, aabb_max_key: erl_geometry.pyerl_geometry.OctreeKey, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.LeafInAabbIterator]
        """
    def iter_leaf_of_node(self, node_key: OctreeKey, node_depth: int, max_depth: int = ...) -> Iterator[PyObjectOccupancyOctreeD.LeafOfNodeIterator]:
        """iter_leaf_of_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, node_key: erl_geometry.pyerl_geometry.OctreeKey, node_depth: int, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.LeafOfNodeIterator]"""
    def iter_node(self, max_depth: int = ...) -> Iterator[PyObjectOccupancyOctreeD.TreeIterator]:
        """iter_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.TreeIterator]"""
    @overload
    def iter_node_in_aabb(self, aabb_min_x: float, aabb_min_y: float, aabb_min_z: float, aabb_max_x: float, aabb_max_y: float, aabb_max_z: float, max_depth: int = ...) -> Iterator[PyObjectOccupancyOctreeD.TreeInAabbIterator]:
        """iter_node_in_aabb(*args, **kwargs)
        Overloaded function.

        1. iter_node_in_aabb(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, aabb_min_x: float, aabb_min_y: float, aabb_min_z: float, aabb_max_x: float, aabb_max_y: float, aabb_max_z: float, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.TreeInAabbIterator]

        2. iter_node_in_aabb(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, aabb_min_key: erl_geometry.pyerl_geometry.OctreeKey, aabb_max_key: erl_geometry.pyerl_geometry.OctreeKey, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.TreeInAabbIterator]
        """
    @overload
    def iter_node_in_aabb(self, aabb_min_key: OctreeKey, aabb_max_key: OctreeKey, max_depth: int = ...) -> Iterator[PyObjectOccupancyOctreeD.TreeInAabbIterator]:
        """iter_node_in_aabb(*args, **kwargs)
        Overloaded function.

        1. iter_node_in_aabb(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, aabb_min_x: float, aabb_min_y: float, aabb_min_z: float, aabb_max_x: float, aabb_max_y: float, aabb_max_z: float, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.TreeInAabbIterator]

        2. iter_node_in_aabb(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, aabb_min_key: erl_geometry.pyerl_geometry.OctreeKey, aabb_max_key: erl_geometry.pyerl_geometry.OctreeKey, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.TreeInAabbIterator]
        """
    def iter_node_on_ray(self, px: float, py: float, pz: float, vx: float, vy: float, vz: float, max_range: float = ..., node_padding: float = ..., bidirectional: bool = ..., leaf_only: bool = ..., min_node_depth: int = ..., max_node_depth: int = ...) -> Iterator[PyObjectOccupancyOctreeD.NodeOnRayIterator]:
        """iter_node_on_ray(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, px: float, py: float, pz: float, vx: float, vy: float, vz: float, max_range: float = -1, node_padding: float = 0, bidirectional: bool = False, leaf_only: bool = True, min_node_depth: int = 0, max_node_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.NodeOnRayIterator]"""
    @overload
    def iter_north_leaf_neighbor(self, x: float, y: float, z: float, max_leaf_depth: int = ...) -> Iterator[PyObjectOccupancyOctreeD.NorthLeafNeighborIterator]:
        """iter_north_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_north_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.NorthLeafNeighborIterator]

        2. iter_north_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.NorthLeafNeighborIterator]
        """
    @overload
    def iter_north_leaf_neighbor(self, key: OctreeKey, key_depth: int, max_leaf_depth: int = ...) -> Iterator[PyObjectOccupancyOctreeD.NorthLeafNeighborIterator]:
        """iter_north_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_north_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.NorthLeafNeighborIterator]

        2. iter_north_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.NorthLeafNeighborIterator]
        """
    @overload
    def iter_south_leaf_neighbor(self, x: float, y: float, z: float, max_leaf_depth: int = ...) -> Iterator[PyObjectOccupancyOctreeD.SouthLeafNeighborIterator]:
        """iter_south_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_south_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.SouthLeafNeighborIterator]

        2. iter_south_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.SouthLeafNeighborIterator]
        """
    @overload
    def iter_south_leaf_neighbor(self, key: OctreeKey, key_depth: int, max_leaf_depth: int = ...) -> Iterator[PyObjectOccupancyOctreeD.SouthLeafNeighborIterator]:
        """iter_south_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_south_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.SouthLeafNeighborIterator]

        2. iter_south_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.SouthLeafNeighborIterator]
        """
    @overload
    def iter_top_leaf_neighbor(self, x: float, y: float, z: float, max_leaf_depth: int = ...) -> Iterator[PyObjectOccupancyOctreeD.TopLeafNeighborIterator]:
        """iter_top_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_top_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.TopLeafNeighborIterator]

        2. iter_top_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.TopLeafNeighborIterator]
        """
    @overload
    def iter_top_leaf_neighbor(self, key: OctreeKey, key_depth: int, max_leaf_depth: int = ...) -> Iterator[PyObjectOccupancyOctreeD.TopLeafNeighborIterator]:
        """iter_top_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_top_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.TopLeafNeighborIterator]

        2. iter_top_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.TopLeafNeighborIterator]
        """
    @overload
    def iter_west_leaf_neighbor(self, x: float, y: float, z: float, max_leaf_depth: int = ...) -> Iterator[PyObjectOccupancyOctreeD.WestLeafNeighborIterator]:
        """iter_west_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_west_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.WestLeafNeighborIterator]

        2. iter_west_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.WestLeafNeighborIterator]
        """
    @overload
    def iter_west_leaf_neighbor(self, key: OctreeKey, key_depth: int, max_leaf_depth: int = ...) -> Iterator[PyObjectOccupancyOctreeD.WestLeafNeighborIterator]:
        """iter_west_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_west_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.WestLeafNeighborIterator]

        2. iter_west_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD.WestLeafNeighborIterator]
        """
    @overload
    def key_to_coord(self, key: int) -> float:
        """key_to_coord(*args, **kwargs)
        Overloaded function.

        1. key_to_coord(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, key: int) -> float

        2. key_to_coord(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, key: int, depth: int) -> float

        3. key_to_coord(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey) -> tuple[float, float, float]

        4. key_to_coord(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> tuple[float, float, float]
        """
    @overload
    def key_to_coord(self, key: int, depth: int) -> float:
        """key_to_coord(*args, **kwargs)
        Overloaded function.

        1. key_to_coord(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, key: int) -> float

        2. key_to_coord(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, key: int, depth: int) -> float

        3. key_to_coord(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey) -> tuple[float, float, float]

        4. key_to_coord(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> tuple[float, float, float]
        """
    @overload
    def key_to_coord(self, key: OctreeKey) -> tuple[float, float, float]:
        """key_to_coord(*args, **kwargs)
        Overloaded function.

        1. key_to_coord(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, key: int) -> float

        2. key_to_coord(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, key: int, depth: int) -> float

        3. key_to_coord(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey) -> tuple[float, float, float]

        4. key_to_coord(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> tuple[float, float, float]
        """
    @overload
    def key_to_coord(self, key: OctreeKey, depth: int) -> tuple[float, float, float]:
        """key_to_coord(*args, **kwargs)
        Overloaded function.

        1. key_to_coord(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, key: int) -> float

        2. key_to_coord(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, key: int, depth: int) -> float

        3. key_to_coord(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey) -> tuple[float, float, float]

        4. key_to_coord(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> tuple[float, float, float]
        """
    def prune(self) -> None:
        """prune(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD) -> None"""
    def prune_node(self, node: PyObjectOccupancyOctreeNode) -> bool:
        """prune_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, node: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode) -> bool"""
    def sample_positions(self, num_positions: int) -> list[numpy.ndarray[numpy.float64[3, 1]]]:
        """sample_positions(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, num_positions: int) -> list[numpy.ndarray[numpy.float64[3, 1]]]"""
    @overload
    def search(self, x: float, y: float, z: float, max_depth: int = ...) -> PyObjectOccupancyOctreeNode:
        """search(*args, **kwargs)
        Overloaded function.

        1. search(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, x: float, y: float, z: float, max_depth: int = 0) -> erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode

        2. search(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, max_depth: int = 0) -> erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode
        """
    @overload
    def search(self, key: OctreeKey, max_depth: int = ...) -> PyObjectOccupancyOctreeNode:
        """search(*args, **kwargs)
        Overloaded function.

        1. search(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, x: float, y: float, z: float, max_depth: int = 0) -> erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode

        2. search(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, max_depth: int = 0) -> erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode
        """
    def to_max_likelihood(self) -> None:
        """to_max_likelihood(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD) -> None"""
    def update_inner_occupancy(self) -> None:
        """update_inner_occupancy(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD) -> None"""
    @overload
    def update_node(self, x: float, y: float, z: float, occupied: bool, lazy_eval: bool) -> PyObjectOccupancyOctreeNode:
        """update_node(*args, **kwargs)
        Overloaded function.

        1. update_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, x: float, y: float, z: float, occupied: bool, lazy_eval: bool) -> erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode

        2. update_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, node_key: erl_geometry.pyerl_geometry.OctreeKey, occupied: bool, lazy_eval: bool) -> erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode

        3. update_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, x: float, y: float, z: float, log_odds_delta: float, lazy_eval: bool) -> erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode

        4. update_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, node_key: erl_geometry.pyerl_geometry.OctreeKey, log_odds_delta: float, lazy_eval: bool) -> erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode
        """
    @overload
    def update_node(self, node_key: OctreeKey, occupied: bool, lazy_eval: bool) -> PyObjectOccupancyOctreeNode:
        """update_node(*args, **kwargs)
        Overloaded function.

        1. update_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, x: float, y: float, z: float, occupied: bool, lazy_eval: bool) -> erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode

        2. update_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, node_key: erl_geometry.pyerl_geometry.OctreeKey, occupied: bool, lazy_eval: bool) -> erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode

        3. update_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, x: float, y: float, z: float, log_odds_delta: float, lazy_eval: bool) -> erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode

        4. update_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, node_key: erl_geometry.pyerl_geometry.OctreeKey, log_odds_delta: float, lazy_eval: bool) -> erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode
        """
    @overload
    def update_node(self, x: float, y: float, z: float, log_odds_delta: float, lazy_eval: bool) -> PyObjectOccupancyOctreeNode:
        """update_node(*args, **kwargs)
        Overloaded function.

        1. update_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, x: float, y: float, z: float, occupied: bool, lazy_eval: bool) -> erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode

        2. update_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, node_key: erl_geometry.pyerl_geometry.OctreeKey, occupied: bool, lazy_eval: bool) -> erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode

        3. update_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, x: float, y: float, z: float, log_odds_delta: float, lazy_eval: bool) -> erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode

        4. update_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, node_key: erl_geometry.pyerl_geometry.OctreeKey, log_odds_delta: float, lazy_eval: bool) -> erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode
        """
    @overload
    def update_node(self, node_key: OctreeKey, log_odds_delta: float, lazy_eval: bool) -> PyObjectOccupancyOctreeNode:
        """update_node(*args, **kwargs)
        Overloaded function.

        1. update_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, x: float, y: float, z: float, occupied: bool, lazy_eval: bool) -> erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode

        2. update_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, node_key: erl_geometry.pyerl_geometry.OctreeKey, occupied: bool, lazy_eval: bool) -> erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode

        3. update_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, x: float, y: float, z: float, log_odds_delta: float, lazy_eval: bool) -> erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode

        4. update_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, node_key: erl_geometry.pyerl_geometry.OctreeKey, log_odds_delta: float, lazy_eval: bool) -> erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode
        """
    def visualize(self, leaf_only: bool = ..., scaling: float = ..., area_min: numpy.ndarray[numpy.float64[3, 1]] = ..., area_max: numpy.ndarray[numpy.float64[3, 1]] = ..., border_color: numpy.ndarray[numpy.float64[3, 1]] = ..., occupied_color: numpy.ndarray[numpy.float64[3, 1]] = ..., occupied_only: bool = ..., draw_node_boxes: bool = ..., draw_node_borders: bool = ..., window_width: int = ..., window_height: int = ..., window_left: int = ..., window_top: int = ...) -> None:
        """visualize(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD, leaf_only: bool = False, scaling: float = 1.0, area_min: numpy.ndarray[numpy.float64[3, 1]] = array([-1., -1., -1.]), area_max: numpy.ndarray[numpy.float64[3, 1]] = array([1., 1., 1.]), border_color: numpy.ndarray[numpy.float64[3, 1]] = array([0., 0., 0.]), occupied_color: numpy.ndarray[numpy.float64[3, 1]] = array([0.5, 0.5, 0.5]), occupied_only: bool = False, draw_node_boxes: bool = True, draw_node_borders: bool = True, window_width: int = 1920, window_height: int = 1080, window_left: int = 50, window_top: int = 50) -> None"""
    @property
    def memory_usage(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD) -> int"""
    @property
    def memory_usage_per_node(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD) -> int"""
    @property
    def metric_aabb(self) -> Aabb3Dd:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD) -> erl_geometry.pyerl_geometry.Aabb3Dd"""
    @property
    def metric_max(self) -> numpy.ndarray[numpy.float64[3, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD) -> numpy.ndarray[numpy.float64[3, 1]]"""
    @property
    def metric_min(self) -> numpy.ndarray[numpy.float64[3, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD) -> numpy.ndarray[numpy.float64[3, 1]]"""
    @property
    def metric_min_max(self) -> tuple[numpy.ndarray[numpy.float64[3, 1]], numpy.ndarray[numpy.float64[3, 1]]]:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD) -> tuple[numpy.ndarray[numpy.float64[3, 1]], numpy.ndarray[numpy.float64[3, 1]]]"""
    @property
    def metric_size(self) -> numpy.ndarray[numpy.float64[3, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD) -> numpy.ndarray[numpy.float64[3, 1]]"""
    @property
    def number_of_leaf_nodes(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD) -> int"""
    @property
    def number_of_nodes(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD) -> int"""
    @property
    def resolution(self) -> float:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD) -> float"""
    @property
    def root(self) -> PyObjectOccupancyOctreeNode:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD) -> erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode"""
    @property
    def setting(self) -> OccupancyOctreeBaseSetting:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD) -> erl_geometry.pyerl_geometry.OccupancyOctreeBaseSetting"""
    @property
    def tree_center(self) -> numpy.ndarray[numpy.float64[3, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD) -> numpy.ndarray[numpy.float64[3, 1]]"""
    @property
    def tree_center_key(self) -> OctreeKey:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD) -> erl_geometry.pyerl_geometry.OctreeKey"""
    @property
    def tree_depth(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD) -> int"""
    @property
    def tree_max_half_size(self) -> numpy.ndarray[numpy.float64[3, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeD) -> numpy.ndarray[numpy.float64[3, 1]]"""

class PyObjectOccupancyOctreeF(AbstractOccupancyOctreeF):
    class BatchRayCaster:
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""
        def step(self, max_depth: numpy.ndarray[bool[m, 1]] = ...) -> PyObjectOccupancyOctreeF.BatchRayCaster:
            """step(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.BatchRayCaster, max_depth: numpy.ndarray[bool[m, 1]] = 0) -> erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.BatchRayCaster"""
        @property
        def ever_hit_flags(self) -> numpy.ndarray[bool[m, 1]]:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.BatchRayCaster) -> numpy.ndarray[bool[m, 1]]"""
        @property
        def frontier_keys(self) -> list[OctreeKey]:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.BatchRayCaster) -> list[erl_geometry.pyerl_geometry.OctreeKey]"""
        @property
        def frontier_nodes(self) -> list[PyObjectOccupancyOctreeNode]:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.BatchRayCaster) -> list[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode]"""
        @property
        def frontier_ray_indices(self) -> list[list[int]]:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.BatchRayCaster) -> list[list[int]]"""
        @property
        def hit_distances(self) -> numpy.ndarray[numpy.float32[m, 1]]:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.BatchRayCaster) -> numpy.ndarray[numpy.float32[m, 1]]"""
        @property
        def hit_flags(self) -> numpy.ndarray[bool[m, 1]]:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.BatchRayCaster) -> numpy.ndarray[bool[m, 1]]"""
        @property
        def hit_nodes(self) -> list[PyObjectOccupancyOctreeNode]:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.BatchRayCaster) -> list[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode]"""
        @property
        def hit_positions(self) -> list[numpy.ndarray[numpy.float32[3, 1]]]:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.BatchRayCaster) -> list[numpy.ndarray[numpy.float32[3, 1]]]"""
        @property
        def num_rays(self) -> int:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.BatchRayCaster) -> int"""
        @property
        def ray_directions(self) -> numpy.ndarray[numpy.float32[3, n]]:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.BatchRayCaster) -> numpy.ndarray[numpy.float32[3, n]]"""
        @property
        def ray_origins(self) -> numpy.ndarray[numpy.float32[3, n]]:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.BatchRayCaster) -> numpy.ndarray[numpy.float32[3, n]]"""

    class BottomLeafNeighborIterator(PyObjectOccupancyOctreeF.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class Drawer:
        def __init__(self, setting: OccupancyOctreeDrawerSetting, octree: PyObjectOccupancyOctreeF = ...) -> None:
            """__init__(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.Drawer, setting: erl_geometry.pyerl_geometry.OccupancyOctreeDrawerSetting, octree: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF = None) -> None"""
        @staticmethod
        def Setting() -> OccupancyOctreeDrawerSetting:
            """Setting() -> erl_geometry.pyerl_geometry.OccupancyOctreeDrawerSetting"""
        def draw_leaves(self, filename: str) -> None:
            """draw_leaves(*args, **kwargs)
            Overloaded function.

            1. draw_leaves(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.Drawer) -> list[open3d::geometry::Geometry]

            2. draw_leaves(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.Drawer, filename: str) -> None
            """
        def draw_tree(self, filename: str) -> None:
            """draw_tree(*args, **kwargs)
            Overloaded function.

            1. draw_tree(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.Drawer) -> list[open3d::geometry::Geometry]

            2. draw_tree(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.Drawer, filename: str) -> None
            """
        def set_draw_leaf_callback(self, callback) -> None:
            """set_draw_leaf_callback(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.Drawer, callback: Callable[[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.Drawer, list[open3d::geometry::Geometry], erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.LeafInAabbIterator], None]) -> None"""
        def set_draw_tree_callback(self, callback) -> None:
            """set_draw_tree_callback(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.Drawer, callback: Callable[[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.Drawer, list[open3d::geometry::Geometry], erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.TreeInAabbIterator], None]) -> None"""
        @property
        def setting(self) -> OccupancyOctreeDrawerSetting:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.Drawer) -> erl_geometry.pyerl_geometry.OccupancyOctreeDrawerSetting"""

    class EastLeafNeighborIterator(PyObjectOccupancyOctreeF.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class IteratorBase(AbstractOctreeF.OctreeNodeIterator):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""
        def __eq__(self, arg0: PyObjectOccupancyOctreeF.IteratorBase) -> bool:
            """__eq__(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.IteratorBase, arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.IteratorBase) -> bool"""
        def __ne__(self, arg0: PyObjectOccupancyOctreeF.IteratorBase) -> bool:
            """__ne__(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.IteratorBase, arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.IteratorBase) -> bool"""
        @property
        def index_key(self) -> OctreeKey:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.IteratorBase) -> erl_geometry.pyerl_geometry.OctreeKey"""
        @property
        def key(self) -> OctreeKey:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.IteratorBase) -> erl_geometry.pyerl_geometry.OctreeKey"""
        @property
        def node(self) -> PyObjectOccupancyOctreeNode:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.IteratorBase) -> erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode"""
        @property
        def node_aabb(self) -> Aabb3Df:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.IteratorBase) -> erl_geometry.pyerl_geometry.Aabb3Df"""

    class LeafInAabbIterator(PyObjectOccupancyOctreeF.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class LeafIterator(PyObjectOccupancyOctreeF.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class LeafOfNodeIterator(PyObjectOccupancyOctreeF.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class NodeOnRayIterator(PyObjectOccupancyOctreeF.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""
        @property
        def distance(self) -> float:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.NodeOnRayIterator) -> float"""

    class NorthLeafNeighborIterator(PyObjectOccupancyOctreeF.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class SouthLeafNeighborIterator(PyObjectOccupancyOctreeF.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class TopLeafNeighborIterator(PyObjectOccupancyOctreeF.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class TreeInAabbIterator(PyObjectOccupancyOctreeF.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class TreeIterator(PyObjectOccupancyOctreeF.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class WestLeafNeighborIterator(PyObjectOccupancyOctreeF.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""
    @overload
    def __init__(self) -> None:
        """__init__(*args, **kwargs)
        Overloaded function.

        1. __init__(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF) -> None

        2. __init__(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, setting: erl_geometry.pyerl_geometry.OccupancyOctreeBaseSetting) -> None

        3. __init__(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, filename: str) -> None
        """
    @overload
    def __init__(self, setting: OccupancyOctreeBaseSetting) -> None:
        """__init__(*args, **kwargs)
        Overloaded function.

        1. __init__(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF) -> None

        2. __init__(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, setting: erl_geometry.pyerl_geometry.OccupancyOctreeBaseSetting) -> None

        3. __init__(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, filename: str) -> None
        """
    @overload
    def __init__(self, filename: str) -> None:
        """__init__(*args, **kwargs)
        Overloaded function.

        1. __init__(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF) -> None

        2. __init__(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, setting: erl_geometry.pyerl_geometry.OccupancyOctreeBaseSetting) -> None

        3. __init__(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, filename: str) -> None
        """
    def Setting(self) -> OccupancyOctreeBaseSetting:
        """Setting() -> erl_geometry.pyerl_geometry.OccupancyOctreeBaseSetting"""
    @overload
    def adjust_key_to_depth(self, key: int, depth: int) -> int:
        """adjust_key_to_depth(*args, **kwargs)
        Overloaded function.

        1. adjust_key_to_depth(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, key: int, depth: int) -> int

        2. adjust_key_to_depth(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> erl_geometry.pyerl_geometry.OctreeKey
        """
    @overload
    def adjust_key_to_depth(self, key: OctreeKey, depth: int) -> OctreeKey:
        """adjust_key_to_depth(*args, **kwargs)
        Overloaded function.

        1. adjust_key_to_depth(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, key: int, depth: int) -> int

        2. adjust_key_to_depth(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> erl_geometry.pyerl_geometry.OctreeKey
        """
    def cast_ray(self, px: float, py: float, pz: float, vx: float, vy: float, vz: float, ignore_unknown: bool, max_range: float) -> dict:
        """cast_ray(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, px: float, py: float, pz: float, vx: float, vy: float, vz: float, ignore_unknown: bool, max_range: float) -> dict"""
    @overload
    def cast_rays(self, position: numpy.ndarray[numpy.float32[3, 1]], rotation: numpy.ndarray[numpy.float32[3, 3], flags.f_contiguous], azimuth_angles: numpy.ndarray[numpy.float32[m, 1]], elevation_angles: numpy.ndarray[numpy.float32[m, 1]], ignore_unknown: bool, max_range: float, prune_rays: bool, parallel: bool) -> dict:
        """cast_rays(*args, **kwargs)
        Overloaded function.

        1. cast_rays(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, position: numpy.ndarray[numpy.float32[3, 1]], rotation: numpy.ndarray[numpy.float32[3, 3], flags.f_contiguous], azimuth_angles: numpy.ndarray[numpy.float32[m, 1]], elevation_angles: numpy.ndarray[numpy.float32[m, 1]], ignore_unknown: bool, max_range: float, prune_rays: bool, parallel: bool) -> dict

        2. cast_rays(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, positions: numpy.ndarray[numpy.float32[3, n], flags.f_contiguous], directions: numpy.ndarray[numpy.float32[3, n], flags.f_contiguous], ignore_unknown: bool, max_range: float, prune_rays: bool, parallel: bool) -> dict
        """
    @overload
    def cast_rays(self, positions: numpy.ndarray[numpy.float32[3, n], flags.f_contiguous], directions: numpy.ndarray[numpy.float32[3, n], flags.f_contiguous], ignore_unknown: bool, max_range: float, prune_rays: bool, parallel: bool) -> dict:
        """cast_rays(*args, **kwargs)
        Overloaded function.

        1. cast_rays(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, position: numpy.ndarray[numpy.float32[3, 1]], rotation: numpy.ndarray[numpy.float32[3, 3], flags.f_contiguous], azimuth_angles: numpy.ndarray[numpy.float32[m, 1]], elevation_angles: numpy.ndarray[numpy.float32[m, 1]], ignore_unknown: bool, max_range: float, prune_rays: bool, parallel: bool) -> dict

        2. cast_rays(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, positions: numpy.ndarray[numpy.float32[3, n], flags.f_contiguous], directions: numpy.ndarray[numpy.float32[3, n], flags.f_contiguous], ignore_unknown: bool, max_range: float, prune_rays: bool, parallel: bool) -> dict
        """
    def clear(self) -> None:
        """clear(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF) -> None"""
    def compute_bottom_neighbor_key(self, key: OctreeKey, depth: int) -> OctreeKey | None:
        """compute_bottom_neighbor_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]"""
    def compute_common_ancestor_key(self, arg0: OctreeKey, arg1: OctreeKey) -> tuple[OctreeKey, int]:
        """compute_common_ancestor_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, arg0: erl_geometry.pyerl_geometry.OctreeKey, arg1: erl_geometry.pyerl_geometry.OctreeKey) -> tuple[erl_geometry.pyerl_geometry.OctreeKey, int]"""
    def compute_east_neighbor_key(self, key: OctreeKey, depth: int) -> OctreeKey | None:
        """compute_east_neighbor_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]"""
    def compute_north_neighbor_key(self, key: OctreeKey, depth: int) -> OctreeKey | None:
        """compute_north_neighbor_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]"""
    def compute_ray_coords(self, sx: float, sy: float, sz: float, ex: float, ey: float, ez: float) -> list[numpy.ndarray[numpy.float32[3, 1]]] | None:
        """compute_ray_coords(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, sx: float, sy: float, sz: float, ex: float, ey: float, ez: float) -> Optional[list[numpy.ndarray[numpy.float32[3, 1]]]]"""
    def compute_ray_keys(self, sx: float, sy: float, sz: float, ex: float, ey: float, ez: float) -> list[OctreeKey] | None:
        """compute_ray_keys(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, sx: float, sy: float, sz: float, ex: float, ey: float, ez: float) -> Optional[list[erl_geometry.pyerl_geometry.OctreeKey]]"""
    def compute_south_neighbor_key(self, key: OctreeKey, depth: int) -> OctreeKey | None:
        """compute_south_neighbor_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]"""
    def compute_top_neighbor_key(self, key: OctreeKey, depth: int) -> OctreeKey | None:
        """compute_top_neighbor_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]"""
    def compute_west_neighbor_key(self, key: OctreeKey, depth: int) -> OctreeKey | None:
        """compute_west_neighbor_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]"""
    @overload
    def coord_to_key(self, coordinate: float) -> int:
        """coord_to_key(*args, **kwargs)
        Overloaded function.

        1. coord_to_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, coordinate: float) -> int

        2. coord_to_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, coordinate: float, depth: int) -> int

        3. coord_to_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, x: float, y: float, z: float) -> erl_geometry.pyerl_geometry.OctreeKey

        4. coord_to_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, x: float, y: float, z: float, depth: int) -> erl_geometry.pyerl_geometry.OctreeKey
        """
    @overload
    def coord_to_key(self, coordinate: float, depth: int) -> int:
        """coord_to_key(*args, **kwargs)
        Overloaded function.

        1. coord_to_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, coordinate: float) -> int

        2. coord_to_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, coordinate: float, depth: int) -> int

        3. coord_to_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, x: float, y: float, z: float) -> erl_geometry.pyerl_geometry.OctreeKey

        4. coord_to_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, x: float, y: float, z: float, depth: int) -> erl_geometry.pyerl_geometry.OctreeKey
        """
    @overload
    def coord_to_key(self, x: float, y: float, z: float) -> OctreeKey:
        """coord_to_key(*args, **kwargs)
        Overloaded function.

        1. coord_to_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, coordinate: float) -> int

        2. coord_to_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, coordinate: float, depth: int) -> int

        3. coord_to_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, x: float, y: float, z: float) -> erl_geometry.pyerl_geometry.OctreeKey

        4. coord_to_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, x: float, y: float, z: float, depth: int) -> erl_geometry.pyerl_geometry.OctreeKey
        """
    @overload
    def coord_to_key(self, x: float, y: float, z: float, depth: int) -> OctreeKey:
        """coord_to_key(*args, **kwargs)
        Overloaded function.

        1. coord_to_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, coordinate: float) -> int

        2. coord_to_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, coordinate: float, depth: int) -> int

        3. coord_to_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, x: float, y: float, z: float) -> erl_geometry.pyerl_geometry.OctreeKey

        4. coord_to_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, x: float, y: float, z: float, depth: int) -> erl_geometry.pyerl_geometry.OctreeKey
        """
    @overload
    def coord_to_key_checked(self, coordinate: float) -> int | None:
        """coord_to_key_checked(*args, **kwargs)
        Overloaded function.

        1. coord_to_key_checked(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, coordinate: float) -> Optional[int]

        2. coord_to_key_checked(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, coordinate: float, depth: int) -> Optional[int]

        3. coord_to_key_checked(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, x: float, y: float, z: float) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]

        4. coord_to_key_checked(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, x: float, y: float, z: float, depth: int) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]
        """
    @overload
    def coord_to_key_checked(self, coordinate: float, depth: int) -> int | None:
        """coord_to_key_checked(*args, **kwargs)
        Overloaded function.

        1. coord_to_key_checked(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, coordinate: float) -> Optional[int]

        2. coord_to_key_checked(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, coordinate: float, depth: int) -> Optional[int]

        3. coord_to_key_checked(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, x: float, y: float, z: float) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]

        4. coord_to_key_checked(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, x: float, y: float, z: float, depth: int) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]
        """
    @overload
    def coord_to_key_checked(self, x: float, y: float, z: float) -> OctreeKey | None:
        """coord_to_key_checked(*args, **kwargs)
        Overloaded function.

        1. coord_to_key_checked(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, coordinate: float) -> Optional[int]

        2. coord_to_key_checked(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, coordinate: float, depth: int) -> Optional[int]

        3. coord_to_key_checked(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, x: float, y: float, z: float) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]

        4. coord_to_key_checked(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, x: float, y: float, z: float, depth: int) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]
        """
    @overload
    def coord_to_key_checked(self, x: float, y: float, z: float, depth: int) -> OctreeKey | None:
        """coord_to_key_checked(*args, **kwargs)
        Overloaded function.

        1. coord_to_key_checked(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, coordinate: float) -> Optional[int]

        2. coord_to_key_checked(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, coordinate: float, depth: int) -> Optional[int]

        3. coord_to_key_checked(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, x: float, y: float, z: float) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]

        4. coord_to_key_checked(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, x: float, y: float, z: float, depth: int) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]
        """
    def create_node_child(self, node: PyObjectOccupancyOctreeNode, child_idx: int) -> PyObjectOccupancyOctreeNode:
        """create_node_child(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, node: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode, child_idx: int) -> erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode"""
    @overload
    def delete_node(self, x: float, y: float, z: float, depth: int) -> int:
        """delete_node(*args, **kwargs)
        Overloaded function.

        1. delete_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, x: float, y: float, z: float, depth: int) -> int

        2. delete_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> None
        """
    @overload
    def delete_node(self, key: OctreeKey, depth: int) -> None:
        """delete_node(*args, **kwargs)
        Overloaded function.

        1. delete_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, x: float, y: float, z: float, depth: int) -> int

        2. delete_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> None
        """
    def delete_node_child(self, node: PyObjectOccupancyOctreeNode, child_idx: int, key: OctreeKey) -> int:
        """delete_node_child(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, node: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode, child_idx: int, key: erl_geometry.pyerl_geometry.OctreeKey) -> int"""
    def expand(self) -> None:
        """expand(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF) -> None"""
    def expand_node(self, node: PyObjectOccupancyOctreeNode) -> None:
        """expand_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, node: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode) -> None"""
    def get_batch_ray_caster(self, *args, **kwargs):
        """get_batch_ray_caster(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, origins: numpy.ndarray[numpy.float32[3, n]], directions: numpy.ndarray[numpy.float32[3, n]], max_ranges: numpy.ndarray[numpy.float32[m, 1]] = array([], dtype=float32), node_paddings: numpy.ndarray[numpy.float32[m, 1]] = array([], dtype=float32), bidirectional_flags: numpy.ndarray[bool[m, 1]] = array([], dtype=bool), leaf_only_flags: numpy.ndarray[bool[m, 1]] = array([], dtype=bool), min_node_depths: numpy.ndarray[numpy.int32[m, 1]] = array([], dtype=int32), max_node_depths: numpy.ndarray[numpy.int32[m, 1]] = array([], dtype=int32)) -> erl::geometry::OccupancyNdTreeBatchRayCaster<erl::geometry::OccupancyOctreeBase<float, erl::geometry::PyObjectOccupancyOctreeNode, erl::geometry::OccupancyOctreeBaseSetting>, 3>"""
    def get_node_child(self, node: PyObjectOccupancyOctreeNode, child_idx: int) -> PyObjectOccupancyOctreeNode:
        """get_node_child(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, node: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode, child_idx: int) -> erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode"""
    def get_node_size(self, depth: int) -> float:
        """get_node_size(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, depth: int) -> float"""
    @overload
    def insert_node(self, x: float, y: float, z: float, depth: int) -> PyObjectOccupancyOctreeNode:
        """insert_node(*args, **kwargs)
        Overloaded function.

        1. insert_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, x: float, y: float, z: float, depth: int) -> erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode

        2. insert_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode
        """
    @overload
    def insert_node(self, key: OctreeKey, depth: int) -> PyObjectOccupancyOctreeNode:
        """insert_node(*args, **kwargs)
        Overloaded function.

        1. insert_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, x: float, y: float, z: float, depth: int) -> erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode

        2. insert_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode
        """
    def insert_point_cloud(self, points: numpy.ndarray[numpy.float32[3, n], flags.f_contiguous], sensor_origin: numpy.ndarray[numpy.float32[3, 1]], min_range: float, max_range: float, with_count: bool, parallel: bool, lazy_eval: bool, discrete: bool) -> None:
        """insert_point_cloud(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, points: numpy.ndarray[numpy.float32[3, n], flags.f_contiguous], sensor_origin: numpy.ndarray[numpy.float32[3, 1]], min_range: float, max_range: float, with_count: bool, parallel: bool, lazy_eval: bool, discrete: bool) -> None"""
    def insert_point_cloud_rays(self, points: numpy.ndarray[numpy.float32[3, n], flags.f_contiguous], sensor_origin: numpy.ndarray[numpy.float32[3, 1]], min_range: float, max_range: float, parallel: bool, lazy_eval: bool) -> None:
        """insert_point_cloud_rays(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, points: numpy.ndarray[numpy.float32[3, n], flags.f_contiguous], sensor_origin: numpy.ndarray[numpy.float32[3, 1]], min_range: float, max_range: float, parallel: bool, lazy_eval: bool) -> None"""
    def insert_ray(self, sx: float, sy: float, sz: float, ex: float, ey: float, ez: float, min_range: float, max_range: float, lazy_eval: bool) -> bool:
        """insert_ray(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, sx: float, sy: float, sz: float, ex: float, ey: float, ez: float, min_range: float, max_range: float, lazy_eval: bool) -> bool"""
    def is_node_collapsible(self, node: PyObjectOccupancyOctreeNode) -> bool:
        """is_node_collapsible(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, node: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode) -> bool"""
    @overload
    def iter_bottom_leaf_neighbor(self, x: float, y: float, z: float, max_leaf_depth: int = ...) -> Iterator[PyObjectOccupancyOctreeF.BottomLeafNeighborIterator]:
        """iter_bottom_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_bottom_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.BottomLeafNeighborIterator]

        2. iter_bottom_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.BottomLeafNeighborIterator]
        """
    @overload
    def iter_bottom_leaf_neighbor(self, key: OctreeKey, key_depth: int, max_leaf_depth: int = ...) -> Iterator[PyObjectOccupancyOctreeF.BottomLeafNeighborIterator]:
        """iter_bottom_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_bottom_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.BottomLeafNeighborIterator]

        2. iter_bottom_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.BottomLeafNeighborIterator]
        """
    @overload
    def iter_east_leaf_neighbor(self, x: float, y: float, z: float, max_leaf_depth: int = ...) -> Iterator[PyObjectOccupancyOctreeF.EastLeafNeighborIterator]:
        """iter_east_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_east_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.EastLeafNeighborIterator]

        2. iter_east_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.EastLeafNeighborIterator]
        """
    @overload
    def iter_east_leaf_neighbor(self, key: OctreeKey, key_depth: int, max_leaf_depth: int = ...) -> Iterator[PyObjectOccupancyOctreeF.EastLeafNeighborIterator]:
        """iter_east_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_east_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.EastLeafNeighborIterator]

        2. iter_east_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.EastLeafNeighborIterator]
        """
    def iter_leaf(self, max_depth: int = ...) -> Iterator[PyObjectOccupancyOctreeF.LeafIterator]:
        """iter_leaf(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.LeafIterator]"""
    @overload
    def iter_leaf_in_aabb(self, aabb_min_x: float, aabb_min_y: float, aabb_min_z: float, aabb_max_x: float, aabb_max_y: float, aabb_max_z: float, max_depth: int = ...) -> Iterator[PyObjectOccupancyOctreeF.LeafInAabbIterator]:
        """iter_leaf_in_aabb(*args, **kwargs)
        Overloaded function.

        1. iter_leaf_in_aabb(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, aabb_min_x: float, aabb_min_y: float, aabb_min_z: float, aabb_max_x: float, aabb_max_y: float, aabb_max_z: float, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.LeafInAabbIterator]

        2. iter_leaf_in_aabb(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, aabb_min_key: erl_geometry.pyerl_geometry.OctreeKey, aabb_max_key: erl_geometry.pyerl_geometry.OctreeKey, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.LeafInAabbIterator]
        """
    @overload
    def iter_leaf_in_aabb(self, aabb_min_key: OctreeKey, aabb_max_key: OctreeKey, max_depth: int = ...) -> Iterator[PyObjectOccupancyOctreeF.LeafInAabbIterator]:
        """iter_leaf_in_aabb(*args, **kwargs)
        Overloaded function.

        1. iter_leaf_in_aabb(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, aabb_min_x: float, aabb_min_y: float, aabb_min_z: float, aabb_max_x: float, aabb_max_y: float, aabb_max_z: float, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.LeafInAabbIterator]

        2. iter_leaf_in_aabb(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, aabb_min_key: erl_geometry.pyerl_geometry.OctreeKey, aabb_max_key: erl_geometry.pyerl_geometry.OctreeKey, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.LeafInAabbIterator]
        """
    def iter_leaf_of_node(self, node_key: OctreeKey, node_depth: int, max_depth: int = ...) -> Iterator[PyObjectOccupancyOctreeF.LeafOfNodeIterator]:
        """iter_leaf_of_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, node_key: erl_geometry.pyerl_geometry.OctreeKey, node_depth: int, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.LeafOfNodeIterator]"""
    def iter_node(self, max_depth: int = ...) -> Iterator[PyObjectOccupancyOctreeF.TreeIterator]:
        """iter_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.TreeIterator]"""
    @overload
    def iter_node_in_aabb(self, aabb_min_x: float, aabb_min_y: float, aabb_min_z: float, aabb_max_x: float, aabb_max_y: float, aabb_max_z: float, max_depth: int = ...) -> Iterator[PyObjectOccupancyOctreeF.TreeInAabbIterator]:
        """iter_node_in_aabb(*args, **kwargs)
        Overloaded function.

        1. iter_node_in_aabb(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, aabb_min_x: float, aabb_min_y: float, aabb_min_z: float, aabb_max_x: float, aabb_max_y: float, aabb_max_z: float, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.TreeInAabbIterator]

        2. iter_node_in_aabb(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, aabb_min_key: erl_geometry.pyerl_geometry.OctreeKey, aabb_max_key: erl_geometry.pyerl_geometry.OctreeKey, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.TreeInAabbIterator]
        """
    @overload
    def iter_node_in_aabb(self, aabb_min_key: OctreeKey, aabb_max_key: OctreeKey, max_depth: int = ...) -> Iterator[PyObjectOccupancyOctreeF.TreeInAabbIterator]:
        """iter_node_in_aabb(*args, **kwargs)
        Overloaded function.

        1. iter_node_in_aabb(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, aabb_min_x: float, aabb_min_y: float, aabb_min_z: float, aabb_max_x: float, aabb_max_y: float, aabb_max_z: float, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.TreeInAabbIterator]

        2. iter_node_in_aabb(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, aabb_min_key: erl_geometry.pyerl_geometry.OctreeKey, aabb_max_key: erl_geometry.pyerl_geometry.OctreeKey, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.TreeInAabbIterator]
        """
    def iter_node_on_ray(self, px: float, py: float, pz: float, vx: float, vy: float, vz: float, max_range: float = ..., node_padding: float = ..., bidirectional: bool = ..., leaf_only: bool = ..., min_node_depth: int = ..., max_node_depth: int = ...) -> Iterator[PyObjectOccupancyOctreeF.NodeOnRayIterator]:
        """iter_node_on_ray(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, px: float, py: float, pz: float, vx: float, vy: float, vz: float, max_range: float = -1, node_padding: float = 0, bidirectional: bool = False, leaf_only: bool = True, min_node_depth: int = 0, max_node_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.NodeOnRayIterator]"""
    @overload
    def iter_north_leaf_neighbor(self, x: float, y: float, z: float, max_leaf_depth: int = ...) -> Iterator[PyObjectOccupancyOctreeF.NorthLeafNeighborIterator]:
        """iter_north_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_north_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.NorthLeafNeighborIterator]

        2. iter_north_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.NorthLeafNeighborIterator]
        """
    @overload
    def iter_north_leaf_neighbor(self, key: OctreeKey, key_depth: int, max_leaf_depth: int = ...) -> Iterator[PyObjectOccupancyOctreeF.NorthLeafNeighborIterator]:
        """iter_north_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_north_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.NorthLeafNeighborIterator]

        2. iter_north_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.NorthLeafNeighborIterator]
        """
    @overload
    def iter_south_leaf_neighbor(self, x: float, y: float, z: float, max_leaf_depth: int = ...) -> Iterator[PyObjectOccupancyOctreeF.SouthLeafNeighborIterator]:
        """iter_south_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_south_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.SouthLeafNeighborIterator]

        2. iter_south_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.SouthLeafNeighborIterator]
        """
    @overload
    def iter_south_leaf_neighbor(self, key: OctreeKey, key_depth: int, max_leaf_depth: int = ...) -> Iterator[PyObjectOccupancyOctreeF.SouthLeafNeighborIterator]:
        """iter_south_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_south_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.SouthLeafNeighborIterator]

        2. iter_south_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.SouthLeafNeighborIterator]
        """
    @overload
    def iter_top_leaf_neighbor(self, x: float, y: float, z: float, max_leaf_depth: int = ...) -> Iterator[PyObjectOccupancyOctreeF.TopLeafNeighborIterator]:
        """iter_top_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_top_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.TopLeafNeighborIterator]

        2. iter_top_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.TopLeafNeighborIterator]
        """
    @overload
    def iter_top_leaf_neighbor(self, key: OctreeKey, key_depth: int, max_leaf_depth: int = ...) -> Iterator[PyObjectOccupancyOctreeF.TopLeafNeighborIterator]:
        """iter_top_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_top_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.TopLeafNeighborIterator]

        2. iter_top_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.TopLeafNeighborIterator]
        """
    @overload
    def iter_west_leaf_neighbor(self, x: float, y: float, z: float, max_leaf_depth: int = ...) -> Iterator[PyObjectOccupancyOctreeF.WestLeafNeighborIterator]:
        """iter_west_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_west_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.WestLeafNeighborIterator]

        2. iter_west_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.WestLeafNeighborIterator]
        """
    @overload
    def iter_west_leaf_neighbor(self, key: OctreeKey, key_depth: int, max_leaf_depth: int = ...) -> Iterator[PyObjectOccupancyOctreeF.WestLeafNeighborIterator]:
        """iter_west_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_west_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.WestLeafNeighborIterator]

        2. iter_west_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF.WestLeafNeighborIterator]
        """
    @overload
    def key_to_coord(self, key: int) -> float:
        """key_to_coord(*args, **kwargs)
        Overloaded function.

        1. key_to_coord(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, key: int) -> float

        2. key_to_coord(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, key: int, depth: int) -> float

        3. key_to_coord(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey) -> tuple[float, float, float]

        4. key_to_coord(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> tuple[float, float, float]
        """
    @overload
    def key_to_coord(self, key: int, depth: int) -> float:
        """key_to_coord(*args, **kwargs)
        Overloaded function.

        1. key_to_coord(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, key: int) -> float

        2. key_to_coord(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, key: int, depth: int) -> float

        3. key_to_coord(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey) -> tuple[float, float, float]

        4. key_to_coord(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> tuple[float, float, float]
        """
    @overload
    def key_to_coord(self, key: OctreeKey) -> tuple[float, float, float]:
        """key_to_coord(*args, **kwargs)
        Overloaded function.

        1. key_to_coord(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, key: int) -> float

        2. key_to_coord(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, key: int, depth: int) -> float

        3. key_to_coord(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey) -> tuple[float, float, float]

        4. key_to_coord(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> tuple[float, float, float]
        """
    @overload
    def key_to_coord(self, key: OctreeKey, depth: int) -> tuple[float, float, float]:
        """key_to_coord(*args, **kwargs)
        Overloaded function.

        1. key_to_coord(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, key: int) -> float

        2. key_to_coord(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, key: int, depth: int) -> float

        3. key_to_coord(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey) -> tuple[float, float, float]

        4. key_to_coord(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> tuple[float, float, float]
        """
    def prune(self) -> None:
        """prune(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF) -> None"""
    def prune_node(self, node: PyObjectOccupancyOctreeNode) -> bool:
        """prune_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, node: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode) -> bool"""
    def sample_positions(self, num_positions: int) -> list[numpy.ndarray[numpy.float32[3, 1]]]:
        """sample_positions(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, num_positions: int) -> list[numpy.ndarray[numpy.float32[3, 1]]]"""
    @overload
    def search(self, x: float, y: float, z: float, max_depth: int = ...) -> PyObjectOccupancyOctreeNode:
        """search(*args, **kwargs)
        Overloaded function.

        1. search(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, x: float, y: float, z: float, max_depth: int = 0) -> erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode

        2. search(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, max_depth: int = 0) -> erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode
        """
    @overload
    def search(self, key: OctreeKey, max_depth: int = ...) -> PyObjectOccupancyOctreeNode:
        """search(*args, **kwargs)
        Overloaded function.

        1. search(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, x: float, y: float, z: float, max_depth: int = 0) -> erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode

        2. search(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, max_depth: int = 0) -> erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode
        """
    def to_max_likelihood(self) -> None:
        """to_max_likelihood(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF) -> None"""
    def update_inner_occupancy(self) -> None:
        """update_inner_occupancy(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF) -> None"""
    @overload
    def update_node(self, x: float, y: float, z: float, occupied: bool, lazy_eval: bool) -> PyObjectOccupancyOctreeNode:
        """update_node(*args, **kwargs)
        Overloaded function.

        1. update_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, x: float, y: float, z: float, occupied: bool, lazy_eval: bool) -> erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode

        2. update_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, node_key: erl_geometry.pyerl_geometry.OctreeKey, occupied: bool, lazy_eval: bool) -> erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode

        3. update_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, x: float, y: float, z: float, log_odds_delta: float, lazy_eval: bool) -> erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode

        4. update_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, node_key: erl_geometry.pyerl_geometry.OctreeKey, log_odds_delta: float, lazy_eval: bool) -> erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode
        """
    @overload
    def update_node(self, node_key: OctreeKey, occupied: bool, lazy_eval: bool) -> PyObjectOccupancyOctreeNode:
        """update_node(*args, **kwargs)
        Overloaded function.

        1. update_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, x: float, y: float, z: float, occupied: bool, lazy_eval: bool) -> erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode

        2. update_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, node_key: erl_geometry.pyerl_geometry.OctreeKey, occupied: bool, lazy_eval: bool) -> erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode

        3. update_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, x: float, y: float, z: float, log_odds_delta: float, lazy_eval: bool) -> erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode

        4. update_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, node_key: erl_geometry.pyerl_geometry.OctreeKey, log_odds_delta: float, lazy_eval: bool) -> erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode
        """
    @overload
    def update_node(self, x: float, y: float, z: float, log_odds_delta: float, lazy_eval: bool) -> PyObjectOccupancyOctreeNode:
        """update_node(*args, **kwargs)
        Overloaded function.

        1. update_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, x: float, y: float, z: float, occupied: bool, lazy_eval: bool) -> erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode

        2. update_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, node_key: erl_geometry.pyerl_geometry.OctreeKey, occupied: bool, lazy_eval: bool) -> erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode

        3. update_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, x: float, y: float, z: float, log_odds_delta: float, lazy_eval: bool) -> erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode

        4. update_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, node_key: erl_geometry.pyerl_geometry.OctreeKey, log_odds_delta: float, lazy_eval: bool) -> erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode
        """
    @overload
    def update_node(self, node_key: OctreeKey, log_odds_delta: float, lazy_eval: bool) -> PyObjectOccupancyOctreeNode:
        """update_node(*args, **kwargs)
        Overloaded function.

        1. update_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, x: float, y: float, z: float, occupied: bool, lazy_eval: bool) -> erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode

        2. update_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, node_key: erl_geometry.pyerl_geometry.OctreeKey, occupied: bool, lazy_eval: bool) -> erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode

        3. update_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, x: float, y: float, z: float, log_odds_delta: float, lazy_eval: bool) -> erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode

        4. update_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, node_key: erl_geometry.pyerl_geometry.OctreeKey, log_odds_delta: float, lazy_eval: bool) -> erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode
        """
    def visualize(self, leaf_only: bool = ..., scaling: float = ..., area_min: numpy.ndarray[numpy.float64[3, 1]] = ..., area_max: numpy.ndarray[numpy.float64[3, 1]] = ..., border_color: numpy.ndarray[numpy.float64[3, 1]] = ..., occupied_color: numpy.ndarray[numpy.float64[3, 1]] = ..., occupied_only: bool = ..., draw_node_boxes: bool = ..., draw_node_borders: bool = ..., window_width: int = ..., window_height: int = ..., window_left: int = ..., window_top: int = ...) -> None:
        """visualize(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF, leaf_only: bool = False, scaling: float = 1.0, area_min: numpy.ndarray[numpy.float64[3, 1]] = array([-1., -1., -1.], dtype=float32), area_max: numpy.ndarray[numpy.float64[3, 1]] = array([1., 1., 1.], dtype=float32), border_color: numpy.ndarray[numpy.float64[3, 1]] = array([0., 0., 0.], dtype=float32), occupied_color: numpy.ndarray[numpy.float64[3, 1]] = array([0.5, 0.5, 0.5], dtype=float32), occupied_only: bool = False, draw_node_boxes: bool = True, draw_node_borders: bool = True, window_width: int = 1920, window_height: int = 1080, window_left: int = 50, window_top: int = 50) -> None"""
    @property
    def memory_usage(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF) -> int"""
    @property
    def memory_usage_per_node(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF) -> int"""
    @property
    def metric_aabb(self) -> Aabb3Df:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF) -> erl_geometry.pyerl_geometry.Aabb3Df"""
    @property
    def metric_max(self) -> numpy.ndarray[numpy.float32[3, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF) -> numpy.ndarray[numpy.float32[3, 1]]"""
    @property
    def metric_min(self) -> numpy.ndarray[numpy.float32[3, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF) -> numpy.ndarray[numpy.float32[3, 1]]"""
    @property
    def metric_min_max(self) -> tuple[numpy.ndarray[numpy.float32[3, 1]], numpy.ndarray[numpy.float32[3, 1]]]:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF) -> tuple[numpy.ndarray[numpy.float32[3, 1]], numpy.ndarray[numpy.float32[3, 1]]]"""
    @property
    def metric_size(self) -> numpy.ndarray[numpy.float32[3, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF) -> numpy.ndarray[numpy.float32[3, 1]]"""
    @property
    def number_of_leaf_nodes(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF) -> int"""
    @property
    def number_of_nodes(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF) -> int"""
    @property
    def resolution(self) -> float:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF) -> float"""
    @property
    def root(self) -> PyObjectOccupancyOctreeNode:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF) -> erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode"""
    @property
    def setting(self) -> OccupancyOctreeBaseSetting:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF) -> erl_geometry.pyerl_geometry.OccupancyOctreeBaseSetting"""
    @property
    def tree_center(self) -> numpy.ndarray[numpy.float32[3, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF) -> numpy.ndarray[numpy.float32[3, 1]]"""
    @property
    def tree_center_key(self) -> OctreeKey:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF) -> erl_geometry.pyerl_geometry.OctreeKey"""
    @property
    def tree_depth(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF) -> int"""
    @property
    def tree_max_half_size(self) -> numpy.ndarray[numpy.float32[3, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeF) -> numpy.ndarray[numpy.float32[3, 1]]"""

class PyObjectOccupancyOctreeNode(AbstractOctreeNode):
    py_object: object
    def __init__(self, *args, **kwargs) -> None:
        """Initialize self.  See help(type(self)) for accurate signature."""
    def add_log_odds(self, log_odds: float) -> None:
        """add_log_odds(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode, log_odds: float) -> None"""
    def allow_update_log_odds(self, delta: float) -> bool:
        """allow_update_log_odds(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode, delta: float) -> bool"""
    def get_child(self, child_idx: int) -> PyObjectOccupancyOctreeNode:
        """get_child(self: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode, child_idx: int) -> erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode"""
    @property
    def log_odds(self) -> float:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode) -> float"""
    @property
    def max_child_log_odds(self) -> float:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode) -> float"""
    @property
    def mean_child_log_odds(self) -> float:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode) -> float"""
    @property
    def occupancy(self) -> float:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyOctreeNode) -> float"""

class PyObjectOccupancyQuadtreeD(AbstractOccupancyQuadtreeD):
    class BatchRayCaster:
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""
        def step(self, mask: numpy.ndarray[bool[m, 1]] = ...) -> PyObjectOccupancyQuadtreeD.BatchRayCaster:
            """step(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.BatchRayCaster, mask: numpy.ndarray[bool[m, 1]] = array([], dtype=bool)) -> erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.BatchRayCaster"""
        @property
        def ever_hit_flags(self) -> numpy.ndarray[bool[m, 1]]:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.BatchRayCaster) -> numpy.ndarray[bool[m, 1]]"""
        @property
        def frontier_keys(self) -> list[QuadtreeKey]:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.BatchRayCaster) -> list[erl_geometry.pyerl_geometry.QuadtreeKey]"""
        @property
        def frontier_nodes(self) -> list[PyObjectOccupancyQuadtreeNode]:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.BatchRayCaster) -> list[erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode]"""
        @property
        def frontier_ray_indices(self) -> list[list[int]]:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.BatchRayCaster) -> list[list[int]]"""
        @property
        def hit_distances(self) -> numpy.ndarray[numpy.float64[m, 1]]:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.BatchRayCaster) -> numpy.ndarray[numpy.float64[m, 1]]"""
        @property
        def hit_flags(self) -> numpy.ndarray[bool[m, 1]]:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.BatchRayCaster) -> numpy.ndarray[bool[m, 1]]"""
        @property
        def hit_nodes(self) -> list[PyObjectOccupancyQuadtreeNode]:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.BatchRayCaster) -> list[erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode]"""
        @property
        def hit_positions(self) -> list[numpy.ndarray[numpy.float64[2, 1]]]:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.BatchRayCaster) -> list[numpy.ndarray[numpy.float64[2, 1]]]"""
        @property
        def num_rays(self) -> int:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.BatchRayCaster) -> int"""
        @property
        def ray_directions(self) -> numpy.ndarray[numpy.float64[2, n]]:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.BatchRayCaster) -> numpy.ndarray[numpy.float64[2, n]]"""
        @property
        def ray_origins(self) -> numpy.ndarray[numpy.float64[2, n]]:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.BatchRayCaster) -> numpy.ndarray[numpy.float64[2, n]]"""

    class Drawer:
        def __init__(self, setting: OccupancyQuadtreeDrawerSettingD, quadtree: PyObjectOccupancyQuadtreeD = ...) -> None:
            """__init__(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.Drawer, setting: erl_geometry.pyerl_geometry.OccupancyQuadtreeDrawerSettingD, quadtree: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD = None) -> None"""
        @staticmethod
        def Setting() -> OccupancyQuadtreeDrawerSettingD:
            """Setting() -> erl_geometry.pyerl_geometry.OccupancyQuadtreeDrawerSettingD"""
        @overload
        def draw_leaves(self) -> Mat:
            """draw_leaves(*args, **kwargs)
            Overloaded function.

            1. draw_leaves(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.Drawer) -> Mat

            2. draw_leaves(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.Drawer, filename: str) -> None
            """
        @overload
        def draw_leaves(self, filename: str) -> None:
            """draw_leaves(*args, **kwargs)
            Overloaded function.

            1. draw_leaves(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.Drawer) -> Mat

            2. draw_leaves(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.Drawer, filename: str) -> None
            """
        @overload
        def draw_tree(self) -> Mat:
            """draw_tree(*args, **kwargs)
            Overloaded function.

            1. draw_tree(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.Drawer) -> Mat

            2. draw_tree(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.Drawer, filename: str) -> None
            """
        @overload
        def draw_tree(self, filename: str) -> None:
            """draw_tree(*args, **kwargs)
            Overloaded function.

            1. draw_tree(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.Drawer) -> Mat

            2. draw_tree(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.Drawer, filename: str) -> None
            """
        def set_draw_leaf_callback(self, callback: Callable[[PyObjectOccupancyQuadtreeD.Drawer, Mat, PyObjectOccupancyQuadtreeD.LeafIterator], None]) -> None:
            """set_draw_leaf_callback(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.Drawer, callback: Callable[[erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.Drawer, Mat, erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.LeafIterator], None]) -> None"""
        def set_draw_tree_callback(self, callback: Callable[[PyObjectOccupancyQuadtreeD.Drawer, Mat, PyObjectOccupancyQuadtreeD.TreeIterator], None]) -> None:
            """set_draw_tree_callback(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.Drawer, callback: Callable[[erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.Drawer, Mat, erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.TreeIterator], None]) -> None"""
        @property
        def grid_map_info(self):
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.Drawer) -> erl::common::GridMapInfo<float, 2>"""
        @property
        def setting(self) -> OccupancyQuadtreeDrawerSettingD:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.Drawer) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeDrawerSettingD"""

    class EastLeafNeighborIterator(PyObjectOccupancyQuadtreeD.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class IteratorBase(AbstractQuadtreeD.QuadtreeNodeIterator):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""
        def __eq__(self, arg0: PyObjectOccupancyQuadtreeD.IteratorBase) -> bool:
            """__eq__(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.IteratorBase, arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.IteratorBase) -> bool"""
        def __ne__(self, arg0: PyObjectOccupancyQuadtreeD.IteratorBase) -> bool:
            """__ne__(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.IteratorBase, arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.IteratorBase) -> bool"""
        @property
        def index_key(self) -> QuadtreeKey:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.IteratorBase) -> erl_geometry.pyerl_geometry.QuadtreeKey"""
        @property
        def key(self) -> QuadtreeKey:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.IteratorBase) -> erl_geometry.pyerl_geometry.QuadtreeKey"""
        @property
        def node(self) -> PyObjectOccupancyQuadtreeNode:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.IteratorBase) -> erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode"""
        @property
        def node_aabb(self) -> Aabb2Dd:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.IteratorBase) -> erl_geometry.pyerl_geometry.Aabb2Dd"""

    class LeafInAabbIterator(PyObjectOccupancyQuadtreeD.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class LeafIterator(PyObjectOccupancyQuadtreeD.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class LeafOfNodeIterator(PyObjectOccupancyQuadtreeD.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class NodeOnRayIterator(PyObjectOccupancyQuadtreeD.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""
        @property
        def distance(self) -> float:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.NodeOnRayIterator) -> float"""

    class NorthLeafNeighborIterator(PyObjectOccupancyQuadtreeD.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class SouthLeafNeighborIterator(PyObjectOccupancyQuadtreeD.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class TreeInAabbIterator(PyObjectOccupancyQuadtreeD.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class TreeIterator(PyObjectOccupancyQuadtreeD.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class WestLeafNeighborIterator(PyObjectOccupancyQuadtreeD.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""
    @overload
    def __init__(self) -> None:
        """__init__(*args, **kwargs)
        Overloaded function.

        1. __init__(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD) -> None

        2. __init__(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, setting: erl_geometry.pyerl_geometry.OccupancyQuadtreeBaseSetting) -> None

        3. __init__(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, filename: str) -> None
        """
    @overload
    def __init__(self, setting: OccupancyQuadtreeBaseSetting) -> None:
        """__init__(*args, **kwargs)
        Overloaded function.

        1. __init__(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD) -> None

        2. __init__(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, setting: erl_geometry.pyerl_geometry.OccupancyQuadtreeBaseSetting) -> None

        3. __init__(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, filename: str) -> None
        """
    @overload
    def __init__(self, filename: str) -> None:
        """__init__(*args, **kwargs)
        Overloaded function.

        1. __init__(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD) -> None

        2. __init__(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, setting: erl_geometry.pyerl_geometry.OccupancyQuadtreeBaseSetting) -> None

        3. __init__(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, filename: str) -> None
        """
    def Setting(self) -> OccupancyQuadtreeBaseSetting:
        """Setting() -> erl_geometry.pyerl_geometry.OccupancyQuadtreeBaseSetting"""
    @overload
    def adjust_key_to_depth(self, key: int, depth: int) -> int:
        """adjust_key_to_depth(*args, **kwargs)
        Overloaded function.

        1. adjust_key_to_depth(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, key: int, depth: int) -> int

        2. adjust_key_to_depth(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> erl_geometry.pyerl_geometry.QuadtreeKey
        """
    @overload
    def adjust_key_to_depth(self, key: QuadtreeKey, depth: int) -> QuadtreeKey:
        """adjust_key_to_depth(*args, **kwargs)
        Overloaded function.

        1. adjust_key_to_depth(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, key: int, depth: int) -> int

        2. adjust_key_to_depth(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> erl_geometry.pyerl_geometry.QuadtreeKey
        """
    def cast_ray(self, px: float, py: float, vx: float, vy: float, ignore_unknown: bool, max_range: float) -> dict:
        """cast_ray(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, px: float, py: float, vx: float, vy: float, ignore_unknown: bool, max_range: float) -> dict"""
    @overload
    def cast_rays(self, position: numpy.ndarray[numpy.float64[2, 1]], rotation: numpy.ndarray[numpy.float64[2, 2], flags.f_contiguous], angles: numpy.ndarray[numpy.float64[m, 1]], ignore_unknown: bool, max_range: float, prune_rays: bool, parallel: bool) -> dict:
        """cast_rays(*args, **kwargs)
        Overloaded function.

        1. cast_rays(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, position: numpy.ndarray[numpy.float64[2, 1]], rotation: numpy.ndarray[numpy.float64[2, 2], flags.f_contiguous], angles: numpy.ndarray[numpy.float64[m, 1]], ignore_unknown: bool, max_range: float, prune_rays: bool, parallel: bool) -> dict

        2. cast_rays(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, positions: numpy.ndarray[numpy.float64[2, n], flags.f_contiguous], directions: numpy.ndarray[numpy.float64[2, n], flags.f_contiguous], ignore_unknown: bool, max_range: float, prune_rays: bool, parallel: bool) -> dict
        """
    @overload
    def cast_rays(self, positions: numpy.ndarray[numpy.float64[2, n], flags.f_contiguous], directions: numpy.ndarray[numpy.float64[2, n], flags.f_contiguous], ignore_unknown: bool, max_range: float, prune_rays: bool, parallel: bool) -> dict:
        """cast_rays(*args, **kwargs)
        Overloaded function.

        1. cast_rays(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, position: numpy.ndarray[numpy.float64[2, 1]], rotation: numpy.ndarray[numpy.float64[2, 2], flags.f_contiguous], angles: numpy.ndarray[numpy.float64[m, 1]], ignore_unknown: bool, max_range: float, prune_rays: bool, parallel: bool) -> dict

        2. cast_rays(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, positions: numpy.ndarray[numpy.float64[2, n], flags.f_contiguous], directions: numpy.ndarray[numpy.float64[2, n], flags.f_contiguous], ignore_unknown: bool, max_range: float, prune_rays: bool, parallel: bool) -> dict
        """
    def clear(self) -> None:
        """clear(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD) -> None"""
    def compute_common_ancestor_key(self, arg0: QuadtreeKey, arg1: QuadtreeKey) -> tuple[QuadtreeKey, int]:
        """compute_common_ancestor_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, arg0: erl_geometry.pyerl_geometry.QuadtreeKey, arg1: erl_geometry.pyerl_geometry.QuadtreeKey) -> tuple[erl_geometry.pyerl_geometry.QuadtreeKey, int]"""
    def compute_east_neighbor_key(self, key: QuadtreeKey, depth: int) -> QuadtreeKey | None:
        """compute_east_neighbor_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]"""
    def compute_north_neighbor_key(self, key: QuadtreeKey, depth: int) -> QuadtreeKey | None:
        """compute_north_neighbor_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]"""
    def compute_ray_coords(self, sx: float, sy: float, ex: float, ey: float) -> list[numpy.ndarray[numpy.float64[2, 1]]] | None:
        """compute_ray_coords(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, sx: float, sy: float, ex: float, ey: float) -> Optional[list[numpy.ndarray[numpy.float64[2, 1]]]]"""
    def compute_ray_keys(self, sx: float, sy: float, ex: float, ey: float) -> list[QuadtreeKey] | None:
        """compute_ray_keys(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, sx: float, sy: float, ex: float, ey: float) -> Optional[list[erl_geometry.pyerl_geometry.QuadtreeKey]]"""
    def compute_south_neighbor_key(self, key: QuadtreeKey, depth: int) -> QuadtreeKey | None:
        """compute_south_neighbor_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]"""
    def compute_west_neighbor_key(self, key: QuadtreeKey, depth: int) -> QuadtreeKey | None:
        """compute_west_neighbor_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]"""
    @overload
    def coord_to_key(self, coordinate: float) -> int:
        """coord_to_key(*args, **kwargs)
        Overloaded function.

        1. coord_to_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, coordinate: float) -> int

        2. coord_to_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, coordinate: float, depth: int) -> int

        3. coord_to_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, x: float, y: float) -> erl_geometry.pyerl_geometry.QuadtreeKey

        4. coord_to_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, x: float, y: float, depth: int) -> erl_geometry.pyerl_geometry.QuadtreeKey
        """
    @overload
    def coord_to_key(self, coordinate: float, depth: int) -> int:
        """coord_to_key(*args, **kwargs)
        Overloaded function.

        1. coord_to_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, coordinate: float) -> int

        2. coord_to_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, coordinate: float, depth: int) -> int

        3. coord_to_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, x: float, y: float) -> erl_geometry.pyerl_geometry.QuadtreeKey

        4. coord_to_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, x: float, y: float, depth: int) -> erl_geometry.pyerl_geometry.QuadtreeKey
        """
    @overload
    def coord_to_key(self, x: float, y: float) -> QuadtreeKey:
        """coord_to_key(*args, **kwargs)
        Overloaded function.

        1. coord_to_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, coordinate: float) -> int

        2. coord_to_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, coordinate: float, depth: int) -> int

        3. coord_to_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, x: float, y: float) -> erl_geometry.pyerl_geometry.QuadtreeKey

        4. coord_to_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, x: float, y: float, depth: int) -> erl_geometry.pyerl_geometry.QuadtreeKey
        """
    @overload
    def coord_to_key(self, x: float, y: float, depth: int) -> QuadtreeKey:
        """coord_to_key(*args, **kwargs)
        Overloaded function.

        1. coord_to_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, coordinate: float) -> int

        2. coord_to_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, coordinate: float, depth: int) -> int

        3. coord_to_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, x: float, y: float) -> erl_geometry.pyerl_geometry.QuadtreeKey

        4. coord_to_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, x: float, y: float, depth: int) -> erl_geometry.pyerl_geometry.QuadtreeKey
        """
    @overload
    def coord_to_key_checked(self, coordinate: float) -> int | None:
        """coord_to_key_checked(*args, **kwargs)
        Overloaded function.

        1. coord_to_key_checked(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, coordinate: float) -> Optional[int]

        2. coord_to_key_checked(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, coordinate: float, depth: int) -> Optional[int]

        3. coord_to_key_checked(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, x: float, y: float) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]

        4. coord_to_key_checked(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, x: float, y: float, depth: int) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]
        """
    @overload
    def coord_to_key_checked(self, coordinate: float, depth: int) -> int | None:
        """coord_to_key_checked(*args, **kwargs)
        Overloaded function.

        1. coord_to_key_checked(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, coordinate: float) -> Optional[int]

        2. coord_to_key_checked(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, coordinate: float, depth: int) -> Optional[int]

        3. coord_to_key_checked(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, x: float, y: float) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]

        4. coord_to_key_checked(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, x: float, y: float, depth: int) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]
        """
    @overload
    def coord_to_key_checked(self, x: float, y: float) -> QuadtreeKey | None:
        """coord_to_key_checked(*args, **kwargs)
        Overloaded function.

        1. coord_to_key_checked(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, coordinate: float) -> Optional[int]

        2. coord_to_key_checked(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, coordinate: float, depth: int) -> Optional[int]

        3. coord_to_key_checked(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, x: float, y: float) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]

        4. coord_to_key_checked(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, x: float, y: float, depth: int) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]
        """
    @overload
    def coord_to_key_checked(self, x: float, y: float, depth: int) -> QuadtreeKey | None:
        """coord_to_key_checked(*args, **kwargs)
        Overloaded function.

        1. coord_to_key_checked(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, coordinate: float) -> Optional[int]

        2. coord_to_key_checked(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, coordinate: float, depth: int) -> Optional[int]

        3. coord_to_key_checked(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, x: float, y: float) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]

        4. coord_to_key_checked(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, x: float, y: float, depth: int) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]
        """
    def create_node_child(self, node: PyObjectOccupancyQuadtreeNode, child_idx: int) -> PyObjectOccupancyQuadtreeNode:
        """create_node_child(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, node: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode, child_idx: int) -> erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode"""
    @overload
    def delete_node(self, x: float, y: float, depth: int) -> int:
        """delete_node(*args, **kwargs)
        Overloaded function.

        1. delete_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, x: float, y: float, depth: int) -> int

        2. delete_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> None
        """
    @overload
    def delete_node(self, key: QuadtreeKey, depth: int) -> None:
        """delete_node(*args, **kwargs)
        Overloaded function.

        1. delete_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, x: float, y: float, depth: int) -> int

        2. delete_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> None
        """
    def delete_node_child(self, node: PyObjectOccupancyQuadtreeNode, child_idx: int, key: QuadtreeKey) -> int:
        """delete_node_child(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, node: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode, child_idx: int, key: erl_geometry.pyerl_geometry.QuadtreeKey) -> int"""
    def expand(self) -> None:
        """expand(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD) -> None"""
    def expand_node(self, node: PyObjectOccupancyQuadtreeNode) -> None:
        """expand_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, node: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode) -> None"""
    def get_batch_ray_caster(self, *args, **kwargs):
        """get_batch_ray_caster(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, origins: numpy.ndarray[numpy.float64[2, n]], directions: numpy.ndarray[numpy.float64[2, n]], max_ranges: numpy.ndarray[numpy.float64[m, 1]] = array([], dtype=float64), node_paddings: numpy.ndarray[numpy.float64[m, 1]] = array([], dtype=float64), bidirectional_flags: numpy.ndarray[bool[m, 1]] = array([], dtype=bool), leaf_only_flags: numpy.ndarray[bool[m, 1]] = array([], dtype=bool), min_node_depths: numpy.ndarray[numpy.int32[m, 1]] = array([], dtype=int32), max_node_depths: numpy.ndarray[numpy.int32[m, 1]] = array([], dtype=int32)) -> erl::geometry::OccupancyNdTreeBatchRayCaster<erl::geometry::OccupancyQuadtreeBase<double, erl::geometry::PyObjectOccupancyQuadtreeNode, erl::geometry::OccupancyQuadtreeBaseSetting>, 2>"""
    def get_node_child(self, node: PyObjectOccupancyQuadtreeNode, child_idx: int) -> PyObjectOccupancyQuadtreeNode:
        """get_node_child(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, node: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode, child_idx: int) -> erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode"""
    def get_node_size(self, depth: int) -> float:
        """get_node_size(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, depth: int) -> float"""
    @overload
    def insert_node(self, x: float, y: float, depth: int) -> PyObjectOccupancyQuadtreeNode:
        """insert_node(*args, **kwargs)
        Overloaded function.

        1. insert_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, x: float, y: float, depth: int) -> erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode

        2. insert_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode
        """
    @overload
    def insert_node(self, key: QuadtreeKey, depth: int) -> PyObjectOccupancyQuadtreeNode:
        """insert_node(*args, **kwargs)
        Overloaded function.

        1. insert_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, x: float, y: float, depth: int) -> erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode

        2. insert_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode
        """
    def insert_point_cloud(self, points: numpy.ndarray[numpy.float64[2, n], flags.f_contiguous], sensor_origin: numpy.ndarray[numpy.float64[2, 1]], min_range: float, max_range: float, with_count: bool, parallel: bool, lazy_eval: bool, discrete: bool) -> None:
        """insert_point_cloud(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, points: numpy.ndarray[numpy.float64[2, n], flags.f_contiguous], sensor_origin: numpy.ndarray[numpy.float64[2, 1]], min_range: float, max_range: float, with_count: bool, parallel: bool, lazy_eval: bool, discrete: bool) -> None"""
    def insert_point_cloud_rays(self, points: numpy.ndarray[numpy.float64[2, n], flags.f_contiguous], sensor_origin: numpy.ndarray[numpy.float64[2, 1]], min_range: float, max_range: float, parallel: bool, lazy_eval: bool) -> None:
        """insert_point_cloud_rays(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, points: numpy.ndarray[numpy.float64[2, n], flags.f_contiguous], sensor_origin: numpy.ndarray[numpy.float64[2, 1]], min_range: float, max_range: float, parallel: bool, lazy_eval: bool) -> None"""
    def insert_ray(self, sx: float, sy: float, ex: float, ey: float, min_range: float, max_range: float, lazy_eval: bool) -> bool:
        """insert_ray(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, sx: float, sy: float, ex: float, ey: float, min_range: float, max_range: float, lazy_eval: bool) -> bool"""
    def is_node_collapsible(self, node: PyObjectOccupancyQuadtreeNode) -> bool:
        """is_node_collapsible(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, node: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode) -> bool"""
    @overload
    def iter_east_leaf_neighbor(self, x: float, y: float, max_leaf_depth: int = ...) -> Iterator[PyObjectOccupancyQuadtreeD.EastLeafNeighborIterator]:
        """iter_east_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_east_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, x: float, y: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.EastLeafNeighborIterator]

        2. iter_east_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.EastLeafNeighborIterator]
        """
    @overload
    def iter_east_leaf_neighbor(self, key: QuadtreeKey, key_depth: int, max_leaf_depth: int = ...) -> Iterator[PyObjectOccupancyQuadtreeD.EastLeafNeighborIterator]:
        """iter_east_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_east_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, x: float, y: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.EastLeafNeighborIterator]

        2. iter_east_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.EastLeafNeighborIterator]
        """
    def iter_leaf(self, max_depth: int = ...) -> Iterator[PyObjectOccupancyQuadtreeD.LeafIterator]:
        """iter_leaf(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.LeafIterator]"""
    @overload
    def iter_leaf_in_aabb(self, aabb_min_x: float, aabb_min_y: float, aabb_max_x: float, aabb_max_y: float, max_depth: int = ...) -> Iterator[PyObjectOccupancyQuadtreeD.LeafInAabbIterator]:
        """iter_leaf_in_aabb(*args, **kwargs)
        Overloaded function.

        1. iter_leaf_in_aabb(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, aabb_min_x: float, aabb_min_y: float, aabb_max_x: float, aabb_max_y: float, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.LeafInAabbIterator]

        2. iter_leaf_in_aabb(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, aabb_min_key: erl_geometry.pyerl_geometry.QuadtreeKey, aabb_max_key: erl_geometry.pyerl_geometry.QuadtreeKey, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.LeafInAabbIterator]
        """
    @overload
    def iter_leaf_in_aabb(self, aabb_min_key: QuadtreeKey, aabb_max_key: QuadtreeKey, max_depth: int = ...) -> Iterator[PyObjectOccupancyQuadtreeD.LeafInAabbIterator]:
        """iter_leaf_in_aabb(*args, **kwargs)
        Overloaded function.

        1. iter_leaf_in_aabb(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, aabb_min_x: float, aabb_min_y: float, aabb_max_x: float, aabb_max_y: float, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.LeafInAabbIterator]

        2. iter_leaf_in_aabb(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, aabb_min_key: erl_geometry.pyerl_geometry.QuadtreeKey, aabb_max_key: erl_geometry.pyerl_geometry.QuadtreeKey, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.LeafInAabbIterator]
        """
    def iter_leaf_of_node(self, node_key: QuadtreeKey, node_depth: int, max_depth: int = ...) -> Iterator[PyObjectOccupancyQuadtreeD.LeafOfNodeIterator]:
        """iter_leaf_of_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, node_key: erl_geometry.pyerl_geometry.QuadtreeKey, node_depth: int, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.LeafOfNodeIterator]"""
    def iter_node(self, max_depth: int = ...) -> Iterator[PyObjectOccupancyQuadtreeD.TreeIterator]:
        """iter_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.TreeIterator]"""
    @overload
    def iter_node_in_aabb(self, aabb_min_x: float, aabb_min_y: float, aabb_max_x: float, aabb_max_y: float, max_depth: int = ...) -> Iterator[PyObjectOccupancyQuadtreeD.TreeInAabbIterator]:
        """iter_node_in_aabb(*args, **kwargs)
        Overloaded function.

        1. iter_node_in_aabb(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, aabb_min_x: float, aabb_min_y: float, aabb_max_x: float, aabb_max_y: float, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.TreeInAabbIterator]

        2. iter_node_in_aabb(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, aabb_min_key: erl_geometry.pyerl_geometry.QuadtreeKey, aabb_max_key: erl_geometry.pyerl_geometry.QuadtreeKey, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.TreeInAabbIterator]
        """
    @overload
    def iter_node_in_aabb(self, aabb_min_key: QuadtreeKey, aabb_max_key: QuadtreeKey, max_depth: int = ...) -> Iterator[PyObjectOccupancyQuadtreeD.TreeInAabbIterator]:
        """iter_node_in_aabb(*args, **kwargs)
        Overloaded function.

        1. iter_node_in_aabb(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, aabb_min_x: float, aabb_min_y: float, aabb_max_x: float, aabb_max_y: float, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.TreeInAabbIterator]

        2. iter_node_in_aabb(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, aabb_min_key: erl_geometry.pyerl_geometry.QuadtreeKey, aabb_max_key: erl_geometry.pyerl_geometry.QuadtreeKey, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.TreeInAabbIterator]
        """
    def iter_node_on_ray(self, px: float, py: float, vx: float, vy: float, max_range: float = ..., node_padding: float = ..., bidirectional: bool = ..., leaf_only: bool = ..., min_node_depth: int = ..., max_node_depth: int = ...) -> Iterator[PyObjectOccupancyQuadtreeD.NodeOnRayIterator]:
        """iter_node_on_ray(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, px: float, py: float, vx: float, vy: float, max_range: float = -1, node_padding: float = 0, bidirectional: bool = False, leaf_only: bool = False, min_node_depth: int = 0, max_node_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.NodeOnRayIterator]"""
    @overload
    def iter_north_leaf_neighbor(self, x: float, y: float, max_leaf_depth: int = ...) -> Iterator[PyObjectOccupancyQuadtreeD.NorthLeafNeighborIterator]:
        """iter_north_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_north_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, x: float, y: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.NorthLeafNeighborIterator]

        2. iter_north_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.NorthLeafNeighborIterator]
        """
    @overload
    def iter_north_leaf_neighbor(self, key: QuadtreeKey, key_depth: int, max_leaf_depth: int = ...) -> Iterator[PyObjectOccupancyQuadtreeD.NorthLeafNeighborIterator]:
        """iter_north_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_north_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, x: float, y: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.NorthLeafNeighborIterator]

        2. iter_north_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.NorthLeafNeighborIterator]
        """
    @overload
    def iter_south_leaf_neighbor(self, x: float, y: float, max_leaf_depth: int = ...) -> Iterator[PyObjectOccupancyQuadtreeD.SouthLeafNeighborIterator]:
        """iter_south_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_south_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, x: float, y: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.SouthLeafNeighborIterator]

        2. iter_south_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.SouthLeafNeighborIterator]
        """
    @overload
    def iter_south_leaf_neighbor(self, key: QuadtreeKey, key_depth: int, max_leaf_depth: int = ...) -> Iterator[PyObjectOccupancyQuadtreeD.SouthLeafNeighborIterator]:
        """iter_south_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_south_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, x: float, y: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.SouthLeafNeighborIterator]

        2. iter_south_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.SouthLeafNeighborIterator]
        """
    @overload
    def iter_west_leaf_neighbor(self, x: float, y: float, max_leaf_depth: int = ...) -> Iterator[PyObjectOccupancyQuadtreeD.WestLeafNeighborIterator]:
        """iter_west_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_west_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, x: float, y: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.WestLeafNeighborIterator]

        2. iter_west_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.WestLeafNeighborIterator]
        """
    @overload
    def iter_west_leaf_neighbor(self, key: QuadtreeKey, key_depth: int, max_leaf_depth: int = ...) -> Iterator[PyObjectOccupancyQuadtreeD.WestLeafNeighborIterator]:
        """iter_west_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_west_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, x: float, y: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.WestLeafNeighborIterator]

        2. iter_west_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD.WestLeafNeighborIterator]
        """
    @overload
    def key_to_coord(self, key: int) -> float:
        """key_to_coord(*args, **kwargs)
        Overloaded function.

        1. key_to_coord(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, key: int) -> float

        2. key_to_coord(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, key: int, depth: int) -> float

        3. key_to_coord(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey) -> tuple[float, float]

        4. key_to_coord(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> tuple[float, float]
        """
    @overload
    def key_to_coord(self, key: int, depth: int) -> float:
        """key_to_coord(*args, **kwargs)
        Overloaded function.

        1. key_to_coord(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, key: int) -> float

        2. key_to_coord(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, key: int, depth: int) -> float

        3. key_to_coord(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey) -> tuple[float, float]

        4. key_to_coord(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> tuple[float, float]
        """
    @overload
    def key_to_coord(self, key: QuadtreeKey) -> tuple[float, float]:
        """key_to_coord(*args, **kwargs)
        Overloaded function.

        1. key_to_coord(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, key: int) -> float

        2. key_to_coord(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, key: int, depth: int) -> float

        3. key_to_coord(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey) -> tuple[float, float]

        4. key_to_coord(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> tuple[float, float]
        """
    @overload
    def key_to_coord(self, key: QuadtreeKey, depth: int) -> tuple[float, float]:
        """key_to_coord(*args, **kwargs)
        Overloaded function.

        1. key_to_coord(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, key: int) -> float

        2. key_to_coord(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, key: int, depth: int) -> float

        3. key_to_coord(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey) -> tuple[float, float]

        4. key_to_coord(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> tuple[float, float]
        """
    def prune(self) -> None:
        """prune(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD) -> None"""
    def prune_node(self, node: PyObjectOccupancyQuadtreeNode) -> bool:
        """prune_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, node: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode) -> bool"""
    def sample_positions(self, num_positions: int) -> list[numpy.ndarray[numpy.float64[2, 1]]]:
        """sample_positions(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, num_positions: int) -> list[numpy.ndarray[numpy.float64[2, 1]]]"""
    @overload
    def search(self, x: float, y: float, max_depth: int = ...) -> PyObjectOccupancyQuadtreeNode:
        """search(*args, **kwargs)
        Overloaded function.

        1. search(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, x: float, y: float, max_depth: int = 0) -> erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode

        2. search(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, max_depth: int = 0) -> erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode
        """
    @overload
    def search(self, key: QuadtreeKey, max_depth: int = ...) -> PyObjectOccupancyQuadtreeNode:
        """search(*args, **kwargs)
        Overloaded function.

        1. search(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, x: float, y: float, max_depth: int = 0) -> erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode

        2. search(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, max_depth: int = 0) -> erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode
        """
    def to_max_likelihood(self) -> None:
        """to_max_likelihood(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD) -> None"""
    def update_inner_occupancy(self) -> None:
        """update_inner_occupancy(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD) -> None"""
    @overload
    def update_node(self, x: float, y: float, occupied: bool, lazy_eval: bool) -> PyObjectOccupancyQuadtreeNode:
        """update_node(*args, **kwargs)
        Overloaded function.

        1. update_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, x: float, y: float, occupied: bool, lazy_eval: bool) -> erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode

        2. update_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, node_key: erl_geometry.pyerl_geometry.QuadtreeKey, occupied: bool, lazy_eval: bool) -> erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode

        3. update_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, x: float, y: float, log_odds_delta: float, lazy_eval: bool) -> erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode

        4. update_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, node_key: erl_geometry.pyerl_geometry.QuadtreeKey, log_odds_delta: float, lazy_eval: bool) -> erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode
        """
    @overload
    def update_node(self, node_key: QuadtreeKey, occupied: bool, lazy_eval: bool) -> PyObjectOccupancyQuadtreeNode:
        """update_node(*args, **kwargs)
        Overloaded function.

        1. update_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, x: float, y: float, occupied: bool, lazy_eval: bool) -> erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode

        2. update_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, node_key: erl_geometry.pyerl_geometry.QuadtreeKey, occupied: bool, lazy_eval: bool) -> erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode

        3. update_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, x: float, y: float, log_odds_delta: float, lazy_eval: bool) -> erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode

        4. update_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, node_key: erl_geometry.pyerl_geometry.QuadtreeKey, log_odds_delta: float, lazy_eval: bool) -> erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode
        """
    @overload
    def update_node(self, x: float, y: float, log_odds_delta: float, lazy_eval: bool) -> PyObjectOccupancyQuadtreeNode:
        """update_node(*args, **kwargs)
        Overloaded function.

        1. update_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, x: float, y: float, occupied: bool, lazy_eval: bool) -> erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode

        2. update_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, node_key: erl_geometry.pyerl_geometry.QuadtreeKey, occupied: bool, lazy_eval: bool) -> erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode

        3. update_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, x: float, y: float, log_odds_delta: float, lazy_eval: bool) -> erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode

        4. update_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, node_key: erl_geometry.pyerl_geometry.QuadtreeKey, log_odds_delta: float, lazy_eval: bool) -> erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode
        """
    @overload
    def update_node(self, node_key: QuadtreeKey, log_odds_delta: float, lazy_eval: bool) -> PyObjectOccupancyQuadtreeNode:
        """update_node(*args, **kwargs)
        Overloaded function.

        1. update_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, x: float, y: float, occupied: bool, lazy_eval: bool) -> erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode

        2. update_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, node_key: erl_geometry.pyerl_geometry.QuadtreeKey, occupied: bool, lazy_eval: bool) -> erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode

        3. update_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, x: float, y: float, log_odds_delta: float, lazy_eval: bool) -> erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode

        4. update_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, node_key: erl_geometry.pyerl_geometry.QuadtreeKey, log_odds_delta: float, lazy_eval: bool) -> erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode
        """
    def visualize(self, leaf_only: bool = ..., area_min: numpy.ndarray[numpy.float32[2, 1]] | None = ..., area_max: numpy.ndarray[numpy.float32[2, 1]] | None = ..., resolution: float = ..., padding: int = ..., bg_color: numpy.ndarray[numpy.int32[4, 1]] = ..., fg_color: numpy.ndarray[numpy.int32[4, 1]] = ..., occupied_color: numpy.ndarray[numpy.int32[4, 1]] = ..., free_color: numpy.ndarray[numpy.int32[4, 1]] = ..., border_color: numpy.ndarray[numpy.int32[4, 1]] = ..., border_thickness: int = ...) -> numpy.ndarray[numpy.uint8[m, n]]:
        """visualize(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD, leaf_only: bool = False, area_min: Optional[numpy.ndarray[numpy.float32[2, 1]]] = None, area_max: Optional[numpy.ndarray[numpy.float32[2, 1]]] = None, resolution: float = 0.1, padding: int = 1, bg_color: numpy.ndarray[numpy.int32[4, 1]] = array([128, 128, 128, 255], dtype=int32), fg_color: numpy.ndarray[numpy.int32[4, 1]] = array([255, 255, 255, 255], dtype=int32), occupied_color: numpy.ndarray[numpy.int32[4, 1]] = array([ 0, 0, 0, 255], dtype=int32), free_color: numpy.ndarray[numpy.int32[4, 1]] = array([255, 255, 255, 255], dtype=int32), border_color: numpy.ndarray[numpy.int32[4, 1]] = array([ 0, 0, 0, 255], dtype=int32), border_thickness: int = 1) -> numpy.ndarray[numpy.uint8[m, n]]"""
    @property
    def memory_usage(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD) -> int"""
    @property
    def memory_usage_per_node(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD) -> int"""
    @property
    def metric_aabb(self) -> Aabb2Dd:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD) -> erl_geometry.pyerl_geometry.Aabb2Dd"""
    @property
    def metric_max(self) -> numpy.ndarray[numpy.float64[2, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD) -> numpy.ndarray[numpy.float64[2, 1]]"""
    @property
    def metric_min(self) -> numpy.ndarray[numpy.float64[2, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD) -> numpy.ndarray[numpy.float64[2, 1]]"""
    @property
    def metric_min_max(self) -> tuple[numpy.ndarray[numpy.float64[2, 1]], numpy.ndarray[numpy.float64[2, 1]]]:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD) -> tuple[numpy.ndarray[numpy.float64[2, 1]], numpy.ndarray[numpy.float64[2, 1]]]"""
    @property
    def metric_size(self) -> numpy.ndarray[numpy.float64[2, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD) -> numpy.ndarray[numpy.float64[2, 1]]"""
    @property
    def number_of_leaf_nodes(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD) -> int"""
    @property
    def number_of_nodes(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD) -> int"""
    @property
    def resolution(self) -> float:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD) -> float"""
    @property
    def root(self) -> PyObjectOccupancyQuadtreeNode:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD) -> erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode"""
    @property
    def setting(self) -> OccupancyQuadtreeBaseSetting:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeBaseSetting"""
    @property
    def tree_center(self) -> numpy.ndarray[numpy.float64[2, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD) -> numpy.ndarray[numpy.float64[2, 1]]"""
    @property
    def tree_center_key(self) -> QuadtreeKey:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD) -> erl_geometry.pyerl_geometry.QuadtreeKey"""
    @property
    def tree_depth(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD) -> int"""
    @property
    def tree_max_half_size(self) -> numpy.ndarray[numpy.float64[2, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeD) -> numpy.ndarray[numpy.float64[2, 1]]"""

class PyObjectOccupancyQuadtreeF(AbstractOccupancyQuadtreeF):
    class BatchRayCaster:
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""
        def step(self, mask: numpy.ndarray[bool[m, 1]] = ...) -> PyObjectOccupancyQuadtreeF.BatchRayCaster:
            """step(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.BatchRayCaster, mask: numpy.ndarray[bool[m, 1]] = array([], dtype=bool)) -> erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.BatchRayCaster"""
        @property
        def ever_hit_flags(self) -> numpy.ndarray[bool[m, 1]]:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.BatchRayCaster) -> numpy.ndarray[bool[m, 1]]"""
        @property
        def frontier_keys(self) -> list[QuadtreeKey]:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.BatchRayCaster) -> list[erl_geometry.pyerl_geometry.QuadtreeKey]"""
        @property
        def frontier_nodes(self) -> list[PyObjectOccupancyQuadtreeNode]:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.BatchRayCaster) -> list[erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode]"""
        @property
        def frontier_ray_indices(self) -> list[list[int]]:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.BatchRayCaster) -> list[list[int]]"""
        @property
        def hit_distances(self) -> numpy.ndarray[numpy.float32[m, 1]]:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.BatchRayCaster) -> numpy.ndarray[numpy.float32[m, 1]]"""
        @property
        def hit_flags(self) -> numpy.ndarray[bool[m, 1]]:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.BatchRayCaster) -> numpy.ndarray[bool[m, 1]]"""
        @property
        def hit_nodes(self) -> list[PyObjectOccupancyQuadtreeNode]:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.BatchRayCaster) -> list[erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode]"""
        @property
        def hit_positions(self) -> list[numpy.ndarray[numpy.float32[2, 1]]]:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.BatchRayCaster) -> list[numpy.ndarray[numpy.float32[2, 1]]]"""
        @property
        def num_rays(self) -> int:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.BatchRayCaster) -> int"""
        @property
        def ray_directions(self) -> numpy.ndarray[numpy.float32[2, n]]:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.BatchRayCaster) -> numpy.ndarray[numpy.float32[2, n]]"""
        @property
        def ray_origins(self) -> numpy.ndarray[numpy.float32[2, n]]:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.BatchRayCaster) -> numpy.ndarray[numpy.float32[2, n]]"""

    class Drawer:
        def __init__(self, setting: OccupancyQuadtreeDrawerSettingF, quadtree: PyObjectOccupancyQuadtreeF = ...) -> None:
            """__init__(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.Drawer, setting: erl_geometry.pyerl_geometry.OccupancyQuadtreeDrawerSettingF, quadtree: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF = None) -> None"""
        @staticmethod
        def Setting() -> OccupancyQuadtreeDrawerSettingF:
            """Setting() -> erl_geometry.pyerl_geometry.OccupancyQuadtreeDrawerSettingF"""
        @overload
        def draw_leaves(self) -> Mat:
            """draw_leaves(*args, **kwargs)
            Overloaded function.

            1. draw_leaves(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.Drawer) -> Mat

            2. draw_leaves(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.Drawer, filename: str) -> None
            """
        @overload
        def draw_leaves(self, filename: str) -> None:
            """draw_leaves(*args, **kwargs)
            Overloaded function.

            1. draw_leaves(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.Drawer) -> Mat

            2. draw_leaves(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.Drawer, filename: str) -> None
            """
        @overload
        def draw_tree(self) -> Mat:
            """draw_tree(*args, **kwargs)
            Overloaded function.

            1. draw_tree(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.Drawer) -> Mat

            2. draw_tree(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.Drawer, filename: str) -> None
            """
        @overload
        def draw_tree(self, filename: str) -> None:
            """draw_tree(*args, **kwargs)
            Overloaded function.

            1. draw_tree(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.Drawer) -> Mat

            2. draw_tree(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.Drawer, filename: str) -> None
            """
        def set_draw_leaf_callback(self, callback: Callable[[PyObjectOccupancyQuadtreeF.Drawer, Mat, PyObjectOccupancyQuadtreeF.LeafIterator], None]) -> None:
            """set_draw_leaf_callback(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.Drawer, callback: Callable[[erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.Drawer, Mat, erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.LeafIterator], None]) -> None"""
        def set_draw_tree_callback(self, callback: Callable[[PyObjectOccupancyQuadtreeF.Drawer, Mat, PyObjectOccupancyQuadtreeF.TreeIterator], None]) -> None:
            """set_draw_tree_callback(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.Drawer, callback: Callable[[erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.Drawer, Mat, erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.TreeIterator], None]) -> None"""
        @property
        def grid_map_info(self):
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.Drawer) -> erl::common::GridMapInfo<float, 2>"""
        @property
        def setting(self) -> OccupancyQuadtreeDrawerSettingF:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.Drawer) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeDrawerSettingF"""

    class EastLeafNeighborIterator(PyObjectOccupancyQuadtreeF.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class IteratorBase(AbstractQuadtreeF.QuadtreeNodeIterator):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""
        def __eq__(self, arg0: PyObjectOccupancyQuadtreeF.IteratorBase) -> bool:
            """__eq__(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.IteratorBase, arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.IteratorBase) -> bool"""
        def __ne__(self, arg0: PyObjectOccupancyQuadtreeF.IteratorBase) -> bool:
            """__ne__(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.IteratorBase, arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.IteratorBase) -> bool"""
        @property
        def index_key(self) -> QuadtreeKey:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.IteratorBase) -> erl_geometry.pyerl_geometry.QuadtreeKey"""
        @property
        def key(self) -> QuadtreeKey:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.IteratorBase) -> erl_geometry.pyerl_geometry.QuadtreeKey"""
        @property
        def node(self) -> PyObjectOccupancyQuadtreeNode:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.IteratorBase) -> erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode"""
        @property
        def node_aabb(self) -> Aabb2Df:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.IteratorBase) -> erl_geometry.pyerl_geometry.Aabb2Df"""

    class LeafInAabbIterator(PyObjectOccupancyQuadtreeF.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class LeafIterator(PyObjectOccupancyQuadtreeF.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class LeafOfNodeIterator(PyObjectOccupancyQuadtreeF.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class NodeOnRayIterator(PyObjectOccupancyQuadtreeF.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""
        @property
        def distance(self) -> float:
            """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.NodeOnRayIterator) -> float"""

    class NorthLeafNeighborIterator(PyObjectOccupancyQuadtreeF.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class SouthLeafNeighborIterator(PyObjectOccupancyQuadtreeF.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class TreeInAabbIterator(PyObjectOccupancyQuadtreeF.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class TreeIterator(PyObjectOccupancyQuadtreeF.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class WestLeafNeighborIterator(PyObjectOccupancyQuadtreeF.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""
    @overload
    def __init__(self) -> None:
        """__init__(*args, **kwargs)
        Overloaded function.

        1. __init__(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF) -> None

        2. __init__(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, setting: erl_geometry.pyerl_geometry.OccupancyQuadtreeBaseSetting) -> None

        3. __init__(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, filename: str) -> None
        """
    @overload
    def __init__(self, setting: OccupancyQuadtreeBaseSetting) -> None:
        """__init__(*args, **kwargs)
        Overloaded function.

        1. __init__(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF) -> None

        2. __init__(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, setting: erl_geometry.pyerl_geometry.OccupancyQuadtreeBaseSetting) -> None

        3. __init__(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, filename: str) -> None
        """
    @overload
    def __init__(self, filename: str) -> None:
        """__init__(*args, **kwargs)
        Overloaded function.

        1. __init__(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF) -> None

        2. __init__(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, setting: erl_geometry.pyerl_geometry.OccupancyQuadtreeBaseSetting) -> None

        3. __init__(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, filename: str) -> None
        """
    def Setting(self) -> OccupancyQuadtreeBaseSetting:
        """Setting() -> erl_geometry.pyerl_geometry.OccupancyQuadtreeBaseSetting"""
    @overload
    def adjust_key_to_depth(self, key: int, depth: int) -> int:
        """adjust_key_to_depth(*args, **kwargs)
        Overloaded function.

        1. adjust_key_to_depth(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, key: int, depth: int) -> int

        2. adjust_key_to_depth(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> erl_geometry.pyerl_geometry.QuadtreeKey
        """
    @overload
    def adjust_key_to_depth(self, key: QuadtreeKey, depth: int) -> QuadtreeKey:
        """adjust_key_to_depth(*args, **kwargs)
        Overloaded function.

        1. adjust_key_to_depth(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, key: int, depth: int) -> int

        2. adjust_key_to_depth(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> erl_geometry.pyerl_geometry.QuadtreeKey
        """
    def cast_ray(self, px: float, py: float, vx: float, vy: float, ignore_unknown: bool, max_range: float) -> dict:
        """cast_ray(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, px: float, py: float, vx: float, vy: float, ignore_unknown: bool, max_range: float) -> dict"""
    @overload
    def cast_rays(self, position: numpy.ndarray[numpy.float32[2, 1]], rotation: numpy.ndarray[numpy.float32[2, 2], flags.f_contiguous], angles: numpy.ndarray[numpy.float32[m, 1]], ignore_unknown: bool, max_range: float, prune_rays: bool, parallel: bool) -> dict:
        """cast_rays(*args, **kwargs)
        Overloaded function.

        1. cast_rays(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, position: numpy.ndarray[numpy.float32[2, 1]], rotation: numpy.ndarray[numpy.float32[2, 2], flags.f_contiguous], angles: numpy.ndarray[numpy.float32[m, 1]], ignore_unknown: bool, max_range: float, prune_rays: bool, parallel: bool) -> dict

        2. cast_rays(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, positions: numpy.ndarray[numpy.float32[2, n], flags.f_contiguous], directions: numpy.ndarray[numpy.float32[2, n], flags.f_contiguous], ignore_unknown: bool, max_range: float, prune_rays: bool, parallel: bool) -> dict
        """
    @overload
    def cast_rays(self, positions: numpy.ndarray[numpy.float32[2, n], flags.f_contiguous], directions: numpy.ndarray[numpy.float32[2, n], flags.f_contiguous], ignore_unknown: bool, max_range: float, prune_rays: bool, parallel: bool) -> dict:
        """cast_rays(*args, **kwargs)
        Overloaded function.

        1. cast_rays(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, position: numpy.ndarray[numpy.float32[2, 1]], rotation: numpy.ndarray[numpy.float32[2, 2], flags.f_contiguous], angles: numpy.ndarray[numpy.float32[m, 1]], ignore_unknown: bool, max_range: float, prune_rays: bool, parallel: bool) -> dict

        2. cast_rays(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, positions: numpy.ndarray[numpy.float32[2, n], flags.f_contiguous], directions: numpy.ndarray[numpy.float32[2, n], flags.f_contiguous], ignore_unknown: bool, max_range: float, prune_rays: bool, parallel: bool) -> dict
        """
    def clear(self) -> None:
        """clear(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF) -> None"""
    def compute_common_ancestor_key(self, arg0: QuadtreeKey, arg1: QuadtreeKey) -> tuple[QuadtreeKey, int]:
        """compute_common_ancestor_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, arg0: erl_geometry.pyerl_geometry.QuadtreeKey, arg1: erl_geometry.pyerl_geometry.QuadtreeKey) -> tuple[erl_geometry.pyerl_geometry.QuadtreeKey, int]"""
    def compute_east_neighbor_key(self, key: QuadtreeKey, depth: int) -> QuadtreeKey | None:
        """compute_east_neighbor_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]"""
    def compute_north_neighbor_key(self, key: QuadtreeKey, depth: int) -> QuadtreeKey | None:
        """compute_north_neighbor_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]"""
    def compute_ray_coords(self, sx: float, sy: float, ex: float, ey: float) -> list[numpy.ndarray[numpy.float32[2, 1]]] | None:
        """compute_ray_coords(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, sx: float, sy: float, ex: float, ey: float) -> Optional[list[numpy.ndarray[numpy.float32[2, 1]]]]"""
    def compute_ray_keys(self, sx: float, sy: float, ex: float, ey: float) -> list[QuadtreeKey] | None:
        """compute_ray_keys(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, sx: float, sy: float, ex: float, ey: float) -> Optional[list[erl_geometry.pyerl_geometry.QuadtreeKey]]"""
    def compute_south_neighbor_key(self, key: QuadtreeKey, depth: int) -> QuadtreeKey | None:
        """compute_south_neighbor_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]"""
    def compute_west_neighbor_key(self, key: QuadtreeKey, depth: int) -> QuadtreeKey | None:
        """compute_west_neighbor_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]"""
    @overload
    def coord_to_key(self, coordinate: float) -> int:
        """coord_to_key(*args, **kwargs)
        Overloaded function.

        1. coord_to_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, coordinate: float) -> int

        2. coord_to_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, coordinate: float, depth: int) -> int

        3. coord_to_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, x: float, y: float) -> erl_geometry.pyerl_geometry.QuadtreeKey

        4. coord_to_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, x: float, y: float, depth: int) -> erl_geometry.pyerl_geometry.QuadtreeKey
        """
    @overload
    def coord_to_key(self, coordinate: float, depth: int) -> int:
        """coord_to_key(*args, **kwargs)
        Overloaded function.

        1. coord_to_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, coordinate: float) -> int

        2. coord_to_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, coordinate: float, depth: int) -> int

        3. coord_to_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, x: float, y: float) -> erl_geometry.pyerl_geometry.QuadtreeKey

        4. coord_to_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, x: float, y: float, depth: int) -> erl_geometry.pyerl_geometry.QuadtreeKey
        """
    @overload
    def coord_to_key(self, x: float, y: float) -> QuadtreeKey:
        """coord_to_key(*args, **kwargs)
        Overloaded function.

        1. coord_to_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, coordinate: float) -> int

        2. coord_to_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, coordinate: float, depth: int) -> int

        3. coord_to_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, x: float, y: float) -> erl_geometry.pyerl_geometry.QuadtreeKey

        4. coord_to_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, x: float, y: float, depth: int) -> erl_geometry.pyerl_geometry.QuadtreeKey
        """
    @overload
    def coord_to_key(self, x: float, y: float, depth: int) -> QuadtreeKey:
        """coord_to_key(*args, **kwargs)
        Overloaded function.

        1. coord_to_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, coordinate: float) -> int

        2. coord_to_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, coordinate: float, depth: int) -> int

        3. coord_to_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, x: float, y: float) -> erl_geometry.pyerl_geometry.QuadtreeKey

        4. coord_to_key(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, x: float, y: float, depth: int) -> erl_geometry.pyerl_geometry.QuadtreeKey
        """
    @overload
    def coord_to_key_checked(self, coordinate: float) -> int | None:
        """coord_to_key_checked(*args, **kwargs)
        Overloaded function.

        1. coord_to_key_checked(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, coordinate: float) -> Optional[int]

        2. coord_to_key_checked(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, coordinate: float, depth: int) -> Optional[int]

        3. coord_to_key_checked(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, x: float, y: float) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]

        4. coord_to_key_checked(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, x: float, y: float, depth: int) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]
        """
    @overload
    def coord_to_key_checked(self, coordinate: float, depth: int) -> int | None:
        """coord_to_key_checked(*args, **kwargs)
        Overloaded function.

        1. coord_to_key_checked(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, coordinate: float) -> Optional[int]

        2. coord_to_key_checked(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, coordinate: float, depth: int) -> Optional[int]

        3. coord_to_key_checked(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, x: float, y: float) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]

        4. coord_to_key_checked(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, x: float, y: float, depth: int) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]
        """
    @overload
    def coord_to_key_checked(self, x: float, y: float) -> QuadtreeKey | None:
        """coord_to_key_checked(*args, **kwargs)
        Overloaded function.

        1. coord_to_key_checked(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, coordinate: float) -> Optional[int]

        2. coord_to_key_checked(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, coordinate: float, depth: int) -> Optional[int]

        3. coord_to_key_checked(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, x: float, y: float) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]

        4. coord_to_key_checked(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, x: float, y: float, depth: int) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]
        """
    @overload
    def coord_to_key_checked(self, x: float, y: float, depth: int) -> QuadtreeKey | None:
        """coord_to_key_checked(*args, **kwargs)
        Overloaded function.

        1. coord_to_key_checked(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, coordinate: float) -> Optional[int]

        2. coord_to_key_checked(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, coordinate: float, depth: int) -> Optional[int]

        3. coord_to_key_checked(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, x: float, y: float) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]

        4. coord_to_key_checked(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, x: float, y: float, depth: int) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]
        """
    def create_node_child(self, node: PyObjectOccupancyQuadtreeNode, child_idx: int) -> PyObjectOccupancyQuadtreeNode:
        """create_node_child(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, node: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode, child_idx: int) -> erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode"""
    @overload
    def delete_node(self, x: float, y: float, depth: int) -> int:
        """delete_node(*args, **kwargs)
        Overloaded function.

        1. delete_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, x: float, y: float, depth: int) -> int

        2. delete_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> None
        """
    @overload
    def delete_node(self, key: QuadtreeKey, depth: int) -> None:
        """delete_node(*args, **kwargs)
        Overloaded function.

        1. delete_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, x: float, y: float, depth: int) -> int

        2. delete_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> None
        """
    def delete_node_child(self, node: PyObjectOccupancyQuadtreeNode, child_idx: int, key: QuadtreeKey) -> int:
        """delete_node_child(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, node: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode, child_idx: int, key: erl_geometry.pyerl_geometry.QuadtreeKey) -> int"""
    def expand(self) -> None:
        """expand(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF) -> None"""
    def expand_node(self, node: PyObjectOccupancyQuadtreeNode) -> None:
        """expand_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, node: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode) -> None"""
    def get_batch_ray_caster(self, *args, **kwargs):
        """get_batch_ray_caster(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, origins: numpy.ndarray[numpy.float32[2, n]], directions: numpy.ndarray[numpy.float32[2, n]], max_ranges: numpy.ndarray[numpy.float32[m, 1]] = array([], dtype=float32), node_paddings: numpy.ndarray[numpy.float32[m, 1]] = array([], dtype=float32), bidirectional_flags: numpy.ndarray[bool[m, 1]] = array([], dtype=bool), leaf_only_flags: numpy.ndarray[bool[m, 1]] = array([], dtype=bool), min_node_depths: numpy.ndarray[numpy.int32[m, 1]] = array([], dtype=int32), max_node_depths: numpy.ndarray[numpy.int32[m, 1]] = array([], dtype=int32)) -> erl::geometry::OccupancyNdTreeBatchRayCaster<erl::geometry::OccupancyQuadtreeBase<float, erl::geometry::PyObjectOccupancyQuadtreeNode, erl::geometry::OccupancyQuadtreeBaseSetting>, 2>"""
    def get_node_child(self, node: PyObjectOccupancyQuadtreeNode, child_idx: int) -> PyObjectOccupancyQuadtreeNode:
        """get_node_child(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, node: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode, child_idx: int) -> erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode"""
    def get_node_size(self, depth: int) -> float:
        """get_node_size(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, depth: int) -> float"""
    @overload
    def insert_node(self, x: float, y: float, depth: int) -> PyObjectOccupancyQuadtreeNode:
        """insert_node(*args, **kwargs)
        Overloaded function.

        1. insert_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, x: float, y: float, depth: int) -> erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode

        2. insert_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode
        """
    @overload
    def insert_node(self, key: QuadtreeKey, depth: int) -> PyObjectOccupancyQuadtreeNode:
        """insert_node(*args, **kwargs)
        Overloaded function.

        1. insert_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, x: float, y: float, depth: int) -> erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode

        2. insert_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode
        """
    def insert_point_cloud(self, points: numpy.ndarray[numpy.float32[2, n], flags.f_contiguous], sensor_origin: numpy.ndarray[numpy.float32[2, 1]], min_range: float, max_range: float, with_count: bool, parallel: bool, lazy_eval: bool, discrete: bool) -> None:
        """insert_point_cloud(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, points: numpy.ndarray[numpy.float32[2, n], flags.f_contiguous], sensor_origin: numpy.ndarray[numpy.float32[2, 1]], min_range: float, max_range: float, with_count: bool, parallel: bool, lazy_eval: bool, discrete: bool) -> None"""
    def insert_point_cloud_rays(self, points: numpy.ndarray[numpy.float32[2, n], flags.f_contiguous], sensor_origin: numpy.ndarray[numpy.float32[2, 1]], min_range: float, max_range: float, parallel: bool, lazy_eval: bool) -> None:
        """insert_point_cloud_rays(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, points: numpy.ndarray[numpy.float32[2, n], flags.f_contiguous], sensor_origin: numpy.ndarray[numpy.float32[2, 1]], min_range: float, max_range: float, parallel: bool, lazy_eval: bool) -> None"""
    def insert_ray(self, sx: float, sy: float, ex: float, ey: float, min_range: float, max_range: float, lazy_eval: bool) -> bool:
        """insert_ray(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, sx: float, sy: float, ex: float, ey: float, min_range: float, max_range: float, lazy_eval: bool) -> bool"""
    def is_node_collapsible(self, node: PyObjectOccupancyQuadtreeNode) -> bool:
        """is_node_collapsible(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, node: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode) -> bool"""
    @overload
    def iter_east_leaf_neighbor(self, x: float, y: float, max_leaf_depth: int = ...) -> Iterator[PyObjectOccupancyQuadtreeF.EastLeafNeighborIterator]:
        """iter_east_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_east_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, x: float, y: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.EastLeafNeighborIterator]

        2. iter_east_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.EastLeafNeighborIterator]
        """
    @overload
    def iter_east_leaf_neighbor(self, key: QuadtreeKey, key_depth: int, max_leaf_depth: int = ...) -> Iterator[PyObjectOccupancyQuadtreeF.EastLeafNeighborIterator]:
        """iter_east_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_east_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, x: float, y: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.EastLeafNeighborIterator]

        2. iter_east_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.EastLeafNeighborIterator]
        """
    def iter_leaf(self, max_depth: int = ...) -> Iterator[PyObjectOccupancyQuadtreeF.LeafIterator]:
        """iter_leaf(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.LeafIterator]"""
    @overload
    def iter_leaf_in_aabb(self, aabb_min_x: float, aabb_min_y: float, aabb_max_x: float, aabb_max_y: float, max_depth: int = ...) -> Iterator[PyObjectOccupancyQuadtreeF.LeafInAabbIterator]:
        """iter_leaf_in_aabb(*args, **kwargs)
        Overloaded function.

        1. iter_leaf_in_aabb(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, aabb_min_x: float, aabb_min_y: float, aabb_max_x: float, aabb_max_y: float, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.LeafInAabbIterator]

        2. iter_leaf_in_aabb(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, aabb_min_key: erl_geometry.pyerl_geometry.QuadtreeKey, aabb_max_key: erl_geometry.pyerl_geometry.QuadtreeKey, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.LeafInAabbIterator]
        """
    @overload
    def iter_leaf_in_aabb(self, aabb_min_key: QuadtreeKey, aabb_max_key: QuadtreeKey, max_depth: int = ...) -> Iterator[PyObjectOccupancyQuadtreeF.LeafInAabbIterator]:
        """iter_leaf_in_aabb(*args, **kwargs)
        Overloaded function.

        1. iter_leaf_in_aabb(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, aabb_min_x: float, aabb_min_y: float, aabb_max_x: float, aabb_max_y: float, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.LeafInAabbIterator]

        2. iter_leaf_in_aabb(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, aabb_min_key: erl_geometry.pyerl_geometry.QuadtreeKey, aabb_max_key: erl_geometry.pyerl_geometry.QuadtreeKey, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.LeafInAabbIterator]
        """
    def iter_leaf_of_node(self, node_key: QuadtreeKey, node_depth: int, max_depth: int = ...) -> Iterator[PyObjectOccupancyQuadtreeF.LeafOfNodeIterator]:
        """iter_leaf_of_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, node_key: erl_geometry.pyerl_geometry.QuadtreeKey, node_depth: int, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.LeafOfNodeIterator]"""
    def iter_node(self, max_depth: int = ...) -> Iterator[PyObjectOccupancyQuadtreeF.TreeIterator]:
        """iter_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.TreeIterator]"""
    @overload
    def iter_node_in_aabb(self, aabb_min_x: float, aabb_min_y: float, aabb_max_x: float, aabb_max_y: float, max_depth: int = ...) -> Iterator[PyObjectOccupancyQuadtreeF.TreeInAabbIterator]:
        """iter_node_in_aabb(*args, **kwargs)
        Overloaded function.

        1. iter_node_in_aabb(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, aabb_min_x: float, aabb_min_y: float, aabb_max_x: float, aabb_max_y: float, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.TreeInAabbIterator]

        2. iter_node_in_aabb(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, aabb_min_key: erl_geometry.pyerl_geometry.QuadtreeKey, aabb_max_key: erl_geometry.pyerl_geometry.QuadtreeKey, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.TreeInAabbIterator]
        """
    @overload
    def iter_node_in_aabb(self, aabb_min_key: QuadtreeKey, aabb_max_key: QuadtreeKey, max_depth: int = ...) -> Iterator[PyObjectOccupancyQuadtreeF.TreeInAabbIterator]:
        """iter_node_in_aabb(*args, **kwargs)
        Overloaded function.

        1. iter_node_in_aabb(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, aabb_min_x: float, aabb_min_y: float, aabb_max_x: float, aabb_max_y: float, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.TreeInAabbIterator]

        2. iter_node_in_aabb(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, aabb_min_key: erl_geometry.pyerl_geometry.QuadtreeKey, aabb_max_key: erl_geometry.pyerl_geometry.QuadtreeKey, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.TreeInAabbIterator]
        """
    def iter_node_on_ray(self, px: float, py: float, vx: float, vy: float, max_range: float = ..., node_padding: float = ..., bidirectional: bool = ..., leaf_only: bool = ..., min_node_depth: int = ..., max_node_depth: int = ...) -> Iterator[PyObjectOccupancyQuadtreeF.NodeOnRayIterator]:
        """iter_node_on_ray(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, px: float, py: float, vx: float, vy: float, max_range: float = -1, node_padding: float = 0, bidirectional: bool = False, leaf_only: bool = False, min_node_depth: int = 0, max_node_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.NodeOnRayIterator]"""
    @overload
    def iter_north_leaf_neighbor(self, x: float, y: float, max_leaf_depth: int = ...) -> Iterator[PyObjectOccupancyQuadtreeF.NorthLeafNeighborIterator]:
        """iter_north_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_north_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, x: float, y: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.NorthLeafNeighborIterator]

        2. iter_north_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.NorthLeafNeighborIterator]
        """
    @overload
    def iter_north_leaf_neighbor(self, key: QuadtreeKey, key_depth: int, max_leaf_depth: int = ...) -> Iterator[PyObjectOccupancyQuadtreeF.NorthLeafNeighborIterator]:
        """iter_north_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_north_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, x: float, y: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.NorthLeafNeighborIterator]

        2. iter_north_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.NorthLeafNeighborIterator]
        """
    @overload
    def iter_south_leaf_neighbor(self, x: float, y: float, max_leaf_depth: int = ...) -> Iterator[PyObjectOccupancyQuadtreeF.SouthLeafNeighborIterator]:
        """iter_south_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_south_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, x: float, y: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.SouthLeafNeighborIterator]

        2. iter_south_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.SouthLeafNeighborIterator]
        """
    @overload
    def iter_south_leaf_neighbor(self, key: QuadtreeKey, key_depth: int, max_leaf_depth: int = ...) -> Iterator[PyObjectOccupancyQuadtreeF.SouthLeafNeighborIterator]:
        """iter_south_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_south_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, x: float, y: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.SouthLeafNeighborIterator]

        2. iter_south_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.SouthLeafNeighborIterator]
        """
    @overload
    def iter_west_leaf_neighbor(self, x: float, y: float, max_leaf_depth: int = ...) -> Iterator[PyObjectOccupancyQuadtreeF.WestLeafNeighborIterator]:
        """iter_west_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_west_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, x: float, y: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.WestLeafNeighborIterator]

        2. iter_west_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.WestLeafNeighborIterator]
        """
    @overload
    def iter_west_leaf_neighbor(self, key: QuadtreeKey, key_depth: int, max_leaf_depth: int = ...) -> Iterator[PyObjectOccupancyQuadtreeF.WestLeafNeighborIterator]:
        """iter_west_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_west_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, x: float, y: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.WestLeafNeighborIterator]

        2. iter_west_leaf_neighbor(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF.WestLeafNeighborIterator]
        """
    @overload
    def key_to_coord(self, key: int) -> float:
        """key_to_coord(*args, **kwargs)
        Overloaded function.

        1. key_to_coord(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, key: int) -> float

        2. key_to_coord(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, key: int, depth: int) -> float

        3. key_to_coord(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey) -> tuple[float, float]

        4. key_to_coord(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> tuple[float, float]
        """
    @overload
    def key_to_coord(self, key: int, depth: int) -> float:
        """key_to_coord(*args, **kwargs)
        Overloaded function.

        1. key_to_coord(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, key: int) -> float

        2. key_to_coord(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, key: int, depth: int) -> float

        3. key_to_coord(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey) -> tuple[float, float]

        4. key_to_coord(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> tuple[float, float]
        """
    @overload
    def key_to_coord(self, key: QuadtreeKey) -> tuple[float, float]:
        """key_to_coord(*args, **kwargs)
        Overloaded function.

        1. key_to_coord(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, key: int) -> float

        2. key_to_coord(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, key: int, depth: int) -> float

        3. key_to_coord(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey) -> tuple[float, float]

        4. key_to_coord(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> tuple[float, float]
        """
    @overload
    def key_to_coord(self, key: QuadtreeKey, depth: int) -> tuple[float, float]:
        """key_to_coord(*args, **kwargs)
        Overloaded function.

        1. key_to_coord(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, key: int) -> float

        2. key_to_coord(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, key: int, depth: int) -> float

        3. key_to_coord(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey) -> tuple[float, float]

        4. key_to_coord(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> tuple[float, float]
        """
    def prune(self) -> None:
        """prune(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF) -> None"""
    def prune_node(self, node: PyObjectOccupancyQuadtreeNode) -> bool:
        """prune_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, node: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode) -> bool"""
    def sample_positions(self, num_positions: int) -> list[numpy.ndarray[numpy.float32[2, 1]]]:
        """sample_positions(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, num_positions: int) -> list[numpy.ndarray[numpy.float32[2, 1]]]"""
    @overload
    def search(self, x: float, y: float, max_depth: int = ...) -> PyObjectOccupancyQuadtreeNode:
        """search(*args, **kwargs)
        Overloaded function.

        1. search(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, x: float, y: float, max_depth: int = 0) -> erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode

        2. search(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, max_depth: int = 0) -> erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode
        """
    @overload
    def search(self, key: QuadtreeKey, max_depth: int = ...) -> PyObjectOccupancyQuadtreeNode:
        """search(*args, **kwargs)
        Overloaded function.

        1. search(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, x: float, y: float, max_depth: int = 0) -> erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode

        2. search(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, max_depth: int = 0) -> erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode
        """
    def to_max_likelihood(self) -> None:
        """to_max_likelihood(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF) -> None"""
    def update_inner_occupancy(self) -> None:
        """update_inner_occupancy(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF) -> None"""
    @overload
    def update_node(self, x: float, y: float, occupied: bool, lazy_eval: bool) -> PyObjectOccupancyQuadtreeNode:
        """update_node(*args, **kwargs)
        Overloaded function.

        1. update_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, x: float, y: float, occupied: bool, lazy_eval: bool) -> erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode

        2. update_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, node_key: erl_geometry.pyerl_geometry.QuadtreeKey, occupied: bool, lazy_eval: bool) -> erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode

        3. update_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, x: float, y: float, log_odds_delta: float, lazy_eval: bool) -> erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode

        4. update_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, node_key: erl_geometry.pyerl_geometry.QuadtreeKey, log_odds_delta: float, lazy_eval: bool) -> erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode
        """
    @overload
    def update_node(self, node_key: QuadtreeKey, occupied: bool, lazy_eval: bool) -> PyObjectOccupancyQuadtreeNode:
        """update_node(*args, **kwargs)
        Overloaded function.

        1. update_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, x: float, y: float, occupied: bool, lazy_eval: bool) -> erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode

        2. update_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, node_key: erl_geometry.pyerl_geometry.QuadtreeKey, occupied: bool, lazy_eval: bool) -> erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode

        3. update_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, x: float, y: float, log_odds_delta: float, lazy_eval: bool) -> erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode

        4. update_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, node_key: erl_geometry.pyerl_geometry.QuadtreeKey, log_odds_delta: float, lazy_eval: bool) -> erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode
        """
    @overload
    def update_node(self, x: float, y: float, log_odds_delta: float, lazy_eval: bool) -> PyObjectOccupancyQuadtreeNode:
        """update_node(*args, **kwargs)
        Overloaded function.

        1. update_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, x: float, y: float, occupied: bool, lazy_eval: bool) -> erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode

        2. update_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, node_key: erl_geometry.pyerl_geometry.QuadtreeKey, occupied: bool, lazy_eval: bool) -> erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode

        3. update_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, x: float, y: float, log_odds_delta: float, lazy_eval: bool) -> erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode

        4. update_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, node_key: erl_geometry.pyerl_geometry.QuadtreeKey, log_odds_delta: float, lazy_eval: bool) -> erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode
        """
    @overload
    def update_node(self, node_key: QuadtreeKey, log_odds_delta: float, lazy_eval: bool) -> PyObjectOccupancyQuadtreeNode:
        """update_node(*args, **kwargs)
        Overloaded function.

        1. update_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, x: float, y: float, occupied: bool, lazy_eval: bool) -> erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode

        2. update_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, node_key: erl_geometry.pyerl_geometry.QuadtreeKey, occupied: bool, lazy_eval: bool) -> erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode

        3. update_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, x: float, y: float, log_odds_delta: float, lazy_eval: bool) -> erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode

        4. update_node(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, node_key: erl_geometry.pyerl_geometry.QuadtreeKey, log_odds_delta: float, lazy_eval: bool) -> erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode
        """
    def visualize(self, leaf_only: bool = ..., area_min: numpy.ndarray[numpy.float32[2, 1]] | None = ..., area_max: numpy.ndarray[numpy.float32[2, 1]] | None = ..., resolution: float = ..., padding: int = ..., bg_color: numpy.ndarray[numpy.int32[4, 1]] = ..., fg_color: numpy.ndarray[numpy.int32[4, 1]] = ..., occupied_color: numpy.ndarray[numpy.int32[4, 1]] = ..., free_color: numpy.ndarray[numpy.int32[4, 1]] = ..., border_color: numpy.ndarray[numpy.int32[4, 1]] = ..., border_thickness: int = ...) -> numpy.ndarray[numpy.uint8[m, n]]:
        """visualize(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF, leaf_only: bool = False, area_min: Optional[numpy.ndarray[numpy.float32[2, 1]]] = None, area_max: Optional[numpy.ndarray[numpy.float32[2, 1]]] = None, resolution: float = 0.1, padding: int = 1, bg_color: numpy.ndarray[numpy.int32[4, 1]] = array([128, 128, 128, 255], dtype=int32), fg_color: numpy.ndarray[numpy.int32[4, 1]] = array([255, 255, 255, 255], dtype=int32), occupied_color: numpy.ndarray[numpy.int32[4, 1]] = array([ 0, 0, 0, 255], dtype=int32), free_color: numpy.ndarray[numpy.int32[4, 1]] = array([255, 255, 255, 255], dtype=int32), border_color: numpy.ndarray[numpy.int32[4, 1]] = array([ 0, 0, 0, 255], dtype=int32), border_thickness: int = 1) -> numpy.ndarray[numpy.uint8[m, n]]"""
    @property
    def memory_usage(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF) -> int"""
    @property
    def memory_usage_per_node(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF) -> int"""
    @property
    def metric_aabb(self) -> Aabb2Df:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF) -> erl_geometry.pyerl_geometry.Aabb2Df"""
    @property
    def metric_max(self) -> numpy.ndarray[numpy.float32[2, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF) -> numpy.ndarray[numpy.float32[2, 1]]"""
    @property
    def metric_min(self) -> numpy.ndarray[numpy.float32[2, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF) -> numpy.ndarray[numpy.float32[2, 1]]"""
    @property
    def metric_min_max(self) -> tuple[numpy.ndarray[numpy.float32[2, 1]], numpy.ndarray[numpy.float32[2, 1]]]:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF) -> tuple[numpy.ndarray[numpy.float32[2, 1]], numpy.ndarray[numpy.float32[2, 1]]]"""
    @property
    def metric_size(self) -> numpy.ndarray[numpy.float32[2, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF) -> numpy.ndarray[numpy.float32[2, 1]]"""
    @property
    def number_of_leaf_nodes(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF) -> int"""
    @property
    def number_of_nodes(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF) -> int"""
    @property
    def resolution(self) -> float:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF) -> float"""
    @property
    def root(self) -> PyObjectOccupancyQuadtreeNode:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF) -> erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode"""
    @property
    def setting(self) -> OccupancyQuadtreeBaseSetting:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF) -> erl_geometry.pyerl_geometry.OccupancyQuadtreeBaseSetting"""
    @property
    def tree_center(self) -> numpy.ndarray[numpy.float32[2, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF) -> numpy.ndarray[numpy.float32[2, 1]]"""
    @property
    def tree_center_key(self) -> QuadtreeKey:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF) -> erl_geometry.pyerl_geometry.QuadtreeKey"""
    @property
    def tree_depth(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF) -> int"""
    @property
    def tree_max_half_size(self) -> numpy.ndarray[numpy.float32[2, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeF) -> numpy.ndarray[numpy.float32[2, 1]]"""

class PyObjectOccupancyQuadtreeNode(AbstractQuadtreeNode):
    py_object: object
    def __init__(self, *args, **kwargs) -> None:
        """Initialize self.  See help(type(self)) for accurate signature."""
    def add_log_odds(self, log_odds: float) -> None:
        """add_log_odds(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode, log_odds: float) -> None"""
    def allow_update_log_odds(self, delta: float) -> bool:
        """allow_update_log_odds(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode, delta: float) -> bool"""
    def get_child(self, child_idx: int) -> PyObjectOccupancyQuadtreeNode:
        """get_child(self: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode, child_idx: int) -> erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode"""
    @property
    def log_odds(self) -> float:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode) -> float"""
    @property
    def max_child_log_odds(self) -> float:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode) -> float"""
    @property
    def mean_child_log_odds(self) -> float:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode) -> float"""
    @property
    def occupancy(self) -> float:
        """(arg0: erl_geometry.pyerl_geometry.PyObjectOccupancyQuadtreeNode) -> float"""

class QuadtreeKey:
    @overload
    def __init__(self) -> None:
        """__init__(*args, **kwargs)
        Overloaded function.

        1. __init__(self: erl_geometry.pyerl_geometry.QuadtreeKey) -> None

        2. __init__(self: erl_geometry.pyerl_geometry.QuadtreeKey, a: int, b: int) -> None
        """
    @overload
    def __init__(self, a: int, b: int) -> None:
        """__init__(*args, **kwargs)
        Overloaded function.

        1. __init__(self: erl_geometry.pyerl_geometry.QuadtreeKey) -> None

        2. __init__(self: erl_geometry.pyerl_geometry.QuadtreeKey, a: int, b: int) -> None
        """
    def to_list(self) -> list:
        """to_list(self: erl_geometry.pyerl_geometry.QuadtreeKey) -> list"""
    def __eq__(self, arg0: QuadtreeKey) -> bool:
        """__eq__(self: erl_geometry.pyerl_geometry.QuadtreeKey, arg0: erl_geometry.pyerl_geometry.QuadtreeKey) -> bool"""
    def __getitem__(self, arg0: int) -> int:
        """__getitem__(self: erl_geometry.pyerl_geometry.QuadtreeKey, arg0: int) -> int"""
    def __iter__(self) -> typing.Iterator[int]:
        """def __iter__(self) -> typing.Iterator[int]"""
    def __hash__(self) -> int:
        """__hash__(self: erl_geometry.pyerl_geometry.QuadtreeKey) -> int"""
    def __ne__(self, arg0: QuadtreeKey) -> bool:
        """__ne__(self: erl_geometry.pyerl_geometry.QuadtreeKey, arg0: erl_geometry.pyerl_geometry.QuadtreeKey) -> bool"""

class RangeSensor3Dd:
    def __init__(self, *args, **kwargs) -> None:
        """Initialize self.  See help(type(self)) for accurate signature."""
    @overload
    def add_mesh(self, mesh_path: str) -> None:
        """add_mesh(*args, **kwargs)
        Overloaded function.

        1. add_mesh(self: erl_geometry.pyerl_geometry.RangeSensor3Dd, mesh_path: str) -> None

        2. add_mesh(self: erl_geometry.pyerl_geometry.RangeSensor3Dd, vertices: numpy.ndarray[numpy.float64[3, n], flags.f_contiguous], triangles: numpy.ndarray[numpy.int32[3, n], flags.f_contiguous]) -> None

        3. add_mesh(self: erl_geometry.pyerl_geometry.RangeSensor3Dd, vertices: list[numpy.ndarray[numpy.float64[3, 1]]], triangles: list[numpy.ndarray[numpy.int32[3, 1]]]) -> None
        """
    @overload
    def add_mesh(self, vertices: numpy.ndarray[numpy.float64[3, n], flags.f_contiguous], triangles: numpy.ndarray[numpy.int32[3, n], flags.f_contiguous]) -> None:
        """add_mesh(*args, **kwargs)
        Overloaded function.

        1. add_mesh(self: erl_geometry.pyerl_geometry.RangeSensor3Dd, mesh_path: str) -> None

        2. add_mesh(self: erl_geometry.pyerl_geometry.RangeSensor3Dd, vertices: numpy.ndarray[numpy.float64[3, n], flags.f_contiguous], triangles: numpy.ndarray[numpy.int32[3, n], flags.f_contiguous]) -> None

        3. add_mesh(self: erl_geometry.pyerl_geometry.RangeSensor3Dd, vertices: list[numpy.ndarray[numpy.float64[3, 1]]], triangles: list[numpy.ndarray[numpy.int32[3, 1]]]) -> None
        """
    @overload
    def add_mesh(self, vertices: list[numpy.ndarray[numpy.float64[3, 1]]], triangles: list[numpy.ndarray[numpy.int32[3, 1]]]) -> None:
        """add_mesh(*args, **kwargs)
        Overloaded function.

        1. add_mesh(self: erl_geometry.pyerl_geometry.RangeSensor3Dd, mesh_path: str) -> None

        2. add_mesh(self: erl_geometry.pyerl_geometry.RangeSensor3Dd, vertices: numpy.ndarray[numpy.float64[3, n], flags.f_contiguous], triangles: numpy.ndarray[numpy.int32[3, n], flags.f_contiguous]) -> None

        3. add_mesh(self: erl_geometry.pyerl_geometry.RangeSensor3Dd, vertices: list[numpy.ndarray[numpy.float64[3, 1]]], triangles: list[numpy.ndarray[numpy.int32[3, 1]]]) -> None
        """
    def scan(self, orientation: numpy.ndarray[numpy.float64[3, 3], flags.f_contiguous], translation: numpy.ndarray[numpy.float64[3, 1]], add_noise: bool = ..., noise_stddev: float = ..., cache_normals: bool = ...) -> numpy.ndarray[numpy.float64[m, n]]:
        """scan(self: erl_geometry.pyerl_geometry.RangeSensor3Dd, orientation: numpy.ndarray[numpy.float64[3, 3], flags.f_contiguous], translation: numpy.ndarray[numpy.float64[3, 1]], add_noise: bool = False, noise_stddev: float = 0.03, cache_normals: bool = False) -> numpy.ndarray[numpy.float64[m, n]]"""
    @property
    def cached_normals(self) -> numpy.ndarray[numpy.float64]:
        """(arg0: erl_geometry.pyerl_geometry.RangeSensor3Dd) -> numpy.ndarray[numpy.float64]"""
    @property
    def optical_pose(self) -> tuple[numpy.ndarray[numpy.float64[3, 3]], numpy.ndarray[numpy.float64[3, 1]]]:
        """(arg0: erl_geometry.pyerl_geometry.RangeSensor3Dd, arg1: numpy.ndarray[numpy.float64[3, 3], flags.f_contiguous], arg2: numpy.ndarray[numpy.float64[3, 1]]) -> tuple[numpy.ndarray[numpy.float64[3, 3]], numpy.ndarray[numpy.float64[3, 1]]]"""
    @property
    def ray_directions_in_frame(self) -> numpy.ndarray[numpy.float64]:
        """(arg0: erl_geometry.pyerl_geometry.RangeSensor3Dd) -> numpy.ndarray[numpy.float64]"""

class RangeSensor3Df:
    def __init__(self, *args, **kwargs) -> None:
        """Initialize self.  See help(type(self)) for accurate signature."""
    @overload
    def add_mesh(self, mesh_path: str) -> None:
        """add_mesh(*args, **kwargs)
        Overloaded function.

        1. add_mesh(self: erl_geometry.pyerl_geometry.RangeSensor3Df, mesh_path: str) -> None

        2. add_mesh(self: erl_geometry.pyerl_geometry.RangeSensor3Df, vertices: numpy.ndarray[numpy.float32[3, n], flags.f_contiguous], triangles: numpy.ndarray[numpy.int32[3, n], flags.f_contiguous]) -> None

        3. add_mesh(self: erl_geometry.pyerl_geometry.RangeSensor3Df, vertices: list[numpy.ndarray[numpy.float32[3, 1]]], triangles: list[numpy.ndarray[numpy.int32[3, 1]]]) -> None
        """
    @overload
    def add_mesh(self, vertices: numpy.ndarray[numpy.float32[3, n], flags.f_contiguous], triangles: numpy.ndarray[numpy.int32[3, n], flags.f_contiguous]) -> None:
        """add_mesh(*args, **kwargs)
        Overloaded function.

        1. add_mesh(self: erl_geometry.pyerl_geometry.RangeSensor3Df, mesh_path: str) -> None

        2. add_mesh(self: erl_geometry.pyerl_geometry.RangeSensor3Df, vertices: numpy.ndarray[numpy.float32[3, n], flags.f_contiguous], triangles: numpy.ndarray[numpy.int32[3, n], flags.f_contiguous]) -> None

        3. add_mesh(self: erl_geometry.pyerl_geometry.RangeSensor3Df, vertices: list[numpy.ndarray[numpy.float32[3, 1]]], triangles: list[numpy.ndarray[numpy.int32[3, 1]]]) -> None
        """
    @overload
    def add_mesh(self, vertices: list[numpy.ndarray[numpy.float32[3, 1]]], triangles: list[numpy.ndarray[numpy.int32[3, 1]]]) -> None:
        """add_mesh(*args, **kwargs)
        Overloaded function.

        1. add_mesh(self: erl_geometry.pyerl_geometry.RangeSensor3Df, mesh_path: str) -> None

        2. add_mesh(self: erl_geometry.pyerl_geometry.RangeSensor3Df, vertices: numpy.ndarray[numpy.float32[3, n], flags.f_contiguous], triangles: numpy.ndarray[numpy.int32[3, n], flags.f_contiguous]) -> None

        3. add_mesh(self: erl_geometry.pyerl_geometry.RangeSensor3Df, vertices: list[numpy.ndarray[numpy.float32[3, 1]]], triangles: list[numpy.ndarray[numpy.int32[3, 1]]]) -> None
        """
    def scan(self, orientation: numpy.ndarray[numpy.float32[3, 3], flags.f_contiguous], translation: numpy.ndarray[numpy.float32[3, 1]], add_noise: bool = ..., noise_stddev: float = ..., cache_normals: bool = ...) -> numpy.ndarray[numpy.float32[m, n]]:
        """scan(self: erl_geometry.pyerl_geometry.RangeSensor3Df, orientation: numpy.ndarray[numpy.float32[3, 3], flags.f_contiguous], translation: numpy.ndarray[numpy.float32[3, 1]], add_noise: bool = False, noise_stddev: float = 0.03, cache_normals: bool = False) -> numpy.ndarray[numpy.float32[m, n]]"""
    @property
    def cached_normals(self) -> numpy.ndarray[numpy.float64]:
        """(arg0: erl_geometry.pyerl_geometry.RangeSensor3Df) -> numpy.ndarray[numpy.float64]"""
    @property
    def optical_pose(self) -> tuple[numpy.ndarray[numpy.float32[3, 3]], numpy.ndarray[numpy.float32[3, 1]]]:
        """(arg0: erl_geometry.pyerl_geometry.RangeSensor3Df, arg1: numpy.ndarray[numpy.float32[3, 3], flags.f_contiguous], arg2: numpy.ndarray[numpy.float32[3, 1]]) -> tuple[numpy.ndarray[numpy.float32[3, 3]], numpy.ndarray[numpy.float32[3, 1]]]"""
    @property
    def ray_directions_in_frame(self) -> numpy.ndarray[numpy.float64]:
        """(arg0: erl_geometry.pyerl_geometry.RangeSensor3Df) -> numpy.ndarray[numpy.float64]"""

class RangeSensorFrame3Dd:
    class Setting(YamlableBase):
        col_margin: int
        row_margin: int
        valid_range_max: float
        valid_range_min: float
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""
    def __init__(self, *args, **kwargs) -> None:
        """Initialize self.  See help(type(self)) for accurate signature."""
    def compute_closest_end_point(self, position_world: numpy.ndarray[numpy.float64[3, 1]], brute_force: bool = ...) -> dict:
        """compute_closest_end_point(self: erl_geometry.pyerl_geometry.RangeSensorFrame3Dd, position_world: numpy.ndarray[numpy.float64[3, 1]], brute_force: bool = False) -> dict"""
    def compute_frame_coords(self, xyz_frame: numpy.ndarray[numpy.float64[3, 1]]) -> dict:
        """compute_frame_coords(self: erl_geometry.pyerl_geometry.RangeSensorFrame3Dd, xyz_frame: numpy.ndarray[numpy.float64[3, 1]]) -> dict"""
    def compute_rays_at(self, position_world: numpy.ndarray[numpy.float64[3, 1]]) -> dict:
        """compute_rays_at(self: erl_geometry.pyerl_geometry.RangeSensorFrame3Dd, position_world: numpy.ndarray[numpy.float64[3, 1]]) -> dict"""
    def coords_is_in_frame(self, frame_coords: numpy.ndarray[numpy.float64[2, 1]]) -> bool:
        """coords_is_in_frame(self: erl_geometry.pyerl_geometry.RangeSensorFrame3Dd, frame_coords: numpy.ndarray[numpy.float64[2, 1]]) -> bool"""
    def dir_frame_to_world(self, dir_frame: numpy.ndarray[numpy.float64[3, 1]]) -> numpy.ndarray[numpy.float64[3, 1]]:
        """dir_frame_to_world(self: erl_geometry.pyerl_geometry.RangeSensorFrame3Dd, dir_frame: numpy.ndarray[numpy.float64[3, 1]]) -> numpy.ndarray[numpy.float64[3, 1]]"""
    def dir_world_to_frame(self, dir_world: numpy.ndarray[numpy.float64[3, 1]]) -> numpy.ndarray[numpy.float64[3, 1]]:
        """dir_world_to_frame(self: erl_geometry.pyerl_geometry.RangeSensorFrame3Dd, dir_world: numpy.ndarray[numpy.float64[3, 1]]) -> numpy.ndarray[numpy.float64[3, 1]]"""
    def pos_frame_to_world(self, pos_frame: numpy.ndarray[numpy.float64[3, 1]]) -> numpy.ndarray[numpy.float64[3, 1]]:
        """pos_frame_to_world(self: erl_geometry.pyerl_geometry.RangeSensorFrame3Dd, pos_frame: numpy.ndarray[numpy.float64[3, 1]]) -> numpy.ndarray[numpy.float64[3, 1]]"""
    def pos_world_to_frame(self, pos_world: numpy.ndarray[numpy.float64[3, 1]]) -> numpy.ndarray[numpy.float64[3, 1]]:
        """pos_world_to_frame(self: erl_geometry.pyerl_geometry.RangeSensorFrame3Dd, pos_world: numpy.ndarray[numpy.float64[3, 1]]) -> numpy.ndarray[numpy.float64[3, 1]]"""
    def position_is_in_frame(self, xyz_frame: numpy.ndarray[numpy.float64[3, 1]]) -> bool:
        """position_is_in_frame(self: erl_geometry.pyerl_geometry.RangeSensorFrame3Dd, xyz_frame: numpy.ndarray[numpy.float64[3, 1]]) -> bool"""
    @overload
    def sample_along_rays(self, num_samples_per_ray: int, max_in_obstacle_dist: float, sampled_rays_ratio: float) -> dict:
        """sample_along_rays(*args, **kwargs)
        Overloaded function.

        1. sample_along_rays(self: erl_geometry.pyerl_geometry.RangeSensorFrame3Dd, num_samples_per_ray: int, max_in_obstacle_dist: float, sampled_rays_ratio: float) -> dict

        2. sample_along_rays(self: erl_geometry.pyerl_geometry.RangeSensorFrame3Dd, range_step: float, max_in_obstacle_dist: float, sampled_rays_ratio: float) -> dict
        """
    @overload
    def sample_along_rays(self, range_step: float, max_in_obstacle_dist: float, sampled_rays_ratio: float) -> dict:
        """sample_along_rays(*args, **kwargs)
        Overloaded function.

        1. sample_along_rays(self: erl_geometry.pyerl_geometry.RangeSensorFrame3Dd, num_samples_per_ray: int, max_in_obstacle_dist: float, sampled_rays_ratio: float) -> dict

        2. sample_along_rays(self: erl_geometry.pyerl_geometry.RangeSensorFrame3Dd, range_step: float, max_in_obstacle_dist: float, sampled_rays_ratio: float) -> dict
        """
    def sample_in_region_hpr(self, num_positions: int, num_near_surface_samples_per_ray: int, num_along_ray_samples_per_ray: int, max_in_obstacle_dist: float, parallel: bool) -> dict:
        """sample_in_region_hpr(self: erl_geometry.pyerl_geometry.RangeSensorFrame3Dd, num_positions: int, num_near_surface_samples_per_ray: int, num_along_ray_samples_per_ray: int, max_in_obstacle_dist: float, parallel: bool) -> dict"""
    def sample_in_region_vrs(self, num_hit_points: int, num_samples_per_azimuth_segment: int, num_azimuth_segments: int, parallel: bool) -> dict:
        """sample_in_region_vrs(self: erl_geometry.pyerl_geometry.RangeSensorFrame3Dd, num_hit_points: int, num_samples_per_azimuth_segment: int, num_azimuth_segments: int, parallel: bool) -> dict"""
    def sample_near_surface(self, num_samples_per_ray: int, max_offset: float, sampled_rays_ratio: float) -> dict:
        """sample_near_surface(self: erl_geometry.pyerl_geometry.RangeSensorFrame3Dd, num_samples_per_ray: int, max_offset: float, sampled_rays_ratio: float) -> dict"""
    @property
    def end_points_in_frame(self) -> numpy.ndarray[numpy.float64]:
        """(arg0: erl_geometry.pyerl_geometry.RangeSensorFrame3Dd) -> numpy.ndarray[numpy.float64]"""
    @property
    def end_points_in_world(self) -> numpy.ndarray[numpy.float64]:
        """(arg0: erl_geometry.pyerl_geometry.RangeSensorFrame3Dd) -> numpy.ndarray[numpy.float64]"""
    @property
    def frame_coords(self) -> numpy.ndarray[numpy.float64]:
        """(arg0: erl_geometry.pyerl_geometry.RangeSensorFrame3Dd) -> numpy.ndarray[numpy.float64]"""
    @property
    def hit_mask(self) -> numpy.ndarray[bool[m, n]]:
        """(arg0: erl_geometry.pyerl_geometry.RangeSensorFrame3Dd) -> numpy.ndarray[bool[m, n]]"""
    @property
    def hit_points_world(self) -> list[numpy.ndarray[numpy.float64[3, 1]]]:
        """(arg0: erl_geometry.pyerl_geometry.RangeSensorFrame3Dd) -> list[numpy.ndarray[numpy.float64[3, 1]]]"""
    @property
    def hit_ray_indices(self) -> list[tuple[int, int]]:
        """(arg0: erl_geometry.pyerl_geometry.RangeSensorFrame3Dd) -> list[tuple[int, int]]"""
    @property
    def is_valid(self) -> bool:
        """(arg0: erl_geometry.pyerl_geometry.RangeSensorFrame3Dd) -> bool"""
    @property
    def max_valid_range(self) -> float:
        """(arg0: erl_geometry.pyerl_geometry.RangeSensorFrame3Dd) -> float"""
    @property
    def min_valid_range(self) -> float:
        """(arg0: erl_geometry.pyerl_geometry.RangeSensorFrame3Dd) -> float"""
    @property
    def num_hit_rays(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.RangeSensorFrame3Dd) -> int"""
    @property
    def num_rays(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.RangeSensorFrame3Dd) -> int"""
    @property
    def pose_matrix(self) -> numpy.ndarray[numpy.float64[4, 4]]:
        """(arg0: erl_geometry.pyerl_geometry.RangeSensorFrame3Dd) -> numpy.ndarray[numpy.float64[4, 4]]"""
    @property
    def ranges(self) -> numpy.ndarray[numpy.float64[m, n]]:
        """(arg0: erl_geometry.pyerl_geometry.RangeSensorFrame3Dd) -> numpy.ndarray[numpy.float64[m, n]]"""
    @property
    def ray_directions_in_frame(self) -> numpy.ndarray[numpy.float64]:
        """(arg0: erl_geometry.pyerl_geometry.RangeSensorFrame3Dd) -> numpy.ndarray[numpy.float64]"""
    @property
    def ray_directions_in_world(self) -> numpy.ndarray[numpy.float64]:
        """(arg0: erl_geometry.pyerl_geometry.RangeSensorFrame3Dd) -> numpy.ndarray[numpy.float64]"""
    @property
    def rotation_matrix(self) -> numpy.ndarray[numpy.float64[3, 3]]:
        """(arg0: erl_geometry.pyerl_geometry.RangeSensorFrame3Dd) -> numpy.ndarray[numpy.float64[3, 3]]"""
    @property
    def translation_vector(self) -> numpy.ndarray[numpy.float64[3, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.RangeSensorFrame3Dd) -> numpy.ndarray[numpy.float64[3, 1]]"""

class RangeSensorFrame3Df:
    class Setting(YamlableBase):
        col_margin: int
        row_margin: int
        valid_range_max: float
        valid_range_min: float
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""
    def __init__(self, *args, **kwargs) -> None:
        """Initialize self.  See help(type(self)) for accurate signature."""
    def compute_closest_end_point(self, position_world: numpy.ndarray[numpy.float32[3, 1]], brute_force: bool = ...) -> dict:
        """compute_closest_end_point(self: erl_geometry.pyerl_geometry.RangeSensorFrame3Df, position_world: numpy.ndarray[numpy.float32[3, 1]], brute_force: bool = False) -> dict"""
    def compute_frame_coords(self, xyz_frame: numpy.ndarray[numpy.float32[3, 1]]) -> dict:
        """compute_frame_coords(self: erl_geometry.pyerl_geometry.RangeSensorFrame3Df, xyz_frame: numpy.ndarray[numpy.float32[3, 1]]) -> dict"""
    def compute_rays_at(self, position_world: numpy.ndarray[numpy.float32[3, 1]]) -> dict:
        """compute_rays_at(self: erl_geometry.pyerl_geometry.RangeSensorFrame3Df, position_world: numpy.ndarray[numpy.float32[3, 1]]) -> dict"""
    def coords_is_in_frame(self, frame_coords: numpy.ndarray[numpy.float32[2, 1]]) -> bool:
        """coords_is_in_frame(self: erl_geometry.pyerl_geometry.RangeSensorFrame3Df, frame_coords: numpy.ndarray[numpy.float32[2, 1]]) -> bool"""
    def dir_frame_to_world(self, dir_frame: numpy.ndarray[numpy.float32[3, 1]]) -> numpy.ndarray[numpy.float32[3, 1]]:
        """dir_frame_to_world(self: erl_geometry.pyerl_geometry.RangeSensorFrame3Df, dir_frame: numpy.ndarray[numpy.float32[3, 1]]) -> numpy.ndarray[numpy.float32[3, 1]]"""
    def dir_world_to_frame(self, dir_world: numpy.ndarray[numpy.float32[3, 1]]) -> numpy.ndarray[numpy.float32[3, 1]]:
        """dir_world_to_frame(self: erl_geometry.pyerl_geometry.RangeSensorFrame3Df, dir_world: numpy.ndarray[numpy.float32[3, 1]]) -> numpy.ndarray[numpy.float32[3, 1]]"""
    def pos_frame_to_world(self, pos_frame: numpy.ndarray[numpy.float32[3, 1]]) -> numpy.ndarray[numpy.float32[3, 1]]:
        """pos_frame_to_world(self: erl_geometry.pyerl_geometry.RangeSensorFrame3Df, pos_frame: numpy.ndarray[numpy.float32[3, 1]]) -> numpy.ndarray[numpy.float32[3, 1]]"""
    def pos_world_to_frame(self, pos_world: numpy.ndarray[numpy.float32[3, 1]]) -> numpy.ndarray[numpy.float32[3, 1]]:
        """pos_world_to_frame(self: erl_geometry.pyerl_geometry.RangeSensorFrame3Df, pos_world: numpy.ndarray[numpy.float32[3, 1]]) -> numpy.ndarray[numpy.float32[3, 1]]"""
    def position_is_in_frame(self, xyz_frame: numpy.ndarray[numpy.float32[3, 1]]) -> bool:
        """position_is_in_frame(self: erl_geometry.pyerl_geometry.RangeSensorFrame3Df, xyz_frame: numpy.ndarray[numpy.float32[3, 1]]) -> bool"""
    @overload
    def sample_along_rays(self, num_samples_per_ray: int, max_in_obstacle_dist: float, sampled_rays_ratio: float) -> dict:
        """sample_along_rays(*args, **kwargs)
        Overloaded function.

        1. sample_along_rays(self: erl_geometry.pyerl_geometry.RangeSensorFrame3Df, num_samples_per_ray: int, max_in_obstacle_dist: float, sampled_rays_ratio: float) -> dict

        2. sample_along_rays(self: erl_geometry.pyerl_geometry.RangeSensorFrame3Df, range_step: float, max_in_obstacle_dist: float, sampled_rays_ratio: float) -> dict
        """
    @overload
    def sample_along_rays(self, range_step: float, max_in_obstacle_dist: float, sampled_rays_ratio: float) -> dict:
        """sample_along_rays(*args, **kwargs)
        Overloaded function.

        1. sample_along_rays(self: erl_geometry.pyerl_geometry.RangeSensorFrame3Df, num_samples_per_ray: int, max_in_obstacle_dist: float, sampled_rays_ratio: float) -> dict

        2. sample_along_rays(self: erl_geometry.pyerl_geometry.RangeSensorFrame3Df, range_step: float, max_in_obstacle_dist: float, sampled_rays_ratio: float) -> dict
        """
    def sample_in_region_hpr(self, num_positions: int, num_near_surface_samples_per_ray: int, num_along_ray_samples_per_ray: int, max_in_obstacle_dist: float, parallel: bool) -> dict:
        """sample_in_region_hpr(self: erl_geometry.pyerl_geometry.RangeSensorFrame3Df, num_positions: int, num_near_surface_samples_per_ray: int, num_along_ray_samples_per_ray: int, max_in_obstacle_dist: float, parallel: bool) -> dict"""
    def sample_in_region_vrs(self, num_hit_points: int, num_samples_per_azimuth_segment: int, num_azimuth_segments: int, parallel: bool) -> dict:
        """sample_in_region_vrs(self: erl_geometry.pyerl_geometry.RangeSensorFrame3Df, num_hit_points: int, num_samples_per_azimuth_segment: int, num_azimuth_segments: int, parallel: bool) -> dict"""
    def sample_near_surface(self, num_samples_per_ray: int, max_offset: float, sampled_rays_ratio: float) -> dict:
        """sample_near_surface(self: erl_geometry.pyerl_geometry.RangeSensorFrame3Df, num_samples_per_ray: int, max_offset: float, sampled_rays_ratio: float) -> dict"""
    @property
    def end_points_in_frame(self) -> numpy.ndarray[numpy.float32]:
        """(arg0: erl_geometry.pyerl_geometry.RangeSensorFrame3Df) -> numpy.ndarray[numpy.float32]"""
    @property
    def end_points_in_world(self) -> numpy.ndarray[numpy.float32]:
        """(arg0: erl_geometry.pyerl_geometry.RangeSensorFrame3Df) -> numpy.ndarray[numpy.float32]"""
    @property
    def frame_coords(self) -> numpy.ndarray[numpy.float32]:
        """(arg0: erl_geometry.pyerl_geometry.RangeSensorFrame3Df) -> numpy.ndarray[numpy.float32]"""
    @property
    def hit_mask(self) -> numpy.ndarray[bool[m, n]]:
        """(arg0: erl_geometry.pyerl_geometry.RangeSensorFrame3Df) -> numpy.ndarray[bool[m, n]]"""
    @property
    def hit_points_world(self) -> list[numpy.ndarray[numpy.float32[3, 1]]]:
        """(arg0: erl_geometry.pyerl_geometry.RangeSensorFrame3Df) -> list[numpy.ndarray[numpy.float32[3, 1]]]"""
    @property
    def hit_ray_indices(self) -> list[tuple[int, int]]:
        """(arg0: erl_geometry.pyerl_geometry.RangeSensorFrame3Df) -> list[tuple[int, int]]"""
    @property
    def is_valid(self) -> bool:
        """(arg0: erl_geometry.pyerl_geometry.RangeSensorFrame3Df) -> bool"""
    @property
    def max_valid_range(self) -> float:
        """(arg0: erl_geometry.pyerl_geometry.RangeSensorFrame3Df) -> float"""
    @property
    def min_valid_range(self) -> float:
        """(arg0: erl_geometry.pyerl_geometry.RangeSensorFrame3Df) -> float"""
    @property
    def num_hit_rays(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.RangeSensorFrame3Df) -> int"""
    @property
    def num_rays(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.RangeSensorFrame3Df) -> int"""
    @property
    def pose_matrix(self) -> numpy.ndarray[numpy.float32[4, 4]]:
        """(arg0: erl_geometry.pyerl_geometry.RangeSensorFrame3Df) -> numpy.ndarray[numpy.float32[4, 4]]"""
    @property
    def ranges(self) -> numpy.ndarray[numpy.float32[m, n]]:
        """(arg0: erl_geometry.pyerl_geometry.RangeSensorFrame3Df) -> numpy.ndarray[numpy.float32[m, n]]"""
    @property
    def ray_directions_in_frame(self) -> numpy.ndarray[numpy.float32]:
        """(arg0: erl_geometry.pyerl_geometry.RangeSensorFrame3Df) -> numpy.ndarray[numpy.float32]"""
    @property
    def ray_directions_in_world(self) -> numpy.ndarray[numpy.float32]:
        """(arg0: erl_geometry.pyerl_geometry.RangeSensorFrame3Df) -> numpy.ndarray[numpy.float32]"""
    @property
    def rotation_matrix(self) -> numpy.ndarray[numpy.float32[3, 3]]:
        """(arg0: erl_geometry.pyerl_geometry.RangeSensorFrame3Df) -> numpy.ndarray[numpy.float32[3, 3]]"""
    @property
    def translation_vector(self) -> numpy.ndarray[numpy.float32[3, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.RangeSensorFrame3Df) -> numpy.ndarray[numpy.float32[3, 1]]"""

class Ray2D(Primitive2D):
    direction: numpy.ndarray[numpy.float64[2, 1]]
    origin: numpy.ndarray[numpy.float64[2, 1]]
    def __init__(self, id: int, origin: numpy.ndarray[numpy.float64[2, 1]], direction: numpy.ndarray[numpy.float64[2, 1]]) -> None:
        """__init__(self: erl_geometry.pyerl_geometry.Ray2D, id: int, origin: numpy.ndarray[numpy.float64[2, 1]], direction: numpy.ndarray[numpy.float64[2, 1]]) -> None"""

class Rectangle2D(Primitive2D):
    center: numpy.ndarray[numpy.float64[2, 1]]
    orientation_angle: float
    def __init__(self, id: int, center: numpy.ndarray[numpy.float64[2, 1]], half_sizes: numpy.ndarray[numpy.float64[2, 1]], angle: float) -> None:
        """__init__(self: erl_geometry.pyerl_geometry.Rectangle2D, id: int, center: numpy.ndarray[numpy.float64[2, 1]], half_sizes: numpy.ndarray[numpy.float64[2, 1]], angle: float) -> None"""
    def compute_points_on_boundary(self, num_points: int) -> list[numpy.ndarray[numpy.float64[2, 1]]]:
        """compute_points_on_boundary(self: erl_geometry.pyerl_geometry.Rectangle2D, num_points: int) -> list[numpy.ndarray[numpy.float64[2, 1]]]"""
    def translate(self, translation: numpy.ndarray[numpy.float64[2, 1]]) -> Rectangle2D:
        """translate(self: erl_geometry.pyerl_geometry.Rectangle2D, translation: numpy.ndarray[numpy.float64[2, 1]]) -> erl_geometry.pyerl_geometry.Rectangle2D"""
    @property
    def half_sizes(self) -> numpy.ndarray[numpy.float64[2, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.Rectangle2D) -> numpy.ndarray[numpy.float64[2, 1]]"""
    @property
    def rotation_matrix(self) -> numpy.ndarray[numpy.float64[2, 2]]:
        """(arg0: erl_geometry.pyerl_geometry.Rectangle2D) -> numpy.ndarray[numpy.float64[2, 2]]"""

class RgbdCamera3Dd(CameraBase3Dd):
    def __init__(self, setting: CameraIntrinsicD) -> None:
        """__init__(self: erl_geometry.pyerl_geometry.RgbdCamera3Dd, setting: erl_geometry.pyerl_geometry.CameraIntrinsicD) -> None"""
    @staticmethod
    def Setting() -> CameraIntrinsicD:
        """Setting() -> erl_geometry.pyerl_geometry.CameraIntrinsicD"""
    def add_mesh(self, mesh_path: str) -> None:
        """add_mesh(self: erl_geometry.pyerl_geometry.RgbdCamera3Dd, mesh_path: str) -> None"""
    def scan(self, arg0: numpy.ndarray[numpy.float64[3, 3]], arg1: numpy.ndarray[numpy.float64[3, 1]]) -> dict:
        """scan(self: erl_geometry.pyerl_geometry.RgbdCamera3Dd, arg0: numpy.ndarray[numpy.float64[3, 3]], arg1: numpy.ndarray[numpy.float64[3, 1]]) -> dict"""
    @property
    def ray_directions_in_frame(self) -> numpy.ndarray[numpy.float64]:
        """(arg0: erl_geometry.pyerl_geometry.RgbdCamera3Dd) -> numpy.ndarray[numpy.float64]"""
    @property
    def setting(self) -> CameraIntrinsicD:
        """(arg0: erl_geometry.pyerl_geometry.RgbdCamera3Dd) -> erl_geometry.pyerl_geometry.CameraIntrinsicD"""

class RgbdCamera3Df(CameraBase3Df):
    def __init__(self, setting: CameraIntrinsicF) -> None:
        """__init__(self: erl_geometry.pyerl_geometry.RgbdCamera3Df, setting: erl_geometry.pyerl_geometry.CameraIntrinsicF) -> None"""
    @staticmethod
    def Setting() -> CameraIntrinsicF:
        """Setting() -> erl_geometry.pyerl_geometry.CameraIntrinsicF"""
    def add_mesh(self, mesh_path: str) -> None:
        """add_mesh(self: erl_geometry.pyerl_geometry.RgbdCamera3Df, mesh_path: str) -> None"""
    def scan(self, arg0: numpy.ndarray[numpy.float32[3, 3]], arg1: numpy.ndarray[numpy.float32[3, 1]]) -> dict:
        """scan(self: erl_geometry.pyerl_geometry.RgbdCamera3Df, arg0: numpy.ndarray[numpy.float32[3, 3]], arg1: numpy.ndarray[numpy.float32[3, 1]]) -> dict"""
    @property
    def ray_directions_in_frame(self) -> numpy.ndarray[numpy.float32]:
        """(arg0: erl_geometry.pyerl_geometry.RgbdCamera3Df) -> numpy.ndarray[numpy.float32]"""
    @property
    def setting(self) -> CameraIntrinsicF:
        """(arg0: erl_geometry.pyerl_geometry.RgbdCamera3Df) -> erl_geometry.pyerl_geometry.CameraIntrinsicF"""

class RgbdFrame3Dd(DepthFrame3Dd):
    def __init__(self, setting: DepthFrame3Dd.Setting) -> None:
        """__init__(self: erl_geometry.pyerl_geometry.RgbdFrame3Dd, setting: erl_geometry.pyerl_geometry.DepthFrame3Dd.Setting) -> None"""
    def convert_to_point_cloud(self, in_world_frame: bool) -> dict:
        """convert_to_point_cloud(self: erl_geometry.pyerl_geometry.RgbdFrame3Dd, in_world_frame: bool) -> dict"""
    def update_rgbd(self, rotation: numpy.ndarray[numpy.float64[3, 3], flags.f_contiguous], translation: numpy.ndarray[numpy.float64[3, 1]], depth: numpy.ndarray[numpy.float64[m, n]], rgb: Mat) -> None:
        """update_rgbd(self: erl_geometry.pyerl_geometry.RgbdFrame3Dd, rotation: numpy.ndarray[numpy.float64[3, 3], flags.f_contiguous], translation: numpy.ndarray[numpy.float64[3, 1]], depth: numpy.ndarray[numpy.float64[m, n]], rgb: Mat) -> None"""

class RgbdFrame3Df(DepthFrame3Df):
    def __init__(self, setting: DepthFrame3Df.Setting) -> None:
        """__init__(self: erl_geometry.pyerl_geometry.RgbdFrame3Df, setting: erl_geometry.pyerl_geometry.DepthFrame3Df.Setting) -> None"""
    def convert_to_point_cloud(self, in_world_frame: bool) -> dict:
        """convert_to_point_cloud(self: erl_geometry.pyerl_geometry.RgbdFrame3Df, in_world_frame: bool) -> dict"""
    def update_rgbd(self, rotation: numpy.ndarray[numpy.float32[3, 3], flags.f_contiguous], translation: numpy.ndarray[numpy.float32[3, 1]], depth: numpy.ndarray[numpy.float32[m, n]], rgb: Mat) -> None:
        """update_rgbd(self: erl_geometry.pyerl_geometry.RgbdFrame3Df, rotation: numpy.ndarray[numpy.float32[3, 3], flags.f_contiguous], translation: numpy.ndarray[numpy.float32[3, 1]], depth: numpy.ndarray[numpy.float32[m, n]], rgb: Mat) -> None"""

class Segment2D(Line2D):
    def __init__(self, id: int, p0: numpy.ndarray[numpy.float64[2, 1]], p1: numpy.ndarray[numpy.float64[2, 1]]) -> None:
        """__init__(self: erl_geometry.pyerl_geometry.Segment2D, id: int, p0: numpy.ndarray[numpy.float64[2, 1]], p1: numpy.ndarray[numpy.float64[2, 1]]) -> None"""

class SemiSparseNdTreeSetting(NdTreeSetting):
    full_depth: int
    init_voxel_num: int
    def __init__(self) -> None:
        """__init__(self: erl_geometry.pyerl_geometry.SemiSparseNdTreeSetting) -> None"""

class SemiSparseOctreeD(AbstractOctreeD):
    class BottomLeafNeighborIterator(SemiSparseOctreeD.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class Drawer:
        def __init__(self, setting, octree: SemiSparseOctreeD = ...) -> None:
            """__init__(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD.Drawer, setting: erl::geometry::AbstractOctreeDrawer::Setting, octree: erl_geometry.pyerl_geometry.SemiSparseOctreeD = None) -> None"""
        @staticmethod
        def Setting(*args, **kwargs):
            """Setting() -> erl::geometry::AbstractOctreeDrawer::Setting"""
        def draw_leaves(self, filename: str) -> None:
            """draw_leaves(*args, **kwargs)
            Overloaded function.

            1. draw_leaves(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD.Drawer) -> list[open3d::geometry::Geometry]

            2. draw_leaves(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD.Drawer, filename: str) -> None
            """
        def draw_tree(self, filename: str) -> None:
            """draw_tree(*args, **kwargs)
            Overloaded function.

            1. draw_tree(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD.Drawer) -> list[open3d::geometry::Geometry]

            2. draw_tree(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD.Drawer, filename: str) -> None
            """
        def set_draw_leaf_callback(self, callback) -> None:
            """set_draw_leaf_callback(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD.Drawer, callback: Callable[[erl_geometry.pyerl_geometry.SemiSparseOctreeD.Drawer, list[open3d::geometry::Geometry], erl_geometry.pyerl_geometry.SemiSparseOctreeD.LeafInAabbIterator], None]) -> None"""
        def set_draw_tree_callback(self, callback) -> None:
            """set_draw_tree_callback(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD.Drawer, callback: Callable[[erl_geometry.pyerl_geometry.SemiSparseOctreeD.Drawer, list[open3d::geometry::Geometry], erl_geometry.pyerl_geometry.SemiSparseOctreeD.TreeInAabbIterator], None]) -> None"""
        @property
        def setting(self):
            """(arg0: erl_geometry.pyerl_geometry.SemiSparseOctreeD.Drawer) -> erl::geometry::AbstractOctreeDrawer::Setting"""

    class EastLeafNeighborIterator(SemiSparseOctreeD.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class IteratorBase(AbstractOctreeD.OctreeNodeIterator):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""
        def __eq__(self, arg0: SemiSparseOctreeD.IteratorBase) -> bool:
            """__eq__(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD.IteratorBase, arg0: erl_geometry.pyerl_geometry.SemiSparseOctreeD.IteratorBase) -> bool"""
        def __ne__(self, arg0: SemiSparseOctreeD.IteratorBase) -> bool:
            """__ne__(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD.IteratorBase, arg0: erl_geometry.pyerl_geometry.SemiSparseOctreeD.IteratorBase) -> bool"""
        @property
        def index_key(self) -> OctreeKey:
            """(arg0: erl_geometry.pyerl_geometry.SemiSparseOctreeD.IteratorBase) -> erl_geometry.pyerl_geometry.OctreeKey"""
        @property
        def key(self) -> OctreeKey:
            """(arg0: erl_geometry.pyerl_geometry.SemiSparseOctreeD.IteratorBase) -> erl_geometry.pyerl_geometry.OctreeKey"""
        @property
        def node(self) -> SemiSparseOctreeNode:
            """(arg0: erl_geometry.pyerl_geometry.SemiSparseOctreeD.IteratorBase) -> erl_geometry.pyerl_geometry.SemiSparseOctreeNode"""
        @property
        def node_aabb(self) -> Aabb3Dd:
            """(arg0: erl_geometry.pyerl_geometry.SemiSparseOctreeD.IteratorBase) -> erl_geometry.pyerl_geometry.Aabb3Dd"""

    class LeafInAabbIterator(SemiSparseOctreeD.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class LeafIterator(SemiSparseOctreeD.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class LeafOfNodeIterator(SemiSparseOctreeD.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class NodeOnRayIterator(SemiSparseOctreeD.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""
        @property
        def distance(self) -> float:
            """(arg0: erl_geometry.pyerl_geometry.SemiSparseOctreeD.NodeOnRayIterator) -> float"""

    class NorthLeafNeighborIterator(SemiSparseOctreeD.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class SouthLeafNeighborIterator(SemiSparseOctreeD.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class TopLeafNeighborIterator(SemiSparseOctreeD.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class TreeInAabbIterator(SemiSparseOctreeD.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class TreeIterator(SemiSparseOctreeD.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class WestLeafNeighborIterator(SemiSparseOctreeD.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""
    @overload
    def __init__(self) -> None:
        """__init__(*args, **kwargs)
        Overloaded function.

        1. __init__(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD) -> None

        2. __init__(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, setting: erl_geometry.pyerl_geometry.SemiSparseNdTreeSetting) -> None

        3. __init__(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, filename: str) -> None
        """
    @overload
    def __init__(self, setting: SemiSparseNdTreeSetting) -> None:
        """__init__(*args, **kwargs)
        Overloaded function.

        1. __init__(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD) -> None

        2. __init__(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, setting: erl_geometry.pyerl_geometry.SemiSparseNdTreeSetting) -> None

        3. __init__(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, filename: str) -> None
        """
    @overload
    def __init__(self, filename: str) -> None:
        """__init__(*args, **kwargs)
        Overloaded function.

        1. __init__(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD) -> None

        2. __init__(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, setting: erl_geometry.pyerl_geometry.SemiSparseNdTreeSetting) -> None

        3. __init__(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, filename: str) -> None
        """
    def Setting(self) -> SemiSparseNdTreeSetting:
        """Setting() -> erl_geometry.pyerl_geometry.SemiSparseNdTreeSetting"""
    @overload
    def adjust_key_to_depth(self, key: int, depth: int) -> int:
        """adjust_key_to_depth(*args, **kwargs)
        Overloaded function.

        1. adjust_key_to_depth(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, key: int, depth: int) -> int

        2. adjust_key_to_depth(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> erl_geometry.pyerl_geometry.OctreeKey
        """
    @overload
    def adjust_key_to_depth(self, key: OctreeKey, depth: int) -> OctreeKey:
        """adjust_key_to_depth(*args, **kwargs)
        Overloaded function.

        1. adjust_key_to_depth(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, key: int, depth: int) -> int

        2. adjust_key_to_depth(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> erl_geometry.pyerl_geometry.OctreeKey
        """
    def clear(self) -> None:
        """clear(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD) -> None"""
    def compute_bottom_neighbor_key(self, key: OctreeKey, depth: int) -> OctreeKey | None:
        """compute_bottom_neighbor_key(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]"""
    def compute_common_ancestor_key(self, arg0: OctreeKey, arg1: OctreeKey) -> tuple[OctreeKey, int]:
        """compute_common_ancestor_key(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, arg0: erl_geometry.pyerl_geometry.OctreeKey, arg1: erl_geometry.pyerl_geometry.OctreeKey) -> tuple[erl_geometry.pyerl_geometry.OctreeKey, int]"""
    def compute_east_neighbor_key(self, key: OctreeKey, depth: int) -> OctreeKey | None:
        """compute_east_neighbor_key(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]"""
    def compute_north_neighbor_key(self, key: OctreeKey, depth: int) -> OctreeKey | None:
        """compute_north_neighbor_key(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]"""
    def compute_ray_coords(self, sx: float, sy: float, sz: float, ex: float, ey: float, ez: float) -> list[numpy.ndarray[numpy.float64[3, 1]]] | None:
        """compute_ray_coords(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, sx: float, sy: float, sz: float, ex: float, ey: float, ez: float) -> Optional[list[numpy.ndarray[numpy.float64[3, 1]]]]"""
    def compute_ray_keys(self, sx: float, sy: float, sz: float, ex: float, ey: float, ez: float) -> list[OctreeKey] | None:
        """compute_ray_keys(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, sx: float, sy: float, sz: float, ex: float, ey: float, ez: float) -> Optional[list[erl_geometry.pyerl_geometry.OctreeKey]]"""
    def compute_south_neighbor_key(self, key: OctreeKey, depth: int) -> OctreeKey | None:
        """compute_south_neighbor_key(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]"""
    def compute_top_neighbor_key(self, key: OctreeKey, depth: int) -> OctreeKey | None:
        """compute_top_neighbor_key(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]"""
    def compute_west_neighbor_key(self, key: OctreeKey, depth: int) -> OctreeKey | None:
        """compute_west_neighbor_key(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]"""
    @overload
    def coord_to_key(self, coordinate: float) -> int:
        """coord_to_key(*args, **kwargs)
        Overloaded function.

        1. coord_to_key(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, coordinate: float) -> int

        2. coord_to_key(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, coordinate: float, depth: int) -> int

        3. coord_to_key(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, x: float, y: float, z: float) -> erl_geometry.pyerl_geometry.OctreeKey

        4. coord_to_key(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, x: float, y: float, z: float, depth: int) -> erl_geometry.pyerl_geometry.OctreeKey
        """
    @overload
    def coord_to_key(self, coordinate: float, depth: int) -> int:
        """coord_to_key(*args, **kwargs)
        Overloaded function.

        1. coord_to_key(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, coordinate: float) -> int

        2. coord_to_key(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, coordinate: float, depth: int) -> int

        3. coord_to_key(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, x: float, y: float, z: float) -> erl_geometry.pyerl_geometry.OctreeKey

        4. coord_to_key(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, x: float, y: float, z: float, depth: int) -> erl_geometry.pyerl_geometry.OctreeKey
        """
    @overload
    def coord_to_key(self, x: float, y: float, z: float) -> OctreeKey:
        """coord_to_key(*args, **kwargs)
        Overloaded function.

        1. coord_to_key(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, coordinate: float) -> int

        2. coord_to_key(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, coordinate: float, depth: int) -> int

        3. coord_to_key(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, x: float, y: float, z: float) -> erl_geometry.pyerl_geometry.OctreeKey

        4. coord_to_key(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, x: float, y: float, z: float, depth: int) -> erl_geometry.pyerl_geometry.OctreeKey
        """
    @overload
    def coord_to_key(self, x: float, y: float, z: float, depth: int) -> OctreeKey:
        """coord_to_key(*args, **kwargs)
        Overloaded function.

        1. coord_to_key(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, coordinate: float) -> int

        2. coord_to_key(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, coordinate: float, depth: int) -> int

        3. coord_to_key(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, x: float, y: float, z: float) -> erl_geometry.pyerl_geometry.OctreeKey

        4. coord_to_key(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, x: float, y: float, z: float, depth: int) -> erl_geometry.pyerl_geometry.OctreeKey
        """
    @overload
    def coord_to_key_checked(self, coordinate: float) -> int | None:
        """coord_to_key_checked(*args, **kwargs)
        Overloaded function.

        1. coord_to_key_checked(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, coordinate: float) -> Optional[int]

        2. coord_to_key_checked(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, coordinate: float, depth: int) -> Optional[int]

        3. coord_to_key_checked(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, x: float, y: float, z: float) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]

        4. coord_to_key_checked(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, x: float, y: float, z: float, depth: int) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]
        """
    @overload
    def coord_to_key_checked(self, coordinate: float, depth: int) -> int | None:
        """coord_to_key_checked(*args, **kwargs)
        Overloaded function.

        1. coord_to_key_checked(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, coordinate: float) -> Optional[int]

        2. coord_to_key_checked(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, coordinate: float, depth: int) -> Optional[int]

        3. coord_to_key_checked(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, x: float, y: float, z: float) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]

        4. coord_to_key_checked(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, x: float, y: float, z: float, depth: int) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]
        """
    @overload
    def coord_to_key_checked(self, x: float, y: float, z: float) -> OctreeKey | None:
        """coord_to_key_checked(*args, **kwargs)
        Overloaded function.

        1. coord_to_key_checked(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, coordinate: float) -> Optional[int]

        2. coord_to_key_checked(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, coordinate: float, depth: int) -> Optional[int]

        3. coord_to_key_checked(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, x: float, y: float, z: float) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]

        4. coord_to_key_checked(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, x: float, y: float, z: float, depth: int) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]
        """
    @overload
    def coord_to_key_checked(self, x: float, y: float, z: float, depth: int) -> OctreeKey | None:
        """coord_to_key_checked(*args, **kwargs)
        Overloaded function.

        1. coord_to_key_checked(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, coordinate: float) -> Optional[int]

        2. coord_to_key_checked(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, coordinate: float, depth: int) -> Optional[int]

        3. coord_to_key_checked(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, x: float, y: float, z: float) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]

        4. coord_to_key_checked(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, x: float, y: float, z: float, depth: int) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]
        """
    def create_node_child(self, node: SemiSparseOctreeNode, child_idx: int) -> SemiSparseOctreeNode:
        """create_node_child(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, node: erl_geometry.pyerl_geometry.SemiSparseOctreeNode, child_idx: int) -> erl_geometry.pyerl_geometry.SemiSparseOctreeNode"""
    @overload
    def delete_node(self, x: float, y: float, z: float, depth: int) -> int:
        """delete_node(*args, **kwargs)
        Overloaded function.

        1. delete_node(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, x: float, y: float, z: float, depth: int) -> int

        2. delete_node(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> None
        """
    @overload
    def delete_node(self, key: OctreeKey, depth: int) -> None:
        """delete_node(*args, **kwargs)
        Overloaded function.

        1. delete_node(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, x: float, y: float, z: float, depth: int) -> int

        2. delete_node(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> None
        """
    def delete_node_child(self, node: SemiSparseOctreeNode, child_idx: int, key: OctreeKey) -> int:
        """delete_node_child(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, node: erl_geometry.pyerl_geometry.SemiSparseOctreeNode, child_idx: int, key: erl_geometry.pyerl_geometry.OctreeKey) -> int"""
    def expand(self) -> None:
        """expand(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD) -> None"""
    def expand_node(self, node: SemiSparseOctreeNode) -> None:
        """expand_node(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, node: erl_geometry.pyerl_geometry.SemiSparseOctreeNode) -> None"""
    def find_voxel_index(self, key: OctreeKey) -> int:
        """find_voxel_index(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey) -> int"""
    def find_voxel_indices(self, points: numpy.ndarray[numpy.float64[3, n]], parallel: bool) -> list[int]:
        """find_voxel_indices(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, points: numpy.ndarray[numpy.float64[3, n]], parallel: bool) -> list[int]"""
    def get_node_child(self, node: SemiSparseOctreeNode, child_idx: int) -> SemiSparseOctreeNode:
        """get_node_child(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, node: erl_geometry.pyerl_geometry.SemiSparseOctreeNode, child_idx: int) -> erl_geometry.pyerl_geometry.SemiSparseOctreeNode"""
    def get_node_size(self, depth: int) -> float:
        """get_node_size(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, depth: int) -> float"""
    @overload
    def insert_node(self, x: float, y: float, z: float, depth: int) -> SemiSparseOctreeNode:
        """insert_node(*args, **kwargs)
        Overloaded function.

        1. insert_node(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, x: float, y: float, z: float, depth: int) -> erl_geometry.pyerl_geometry.SemiSparseOctreeNode

        2. insert_node(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> erl_geometry.pyerl_geometry.SemiSparseOctreeNode
        """
    @overload
    def insert_node(self, key: OctreeKey, depth: int) -> SemiSparseOctreeNode:
        """insert_node(*args, **kwargs)
        Overloaded function.

        1. insert_node(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, x: float, y: float, z: float, depth: int) -> erl_geometry.pyerl_geometry.SemiSparseOctreeNode

        2. insert_node(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> erl_geometry.pyerl_geometry.SemiSparseOctreeNode
        """
    def insert_point(self, key: OctreeKey, max_depth: int) -> int:
        """insert_point(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, max_depth: int) -> int"""
    def insert_points(self, points: numpy.ndarray[numpy.float64[3, n]]) -> list[int]:
        """insert_points(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, points: numpy.ndarray[numpy.float64[3, n]]) -> list[int]"""
    def is_node_collapsible(self, node: SemiSparseOctreeNode) -> bool:
        """is_node_collapsible(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, node: erl_geometry.pyerl_geometry.SemiSparseOctreeNode) -> bool"""
    @overload
    def iter_bottom_leaf_neighbor(self, x: float, y: float, z: float, max_leaf_depth: int = ...) -> Iterator[SemiSparseOctreeD.BottomLeafNeighborIterator]:
        """iter_bottom_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_bottom_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeD.BottomLeafNeighborIterator]

        2. iter_bottom_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeD.BottomLeafNeighborIterator]
        """
    @overload
    def iter_bottom_leaf_neighbor(self, key: OctreeKey, key_depth: int, max_leaf_depth: int = ...) -> Iterator[SemiSparseOctreeD.BottomLeafNeighborIterator]:
        """iter_bottom_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_bottom_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeD.BottomLeafNeighborIterator]

        2. iter_bottom_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeD.BottomLeafNeighborIterator]
        """
    @overload
    def iter_east_leaf_neighbor(self, x: float, y: float, z: float, max_leaf_depth: int = ...) -> Iterator[SemiSparseOctreeD.EastLeafNeighborIterator]:
        """iter_east_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_east_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeD.EastLeafNeighborIterator]

        2. iter_east_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeD.EastLeafNeighborIterator]
        """
    @overload
    def iter_east_leaf_neighbor(self, key: OctreeKey, key_depth: int, max_leaf_depth: int = ...) -> Iterator[SemiSparseOctreeD.EastLeafNeighborIterator]:
        """iter_east_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_east_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeD.EastLeafNeighborIterator]

        2. iter_east_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeD.EastLeafNeighborIterator]
        """
    def iter_leaf(self, max_depth: int = ...) -> Iterator[SemiSparseOctreeD.LeafIterator]:
        """iter_leaf(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeD.LeafIterator]"""
    @overload
    def iter_leaf_in_aabb(self, aabb_min_x: float, aabb_min_y: float, aabb_min_z: float, aabb_max_x: float, aabb_max_y: float, aabb_max_z: float, max_depth: int = ...) -> Iterator[SemiSparseOctreeD.LeafInAabbIterator]:
        """iter_leaf_in_aabb(*args, **kwargs)
        Overloaded function.

        1. iter_leaf_in_aabb(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, aabb_min_x: float, aabb_min_y: float, aabb_min_z: float, aabb_max_x: float, aabb_max_y: float, aabb_max_z: float, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeD.LeafInAabbIterator]

        2. iter_leaf_in_aabb(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, aabb_min_key: erl_geometry.pyerl_geometry.OctreeKey, aabb_max_key: erl_geometry.pyerl_geometry.OctreeKey, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeD.LeafInAabbIterator]
        """
    @overload
    def iter_leaf_in_aabb(self, aabb_min_key: OctreeKey, aabb_max_key: OctreeKey, max_depth: int = ...) -> Iterator[SemiSparseOctreeD.LeafInAabbIterator]:
        """iter_leaf_in_aabb(*args, **kwargs)
        Overloaded function.

        1. iter_leaf_in_aabb(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, aabb_min_x: float, aabb_min_y: float, aabb_min_z: float, aabb_max_x: float, aabb_max_y: float, aabb_max_z: float, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeD.LeafInAabbIterator]

        2. iter_leaf_in_aabb(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, aabb_min_key: erl_geometry.pyerl_geometry.OctreeKey, aabb_max_key: erl_geometry.pyerl_geometry.OctreeKey, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeD.LeafInAabbIterator]
        """
    def iter_leaf_of_node(self, node_key: OctreeKey, node_depth: int, max_depth: int = ...) -> Iterator[SemiSparseOctreeD.LeafOfNodeIterator]:
        """iter_leaf_of_node(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, node_key: erl_geometry.pyerl_geometry.OctreeKey, node_depth: int, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeD.LeafOfNodeIterator]"""
    def iter_node(self, max_depth: int = ...) -> Iterator[SemiSparseOctreeD.TreeIterator]:
        """iter_node(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeD.TreeIterator]"""
    @overload
    def iter_node_in_aabb(self, aabb_min_x: float, aabb_min_y: float, aabb_min_z: float, aabb_max_x: float, aabb_max_y: float, aabb_max_z: float, max_depth: int = ...) -> Iterator[SemiSparseOctreeD.TreeInAabbIterator]:
        """iter_node_in_aabb(*args, **kwargs)
        Overloaded function.

        1. iter_node_in_aabb(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, aabb_min_x: float, aabb_min_y: float, aabb_min_z: float, aabb_max_x: float, aabb_max_y: float, aabb_max_z: float, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeD.TreeInAabbIterator]

        2. iter_node_in_aabb(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, aabb_min_key: erl_geometry.pyerl_geometry.OctreeKey, aabb_max_key: erl_geometry.pyerl_geometry.OctreeKey, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeD.TreeInAabbIterator]
        """
    @overload
    def iter_node_in_aabb(self, aabb_min_key: OctreeKey, aabb_max_key: OctreeKey, max_depth: int = ...) -> Iterator[SemiSparseOctreeD.TreeInAabbIterator]:
        """iter_node_in_aabb(*args, **kwargs)
        Overloaded function.

        1. iter_node_in_aabb(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, aabb_min_x: float, aabb_min_y: float, aabb_min_z: float, aabb_max_x: float, aabb_max_y: float, aabb_max_z: float, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeD.TreeInAabbIterator]

        2. iter_node_in_aabb(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, aabb_min_key: erl_geometry.pyerl_geometry.OctreeKey, aabb_max_key: erl_geometry.pyerl_geometry.OctreeKey, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeD.TreeInAabbIterator]
        """
    def iter_node_on_ray(self, px: float, py: float, pz: float, vx: float, vy: float, vz: float, max_range: float = ..., node_padding: float = ..., bidirectional: bool = ..., leaf_only: bool = ..., min_node_depth: int = ..., max_node_depth: int = ...) -> Iterator[SemiSparseOctreeD.NodeOnRayIterator]:
        """iter_node_on_ray(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, px: float, py: float, pz: float, vx: float, vy: float, vz: float, max_range: float = -1, node_padding: float = 0, bidirectional: bool = False, leaf_only: bool = True, min_node_depth: int = 0, max_node_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeD.NodeOnRayIterator]"""
    @overload
    def iter_north_leaf_neighbor(self, x: float, y: float, z: float, max_leaf_depth: int = ...) -> Iterator[SemiSparseOctreeD.NorthLeafNeighborIterator]:
        """iter_north_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_north_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeD.NorthLeafNeighborIterator]

        2. iter_north_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeD.NorthLeafNeighborIterator]
        """
    @overload
    def iter_north_leaf_neighbor(self, key: OctreeKey, key_depth: int, max_leaf_depth: int = ...) -> Iterator[SemiSparseOctreeD.NorthLeafNeighborIterator]:
        """iter_north_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_north_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeD.NorthLeafNeighborIterator]

        2. iter_north_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeD.NorthLeafNeighborIterator]
        """
    @overload
    def iter_south_leaf_neighbor(self, x: float, y: float, z: float, max_leaf_depth: int = ...) -> Iterator[SemiSparseOctreeD.SouthLeafNeighborIterator]:
        """iter_south_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_south_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeD.SouthLeafNeighborIterator]

        2. iter_south_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeD.SouthLeafNeighborIterator]
        """
    @overload
    def iter_south_leaf_neighbor(self, key: OctreeKey, key_depth: int, max_leaf_depth: int = ...) -> Iterator[SemiSparseOctreeD.SouthLeafNeighborIterator]:
        """iter_south_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_south_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeD.SouthLeafNeighborIterator]

        2. iter_south_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeD.SouthLeafNeighborIterator]
        """
    @overload
    def iter_top_leaf_neighbor(self, x: float, y: float, z: float, max_leaf_depth: int = ...) -> Iterator[SemiSparseOctreeD.TopLeafNeighborIterator]:
        """iter_top_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_top_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeD.TopLeafNeighborIterator]

        2. iter_top_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeD.TopLeafNeighborIterator]
        """
    @overload
    def iter_top_leaf_neighbor(self, key: OctreeKey, key_depth: int, max_leaf_depth: int = ...) -> Iterator[SemiSparseOctreeD.TopLeafNeighborIterator]:
        """iter_top_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_top_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeD.TopLeafNeighborIterator]

        2. iter_top_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeD.TopLeafNeighborIterator]
        """
    @overload
    def iter_west_leaf_neighbor(self, x: float, y: float, z: float, max_leaf_depth: int = ...) -> Iterator[SemiSparseOctreeD.WestLeafNeighborIterator]:
        """iter_west_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_west_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeD.WestLeafNeighborIterator]

        2. iter_west_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeD.WestLeafNeighborIterator]
        """
    @overload
    def iter_west_leaf_neighbor(self, key: OctreeKey, key_depth: int, max_leaf_depth: int = ...) -> Iterator[SemiSparseOctreeD.WestLeafNeighborIterator]:
        """iter_west_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_west_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeD.WestLeafNeighborIterator]

        2. iter_west_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeD.WestLeafNeighborIterator]
        """
    @overload
    def key_to_coord(self, key: int) -> float:
        """key_to_coord(*args, **kwargs)
        Overloaded function.

        1. key_to_coord(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, key: int) -> float

        2. key_to_coord(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, key: int, depth: int) -> float

        3. key_to_coord(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey) -> tuple[float, float, float]

        4. key_to_coord(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> tuple[float, float, float]
        """
    @overload
    def key_to_coord(self, key: int, depth: int) -> float:
        """key_to_coord(*args, **kwargs)
        Overloaded function.

        1. key_to_coord(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, key: int) -> float

        2. key_to_coord(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, key: int, depth: int) -> float

        3. key_to_coord(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey) -> tuple[float, float, float]

        4. key_to_coord(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> tuple[float, float, float]
        """
    @overload
    def key_to_coord(self, key: OctreeKey) -> tuple[float, float, float]:
        """key_to_coord(*args, **kwargs)
        Overloaded function.

        1. key_to_coord(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, key: int) -> float

        2. key_to_coord(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, key: int, depth: int) -> float

        3. key_to_coord(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey) -> tuple[float, float, float]

        4. key_to_coord(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> tuple[float, float, float]
        """
    @overload
    def key_to_coord(self, key: OctreeKey, depth: int) -> tuple[float, float, float]:
        """key_to_coord(*args, **kwargs)
        Overloaded function.

        1. key_to_coord(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, key: int) -> float

        2. key_to_coord(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, key: int, depth: int) -> float

        3. key_to_coord(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey) -> tuple[float, float, float]

        4. key_to_coord(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> tuple[float, float, float]
        """
    def prune(self) -> None:
        """prune(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD) -> None"""
    def prune_node(self, node: SemiSparseOctreeNode) -> bool:
        """prune_node(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, node: erl_geometry.pyerl_geometry.SemiSparseOctreeNode) -> bool"""
    @overload
    def search(self, x: float, y: float, z: float, max_depth: int = ...) -> SemiSparseOctreeNode:
        """search(*args, **kwargs)
        Overloaded function.

        1. search(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, x: float, y: float, z: float, max_depth: int = 0) -> erl_geometry.pyerl_geometry.SemiSparseOctreeNode

        2. search(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, max_depth: int = 0) -> erl_geometry.pyerl_geometry.SemiSparseOctreeNode
        """
    @overload
    def search(self, key: OctreeKey, max_depth: int = ...) -> SemiSparseOctreeNode:
        """search(*args, **kwargs)
        Overloaded function.

        1. search(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, x: float, y: float, z: float, max_depth: int = 0) -> erl_geometry.pyerl_geometry.SemiSparseOctreeNode

        2. search(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, key: erl_geometry.pyerl_geometry.OctreeKey, max_depth: int = 0) -> erl_geometry.pyerl_geometry.SemiSparseOctreeNode
        """
    def visualize(self, leaf_only: bool = ..., scaling: float = ..., area_min: numpy.ndarray[numpy.float64[3, 1]] = ..., area_max: numpy.ndarray[numpy.float64[3, 1]] = ..., border_color: numpy.ndarray[numpy.float64[3, 1]] = ..., window_width: int = ..., window_height: int = ..., window_left: int = ..., window_top: int = ...) -> None:
        """visualize(self: erl_geometry.pyerl_geometry.SemiSparseOctreeD, leaf_only: bool = False, scaling: float = 1.0, area_min: numpy.ndarray[numpy.float64[3, 1]] = array([-1., -1., -1.], dtype=float32), area_max: numpy.ndarray[numpy.float64[3, 1]] = array([1., 1., 1.], dtype=float32), border_color: numpy.ndarray[numpy.float64[3, 1]] = array([0., 0., 0.], dtype=float32), window_width: int = 1920, window_height: int = 1080, window_left: int = 50, window_top: int = 50) -> None"""
    @property
    def children(self):
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseOctreeD) -> list[Annotated[list[int], FixedSize(8)]]"""
    @property
    def memory_usage(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseOctreeD) -> int"""
    @property
    def memory_usage_per_node(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseOctreeD) -> int"""
    @property
    def metric_aabb(self) -> Aabb3Dd:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseOctreeD) -> erl_geometry.pyerl_geometry.Aabb3Dd"""
    @property
    def metric_max(self) -> numpy.ndarray[numpy.float64[3, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseOctreeD) -> numpy.ndarray[numpy.float64[3, 1]]"""
    @property
    def metric_min(self) -> numpy.ndarray[numpy.float64[3, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseOctreeD) -> numpy.ndarray[numpy.float64[3, 1]]"""
    @property
    def metric_min_max(self) -> tuple[numpy.ndarray[numpy.float64[3, 1]], numpy.ndarray[numpy.float64[3, 1]]]:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseOctreeD) -> tuple[numpy.ndarray[numpy.float64[3, 1]], numpy.ndarray[numpy.float64[3, 1]]]"""
    @property
    def metric_size(self) -> numpy.ndarray[numpy.float64[3, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseOctreeD) -> numpy.ndarray[numpy.float64[3, 1]]"""
    @property
    def num_vertices(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseOctreeD) -> int"""
    @property
    def number_of_leaf_nodes(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseOctreeD) -> int"""
    @property
    def number_of_nodes(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseOctreeD) -> int"""
    @property
    def parents(self) -> list[int]:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseOctreeD) -> list[int]"""
    @property
    def resolution(self) -> float:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseOctreeD) -> float"""
    @property
    def root(self) -> SemiSparseOctreeNode:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseOctreeD) -> erl_geometry.pyerl_geometry.SemiSparseOctreeNode"""
    @property
    def setting(self) -> SemiSparseNdTreeSetting:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseOctreeD) -> erl_geometry.pyerl_geometry.SemiSparseNdTreeSetting"""
    @property
    def tree_center(self) -> numpy.ndarray[numpy.float64[3, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseOctreeD) -> numpy.ndarray[numpy.float64[3, 1]]"""
    @property
    def tree_center_key(self) -> OctreeKey:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseOctreeD) -> erl_geometry.pyerl_geometry.OctreeKey"""
    @property
    def tree_depth(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseOctreeD) -> int"""
    @property
    def tree_max_half_size(self) -> numpy.ndarray[numpy.float64[3, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseOctreeD) -> numpy.ndarray[numpy.float64[3, 1]]"""
    @property
    def vertex_keys(self) -> list[OctreeKey]:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseOctreeD) -> list[erl_geometry.pyerl_geometry.OctreeKey]"""
    @property
    def vertices(self):
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseOctreeD) -> list[Annotated[list[int], FixedSize(8)]]"""
    @property
    def voxels(self):
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseOctreeD) -> list[Annotated[list[int], FixedSize(4)]]"""

class SemiSparseOctreeF(AbstractOctreeF):
    class BottomLeafNeighborIterator(SemiSparseOctreeF.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class Drawer:
        def __init__(self, setting, octree: SemiSparseOctreeF = ...) -> None:
            """__init__(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF.Drawer, setting: erl::geometry::AbstractOctreeDrawer::Setting, octree: erl_geometry.pyerl_geometry.SemiSparseOctreeF = None) -> None"""
        @staticmethod
        def Setting(*args, **kwargs):
            """Setting() -> erl::geometry::AbstractOctreeDrawer::Setting"""
        def draw_leaves(self, filename: str) -> None:
            """draw_leaves(*args, **kwargs)
            Overloaded function.

            1. draw_leaves(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF.Drawer) -> list[open3d::geometry::Geometry]

            2. draw_leaves(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF.Drawer, filename: str) -> None
            """
        def draw_tree(self, filename: str) -> None:
            """draw_tree(*args, **kwargs)
            Overloaded function.

            1. draw_tree(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF.Drawer) -> list[open3d::geometry::Geometry]

            2. draw_tree(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF.Drawer, filename: str) -> None
            """
        def set_draw_leaf_callback(self, callback) -> None:
            """set_draw_leaf_callback(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF.Drawer, callback: Callable[[erl_geometry.pyerl_geometry.SemiSparseOctreeF.Drawer, list[open3d::geometry::Geometry], erl_geometry.pyerl_geometry.SemiSparseOctreeF.LeafInAabbIterator], None]) -> None"""
        def set_draw_tree_callback(self, callback) -> None:
            """set_draw_tree_callback(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF.Drawer, callback: Callable[[erl_geometry.pyerl_geometry.SemiSparseOctreeF.Drawer, list[open3d::geometry::Geometry], erl_geometry.pyerl_geometry.SemiSparseOctreeF.TreeInAabbIterator], None]) -> None"""
        @property
        def setting(self):
            """(arg0: erl_geometry.pyerl_geometry.SemiSparseOctreeF.Drawer) -> erl::geometry::AbstractOctreeDrawer::Setting"""

    class EastLeafNeighborIterator(SemiSparseOctreeF.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class IteratorBase(AbstractOctreeF.OctreeNodeIterator):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""
        def __eq__(self, arg0: SemiSparseOctreeF.IteratorBase) -> bool:
            """__eq__(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF.IteratorBase, arg0: erl_geometry.pyerl_geometry.SemiSparseOctreeF.IteratorBase) -> bool"""
        def __ne__(self, arg0: SemiSparseOctreeF.IteratorBase) -> bool:
            """__ne__(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF.IteratorBase, arg0: erl_geometry.pyerl_geometry.SemiSparseOctreeF.IteratorBase) -> bool"""
        @property
        def index_key(self) -> OctreeKey:
            """(arg0: erl_geometry.pyerl_geometry.SemiSparseOctreeF.IteratorBase) -> erl_geometry.pyerl_geometry.OctreeKey"""
        @property
        def key(self) -> OctreeKey:
            """(arg0: erl_geometry.pyerl_geometry.SemiSparseOctreeF.IteratorBase) -> erl_geometry.pyerl_geometry.OctreeKey"""
        @property
        def node(self) -> SemiSparseOctreeNode:
            """(arg0: erl_geometry.pyerl_geometry.SemiSparseOctreeF.IteratorBase) -> erl_geometry.pyerl_geometry.SemiSparseOctreeNode"""
        @property
        def node_aabb(self) -> Aabb3Df:
            """(arg0: erl_geometry.pyerl_geometry.SemiSparseOctreeF.IteratorBase) -> erl_geometry.pyerl_geometry.Aabb3Df"""

    class LeafInAabbIterator(SemiSparseOctreeF.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class LeafIterator(SemiSparseOctreeF.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class LeafOfNodeIterator(SemiSparseOctreeF.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class NodeOnRayIterator(SemiSparseOctreeF.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""
        @property
        def distance(self) -> float:
            """(arg0: erl_geometry.pyerl_geometry.SemiSparseOctreeF.NodeOnRayIterator) -> float"""

    class NorthLeafNeighborIterator(SemiSparseOctreeF.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class SouthLeafNeighborIterator(SemiSparseOctreeF.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class TopLeafNeighborIterator(SemiSparseOctreeF.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class TreeInAabbIterator(SemiSparseOctreeF.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class TreeIterator(SemiSparseOctreeF.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class WestLeafNeighborIterator(SemiSparseOctreeF.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""
    @overload
    def __init__(self) -> None:
        """__init__(*args, **kwargs)
        Overloaded function.

        1. __init__(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF) -> None

        2. __init__(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, setting: erl_geometry.pyerl_geometry.SemiSparseNdTreeSetting) -> None

        3. __init__(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, filename: str) -> None
        """
    @overload
    def __init__(self, setting: SemiSparseNdTreeSetting) -> None:
        """__init__(*args, **kwargs)
        Overloaded function.

        1. __init__(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF) -> None

        2. __init__(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, setting: erl_geometry.pyerl_geometry.SemiSparseNdTreeSetting) -> None

        3. __init__(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, filename: str) -> None
        """
    @overload
    def __init__(self, filename: str) -> None:
        """__init__(*args, **kwargs)
        Overloaded function.

        1. __init__(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF) -> None

        2. __init__(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, setting: erl_geometry.pyerl_geometry.SemiSparseNdTreeSetting) -> None

        3. __init__(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, filename: str) -> None
        """
    def Setting(self) -> SemiSparseNdTreeSetting:
        """Setting() -> erl_geometry.pyerl_geometry.SemiSparseNdTreeSetting"""
    @overload
    def adjust_key_to_depth(self, key: int, depth: int) -> int:
        """adjust_key_to_depth(*args, **kwargs)
        Overloaded function.

        1. adjust_key_to_depth(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, key: int, depth: int) -> int

        2. adjust_key_to_depth(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> erl_geometry.pyerl_geometry.OctreeKey
        """
    @overload
    def adjust_key_to_depth(self, key: OctreeKey, depth: int) -> OctreeKey:
        """adjust_key_to_depth(*args, **kwargs)
        Overloaded function.

        1. adjust_key_to_depth(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, key: int, depth: int) -> int

        2. adjust_key_to_depth(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> erl_geometry.pyerl_geometry.OctreeKey
        """
    def clear(self) -> None:
        """clear(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF) -> None"""
    def compute_bottom_neighbor_key(self, key: OctreeKey, depth: int) -> OctreeKey | None:
        """compute_bottom_neighbor_key(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]"""
    def compute_common_ancestor_key(self, arg0: OctreeKey, arg1: OctreeKey) -> tuple[OctreeKey, int]:
        """compute_common_ancestor_key(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, arg0: erl_geometry.pyerl_geometry.OctreeKey, arg1: erl_geometry.pyerl_geometry.OctreeKey) -> tuple[erl_geometry.pyerl_geometry.OctreeKey, int]"""
    def compute_east_neighbor_key(self, key: OctreeKey, depth: int) -> OctreeKey | None:
        """compute_east_neighbor_key(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]"""
    def compute_north_neighbor_key(self, key: OctreeKey, depth: int) -> OctreeKey | None:
        """compute_north_neighbor_key(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]"""
    def compute_ray_coords(self, sx: float, sy: float, sz: float, ex: float, ey: float, ez: float) -> list[numpy.ndarray[numpy.float32[3, 1]]] | None:
        """compute_ray_coords(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, sx: float, sy: float, sz: float, ex: float, ey: float, ez: float) -> Optional[list[numpy.ndarray[numpy.float32[3, 1]]]]"""
    def compute_ray_keys(self, sx: float, sy: float, sz: float, ex: float, ey: float, ez: float) -> list[OctreeKey] | None:
        """compute_ray_keys(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, sx: float, sy: float, sz: float, ex: float, ey: float, ez: float) -> Optional[list[erl_geometry.pyerl_geometry.OctreeKey]]"""
    def compute_south_neighbor_key(self, key: OctreeKey, depth: int) -> OctreeKey | None:
        """compute_south_neighbor_key(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]"""
    def compute_top_neighbor_key(self, key: OctreeKey, depth: int) -> OctreeKey | None:
        """compute_top_neighbor_key(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]"""
    def compute_west_neighbor_key(self, key: OctreeKey, depth: int) -> OctreeKey | None:
        """compute_west_neighbor_key(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]"""
    @overload
    def coord_to_key(self, coordinate: float) -> int:
        """coord_to_key(*args, **kwargs)
        Overloaded function.

        1. coord_to_key(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, coordinate: float) -> int

        2. coord_to_key(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, coordinate: float, depth: int) -> int

        3. coord_to_key(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, x: float, y: float, z: float) -> erl_geometry.pyerl_geometry.OctreeKey

        4. coord_to_key(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, x: float, y: float, z: float, depth: int) -> erl_geometry.pyerl_geometry.OctreeKey
        """
    @overload
    def coord_to_key(self, coordinate: float, depth: int) -> int:
        """coord_to_key(*args, **kwargs)
        Overloaded function.

        1. coord_to_key(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, coordinate: float) -> int

        2. coord_to_key(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, coordinate: float, depth: int) -> int

        3. coord_to_key(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, x: float, y: float, z: float) -> erl_geometry.pyerl_geometry.OctreeKey

        4. coord_to_key(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, x: float, y: float, z: float, depth: int) -> erl_geometry.pyerl_geometry.OctreeKey
        """
    @overload
    def coord_to_key(self, x: float, y: float, z: float) -> OctreeKey:
        """coord_to_key(*args, **kwargs)
        Overloaded function.

        1. coord_to_key(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, coordinate: float) -> int

        2. coord_to_key(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, coordinate: float, depth: int) -> int

        3. coord_to_key(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, x: float, y: float, z: float) -> erl_geometry.pyerl_geometry.OctreeKey

        4. coord_to_key(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, x: float, y: float, z: float, depth: int) -> erl_geometry.pyerl_geometry.OctreeKey
        """
    @overload
    def coord_to_key(self, x: float, y: float, z: float, depth: int) -> OctreeKey:
        """coord_to_key(*args, **kwargs)
        Overloaded function.

        1. coord_to_key(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, coordinate: float) -> int

        2. coord_to_key(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, coordinate: float, depth: int) -> int

        3. coord_to_key(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, x: float, y: float, z: float) -> erl_geometry.pyerl_geometry.OctreeKey

        4. coord_to_key(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, x: float, y: float, z: float, depth: int) -> erl_geometry.pyerl_geometry.OctreeKey
        """
    @overload
    def coord_to_key_checked(self, coordinate: float) -> int | None:
        """coord_to_key_checked(*args, **kwargs)
        Overloaded function.

        1. coord_to_key_checked(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, coordinate: float) -> Optional[int]

        2. coord_to_key_checked(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, coordinate: float, depth: int) -> Optional[int]

        3. coord_to_key_checked(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, x: float, y: float, z: float) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]

        4. coord_to_key_checked(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, x: float, y: float, z: float, depth: int) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]
        """
    @overload
    def coord_to_key_checked(self, coordinate: float, depth: int) -> int | None:
        """coord_to_key_checked(*args, **kwargs)
        Overloaded function.

        1. coord_to_key_checked(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, coordinate: float) -> Optional[int]

        2. coord_to_key_checked(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, coordinate: float, depth: int) -> Optional[int]

        3. coord_to_key_checked(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, x: float, y: float, z: float) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]

        4. coord_to_key_checked(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, x: float, y: float, z: float, depth: int) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]
        """
    @overload
    def coord_to_key_checked(self, x: float, y: float, z: float) -> OctreeKey | None:
        """coord_to_key_checked(*args, **kwargs)
        Overloaded function.

        1. coord_to_key_checked(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, coordinate: float) -> Optional[int]

        2. coord_to_key_checked(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, coordinate: float, depth: int) -> Optional[int]

        3. coord_to_key_checked(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, x: float, y: float, z: float) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]

        4. coord_to_key_checked(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, x: float, y: float, z: float, depth: int) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]
        """
    @overload
    def coord_to_key_checked(self, x: float, y: float, z: float, depth: int) -> OctreeKey | None:
        """coord_to_key_checked(*args, **kwargs)
        Overloaded function.

        1. coord_to_key_checked(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, coordinate: float) -> Optional[int]

        2. coord_to_key_checked(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, coordinate: float, depth: int) -> Optional[int]

        3. coord_to_key_checked(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, x: float, y: float, z: float) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]

        4. coord_to_key_checked(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, x: float, y: float, z: float, depth: int) -> Optional[erl_geometry.pyerl_geometry.OctreeKey]
        """
    def create_node_child(self, node: SemiSparseOctreeNode, child_idx: int) -> SemiSparseOctreeNode:
        """create_node_child(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, node: erl_geometry.pyerl_geometry.SemiSparseOctreeNode, child_idx: int) -> erl_geometry.pyerl_geometry.SemiSparseOctreeNode"""
    @overload
    def delete_node(self, x: float, y: float, z: float, depth: int) -> int:
        """delete_node(*args, **kwargs)
        Overloaded function.

        1. delete_node(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, x: float, y: float, z: float, depth: int) -> int

        2. delete_node(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> None
        """
    @overload
    def delete_node(self, key: OctreeKey, depth: int) -> None:
        """delete_node(*args, **kwargs)
        Overloaded function.

        1. delete_node(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, x: float, y: float, z: float, depth: int) -> int

        2. delete_node(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> None
        """
    def delete_node_child(self, node: SemiSparseOctreeNode, child_idx: int, key: OctreeKey) -> int:
        """delete_node_child(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, node: erl_geometry.pyerl_geometry.SemiSparseOctreeNode, child_idx: int, key: erl_geometry.pyerl_geometry.OctreeKey) -> int"""
    def expand(self) -> None:
        """expand(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF) -> None"""
    def expand_node(self, node: SemiSparseOctreeNode) -> None:
        """expand_node(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, node: erl_geometry.pyerl_geometry.SemiSparseOctreeNode) -> None"""
    def find_voxel_index(self, key: OctreeKey) -> int:
        """find_voxel_index(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey) -> int"""
    def find_voxel_indices(self, points: numpy.ndarray[numpy.float32[3, n]], parallel: bool) -> list[int]:
        """find_voxel_indices(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, points: numpy.ndarray[numpy.float32[3, n]], parallel: bool) -> list[int]"""
    def get_node_child(self, node: SemiSparseOctreeNode, child_idx: int) -> SemiSparseOctreeNode:
        """get_node_child(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, node: erl_geometry.pyerl_geometry.SemiSparseOctreeNode, child_idx: int) -> erl_geometry.pyerl_geometry.SemiSparseOctreeNode"""
    def get_node_size(self, depth: int) -> float:
        """get_node_size(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, depth: int) -> float"""
    @overload
    def insert_node(self, x: float, y: float, z: float, depth: int) -> SemiSparseOctreeNode:
        """insert_node(*args, **kwargs)
        Overloaded function.

        1. insert_node(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, x: float, y: float, z: float, depth: int) -> erl_geometry.pyerl_geometry.SemiSparseOctreeNode

        2. insert_node(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> erl_geometry.pyerl_geometry.SemiSparseOctreeNode
        """
    @overload
    def insert_node(self, key: OctreeKey, depth: int) -> SemiSparseOctreeNode:
        """insert_node(*args, **kwargs)
        Overloaded function.

        1. insert_node(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, x: float, y: float, z: float, depth: int) -> erl_geometry.pyerl_geometry.SemiSparseOctreeNode

        2. insert_node(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> erl_geometry.pyerl_geometry.SemiSparseOctreeNode
        """
    def insert_point(self, key: OctreeKey, max_depth: int) -> int:
        """insert_point(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, max_depth: int) -> int"""
    def insert_points(self, points: numpy.ndarray[numpy.float32[3, n]]) -> list[int]:
        """insert_points(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, points: numpy.ndarray[numpy.float32[3, n]]) -> list[int]"""
    def is_node_collapsible(self, node: SemiSparseOctreeNode) -> bool:
        """is_node_collapsible(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, node: erl_geometry.pyerl_geometry.SemiSparseOctreeNode) -> bool"""
    @overload
    def iter_bottom_leaf_neighbor(self, x: float, y: float, z: float, max_leaf_depth: int = ...) -> Iterator[SemiSparseOctreeF.BottomLeafNeighborIterator]:
        """iter_bottom_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_bottom_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeF.BottomLeafNeighborIterator]

        2. iter_bottom_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeF.BottomLeafNeighborIterator]
        """
    @overload
    def iter_bottom_leaf_neighbor(self, key: OctreeKey, key_depth: int, max_leaf_depth: int = ...) -> Iterator[SemiSparseOctreeF.BottomLeafNeighborIterator]:
        """iter_bottom_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_bottom_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeF.BottomLeafNeighborIterator]

        2. iter_bottom_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeF.BottomLeafNeighborIterator]
        """
    @overload
    def iter_east_leaf_neighbor(self, x: float, y: float, z: float, max_leaf_depth: int = ...) -> Iterator[SemiSparseOctreeF.EastLeafNeighborIterator]:
        """iter_east_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_east_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeF.EastLeafNeighborIterator]

        2. iter_east_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeF.EastLeafNeighborIterator]
        """
    @overload
    def iter_east_leaf_neighbor(self, key: OctreeKey, key_depth: int, max_leaf_depth: int = ...) -> Iterator[SemiSparseOctreeF.EastLeafNeighborIterator]:
        """iter_east_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_east_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeF.EastLeafNeighborIterator]

        2. iter_east_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeF.EastLeafNeighborIterator]
        """
    def iter_leaf(self, max_depth: int = ...) -> Iterator[SemiSparseOctreeF.LeafIterator]:
        """iter_leaf(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeF.LeafIterator]"""
    @overload
    def iter_leaf_in_aabb(self, aabb_min_x: float, aabb_min_y: float, aabb_min_z: float, aabb_max_x: float, aabb_max_y: float, aabb_max_z: float, max_depth: int = ...) -> Iterator[SemiSparseOctreeF.LeafInAabbIterator]:
        """iter_leaf_in_aabb(*args, **kwargs)
        Overloaded function.

        1. iter_leaf_in_aabb(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, aabb_min_x: float, aabb_min_y: float, aabb_min_z: float, aabb_max_x: float, aabb_max_y: float, aabb_max_z: float, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeF.LeafInAabbIterator]

        2. iter_leaf_in_aabb(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, aabb_min_key: erl_geometry.pyerl_geometry.OctreeKey, aabb_max_key: erl_geometry.pyerl_geometry.OctreeKey, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeF.LeafInAabbIterator]
        """
    @overload
    def iter_leaf_in_aabb(self, aabb_min_key: OctreeKey, aabb_max_key: OctreeKey, max_depth: int = ...) -> Iterator[SemiSparseOctreeF.LeafInAabbIterator]:
        """iter_leaf_in_aabb(*args, **kwargs)
        Overloaded function.

        1. iter_leaf_in_aabb(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, aabb_min_x: float, aabb_min_y: float, aabb_min_z: float, aabb_max_x: float, aabb_max_y: float, aabb_max_z: float, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeF.LeafInAabbIterator]

        2. iter_leaf_in_aabb(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, aabb_min_key: erl_geometry.pyerl_geometry.OctreeKey, aabb_max_key: erl_geometry.pyerl_geometry.OctreeKey, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeF.LeafInAabbIterator]
        """
    def iter_leaf_of_node(self, node_key: OctreeKey, node_depth: int, max_depth: int = ...) -> Iterator[SemiSparseOctreeF.LeafOfNodeIterator]:
        """iter_leaf_of_node(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, node_key: erl_geometry.pyerl_geometry.OctreeKey, node_depth: int, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeF.LeafOfNodeIterator]"""
    def iter_node(self, max_depth: int = ...) -> Iterator[SemiSparseOctreeF.TreeIterator]:
        """iter_node(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeF.TreeIterator]"""
    @overload
    def iter_node_in_aabb(self, aabb_min_x: float, aabb_min_y: float, aabb_min_z: float, aabb_max_x: float, aabb_max_y: float, aabb_max_z: float, max_depth: int = ...) -> Iterator[SemiSparseOctreeF.TreeInAabbIterator]:
        """iter_node_in_aabb(*args, **kwargs)
        Overloaded function.

        1. iter_node_in_aabb(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, aabb_min_x: float, aabb_min_y: float, aabb_min_z: float, aabb_max_x: float, aabb_max_y: float, aabb_max_z: float, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeF.TreeInAabbIterator]

        2. iter_node_in_aabb(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, aabb_min_key: erl_geometry.pyerl_geometry.OctreeKey, aabb_max_key: erl_geometry.pyerl_geometry.OctreeKey, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeF.TreeInAabbIterator]
        """
    @overload
    def iter_node_in_aabb(self, aabb_min_key: OctreeKey, aabb_max_key: OctreeKey, max_depth: int = ...) -> Iterator[SemiSparseOctreeF.TreeInAabbIterator]:
        """iter_node_in_aabb(*args, **kwargs)
        Overloaded function.

        1. iter_node_in_aabb(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, aabb_min_x: float, aabb_min_y: float, aabb_min_z: float, aabb_max_x: float, aabb_max_y: float, aabb_max_z: float, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeF.TreeInAabbIterator]

        2. iter_node_in_aabb(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, aabb_min_key: erl_geometry.pyerl_geometry.OctreeKey, aabb_max_key: erl_geometry.pyerl_geometry.OctreeKey, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeF.TreeInAabbIterator]
        """
    def iter_node_on_ray(self, px: float, py: float, pz: float, vx: float, vy: float, vz: float, max_range: float = ..., node_padding: float = ..., bidirectional: bool = ..., leaf_only: bool = ..., min_node_depth: int = ..., max_node_depth: int = ...) -> Iterator[SemiSparseOctreeF.NodeOnRayIterator]:
        """iter_node_on_ray(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, px: float, py: float, pz: float, vx: float, vy: float, vz: float, max_range: float = -1, node_padding: float = 0, bidirectional: bool = False, leaf_only: bool = True, min_node_depth: int = 0, max_node_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeF.NodeOnRayIterator]"""
    @overload
    def iter_north_leaf_neighbor(self, x: float, y: float, z: float, max_leaf_depth: int = ...) -> Iterator[SemiSparseOctreeF.NorthLeafNeighborIterator]:
        """iter_north_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_north_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeF.NorthLeafNeighborIterator]

        2. iter_north_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeF.NorthLeafNeighborIterator]
        """
    @overload
    def iter_north_leaf_neighbor(self, key: OctreeKey, key_depth: int, max_leaf_depth: int = ...) -> Iterator[SemiSparseOctreeF.NorthLeafNeighborIterator]:
        """iter_north_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_north_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeF.NorthLeafNeighborIterator]

        2. iter_north_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeF.NorthLeafNeighborIterator]
        """
    @overload
    def iter_south_leaf_neighbor(self, x: float, y: float, z: float, max_leaf_depth: int = ...) -> Iterator[SemiSparseOctreeF.SouthLeafNeighborIterator]:
        """iter_south_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_south_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeF.SouthLeafNeighborIterator]

        2. iter_south_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeF.SouthLeafNeighborIterator]
        """
    @overload
    def iter_south_leaf_neighbor(self, key: OctreeKey, key_depth: int, max_leaf_depth: int = ...) -> Iterator[SemiSparseOctreeF.SouthLeafNeighborIterator]:
        """iter_south_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_south_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeF.SouthLeafNeighborIterator]

        2. iter_south_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeF.SouthLeafNeighborIterator]
        """
    @overload
    def iter_top_leaf_neighbor(self, x: float, y: float, z: float, max_leaf_depth: int = ...) -> Iterator[SemiSparseOctreeF.TopLeafNeighborIterator]:
        """iter_top_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_top_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeF.TopLeafNeighborIterator]

        2. iter_top_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeF.TopLeafNeighborIterator]
        """
    @overload
    def iter_top_leaf_neighbor(self, key: OctreeKey, key_depth: int, max_leaf_depth: int = ...) -> Iterator[SemiSparseOctreeF.TopLeafNeighborIterator]:
        """iter_top_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_top_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeF.TopLeafNeighborIterator]

        2. iter_top_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeF.TopLeafNeighborIterator]
        """
    @overload
    def iter_west_leaf_neighbor(self, x: float, y: float, z: float, max_leaf_depth: int = ...) -> Iterator[SemiSparseOctreeF.WestLeafNeighborIterator]:
        """iter_west_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_west_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeF.WestLeafNeighborIterator]

        2. iter_west_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeF.WestLeafNeighborIterator]
        """
    @overload
    def iter_west_leaf_neighbor(self, key: OctreeKey, key_depth: int, max_leaf_depth: int = ...) -> Iterator[SemiSparseOctreeF.WestLeafNeighborIterator]:
        """iter_west_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_west_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, x: float, y: float, z: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeF.WestLeafNeighborIterator]

        2. iter_west_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseOctreeF.WestLeafNeighborIterator]
        """
    @overload
    def key_to_coord(self, key: int) -> float:
        """key_to_coord(*args, **kwargs)
        Overloaded function.

        1. key_to_coord(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, key: int) -> float

        2. key_to_coord(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, key: int, depth: int) -> float

        3. key_to_coord(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey) -> tuple[float, float, float]

        4. key_to_coord(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> tuple[float, float, float]
        """
    @overload
    def key_to_coord(self, key: int, depth: int) -> float:
        """key_to_coord(*args, **kwargs)
        Overloaded function.

        1. key_to_coord(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, key: int) -> float

        2. key_to_coord(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, key: int, depth: int) -> float

        3. key_to_coord(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey) -> tuple[float, float, float]

        4. key_to_coord(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> tuple[float, float, float]
        """
    @overload
    def key_to_coord(self, key: OctreeKey) -> tuple[float, float, float]:
        """key_to_coord(*args, **kwargs)
        Overloaded function.

        1. key_to_coord(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, key: int) -> float

        2. key_to_coord(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, key: int, depth: int) -> float

        3. key_to_coord(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey) -> tuple[float, float, float]

        4. key_to_coord(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> tuple[float, float, float]
        """
    @overload
    def key_to_coord(self, key: OctreeKey, depth: int) -> tuple[float, float, float]:
        """key_to_coord(*args, **kwargs)
        Overloaded function.

        1. key_to_coord(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, key: int) -> float

        2. key_to_coord(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, key: int, depth: int) -> float

        3. key_to_coord(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey) -> tuple[float, float, float]

        4. key_to_coord(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, depth: int) -> tuple[float, float, float]
        """
    def prune(self) -> None:
        """prune(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF) -> None"""
    def prune_node(self, node: SemiSparseOctreeNode) -> bool:
        """prune_node(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, node: erl_geometry.pyerl_geometry.SemiSparseOctreeNode) -> bool"""
    @overload
    def search(self, x: float, y: float, z: float, max_depth: int = ...) -> SemiSparseOctreeNode:
        """search(*args, **kwargs)
        Overloaded function.

        1. search(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, x: float, y: float, z: float, max_depth: int = 0) -> erl_geometry.pyerl_geometry.SemiSparseOctreeNode

        2. search(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, max_depth: int = 0) -> erl_geometry.pyerl_geometry.SemiSparseOctreeNode
        """
    @overload
    def search(self, key: OctreeKey, max_depth: int = ...) -> SemiSparseOctreeNode:
        """search(*args, **kwargs)
        Overloaded function.

        1. search(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, x: float, y: float, z: float, max_depth: int = 0) -> erl_geometry.pyerl_geometry.SemiSparseOctreeNode

        2. search(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, key: erl_geometry.pyerl_geometry.OctreeKey, max_depth: int = 0) -> erl_geometry.pyerl_geometry.SemiSparseOctreeNode
        """
    def visualize(self, leaf_only: bool = ..., scaling: float = ..., area_min: numpy.ndarray[numpy.float64[3, 1]] = ..., area_max: numpy.ndarray[numpy.float64[3, 1]] = ..., border_color: numpy.ndarray[numpy.float64[3, 1]] = ..., window_width: int = ..., window_height: int = ..., window_left: int = ..., window_top: int = ...) -> None:
        """visualize(self: erl_geometry.pyerl_geometry.SemiSparseOctreeF, leaf_only: bool = False, scaling: float = 1.0, area_min: numpy.ndarray[numpy.float64[3, 1]] = array([-1., -1., -1.], dtype=float32), area_max: numpy.ndarray[numpy.float64[3, 1]] = array([1., 1., 1.], dtype=float32), border_color: numpy.ndarray[numpy.float64[3, 1]] = array([0., 0., 0.], dtype=float32), window_width: int = 1920, window_height: int = 1080, window_left: int = 50, window_top: int = 50) -> None"""
    @property
    def children(self):
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseOctreeF) -> list[Annotated[list[int], FixedSize(8)]]"""
    @property
    def memory_usage(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseOctreeF) -> int"""
    @property
    def memory_usage_per_node(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseOctreeF) -> int"""
    @property
    def metric_aabb(self) -> Aabb3Df:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseOctreeF) -> erl_geometry.pyerl_geometry.Aabb3Df"""
    @property
    def metric_max(self) -> numpy.ndarray[numpy.float32[3, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseOctreeF) -> numpy.ndarray[numpy.float32[3, 1]]"""
    @property
    def metric_min(self) -> numpy.ndarray[numpy.float32[3, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseOctreeF) -> numpy.ndarray[numpy.float32[3, 1]]"""
    @property
    def metric_min_max(self) -> tuple[numpy.ndarray[numpy.float32[3, 1]], numpy.ndarray[numpy.float32[3, 1]]]:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseOctreeF) -> tuple[numpy.ndarray[numpy.float32[3, 1]], numpy.ndarray[numpy.float32[3, 1]]]"""
    @property
    def metric_size(self) -> numpy.ndarray[numpy.float32[3, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseOctreeF) -> numpy.ndarray[numpy.float32[3, 1]]"""
    @property
    def num_vertices(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseOctreeF) -> int"""
    @property
    def number_of_leaf_nodes(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseOctreeF) -> int"""
    @property
    def number_of_nodes(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseOctreeF) -> int"""
    @property
    def parents(self) -> list[int]:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseOctreeF) -> list[int]"""
    @property
    def resolution(self) -> float:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseOctreeF) -> float"""
    @property
    def root(self) -> SemiSparseOctreeNode:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseOctreeF) -> erl_geometry.pyerl_geometry.SemiSparseOctreeNode"""
    @property
    def setting(self) -> SemiSparseNdTreeSetting:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseOctreeF) -> erl_geometry.pyerl_geometry.SemiSparseNdTreeSetting"""
    @property
    def tree_center(self) -> numpy.ndarray[numpy.float32[3, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseOctreeF) -> numpy.ndarray[numpy.float32[3, 1]]"""
    @property
    def tree_center_key(self) -> OctreeKey:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseOctreeF) -> erl_geometry.pyerl_geometry.OctreeKey"""
    @property
    def tree_depth(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseOctreeF) -> int"""
    @property
    def tree_max_half_size(self) -> numpy.ndarray[numpy.float32[3, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseOctreeF) -> numpy.ndarray[numpy.float32[3, 1]]"""
    @property
    def vertex_keys(self) -> list[OctreeKey]:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseOctreeF) -> list[erl_geometry.pyerl_geometry.OctreeKey]"""
    @property
    def vertices(self):
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseOctreeF) -> list[Annotated[list[int], FixedSize(8)]]"""
    @property
    def voxels(self):
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseOctreeF) -> list[Annotated[list[int], FixedSize(4)]]"""

class SemiSparseOctreeNode(AbstractOctreeNode):
    def __init__(self, *args, **kwargs) -> None:
        """Initialize self.  See help(type(self)) for accurate signature."""
    @property
    def node_index(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseOctreeNode) -> int"""

class SemiSparseQuadtreeD(AbstractQuadtreeD):
    class Drawer:
        def __init__(self, setting, quadtree: SemiSparseQuadtreeD = ...) -> None:
            """__init__(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD.Drawer, setting: erl::geometry::AbstractQuadtreeDrawer::Setting, quadtree: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD = None) -> None"""
        @staticmethod
        def Setting(*args, **kwargs):
            """Setting() -> erl::geometry::AbstractQuadtreeDrawer::Setting"""
        @overload
        def draw_leaves(self) -> Mat:
            """draw_leaves(*args, **kwargs)
            Overloaded function.

            1. draw_leaves(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD.Drawer) -> Mat

            2. draw_leaves(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD.Drawer, filename: str) -> None
            """
        @overload
        def draw_leaves(self, filename: str) -> None:
            """draw_leaves(*args, **kwargs)
            Overloaded function.

            1. draw_leaves(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD.Drawer) -> Mat

            2. draw_leaves(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD.Drawer, filename: str) -> None
            """
        @overload
        def draw_tree(self) -> Mat:
            """draw_tree(*args, **kwargs)
            Overloaded function.

            1. draw_tree(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD.Drawer) -> Mat

            2. draw_tree(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD.Drawer, filename: str) -> None
            """
        @overload
        def draw_tree(self, filename: str) -> None:
            """draw_tree(*args, **kwargs)
            Overloaded function.

            1. draw_tree(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD.Drawer) -> Mat

            2. draw_tree(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD.Drawer, filename: str) -> None
            """
        def set_draw_leaf_callback(self, callback: Callable[[SemiSparseQuadtreeD.Drawer, Mat, SemiSparseQuadtreeD.LeafInAabbIterator], None]) -> None:
            """set_draw_leaf_callback(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD.Drawer, callback: Callable[[erl_geometry.pyerl_geometry.SemiSparseQuadtreeD.Drawer, Mat, erl_geometry.pyerl_geometry.SemiSparseQuadtreeD.LeafInAabbIterator], None]) -> None"""
        def set_draw_tree_callback(self, callback: Callable[[SemiSparseQuadtreeD.Drawer, Mat, SemiSparseQuadtreeD.TreeInAabbIterator], None]) -> None:
            """set_draw_tree_callback(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD.Drawer, callback: Callable[[erl_geometry.pyerl_geometry.SemiSparseQuadtreeD.Drawer, Mat, erl_geometry.pyerl_geometry.SemiSparseQuadtreeD.TreeInAabbIterator], None]) -> None"""
        @property
        def grid_map_info(self):
            """(arg0: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD.Drawer) -> erl::common::GridMapInfo<float, 2>"""
        @property
        def setting(self):
            """(arg0: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD.Drawer) -> erl::geometry::AbstractQuadtreeDrawer::Setting"""

    class EastLeafNeighborIterator(SemiSparseQuadtreeD.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class IteratorBase(AbstractQuadtreeD.QuadtreeNodeIterator):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""
        def __eq__(self, arg0: SemiSparseQuadtreeD.IteratorBase) -> bool:
            """__eq__(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD.IteratorBase, arg0: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD.IteratorBase) -> bool"""
        def __ne__(self, arg0: SemiSparseQuadtreeD.IteratorBase) -> bool:
            """__ne__(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD.IteratorBase, arg0: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD.IteratorBase) -> bool"""
        @property
        def index_key(self) -> QuadtreeKey:
            """(arg0: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD.IteratorBase) -> erl_geometry.pyerl_geometry.QuadtreeKey"""
        @property
        def key(self) -> QuadtreeKey:
            """(arg0: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD.IteratorBase) -> erl_geometry.pyerl_geometry.QuadtreeKey"""
        @property
        def node(self) -> SemiSparseQuadtreeNode:
            """(arg0: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD.IteratorBase) -> erl_geometry.pyerl_geometry.SemiSparseQuadtreeNode"""
        @property
        def node_aabb(self) -> Aabb2Dd:
            """(arg0: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD.IteratorBase) -> erl_geometry.pyerl_geometry.Aabb2Dd"""

    class LeafInAabbIterator(SemiSparseQuadtreeD.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class LeafIterator(SemiSparseQuadtreeD.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class LeafOfNodeIterator(SemiSparseQuadtreeD.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class NodeOnRayIterator(SemiSparseQuadtreeD.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""
        @property
        def distance(self) -> float:
            """(arg0: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD.NodeOnRayIterator) -> float"""

    class NorthLeafNeighborIterator(SemiSparseQuadtreeD.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class SouthLeafNeighborIterator(SemiSparseQuadtreeD.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class TreeInAabbIterator(SemiSparseQuadtreeD.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class TreeIterator(SemiSparseQuadtreeD.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class WestLeafNeighborIterator(SemiSparseQuadtreeD.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""
    @overload
    def __init__(self) -> None:
        """__init__(*args, **kwargs)
        Overloaded function.

        1. __init__(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD) -> None

        2. __init__(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, setting: erl_geometry.pyerl_geometry.SemiSparseNdTreeSetting) -> None

        3. __init__(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, filename: str) -> None
        """
    @overload
    def __init__(self, setting: SemiSparseNdTreeSetting) -> None:
        """__init__(*args, **kwargs)
        Overloaded function.

        1. __init__(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD) -> None

        2. __init__(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, setting: erl_geometry.pyerl_geometry.SemiSparseNdTreeSetting) -> None

        3. __init__(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, filename: str) -> None
        """
    @overload
    def __init__(self, filename: str) -> None:
        """__init__(*args, **kwargs)
        Overloaded function.

        1. __init__(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD) -> None

        2. __init__(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, setting: erl_geometry.pyerl_geometry.SemiSparseNdTreeSetting) -> None

        3. __init__(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, filename: str) -> None
        """
    def Setting(self) -> SemiSparseNdTreeSetting:
        """Setting() -> erl_geometry.pyerl_geometry.SemiSparseNdTreeSetting"""
    @overload
    def adjust_key_to_depth(self, key: int, depth: int) -> int:
        """adjust_key_to_depth(*args, **kwargs)
        Overloaded function.

        1. adjust_key_to_depth(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, key: int, depth: int) -> int

        2. adjust_key_to_depth(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> erl_geometry.pyerl_geometry.QuadtreeKey
        """
    @overload
    def adjust_key_to_depth(self, key: QuadtreeKey, depth: int) -> QuadtreeKey:
        """adjust_key_to_depth(*args, **kwargs)
        Overloaded function.

        1. adjust_key_to_depth(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, key: int, depth: int) -> int

        2. adjust_key_to_depth(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> erl_geometry.pyerl_geometry.QuadtreeKey
        """
    def clear(self) -> None:
        """clear(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD) -> None"""
    def compute_common_ancestor_key(self, arg0: QuadtreeKey, arg1: QuadtreeKey) -> tuple[QuadtreeKey, int]:
        """compute_common_ancestor_key(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, arg0: erl_geometry.pyerl_geometry.QuadtreeKey, arg1: erl_geometry.pyerl_geometry.QuadtreeKey) -> tuple[erl_geometry.pyerl_geometry.QuadtreeKey, int]"""
    def compute_east_neighbor_key(self, key: QuadtreeKey, depth: int) -> QuadtreeKey | None:
        """compute_east_neighbor_key(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]"""
    def compute_north_neighbor_key(self, key: QuadtreeKey, depth: int) -> QuadtreeKey | None:
        """compute_north_neighbor_key(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]"""
    def compute_ray_coords(self, sx: float, sy: float, ex: float, ey: float) -> list[numpy.ndarray[numpy.float64[2, 1]]] | None:
        """compute_ray_coords(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, sx: float, sy: float, ex: float, ey: float) -> Optional[list[numpy.ndarray[numpy.float64[2, 1]]]]"""
    def compute_ray_keys(self, sx: float, sy: float, ex: float, ey: float) -> list[QuadtreeKey] | None:
        """compute_ray_keys(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, sx: float, sy: float, ex: float, ey: float) -> Optional[list[erl_geometry.pyerl_geometry.QuadtreeKey]]"""
    def compute_south_neighbor_key(self, key: QuadtreeKey, depth: int) -> QuadtreeKey | None:
        """compute_south_neighbor_key(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]"""
    def compute_west_neighbor_key(self, key: QuadtreeKey, depth: int) -> QuadtreeKey | None:
        """compute_west_neighbor_key(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]"""
    @overload
    def coord_to_key(self, coordinate: float) -> int:
        """coord_to_key(*args, **kwargs)
        Overloaded function.

        1. coord_to_key(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, coordinate: float) -> int

        2. coord_to_key(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, coordinate: float, depth: int) -> int

        3. coord_to_key(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, x: float, y: float) -> erl_geometry.pyerl_geometry.QuadtreeKey

        4. coord_to_key(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, x: float, y: float, depth: int) -> erl_geometry.pyerl_geometry.QuadtreeKey
        """
    @overload
    def coord_to_key(self, coordinate: float, depth: int) -> int:
        """coord_to_key(*args, **kwargs)
        Overloaded function.

        1. coord_to_key(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, coordinate: float) -> int

        2. coord_to_key(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, coordinate: float, depth: int) -> int

        3. coord_to_key(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, x: float, y: float) -> erl_geometry.pyerl_geometry.QuadtreeKey

        4. coord_to_key(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, x: float, y: float, depth: int) -> erl_geometry.pyerl_geometry.QuadtreeKey
        """
    @overload
    def coord_to_key(self, x: float, y: float) -> QuadtreeKey:
        """coord_to_key(*args, **kwargs)
        Overloaded function.

        1. coord_to_key(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, coordinate: float) -> int

        2. coord_to_key(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, coordinate: float, depth: int) -> int

        3. coord_to_key(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, x: float, y: float) -> erl_geometry.pyerl_geometry.QuadtreeKey

        4. coord_to_key(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, x: float, y: float, depth: int) -> erl_geometry.pyerl_geometry.QuadtreeKey
        """
    @overload
    def coord_to_key(self, x: float, y: float, depth: int) -> QuadtreeKey:
        """coord_to_key(*args, **kwargs)
        Overloaded function.

        1. coord_to_key(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, coordinate: float) -> int

        2. coord_to_key(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, coordinate: float, depth: int) -> int

        3. coord_to_key(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, x: float, y: float) -> erl_geometry.pyerl_geometry.QuadtreeKey

        4. coord_to_key(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, x: float, y: float, depth: int) -> erl_geometry.pyerl_geometry.QuadtreeKey
        """
    @overload
    def coord_to_key_checked(self, coordinate: float) -> int | None:
        """coord_to_key_checked(*args, **kwargs)
        Overloaded function.

        1. coord_to_key_checked(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, coordinate: float) -> Optional[int]

        2. coord_to_key_checked(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, coordinate: float, depth: int) -> Optional[int]

        3. coord_to_key_checked(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, x: float, y: float) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]

        4. coord_to_key_checked(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, x: float, y: float, depth: int) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]
        """
    @overload
    def coord_to_key_checked(self, coordinate: float, depth: int) -> int | None:
        """coord_to_key_checked(*args, **kwargs)
        Overloaded function.

        1. coord_to_key_checked(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, coordinate: float) -> Optional[int]

        2. coord_to_key_checked(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, coordinate: float, depth: int) -> Optional[int]

        3. coord_to_key_checked(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, x: float, y: float) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]

        4. coord_to_key_checked(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, x: float, y: float, depth: int) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]
        """
    @overload
    def coord_to_key_checked(self, x: float, y: float) -> QuadtreeKey | None:
        """coord_to_key_checked(*args, **kwargs)
        Overloaded function.

        1. coord_to_key_checked(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, coordinate: float) -> Optional[int]

        2. coord_to_key_checked(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, coordinate: float, depth: int) -> Optional[int]

        3. coord_to_key_checked(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, x: float, y: float) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]

        4. coord_to_key_checked(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, x: float, y: float, depth: int) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]
        """
    @overload
    def coord_to_key_checked(self, x: float, y: float, depth: int) -> QuadtreeKey | None:
        """coord_to_key_checked(*args, **kwargs)
        Overloaded function.

        1. coord_to_key_checked(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, coordinate: float) -> Optional[int]

        2. coord_to_key_checked(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, coordinate: float, depth: int) -> Optional[int]

        3. coord_to_key_checked(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, x: float, y: float) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]

        4. coord_to_key_checked(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, x: float, y: float, depth: int) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]
        """
    def create_node_child(self, node: SemiSparseQuadtreeNode, child_idx: int) -> SemiSparseQuadtreeNode:
        """create_node_child(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, node: erl_geometry.pyerl_geometry.SemiSparseQuadtreeNode, child_idx: int) -> erl_geometry.pyerl_geometry.SemiSparseQuadtreeNode"""
    @overload
    def delete_node(self, x: float, y: float, depth: int) -> int:
        """delete_node(*args, **kwargs)
        Overloaded function.

        1. delete_node(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, x: float, y: float, depth: int) -> int

        2. delete_node(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> None
        """
    @overload
    def delete_node(self, key: QuadtreeKey, depth: int) -> None:
        """delete_node(*args, **kwargs)
        Overloaded function.

        1. delete_node(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, x: float, y: float, depth: int) -> int

        2. delete_node(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> None
        """
    def delete_node_child(self, node: SemiSparseQuadtreeNode, child_idx: int, key: QuadtreeKey) -> int:
        """delete_node_child(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, node: erl_geometry.pyerl_geometry.SemiSparseQuadtreeNode, child_idx: int, key: erl_geometry.pyerl_geometry.QuadtreeKey) -> int"""
    def expand(self) -> None:
        """expand(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD) -> None"""
    def expand_node(self, node: SemiSparseQuadtreeNode) -> None:
        """expand_node(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, node: erl_geometry.pyerl_geometry.SemiSparseQuadtreeNode) -> None"""
    def find_voxel_index(self, key: QuadtreeKey) -> int:
        """find_voxel_index(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey) -> int"""
    def find_voxel_indices(self, points: numpy.ndarray[numpy.float64[2, n]], parallel: bool) -> list[int]:
        """find_voxel_indices(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, points: numpy.ndarray[numpy.float64[2, n]], parallel: bool) -> list[int]"""
    def get_node_child(self, node: SemiSparseQuadtreeNode, child_idx: int) -> SemiSparseQuadtreeNode:
        """get_node_child(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, node: erl_geometry.pyerl_geometry.SemiSparseQuadtreeNode, child_idx: int) -> erl_geometry.pyerl_geometry.SemiSparseQuadtreeNode"""
    def get_node_size(self, depth: int) -> float:
        """get_node_size(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, depth: int) -> float"""
    @overload
    def insert_node(self, x: float, y: float, depth: int) -> SemiSparseQuadtreeNode:
        """insert_node(*args, **kwargs)
        Overloaded function.

        1. insert_node(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, x: float, y: float, depth: int) -> erl_geometry.pyerl_geometry.SemiSparseQuadtreeNode

        2. insert_node(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> erl_geometry.pyerl_geometry.SemiSparseQuadtreeNode
        """
    @overload
    def insert_node(self, key: QuadtreeKey, depth: int) -> SemiSparseQuadtreeNode:
        """insert_node(*args, **kwargs)
        Overloaded function.

        1. insert_node(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, x: float, y: float, depth: int) -> erl_geometry.pyerl_geometry.SemiSparseQuadtreeNode

        2. insert_node(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> erl_geometry.pyerl_geometry.SemiSparseQuadtreeNode
        """
    def insert_point(self, key: QuadtreeKey, max_depth: int) -> int:
        """insert_point(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, max_depth: int) -> int"""
    def insert_points(self, points: numpy.ndarray[numpy.float64[2, n]]) -> list[int]:
        """insert_points(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, points: numpy.ndarray[numpy.float64[2, n]]) -> list[int]"""
    def is_node_collapsible(self, node: SemiSparseQuadtreeNode) -> bool:
        """is_node_collapsible(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, node: erl_geometry.pyerl_geometry.SemiSparseQuadtreeNode) -> bool"""
    @overload
    def iter_east_leaf_neighbor(self, x: float, y: float, max_leaf_depth: int = ...) -> Iterator[SemiSparseQuadtreeD.EastLeafNeighborIterator]:
        """iter_east_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_east_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, x: float, y: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseQuadtreeD.EastLeafNeighborIterator]

        2. iter_east_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseQuadtreeD.EastLeafNeighborIterator]
        """
    @overload
    def iter_east_leaf_neighbor(self, key: QuadtreeKey, key_depth: int, max_leaf_depth: int = ...) -> Iterator[SemiSparseQuadtreeD.EastLeafNeighborIterator]:
        """iter_east_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_east_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, x: float, y: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseQuadtreeD.EastLeafNeighborIterator]

        2. iter_east_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseQuadtreeD.EastLeafNeighborIterator]
        """
    def iter_leaf(self, max_depth: int = ...) -> Iterator[SemiSparseQuadtreeD.LeafIterator]:
        """iter_leaf(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseQuadtreeD.LeafIterator]"""
    @overload
    def iter_leaf_in_aabb(self, aabb_min_x: float, aabb_min_y: float, aabb_max_x: float, aabb_max_y: float, max_depth: int = ...) -> Iterator[SemiSparseQuadtreeD.LeafInAabbIterator]:
        """iter_leaf_in_aabb(*args, **kwargs)
        Overloaded function.

        1. iter_leaf_in_aabb(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, aabb_min_x: float, aabb_min_y: float, aabb_max_x: float, aabb_max_y: float, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseQuadtreeD.LeafInAabbIterator]

        2. iter_leaf_in_aabb(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, aabb_min_key: erl_geometry.pyerl_geometry.QuadtreeKey, aabb_max_key: erl_geometry.pyerl_geometry.QuadtreeKey, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseQuadtreeD.LeafInAabbIterator]
        """
    @overload
    def iter_leaf_in_aabb(self, aabb_min_key: QuadtreeKey, aabb_max_key: QuadtreeKey, max_depth: int = ...) -> Iterator[SemiSparseQuadtreeD.LeafInAabbIterator]:
        """iter_leaf_in_aabb(*args, **kwargs)
        Overloaded function.

        1. iter_leaf_in_aabb(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, aabb_min_x: float, aabb_min_y: float, aabb_max_x: float, aabb_max_y: float, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseQuadtreeD.LeafInAabbIterator]

        2. iter_leaf_in_aabb(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, aabb_min_key: erl_geometry.pyerl_geometry.QuadtreeKey, aabb_max_key: erl_geometry.pyerl_geometry.QuadtreeKey, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseQuadtreeD.LeafInAabbIterator]
        """
    def iter_leaf_of_node(self, node_key: QuadtreeKey, node_depth: int, max_depth: int = ...) -> Iterator[SemiSparseQuadtreeD.LeafOfNodeIterator]:
        """iter_leaf_of_node(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, node_key: erl_geometry.pyerl_geometry.QuadtreeKey, node_depth: int, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseQuadtreeD.LeafOfNodeIterator]"""
    def iter_node(self, max_depth: int = ...) -> Iterator[SemiSparseQuadtreeD.TreeIterator]:
        """iter_node(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseQuadtreeD.TreeIterator]"""
    @overload
    def iter_node_in_aabb(self, aabb_min_x: float, aabb_min_y: float, aabb_max_x: float, aabb_max_y: float, max_depth: int = ...) -> Iterator[SemiSparseQuadtreeD.TreeInAabbIterator]:
        """iter_node_in_aabb(*args, **kwargs)
        Overloaded function.

        1. iter_node_in_aabb(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, aabb_min_x: float, aabb_min_y: float, aabb_max_x: float, aabb_max_y: float, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseQuadtreeD.TreeInAabbIterator]

        2. iter_node_in_aabb(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, aabb_min_key: erl_geometry.pyerl_geometry.QuadtreeKey, aabb_max_key: erl_geometry.pyerl_geometry.QuadtreeKey, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseQuadtreeD.TreeInAabbIterator]
        """
    @overload
    def iter_node_in_aabb(self, aabb_min_key: QuadtreeKey, aabb_max_key: QuadtreeKey, max_depth: int = ...) -> Iterator[SemiSparseQuadtreeD.TreeInAabbIterator]:
        """iter_node_in_aabb(*args, **kwargs)
        Overloaded function.

        1. iter_node_in_aabb(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, aabb_min_x: float, aabb_min_y: float, aabb_max_x: float, aabb_max_y: float, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseQuadtreeD.TreeInAabbIterator]

        2. iter_node_in_aabb(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, aabb_min_key: erl_geometry.pyerl_geometry.QuadtreeKey, aabb_max_key: erl_geometry.pyerl_geometry.QuadtreeKey, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseQuadtreeD.TreeInAabbIterator]
        """
    def iter_node_on_ray(self, px: float, py: float, vx: float, vy: float, max_range: float = ..., node_padding: float = ..., bidirectional: bool = ..., leaf_only: bool = ..., min_node_depth: int = ..., max_node_depth: int = ...) -> Iterator[SemiSparseQuadtreeD.NodeOnRayIterator]:
        """iter_node_on_ray(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, px: float, py: float, vx: float, vy: float, max_range: float = -1, node_padding: float = 0, bidirectional: bool = False, leaf_only: bool = False, min_node_depth: int = 0, max_node_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseQuadtreeD.NodeOnRayIterator]"""
    @overload
    def iter_north_leaf_neighbor(self, x: float, y: float, max_leaf_depth: int = ...) -> Iterator[SemiSparseQuadtreeD.NorthLeafNeighborIterator]:
        """iter_north_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_north_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, x: float, y: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseQuadtreeD.NorthLeafNeighborIterator]

        2. iter_north_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseQuadtreeD.NorthLeafNeighborIterator]
        """
    @overload
    def iter_north_leaf_neighbor(self, key: QuadtreeKey, key_depth: int, max_leaf_depth: int = ...) -> Iterator[SemiSparseQuadtreeD.NorthLeafNeighborIterator]:
        """iter_north_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_north_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, x: float, y: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseQuadtreeD.NorthLeafNeighborIterator]

        2. iter_north_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseQuadtreeD.NorthLeafNeighborIterator]
        """
    @overload
    def iter_south_leaf_neighbor(self, x: float, y: float, max_leaf_depth: int = ...) -> Iterator[SemiSparseQuadtreeD.SouthLeafNeighborIterator]:
        """iter_south_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_south_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, x: float, y: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseQuadtreeD.SouthLeafNeighborIterator]

        2. iter_south_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseQuadtreeD.SouthLeafNeighborIterator]
        """
    @overload
    def iter_south_leaf_neighbor(self, key: QuadtreeKey, key_depth: int, max_leaf_depth: int = ...) -> Iterator[SemiSparseQuadtreeD.SouthLeafNeighborIterator]:
        """iter_south_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_south_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, x: float, y: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseQuadtreeD.SouthLeafNeighborIterator]

        2. iter_south_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseQuadtreeD.SouthLeafNeighborIterator]
        """
    @overload
    def iter_west_leaf_neighbor(self, x: float, y: float, max_leaf_depth: int = ...) -> Iterator[SemiSparseQuadtreeD.WestLeafNeighborIterator]:
        """iter_west_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_west_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, x: float, y: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseQuadtreeD.WestLeafNeighborIterator]

        2. iter_west_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseQuadtreeD.WestLeafNeighborIterator]
        """
    @overload
    def iter_west_leaf_neighbor(self, key: QuadtreeKey, key_depth: int, max_leaf_depth: int = ...) -> Iterator[SemiSparseQuadtreeD.WestLeafNeighborIterator]:
        """iter_west_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_west_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, x: float, y: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseQuadtreeD.WestLeafNeighborIterator]

        2. iter_west_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseQuadtreeD.WestLeafNeighborIterator]
        """
    @overload
    def key_to_coord(self, key: int) -> float:
        """key_to_coord(*args, **kwargs)
        Overloaded function.

        1. key_to_coord(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, key: int) -> float

        2. key_to_coord(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, key: int, depth: int) -> float

        3. key_to_coord(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey) -> tuple[float, float]

        4. key_to_coord(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> tuple[float, float]
        """
    @overload
    def key_to_coord(self, key: int, depth: int) -> float:
        """key_to_coord(*args, **kwargs)
        Overloaded function.

        1. key_to_coord(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, key: int) -> float

        2. key_to_coord(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, key: int, depth: int) -> float

        3. key_to_coord(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey) -> tuple[float, float]

        4. key_to_coord(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> tuple[float, float]
        """
    @overload
    def key_to_coord(self, key: QuadtreeKey) -> tuple[float, float]:
        """key_to_coord(*args, **kwargs)
        Overloaded function.

        1. key_to_coord(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, key: int) -> float

        2. key_to_coord(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, key: int, depth: int) -> float

        3. key_to_coord(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey) -> tuple[float, float]

        4. key_to_coord(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> tuple[float, float]
        """
    @overload
    def key_to_coord(self, key: QuadtreeKey, depth: int) -> tuple[float, float]:
        """key_to_coord(*args, **kwargs)
        Overloaded function.

        1. key_to_coord(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, key: int) -> float

        2. key_to_coord(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, key: int, depth: int) -> float

        3. key_to_coord(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey) -> tuple[float, float]

        4. key_to_coord(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> tuple[float, float]
        """
    def prune(self) -> None:
        """prune(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD) -> None"""
    def prune_node(self, node: SemiSparseQuadtreeNode) -> bool:
        """prune_node(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, node: erl_geometry.pyerl_geometry.SemiSparseQuadtreeNode) -> bool"""
    @overload
    def search(self, x: float, y: float, max_depth: int = ...) -> SemiSparseQuadtreeNode:
        """search(*args, **kwargs)
        Overloaded function.

        1. search(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, x: float, y: float, max_depth: int = 0) -> erl_geometry.pyerl_geometry.SemiSparseQuadtreeNode

        2. search(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, max_depth: int = 0) -> erl_geometry.pyerl_geometry.SemiSparseQuadtreeNode
        """
    @overload
    def search(self, key: QuadtreeKey, max_depth: int = ...) -> SemiSparseQuadtreeNode:
        """search(*args, **kwargs)
        Overloaded function.

        1. search(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, x: float, y: float, max_depth: int = 0) -> erl_geometry.pyerl_geometry.SemiSparseQuadtreeNode

        2. search(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, key: erl_geometry.pyerl_geometry.QuadtreeKey, max_depth: int = 0) -> erl_geometry.pyerl_geometry.SemiSparseQuadtreeNode
        """
    def visualize(self, leaf_only: bool = ..., area_min: numpy.ndarray[numpy.float32[2, 1]] | None = ..., area_max: numpy.ndarray[numpy.float32[2, 1]] | None = ..., resolution: float = ..., padding: int = ..., bg_color: numpy.ndarray[numpy.int32[4, 1]] = ..., fg_color: numpy.ndarray[numpy.int32[4, 1]] = ..., border_color: numpy.ndarray[numpy.int32[4, 1]] = ..., border_thickness: int = ...) -> numpy.ndarray[numpy.uint8[m, n]]:
        """visualize(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD, leaf_only: bool = False, area_min: Optional[numpy.ndarray[numpy.float32[2, 1]]] = None, area_max: Optional[numpy.ndarray[numpy.float32[2, 1]]] = None, resolution: float = 0.1, padding: int = 1, bg_color: numpy.ndarray[numpy.int32[4, 1]] = array([128, 128, 128, 255], dtype=int32), fg_color: numpy.ndarray[numpy.int32[4, 1]] = array([255, 255, 255, 255], dtype=int32), border_color: numpy.ndarray[numpy.int32[4, 1]] = array([ 0, 0, 0, 255], dtype=int32), border_thickness: int = 1) -> numpy.ndarray[numpy.uint8[m, n]]"""
    @property
    def children(self):
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD) -> list[Annotated[list[int], FixedSize(4)]]"""
    @property
    def memory_usage(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD) -> int"""
    @property
    def memory_usage_per_node(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD) -> int"""
    @property
    def metric_aabb(self) -> Aabb2Dd:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD) -> erl_geometry.pyerl_geometry.Aabb2Dd"""
    @property
    def metric_max(self) -> numpy.ndarray[numpy.float64[2, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD) -> numpy.ndarray[numpy.float64[2, 1]]"""
    @property
    def metric_min(self) -> numpy.ndarray[numpy.float64[2, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD) -> numpy.ndarray[numpy.float64[2, 1]]"""
    @property
    def metric_min_max(self) -> tuple[numpy.ndarray[numpy.float64[2, 1]], numpy.ndarray[numpy.float64[2, 1]]]:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD) -> tuple[numpy.ndarray[numpy.float64[2, 1]], numpy.ndarray[numpy.float64[2, 1]]]"""
    @property
    def metric_size(self) -> numpy.ndarray[numpy.float64[2, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD) -> numpy.ndarray[numpy.float64[2, 1]]"""
    @property
    def num_vertices(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD) -> int"""
    @property
    def number_of_leaf_nodes(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD) -> int"""
    @property
    def number_of_nodes(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD) -> int"""
    @property
    def parents(self) -> list[int]:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD) -> list[int]"""
    @property
    def resolution(self) -> float:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD) -> float"""
    @property
    def root(self) -> SemiSparseQuadtreeNode:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD) -> erl_geometry.pyerl_geometry.SemiSparseQuadtreeNode"""
    @property
    def setting(self) -> SemiSparseNdTreeSetting:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD) -> erl_geometry.pyerl_geometry.SemiSparseNdTreeSetting"""
    @property
    def tree_center(self) -> numpy.ndarray[numpy.float64[2, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD) -> numpy.ndarray[numpy.float64[2, 1]]"""
    @property
    def tree_center_key(self) -> QuadtreeKey:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD) -> erl_geometry.pyerl_geometry.QuadtreeKey"""
    @property
    def tree_depth(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD) -> int"""
    @property
    def tree_max_half_size(self) -> numpy.ndarray[numpy.float64[2, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD) -> numpy.ndarray[numpy.float64[2, 1]]"""
    @property
    def vertex_keys(self) -> list[QuadtreeKey]:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD) -> list[erl_geometry.pyerl_geometry.QuadtreeKey]"""
    @property
    def vertices(self):
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD) -> list[Annotated[list[int], FixedSize(4)]]"""
    @property
    def voxels(self):
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseQuadtreeD) -> list[Annotated[list[int], FixedSize(3)]]"""

class SemiSparseQuadtreeF(AbstractQuadtreeF):
    class Drawer:
        def __init__(self, setting, quadtree: SemiSparseQuadtreeF = ...) -> None:
            """__init__(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF.Drawer, setting: erl::geometry::AbstractQuadtreeDrawer::Setting, quadtree: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF = None) -> None"""
        @staticmethod
        def Setting(*args, **kwargs):
            """Setting() -> erl::geometry::AbstractQuadtreeDrawer::Setting"""
        @overload
        def draw_leaves(self) -> Mat:
            """draw_leaves(*args, **kwargs)
            Overloaded function.

            1. draw_leaves(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF.Drawer) -> Mat

            2. draw_leaves(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF.Drawer, filename: str) -> None
            """
        @overload
        def draw_leaves(self, filename: str) -> None:
            """draw_leaves(*args, **kwargs)
            Overloaded function.

            1. draw_leaves(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF.Drawer) -> Mat

            2. draw_leaves(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF.Drawer, filename: str) -> None
            """
        @overload
        def draw_tree(self) -> Mat:
            """draw_tree(*args, **kwargs)
            Overloaded function.

            1. draw_tree(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF.Drawer) -> Mat

            2. draw_tree(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF.Drawer, filename: str) -> None
            """
        @overload
        def draw_tree(self, filename: str) -> None:
            """draw_tree(*args, **kwargs)
            Overloaded function.

            1. draw_tree(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF.Drawer) -> Mat

            2. draw_tree(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF.Drawer, filename: str) -> None
            """
        def set_draw_leaf_callback(self, callback: Callable[[SemiSparseQuadtreeF.Drawer, Mat, SemiSparseQuadtreeF.LeafInAabbIterator], None]) -> None:
            """set_draw_leaf_callback(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF.Drawer, callback: Callable[[erl_geometry.pyerl_geometry.SemiSparseQuadtreeF.Drawer, Mat, erl_geometry.pyerl_geometry.SemiSparseQuadtreeF.LeafInAabbIterator], None]) -> None"""
        def set_draw_tree_callback(self, callback: Callable[[SemiSparseQuadtreeF.Drawer, Mat, SemiSparseQuadtreeF.TreeInAabbIterator], None]) -> None:
            """set_draw_tree_callback(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF.Drawer, callback: Callable[[erl_geometry.pyerl_geometry.SemiSparseQuadtreeF.Drawer, Mat, erl_geometry.pyerl_geometry.SemiSparseQuadtreeF.TreeInAabbIterator], None]) -> None"""
        @property
        def grid_map_info(self):
            """(arg0: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF.Drawer) -> erl::common::GridMapInfo<float, 2>"""
        @property
        def setting(self):
            """(arg0: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF.Drawer) -> erl::geometry::AbstractQuadtreeDrawer::Setting"""

    class EastLeafNeighborIterator(SemiSparseQuadtreeF.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class IteratorBase(AbstractQuadtreeF.QuadtreeNodeIterator):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""
        def __eq__(self, arg0: SemiSparseQuadtreeF.IteratorBase) -> bool:
            """__eq__(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF.IteratorBase, arg0: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF.IteratorBase) -> bool"""
        def __ne__(self, arg0: SemiSparseQuadtreeF.IteratorBase) -> bool:
            """__ne__(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF.IteratorBase, arg0: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF.IteratorBase) -> bool"""
        @property
        def index_key(self) -> QuadtreeKey:
            """(arg0: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF.IteratorBase) -> erl_geometry.pyerl_geometry.QuadtreeKey"""
        @property
        def key(self) -> QuadtreeKey:
            """(arg0: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF.IteratorBase) -> erl_geometry.pyerl_geometry.QuadtreeKey"""
        @property
        def node(self) -> SemiSparseQuadtreeNode:
            """(arg0: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF.IteratorBase) -> erl_geometry.pyerl_geometry.SemiSparseQuadtreeNode"""
        @property
        def node_aabb(self) -> Aabb2Df:
            """(arg0: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF.IteratorBase) -> erl_geometry.pyerl_geometry.Aabb2Df"""

    class LeafInAabbIterator(SemiSparseQuadtreeF.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class LeafIterator(SemiSparseQuadtreeF.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class LeafOfNodeIterator(SemiSparseQuadtreeF.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class NodeOnRayIterator(SemiSparseQuadtreeF.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""
        @property
        def distance(self) -> float:
            """(arg0: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF.NodeOnRayIterator) -> float"""

    class NorthLeafNeighborIterator(SemiSparseQuadtreeF.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class SouthLeafNeighborIterator(SemiSparseQuadtreeF.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class TreeInAabbIterator(SemiSparseQuadtreeF.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class TreeIterator(SemiSparseQuadtreeF.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""

    class WestLeafNeighborIterator(SemiSparseQuadtreeF.IteratorBase):
        def __init__(self, *args, **kwargs) -> None:
            """Initialize self.  See help(type(self)) for accurate signature."""
    @overload
    def __init__(self) -> None:
        """__init__(*args, **kwargs)
        Overloaded function.

        1. __init__(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF) -> None

        2. __init__(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, setting: erl_geometry.pyerl_geometry.SemiSparseNdTreeSetting) -> None

        3. __init__(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, filename: str) -> None
        """
    @overload
    def __init__(self, setting: SemiSparseNdTreeSetting) -> None:
        """__init__(*args, **kwargs)
        Overloaded function.

        1. __init__(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF) -> None

        2. __init__(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, setting: erl_geometry.pyerl_geometry.SemiSparseNdTreeSetting) -> None

        3. __init__(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, filename: str) -> None
        """
    @overload
    def __init__(self, filename: str) -> None:
        """__init__(*args, **kwargs)
        Overloaded function.

        1. __init__(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF) -> None

        2. __init__(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, setting: erl_geometry.pyerl_geometry.SemiSparseNdTreeSetting) -> None

        3. __init__(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, filename: str) -> None
        """
    def Setting(self) -> SemiSparseNdTreeSetting:
        """Setting() -> erl_geometry.pyerl_geometry.SemiSparseNdTreeSetting"""
    @overload
    def adjust_key_to_depth(self, key: int, depth: int) -> int:
        """adjust_key_to_depth(*args, **kwargs)
        Overloaded function.

        1. adjust_key_to_depth(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, key: int, depth: int) -> int

        2. adjust_key_to_depth(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> erl_geometry.pyerl_geometry.QuadtreeKey
        """
    @overload
    def adjust_key_to_depth(self, key: QuadtreeKey, depth: int) -> QuadtreeKey:
        """adjust_key_to_depth(*args, **kwargs)
        Overloaded function.

        1. adjust_key_to_depth(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, key: int, depth: int) -> int

        2. adjust_key_to_depth(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> erl_geometry.pyerl_geometry.QuadtreeKey
        """
    def clear(self) -> None:
        """clear(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF) -> None"""
    def compute_common_ancestor_key(self, arg0: QuadtreeKey, arg1: QuadtreeKey) -> tuple[QuadtreeKey, int]:
        """compute_common_ancestor_key(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, arg0: erl_geometry.pyerl_geometry.QuadtreeKey, arg1: erl_geometry.pyerl_geometry.QuadtreeKey) -> tuple[erl_geometry.pyerl_geometry.QuadtreeKey, int]"""
    def compute_east_neighbor_key(self, key: QuadtreeKey, depth: int) -> QuadtreeKey | None:
        """compute_east_neighbor_key(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]"""
    def compute_north_neighbor_key(self, key: QuadtreeKey, depth: int) -> QuadtreeKey | None:
        """compute_north_neighbor_key(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]"""
    def compute_ray_coords(self, sx: float, sy: float, ex: float, ey: float) -> list[numpy.ndarray[numpy.float32[2, 1]]] | None:
        """compute_ray_coords(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, sx: float, sy: float, ex: float, ey: float) -> Optional[list[numpy.ndarray[numpy.float32[2, 1]]]]"""
    def compute_ray_keys(self, sx: float, sy: float, ex: float, ey: float) -> list[QuadtreeKey] | None:
        """compute_ray_keys(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, sx: float, sy: float, ex: float, ey: float) -> Optional[list[erl_geometry.pyerl_geometry.QuadtreeKey]]"""
    def compute_south_neighbor_key(self, key: QuadtreeKey, depth: int) -> QuadtreeKey | None:
        """compute_south_neighbor_key(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]"""
    def compute_west_neighbor_key(self, key: QuadtreeKey, depth: int) -> QuadtreeKey | None:
        """compute_west_neighbor_key(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]"""
    @overload
    def coord_to_key(self, coordinate: float) -> int:
        """coord_to_key(*args, **kwargs)
        Overloaded function.

        1. coord_to_key(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, coordinate: float) -> int

        2. coord_to_key(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, coordinate: float, depth: int) -> int

        3. coord_to_key(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, x: float, y: float) -> erl_geometry.pyerl_geometry.QuadtreeKey

        4. coord_to_key(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, x: float, y: float, depth: int) -> erl_geometry.pyerl_geometry.QuadtreeKey
        """
    @overload
    def coord_to_key(self, coordinate: float, depth: int) -> int:
        """coord_to_key(*args, **kwargs)
        Overloaded function.

        1. coord_to_key(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, coordinate: float) -> int

        2. coord_to_key(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, coordinate: float, depth: int) -> int

        3. coord_to_key(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, x: float, y: float) -> erl_geometry.pyerl_geometry.QuadtreeKey

        4. coord_to_key(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, x: float, y: float, depth: int) -> erl_geometry.pyerl_geometry.QuadtreeKey
        """
    @overload
    def coord_to_key(self, x: float, y: float) -> QuadtreeKey:
        """coord_to_key(*args, **kwargs)
        Overloaded function.

        1. coord_to_key(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, coordinate: float) -> int

        2. coord_to_key(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, coordinate: float, depth: int) -> int

        3. coord_to_key(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, x: float, y: float) -> erl_geometry.pyerl_geometry.QuadtreeKey

        4. coord_to_key(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, x: float, y: float, depth: int) -> erl_geometry.pyerl_geometry.QuadtreeKey
        """
    @overload
    def coord_to_key(self, x: float, y: float, depth: int) -> QuadtreeKey:
        """coord_to_key(*args, **kwargs)
        Overloaded function.

        1. coord_to_key(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, coordinate: float) -> int

        2. coord_to_key(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, coordinate: float, depth: int) -> int

        3. coord_to_key(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, x: float, y: float) -> erl_geometry.pyerl_geometry.QuadtreeKey

        4. coord_to_key(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, x: float, y: float, depth: int) -> erl_geometry.pyerl_geometry.QuadtreeKey
        """
    @overload
    def coord_to_key_checked(self, coordinate: float) -> int | None:
        """coord_to_key_checked(*args, **kwargs)
        Overloaded function.

        1. coord_to_key_checked(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, coordinate: float) -> Optional[int]

        2. coord_to_key_checked(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, coordinate: float, depth: int) -> Optional[int]

        3. coord_to_key_checked(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, x: float, y: float) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]

        4. coord_to_key_checked(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, x: float, y: float, depth: int) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]
        """
    @overload
    def coord_to_key_checked(self, coordinate: float, depth: int) -> int | None:
        """coord_to_key_checked(*args, **kwargs)
        Overloaded function.

        1. coord_to_key_checked(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, coordinate: float) -> Optional[int]

        2. coord_to_key_checked(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, coordinate: float, depth: int) -> Optional[int]

        3. coord_to_key_checked(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, x: float, y: float) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]

        4. coord_to_key_checked(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, x: float, y: float, depth: int) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]
        """
    @overload
    def coord_to_key_checked(self, x: float, y: float) -> QuadtreeKey | None:
        """coord_to_key_checked(*args, **kwargs)
        Overloaded function.

        1. coord_to_key_checked(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, coordinate: float) -> Optional[int]

        2. coord_to_key_checked(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, coordinate: float, depth: int) -> Optional[int]

        3. coord_to_key_checked(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, x: float, y: float) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]

        4. coord_to_key_checked(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, x: float, y: float, depth: int) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]
        """
    @overload
    def coord_to_key_checked(self, x: float, y: float, depth: int) -> QuadtreeKey | None:
        """coord_to_key_checked(*args, **kwargs)
        Overloaded function.

        1. coord_to_key_checked(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, coordinate: float) -> Optional[int]

        2. coord_to_key_checked(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, coordinate: float, depth: int) -> Optional[int]

        3. coord_to_key_checked(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, x: float, y: float) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]

        4. coord_to_key_checked(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, x: float, y: float, depth: int) -> Optional[erl_geometry.pyerl_geometry.QuadtreeKey]
        """
    def create_node_child(self, node: SemiSparseQuadtreeNode, child_idx: int) -> SemiSparseQuadtreeNode:
        """create_node_child(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, node: erl_geometry.pyerl_geometry.SemiSparseQuadtreeNode, child_idx: int) -> erl_geometry.pyerl_geometry.SemiSparseQuadtreeNode"""
    @overload
    def delete_node(self, x: float, y: float, depth: int) -> int:
        """delete_node(*args, **kwargs)
        Overloaded function.

        1. delete_node(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, x: float, y: float, depth: int) -> int

        2. delete_node(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> None
        """
    @overload
    def delete_node(self, key: QuadtreeKey, depth: int) -> None:
        """delete_node(*args, **kwargs)
        Overloaded function.

        1. delete_node(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, x: float, y: float, depth: int) -> int

        2. delete_node(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> None
        """
    def delete_node_child(self, node: SemiSparseQuadtreeNode, child_idx: int, key: QuadtreeKey) -> int:
        """delete_node_child(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, node: erl_geometry.pyerl_geometry.SemiSparseQuadtreeNode, child_idx: int, key: erl_geometry.pyerl_geometry.QuadtreeKey) -> int"""
    def expand(self) -> None:
        """expand(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF) -> None"""
    def expand_node(self, node: SemiSparseQuadtreeNode) -> None:
        """expand_node(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, node: erl_geometry.pyerl_geometry.SemiSparseQuadtreeNode) -> None"""
    def find_voxel_index(self, key: QuadtreeKey) -> int:
        """find_voxel_index(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey) -> int"""
    def find_voxel_indices(self, points: numpy.ndarray[numpy.float32[2, n]], parallel: bool) -> list[int]:
        """find_voxel_indices(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, points: numpy.ndarray[numpy.float32[2, n]], parallel: bool) -> list[int]"""
    def get_node_child(self, node: SemiSparseQuadtreeNode, child_idx: int) -> SemiSparseQuadtreeNode:
        """get_node_child(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, node: erl_geometry.pyerl_geometry.SemiSparseQuadtreeNode, child_idx: int) -> erl_geometry.pyerl_geometry.SemiSparseQuadtreeNode"""
    def get_node_size(self, depth: int) -> float:
        """get_node_size(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, depth: int) -> float"""
    @overload
    def insert_node(self, x: float, y: float, depth: int) -> SemiSparseQuadtreeNode:
        """insert_node(*args, **kwargs)
        Overloaded function.

        1. insert_node(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, x: float, y: float, depth: int) -> erl_geometry.pyerl_geometry.SemiSparseQuadtreeNode

        2. insert_node(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> erl_geometry.pyerl_geometry.SemiSparseQuadtreeNode
        """
    @overload
    def insert_node(self, key: QuadtreeKey, depth: int) -> SemiSparseQuadtreeNode:
        """insert_node(*args, **kwargs)
        Overloaded function.

        1. insert_node(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, x: float, y: float, depth: int) -> erl_geometry.pyerl_geometry.SemiSparseQuadtreeNode

        2. insert_node(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> erl_geometry.pyerl_geometry.SemiSparseQuadtreeNode
        """
    def insert_point(self, key: QuadtreeKey, max_depth: int) -> int:
        """insert_point(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, max_depth: int) -> int"""
    def insert_points(self, points: numpy.ndarray[numpy.float32[2, n]]) -> list[int]:
        """insert_points(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, points: numpy.ndarray[numpy.float32[2, n]]) -> list[int]"""
    def is_node_collapsible(self, node: SemiSparseQuadtreeNode) -> bool:
        """is_node_collapsible(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, node: erl_geometry.pyerl_geometry.SemiSparseQuadtreeNode) -> bool"""
    @overload
    def iter_east_leaf_neighbor(self, x: float, y: float, max_leaf_depth: int = ...) -> Iterator[SemiSparseQuadtreeF.EastLeafNeighborIterator]:
        """iter_east_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_east_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, x: float, y: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseQuadtreeF.EastLeafNeighborIterator]

        2. iter_east_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseQuadtreeF.EastLeafNeighborIterator]
        """
    @overload
    def iter_east_leaf_neighbor(self, key: QuadtreeKey, key_depth: int, max_leaf_depth: int = ...) -> Iterator[SemiSparseQuadtreeF.EastLeafNeighborIterator]:
        """iter_east_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_east_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, x: float, y: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseQuadtreeF.EastLeafNeighborIterator]

        2. iter_east_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseQuadtreeF.EastLeafNeighborIterator]
        """
    def iter_leaf(self, max_depth: int = ...) -> Iterator[SemiSparseQuadtreeF.LeafIterator]:
        """iter_leaf(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseQuadtreeF.LeafIterator]"""
    @overload
    def iter_leaf_in_aabb(self, aabb_min_x: float, aabb_min_y: float, aabb_max_x: float, aabb_max_y: float, max_depth: int = ...) -> Iterator[SemiSparseQuadtreeF.LeafInAabbIterator]:
        """iter_leaf_in_aabb(*args, **kwargs)
        Overloaded function.

        1. iter_leaf_in_aabb(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, aabb_min_x: float, aabb_min_y: float, aabb_max_x: float, aabb_max_y: float, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseQuadtreeF.LeafInAabbIterator]

        2. iter_leaf_in_aabb(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, aabb_min_key: erl_geometry.pyerl_geometry.QuadtreeKey, aabb_max_key: erl_geometry.pyerl_geometry.QuadtreeKey, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseQuadtreeF.LeafInAabbIterator]
        """
    @overload
    def iter_leaf_in_aabb(self, aabb_min_key: QuadtreeKey, aabb_max_key: QuadtreeKey, max_depth: int = ...) -> Iterator[SemiSparseQuadtreeF.LeafInAabbIterator]:
        """iter_leaf_in_aabb(*args, **kwargs)
        Overloaded function.

        1. iter_leaf_in_aabb(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, aabb_min_x: float, aabb_min_y: float, aabb_max_x: float, aabb_max_y: float, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseQuadtreeF.LeafInAabbIterator]

        2. iter_leaf_in_aabb(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, aabb_min_key: erl_geometry.pyerl_geometry.QuadtreeKey, aabb_max_key: erl_geometry.pyerl_geometry.QuadtreeKey, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseQuadtreeF.LeafInAabbIterator]
        """
    def iter_leaf_of_node(self, node_key: QuadtreeKey, node_depth: int, max_depth: int = ...) -> Iterator[SemiSparseQuadtreeF.LeafOfNodeIterator]:
        """iter_leaf_of_node(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, node_key: erl_geometry.pyerl_geometry.QuadtreeKey, node_depth: int, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseQuadtreeF.LeafOfNodeIterator]"""
    def iter_node(self, max_depth: int = ...) -> Iterator[SemiSparseQuadtreeF.TreeIterator]:
        """iter_node(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseQuadtreeF.TreeIterator]"""
    @overload
    def iter_node_in_aabb(self, aabb_min_x: float, aabb_min_y: float, aabb_max_x: float, aabb_max_y: float, max_depth: int = ...) -> Iterator[SemiSparseQuadtreeF.TreeInAabbIterator]:
        """iter_node_in_aabb(*args, **kwargs)
        Overloaded function.

        1. iter_node_in_aabb(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, aabb_min_x: float, aabb_min_y: float, aabb_max_x: float, aabb_max_y: float, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseQuadtreeF.TreeInAabbIterator]

        2. iter_node_in_aabb(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, aabb_min_key: erl_geometry.pyerl_geometry.QuadtreeKey, aabb_max_key: erl_geometry.pyerl_geometry.QuadtreeKey, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseQuadtreeF.TreeInAabbIterator]
        """
    @overload
    def iter_node_in_aabb(self, aabb_min_key: QuadtreeKey, aabb_max_key: QuadtreeKey, max_depth: int = ...) -> Iterator[SemiSparseQuadtreeF.TreeInAabbIterator]:
        """iter_node_in_aabb(*args, **kwargs)
        Overloaded function.

        1. iter_node_in_aabb(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, aabb_min_x: float, aabb_min_y: float, aabb_max_x: float, aabb_max_y: float, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseQuadtreeF.TreeInAabbIterator]

        2. iter_node_in_aabb(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, aabb_min_key: erl_geometry.pyerl_geometry.QuadtreeKey, aabb_max_key: erl_geometry.pyerl_geometry.QuadtreeKey, max_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseQuadtreeF.TreeInAabbIterator]
        """
    def iter_node_on_ray(self, px: float, py: float, vx: float, vy: float, max_range: float = ..., node_padding: float = ..., bidirectional: bool = ..., leaf_only: bool = ..., min_node_depth: int = ..., max_node_depth: int = ...) -> Iterator[SemiSparseQuadtreeF.NodeOnRayIterator]:
        """iter_node_on_ray(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, px: float, py: float, vx: float, vy: float, max_range: float = -1, node_padding: float = 0, bidirectional: bool = False, leaf_only: bool = False, min_node_depth: int = 0, max_node_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseQuadtreeF.NodeOnRayIterator]"""
    @overload
    def iter_north_leaf_neighbor(self, x: float, y: float, max_leaf_depth: int = ...) -> Iterator[SemiSparseQuadtreeF.NorthLeafNeighborIterator]:
        """iter_north_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_north_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, x: float, y: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseQuadtreeF.NorthLeafNeighborIterator]

        2. iter_north_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseQuadtreeF.NorthLeafNeighborIterator]
        """
    @overload
    def iter_north_leaf_neighbor(self, key: QuadtreeKey, key_depth: int, max_leaf_depth: int = ...) -> Iterator[SemiSparseQuadtreeF.NorthLeafNeighborIterator]:
        """iter_north_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_north_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, x: float, y: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseQuadtreeF.NorthLeafNeighborIterator]

        2. iter_north_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseQuadtreeF.NorthLeafNeighborIterator]
        """
    @overload
    def iter_south_leaf_neighbor(self, x: float, y: float, max_leaf_depth: int = ...) -> Iterator[SemiSparseQuadtreeF.SouthLeafNeighborIterator]:
        """iter_south_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_south_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, x: float, y: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseQuadtreeF.SouthLeafNeighborIterator]

        2. iter_south_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseQuadtreeF.SouthLeafNeighborIterator]
        """
    @overload
    def iter_south_leaf_neighbor(self, key: QuadtreeKey, key_depth: int, max_leaf_depth: int = ...) -> Iterator[SemiSparseQuadtreeF.SouthLeafNeighborIterator]:
        """iter_south_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_south_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, x: float, y: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseQuadtreeF.SouthLeafNeighborIterator]

        2. iter_south_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseQuadtreeF.SouthLeafNeighborIterator]
        """
    @overload
    def iter_west_leaf_neighbor(self, x: float, y: float, max_leaf_depth: int = ...) -> Iterator[SemiSparseQuadtreeF.WestLeafNeighborIterator]:
        """iter_west_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_west_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, x: float, y: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseQuadtreeF.WestLeafNeighborIterator]

        2. iter_west_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseQuadtreeF.WestLeafNeighborIterator]
        """
    @overload
    def iter_west_leaf_neighbor(self, key: QuadtreeKey, key_depth: int, max_leaf_depth: int = ...) -> Iterator[SemiSparseQuadtreeF.WestLeafNeighborIterator]:
        """iter_west_leaf_neighbor(*args, **kwargs)
        Overloaded function.

        1. iter_west_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, x: float, y: float, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseQuadtreeF.WestLeafNeighborIterator]

        2. iter_west_leaf_neighbor(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, key_depth: int, max_leaf_depth: int = 0) -> Iterator[erl_geometry.pyerl_geometry.SemiSparseQuadtreeF.WestLeafNeighborIterator]
        """
    @overload
    def key_to_coord(self, key: int) -> float:
        """key_to_coord(*args, **kwargs)
        Overloaded function.

        1. key_to_coord(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, key: int) -> float

        2. key_to_coord(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, key: int, depth: int) -> float

        3. key_to_coord(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey) -> tuple[float, float]

        4. key_to_coord(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> tuple[float, float]
        """
    @overload
    def key_to_coord(self, key: int, depth: int) -> float:
        """key_to_coord(*args, **kwargs)
        Overloaded function.

        1. key_to_coord(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, key: int) -> float

        2. key_to_coord(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, key: int, depth: int) -> float

        3. key_to_coord(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey) -> tuple[float, float]

        4. key_to_coord(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> tuple[float, float]
        """
    @overload
    def key_to_coord(self, key: QuadtreeKey) -> tuple[float, float]:
        """key_to_coord(*args, **kwargs)
        Overloaded function.

        1. key_to_coord(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, key: int) -> float

        2. key_to_coord(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, key: int, depth: int) -> float

        3. key_to_coord(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey) -> tuple[float, float]

        4. key_to_coord(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> tuple[float, float]
        """
    @overload
    def key_to_coord(self, key: QuadtreeKey, depth: int) -> tuple[float, float]:
        """key_to_coord(*args, **kwargs)
        Overloaded function.

        1. key_to_coord(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, key: int) -> float

        2. key_to_coord(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, key: int, depth: int) -> float

        3. key_to_coord(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey) -> tuple[float, float]

        4. key_to_coord(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, depth: int) -> tuple[float, float]
        """
    def prune(self) -> None:
        """prune(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF) -> None"""
    def prune_node(self, node: SemiSparseQuadtreeNode) -> bool:
        """prune_node(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, node: erl_geometry.pyerl_geometry.SemiSparseQuadtreeNode) -> bool"""
    @overload
    def search(self, x: float, y: float, max_depth: int = ...) -> SemiSparseQuadtreeNode:
        """search(*args, **kwargs)
        Overloaded function.

        1. search(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, x: float, y: float, max_depth: int = 0) -> erl_geometry.pyerl_geometry.SemiSparseQuadtreeNode

        2. search(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, max_depth: int = 0) -> erl_geometry.pyerl_geometry.SemiSparseQuadtreeNode
        """
    @overload
    def search(self, key: QuadtreeKey, max_depth: int = ...) -> SemiSparseQuadtreeNode:
        """search(*args, **kwargs)
        Overloaded function.

        1. search(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, x: float, y: float, max_depth: int = 0) -> erl_geometry.pyerl_geometry.SemiSparseQuadtreeNode

        2. search(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, key: erl_geometry.pyerl_geometry.QuadtreeKey, max_depth: int = 0) -> erl_geometry.pyerl_geometry.SemiSparseQuadtreeNode
        """
    def visualize(self, leaf_only: bool = ..., area_min: numpy.ndarray[numpy.float32[2, 1]] | None = ..., area_max: numpy.ndarray[numpy.float32[2, 1]] | None = ..., resolution: float = ..., padding: int = ..., bg_color: numpy.ndarray[numpy.int32[4, 1]] = ..., fg_color: numpy.ndarray[numpy.int32[4, 1]] = ..., border_color: numpy.ndarray[numpy.int32[4, 1]] = ..., border_thickness: int = ...) -> numpy.ndarray[numpy.uint8[m, n]]:
        """visualize(self: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF, leaf_only: bool = False, area_min: Optional[numpy.ndarray[numpy.float32[2, 1]]] = None, area_max: Optional[numpy.ndarray[numpy.float32[2, 1]]] = None, resolution: float = 0.1, padding: int = 1, bg_color: numpy.ndarray[numpy.int32[4, 1]] = array([128, 128, 128, 255], dtype=int32), fg_color: numpy.ndarray[numpy.int32[4, 1]] = array([255, 255, 255, 255], dtype=int32), border_color: numpy.ndarray[numpy.int32[4, 1]] = array([ 0, 0, 0, 255], dtype=int32), border_thickness: int = 1) -> numpy.ndarray[numpy.uint8[m, n]]"""
    @property
    def children(self):
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF) -> list[Annotated[list[int], FixedSize(4)]]"""
    @property
    def memory_usage(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF) -> int"""
    @property
    def memory_usage_per_node(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF) -> int"""
    @property
    def metric_aabb(self) -> Aabb2Df:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF) -> erl_geometry.pyerl_geometry.Aabb2Df"""
    @property
    def metric_max(self) -> numpy.ndarray[numpy.float32[2, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF) -> numpy.ndarray[numpy.float32[2, 1]]"""
    @property
    def metric_min(self) -> numpy.ndarray[numpy.float32[2, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF) -> numpy.ndarray[numpy.float32[2, 1]]"""
    @property
    def metric_min_max(self) -> tuple[numpy.ndarray[numpy.float32[2, 1]], numpy.ndarray[numpy.float32[2, 1]]]:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF) -> tuple[numpy.ndarray[numpy.float32[2, 1]], numpy.ndarray[numpy.float32[2, 1]]]"""
    @property
    def metric_size(self) -> numpy.ndarray[numpy.float32[2, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF) -> numpy.ndarray[numpy.float32[2, 1]]"""
    @property
    def num_vertices(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF) -> int"""
    @property
    def number_of_leaf_nodes(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF) -> int"""
    @property
    def number_of_nodes(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF) -> int"""
    @property
    def parents(self) -> list[int]:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF) -> list[int]"""
    @property
    def resolution(self) -> float:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF) -> float"""
    @property
    def root(self) -> SemiSparseQuadtreeNode:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF) -> erl_geometry.pyerl_geometry.SemiSparseQuadtreeNode"""
    @property
    def setting(self) -> SemiSparseNdTreeSetting:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF) -> erl_geometry.pyerl_geometry.SemiSparseNdTreeSetting"""
    @property
    def tree_center(self) -> numpy.ndarray[numpy.float32[2, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF) -> numpy.ndarray[numpy.float32[2, 1]]"""
    @property
    def tree_center_key(self) -> QuadtreeKey:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF) -> erl_geometry.pyerl_geometry.QuadtreeKey"""
    @property
    def tree_depth(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF) -> int"""
    @property
    def tree_max_half_size(self) -> numpy.ndarray[numpy.float32[2, 1]]:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF) -> numpy.ndarray[numpy.float32[2, 1]]"""
    @property
    def vertex_keys(self) -> list[QuadtreeKey]:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF) -> list[erl_geometry.pyerl_geometry.QuadtreeKey]"""
    @property
    def vertices(self):
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF) -> list[Annotated[list[int], FixedSize(4)]]"""
    @property
    def voxels(self):
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseQuadtreeF) -> list[Annotated[list[int], FixedSize(3)]]"""

class SemiSparseQuadtreeNode(AbstractQuadtreeNode):
    def __init__(self, *args, **kwargs) -> None:
        """Initialize self.  See help(type(self)) for accurate signature."""
    @property
    def node_index(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.SemiSparseQuadtreeNode) -> int"""

class Space2D:
    class SignMethod:
        """Algorithm to determine SDF sign.

        Members:

          kPointNormal : Use the normal of the nearest vertex to determine the sign.

          kLineNormal : Use the normal of the nearest line segment to determine the sign.

          kPolygon : Use the nearest object polygon and the winding number algorithm to determine the sign."""
        __members__: ClassVar[dict] = ...  # read-only
        __entries: ClassVar[dict] = ...
        kLineNormal: ClassVar[Space2D.SignMethod] = ...
        kPointNormal: ClassVar[Space2D.SignMethod] = ...
        kPolygon: ClassVar[Space2D.SignMethod] = ...
        def __init__(self, value: int) -> None:
            """__init__(self: erl_geometry.pyerl_geometry.Space2D.SignMethod, value: int) -> None"""
        def __eq__(self, other: object) -> bool:
            """__eq__(self: object, other: object) -> bool"""
        def __ge__(self, other: object) -> bool:
            """__ge__(self: object, other: object) -> bool"""
        def __gt__(self, other: object) -> bool:
            """__gt__(self: object, other: object) -> bool"""
        def __hash__(self) -> int:
            """__hash__(self: object) -> int"""
        def __index__(self) -> int:
            """__index__(self: erl_geometry.pyerl_geometry.Space2D.SignMethod) -> int"""
        def __int__(self) -> int:
            """__int__(self: erl_geometry.pyerl_geometry.Space2D.SignMethod) -> int"""
        def __le__(self, other: object) -> bool:
            """__le__(self: object, other: object) -> bool"""
        def __lt__(self, other: object) -> bool:
            """__lt__(self: object, other: object) -> bool"""
        def __ne__(self, other: object) -> bool:
            """__ne__(self: object, other: object) -> bool"""
        @property
        def name(self) -> str:
            """name(self: object) -> str

            name(self: object) -> str
            """
        @property
        def value(self) -> int:
            """(arg0: erl_geometry.pyerl_geometry.Space2D.SignMethod) -> int"""
    kLineNormal: ClassVar[Space2D.SignMethod] = ...
    kPointNormal: ClassVar[Space2D.SignMethod] = ...
    kPolygon: ClassVar[Space2D.SignMethod] = ...
    @overload
    def __init__(self, ordered_object_vertices: list[numpy.ndarray[numpy.float64[2, n], flags.f_contiguous]], ordered_object_normals: list[numpy.ndarray[numpy.float64[2, n], flags.f_contiguous]]) -> None:
        """__init__(*args, **kwargs)
        Overloaded function.

        1. __init__(self: erl_geometry.pyerl_geometry.Space2D, ordered_object_vertices: list[numpy.ndarray[numpy.float64[2, n], flags.f_contiguous]], ordered_object_normals: list[numpy.ndarray[numpy.float64[2, n], flags.f_contiguous]]) -> None

        2. __init__(self: erl_geometry.pyerl_geometry.Space2D, ordered_object_vertices: list[numpy.ndarray[numpy.float64[2, n], flags.f_contiguous]], outside_flags: numpy.ndarray[bool[m, 1]], delta: float = 0.01, parallel: bool = False) -> None

        3. __init__(self: erl_geometry.pyerl_geometry.Space2D, map_image: numpy.ndarray[numpy.float64[m, n], flags.f_contiguous], grid_map_info: erl::common::GridMapInfo<double, 2>, free_threshold: float, delta: float = 0.01, parallel: bool = False) -> None

        4. __init__(self: erl_geometry.pyerl_geometry.Space2D, space2d: erl_geometry.pyerl_geometry.Space2D) -> None
        """
    @overload
    def __init__(self, ordered_object_vertices: list[numpy.ndarray[numpy.float64[2, n], flags.f_contiguous]], outside_flags: numpy.ndarray[bool[m, 1]], delta: float = ..., parallel: bool = ...) -> None:
        """__init__(*args, **kwargs)
        Overloaded function.

        1. __init__(self: erl_geometry.pyerl_geometry.Space2D, ordered_object_vertices: list[numpy.ndarray[numpy.float64[2, n], flags.f_contiguous]], ordered_object_normals: list[numpy.ndarray[numpy.float64[2, n], flags.f_contiguous]]) -> None

        2. __init__(self: erl_geometry.pyerl_geometry.Space2D, ordered_object_vertices: list[numpy.ndarray[numpy.float64[2, n], flags.f_contiguous]], outside_flags: numpy.ndarray[bool[m, 1]], delta: float = 0.01, parallel: bool = False) -> None

        3. __init__(self: erl_geometry.pyerl_geometry.Space2D, map_image: numpy.ndarray[numpy.float64[m, n], flags.f_contiguous], grid_map_info: erl::common::GridMapInfo<double, 2>, free_threshold: float, delta: float = 0.01, parallel: bool = False) -> None

        4. __init__(self: erl_geometry.pyerl_geometry.Space2D, space2d: erl_geometry.pyerl_geometry.Space2D) -> None
        """
    @overload
    def __init__(self, space2d: Space2D) -> None:
        """__init__(*args, **kwargs)
        Overloaded function.

        1. __init__(self: erl_geometry.pyerl_geometry.Space2D, ordered_object_vertices: list[numpy.ndarray[numpy.float64[2, n], flags.f_contiguous]], ordered_object_normals: list[numpy.ndarray[numpy.float64[2, n], flags.f_contiguous]]) -> None

        2. __init__(self: erl_geometry.pyerl_geometry.Space2D, ordered_object_vertices: list[numpy.ndarray[numpy.float64[2, n], flags.f_contiguous]], outside_flags: numpy.ndarray[bool[m, 1]], delta: float = 0.01, parallel: bool = False) -> None

        3. __init__(self: erl_geometry.pyerl_geometry.Space2D, map_image: numpy.ndarray[numpy.float64[m, n], flags.f_contiguous], grid_map_info: erl::common::GridMapInfo<double, 2>, free_threshold: float, delta: float = 0.01, parallel: bool = False) -> None

        4. __init__(self: erl_geometry.pyerl_geometry.Space2D, space2d: erl_geometry.pyerl_geometry.Space2D) -> None
        """
    def compute_ddf(self, query_points: numpy.ndarray[numpy.float64[2, n], flags.f_contiguous], query_directions: numpy.ndarray[numpy.float64[2, n], flags.f_contiguous], parallel: bool = ...) -> numpy.ndarray[numpy.float64[m, 1]]:
        """compute_ddf(*args, **kwargs)
        Overloaded function.

        1. compute_ddf(self: erl_geometry.pyerl_geometry.Space2D, grid_map_info: erl::common::GridMapInfo<double, 2>, query_directions: numpy.ndarray[numpy.float64[2, n], flags.f_contiguous], parallel: bool = False) -> numpy.ndarray[numpy.float64]

        2. compute_ddf(self: erl_geometry.pyerl_geometry.Space2D, query_points: numpy.ndarray[numpy.float64[2, n], flags.f_contiguous], query_directions: numpy.ndarray[numpy.float64[2, n], flags.f_contiguous], parallel: bool = False) -> numpy.ndarray[numpy.float64[m, 1]]
        """
    def compute_sddf_v1(self, query_points: numpy.ndarray[numpy.float64[2, n], flags.f_contiguous], query_directions: numpy.ndarray[numpy.float64[2, n], flags.f_contiguous], parallel: bool = ...) -> numpy.ndarray[numpy.float64[m, 1]]:
        """compute_sddf_v1(*args, **kwargs)
        Overloaded function.

        1. compute_sddf_v1(self: erl_geometry.pyerl_geometry.Space2D, grid_map_info: erl::common::GridMapInfo<double, 2>, query_directions: numpy.ndarray[numpy.float64[2, n], flags.f_contiguous], parallel: bool = False) -> numpy.ndarray[numpy.float64]

        2. compute_sddf_v1(self: erl_geometry.pyerl_geometry.Space2D, query_points: numpy.ndarray[numpy.float64[2, n], flags.f_contiguous], query_directions: numpy.ndarray[numpy.float64[2, n], flags.f_contiguous], parallel: bool = False) -> numpy.ndarray[numpy.float64[m, 1]]
        """
    def compute_sddf_v2(self, query_points: numpy.ndarray[numpy.float64[2, n], flags.f_contiguous], query_directions: numpy.ndarray[numpy.float64[2, n], flags.f_contiguous], sign_method: Space2D.SignMethod = ..., parallel: bool = ...) -> numpy.ndarray[numpy.float64[m, 1]]:
        """compute_sddf_v2(*args, **kwargs)
        Overloaded function.

        1. compute_sddf_v2(self: erl_geometry.pyerl_geometry.Space2D, map_image_info: erl::common::GridMapInfo<double, 2>, query_directions: numpy.ndarray[numpy.float64[2, n], flags.f_contiguous], sign_method: erl_geometry.pyerl_geometry.Space2D.SignMethod = <SignMethod.kLineNormal: 1>, parallel: bool = False) -> numpy.ndarray[numpy.float64]

        2. compute_sddf_v2(self: erl_geometry.pyerl_geometry.Space2D, query_points: numpy.ndarray[numpy.float64[2, n], flags.f_contiguous], query_directions: numpy.ndarray[numpy.float64[2, n], flags.f_contiguous], sign_method: erl_geometry.pyerl_geometry.Space2D.SignMethod = <SignMethod.kLineNormal: 1>, parallel: bool = False) -> numpy.ndarray[numpy.float64[m, 1]]
        """
    def compute_sdf(self, query_points: numpy.ndarray[numpy.float64[2, n], flags.f_contiguous], sign_method: Space2D.SignMethod = ..., use_kd_tree: bool = ..., parallel: bool = ...) -> numpy.ndarray[numpy.float64[m, 1]]:
        """compute_sdf(self: erl_geometry.pyerl_geometry.Space2D, query_points: numpy.ndarray[numpy.float64[2, n], flags.f_contiguous], sign_method: erl_geometry.pyerl_geometry.Space2D.SignMethod = <SignMethod.kLineNormal: 1>, use_kd_tree: bool = False, parallel: bool = False) -> numpy.ndarray[numpy.float64[m, 1]]"""
    def compute_sdf_greedily(self, q: numpy.ndarray[numpy.float64[2, 1]], sign_method: Space2D.SignMethod) -> float:
        """compute_sdf_greedily(self: erl_geometry.pyerl_geometry.Space2D, q: numpy.ndarray[numpy.float64[2, 1]], sign_method: erl_geometry.pyerl_geometry.Space2D.SignMethod) -> float"""
    def compute_sdf_image(self, *args, **kwargs):
        """compute_sdf_image(self: erl_geometry.pyerl_geometry.Space2D, grid_map_info: erl::common::GridMapInfo<double, 2>, sign_method: erl_geometry.pyerl_geometry.Space2D.SignMethod = <SignMethod.kLineNormal: 1>, use_kdtree: bool = False, parallel: bool = False) -> numpy.ndarray[numpy.float64[m, n]]"""
    def compute_sdf_with_kdtree(self, q: numpy.ndarray[numpy.float64[2, 1]], sign_method: Space2D.SignMethod) -> float:
        """compute_sdf_with_kdtree(self: erl_geometry.pyerl_geometry.Space2D, q: numpy.ndarray[numpy.float64[2, 1]], sign_method: erl_geometry.pyerl_geometry.Space2D.SignMethod) -> float"""
    def generate_map_image(self, *args, **kwargs):
        """generate_map_image(self: erl_geometry.pyerl_geometry.Space2D, grid_map_info: erl::common::GridMapInfo<double, 2>, anti_aliased: bool = False) -> numpy.ndarray[numpy.uint8[m, n]]"""
    @staticmethod
    def get_sign_method_from_name(sign_method_name: str) -> Space2D.SignMethod:
        """get_sign_method_from_name(sign_method_name: str) -> erl_geometry.pyerl_geometry.Space2D.SignMethod"""
    @staticmethod
    def get_sign_method_name(sign_method: Space2D.SignMethod) -> str:
        """get_sign_method_name(sign_method: erl_geometry.pyerl_geometry.Space2D.SignMethod) -> str"""
    @property
    def surface(self) -> Surface2D:
        """(arg0: erl_geometry.pyerl_geometry.Space2D) -> erl_geometry.pyerl_geometry.Surface2D"""

class Surface2D:
    lines_to_vertices: numpy.ndarray[numpy.int32[2, n]]
    normals: numpy.ndarray[numpy.float64[2, n]]
    objects_to_lines: numpy.ndarray[numpy.int32[2, n]]
    outside_flags: numpy.ndarray[bool[m, 1]]
    vertices: numpy.ndarray[numpy.float64[2, n]]
    vertices_to_objects: numpy.ndarray[numpy.int32[m, 1]]
    @overload
    def __init__(self, vertices: numpy.ndarray[numpy.float64[2, n]], normals: numpy.ndarray[numpy.float64[2, n]], lines2vertices: numpy.ndarray[numpy.int32[2, n]], objects2lines: numpy.ndarray[numpy.int32[2, n]], outside_flags: numpy.ndarray[bool[m, 1]]) -> None:
        """__init__(*args, **kwargs)
        Overloaded function.

        1. __init__(self: erl_geometry.pyerl_geometry.Surface2D, vertices: numpy.ndarray[numpy.float64[2, n]], normals: numpy.ndarray[numpy.float64[2, n]], lines2vertices: numpy.ndarray[numpy.int32[2, n]], objects2lines: numpy.ndarray[numpy.int32[2, n]], outside_flags: numpy.ndarray[bool[m, 1]]) -> None

        2. __init__(self: erl_geometry.pyerl_geometry.Surface2D, surface: erl_geometry.pyerl_geometry.Surface2D) -> None
        """
    @overload
    def __init__(self, surface: Surface2D) -> None:
        """__init__(*args, **kwargs)
        Overloaded function.

        1. __init__(self: erl_geometry.pyerl_geometry.Surface2D, vertices: numpy.ndarray[numpy.float64[2, n]], normals: numpy.ndarray[numpy.float64[2, n]], lines2vertices: numpy.ndarray[numpy.int32[2, n]], objects2lines: numpy.ndarray[numpy.int32[2, n]], outside_flags: numpy.ndarray[bool[m, 1]]) -> None

        2. __init__(self: erl_geometry.pyerl_geometry.Surface2D, surface: erl_geometry.pyerl_geometry.Surface2D) -> None
        """
    def get_object_normals(self, index_object: int) -> numpy.ndarray[numpy.float64[2, n]]:
        """get_object_normals(self: erl_geometry.pyerl_geometry.Surface2D, index_object: int) -> numpy.ndarray[numpy.float64[2, n]]"""
    def get_object_vertices(self, index_object: int) -> numpy.ndarray[numpy.float64[2, n]]:
        """get_object_vertices(self: erl_geometry.pyerl_geometry.Surface2D, index_object: int) -> numpy.ndarray[numpy.float64[2, n]]"""
    def get_vertex_neighbors(self, index_vertex: int) -> tuple[int, int]:
        """get_vertex_neighbors(self: erl_geometry.pyerl_geometry.Surface2D, index_vertex: int) -> tuple[int, int]"""
    @property
    def normals_available(self) -> bool:
        """(arg0: erl_geometry.pyerl_geometry.Surface2D) -> bool"""
    @property
    def num_lines(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.Surface2D) -> int"""
    @property
    def num_objects(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.Surface2D) -> int"""
    @property
    def num_vertices(self) -> int:
        """(arg0: erl_geometry.pyerl_geometry.Surface2D) -> int"""
    @property
    def outside_flags_available(self) -> bool:
        """(arg0: erl_geometry.pyerl_geometry.Surface2D) -> bool"""

class TrajectoryD:
    def __init__(self) -> None:
        """__init__(self: erl_geometry.pyerl_geometry.TrajectoryD) -> None"""
    @staticmethod
    def load_2d(filename: str, binary: bool) -> list[numpy.ndarray[numpy.float64[2, 1]]]:
        """load_2d(filename: str, binary: bool) -> list[numpy.ndarray[numpy.float64[2, 1]]]"""
    @staticmethod
    def load_3d(filename: str, binary: bool) -> list[numpy.ndarray[numpy.float64[3, 1]]]:
        """load_3d(filename: str, binary: bool) -> list[numpy.ndarray[numpy.float64[3, 1]]]"""
    @staticmethod
    def load_se2(filename: str, binary: bool) -> list[tuple[numpy.ndarray[numpy.float64[2, 2]], numpy.ndarray[numpy.float64[2, 1]]]]:
        """load_se2(filename: str, binary: bool) -> list[tuple[numpy.ndarray[numpy.float64[2, 2]], numpy.ndarray[numpy.float64[2, 1]]]]"""
    @staticmethod
    def load_se3(filename: str, binary: bool) -> list[tuple[numpy.ndarray[numpy.float64[3, 3]], numpy.ndarray[numpy.float64[3, 1]]]]:
        """load_se3(filename: str, binary: bool) -> list[tuple[numpy.ndarray[numpy.float64[3, 3]], numpy.ndarray[numpy.float64[3, 1]]]]"""

class TrajectoryF:
    def __init__(self) -> None:
        """__init__(self: erl_geometry.pyerl_geometry.TrajectoryF) -> None"""
    @staticmethod
    def load_2d(filename: str, binary: bool) -> list[numpy.ndarray[numpy.float32[2, 1]]]:
        """load_2d(filename: str, binary: bool) -> list[numpy.ndarray[numpy.float32[2, 1]]]"""
    @staticmethod
    def load_3d(filename: str, binary: bool) -> list[numpy.ndarray[numpy.float32[3, 1]]]:
        """load_3d(filename: str, binary: bool) -> list[numpy.ndarray[numpy.float32[3, 1]]]"""
    @staticmethod
    def load_se2(filename: str, binary: bool) -> list[tuple[numpy.ndarray[numpy.float32[2, 2]], numpy.ndarray[numpy.float32[2, 1]]]]:
        """load_se2(filename: str, binary: bool) -> list[tuple[numpy.ndarray[numpy.float32[2, 2]], numpy.ndarray[numpy.float32[2, 1]]]]"""
    @staticmethod
    def load_se3(filename: str, binary: bool) -> list[tuple[numpy.ndarray[numpy.float32[3, 3]], numpy.ndarray[numpy.float32[3, 1]]]]:
        """load_se3(filename: str, binary: bool) -> list[tuple[numpy.ndarray[numpy.float32[3, 3]], numpy.ndarray[numpy.float32[3, 1]]]]"""

class YamlableBase:
    def __init__(self, *args, **kwargs) -> None:
        """Initialize self.  See help(type(self)) for accurate signature."""
    def as_yaml_file(self, yaml_file: str) -> None:
        """as_yaml_file(self: erl_geometry.pyerl_geometry.YamlableBase, yaml_file: str) -> None"""
    def as_yaml_string(self) -> str:
        """as_yaml_string(self: erl_geometry.pyerl_geometry.YamlableBase) -> str"""
    def from_yaml_file(self, yaml_file: str) -> bool:
        """from_yaml_file(self: erl_geometry.pyerl_geometry.YamlableBase, yaml_file: str) -> bool"""
    def from_yaml_string(self, yaml_str: str) -> bool:
        """from_yaml_string(self: erl_geometry.pyerl_geometry.YamlableBase, yaml_str: str) -> bool"""

def bresenham_2d(start: numpy.ndarray[numpy.int32[2, 1]], end: numpy.ndarray[numpy.int32[2, 1]], stop: Callable[[int, int], bool] | None = ...) -> numpy.ndarray[numpy.int32[2, n]]:
    """bresenham_2d(start: numpy.ndarray[numpy.int32[2, 1]], end: numpy.ndarray[numpy.int32[2, 1]], stop: Optional[Callable[[int, int], bool]] = None) -> numpy.ndarray[numpy.int32[2, n]]"""
def compute_intersection_between_line_and_ellipse_2d(x0: float, y0: float, x1: float, y1: float, a: float, b: float) -> tuple:
    """compute_intersection_between_line_and_ellipse_2d(x0: float, y0: float, x1: float, y1: float, a: float, b: float) -> tuple"""
def compute_intersection_between_line_and_ellipsoid_3d(x0: float, y0: float, z0: float, x1: float, y1: float, z1: float, a: float, b: float, c: float) -> tuple:
    """compute_intersection_between_line_and_ellipsoid_3d(x0: float, y0: float, z0: float, x1: float, y1: float, z1: float, a: float, b: float, c: float) -> tuple"""
def compute_intersection_between_ray_and_aabb_2d(ray_start_point: numpy.ndarray[numpy.float64[2, 1]], ray_direction: numpy.ndarray[numpy.float64[2, 1]], aabb_min: numpy.ndarray[numpy.float64[2, 1]], aabb_max: numpy.ndarray[numpy.float64[2, 1]]) -> dict:
    """compute_intersection_between_ray_and_aabb_2d(ray_start_point: numpy.ndarray[numpy.float64[2, 1]], ray_direction: numpy.ndarray[numpy.float64[2, 1]], aabb_min: numpy.ndarray[numpy.float64[2, 1]], aabb_max: numpy.ndarray[numpy.float64[2, 1]]) -> dict"""
def compute_intersection_between_ray_and_aabb_3d(ray_start_point: numpy.ndarray[numpy.float64[3, 1]], ray_direction: numpy.ndarray[numpy.float64[3, 1]], aabb_min: numpy.ndarray[numpy.float64[3, 1]], aabb_max: numpy.ndarray[numpy.float64[3, 1]]) -> dict:
    """compute_intersection_between_ray_and_aabb_3d(ray_start_point: numpy.ndarray[numpy.float64[3, 1]], ray_direction: numpy.ndarray[numpy.float64[3, 1]], aabb_min: numpy.ndarray[numpy.float64[3, 1]], aabb_max: numpy.ndarray[numpy.float64[3, 1]]) -> dict"""
def compute_intersection_between_ray_and_line_2d(ray_start_point: numpy.ndarray[numpy.float64[2, 1]], ray_direction: numpy.ndarray[numpy.float64[2, 1]], segment_point1: numpy.ndarray[numpy.float64[2, 1]], segment_point2: numpy.ndarray[numpy.float64[2, 1]]) -> tuple:
    """compute_intersection_between_ray_and_line_2d(ray_start_point: numpy.ndarray[numpy.float64[2, 1]], ray_direction: numpy.ndarray[numpy.float64[2, 1]], segment_point1: numpy.ndarray[numpy.float64[2, 1]], segment_point2: numpy.ndarray[numpy.float64[2, 1]]) -> tuple"""
def compute_nearest_distance_from_point_to_line_segment_2d(point_x: float, point_y: float, line_segment_x1: float, line_segment_y1: float, line_segment_x2: float, line_segment_y2: float) -> float:
    """compute_nearest_distance_from_point_to_line_segment_2d(point_x: float, point_y: float, line_segment_x1: float, line_segment_y1: float, line_segment_x2: float, line_segment_y2: float) -> float"""
def compute_pixels_of_polygon_contour(polygon_vertices: numpy.ndarray[numpy.int32[2, n], flags.f_contiguous]) -> numpy.ndarray[numpy.int32[2, n]]:
    """compute_pixels_of_polygon_contour(polygon_vertices: numpy.ndarray[numpy.int32[2, n], flags.f_contiguous]) -> numpy.ndarray[numpy.int32[2, n]]"""
def convert_path_2d_to_3d_float32(path_2d: numpy.ndarray[numpy.float32[3, n], flags.f_contiguous], z: float) -> list[numpy.ndarray[numpy.float32[4, 4]]]:
    """convert_path_2d_to_3d_float32(path_2d: numpy.ndarray[numpy.float32[3, n], flags.f_contiguous], z: float) -> list[numpy.ndarray[numpy.float32[4, 4]]]"""
def convert_path_2d_to_3d_float64(path_2d: numpy.ndarray[numpy.float64[3, n], flags.f_contiguous], z: float) -> list[numpy.ndarray[numpy.float64[4, 4]]]:
    """convert_path_2d_to_3d_float64(path_2d: numpy.ndarray[numpy.float64[3, n], flags.f_contiguous], z: float) -> list[numpy.ndarray[numpy.float64[4, 4]]]"""
def create_ellipsoid_mesh(a: float, b: float, c: float, num_azimuths: int = ..., num_elevations: int = ...) -> tuple:
    """create_ellipsoid_mesh(a: float, b: float, c: float, num_azimuths: int = 360, num_elevations: int = 180) -> tuple"""
def find_voxel_indices(codes: torch.Tensor, dims: int, level: int, children: torch.Tensor, parallel: bool = ...) -> torch.Tensor:
    """find_voxel_indices(codes: torch.Tensor, dims: int, level: int, children: torch.Tensor, parallel: bool = True) -> torch.Tensor


    Find voxel indices from morton codes and tree structure.

    Args:
        codes (torch.Tensor): Tensor of morton code with dtype torch.uint32 or torch.uint64.
        dims (int): Space dimension, 2 or 3.
        level (int): The level to start the search, usually tree_depth - 1.
        children (torch.Tensor): Tensor of shape (M, 2^dims) that stores the tree structure.
        parallel (bool, optional): If true, use parallel for to speed up the search when using CPU. Default is True.

    Returns:
        torch.Tensor: Tensor to store the found voxel indices, has the same shape as codes and the same dtype as children.
    """
def hidden_point_removal(points: numpy.ndarray[numpy.float64[3, n], flags.f_contiguous], view_position: numpy.ndarray[numpy.float64[3, 1]], radius: float, fast: bool = ..., joggle_inputs: bool = ..., return_meshes: bool = ...) -> dict:
    """hidden_point_removal(points: numpy.ndarray[numpy.float64[3, n], flags.f_contiguous], view_position: numpy.ndarray[numpy.float64[3, 1]], radius: float, fast: bool = False, joggle_inputs: bool = False, return_meshes: bool = False) -> dict

    Remove hidden points from the point cloud.

    Args:
        points: A 3xN matrix of points.
        view_position: The position of the camera.
        radius: The radius of the sphere.
        fast: If true, will run QHull with `Q3 Q5 Q8`.
        joggle_inputs: If true, will run QHull with `QJ`.
    Returns:
        A dictionary containing the visible point indices and optionally the mesh vertices and triangles.
    """
def marching_square(img: numpy.ndarray[numpy.float64[m, n], flags.f_contiguous], iso_value: float) -> tuple:
    """marching_square(img: numpy.ndarray[numpy.float64[m, n], flags.f_contiguous], iso_value: float) -> tuple"""
def morton_decode(codes: torch.Tensor, dims: int) -> torch.Tensor:
    """morton_decode(codes: torch.Tensor, dims: int) -> torch.Tensor


    Decode morton codes to coordinates.

    Args:
        codes (torch.Tensor): Tensor of morton code with dtype torch.uint32 or torch.uint64.
        dims (int): Space dimension, 2 or 3.

    Returns:
        torch.Tensor: Output tensor of shape (..., dims) with dtype torch.uint16 or torch.uint32.
    """
def morton_encode(coords: torch.Tensor) -> torch.Tensor:
    """morton_encode(coords: torch.Tensor) -> torch.Tensor


    Encode coordinates to morton codes.

    Args:
        coords (torch.Tensor): Tensor of shape (D1, ..., D2, dims) with dtype torch.uint16 or torch.uint32.

    Returns:
        torch.Tensor: Output tensor of shape (D1, ..., D2) with dtype torch.uint32 or torch.uint64.
    """
def parallel_hidden_point_removal(points: numpy.ndarray[numpy.float64[3, n], flags.f_contiguous], view_positions: numpy.ndarray[numpy.float64[3, n], flags.f_contiguous], radii: numpy.ndarray[numpy.float64[m, 1]], fast: bool = ..., joggle_inputs: bool = ..., return_meshes: bool = ...) -> dict:
    """parallel_hidden_point_removal(points: numpy.ndarray[numpy.float64[3, n], flags.f_contiguous], view_positions: numpy.ndarray[numpy.float64[3, n], flags.f_contiguous], radii: numpy.ndarray[numpy.float64[m, 1]], fast: bool = False, joggle_inputs: bool = False, return_meshes: bool = False) -> dict

    Remove hidden points from the point cloud w.r.t. multiple camera positions. The function is parallelized.

    Args:
        points: A 3xN matrix of points.
        view_positions: A 3xM matrix of camera positions.
        radii: The radius of the spherical projection.
        fast: If true, will run QHull with `Q3 Q5 Q8`.
        joggle_inputs: If true, will run QHull with `QJ`.
    Returns:
        A dictionary containing the visible point indices and optionally the mesh vertices and triangles for each camera position.
    """
def winding_number(p: numpy.ndarray[numpy.float64[2, 1]], vertices: numpy.ndarray[numpy.float64[2, n], flags.f_contiguous]) -> int:
    """winding_number(p: numpy.ndarray[numpy.float64[2, 1]], vertices: numpy.ndarray[numpy.float64[2, n], flags.f_contiguous]) -> int"""
