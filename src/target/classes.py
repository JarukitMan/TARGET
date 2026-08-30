from dataclasses import dataclass
from enum import Enum


class RoadType(str, Enum):
    ROUNDABOUT = "roundabout"
    STRAIGHT = "straight road"
    T_INTERSECTION = "t-intersection"
    INTERSECTION = "intersection"


class Prop(str, Enum):
    CROSSWALK = "crosswalk"
    STOP_SIGN = "stop sign"
    SPEED_SIGN = "speed sign"
    TRAFFIC_LIGHT = "traffic light"
    NO_TURN = "no turn left sign"
    DO_NOT_ENTER = "do not enter sign"


class RoadMarker(str, Enum):
    NONE = "None"
    SOLID_LINE = "solid line"
    # BROKEN_LINE = "broken line"


class Weather(str, Enum):
    SUNNY = "sunny"
    FOGGY = "foggy"
    RAINY = "rainy"
    DUSTY = "dusty"
    WET = "wet"


class Time(str, Enum):
    DAY = "daytime"
    NIGHT = "nighttime"


class Behavior(str, Enum):
    STATIC = "static"
    GO_FORWARD = "go forward"
    TURN_LEFT = "turn left"
    TURN_RIGHT = "turn right"
    CHANGE_LANE_TO_LEFT = "change lane to left"
    CHANGE_LANE_TO_RIGHT = "change lane to right"


class RelationType(str, Enum):
    ON = "on"
    IN = "in"
    FRONT = "front"
    BEHIND = "behind"
    LEFT = "left"
    RIGHT = "right"
    LEFT_BEHIND = "left_behind"
    RIGHT_BEHIND = "right_behind"
    LEFT_FRONT = "left_front"
    RIGHT_FRONT = "right_front"


class Distance(float, Enum):
    CONTACT = 0
    IMMEDIATE = 1
    NEAR = 2.5
    MEDIUM = 5
    FAR = 10


@dataclass(unsafe_hash=True)
class Relation:
    relation: RelationType
    distance: Distance
    object_name: str


@dataclass(frozen=True)
class Actor:
    name: str
    relation: Relation
    behavior: Behavior


@dataclass(frozen=True)
class Road:
    lane_count: int
    one_way: bool
    marker: RoadMarker
    road_type: RoadType
    props: list[Prop]


@dataclass(frozen=True)
class Configuration:
    actors: list[Actor]
    road: Road
    weather: Weather
    time: Time
