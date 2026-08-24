from dataclasses import dataclass
from enum import Enum


@dataclass
class Actor:
    behavior: str

@dataclass
class Road:
    behavior: str

class Weather(str, Enum):
    SUNNY = 'sunny'
    FOGGY = 'foggy'
    RAINY = 'rainy'
    DUSTY = 'dusty'
    WET = 'wet'

class Time(str, Enum):
    DAY = 'daytime'
    NIGHT = 'nighttime'

@dataclass
class Configuration:
    actors: list[Actor]
    road: Road
    weather: Weather
    time: Time
