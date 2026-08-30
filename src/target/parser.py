
import carla
import opendriveparser.elements.openDrive as o
import target.classes as c
import target.road_topology as r


def parse_config(config: str) -> c.Configuration:
    raise NotImplementedError


def topology_to_routes(
    map: o.OpenDrive, topology: list[tuple[carla.Waypoint, carla.Waypoint]]
):
    return [
        r.Route(i, waypoint[0], waypoint[1], map) for i, waypoint in enumerate(topology)
    ]
