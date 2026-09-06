import yaml as y

import carla
import target.opendriveparser.elements.openDrive as o
import target.classes as c
import target.road_topology as r


def parse_config(config: str) -> c.Configuration:
    # Default configuration
    road = c.Road(2, False, c.RoadMarker.NONE, c.RoadType.STRAIGHT, [])
    configuration = c.Configuration([], road, c.Weather.SUNNY, c.Time.DAY)
    dictionary = y.load(config, y.FullLoader)

    # Extracted environment
    environment = dictionary["Environment"]
    if environment is not None:
        time = environment["Time"]
        weather = environment["Weather"]

        if time is not None:
            configuration = c.Configuration(configuration.actors, configuration.road, configuration.weather, time)
        if weather is not None:
            configuration = c.Configuration(configuration.actors, configuration.road, weather, configuration.time)

    # It's pretty long so I put it in a different function
    actors = get_actors(dictionary["Actors"])
    configuration = c.Configuration(actors, configuration.road, configuration.weather, configuration.time)

    road_network = dictionary["Road network"]
    if road_network is not None:
        road_marker = road_network["Road marker"]
        # Extra
        road_one_way = road_network["Road one way"]
        road_lane_count = road_network["Road lane count"]
        # End extra
        road_type = road_network["Road type"]
        traffic_signs = road_network["Traffic signs"]

        if road_marker is not None:
            road = c.Road(road.lane_count, road.one_way, road_marker, road.road_type, road.props)
        if road_one_way is not None:
            road = c.Road(road.lane_count, road_one_way, road.marker, road.road_type, road.props)
        if isinstance(road_lane_count, int):
            road = c.Road(road_lane_count, road.one_way, road.marker, road.road_type, road.props)
        if road_type is not None:
            road = c.Road(road.lane_count, road.one_way, road.marker, road_type, road.props)
        # Signs might be a list because I extended it.
        if isinstance(traffic_signs, str):
            road = c.Road(road.lane_count, road.one_way, road.marker, road.road_type, [traffic_signs])
        elif isinstance(traffic_signs, list):
            road = c.Road(road.lane_count, road.one_way, road.marker, road.road_type, traffic_signs)
        configuration = c.Configuration(configuration.actors, road, configuration.weather, configuration.time)

    return configuration

def get_actors(actors_dictionary: dict) -> list[c.Actor]:
    actors: list[c.Actor] = []
    if actors_dictionary is not None:
        for actor_name, actor_dictionary in actors_dictionary:
            # Default actor
            actor_relation = c.Relation(c.RelationType.FRONT, c.Distance.MEDIUM, "ego")
            actor = c.Actor(actor_name, actor_relation, c.Behavior.GO_FORWARD)

            # Rename ego to, well, ego.
            if actor_name == "Ego vehicle":
                actor = c.Actor("ego", actor.relation, actor.behavior)

            actor_behavior = actor_dictionary["Behavior"]
            if actor_behavior is not None:
                actor = c.Actor(actor.name, actor.relation, actor_behavior)

            actor_position = actor_dictionary["Position"]
            if actor_position is not None:
                actor_position_reference = actor_position["Position reference"]
                actor_position_relation = actor_position["Position relation"]
                actor_distance = actor_position["Distance from reference"]
                # Fallback
                actor_distance_number = c.Distance.MEDIUM

                if actor_position_reference is not None:
                    actor_relation = c.Relation(actor_relation.relation, actor_relation.distance, actor_position_reference)
                if actor_position_relation is not None:
                    actor_relation = c.Relation(actor_position_relation, actor_relation.distance, actor_relation.object_name)
                if actor_distance is not None:
                    if actor_distance == "contact":
                        actor_distance_number = c.Distance.CONTACT
                    elif actor_distance == "immediate":
                        actor_distance_number = c.Distance.IMMEDIATE
                    elif actor_distance == "near":
                        actor_distance_number = c.Distance.NEAR
                    elif actor_distance == "medium":
                        actor_distance_number = c.Distance.MEDIUM
                    elif actor_distance == "far":
                        actor_distance_number = c.Distance.FAR

                actor_relation = c.Relation(actor_relation.relation, actor_distance_number, actor_relation.object_name)
                actor = c.Actor(actor.name, actor_relation, actor.behavior)

            actors.append(actor)
    return actors
    

def topology_to_routes(
    map: o.OpenDrive, topology: list[tuple[carla.Waypoint, carla.Waypoint]]
):
    return [
        r.Route(i, waypoint[0], waypoint[1], map) for i, waypoint in enumerate(topology)
    ]
