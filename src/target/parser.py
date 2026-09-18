import yaml as y

import carla
import target.classes as c
import target.opendriveparser.elements.openDrive as o
import target.road_topology as r


def parse_config(config: str) -> c.Configuration:
    # Default configuration
    road = c.Road(0, False, c.RoadMarker.ANY, c.RoadType.ANY, [])
    configuration = c.Configuration([], road, c.Weather.SUNNY, c.Time.DAY)
    dictionary = y.load(config, y.FullLoader)

    # Extracted environment
    if "Environment" in dictionary:
        environment = dictionary["Environment"]

        if "Time" in environment:
            time = environment["Time"]
            configuration = c.Configuration(configuration.actors, configuration.road, configuration.weather, time)

        if "Weather" in environment:
            weather = environment["Weather"]
            configuration = c.Configuration(configuration.actors, configuration.road, weather, configuration.time)

    # It's pretty long so I put it in a different function
    actors = get_actors(dictionary["Actors"])
    configuration = c.Configuration(actors, configuration.road, configuration.weather, configuration.time)

    if "Road network" in dictionary:
        road_network = dictionary["Road network"]


        if "Road marker" in road_network:
            road_marker = road_network["Road marker"]
            road = c.Road(road.lane_count, road.one_way, road_marker, road.road_type, road.props)
        # Extra
        if "Road one way" in road_network:
            road_one_way = road_network["Road one way"]
            road = c.Road(road.lane_count, road_one_way, road.marker, road.road_type, road.props)
        if "Road lane count" in road_network:
            road_lane_count = road_network["Road lane count"]
            if isinstance(road_lane_count, int):
                road = c.Road(road_lane_count, road.one_way, road.marker, road.road_type, road.props)
        # End extra
        if "Road type" in road_network:
            road_type = road_network["Road type"]
            road = c.Road(road.lane_count, road.one_way, road.marker, road_type, road.props)
        if "Traffic signs" in road_network:
            traffic_signs = road_network["Traffic signs"]
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
        for actor_name, actor_dictionary in actors_dictionary.items():
            # Default actor
            actor_relation = c.Relation(c.RelationType.FRONT, c.Distance.MEDIUM, "ego")
            actor = c.Actor(actor_name, actor_relation, c.Behavior.GO_FORWARD)

            # Rename ego to, well, ego.
            if actor_name == "Ego vehicle":
                actor = c.Actor("ego", actor.relation, actor.behavior)

            if "Behavior" in actor_dictionary:
                actor_behavior = actor_dictionary["Behavior"]
                actor = c.Actor(actor.name, actor.relation, actor_behavior)

            if "Position" in actor_dictionary:
                actor_position = actor_dictionary["Position"]

                if "Position reference" in actor_position:
                    actor_position_reference = actor_position["Position reference"]
                    if actor_position_reference == "Ego vehicle":
                        actor_position_reference = "ego"
                    actor_relation = c.Relation(actor_relation.relation, actor_relation.distance, actor_position_reference)

                if "Position relation" in actor_position:
                    actor_position_relation = actor_position["Position relation"]
                    actor_relation = c.Relation(actor_position_relation, actor_relation.distance, actor_relation.object_name)

                if "Distance from reference" in actor_position:
                    actor_distance = actor_position["Distance from reference"]
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
                    else:
                        # Fallback
                        actor_distance_number = c.Distance.MEDIUM
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
