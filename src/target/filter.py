import py_trees as t

import carla
import opendriveparser.elements.openDrive as o
import scenario_runner.srunner.scenariomanager.scenarioatomics.atomic_behaviors as a
import scenario_runner.srunner.scenarios.open_scenario as s
import target.classes as c
import target.road_topology as r


# This function returns a new scenario with the weather defined by the user.
def set_weather(scenario: s.BasicScenario, weather: c.Weather) -> s.BasicScenario:

    new_scenario = scenario
    weather_behavior = t.composites.Sequence(
        policy=t.common.ParallelPolicy.SUCCESS_ON_ONE
    )

    for i in range(100):
        # FIXME: This is assuming new_weather is not frozen.
        new_weather = scenario.world.get_weather()
        if weather == c.Weather.FOGGY:
            new_weather.fog_density = min(i * 3, 100)
        if weather == c.Weather.RAINY:
            new_weather.precipitation = min(i * 3, 100)
        if weather == c.Weather.DUSTY:
            new_weather.dust_storm = min(i * 3, 100)
        if weather == c.Weather.WET:
            new_weather.wetness = min(i * 10, 100)

        weather_behavior.add_child(a.ChangeWeather(new_weather))
        weather_behavior.add_child(a.Idle(0.2))

    if new_scenario.behavior_tree:
        new_scenario.behavior_tree.add_child(weather_behavior)
    else:
        new_scenario.behavior_tree = t.composites.Sequence(children=weather_behavior)

    return new_scenario

# This function returns a new scenario with the time defined by the user.
def set_time(scenario: s.BasicScenario, time: c.Time) -> s.BasicScenario:

    new_scenario = scenario
    time_behavior = t.composites.Sequence(
        policy=t.common.ParallelPolicy.SUCCESS_ON_ONE
    )

    for i in range(100):
        # FIXME: This is assuming new_time is not frozen.
        new_time = scenario.world.get_weather()
        if time == c.Time.DAY:
            # TODO: Check if this is fine.
            new_time.sun_altitude_angle = min(i * 3, 100)
        if time == c.Time.NIGHT:
            new_time.sun_altitude_angle = i * 3 / 4

        time_behavior.add_child(a.ChangeWeather(new_time))
        time_behavior.add_child(a.Idle(0.2))

    if new_scenario.behavior_tree:
        new_scenario.behavior_tree.add_child(time_behavior)
    else:
        new_scenario.behavior_tree = t.composites.Sequence(children=time_behavior)

    return new_scenario


# This function filters the routes where the road type.
def find_road_type(routes: list[r.Route], road_type: c.RoadType) -> list[r.Route]:

    # Hard-coded.
    if road_type == c.RoadType.ROUNDABOUT:
        spawn_point, goal = r.create_specific_route(routes, 30, 5, 28, -2)
        routes = [r.Route(999, spawn_point, goal, map)]
    elif road_type == c.RoadType.STRAIGHT:
        routes = [
            route
            for route in routes
            if not route.is_intersection(routes) and not route.is_t_intersection(routes)
        ]
    elif road_type == c.RoadType.T_INTERSECTION:
        routes = [route for route in routes if route.is_t_intersection(routes)]
    elif road_type == c.RoadType.INTERSECTION:
        routes = [route for route in routes if route.is_intersection(routes)]

    return routes


# This function finds the routes where all the props required by the scenario exist.
def find_props(
    routes: list[r.Route], props: list[c.Prop], map: o.OpenDrive
) -> list[r.Route]:
    for prop in props:
        if prop == c.Prop.CROSSWALK:
            routes = [route for route in routes if route.has_crosswalk]
        if prop == c.Prop.SPEED_SIGN:
            routes = [route for route in routes if route.has_speed_sign]
        if prop == c.Prop.STOP_SIGN:
            routes = [route for route in routes if route.has_stop_sign]
        if prop == c.Prop.TRAFFIC_LIGHT:
            routes = [route for route in routes if route.has_traffic_light]
        if prop == c.Prop.NO_TURN:
            routes = [route for route in routes if route.is_no_turn(map, routes)]
        if prop == c.Prop.DO_NOT_ENTER:
            routes = [route for route in routes if route.do_not_enter]

    return routes


# This function filters the routes by the road marker.
def find_marker(routes: list[r.Route], marker: c.RoadMarker) -> list[r.Route]:
    if marker == c.RoadMarker.SOLID_LINE:
        routes = [
            route
            for route in routes
            if "solid" in str(route.road_mark_left).lower()
            and "solid" in str(route.road_mark_right).lower()
        ]
    return routes


# This function filters the routes by the amount of lanes.
def find_lane_count(routes: list[r.Route], lane_count: int) -> list[r.Route]:
    return [route for route in routes if route.num_lanes == lane_count]


# This function filters by the actions all the actors want to take.
# It really only checks for lane changes, though, since those are easy.
def filter_actors(routes: list[r.Route], actors: list[c.Actor]) -> list[r.Route]:
    for actor in actors:
        if actor.behavior == c.Behavior.CHANGE_LANE_TO_LEFT:
            # NOTE: Originally, there was a `and route.start_lane > route.num_lanes` clause.
            routes = [route for route in routes if route.start_lane < 0]
        elif actor.behavior == c.Behavior.CHANGE_LANE_TO_RIGHT:
            routes = [route for route in routes if route.start_lane > 0]
        # elif actor.behavior == c.Behavior.GO_FORWARD:
        #     pass
        # elif actor.behavior == c.Behavior.STATIC:
        #     pass
        # elif actor.behavior == c.Behavior.TURN_LEFT:
        #     pass
        # elif actor.behavior == c.Behavior.TURN_RIGHT:
        #     pass
    return routes


# This function gets the actor positions to use in defining the scenario_runner configuration.
def get_actor_positions(route: r.Route, actors: list[c.Actor]) -> list[carla.Waypoint]:

    # I use a dictionary so that I could fetch pre-existing reference waypoints to use.
    waypoints = dict[str, carla.Waypoint]()
    while len(actors) > len(waypoints):
        for actor in actors:
            # This block gets the reference waypoint that I use to get the current actor's waypoint.
            if actor.relation.object_name in [actor.name for actor in actors]:
                if actor.relation.object_name in waypoints:
                    object_waypoint = waypoints[actor.relation.object_name]
                else:
                    # This is a continue not an exception because maybe its dependency exists, just not reached yet.
                    continue
            else:
                # This is in the case where the reference is the road. I don't really care about the details.
                object_waypoint = route.start_waypoint

            # This block gets the possible waypoints from the reference waypoint
            possible_waypoints = []
            if actor.relation.relation == c.RelationType.BEHIND:
                possible_waypoints = object_waypoint.previous(actor.relation.distance)
            elif actor.relation.relation == c.RelationType.FRONT:
                possible_waypoints = object_waypoint.next(actor.relation.distance)
            elif actor.relation.relation == c.RelationType.IN:
                possible_waypoints = [object_waypoint]
            elif actor.relation.relation == c.RelationType.LEFT:
                possible_waypoint = object_waypoint.get_left_lane()
                if possible_waypoint != None:
                    possible_waypoints = [possible_waypoint]
            elif actor.relation.relation == c.RelationType.LEFT_BEHIND:
                possible_waypoint = object_waypoint.get_left_lane()
                if possible_waypoint != None:
                    possible_waypoints = possible_waypoint.previous(
                        actor.relation.distance
                    )
            elif actor.relation.relation == c.RelationType.LEFT_FRONT:
                possible_waypoint = object_waypoint.get_left_lane()
                if possible_waypoint != None:
                    possible_waypoints = possible_waypoint.next(actor.relation.distance)
            elif actor.relation.relation == c.RelationType.ON:
                possible_waypoints = [object_waypoint]
            elif actor.relation.relation == c.RelationType.RIGHT:
                possible_waypoint = object_waypoint.get_right_lane()
                if possible_waypoint != None:
                    possible_waypoints = [possible_waypoint]
            elif actor.relation.relation == c.RelationType.RIGHT_BEHIND:
                possible_waypoint = object_waypoint.get_right_lane()
                if possible_waypoint != None:
                    possible_waypoints = possible_waypoint.previous(
                        actor.relation.distance
                    )
            elif actor.relation.relation == c.RelationType.RIGHT_FRONT:
                possible_waypoint = object_waypoint.get_right_lane()
                if possible_waypoint != None:
                    possible_waypoints = possible_waypoint.next(actor.relation.distance)

            # This part actually assigns the possible waypoints to the dictionary of waypoints.
            # If there's no possible waypoints, that actor is impossible to construct.
            if possible_waypoints != []:
                waypoints[actor.name] = possible_waypoints[0]
            else:
                raise Exception(
                    f"No possible waypoints that fit the criteria: {actor.relation}"
                )

    return list[carla.Waypoint](waypoints.values())
