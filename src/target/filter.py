import copy

import py_trees as t

import carla
import carla.command
import target.opendriveparser.elements.openDrive as o
import srunner.scenariomanager.scenarioatomics.atomic_behaviors as a
import srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions as at
import srunner.scenarios.open_scenario as s
import srunner.tools.route_manipulation as m
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


def set_traffic_light(scenario: s.BasicScenario, red_light: bool) -> s.BasicScenario:

    new_scenario = scenario
    traffic_light_behavior = t.composites.Sequence(
        policy=t.common.ParallelPolicy.SUCCESS_ON_ONE
    )
    traffic_lights = m.CarlaDataProvider._traffic_light_map

    for traffic_light in traffic_lights:
        if red_light:
            traffic_light_behavior.add_child(a.TrafficLightStateSetter(traffic_light, carla.TrafficLightState.Red))
        else:
            traffic_light.set_red_time(3)
            traffic_light_behavior.add_child(a.TrafficLightStateSetter(traffic_light, carla.TrafficLightState.Green))

    traffic_light_behavior.add_child(a.Idle(60))

    if new_scenario.behavior_tree:
        new_scenario.behavior_tree.add_child(traffic_light_behavior)
    else:
        new_scenario.behavior_tree = t.composites.Sequence(children=traffic_light_behavior)

    return new_scenario

# Helper
def find_goal(behavior: c.Behavior, waypoint: carla.Waypoint) -> carla.Waypoint:
    futures = waypoint.next(5)
    if futures == []:
        raise ValueError

    if behavior == c.Behavior.CHANGE_LANE_TO_LEFT:
        future = futures[0].get_left_lane()
    elif behavior == c.Behavior.CHANGE_LANE_TO_RIGHT:
        future = futures[0].get_right_lane()
    else:
        future = futures[0]

    if future is not None:
        return future
    else:
        raise ValueError

# NOTE: Taken from the original target.
# Used to generate routes for the NPC actors to take.
# I don't know why the first output is returned if it's ignored but whatever.
def gen_npc_route(global_plan_gps, global_plan_world_coord):
    ds_ids = m.downsample_route(global_plan_world_coord, 1)
    route_world_coord = [(global_plan_world_coord[x][0], global_plan_world_coord[x][1])
                                     for x in ds_ids]
    route_plan = [global_plan_gps[x] for x in ds_ids]
    return route_plan, route_world_coord
    
# NOTE: Modified from the original TARGET code.
def set_behavior(scenario: s.BasicScenario, waypoints: dict[c.Actor, carla.Waypoint]) -> s.BasicScenario:
    # behaviors for other actors
    if len(waypoints) <= 1:
        return scenario

    new_scenario = scenario
    new_waypoints = {actor: waypoint for actor, waypoint in waypoints.items() if actor.name != "ego"}
    for i, (actor, waypoint) in enumerate(new_waypoints.items()):

        start_location = waypoint.transform.location
        goal_location = find_goal(actor.behavior, waypoint).transform.location

        gps_route, route = m.interpolate_trajectory([start_location, goal_location])
        _, actor_plan_temp  = gen_npc_route(gps_route, route)
        actor_plan = [(m.CarlaDataProvider.get_map().get_waypoint(step[0].location)) for step in actor_plan_temp]

        for p in actor_plan_temp:
            waypoint = m.CarlaDataProvider.get_map().get_waypoint(p[0].location)
            actor_plan.append((waypoint, m.RoadOption.LANEFOLLOW))
        actor_behavior = t.composites.Sequence(policy=t.common.ParallelPolicy.SUCCESS_ON_ONE)
        driving_distance = at.DriveDistance(
            scenario.other_actors[0],
            50,
            name="Distance")

        waypoint_follower = a.WaypointFollower(scenario.other_actors[i], 9, plan=actor_plan, avoid_collision=False)
        actor_behavior.add_child(waypoint_follower)
        actor_behavior.add_child(driving_distance)

        if new_scenario.behavior_tree:
            new_scenario.behavior_tree.add_child(actor_behavior)
        else:
            new_scenario.behavior_tree = t.composites.Sequence(children=actor_behavior)

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
    elif road_type == c.RoadType.ANY:
        pass

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
    if marker == c.RoadMarker.ANY:
        pass
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
    if lane_count > 0:
        return [route for route in routes if route.num_lanes == lane_count]
    else:
        return routes


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
# Iterates through all the possible routes and set the waypoints of the actors that reference the road
# before passing it to the recursive find_combination.
def get_actor_positions(routes: list[r.Route], actors: list[c.Actor], client: carla.Client) -> dict[c.Actor, carla.Waypoint]:
    world = client.get_world()
    blueprint = world.get_blueprint_library().find("vehicle.tesla.model3")
    actor_names = [actor.name for actor in actors]
    dependent_actors = [actor for actor in actors if actor.relation.object_name in actor_names]
    for route in routes:
        possible_concrete_waypoints = find_possible_concrete_waypoints(route.start_waypoint, actors, actor_names)
                
        for concrete_waypoints in possible_concrete_waypoints:
            # print(concrete_waypoints)
            concrete_actors = list[carla.Actor]()
            for concrete_waypoint in concrete_waypoints.values():
                transform = concrete_waypoint.transform
                transform.location.z = 0.5
                concrete_actor = world.try_spawn_actor(blueprint, transform)
                if concrete_actor is not None:
                    concrete_actors.append(concrete_actor)
                else:
                    print("Can't spawn concrete actor! Might cause bugs!")
            combination = find_combination(dependent_actors, concrete_waypoints, world, blueprint)
            [concrete_actor.destroy() for concrete_actor in concrete_actors]
            if combination is not None:
                print(combination)
                return combination
    raise ValueError("We can't find a suitable spawn point for vehicles as detailed in this rule. Sorry.")

def find_possible_concrete_waypoints(waypoint: carla.Waypoint, actors: list[c.Actor], actor_names: list[str]) -> list[dict[c.Actor, carla.Waypoint]]:
    # Recursion base-case
    if actors == []:
        return []
    actor = actors[0]
    # Skip this one if it's not a concrete waypoint
    if actor.relation.object_name in actor_names:
        return find_possible_concrete_waypoints(waypoint, actors[1:], actor_names)

    concrete_waypoints = transform_based_on_relation(actor, waypoint)
    search_result = find_possible_concrete_waypoints(waypoint, actors[1:], actor_names)
    # Handle cases where this is the last valid member.
    if search_result == []:
        return [{actor: concrete_waypoint} for concrete_waypoint in concrete_waypoints]

    output = list[dict[c.Actor, carla.Waypoint]]()
    for concrete_waypoint in concrete_waypoints:
        new_search_result = list[dict[c.Actor, carla.Waypoint]]()
        for member in search_result:
            new_member = member
            new_member[actor] = concrete_waypoint
            new_search_result.append(new_member)
        output = output + new_search_result

    return output

# TODO: This needs to be a recursive search so that it covers every combination.
# Inputs would be: (anchor waypoint, world (or client), actors)
    # Find places to spawn the guy from waypoints dictionary, if reference doesn't exist yet, call on the reference.
    # We are not afraid of cyclic dependencies because you REALLY shouldn't write a rule with cyclic dependencies anyways.
    # Spawn the guy
    # Recursively call for the next actor
    # If fail, go to the next possible spawn of the guy
def find_combination(actors: list[c.Actor], waypoints: dict[c.Actor, carla.Waypoint], world: carla.World, blueprint: carla.ActorBlueprint) -> dict[c.Actor, carla.Waypoint] | None:
    # Recursion base-case
    if actors == []:
        return waypoints


    main_actor = actors[0]
    # Find the reference this actor uses in the dictionary.
    for actor, waypoint in waypoints.items():
        if actor.name == main_actor.relation.object_name:
            reference_waypoint = waypoint
            break
    else:
        for actor in actors:
            if actor.name == main_actor.relation.object_name:
                reference_actor = actor
                break
        else:
            print("Can't find the actor's relation object!", main_actor.name, main_actor.relation.object_name)
            return None
        new_actors = actors
        new_actors.remove(reference_actor)
        new_actors = [reference_actor] + new_actors
        new_waypoints = find_combination(new_actors, waypoints, world, blueprint)
        if new_waypoints is None:
            return None
        reference_waypoint = new_waypoints[actor]

    # Get the possible waypoints this guy can use.
    possible_waypoints = transform_based_on_relation(main_actor, reference_waypoint)

    # This block tries to recursively call the children to try all the possible locations this one could be in
    for possible_waypoint in possible_waypoints:
        transform = possible_waypoint.transform
        transform.location.z = 0.5
        actor = world.try_spawn_actor(blueprint, transform)
        if actor is not None:
            print("Spawned actor at", transform)
            new_waypoints = waypoints
            new_waypoints[main_actor] = possible_waypoint
            search_result = find_combination(actors[1:], new_waypoints, world, blueprint)
            # This only returns when there is a result, so this loop will not break until
            # all possibilities have been exhausted or we find a combination.
            if search_result is not None:
                # Cleanup after the work is done.
                actor.destroy()
                new_search_result = search_result
                new_search_result[main_actor] = possible_waypoint
                return new_search_result
            # Cleanup possible obstacle from previous search attempt.
            actor.destroy()
        else:
            print("Can't spawn actor at", transform)
    # All possibilities exhausted
    return None

# This helper function gets the possible waypoints from the reference waypoint
def transform_based_on_relation(actor: c.Actor, object_waypoint: carla.Waypoint) -> list[carla.Waypoint]:
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

    return possible_waypoints

