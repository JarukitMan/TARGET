import py_trees as t

import carla
import carla.command
import srunner.scenariomanager.scenarioatomics.atomic_behaviors as a
import srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions as at
import srunner.scenarios.open_scenario as s
import srunner.tools.route_manipulation as m
import target.classes as c

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


