import json

import py_trees

import target.filter as f
import target.road_topology as r
from scenario_runner.srunner.scenariomanager.scenarioatomics.atomic_criteria import (
    CollisionTest,
)
from scenario_runner.srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (
    InTriggerDistanceToLocation,
)
from scenario_runner.srunner.scenarios.basic_scenario import BasicScenario
from target.classes import Configuration, Prop
from target.filter import set_behavior, set_traffic_light, set_weather


class ParsedScenario(BasicScenario):

    def _create_behavior(self) -> py_trees.composites.Composite:
        self.behavior_config: Configuration = json.loads(self.config['target'])

        # This part filters the routes.
        map = self.world.get_map()
        routes = [r.Route(i, t[0], t[1], map) for i, t in enumerate(map.get_topology())]
        routes = f.find_road_type(routes, self.behavior_config.road.road_type)
        routes = f.find_marker(routes, self.behavior_config.road.marker)
        routes = f.find_lane_count(routes, self.behavior_config.road.lane_count)
        routes = f.find_props(routes, self.behavior_config.road.props, map)
        routes = f.filter_actors(routes, self.behavior_config.actors)
        waypoints = f.get_actor_positions(routes[0], self.behavior_config.actors)

        # This part sets the behaviors.
        new_self = set_traffic_light(self, Prop.TRAFFIC_LIGHT in self.behavior_config.road.props)
        new_self = set_weather(new_self, self.behavior_config.weather)
        new_self = set_behavior(new_self, waypoints)


        end_condition = InTriggerDistanceToLocation(self.ego_vehicles[0],
                                                    next(waypoint for actor, waypoint in waypoints.items() if actor.name == 'ego'),
                                                    3,
                                                    name="ego reaches its destination")

        # NOTE: Can't be none so if it's none here something is seriously wrong.
        if new_self.behavior_tree is None:
            raise ValueError
        new_self.behavior_tree.add_child(end_condition)
        # py_trees.display.render_dot_tree(root)
        return new_self.behavior_tree

    # TODO: We'll decide on what to test later...
    def _create_test_criteria(self):
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0], terminate_on_failure=self.terminate_on_failure)
        criteria.append(collision_criterion)

        # keep_clear_criterion = KeepClearTest(self.ego_vehicles[0], terminate_on_failure=self.terminate_on_failure)
        # criteria.append(keep_clear_criterion)

        # if self.route_info and self.route_info['stop_sign']:
        #     stop_sign_criterion = RunningStopTest(self.ego_vehicles[0],
        #                                           list_stop_signs=self.route_info['stop_sign'],
        #                                           terminate_on_failure=self.terminate_on_failure)
        #     criteria.append(stop_sign_criterion)

        # if self.route_info and self.route_info['traffic_light']:
        #     red_light_criterion = RunningRedLightTest(self.ego_vehicles[0],
        #                                               terminate_on_failure=self.terminate_on_failure)
        #     criteria.append(red_light_criterion)

        # if self.behavior_config['Oracle'] and 'yield' == self.behavior_config['Oracle']['longitudinal']:
        #     give_way_criterion = GiveWayTest(self.ego_vehicles[0], self.actor_info['ego_vehicle']['route'][
        #             -1].start_waypoint,
        #                                      terminate_on_failure=self.terminate_on_failure)
        #     criteria.append(give_way_criterion)

        # if self.behavior_config['Oracle'] and 'keep lane' == self.behavior_config['Oracle']['lateral']:
        #     keep_lane_criterion = KeepLaneTest(self.ego_vehicles[0], terminate_on_failure=self.terminate_on_failure)
        #     criteria.append(keep_lane_criterion)

        # if self.behavior_config['Oracle'] and 'decelerate' == self.behavior_config['Oracle']['longitudinal']:
        #     decelerate_criterion = DecelerateTest(self.ego_vehicles[0], terminate_on_failure=self.terminate_on_failure)
        #     criteria.append(decelerate_criterion)

        # if self.behavior_config['Oracle'] and 'no turn left' == self.behavior_config['Oracle']['lateral']:
        #     no_turn_left_criterion = NoTurnTest(self.ego_vehicles[0],
        #                                             turn_location=self.actor_info['ego_vehicle']['route'][
        #                                                 -1].end_waypoint.transform.location,
        #                                             terminate_on_failure=self.terminate_on_failure)
        #     criteria.append(no_turn_left_criterion)

        # keep_distance_criterion = KeepDistanceTest(self.ego_vehicles[0], self.other_actors,
        #                                            terminate_on_failure=self.terminate_on_failure)
        # criteria.append(keep_distance_criterion)
        return criteria

