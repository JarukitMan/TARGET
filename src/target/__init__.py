import argparse
import json

# NOTE: For some reason shows as an error on the linter. etree is definitely there.
from lxml import etree

import carla as c
import target.opendriveparser.parser as o
import target.filter as f
import target.parser as p
import target.road_topology as r


# TODO: Make this file the `runner`. Meaning this parses the flags and calls the scenario runner.
# Read the configuration
# Get the routes
# Create a basic scenario
# Modify it with set_X filters
# Run the scenario
def main() -> None:

    # TODO: Make actual flags.
    config_name = "TODO"
    map_name = "TODO"
    ip = "127.0.0.1"
    port = 2000

    with open(config_name, "r") as config_file:
        configuration = p.parse_config(config_file.read())

    with open(map_name, "r") as map_file:
        # Copied from original TARGET.
        map_xml = etree.parse(map_file).getroot()
        map: o.OpenDrive = o.parse_opendrive(map_xml)

    # Connect to CARLA Simulator and get the map.
    client = c.Client(host=ip, port=port)
    world = client.get_world()
    carla_map = world.get_map()

    # Based on this part of the original TARGET.
    # topology = current_map.get_topology()
    # routes = []
    # for i, t in enumerate(topology):
    #     route = Route(i, t[0], t[1], map_info)
    #     routes.append(route)
    routes = [r.Route(i, t[0], t[1], map) for i, t in enumerate(carla_map.get_topology())]

    # This part filters the routes.
    routes = f.find_road_type(routes, configuration.road.road_type)
    routes = f.find_marker(routes, configuration.road.marker)
    routes = f.find_lane_count(routes, configuration.road.lane_count)
    routes = f.find_props(routes, configuration.road.props, map)
    routes = f.filter_actors(routes, configuration.actors)
    waypoints = f.get_actor_positions(routes[0], configuration.actors)

    # I see that in TARGET they use an extended ScenarioRunner and it doesn't look too bad.
    # NOTE: I can't just create a scenario. I can either take the entirety of ScenarioRunner, or I can turn ALL this into a scenario. I think all this fits into the init.
    # TODO: Use this to generate the XML.
    # NOTE: Thanks to:
                    # # Any other possible element, add it as a config attribute
                    # else:
                    #     config.other_parameters[elem.tag] = elem.attrib
    # We can secretly feed the scenario the configuration object by dumping it in the XML file. Perhaps as a JSON string.

    # scenario_config = sc.ScenarioConfiguration()
    # scenario = s.BasicScenario("TARGET Scenario", srunner.ego_vehicles, scenario_config, world)
    # scenario = f.set_weather(scenario, configuration.weather)
    # scenario = f.set_time(scenario, configuration.time)
    xml = etree.Element("scenarios")
    scenario_xml = etree.SubElement(xml, "scenario", name=config_name, type=config_name, town=map_name)
    target_xml = etree.SubElement(scenario_xml, "target")
    _ = etree.SubElement(target_xml, json.dumps(configuration))
    for actor, waypoint in waypoints.items():
        if actor.name == "ego":
            name = "ego_vehicle"
        else:
            name = "other_actor"

        _ = etree.SubElement(scenario_xml, name, x=waypoint.transform.location.x, y=waypoint.transform.location.y, z=waypoint.transform.location.z, yaw=waypoint.transform.rotation.yaw, model="vehicle.tesla.model3")
    xml_tree = etree.ElementTree(xml)
    xml_tree.write(config_name + ".xml", pretty_print=True)
