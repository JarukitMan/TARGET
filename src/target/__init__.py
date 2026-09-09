import argparse
import json

# NOTE: For some reason shows as an error on the linter (ty). etree is definitely there.
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

    # Required arguments
    arguments_parser = argparse.ArgumentParser("target")
    arguments_parser.add_argument("config_name")
    arguments_parser.add_argument("carla_map_name")
    arguments_parser.add_argument("opendrive_map_name")

    # Optional flags
    arguments_parser.add_argument("--carla-ip", default="127.0.0.1")
    arguments_parser.add_argument("--carla-port", default=2000)
    arguments_parser.add_argument("--output")

    arguments = arguments_parser.parse_args()

    config_name = arguments.config_name
    carla_map_name = arguments.carla_map_name
    opendrive_map_name = arguments.opendrive_map_name
    ip = arguments.carla_ip
    port = arguments.port
    if arguments.output is not None:
        output = arguments.output
    else:
        output = config_name + ".xml"

    with open(config_name, "r") as config_file:
        configuration = p.parse_config(config_file.read())

    with open(opendrive_map_name, "r") as opendrive_map_file:
        # Copied from original TARGET.
        opendrive_map_xml = etree.parse(opendrive_map_file).getroot()
        opendrive_map: o.OpenDrive = o.parse_opendrive(opendrive_map_xml)

    # Connect to CARLA Simulator and get the map.
    client = c.Client(host=ip, port=port)
    client.load_world_if_different(carla_map_name)
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
    routes = f.find_props(routes, configuration.road.props, opendrive_map)
    routes = f.filter_actors(routes, configuration.actors)
    # FIXME: Handle cases where there are no applicable routes.
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
    scenario_xml = etree.SubElement(xml, "scenario", name=config_name, type=config_name, town=carla_map_name)
    target_xml = etree.SubElement(scenario_xml, "target")
    _ = etree.SubElement(target_xml, json.dumps(configuration))
    for actor, waypoint in waypoints.items():
        if actor.name == "ego":
            name = "ego_vehicle"
        else:
            name = "other_actor"

        _ = etree.SubElement(scenario_xml, name, x=waypoint.transform.location.x, y=waypoint.transform.location.y, z=waypoint.transform.location.z, yaw=waypoint.transform.rotation.yaw, model="vehicle.tesla.model3")
    xml_tree = etree.ElementTree(xml)
    xml_tree.write(output, pretty_print=True)
