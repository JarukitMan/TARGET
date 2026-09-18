# STADSUNT's reimplementation of TARGET's scenario parser.

## TODO
- [x] Create the types.
- [x] Create the engine.
- [x] Integrate PyYAML.
- [x] Implement set_actor_behavior
- [x] Feed the configuration into ParsedScenario
- [x] Integrate argparse.
- [x] Implement "Any" enumeration in case a constraint is not specified.
- [ ] BUG: The spawn points are reported to collide with some object, and so vehicles cannot be spawned. Might need to do some next iteration with the waypoints if all generated waypoints are not possible.
- [ ] Integrate carla-simulator/scenario_runner

## DSL Difference
- Road signs can now be a list.
- Road lane count is now independent of road type. (int)
- Road marker is now independent of road type.
- Things not specified no longer have a default, instead allowing anything in that category.

## Usage
target SCENARIO_YAML_FILE CARLA_MAP_NAME OPENDRIVE_MAP_XODR_FILE

### flags
- --carla-port: Changes the CARLA port. (Default: 2000)
- --output OUTPUT: Prints the xml file to the output file instead of the standard output.

## Dependencies
Installation script included at `install_dependencies.sh`.
All tools are expected to be in the same directory.
You can pass the options as flags to the program to indicate alternate executable locations. 
- curl (Not included in the install script.)
- tar (Also not included in the install script.)
- uv
- Python 3.12
- CARLA 0.9.16
- CARLA 0.9.16's agents folder
- carla-simulator/scenario_runner:0.9.16
Then please initialize the dependencies as packages yourself using `uv init {DIRECTORY_NAME}` and adding their dependencies.
I will add it to the shell script later.

## Installation
After installing all the dependencies correctly, run `uv tool install .`.

## NOTES
This project uses a vendored version of Stefan Urban's OpenDRIVE parser. It is inside the src/target/opendriveparser directory.
