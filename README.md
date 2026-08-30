# STADSUNT's reimplementation of TARGET's scenario parser.

## TODO
- [x] Create the types.
- [x] Create the engine.
- [ ] Integrate PyYAML.
- [ ] Integrate argparse.
- [ ] Integrate carla-simulator/scenario_runner

## Usage
target MAP.xodr SCENARIO.yaml

### flags
- --only-generate-xml: Only generates the XML file.
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

## Installation
After installing all the dependencies correctly, run `uv tool install target`.
