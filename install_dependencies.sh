# Tools
mkdir tarballs

curl -LsSf https://astral.sh/uv/install.sh | sh
curl --location https://github.com/carla-simulator/scenario_runner/archive/refs/tags/v0.9.16.tar.gz --output tarballs/scenario_runner.tar.gz
curl --location https://tiny.carla.org/carla-0-9-16-linux --output tarballs/carla.tar.gz

mkdir scenario_runner
tar xzf tarballs/scenario_runner.tar.gz -C scenario_runner
mv scenario_runner/scenario_runner-0.9.16/* scenario_runner
rm -r scenario_runner/scenario_runner-0.9.16

mkdir carla
tar xzf tarballs/carla.tar.gz -C carla

# For some reason, you need to manually get the agents file from CARLA source. It's not built or provided as a package.
cp --recursive carla/PythonAPI/carla/agents .
