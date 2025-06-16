### World Package

This is a package  to launch a webots world running a supervisor node: [webots_worlds](https://gitlab.inesctec.pt/agrob/Agrob4Simulation/-/tree/webots/webots_worlds).

To launch webots and open your world add it inside `/worlds` directory and run:
```bash
ros2 launch webots_worlds world_launch.py world:={your_world}.wbt
```