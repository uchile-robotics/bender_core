
# uchile_footprint_generator

This ROS package provides a node to dynamically compute the robot footprint. The footprint is based on a baseline polygon which is augmented by the convex hull of the required robot frames.

The `footprint_generator.py` node was designed to publish a dynamic footprint for navigation purposes. The topic can be used as an input to the costmap_2d package from the Navigation Stack.
