"""Keep plain pytest runs off the live robot's ROS graph.

ctest gives every launch test its own ROS_DOMAIN_ID. A direct `pytest test/` on a
workstation that also runs the robot stack (domain 0 by default) would otherwise
publish test safety states, commands and TF into the live graph.
"""
import os

os.environ.setdefault("ROS_DOMAIN_ID", "231")
