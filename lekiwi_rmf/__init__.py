"""LeKiwi ROS 2 integration."""

from pkgutil import extend_path

# Source-tree tests also need the action types generated in the build overlay.
__path__ = extend_path(__path__, __name__)
