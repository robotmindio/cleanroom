# Native ROS reliability patches

`scripts/build-native.sh` builds these pinned packages into the selected compute
workspace. The system ROS installation remains the underlay. The installer,
split deployer and CI invoke the same builder.

- **class_loader 2.7.1**, commit `c404b82f04e6b0cce5c6205626b2d35fdf4dc882`:
  Linux `RTLD_NODELETE` retains plugin code until process exit. ROS executors
  also retain weak control blocks after plugin instances are destroyed; their
  destructors need that code even after callbacks stop. Plugin objects still
  undergo normal destruction. This bounds retained mappings to the set of
  distinct plugins loaded; changing plugin code requires a service restart,
  which split deployment already performs.
- **rclcpp 28.1.22**, commit `3aa906a2c7ad13d1623b31a55731993f56538e72`:
  `CallbackGroup::remove_waitable` must erase expired registrations as well as
  live matching pointers. Action deleters execute after their strong count
  reaches zero, so locking their weak registration fails. Keeping that weak
  control block until the callback group dies calls unloaded plugin code.
  The native action-client regression and MoveIt SIGINT probe exercise this.
- **Nav2 1.3.13**, commit `f4108e5b1c2bce804a1aa0c7be6673a8eb4a1501`:
  bounded service discovery and response share one deadline; the timed
  lifecycle transition returns `response->success`; the lifecycle manager
  applies its configured timeout to transitions, state queries and reconnection.
  Its navigation launch passes the tracked parameter file to the manager.
  LeKiwi config selects ten seconds. Tests cover delayed responses, deadline
  expiry, refused transitions and a disappearing service.

- **RViz Ogre vendor 14.1.23**, commit
  `feb01669f1297df2af755ce9cd2ed18083e7a8b2`, Ogre 1.12.10:
  reading a shader link log must not call `glValidateProgram` before Ogre
  assigns sampler units. Validation at that point sees both different sampler
  types on default unit zero and logs a false error. The patch removes that
  premature validation; actual link status and OpenGL error reporting remain.
  Shader code, palette textures and map rendering are unchanged.

The changes preserve the packaged ABI. Review these patches against new native
versions before changing their pins. Shutdown evidence records the selected
native package prefixes as well as the system package versions.
