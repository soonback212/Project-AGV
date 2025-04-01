jdamr@jdamr-pc:~/agv_ws$ colcon build --packages-select cartographer_ros
[2.872s] WARNING:colcon.colcon_core.package_selection:Some selected packages are already built in one or more underlay workspaces:
	'cartographer_ros' is in: /opt/ros/humble
If a package in a merged underlay workspace is overridden and it installs headers, then all packages in the overlay must sort their include directories by workspace order. Failure to do so may result in build failures or undefined behavior at run time.
If the overridden package is used by another package in any underlay, then the overriding package in the overlay must be API and ABI compatible or undefined behavior at run time may occur.

If you understand the risks and want to override a package anyways, add the following to the command line:
	--allow-overriding cartographer_ros

This may be promoted to an error in a future release of colcon-override-check.
Starting >>> cartographer_ros
[2.927s] WARNING:colcon.colcon_core.shell:The following packages are in the workspace but haven't been built:
- cartographer_ros_msgs
They are being used from the following locations instead:
- /opt/ros/humble
To suppress this warning ignore these packages in the workspace:
--packages-ignore cartographer_ros_msgs
--- stderr: cartographer_ros                         
CMake Error at CMakeLists.txt:54 (find_package):
  By not providing "Findcatkin.cmake" in CMAKE_MODULE_PATH this project has
  asked CMake to find a package configuration file provided by "catkin", but
  CMake did not find one.

  Could not find a package configuration file provided by "catkin" with any
  of the following names:

    catkinConfig.cmake
    catkin-config.cmake

  Add the installation prefix of "catkin" to CMAKE_PREFIX_PATH or set
  "catkin_DIR" to a directory containing one of the above files.  If "catkin"
  provides a separate development package or SDK, be sure it has been
  installed.


---
Failed   <<< cartographer_ros [6.15s, exited with code 1]

Summary: 0 packages finished [8.29s]
  1 package failed: cartographer_ros
  1 package had stderr output: cartographer_ros

