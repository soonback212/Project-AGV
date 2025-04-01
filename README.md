The class required for this panel, 'rviz/Displays', could not be loaded.
Error:
According to the loaded plugin descriptions the class rviz/Displays with base class type rviz_common::Panel does not exist. Declared types are nav2_rviz_plugins/Navigation 2 slam_toolbox::SlamToolboxPlugin


[robot_state_publisher-6] [INFO] [1743493236.815141162] [rclcpp]: signal_handler(signum=2)
[cartographer_occupancy_grid_node-9] [INFO] [1743493236.815141200] [rclcpp]: signal_handler(signum=2)
[static_transform_publisher-5] [INFO] [1743493236.815690032] [rclcpp]: signal_handler(signum=2)
[static_transform_publisher-4] [INFO] [1743493236.815865124] [rclcpp]: signal_handler(signum=2)
[robot_state_publisher-3] [INFO] [1743493236.816146142] [rclcpp]: signal_handler(signum=2)
[static_transform_publisher-2] [INFO] [1743493236.816285308] [rclcpp]: signal_handler(signum=2)
[ldlidar_sl_ros2_node-1] [INFO] [1743493236.816760770] [rclcpp]: signal_handler(signum=2)
[imu_parser_node-10] Traceback (most recent call last):
[imu_parser_node-10]   File "/home/jdamr/agv_ws/install/slam_control/lib/slam_control/imu_parser_node", line 33, in <module>
[imu_parser_node-10]     sys.exit(load_entry_point('slam-control==0.0.0', 'console_scripts', 'imu_parser_node')())
[imu_parser_node-10]   File "/home/jdamr/agv_ws/install/slam_control/lib/python3.10/site-packages/slam_control/imu_parser_node.py", line 118, in main
[imu_parser_node-10]     rclpy.spin(node)
[imu_parser_node-10]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/__init__.py", line 226, in spin
[imu_parser_node-10]     executor.spin_once()
[imu_parser_node-10]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 751, in spin_once
[imu_parser_node-10]     self._spin_once_impl(timeout_sec)
[imu_parser_node-10]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 740, in _spin_once_impl
[imu_parser_node-10]     handler, entity, node = self.wait_for_ready_callbacks(timeout_sec=timeout_sec)
[imu_parser_node-10]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 723, in wait_for_ready_callbacks
[imu_parser_node-10]     return next(self._cb_iter)
[imu_parser_node-10]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 620, in _wait_for_ready_callbacks
[imu_parser_node-10]     wait_set.wait(timeout_nsec)
[imu_parser_node-10] KeyboardInterrupt
[ldlidar_sl_ros2_node-1] [INFO] [1743493237.001633978] [ldlidar_publisher_ld14]: this node of ldlidar_published is end
[INFO] [static_transform_publisher-5]: process has finished cleanly [pid 34220]
[INFO] [static_transform_publisher-2]: process has finished cleanly [pid 34214]
[INFO] [robot_state_publisher-3]: process has finished cleanly [pid 34216]
[ERROR] [imu_parser_node-10]: process has died [pid 34238, exit code -2, cmd '/home/jdamr/agv_ws/install/slam_control/lib/slam_control/imu_parser_node --ros-args -r __node:=imu_parser_node'].
[INFO] [cartographer_occupancy_grid_node-9]: process has finished cleanly [pid 34229]
[ldlidar_sl_ros2_node-1] [LDS][INFO][1743493186.243987171][Actual BaudRate reported:115200]
[INFO] [static_transform_publisher-4]: process has finished cleanly [pid 34218]
[INFO] [ldlidar_sl_ros2_node-1]: process has finished cleanly [pid 34212]
[INFO] [robot_state_publisher-6]: process has finished cleanly [pid 34222]
