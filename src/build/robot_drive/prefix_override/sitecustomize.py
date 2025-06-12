import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/monkeydluffy/ros2_packages/SLAM-ROS2-Backend/src/install/robot_drive'
