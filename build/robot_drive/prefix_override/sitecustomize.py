import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/monkeydluffy/SLAM-ROS2-Backend/install/robot_drive'
