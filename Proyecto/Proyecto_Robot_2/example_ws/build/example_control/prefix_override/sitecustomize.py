import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/robousr/ROS2Dev/Workspaces/Proyecto_Robot_2/example_ws/install/example_control'
