import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/benjaminliu/rover-ui/ros2_ws/install/sc_rover_sim'
