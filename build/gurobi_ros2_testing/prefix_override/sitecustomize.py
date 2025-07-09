import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/monkorusan/ros2_ws/src/gurobi_ros2_testing/install/gurobi_ros2_testing'
