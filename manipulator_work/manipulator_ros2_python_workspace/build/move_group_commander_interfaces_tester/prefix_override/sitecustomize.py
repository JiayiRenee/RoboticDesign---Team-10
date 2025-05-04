import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/mscrobotics2425laptop27/Documents/University/robotic_systems_design_project/RoboticDesign---Team-10/manipulator_work/manipulator_ros2_python_workspace/install/move_group_commander_interfaces_tester'
