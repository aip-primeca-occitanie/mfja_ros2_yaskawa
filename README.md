runmicrorosdocker


colcon build --mixin release --parallel-workers 4 --packages-select mfja_motoman_hc10 motoman_hc10_moveit_config
source install/setup.bash

ros2 launch motoman_hc10_moveit_config demo_real_right.launch.py 

python3 create_trajectory_from_current.py --robot right --delta 0 0 0 0 1 0 --filename ../trajectories/test_right_1.csv

python3 execute_trajectory.py ../trajectories/test_right_1.csv 
