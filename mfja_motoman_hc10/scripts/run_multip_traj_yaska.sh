#!/usr/bin/env bash

# ==========================================================
# Run multiple trajectories sequentially on left or right robot
# Usage:
#   ./run_multiple_trajectories.sh <num_trajectories> <left|right> [base_name]
# Example:
#   ./run_multiple_trajectories.sh 3 left trajectory
# ==========================================================

# === CONFIG ===
DEFAULT_BASE_NAME="trajectoireRadMomoRoro"


# Left robot launch parameters
LEFT_PACKAGE="motoman_hc10_moveit_config"
LEFT_LAUNCH="execute_trajectory.launch"
LEFT_IP="10.100.12.134"
LEFT_EXTRA="left_robot:=true"

# Right robot launch parameters (adjust as needed)
RIGHT_PACKAGE="motoman_hc10_moveit_config"
RIGHT_LAUNCH="execute_trajectory.launch"
RIGHT_IP="10.100.12.133"    # <-- change this IP for right robot
RIGHT_EXTRA="left_robot:=false"



# === INPUT CHECK ===
if [ $# -lt 2 ]; then
    echo "Usage: $0 <num_trajectories> <left|right> [base_name]"
    exit 1
fi

NUM_TRAJ=$1
SIDE=$2
BASE_NAME=${3:-$DEFAULT_BASE_NAME}

# === SELECT LAUNCH PARAMETERS BASED ON SIDE ===
if [ "$SIDE" == "left" ]; then
    PACKAGE=$LEFT_PACKAGE
    LAUNCH=$LEFT_LAUNCH
    IP=$LEFT_IP
    EXTRA_ARGS=$LEFT_EXTRA
    ROS_NODE_NAME="/yaskawa_LEFT/execute_trajectory"    # Change this to match your node name
elif [ "$SIDE" == "right" ]; then
    PACKAGE=$RIGHT_PACKAGE
    LAUNCH=$RIGHT_LAUNCH
    IP=$RIGHT_IP
    EXTRA_ARGS=$RIGHT_EXTRA
    ROS_NODE_NAME="/yaskawa_RIGHT/execute_trajectory"    # Change this to match your node name
else
    echo "Error: SIDE must be either 'left' or 'right'"
    exit 1
fi

# === LOOP OVER TRAJECTORIES ===
for (( i=1; i<=NUM_TRAJ; i++ ))
do

    # === Source catkin === Necessary every time as new terminal is open each loop
    source ~/catkin_ws/devel/setup.bash
    sleep 1

    # === Enable robot === Necessary every time as robot will be deactivated to use gripper manually
    if [ "$SIDE" == "left" ]; then
    	rosservice call /yaskawa_LEFT/robot_enable
    else
        rosservice call /yaskawa_RIGHT/robot_enable
    fi
    sleep 2

    TRAJ_NAME="${BASE_NAME}${i}.csv"

    echo "==============================================="
    echo " Launching trajectory: $TRAJ_NAME"
    echo " Robot side: $SIDE | IP: $IP"
    echo "==============================================="

    # Launch in background
    echo "roslaunch $PACKAGE $LAUNCH $EXTRA_ARGS filename:=TRAJ_NAME"
    roslaunch $PACKAGE $LAUNCH $EXTRA_ARGS "filename:="$TRAJ_NAME
    #LAUNCH_PID=$!

    # # Wait for node to start
    # echo "⏳ Waiting for node $ROS_NODE_NAME to start..."
    # while ! rosnode list | grep -q "$ROS_NODE_NAME"; do
    #     sleep 0.5
    # done
    # echo "✅ Node started: $ROS_NODE_NAME"

    # Wait for node to finish
    # echo "⏳ Waiting for node $ROS_NODE_NAME to finish trajectory $TRAJ_NAME..."
    while rosnode list | grep -q "$ROS_NODE_NAME"; do
        sleep 1
    done
    echo "✅ Trajectory $TRAJ_NAME finished."

    # # Kill the launch if still hanging
    # kill $LAUNCH_PID 2>/dev/null
    

    # Wait for user before next trajectory
    if [ $i -lt $NUM_TRAJ ]; then
        read -p "Press ENTER to launch next trajectory..."
    fi
done

echo "🎯 All $NUM_TRAJ trajectories completed."
