#!/usr/bin/env bash

# ==========================================================
# Run multiple trajectories sequentially on left or right robot
# Usage:
#   ./run_multiple_trajectories.sh <num_trajectories> <left|right> [base_name]
# Example:
#   ./run_multiple_trajectories.sh 3 left trajectory
# ==========================================================

DEFAULT_BASE_NAME="trajectoireRadMomoRoro"
PACKAGE="mfja_motoman_hc10"            
SCRIPT_NAME="execute_trajectory.py"     

if [ $# -lt 2 ]; then
    echo "Usage: $0 <num_trajectories> <left|right> [base_name]"
    exit 1
fi

NUM_TRAJ=$1
SIDE=$2
BASE_NAME=${3:-$DEFAULT_BASE_NAME}

if [ "$SIDE" == "left" ]; then
    ARM_ARG="left"
    SEARCH_PATTERN="yaskawa_LEFT.*motoros2_single_trajectory_executor"
elif [ "$SIDE" == "right" ]; then
    ARM_ARG="right"
    SEARCH_PATTERN="yaskawa_RIGHT.*motoros2_single_trajectory_executor"
else
    echo "Error: SIDE must be either 'left' or 'right'"
    exit 1
fi

if [ -f ~/colcon_ws/install/setup.bash ]; then
    source ~/colcon_ws/install/setup.bash
fi

for (( i=1; i<=NUM_TRAJ; i++ ))
do
    TRAJ_FILE="${BASE_NAME}${i}.csv"

    echo ""
    echo "==============================================="
    echo " Launching trajectory ($i/$NUM_TRAJ): $TRAJ_FILE"
    echo " Target robot side: $SIDE"
    echo "==============================================="

    # Launch node in foreground directly using wait, OR background + PID wait
    ros2 run $PACKAGE $SCRIPT_NAME "$TRAJ_FILE" --robot $ARM_ARG &
    NODE_PID=$!

    # Wait for node process directly instead of polling ros2 node list
    wait $NODE_PID

    echo "======> ✅ Trajectory $TRAJ_FILE finished !!!"

    if [ $i -lt $NUM_TRAJ ]; then
        echo ""
        read -p "Press [ENTER] to continue to trajectory $((i+1))..."
        echo ""
    fi
done

echo "🎯 All $NUM_TRAJ trajectories completed successfully."