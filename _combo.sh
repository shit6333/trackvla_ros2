set +u
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
set -u

check() {
    SM=$1
    CM=$2
    echo "=================================================="
    echo "smoother=$SM  monitor=$CM"
    ros2 launch vla_tracking tracking.launch.py \
        backend:=fake \
        enable_velocity_smoother:=$SM \
        enable_collision_monitor:=$CM > /tmp/launch.log 2>&1 &
    LP=$!
    sleep 14
    echo "-- nodes --"
    ros2 node list 2>/dev/null | sort | tr '\n' ' '; echo
    echo "-- /cmd_vel --"
    ros2 topic info /cmd_vel 2>/dev/null | grep -E "Publisher count|Subscription count"
    echo "-- routing line --"
    grep -o "executor publishes.*" /tmp/launch.log | head -1
    echo "-- lifecycle states --"
    for n in velocity_smoother collision_monitor; do
        S=$(ros2 lifecycle get /$n 2>/dev/null) && echo "  /$n: $S"
    done
    kill $LP 2>/dev/null
    sleep 5
    pkill -f vla_inference_node; pkill -f trajectory_executor_node
    pkill -f velocity_smoother; pkill -f collision_monitor; pkill -f lifecycle_manager
    sleep 3
}

check false false
check true  false
check false true
check true  true
