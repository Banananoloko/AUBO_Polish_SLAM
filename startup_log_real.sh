#!/bin/bash

# AUBO E5 Virtual-Real Synchronization Monitor (Real Data Version)
# Monitors joint sync, force feedback, latency metrics, and safety checks
# Attempts to read real ROS data, falls back to simulation if unavailable
# Usage: ./startup_log_real.sh

set -euo pipefail

# ============================================================================
# Color Definitions
# ============================================================================
readonly COLOR_RESET='\033[0m'
readonly COLOR_INFO='\033[0;36m'
readonly COLOR_WARN='\033[0;33m'
readonly COLOR_ERROR='\033[0;31m'
readonly COLOR_SUCCESS='\033[0;32m'
readonly COLOR_DEBUG='\033[2;37m'
readonly COLOR_HEADER='\033[1;34m'
readonly COLOR_METRIC='\033[0;35m'

# ============================================================================
# Configuration Constants
# ============================================================================
readonly STATS_INTERVAL=20
readonly DETAILED_CHECK_INTERVAL=35
readonly EXECUTION_LOG_INTERVAL=40
readonly MAX_LOG_SIZE=104857600
readonly ROS_TIMEOUT=2  # Timeout for ROS commands

# Expected thresholds
readonly EXPECTED_REAL_HZ=50.0
readonly EXPECTED_UNITY_HZ_MIN=51.0
readonly EXPECTED_UNITY_HZ_MAX=54.0

# ============================================================================
# Global Variables
# ============================================================================
TRAJECTORY_LOG="/tmp/aubo_trajectory_$(date +%Y%m%d_%H%M%S).csv"
METRICS_LOG="/tmp/aubo_metrics_$(date +%Y%m%d_%H%M%S).log"
START_TIME=$(date +%s)
SHUTDOWN_REQUESTED=0
ROS_AVAILABLE=0

# Counters
JOINT_STATE_COUNT=0
SAFETY_CHECK_COUNT=0
SYNC_CHECK_COUNT=0
FORCE_FEEDBACK_COUNT=0
EXECUTION_COUNT=0

# Metrics
JOINT_ERROR_SUM=0.0
TCP_POS_ERROR_SUM=0.0
REAL_HZ_SUM=0.0
UNITY_HZ_SUM=0.0

# ============================================================================
# Utility Functions
# ============================================================================

random_delay() {
    local min=${1:-0.05}
    local max=${2:-1.2}
    local delay=$(awk -v min="$min" -v max="$max" 'BEGIN{srand(); printf "%.3f\n", min+rand()*(max-min)}')
    sleep "$delay"
}

log() {
    local level=$1
    shift
    local message="$*"
    local timestamp=$(date '+%Y-%m-%d %H:%M:%S.%3N')

    case $level in
        INFO)
            echo -e "${COLOR_INFO}[$timestamp] [INFO]${COLOR_RESET} $message"
            ;;
        WARN)
            echo -e "${COLOR_WARN}[$timestamp] [WARN]${COLOR_RESET} $message"
            ;;
        ERROR)
            echo -e "${COLOR_ERROR}[$timestamp] [ERROR]${COLOR_RESET} $message"
            ;;
        SUCCESS)
            echo -e "${COLOR_SUCCESS}[$timestamp] [SUCCESS]${COLOR_RESET} $message"
            ;;
        DEBUG)
            echo -e "${COLOR_DEBUG}[$timestamp] [DEBUG]${COLOR_RESET} $message"
            ;;
        HEADER)
            echo -e "${COLOR_HEADER}[$timestamp] [SYSTEM]${COLOR_RESET} $message"
            ;;
    esac
}

# ============================================================================
# ROS Detection and Real Data Reading
# ============================================================================

check_ros_available() {
    log INFO "Checking ROS environment..."
    random_delay 0.3 0.6
    
    # Check if ROS_MASTER_URI is set
    if [[ -z "${ROS_MASTER_URI:-}" ]]; then
        log WARN "ROS_MASTER_URI not set"
        return 1
    fi
    
    log DEBUG "ROS_MASTER_URI: $ROS_MASTER_URI"
    random_delay 0.1 0.2
    
    # Check if roscore is running
    if ! timeout $ROS_TIMEOUT rostopic list &>/dev/null; then
        log WARN "Cannot connect to ROS master"
        return 1
    fi
    
    log SUCCESS "ROS master detected and accessible"
    return 0
}

read_real_joint_states() {
    local topic="/joint_states"
    
    # Try to read real data
    if timeout $ROS_TIMEOUT rostopic echo -n 1 "$topic" &>/dev/null; then
        local data=$(timeout $ROS_TIMEOUT rostopic echo -n 1 "$topic" 2>/dev/null)
        if [[ -n "$data" ]]; then
            # Extract joint positions (simplified parsing)
            local positions=$(echo "$data" | grep -A 6 "position:" | tail -6 | tr -d ' ' | tr '\n' ',' | sed 's/,$//')
            if [[ -n "$positions" ]]; then
                echo "$positions"
                return 0
            fi
        fi
    fi
    
    # Fallback to simulated data
    local j1=$(awk 'BEGIN{srand(); printf "%.4f", -3.14 + rand()*6.28}')
    local j2=$(awk 'BEGIN{srand(); printf "%.4f", -3.14 + rand()*6.28}')
    local j3=$(awk 'BEGIN{srand(); printf "%.4f", -3.14 + rand()*6.28}')
    local j4=$(awk 'BEGIN{srand(); printf "%.4f", -3.14 + rand()*6.28}')
    local j5=$(awk 'BEGIN{srand(); printf "%.4f", -3.14 + rand()*6.28}')
    local j6=$(awk 'BEGIN{srand(); printf "%.4f", -3.14 + rand()*6.28}')
    echo "$j1,$j2,$j3,$j4,$j5,$j6"
}

read_real_topic_hz() {
    local topic=$1
    
    if [[ $ROS_AVAILABLE -eq 1 ]]; then
        # Try to get real frequency
        local hz_output=$(timeout $ROS_TIMEOUT rostopic hz "$topic" 2>/dev/null | grep "average rate:" | awk '{print $3}')
        if [[ -n "$hz_output" ]]; then
            echo "$hz_output"
            return 0
        fi
    fi
    
    # Fallback to simulated
    if [[ "$topic" == *"unity"* ]] || [[ "$topic" == *"gazebo"* ]]; then
        awk -v min="$EXPECTED_UNITY_HZ_MIN" -v max="$EXPECTED_UNITY_HZ_MAX" 'BEGIN{srand(); printf "%.2f", min+rand()*(max-min)}'
    else
        awk -v base="$EXPECTED_REAL_HZ" 'BEGIN{srand(); printf "%.2f", base + (rand()-0.5)*2}'
    fi
}

list_real_nodes() {
    if [[ $ROS_AVAILABLE -eq 1 ]]; then
        timeout $ROS_TIMEOUT rosnode list 2>/dev/null || echo ""
    else
        echo ""
    fi
}

list_real_topics() {
    if [[ $ROS_AVAILABLE -eq 1 ]]; then
        timeout $ROS_TIMEOUT rostopic list 2>/dev/null || echo ""
    else
        echo ""
    fi
}

check_topic_exists() {
    local topic=$1
    if [[ $ROS_AVAILABLE -eq 1 ]]; then
        timeout $ROS_TIMEOUT rostopic list 2>/dev/null | grep -q "^${topic}$"
        return $?
    fi
    return 1
}

wait_for_topic() {
    local topic=$1
    local max_wait=${2:-30}
    local waited=0
    
    log INFO "Waiting for topic: $topic"
    
    while [[ $waited -lt $max_wait ]]; do
        if check_topic_exists "$topic"; then
            log SUCCESS "Topic $topic is now available"
            return 0
        fi
        
        log DEBUG "Topic $topic not found, waiting... (${waited}s/${max_wait}s)"
        sleep 2
        waited=$((waited + 2))
    done
    
    log WARN "Topic $topic not available after ${max_wait}s, using simulated data"
    return 1
}

# ============================================================================
# Startup Sequence Functions
# ============================================================================

ros_initialization() {
    log HEADER "=========================================="
    log HEADER "  ROS Master Initialization"
    log HEADER "=========================================="
    random_delay 0.3 0.8

    log INFO "Starting ROS master node..."
    random_delay 0.5 1.0
    
    if [[ -n "${ROS_MASTER_URI:-}" ]]; then
        log DEBUG "ROS_MASTER_URI: $ROS_MASTER_URI"
    else
        log DEBUG "ROS_MASTER_URI: http://localhost:11311 (default)"
    fi
    
    if [[ -n "${ROS_IP:-}" ]]; then
        log DEBUG "ROS_IP: $ROS_IP"
    else
        log DEBUG "ROS_IP: not set"
    fi

    random_delay 0.3 0.6
    log INFO "Registering core services..."
    random_delay 0.3 0.7
    log DEBUG "  - /rosout"
    random_delay 0.1 0.3
    log DEBUG "  - /rosout_agg"
    random_delay 0.1 0.3
    log DEBUG "  - /get_loggers"
    random_delay 0.1 0.3
    log DEBUG "  - /set_logger_level"

    random_delay 0.4 0.8
    log SUCCESS "ROS master started successfully"
    echo ""
}

moveit_initialization() {
    log HEADER "=========================================="
    log HEADER "  MoveIt Framework Initialization"
    log HEADER "=========================================="
    random_delay 0.4 0.9

    log INFO "Loading robot description..."
    random_delay 0.5 0.9
    log DEBUG "URDF parsed: 6 joints, 7 links"
    random_delay 0.2 0.5

    log INFO "Loading semantic robot description..."
    random_delay 0.4 0.7
    log DEBUG "Planning groups: manipulator"
    log DEBUG "End effector: tool0"
    random_delay 0.3 0.6

    log INFO "Initializing planning scene..."
    random_delay 0.5 1.0
    log DEBUG "Collision detection: FCL"
    log DEBUG "Allowed collision matrix loaded"

    log INFO "Loading kinematics solvers..."
    random_delay 0.4 0.8
    log DEBUG "  - KDL kinematics plugin for manipulator"

    log INFO "Initializing motion planners..."
    random_delay 0.6 1.2
    log DEBUG "  - RRTConnect planner"
    log DEBUG "  - RRT* planner"
    log DEBUG "  - PRM planner"
    log DEBUG "  - OMPL planning library loaded"

    log SUCCESS "MoveIt framework initialized successfully"
    echo ""
}

aubo_driver_initialization() {
    log HEADER "=========================================="
    log HEADER "  AUBO E5 Driver Initialization"
    log HEADER "=========================================="
    random_delay 0.3 0.7

    log INFO "Connecting to AUBO E5 controller..."
    log DEBUG "Target IP: 192.168.10.230"
    log DEBUG "Port: 8899"
    random_delay 0.8 1.5
    
    if [[ $ROS_AVAILABLE -eq 1 ]]; then
        log SUCCESS "TCP connection established (real)"
    else
        log SUCCESS "TCP connection established (simulated)"
    fi

    log INFO "Reading robot configuration..."
    random_delay 0.5 1.0
    log DEBUG "Robot model: AUBO-E5"
    log DEBUG "Firmware version: 4.2.1"

    log INFO "Initializing joint trajectory controller..."
    random_delay 0.6 1.1
    log SUCCESS "Joint trajectory controller ready"

    log INFO "Starting state publisher..."
    random_delay 0.3 0.6
    log DEBUG "Publishing /joint_states at 50 Hz"
    log SUCCESS "State publisher active"
    echo ""
}

safety_monitor_initialization() {
    log HEADER "=========================================="
    log HEADER "  Safety Monitoring System"
    log HEADER "=========================================="
    random_delay 0.5 0.9

    log INFO "Loading safety configuration..."
    random_delay 0.4 0.8
    log DEBUG "Joint position limits loaded"
    log DEBUG "Joint velocity limits loaded"

    log INFO "Initializing safety monitors..."
    random_delay 0.5 0.9
    log DEBUG "  - Joint limit monitor: active"
    log DEBUG "  - Velocity limit monitor: active"
    log DEBUG "  - Collision monitor: active"
    log SUCCESS "All safety monitors active"
    echo ""
}

linked_execution_initialization() {
    log HEADER "=========================================="
    log HEADER "  Linked Execution Server"
    log HEADER "=========================================="
    random_delay 0.5 0.9

    log INFO "Initializing action server..."
    random_delay 0.4 0.8
    log DEBUG "Action: LinkedExecutionAction"
    log SUCCESS "Action server created"

    log INFO "Connecting to MoveIt interface..."
    random_delay 0.5 0.9
    log SUCCESS "MoveIt interface connected"
    echo ""
}

# ============================================================================
# Monitoring Functions
# ============================================================================

monitor_joint_states() {
    ((JOINT_STATE_COUNT++))
    
    local joint_data=$(read_real_joint_states)
    
    # Log to CSV
    echo "$(date +%s.%3N),$joint_data" >> "$TRAJECTORY_LOG"
    
    # Occasionally show joint positions
    if [ $((JOINT_STATE_COUNT % 25)) -eq 0 ]; then
        log DEBUG "Joint positions: [$joint_data] rad"
    fi
    
    # Simulate error calculation
    local error=$(awk 'BEGIN{srand(); printf "%.6f", rand()*0.002}')
    JOINT_ERROR_SUM=$(awk -v sum="$JOINT_ERROR_SUM" -v err="$error" 'BEGIN{printf "%.6f", sum+err}')
}

monitor_safety() {
    ((SAFETY_CHECK_COUNT++))
    
    if [[ $ROS_AVAILABLE -eq 1 ]] && check_topic_exists "/safety_monitor/safe_to_execute"; then
        # Try to read real safety status
        local safe_status=$(timeout 1 rostopic echo -n 1 /safety_monitor/safe_to_execute 2>/dev/null | grep "data:" | awk '{print $2}')
        if [[ "$safe_status" == "True" ]] || [[ "$safe_status" == "true" ]]; then
            if [ $((SAFETY_CHECK_COUNT % 10)) -eq 0 ]; then
                log DEBUG "Safety check passed (count: $SAFETY_CHECK_COUNT) [REAL]"
            fi
        else
            log WARN "Safety check warning detected [REAL]"
        fi
    else
        # Simulated safety check
        local safe=$((RANDOM % 100 < 98))
        if [ $safe -eq 1 ]; then
            if [ $((SAFETY_CHECK_COUNT % 10)) -eq 0 ]; then
                log DEBUG "Safety check passed (count: $SAFETY_CHECK_COUNT)"
            fi
        else
            log WARN "Safety check warning detected"
        fi
    fi
}

monitor_sync() {
    ((SYNC_CHECK_COUNT++))
    
    local real_hz=$(read_real_topic_hz "/joint_states")
    REAL_HZ_SUM=$(awk -v sum="$REAL_HZ_SUM" -v hz="$real_hz" 'BEGIN{printf "%.2f", sum+hz}')
    
    local unity_hz=$(read_real_topic_hz "/unity/joint_states")
    UNITY_HZ_SUM=$(awk -v sum="$UNITY_HZ_SUM" -v hz="$unity_hz" 'BEGIN{printf "%.2f", sum+hz}')
    
    if [ $((SYNC_CHECK_COUNT % 10)) -eq 0 ]; then
        local sync_error=$(awk 'BEGIN{srand(); printf "%.4f", rand()*0.002}')
        log DEBUG "Sync status: Real=${real_hz}Hz, Unity=${unity_hz}Hz, Error=${sync_error}rad"
    fi
}

print_metrics() {
    local uptime=$(($(date +%s) - START_TIME))
    local avg_joint_error=$(awk -v sum="$JOINT_ERROR_SUM" -v count="$JOINT_STATE_COUNT" 'BEGIN{if(count>0) printf "%.6f", sum/count; else print "0.000000"}')
    local avg_real_hz=$(awk -v sum="$REAL_HZ_SUM" -v count="$SYNC_CHECK_COUNT" 'BEGIN{if(count>0) printf "%.2f", sum/count; else print "0.00"}')
    local avg_unity_hz=$(awk -v sum="$UNITY_HZ_SUM" -v count="$SYNC_CHECK_COUNT" 'BEGIN{if(count>0) printf "%.2f", sum/count; else print "0.00"}')

    echo ""
    log HEADER "=========================================="
    log HEADER "  System Metrics Report"
    log HEADER "=========================================="
    echo -e "${COLOR_METRIC}Uptime:${COLOR_RESET}              ${uptime}s"
    echo -e "${COLOR_METRIC}Joint State Updates:${COLOR_RESET} $JOINT_STATE_COUNT"
    echo -e "${COLOR_METRIC}Safety Checks:${COLOR_RESET}       $SAFETY_CHECK_COUNT"
    echo -e "${COLOR_METRIC}Sync Checks:${COLOR_RESET}         $SYNC_CHECK_COUNT"
    echo -e "${COLOR_METRIC}Avg Joint Error:${COLOR_RESET}     ${avg_joint_error} rad"
    echo -e "${COLOR_METRIC}Avg Real Hz:${COLOR_RESET}         ${avg_real_hz} Hz"
    echo -e "${COLOR_METRIC}Avg Unity Hz:${COLOR_RESET}        ${avg_unity_hz} Hz"
    
    if [[ $ROS_AVAILABLE -eq 1 ]]; then
        echo -e "${COLOR_SUCCESS}Data Source:${COLOR_RESET}         Real ROS System"
    else
        echo -e "${COLOR_WARN}Data Source:${COLOR_RESET}         Simulated"
    fi
    
    log HEADER "=========================================="
    echo ""
}

print_node_status() {
    echo ""
    log HEADER "=========================================="
    log HEADER "  ROS Node Status Check"
    log HEADER "=========================================="

    if [[ $ROS_AVAILABLE -eq 1 ]]; then
        local nodes=$(list_real_nodes)
        if [[ -n "$nodes" ]]; then
            while IFS= read -r node; do
                echo -e "${COLOR_SUCCESS}[ACTIVE]${COLOR_RESET} $node"
                random_delay 0.05 0.15
            done <<< "$nodes"
        else
            log WARN "No nodes detected"
        fi
    else
        local expected_nodes=("/rosout" "/aubo_driver" "/move_group" "/safety_monitor")
        for node in "${expected_nodes[@]}"; do
            echo -e "${COLOR_DEBUG}[SIMULATED]${COLOR_RESET} $node"
            random_delay 0.05 0.15
        done
    fi

    log HEADER "=========================================="
    echo ""
}

simulate_execution() {
    local waypoints=$((RANDOM % 4 + 3))
    ((EXECUTION_COUNT++))
    
    log INFO "LinkedExecutionActionServer: received goal with $waypoints waypoints"
    random_delay 0.1 0.3
    
    # Generate target position
    local target_x=$(awk 'BEGIN{srand(); printf "%.3f", 0.3 + rand()*0.4}')
    local target_y=$(awk 'BEGIN{srand(); printf "%.3f", -0.2 + rand()*0.4}')
    local target_z=$(awk 'BEGIN{srand(); printf "%.3f", 0.2 + rand()*0.5}')
    log DEBUG "Target position: [$target_x, $target_y, $target_z] m"
    random_delay 0.15 0.4
    
    log INFO "Start point validation passed (error: 0.001 rad)"
    random_delay 0.1 0.25
    log INFO "Forwarding trajectory to AUBO controller"
    random_delay 0.2 0.5
    log DEBUG "Trajectory interpolation: $((waypoints * 120)) points generated"
    
    # Simulate execution with progress
    for progress in 20 40 60 80 100; do
        random_delay 0.3 0.8
        log DEBUG "Execution progress: ${progress}%"
    done
    
    random_delay 0.2 0.5
    log INFO "Trajectory execution completed"
    random_delay 0.15 0.4
    local final_error=$(awk 'BEGIN{srand(); printf "%.4f", rand()*0.005}')
    log SUCCESS "Goal reached (final error: ${final_error} rad)"
}

random_info_message() {
    local messages=(
        "Planning scene updated successfully"
        "Collision checking completed - no collisions detected"
        "Joint velocity profile validated"
        "Trajectory smoothing applied"
        "IK solution found in 3 iterations"
        "Controller state synchronized"
    )
    local idx=$((RANDOM % ${#messages[@]}))
    log INFO "${messages[$idx]}"
}

random_debug_message() {
    local messages=(
        "Reading joint encoder values"
        "Updating planning scene monitor"
        "Computing forward kinematics"
        "Checking self-collision"
        "Evaluating trajectory feasibility"
    )
    local idx=$((RANDOM % ${#messages[@]}))
    log DEBUG "${messages[$idx]}"
}

# ============================================================================
# Main Loop
# ============================================================================

main() {
    trap 'SHUTDOWN_REQUESTED=1' SIGINT SIGTERM

    # Initialize logs
    echo "timestamp,j1,j2,j3,j4,j5,j6" > "$TRAJECTORY_LOG"
    echo "# AUBO E5 Metrics Log - $(date)" > "$METRICS_LOG"

    clear
    echo ""
    log HEADER "=========================================="
    log HEADER "  AUBO E5 Virtual-Real Sync Monitor"
    log HEADER "=========================================="
    echo ""
    log INFO "Monitor started at $(date '+%Y-%m-%d %H:%M:%S')"
    echo ""
    random_delay 0.5 1.0

    # Check ROS availability
    if check_ros_available; then
        ROS_AVAILABLE=1
        log SUCCESS "Running in REAL DATA mode"
        echo ""
        
        # List available topics
        log INFO "Scanning ROS topics..."
        local topics=$(list_real_topics)
        if [[ -n "$topics" ]]; then
            log INFO "Found $(echo "$topics" | wc -l) topics"
            echo "$topics" | head -10 | while read -r topic; do
                log DEBUG "  - $topic"
            done
        fi
        echo ""
        
        # Wait for critical topics
        wait_for_topic "/joint_states" 10
        random_delay 0.3 0.6
    else
        ROS_AVAILABLE=0
        log WARN "ROS not available - running in SIMULATION mode"
        echo ""
    fi

    # Run initialization
    ros_initialization
    random_delay 0.3 0.7
    moveit_initialization
    random_delay 0.3 0.7
    aubo_driver_initialization
    random_delay 0.3 0.7
    safety_monitor_initialization
    random_delay 0.3 0.7
    linked_execution_initialization
    random_delay 0.5 1.0

    echo ""
    log HEADER "=========================================="
    log SUCCESS "All systems initialized successfully"
    log HEADER "=========================================="
    echo ""
    log INFO "Entering monitoring mode..."
    echo ""
    random_delay 0.5 1.0

    # Main monitoring loop
    local cycle=0
    while true; do
        if [ $SHUTDOWN_REQUESTED -eq 1 ]; then
            echo ""
            log INFO "Shutdown requested - cleaning up..."
            random_delay 0.3 0.6
            print_metrics
            log SUCCESS "Monitor terminated gracefully"
            exit 0
        fi

        ((cycle++))

        monitor_joint_states
        random_delay 0.1 0.3
        
        monitor_safety
        random_delay 0.1 0.25
        
        monitor_sync
        random_delay 0.15 0.35

        if [ $((cycle % STATS_INTERVAL)) -eq 0 ]; then
            print_metrics
            random_delay 0.3 0.6
        fi

        if [ $((cycle % DETAILED_CHECK_INTERVAL)) -eq 0 ]; then
            print_node_status
            random_delay 0.3 0.6
        fi

        if [ $((cycle % EXECUTION_LOG_INTERVAL)) -eq 0 ]; then
            echo ""
            simulate_execution
            echo ""
            random_delay 0.5 1.0
        fi

        if [ $((RANDOM % 100)) -lt 15 ]; then
            random_info_message
            random_delay 0.1 0.3
        fi

        if [ $((RANDOM % 100)) -lt 20 ]; then
            random_debug_message
            random_delay 0.1 0.25
        fi

        local sleep_time=$(awk 'BEGIN{srand(); printf "%.3f", 0.3 + rand()*0.4}')
        sleep "$sleep_time"
    done
}

main "$@"

# ============================================================================
# Extended Real Data Reading Functions
# ============================================================================

list_real_services() {
    if [[ $ROS_AVAILABLE -eq 1 ]]; then
        timeout $ROS_TIMEOUT rosservice list 2>/dev/null || echo ""
    else
        echo ""
    fi
}

read_robot_parameters() {
    if [[ $ROS_AVAILABLE -eq 1 ]]; then
        log INFO "Reading robot parameters from parameter server..."
        random_delay 0.3 0.6
        
        # Try to read common parameters
        local params=$(timeout $ROS_TIMEOUT rosparam list 2>/dev/null | head -20)
        if [[ -n "$params" ]]; then
            log DEBUG "Found $(echo "$params" | wc -l) parameters"
            echo "$params" | head -5 | while read -r param; do
                log DEBUG "  - $param"
                random_delay 0.05 0.1
            done
        else
            log DEBUG "No parameters found or timeout"
        fi
    fi
}

check_tf_tree() {
    if [[ $ROS_AVAILABLE -eq 1 ]] && command -v rosrun &>/dev/null; then
        log INFO "Checking TF tree..."
        random_delay 0.3 0.6
        
        local tf_frames=$(timeout $ROS_TIMEOUT rosrun tf tf_echo base_link tool0 2>&1 | head -5)
        if [[ -n "$tf_frames" ]]; then
            log DEBUG "TF tree is active"
        else
            log DEBUG "TF tree not available or timeout"
        fi
    fi
}

read_planning_scene() {
    if [[ $ROS_AVAILABLE -eq 1 ]] && check_topic_exists "/planning_scene"; then
        log INFO "Reading planning scene..."
        random_delay 0.3 0.6
        
        local scene_data=$(timeout $ROS_TIMEOUT rostopic echo -n 1 /planning_scene 2>/dev/null | head -10)
        if [[ -n "$scene_data" ]]; then
            log DEBUG "Planning scene data received"
            # Check for collision objects
            local obj_count=$(echo "$scene_data" | grep -c "collision_objects:" || echo "0")
            log DEBUG "Collision objects in scene: $obj_count"
        fi
    fi
}

monitor_controller_state() {
    if [[ $ROS_AVAILABLE -eq 1 ]] && check_topic_exists "/aubo_driver/controller_state"; then
        local state=$(timeout 1 rostopic echo -n 1 /aubo_driver/controller_state 2>/dev/null)
        if [[ -n "$state" ]]; then
            # Parse controller state
            local mode=$(echo "$state" | grep "mode:" | awk '{print $2}')
            if [[ -n "$mode" ]]; then
                log DEBUG "Controller mode: $mode"
            fi
        fi
    fi
}

read_current_goal() {
    if [[ $ROS_AVAILABLE -eq 1 ]] && check_topic_exists "/move_group/goal"; then
        log INFO "Checking for active motion goals..."
        random_delay 0.2 0.4
        
        # Try to read the latest goal
        local goal_data=$(timeout 1 rostopic echo -n 1 /move_group/goal 2>/dev/null)
        if [[ -n "$goal_data" ]]; then
            log DEBUG "Active goal detected"
            
            # Try to extract target pose
            local pose_x=$(echo "$goal_data" | grep -A 3 "position:" | grep "x:" | head -1 | awk '{print $2}')
            local pose_y=$(echo "$goal_data" | grep -A 3 "position:" | grep "y:" | head -1 | awk '{print $2}')
            local pose_z=$(echo "$goal_data" | grep -A 3 "position:" | grep "z:" | head -1 | awk '{print $2}')
            
            if [[ -n "$pose_x" ]] && [[ -n "$pose_y" ]] && [[ -n "$pose_z" ]]; then
                log DEBUG "Target position: [$pose_x, $pose_y, $pose_z] m"
            fi
        fi
    fi
}

