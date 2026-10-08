#!/bin/bash
# Start / stop the whole stack: Gazebo workcell, move_group, perception_node.
#   src/vision_perception/scripts/stack.sh start   # waits until everything is ready
#   src/vision_perception/scripts/stack.sh stop
#   src/vision_perception/scripts/stack.sh status
#   src/vision_perception/scripts/stack.sh restart-perception   # only the perception node
# Logs and PID files go to $STACK_DIR (default /tmp/so101_stack).

WS="$(cd "$(dirname "$0")/../../.." && pwd)"
STACK_DIR="${STACK_DIR:-/tmp/so101_stack}"
mkdir -p "$STACK_DIR"

source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"

stop() {
  # Kill by saved PID with TERM (background jobs ignore Ctrl-C). Never use
  # `pkill -f`: it can match this script itself. Then kill any leftovers,
  # because two simulations at once break everything.
  for name in perception move_group sim; do
    f="$STACK_DIR/$name.pid"
    [ -f "$f" ] && kill -TERM "$(cat "$f")" 2>/dev/null
    rm -f "$f"
  done
  for i in $(seq 1 20); do
    ps -eo args | grep -qE "[g]z sim server|[m]ove_group --ros|[v]ision_perception.perception_node" || break
    sleep 1
  done
  for p in $(ps -eo pid,args | awk '/[g]z sim|[i]mage_bridge|[p]arameter_bridge|[r]viz2|[r]obot_state_publisher|[g]round_truth_pose_bridge|[m]ove_group --ros|[v]ision_perception.perception_node|[r]os2 launch so101_/{print $1}'); do
    kill -9 "$p" 2>/dev/null
  done
  echo "stack stopped"
}

wait_for() {  # wait_for <seconds> <description> <command...>
  local t=$1 what=$2; shift 2
  for i in $(seq 1 "$t"); do "$@" >/dev/null 2>&1 && { echo "  $what: ready (${i}s)"; return 0; }; sleep 1; done
  echo "  $what: NOT ready after ${t}s -- see $STACK_DIR/*.log"; return 1
}

start() {
  if ps -eo args | grep -qE "[g]z sim server|[m]ove_group --ros"; then
    echo "something is already running -- run '$0 stop' first"; return 1
  fi
  cd "$WS"
  nohup ros2 launch so101_description workcell_gazebo.launch.py > "$STACK_DIR/sim.log" 2>&1 &
  echo $! > "$STACK_DIR/sim.pid"
  wait_for 120 "gazebo + controllers" bash -c \
    "[ \$(ros2 control list_controllers 2>/dev/null | grep -c active) -ge 5 ]" || return 1
  nohup ros2 launch so101_moveit_config move_group.launch.py > "$STACK_DIR/move_group.log" 2>&1 &
  echo $! > "$STACK_DIR/move_group.pid"
  wait_for 120 "move_group" grep -q "You can start planning now" "$STACK_DIR/move_group.log" || return 1
  start_perception || return 1
  echo "stack up (logs in $STACK_DIR)"
}

start_perception() {
  cd "$WS"
  nohup "$WS/.venv/bin/python" -m vision_perception.perception_node --ros-args -p use_sim_time:=true \
    > "$STACK_DIR/perception.log" 2>&1 &
  echo $! > "$STACK_DIR/perception.pid"
  wait_for 120 "perception_node" grep -q "Ready" "$STACK_DIR/perception.log"
}

restart_perception() {
  # Only the perception node (e.g. after changing perception code and rebuilding).
  f="$STACK_DIR/perception.pid"
  [ -f "$f" ] && kill -TERM "$(cat "$f")" 2>/dev/null
  rm -f "$f"
  sleep 2
  start_perception
}

status() {
  for p in "gz sim server" "move_group --ros" "vision_perception.perception_node"; do
    n=$(ps -eo args | grep -F "$p" | grep -vc grep)
    echo "$p: $n process(es)"
  done
}

case "$1" in
  start) start ;;
  stop) stop ;;
  restart) stop; start ;;
  status) status ;;
  restart-perception) restart_perception ;;
  *) echo "usage: $0 start|stop|restart|status|restart-perception"; exit 2 ;;
esac
