#!/usr/bin/env zsh
# One-shot detect_pick_marker_frame e2e: clean slate, pose publisher, program, goal, events, result.
# Usage: ./run_e2e.sh [--no-mock] [--headless]
#   --no-mock   publish the object poses yourself
#   --headless  run without the GUI, for a machine that has none
set -u
cd "$(dirname "$0")"
WS=$(cd ../../../.. && pwd)
source "$WS/setup-grc.zsh" >/dev/null 2>&1

MOCK_WANTED=1
RUN_ARGS=()
for arg in "$@"; do
    case "$arg" in
        --no-mock)  MOCK_WANTED=0 ;;
        --headless) RUN_ARGS+=(--headless) ;;
        *) echo "unknown option: $arg" >&2; exit 2 ;;
    esac
done

# Kill every leftover piece of a previous attempt, by exact process identity.
for pid in $(pgrep -x main); do
    grep -q detect_pick_marker_frame "/proc/$pid/cmdline" 2>/dev/null && kill -9 "$pid"
done
pkill -9 -f "[m]ock_object_pose_publisher" 2>/dev/null
pkill -9 -f "[b]dd_client.py" 2>/dev/null
sleep 1

if [ "$MOCK_WANTED" -eq 1 ]; then
    ros2 run aruco_perception mock_object_pose_publisher \
        --ros-args -p poses_file:="$(pwd)/mock_poses.yml" -p rate_hz:=10.0 &
    MOCK=$!
fi

# Generated beside the model it came from. Each `gen` mints its own timestamped tree, so the
# newest is reused and only built once -- delete generation/ to pick up a changed model.
GEN=$(ls -d generation/detect_pick_marker_frame/*/ 2>/dev/null | tail -1)
if [ -z "$GEN" ]; then
    motion-spec gen detect_pick_marker_frame.robmot -o generation || exit 1
    GEN=$(ls -d generation/detect_pick_marker_frame/*/ | tail -1)
fi
motion-spec build "$GEN" || exit 1
motion-spec run "$GEN" --cwd "$GRC_WS" "${RUN_ARGS[@]}" &
RUN=$!

# The client waits for the action server itself, streams /bdd/events, exits on the result.
python3 bdd_client.py
STATUS=$?

wait "$RUN" 2>/dev/null
[ -n "${MOCK:-}" ] && kill "$MOCK" 2>/dev/null
exit $STATUS
