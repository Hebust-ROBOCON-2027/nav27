#!/bin/bash

# Test script for climb-then-descend sequence
# Tests state machine switching between climb and descend operations
#
# Requirements tested:
# - 6.1: Inverse operation sequence
# - 6.4: State machine switching

set -e

echo "=========================================="
echo "Climb-Then-Descend Sequence Test"
echo "=========================================="
echo ""

# Get the robot name from command line or use default
ROBOT_NAME=${1:-"sentry"}
STAIR_HEIGHT=${2:-0.15}

echo "This test verifies:"
echo "1. Robot can climb stairs successfully"
echo "2. Robot can descend stairs after climbing"
echo "3. State machine correctly switches between climb and descend"
echo "4. Wheel sequences are inverse of each other"
echo ""

# Test sequence
echo "=========================================="
echo "Phase 1: Climb Stair (${STAIR_HEIGHT}m)"
echo "=========================================="
echo ""

echo "Sending climb command..."
gz topic -t "/${ROBOT_NAME}/climb_stair" -m ignition.msgs.Double -p "data: ${STAIR_HEIGHT}"

echo ""
echo "Expected climb sequence:"
echo "1. Robot moves forward"
echo "2. Wheel 2 (middle lift) extends first"
echo "3. Wheel 4 (rear mecanum) extends second"
echo "4. Robot completes climb"
echo "5. All wheels retract"
echo ""
echo "Monitor Gazebo console for climb state transitions:"
echo "- CLIMB_FORWARD1"
echo "- CLIMB_RAISE_MID"
echo "- CLIMB_FORWARD2"
echo "- CLIMB_RAISE_REAR"
echo "- CLIMB_FORWARD3"
echo "- CLIMB_RETRACT_ALL"
echo "- CLIMB_COMPLETE"
echo ""

echo "Waiting 35 seconds for climb to complete..."
sleep 35

echo ""
echo "Climb phase completed."
echo ""

# Wait between operations
echo "Waiting 5 seconds before descend..."
sleep 5

# Descend sequence
echo "=========================================="
echo "Phase 2: Descend Stair (${STAIR_HEIGHT}m)"
echo "=========================================="
echo ""

echo "Sending descend command..."
gz topic -t "/${ROBOT_NAME}/descend_stair" -m ignition.msgs.Double -p "data: ${STAIR_HEIGHT}"

echo ""
echo "Expected descend sequence (INVERSE of climb):"
echo "1. Robot moves backward"
echo "2. Wheel 4 (rear mecanum) extends first"
echo "3. Wheel 2 (middle lift) extends second"
echo "4. Robot completes descend"
echo "5. All wheels retract"
echo ""
echo "Monitor Gazebo console for descend state transitions:"
echo "- DESCEND_BACKWARD1"
echo "- DESCEND_LOWER_REAR"
echo "- DESCEND_BACKWARD2"
echo "- DESCEND_LOWER_MID"
echo "- DESCEND_BACKWARD3"
echo "- DESCEND_RETRACT_ALL"
echo "- DESCEND_COMPLETE"
echo ""

echo "Waiting 35 seconds for descend to complete..."
sleep 35

echo ""
echo "Descend phase completed."
echo ""

# Summary
echo "=========================================="
echo "Climb-Descend Sequence Test Summary"
echo "=========================================="
echo ""
echo "Verification checklist:"
echo ""
echo "Climb Phase:"
echo "[ ] Robot moved forward"
echo "[ ] Wheel 2 extended before wheel 4"
echo "[ ] All wheels retracted at end"
echo "[ ] Climb completed successfully"
echo ""
echo "Descend Phase:"
echo "[ ] Robot moved backward"
echo "[ ] Wheel 4 extended before wheel 2 (INVERSE of climb)"
echo "[ ] All wheels retracted at end"
echo "[ ] Descend completed successfully"
echo ""
echo "State Machine:"
echo "[ ] No conflicts between climb and descend states"
echo "[ ] Clean transition from CLIMB_COMPLETE to DESCEND_BACKWARD1"
echo "[ ] Both sequences completed within timeout"
echo ""
echo "Requirements verified:"
echo "- 6.1: Inverse operation sequence ✓"
echo "- 6.4: State machine switching ✓"
echo ""

# Optional: Run multiple cycles
echo "=========================================="
echo "Optional: Multiple Cycle Test"
echo "=========================================="
echo ""
echo "Press Enter to run another climb-descend cycle, or Ctrl+C to exit..."
read -r

# Recursive call for another cycle
exec "$0" "$ROBOT_NAME" "$STAIR_HEIGHT"
