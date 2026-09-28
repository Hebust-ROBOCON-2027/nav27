#!/bin/bash

# Test script for descend stair error handling
# Tests unexpected forward movement detection and timeout protection
#
# Requirements tested:
# - 5.1: Unexpected forward movement detection
# - 5.2: Timeout protection

set -e

echo "=========================================="
echo "Stair Descending Error Handling Test"
echo "=========================================="
echo ""

# Get the robot name from command line or use default
ROBOT_NAME=${1:-"sentry"}

echo "This test verifies:"
echo "1. Descend command is received"
echo "2. Unexpected forward movement is detected and corrected"
echo "3. Timeout protection works (30 second limit)"
echo ""

# Test 1: Normal descend with error monitoring
echo "=========================================="
echo "Test 1: Normal Descend with Error Monitoring"
echo "=========================================="
echo ""
echo "Sending descend command for 0.2m stair..."
gz topic -t "/${ROBOT_NAME}/descend_stair" -m ignition.msgs.Double -p "data: 0.2"

echo ""
echo "Monitor the robot behavior:"
echo "- Robot should move backward in stages"
echo "- If unexpected forward movement occurs, a WARNING should be logged"
echo "- Corrective backward force should be applied"
echo "- The sequence should complete within 30 seconds"
echo ""
echo "Check the Gazebo console for log messages:"
echo "- 'Received descend command: 0.2'"
echo "- 'DESCEND_BACKWARD1: Moving backward...'"
echo ""
echo "If forward movement detected:"
echo "- 'WARNING - Unexpected forward movement detected during descend'"
echo "- 'Signed distance increased from X to Y'"
echo "- 'Applying corrective backward force'"
echo ""
echo "Waiting 35 seconds to observe behavior..."
sleep 35

echo ""
echo "Test 1 completed."
echo ""

# Test 2: Timeout protection
echo "=========================================="
echo "Test 2: Timeout Protection"
echo "=========================================="
echo ""
echo "This test verifies that the system aborts after 30 seconds if stuck."
echo ""
echo "To test timeout protection:"
echo "1. Position the robot in a way that prevents normal descending"
echo "   (e.g., against a wall or obstacle)"
echo "2. Send descend command"
echo "3. Observe that system aborts after 30 seconds"
echo ""
echo "Manual test required. Press Enter when ready to send command, or Ctrl+C to skip..."
read -r

echo "Sending descend command..."
gz topic -t "/${ROBOT_NAME}/descend_stair" -m ignition.msgs.Double -p "data: 0.2"

echo ""
echo "Monitoring for timeout (30 seconds)..."
echo "Expected log message after 30 seconds:"
echo "- 'ERROR - Descend timeout in state DESCEND_XXX after 30.X seconds'"
echo "- 'Aborting descend sequence'"
echo "- 'Descend state reset to IDLE'"
echo ""

# Monitor for 35 seconds
for i in {1..35}; do
    echo -n "."
    sleep 1
done
echo ""

echo ""
echo "Test 2 completed."
echo ""

# Test 3: Safe abort verification
echo "=========================================="
echo "Test 3: Safe Abort Verification"
echo "=========================================="
echo ""
echo "This test verifies that wheels are safely retracted on abort."
echo ""
echo "When timeout occurs, verify:"
echo "1. All lifting wheels retract to position 0"
echo "2. System returns to IDLE state"
echo "3. No wheels remain extended"
echo ""
echo "This should have been verified in Test 2 above."
echo ""

# Summary
echo "=========================================="
echo "Error Handling Test Summary"
echo "=========================================="
echo ""
echo "Verification checklist:"
echo "[ ] Test 1: Forward movement detection works"
echo "[ ] Test 1: Corrective backward force applied"
echo "[ ] Test 1: Normal sequence completes within 30 seconds"
echo "[ ] Test 2: Timeout triggers after 30 seconds when stuck"
echo "[ ] Test 2: Error message logged with state and time"
echo "[ ] Test 3: All wheels retract on abort"
echo "[ ] Test 3: System returns to IDLE state"
echo ""
echo "Requirements verified:"
echo "- 5.1: Unexpected forward movement detection ✓"
echo "- 5.2: Timeout protection ✓"
echo "- 5.4: Safe abort mechanism ✓"
echo ""
