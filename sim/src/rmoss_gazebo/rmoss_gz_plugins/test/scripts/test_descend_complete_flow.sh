#!/bin/bash

# Comprehensive test script for descend stair functionality
# Tests complete descending flow with different stair heights
# 
# Requirements tested:
# - 2.1-2.7: Complete state machine flow
# - 6.1: Correct lifting wheel sequence

set -e

echo "=========================================="
echo "Stair Descending Complete Flow Test"
echo "=========================================="
echo ""

# Get the robot name from command line or use default
ROBOT_NAME=${1:-"sentry"}

# Test different stair heights
STAIR_HEIGHTS=(0.1 0.15 0.2)

echo "This test will verify:"
echo "1. Complete descending flow for different stair heights"
echo "2. Correct lifting wheel extension sequence (wheel 4 -> wheel 2 -> retract all)"
echo "3. Backward distance control accuracy"
echo "4. State transitions based on position feedback"
echo ""

for HEIGHT in "${STAIR_HEIGHTS[@]}"; do
    echo "=========================================="
    echo "Test: Descending ${HEIGHT}m stair"
    echo "=========================================="
    
    echo "Sending descend command for ${HEIGHT}m stair..."
    gz topic -t "/${ROBOT_NAME}/descend_stair" -m ignition.msgs.Double -p "data: ${HEIGHT}"
    
    echo ""
    echo "Expected sequence:"
    echo "1. DESCEND_BACKWARD1: Robot moves backward until rear wheels suspended"
    echo "2. DESCEND_LOWER_REAR: Wheel 4 (rear mecanum) extends to -(${HEIGHT} + 0.02)m"
    echo "3. DESCEND_BACKWARD2: Robot continues backward until front wheel at edge"
    echo "4. DESCEND_LOWER_MID: Wheel 2 (middle lift) extends to -(${HEIGHT} + 0.07)m"
    echo "5. DESCEND_BACKWARD3: Robot completes backward movement"
    echo "6. DESCEND_RETRACT_ALL: All wheels retract to 0"
    echo "7. DESCEND_COMPLETE: Sequence finished"
    echo ""
    
    echo "Monitor Gazebo console for:"
    echo "- 'Received descend command: ${HEIGHT}'"
    echo "- 'DESCEND_BACKWARD1: Moving backward...'"
    echo "- 'Rear wheels suspended, transitioning to DESCEND_LOWER_REAR'"
    echo "- 'DESCEND_LOWER_REAR: Extending wheel 4 to -X.XX'"
    echo "- 'DESCEND_BACKWARD2: Continuing backward...'"
    echo "- 'Front wheel at edge, transitioning to DESCEND_LOWER_MID'"
    echo "- 'DESCEND_LOWER_MID: Extending wheel 2 to -X.XX'"
    echo "- 'DESCEND_BACKWARD3: Final backward movement...'"
    echo "- 'Descend complete, retracting all wheels'"
    echo "- 'DESCEND COMPLETE'"
    echo ""
    
    echo "Waiting 35 seconds for sequence to complete..."
    sleep 35
    
    echo "Test for ${HEIGHT}m stair completed."
    echo ""
    echo "Press Enter to continue to next test, or Ctrl+C to exit..."
    read -r
done

echo "=========================================="
echo "All stair height tests completed!"
echo "=========================================="
echo ""
echo "Verification checklist:"
echo "[ ] Wheel 4 extended before wheel 2 in all tests"
echo "[ ] Wheel extensions matched stair height (wheel 4: -(h+0.02), wheel 2: -(h+0.07))"
echo "[ ] All wheels retracted at the end"
echo "[ ] Robot moved backward throughout the sequence"
echo "[ ] State transitions occurred at correct positions"
echo "[ ] No timeout errors occurred"
echo "[ ] Sequence completed within 30 seconds for each height"
echo ""
