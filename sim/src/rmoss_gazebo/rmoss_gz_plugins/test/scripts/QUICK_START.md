# Quick Start Guide - Stair Descending Tests

## Prerequisites

1. **Build the plugin:**
   ```bash
   cd rcu_ws
   colcon build --packages-select rmoss_gz_plugins
   source install/setup.bash
   ```

2. **Launch Gazebo with robot:**
   ```bash
   # Launch your robot simulation with StairClimber plugin
   # Example:
   gz sim [your_world.sdf]
   ```

## Quick Test (Automated)

Run all automated tests in one go:

```bash
cd rcu_ws/src/rmoss_gazebo/rmoss_gz_plugins/test/scripts

# Run complete flow verification
./verify_descend_sequence.py

# Run error handling verification
./test_error_handling.py

# Run climb-descend verification
./test_climb_descend_automated.py
```

## Full Test Suite (Manual + Automated)

Run the complete test suite with manual observation:

```bash
cd rcu_ws/src/rmoss_gazebo/rmoss_gz_plugins/test/scripts
./run_all_descend_tests.sh
```

This will run all tests in order and prompt you to verify each step.

## Individual Tests

### Test 8.1: Complete Flow
```bash
./test_descend_complete_flow.sh
```
Tests descending with different stair heights (0.1m, 0.15m, 0.2m).

### Test 8.2: Error Handling
```bash
./test_descend_stair.sh
# or automated:
./test_error_handling.py
```
Tests timeout protection and forward movement detection.

### Test 8.3: Climb-Descend Sequence
```bash
./test_climb_descend_sequence.sh
# or automated:
./test_climb_descend_automated.py
```
Tests climb followed by descend to verify state machine switching.

## Expected Results

All tests should show:
- ✓ Commands sent successfully
- ✓ State transitions occur correctly
- ✓ Wheels extend in correct order
- ✓ Sequences complete within 30 seconds
- ✓ No timeout errors (unless testing timeout)
- ✓ Error handling works when triggered

## Troubleshooting

**Problem:** Commands not received
- **Solution:** Check Gazebo is running and robot name is correct (default: "sentry")

**Problem:** Sequence doesn't complete
- **Solution:** Check robot positioning, verify plugin is loaded, check Gazebo console for errors

**Problem:** Wheels don't extend
- **Solution:** Verify joint names match plugin expectations, check joint limits

## Test Results

Document your results:

```
Date: ___________
Robot: ___________
Gazebo Version: ___________

✓/✗ Test 8.1: Complete flow (0.1m, 0.15m, 0.2m)
✓/✗ Test 8.2: Error handling (timeout, forward movement)
✓/✗ Test 8.3: Climb-descend sequence

Notes:
_________________________________
_________________________________
```

## Next Steps

After all tests pass:
- Document results in test log
- Update requirements verification matrix
- Mark task 8 as complete in tasks.md
- Proceed to task 9 (final checkpoint)
