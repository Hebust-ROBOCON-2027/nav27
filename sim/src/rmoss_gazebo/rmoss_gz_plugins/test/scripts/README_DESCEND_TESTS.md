# Stair Descending Test Suite

This directory contains test scripts for verifying the stair descending functionality.

## Test Scripts

### 1. test_descend_complete_flow.sh
**Purpose:** Manual interactive test for complete descending flow with different stair heights.

**Requirements Tested:**
- 2.1-2.7: Complete state machine flow
- 6.1: Correct lifting wheel sequence

**Usage:**
```bash
./test_descend_complete_flow.sh [robot_name]
```

**What it tests:**
- Descending stairs of heights: 0.1m, 0.15m, 0.2m
- Correct wheel extension sequence (wheel 4 → wheel 2 → retract all)
- Backward distance control
- State transitions based on position feedback

**Expected Output:**
The script will send descend commands for each stair height and provide instructions for manual verification. Monitor the Gazebo console for state transition messages.

### 2. verify_descend_sequence.py
**Purpose:** Automated verification of descending sequence.

**Requirements Tested:**
- 2.1-2.7: State machine flow
- 6.1: Lifting wheel sequence

**Usage:**
```bash
./verify_descend_sequence.py [robot_name]
```

**What it tests:**
- Command sending and reception
- Sequence completion within timeout
- Wheel extension sequence (requires manual observation)
- Backward movement (requires manual observation)

**Exit Codes:**
- 0: All tests passed
- 1: Some tests failed

### 3. test_descend_stair.sh
**Purpose:** Test error handling functionality.

**Requirements Tested:**
- 5.1: Unexpected forward movement detection
- 5.2: Timeout protection
- 5.4: Safe abort mechanism

**Usage:**
```bash
./test_descend_stair.sh [robot_name]
```

**What it tests:**
- Descend command reception
- Unexpected forward movement detection and correction
- Timeout protection (30 seconds)
- Safe wheel retraction on abort

### 4. test_error_handling.py
**Purpose:** Automated error handling verification.

**Requirements Tested:**
- 5.1: Unexpected forward movement detection
- 5.2: Timeout protection
- 5.4: Safe abort mechanism

**Usage:**
```bash
./test_error_handling.py [robot_name]
```

**What it tests:**
- Timeout protection (30 second limit)
- Forward movement detection and correction
- Safe abort with wheel retraction

**Exit Codes:**
- 0: All tests passed
- 1: Some tests failed

### 5. test_climb_descend_sequence.sh
**Purpose:** Test climb-then-descend sequence.

**Requirements Tested:**
- 6.1: Inverse operation sequence
- 6.4: State machine switching

**Usage:**
```bash
./test_climb_descend_sequence.sh [robot_name] [stair_height]
```

**What it tests:**
- Complete climb sequence
- Complete descend sequence after climb
- State machine switching between operations
- Inverse wheel extension order

### 6. test_climb_descend_automated.py
**Purpose:** Automated climb-descend sequence verification.

**Requirements Tested:**
- 6.1: Inverse operation sequence
- 6.4: State machine switching

**Usage:**
```bash
./test_climb_descend_automated.py [robot_name]
```

**What it tests:**
- Climb success
- Descend success after climb
- State machine switching
- Inverse wheel sequence verification

**Exit Codes:**
- 0: All tests passed
- 1: Some tests failed

## Test World

### stair_descend_test.sdf
A test world containing three stairs of different heights:
- Stair 1: 0.1m (blue)
- Stair 2: 0.15m (green)
- Stair 3: 0.2m (red)

**Usage:**
```bash
gz sim ../worlds/stair_descend_test.sdf
```

## Running the Complete Test Suite

### Prerequisites
1. Build the StairClimber plugin:
```bash
cd rcu_ws
colcon build --packages-select rmoss_gz_plugins
source install/setup.bash
```

2. Launch Gazebo with a robot model that has the StairClimber plugin

### Test Execution Order

1. **Start Gazebo simulation** with your robot model
2. **Run complete flow test:**
   ```bash
   cd rcu_ws/src/rmoss_gazebo/rmoss_gz_plugins/test/scripts
   ./test_descend_complete_flow.sh
   ```
3. **Run automated verification:**
   ```bash
   ./verify_descend_sequence.py
   ```
4. **Run error handling test:**
   ```bash
   ./test_descend_stair.sh
   ```
   Or automated version:
   ```bash
   ./test_error_handling.py
   ```
5. **Run climb-descend sequence test:**
   ```bash
   ./test_climb_descend_sequence.sh
   ```
   Or automated version:
   ```bash
   ./test_climb_descend_automated.py
   ```

## Verification Checklist

After running all tests, verify:

- [ ] Wheel 4 (rear mecanum) extends before wheel 2 in all tests
- [ ] Wheel extensions match stair height:
  - Wheel 4: -(height + 0.02)m
  - Wheel 2: -(height + 0.07)m
- [ ] All wheels retract at the end of sequence
- [ ] Robot moves backward throughout the sequence
- [ ] State transitions occur at correct positions
- [ ] No timeout errors occur
- [ ] Sequence completes within 30 seconds for each height
- [ ] Unexpected forward movement is detected and corrected
- [ ] Timeout protection activates after 30 seconds if stuck

## Troubleshooting

### Test fails to send command
- Verify Gazebo is running
- Check robot name matches (default: "sentry")
- Verify StairClimber plugin is loaded

### Sequence doesn't complete
- Check Gazebo console for error messages
- Verify robot is positioned correctly on stair
- Check for timeout messages (30 second limit)

### Wheels don't extend correctly
- Verify joint names in robot model match plugin expectations
- Check joint limits allow required extension
- Review Gazebo console for joint control messages

## Test Results Documentation

Document test results in the following format:

```
Test Date: YYYY-MM-DD
Robot Model: [model name]
Gazebo Version: [version]

Test 8.1 - Complete Flow:
- 0.1m stair: [PASS/FAIL] - [notes]
- 0.15m stair: [PASS/FAIL] - [notes]
- 0.2m stair: [PASS/FAIL] - [notes]

Test 8.2 - Error Handling:
- Timeout protection: [PASS/FAIL] - [notes]
- Forward movement correction: [PASS/FAIL] - [notes]
- Safe abort mechanism: [PASS/FAIL] - [notes]

Test 8.3 - Climb then Descend:
- Climb success: [PASS/FAIL] - [notes]
- Descend success: [PASS/FAIL] - [notes]
- State machine switching: [PASS/FAIL] - [notes]
- Inverse wheel sequence: [PASS/FAIL] - [notes]
```
