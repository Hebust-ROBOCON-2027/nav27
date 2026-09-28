#!/bin/bash

# Master test runner for all stair descending tests
# Executes all test scripts in the recommended order

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_NAME=${1:-"sentry"}

echo "=========================================="
echo "Stair Descending - Complete Test Suite"
echo "=========================================="
echo ""
echo "Robot: ${ROBOT_NAME}"
echo "Test Directory: ${SCRIPT_DIR}"
echo ""
echo "This will run all descending tests:"
echo "1. Complete flow test (8.1)"
echo "2. Error handling test (8.2)"
echo "3. Climb-descend sequence test (8.3)"
echo ""
echo "Prerequisites:"
echo "- Gazebo must be running"
echo "- Robot model must be loaded with StairClimber plugin"
echo "- Robot should be positioned near test stairs"
echo ""

read -p "Press Enter to start tests, or Ctrl+C to cancel..."

# Test 8.1: Complete flow
echo ""
echo "=========================================="
echo "Running Test 8.1: Complete Flow"
echo "=========================================="
echo ""

if [ -f "${SCRIPT_DIR}/test_descend_complete_flow.sh" ]; then
    bash "${SCRIPT_DIR}/test_descend_complete_flow.sh" "${ROBOT_NAME}"
else
    echo "⚠ Test script not found: test_descend_complete_flow.sh"
fi

echo ""
read -p "Test 8.1 completed. Press Enter to continue to Test 8.2..."

# Test 8.2: Error handling
echo ""
echo "=========================================="
echo "Running Test 8.2: Error Handling"
echo "=========================================="
echo ""

if [ -f "${SCRIPT_DIR}/test_descend_stair.sh" ]; then
    bash "${SCRIPT_DIR}/test_descend_stair.sh" "${ROBOT_NAME}"
else
    echo "⚠ Test script not found: test_descend_stair.sh"
fi

echo ""
read -p "Test 8.2 completed. Press Enter to continue to Test 8.3..."

# Test 8.3: Climb-descend sequence
echo ""
echo "=========================================="
echo "Running Test 8.3: Climb-Descend Sequence"
echo "=========================================="
echo ""

if [ -f "${SCRIPT_DIR}/test_climb_descend_sequence.sh" ]; then
    bash "${SCRIPT_DIR}/test_climb_descend_sequence.sh" "${ROBOT_NAME}" 0.15
else
    echo "⚠ Test script not found: test_climb_descend_sequence.sh"
fi

# Final summary
echo ""
echo "=========================================="
echo "All Tests Completed!"
echo "=========================================="
echo ""
echo "Please review the results and complete the verification checklist"
echo "in README_DESCEND_TESTS.md"
echo ""
echo "Test Summary:"
echo "- Test 8.1 (Complete Flow): Check manual observations"
echo "- Test 8.2 (Error Handling): Check manual observations"
echo "- Test 8.3 (Climb-Descend): Check manual observations"
echo ""
echo "For automated verification, run:"
echo "  ./verify_descend_sequence.py ${ROBOT_NAME}"
echo "  ./test_error_handling.py ${ROBOT_NAME}"
echo "  ./test_climb_descend_automated.py ${ROBOT_NAME}"
echo ""
