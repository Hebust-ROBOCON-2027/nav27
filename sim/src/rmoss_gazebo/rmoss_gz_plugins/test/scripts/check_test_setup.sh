#!/bin/bash

# Verification script to check all test files are properly set up

echo "=========================================="
echo "Test Setup Verification"
echo "=========================================="
echo ""

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORLD_DIR="${SCRIPT_DIR}/../worlds"

all_ok=true

# Check test scripts
echo "Checking test scripts..."
scripts=(
    "test_descend_complete_flow.sh"
    "verify_descend_sequence.py"
    "test_descend_stair.sh"
    "test_error_handling.py"
    "test_climb_descend_sequence.sh"
    "test_climb_descend_automated.py"
    "run_all_descend_tests.sh"
)

for script in "${scripts[@]}"; do
    if [ -f "${SCRIPT_DIR}/${script}" ]; then
        if [ -x "${SCRIPT_DIR}/${script}" ]; then
            echo "  ✓ ${script} (executable)"
        else
            echo "  ✗ ${script} (not executable)"
            all_ok=false
        fi
    else
        echo "  ✗ ${script} (missing)"
        all_ok=false
    fi
done

# Check documentation
echo ""
echo "Checking documentation..."
docs=(
    "README_DESCEND_TESTS.md"
    "QUICK_START.md"
)

for doc in "${docs[@]}"; do
    if [ -f "${SCRIPT_DIR}/${doc}" ]; then
        echo "  ✓ ${doc}"
    else
        echo "  ✗ ${doc} (missing)"
        all_ok=false
    fi
done

# Check test world
echo ""
echo "Checking test world..."
if [ -f "${WORLD_DIR}/stair_descend_test.sdf" ]; then
    echo "  ✓ stair_descend_test.sdf"
else
    echo "  ✗ stair_descend_test.sdf (missing)"
    all_ok=false
fi

# Check Gazebo command
echo ""
echo "Checking Gazebo installation..."
if command -v gz &> /dev/null; then
    echo "  ✓ gz command available"
else
    echo "  ✗ gz command not found"
    echo "    Install Gazebo to run tests"
    all_ok=false
fi

# Summary
echo ""
echo "=========================================="
if [ "$all_ok" = true ]; then
    echo "✓ All test files are properly set up!"
    echo ""
    echo "Ready to run tests. See QUICK_START.md for instructions."
    exit 0
else
    echo "✗ Some test files are missing or not executable"
    echo ""
    echo "Fix the issues above before running tests."
    exit 1
fi
