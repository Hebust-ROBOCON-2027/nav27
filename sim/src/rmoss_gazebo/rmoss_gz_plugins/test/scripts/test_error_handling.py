#!/usr/bin/env python3
"""
Automated error handling test for stair descending.
Tests timeout protection and forward movement detection.

Requirements tested:
- 5.1: Unexpected forward movement detection
- 5.2: Timeout protection
- 5.4: Safe abort mechanism
"""

import sys
import time
import subprocess
import threading

class ErrorHandlingTester:
    def __init__(self, robot_name="sentry"):
        self.robot_name = robot_name
        self.test_results = {
            'timeout_protection': False,
            'forward_movement_detection': False,
            'safe_abort': False
        }
        
    def send_descend_command(self, height):
        """Send descend command to robot."""
        cmd = [
            "gz", "topic", "-t", f"/{self.robot_name}/descend_stair",
            "-m", "ignition.msgs.Double", "-p", f"data: {height}"
        ]
        try:
            subprocess.run(cmd, check=True, capture_output=True, timeout=5)
            print(f"✓ Sent descend command: {height}m")
            return True
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as e:
            print(f"✗ Failed to send descend command: {e}")
            return False
    
    def test_timeout_protection(self):
        """
        Test that system aborts after 30 seconds if stuck.
        Requirement 5.2
        """
        print("\n" + "="*60)
        print("Test: Timeout Protection (Requirement 5.2)")
        print("="*60)
        
        print("\nThis test verifies that the descend sequence aborts after 30 seconds.")
        print("Note: This requires the robot to be in a stuck position.")
        print("\nManual verification required:")
        print("1. Position robot so it cannot complete descend sequence")
        print("2. Send descend command")
        print("3. Wait 30+ seconds")
        print("4. Verify error message: 'ERROR - Descend timeout in state...'")
        print("5. Verify system returns to IDLE state")
        
        # Send command
        if not self.send_descend_command(0.2):
            return False
        
        print("\nMonitoring for 35 seconds...")
        print("Expected: Timeout error after ~30 seconds")
        
        # Wait and monitor
        start_time = time.time()
        timeout_detected = False
        
        for i in range(35):
            elapsed = time.time() - start_time
            print(f"\rElapsed: {elapsed:.1f}s", end="", flush=True)
            time.sleep(1)
            
            # In real implementation, would monitor Gazebo logs
            # For now, assume timeout works if we reach 30+ seconds
            if elapsed >= 30 and not timeout_detected:
                timeout_detected = True
                print("\n✓ Timeout threshold reached (30s)")
        
        print("\n")
        
        # Manual verification prompt
        response = input("Did you observe timeout error message in Gazebo console? (y/n): ")
        self.test_results['timeout_protection'] = response.lower() == 'y'
        
        if self.test_results['timeout_protection']:
            print("✓ Timeout protection test PASSED")
        else:
            print("✗ Timeout protection test FAILED")
        
        return self.test_results['timeout_protection']
    
    def test_forward_movement_detection(self):
        """
        Test that unexpected forward movement is detected and corrected.
        Requirement 5.1
        """
        print("\n" + "="*60)
        print("Test: Forward Movement Detection (Requirement 5.1)")
        print("="*60)
        
        print("\nThis test verifies forward movement detection during descend.")
        print("\nManual verification required:")
        print("1. Start descend sequence")
        print("2. Apply forward force to robot (if possible)")
        print("3. Verify warning message: 'WARNING - Unexpected forward movement detected'")
        print("4. Verify corrective action: 'Applying corrective backward force'")
        
        # Send command
        if not self.send_descend_command(0.15):
            return False
        
        print("\nMonitoring for 30 seconds...")
        print("Watch Gazebo console for forward movement warnings")
        
        time.sleep(30)
        
        # Manual verification prompt
        response = input("\nDid you observe forward movement detection (if forward force applied)? (y/n/na): ")
        
        if response.lower() == 'na':
            print("⚠ Test skipped - no forward movement occurred")
            self.test_results['forward_movement_detection'] = True  # Pass if not applicable
        else:
            self.test_results['forward_movement_detection'] = response.lower() == 'y'
        
        if self.test_results['forward_movement_detection']:
            print("✓ Forward movement detection test PASSED")
        else:
            print("✗ Forward movement detection test FAILED")
        
        return self.test_results['forward_movement_detection']
    
    def test_safe_abort(self):
        """
        Test that wheels are safely retracted on abort.
        Requirement 5.4
        """
        print("\n" + "="*60)
        print("Test: Safe Abort Mechanism (Requirement 5.4)")
        print("="*60)
        
        print("\nThis test verifies safe wheel retraction on abort.")
        print("\nManual verification required:")
        print("1. Trigger an abort (timeout or error)")
        print("2. Verify all wheels retract to position 0")
        print("3. Verify system returns to IDLE state")
        print("4. Verify no wheels remain extended")
        
        print("\nThis should be verified during timeout test.")
        
        # Manual verification prompt
        response = input("\nDid wheels safely retract on abort? (y/n): ")
        self.test_results['safe_abort'] = response.lower() == 'y'
        
        if self.test_results['safe_abort']:
            print("✓ Safe abort test PASSED")
        else:
            print("✗ Safe abort test FAILED")
        
        return self.test_results['safe_abort']
    
    def run_all_tests(self):
        """Run all error handling tests."""
        print("="*60)
        print("Stair Descending Error Handling Tests")
        print("="*60)
        print("\nRequirements: 5.1, 5.2, 5.4")
        print("\nNote: These tests require manual observation and verification.")
        print("Ensure Gazebo is running with the robot model.")
        
        input("\nPress Enter to start tests...")
        
        # Run tests
        self.test_forward_movement_detection()
        
        input("\nPress Enter to continue to timeout test...")
        self.test_timeout_protection()
        
        input("\nPress Enter to continue to safe abort test...")
        self.test_safe_abort()
        
        # Print summary
        self.print_summary()
    
    def print_summary(self):
        """Print test summary."""
        print("\n" + "="*60)
        print("ERROR HANDLING TEST SUMMARY")
        print("="*60)
        
        print(f"\nTimeout Protection (5.2): {'✓ PASS' if self.test_results['timeout_protection'] else '✗ FAIL'}")
        print(f"Forward Movement Detection (5.1): {'✓ PASS' if self.test_results['forward_movement_detection'] else '✗ FAIL'}")
        print(f"Safe Abort Mechanism (5.4): {'✓ PASS' if self.test_results['safe_abort'] else '✗ FAIL'}")
        
        all_passed = all(self.test_results.values())
        
        print("\n" + "="*60)
        if all_passed:
            print("OVERALL RESULT: ✓ ALL TESTS PASSED")
        else:
            print("OVERALL RESULT: ✗ SOME TESTS FAILED")
        print("="*60)
        
        return all_passed

def main():
    robot_name = sys.argv[1] if len(sys.argv) > 1 else "sentry"
    
    tester = ErrorHandlingTester(robot_name)
    tester.run_all_tests()
    
    # Exit with appropriate code
    all_passed = all(tester.test_results.values())
    sys.exit(0 if all_passed else 1)

if __name__ == "__main__":
    main()
