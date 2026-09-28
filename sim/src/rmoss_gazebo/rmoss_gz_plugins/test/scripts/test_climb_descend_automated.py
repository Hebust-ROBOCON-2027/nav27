#!/usr/bin/env python3
"""
Automated test for climb-then-descend sequence.
Verifies state machine switching and inverse operation.

Requirements tested:
- 6.1: Inverse operation sequence
- 6.4: State machine switching
"""

import sys
import time
import subprocess

class ClimbDescendTester:
    def __init__(self, robot_name="sentry"):
        self.robot_name = robot_name
        self.test_results = {
            'climb_success': False,
            'descend_success': False,
            'state_switching': False,
            'inverse_sequence': False
        }
        
    def send_climb_command(self, height):
        """Send climb command to robot."""
        cmd = [
            "gz", "topic", "-t", f"/{self.robot_name}/climb_stair",
            "-m", "ignition.msgs.Double", "-p", f"data: {height}"
        ]
        try:
            subprocess.run(cmd, check=True, capture_output=True, timeout=5)
            print(f"✓ Sent climb command: {height}m")
            return True
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as e:
            print(f"✗ Failed to send climb command: {e}")
            return False
    
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
    
    def test_climb_phase(self, height):
        """Test climb phase."""
        print("\n" + "="*60)
        print(f"Phase 1: Climb Stair ({height}m)")
        print("="*60)
        
        print("\nExpected sequence:")
        print("1. Robot moves forward")
        print("2. Wheel 2 (middle lift) extends first")
        print("3. Wheel 4 (rear mecanum) extends second")
        print("4. All wheels retract")
        print("5. Climb completes")
        
        # Send command
        if not self.send_climb_command(height):
            return False
        
        print("\nMonitoring climb sequence (30 seconds)...")
        time.sleep(30)
        
        # Manual verification
        response = input("\nDid climb complete successfully? (y/n): ")
        self.test_results['climb_success'] = response.lower() == 'y'
        
        if self.test_results['climb_success']:
            print("✓ Climb phase PASSED")
        else:
            print("✗ Climb phase FAILED")
        
        return self.test_results['climb_success']
    
    def test_descend_phase(self, height):
        """Test descend phase."""
        print("\n" + "="*60)
        print(f"Phase 2: Descend Stair ({height}m)")
        print("="*60)
        
        print("\nExpected sequence (INVERSE of climb):")
        print("1. Robot moves backward")
        print("2. Wheel 4 (rear mecanum) extends first")
        print("3. Wheel 2 (middle lift) extends second")
        print("4. All wheels retract")
        print("5. Descend completes")
        
        # Wait between operations
        print("\nWaiting 5 seconds before descend...")
        time.sleep(5)
        
        # Send command
        if not self.send_descend_command(height):
            return False
        
        print("\nMonitoring descend sequence (30 seconds)...")
        time.sleep(30)
        
        # Manual verification
        response = input("\nDid descend complete successfully? (y/n): ")
        self.test_results['descend_success'] = response.lower() == 'y'
        
        if self.test_results['descend_success']:
            print("✓ Descend phase PASSED")
        else:
            print("✗ Descend phase FAILED")
        
        return self.test_results['descend_success']
    
    def verify_state_switching(self):
        """Verify state machine switching."""
        print("\n" + "="*60)
        print("Verification: State Machine Switching")
        print("="*60)
        
        print("\nVerify in Gazebo console:")
        print("1. Clean transition from CLIMB_COMPLETE to DESCEND_BACKWARD1")
        print("2. No state conflicts or errors")
        print("3. Both sequences completed within timeout")
        
        response = input("\nDid state machine switch correctly? (y/n): ")
        self.test_results['state_switching'] = response.lower() == 'y'
        
        if self.test_results['state_switching']:
            print("✓ State switching PASSED")
        else:
            print("✗ State switching FAILED")
        
        return self.test_results['state_switching']
    
    def verify_inverse_sequence(self):
        """Verify inverse operation sequence."""
        print("\n" + "="*60)
        print("Verification: Inverse Operation Sequence")
        print("="*60)
        
        print("\nVerify wheel extension order:")
        print("Climb:   Wheel 2 → Wheel 4 → Retract")
        print("Descend: Wheel 4 → Wheel 2 → Retract (INVERSE)")
        
        response = input("\nWas the wheel sequence inverse? (y/n): ")
        self.test_results['inverse_sequence'] = response.lower() == 'y'
        
        if self.test_results['inverse_sequence']:
            print("✓ Inverse sequence PASSED")
        else:
            print("✗ Inverse sequence FAILED")
        
        return self.test_results['inverse_sequence']
    
    def run_test_cycle(self, height):
        """Run one complete climb-descend cycle."""
        print("\n" + "="*60)
        print(f"Climb-Descend Cycle Test ({height}m)")
        print("="*60)
        
        # Run climb
        if not self.test_climb_phase(height):
            print("\n⚠ Climb failed, skipping descend phase")
            return False
        
        # Run descend
        if not self.test_descend_phase(height):
            print("\n⚠ Descend failed")
            return False
        
        # Verify switching
        self.verify_state_switching()
        
        # Verify inverse sequence
        self.verify_inverse_sequence()
        
        return True
    
    def run_all_tests(self):
        """Run all climb-descend tests."""
        print("="*60)
        print("Climb-Then-Descend Sequence Tests")
        print("="*60)
        print("\nRequirements: 6.1, 6.4")
        print("\nNote: These tests require manual observation.")
        print("Ensure Gazebo is running with the robot model.")
        
        input("\nPress Enter to start test...")
        
        # Test with 0.15m stair
        self.run_test_cycle(0.15)
        
        # Optional: Test multiple cycles
        while True:
            response = input("\nRun another cycle? (y/n): ")
            if response.lower() != 'y':
                break
            
            height = input("Enter stair height (default 0.15): ") or "0.15"
            try:
                height = float(height)
                self.run_test_cycle(height)
            except ValueError:
                print("Invalid height, skipping")
        
        # Print summary
        self.print_summary()
    
    def print_summary(self):
        """Print test summary."""
        print("\n" + "="*60)
        print("CLIMB-DESCEND TEST SUMMARY")
        print("="*60)
        
        print(f"\nClimb Success: {'✓ PASS' if self.test_results['climb_success'] else '✗ FAIL'}")
        print(f"Descend Success: {'✓ PASS' if self.test_results['descend_success'] else '✗ FAIL'}")
        print(f"State Switching (6.4): {'✓ PASS' if self.test_results['state_switching'] else '✗ FAIL'}")
        print(f"Inverse Sequence (6.1): {'✓ PASS' if self.test_results['inverse_sequence'] else '✗ FAIL'}")
        
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
    
    tester = ClimbDescendTester(robot_name)
    tester.run_all_tests()
    
    # Exit with appropriate code
    all_passed = all(tester.test_results.values())
    sys.exit(0 if all_passed else 1)

if __name__ == "__main__":
    main()
