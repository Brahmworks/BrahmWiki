#!/usr/bin/env python3
"""
Quick Test Runner for ESP32 Robot Controller

This script provides a simplified interface to test the ESP32 communication.
"""

import subprocess
import sys
import os

def run_test(test_type="all", esp32_ip=None):
    """Run a specific test"""
    
    script_dir = os.path.dirname(os.path.abspath(__file__))
    test_script = os.path.join(script_dir, "test_client.py")
    
    cmd = [sys.executable, test_script]
    
    if esp32_ip:
        cmd.extend(["--esp32-ip", esp32_ip])
    
    if test_type != "all":
        cmd.extend(["--test", test_type])
    
    print(f"Running: {' '.join(cmd)}")
    print("-" * 60)
    
    try:
        subprocess.run(cmd, check=True)
    except subprocess.CalledProcessError as e:
        print(f"Error: Test failed with exit code {e.returncode}")
        return False
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
        return False
    
    return True

def main():
    """Main menu"""
    print("=" * 60)
    print("ESP32 Robot Controller - Test Menu")
    print("=" * 60)
    print()
    print("Options:")
    print("  1. Run all tests (auto-discover ESP32)")
    print("  2. Run all tests (specify ESP32 IP)")
    print("  3. Test torso motor")
    print("  4. Test neck motor")
    print("  5. Test head motor")
    print("  6. Test touch sensor")
    print("  7. Test LED states")
    print("  8. Test battery info")
    print("  9. Test reboot signal")
    print("  0. Exit")
    print()
    
    tests = {
        "1": ("all", None),
        "2": ("all", None),  # Will prompt for IP
        "3": ("torso", None),
        "4": ("neck", None),
        "5": ("head", None),
        "6": ("touch", None),
        "7": ("led", None),
        "8": ("battery", None),
        "9": ("reboot", None),
    }
    
    while True:
        choice = input("Select test (0-9): ").strip()
        
        if choice == "0":
            print("Exiting...")
            break
        
        if choice not in tests:
            print("Invalid selection. Please try again.")
            continue
        
        test_type, ip = tests[choice]
        
        if choice == "2":
            ip = input("Enter ESP32 IP address: ").strip()
            if not ip:
                print("IP address required")
                continue
        
        print()
        if run_test(test_type, ip):
            print("\n✅ Test completed successfully")
        else:
            print("\n❌ Test failed")
        
        print()
        input("Press Enter to continue...")
        print()

if __name__ == "__main__":
    if len(sys.argv) > 1:
        # Command line mode
        if sys.argv[1] in ["torso", "neck", "head", "touch", "led", "battery", "reboot"]:
            run_test(sys.argv[1])
        elif sys.argv[1] == "all":
            run_test("all")
        else:
            print(f"Usage: python3 run_test.py [all|torso|neck|head|touch|led|battery|reboot]")
    else:
        # Interactive menu
        main()
