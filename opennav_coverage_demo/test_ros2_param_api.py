#!/usr/bin/env python3
"""
Test script for the ROS2 parameter update functionality in the coverage HTTP API.
This script demonstrates how to use the simple parameter endpoints to update
both YAML configuration files and live ROS2 parameters.
"""

import requests
import json
import time

# API base URL
BASE_URL = "http://localhost:5000"

def test_parameter_endpoints():
    """Test all parameter endpoints with ROS2 parameter updates"""
    
    # First set the config file
    print("Setting config file...")
    response = requests.post(f"{BASE_URL}/config_file", 
                           json={"file_path": "/path/to/your/config.yaml"})
    print(f"Config file response: {response.json()}")
    
    # Test parameters to update
    test_params = [
        ("robot_width", 0.8),
        ("operation_width", 2.5),
        ("min_turning_radius", 1.2),
        ("headland_width", 0.3),
        ("swath_angle", 1.57),  # 90 degrees in radians
        ("allow_overlap", True)
    ]
    
    for param_name, test_value in test_params:
        print(f"\n--- Testing {param_name} ---")
        
        # Get current value
        response = requests.get(f"{BASE_URL}/{param_name}")
        if response.status_code == 200:
            current_data = response.json()
            print(f"Current {param_name}: {current_data.get('value')}")
        else:
            print(f"Failed to get current {param_name}: {response.text}")
        
        # Set new value
        print(f"Setting {param_name} to {test_value}...")
        response = requests.post(f"{BASE_URL}/{param_name}", 
                               json={"value": test_value})
        
        if response.status_code == 200:
            result = response.json()
            print(f"✅ Update successful!")
            print(f"   Parameter: {result['parameter']}")
            print(f"   Old value: {result['old_value']}")
            print(f"   New value: {result['new_value']}")
            print(f"   ROS2 param updated: {result['ros2_parameter_updated']}")
            
            if 'warning' in result:
                print(f"   ⚠️  Warning: {result['warning']}")
                
        else:
            print(f"❌ Update failed: {response.text}")
        
        # Verify the change
        time.sleep(0.5)  # Small delay
        response = requests.get(f"{BASE_URL}/{param_name}")
        if response.status_code == 200:
            verify_data = response.json()
            print(f"Verified {param_name}: {verify_data.get('value')}")
        
        time.sleep(1)  # Pause between tests

def test_boolean_parameter():
    """Test boolean parameter specifically"""
    print("\n--- Testing Boolean Parameter (allow_overlap) ---")
    
    # Test setting to False
    response = requests.post(f"{BASE_URL}/allow_overlap", 
                           json={"value": False})
    print(f"Set to False: {response.json()}")
    
    # Test setting to True
    response = requests.post(f"{BASE_URL}/allow_overlap", 
                           json={"value": True})
    print(f"Set to True: {response.json()}")

def test_error_cases():
    """Test error handling"""
    print("\n--- Testing Error Cases ---")
    
    # Test invalid value type
    response = requests.post(f"{BASE_URL}/robot_width", 
                           json={"value": "invalid"})
    print(f"Invalid type response: {response.json()}")
    
    # Test missing value
    response = requests.post(f"{BASE_URL}/robot_width", 
                           json={})
    print(f"Missing value response: {response.json()}")

if __name__ == "__main__":
    print("🚀 Testing ROS2 Parameter Update API")
    print("=" * 50)
    
    try:
        test_parameter_endpoints()
        test_boolean_parameter()
        test_error_cases()
        
        print("\n" + "=" * 50)
        print("✅ All tests completed!")
        
    except requests.exceptions.ConnectionError:
        print("❌ Could not connect to the API server.")
        print("Make sure the coverage HTTP demo is running on localhost:5000")
    except Exception as e:
        print(f"❌ Test failed with error: {str(e)}") 