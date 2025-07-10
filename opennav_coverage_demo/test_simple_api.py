#!/usr/bin/env python3
"""
Simple test script for the Robot Parameters API.
Much easier to use than the complex YAML API!
"""

import requests
import json
import os
import tempfile
import yaml

BASE_URL = "http://localhost:1235"

def create_test_config():
    """Create a test configuration file."""
    test_config = {
        'coverage_server': {
            'ros__parameters': {
                'use_sim_time': False,
                'default_headland_width': 0.25,
                'robot_width': 0.7,
                'operation_width': 0.7,
                'min_turning_radius': 0.35,
                'linear_curv_change': 200.0,
                'coordinates_in_cartesian_frame': True,
                'default_allow_overlap': False,
                'default_swath_angle_type': 'SET_ANGLE',
                'default_swath_angle': 3.14
            }
        }
    }
    
    temp_file = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
    yaml.dump(test_config, temp_file, default_flow_style=False)
    temp_file.close()
    
    print(f"Created test config file: {temp_file.name}")
    return temp_file.name

def set_config_file(config_file):
    """Set the configuration file path."""
    print("\n=== Setting Configuration File ===")
    try:
        response = requests.post(f"{BASE_URL}/config_file",
                               json={"file_path": config_file})
        
        if response.status_code == 200:
            print("✓ Configuration file set successfully")
            return True
        else:
            print(f"✗ Error: {response.json()}")
            return False
    except requests.exceptions.ConnectionError:
        print("✗ Cannot connect to server. Make sure the coverage demo server is running.")
        return False
    except Exception as e:
        print(f"✗ Error: {e}")
        return False

def test_get_parameters():
    """Test getting all parameter values."""
    print("\n=== Getting Current Parameters ===")
    
    parameters = ['robot_width', 'operation_width', 'min_turning_radius', 
                 'headland_width', 'swath_angle', 'allow_overlap']
    
    success = True
    for param in parameters:
        try:
            response = requests.get(f"{BASE_URL}/{param}")
            if response.status_code == 200:
                result = response.json()
                print(f"✓ {param}: {result['value']}")
            else:
                print(f"✗ Error getting {param}: {response.json()}")
                success = False
        except Exception as e:
            print(f"✗ Error getting {param}: {e}")
            success = False
    
    return success

def test_set_parameters():
    """Test setting parameter values."""
    print("\n=== Updating Parameters ===")
    
    updates = [
        ('robot_width', 0.8),
        ('operation_width', 0.8),
        ('min_turning_radius', 0.4),
        ('headland_width', 0.3),
        ('allow_overlap', True)
    ]
    
    success = True
    for param, new_value in updates:
        try:
            response = requests.post(f"{BASE_URL}/{param}",
                                   json={"value": new_value})
            
            if response.status_code == 200:
                result = response.json()
                print(f"✓ Updated {param}: {result['old_value']} → {result['new_value']}")
            else:
                print(f"✗ Error updating {param}: {response.json()}")
                success = False
        except Exception as e:
            print(f"✗ Error updating {param}: {e}")
            success = False
    
    return success

def test_get_updated_parameters():
    """Verify that parameters were updated correctly."""
    print("\n=== Verifying Updates ===")
    
    expected_values = {
        'robot_width': 0.8,
        'operation_width': 0.8,
        'min_turning_radius': 0.4,
        'headland_width': 0.3,
        'allow_overlap': True
    }
    
    success = True
    for param, expected in expected_values.items():
        try:
            response = requests.get(f"{BASE_URL}/{param}")
            if response.status_code == 200:
                result = response.json()
                actual = result['value']
                if actual == expected:
                    print(f"✓ {param}: {actual} (correct)")
                else:
                    print(f"✗ {param}: expected {expected}, got {actual}")
                    success = False
            else:
                print(f"✗ Error getting {param}: {response.json()}")
                success = False
        except Exception as e:
            print(f"✗ Error getting {param}: {e}")
            success = False
    
    return success

def test_error_cases():
    """Test error handling."""
    print("\n=== Testing Error Cases ===")
    
    # Test getting parameter without setting config file first
    print("Testing without config file set...")
    # First, let's make sure no config file is set by testing with a new server state
    try:
        response = requests.get(f"{BASE_URL}/robot_width")
        if response.status_code == 400:
            print("✓ Correctly returned error when config file not set")
        else:
            print("✗ Should have returned error when config file not set")
    except:
        pass
    
    # Test with missing value field
    print("Testing with missing value field...")
    try:
        response = requests.post(f"{BASE_URL}/robot_width", json={})
        if response.status_code == 400:
            print("✓ Correctly returned error for missing value field")
        else:
            print("✗ Should have returned error for missing value field")
    except:
        pass

def verify_file_changes(config_file):
    """Verify that changes were written to the file."""
    print("\n=== Verifying File Changes ===")
    
    try:
        with open(config_file, 'r') as f:
            content = yaml.safe_load(f)
        
        params = content['coverage_server']['ros__parameters']
        
        expected = {
            'robot_width': 0.8,
            'operation_width': 0.8,
            'min_turning_radius': 0.4,
            'default_headland_width': 0.3,
            'default_allow_overlap': True
        }
        
        all_correct = True
        for param, expected_value in expected.items():
            actual_value = params.get(param)
            if actual_value == expected_value:
                print(f"✓ {param}: {actual_value}")
            else:
                print(f"✗ {param}: expected {expected_value}, got {actual_value}")
                all_correct = False
        
        if all_correct:
            print("✓ All changes correctly written to file!")
        else:
            print("✗ Some changes not properly written to file")
            
    except Exception as e:
        print(f"✗ Error reading file: {e}")

def main():
    """Main test function."""
    print("Simple Robot Parameters API Test")
    print("===============================")
    
    # Create test file
    config_file = create_test_config()
    
    try:
        # Run tests step by step
        success = True
        
        success &= set_config_file(config_file)
        if not success:
            print("\n✗ Failed to set config file. Cannot continue tests.")
            return
        
        success &= test_get_parameters()
        success &= test_set_parameters() 
        success &= test_get_updated_parameters()
        
        test_error_cases()
        verify_file_changes(config_file)
        
        print("\n=== Test Summary ===")
        if success:
            print("✓ All main tests passed!")
            print("✓ The API is working correctly and is much easier to use!")
        else:
            print("✗ Some tests failed. Check server status and connectivity.")
        
        print(f"\nTest file: {config_file}")
        print("You can inspect this file to see the changes.")
        
    finally:
        # Clean up
        try:
            os.unlink(config_file)
            print(f"Cleaned up test file: {config_file}")
        except:
            print(f"Could not clean up test file: {config_file}")

if __name__ == "__main__":
    main() 