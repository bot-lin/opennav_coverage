# Costmap Processing Fix for demo_coverage_http.py

## Problem
The costmap processing was failing with the following errors:
1. `int() argument must be a string, a bytes-like object or a real number, not 'dict'`
2. `cannot access local variable 'free_contours_world' where it is not associated with a value`
3. `Invalid costmap message format - missing 'data' or 'metadata' keys`

These errors occurred because:
1. The HTTP service returns data in a wrapper format: `{'code': 0, 'data': actual_costmap}`
2. The costmap data contained dictionaries instead of plain numbers in some cases
3. The `free_contours_world` variable wasn't initialized before the try block

## Solution
Applied several fixes to handle different costmap data formats robustly:

### 1. Handle HTTP Service Wrapper Format
```python
# Handle different response formats from the HTTP service
# The service may return {'code': 0, 'data': actual_costmap_data}
actual_costmap_data = msg
if 'code' in msg and 'data' in msg:
    # HTTP service wrapper format
    if msg['code'] == 0:
        actual_costmap_data = msg['data']
        self.get_logger().info("Using costmap data from HTTP service wrapper")
    else:
        self.get_logger().error(f"HTTP service returned error code: {msg['code']}")
        return []

# Use the actual costmap data for processing
msg = actual_costmap_data
```

### 2. Added Debug Logging
```python
# Debug: Print the structure of the received message
self.get_logger().info(f"Costmap message type: {type(msg)}")
self.get_logger().info(f"Costmap keys: {list(msg.keys()) if isinstance(msg, dict) else 'Not a dict'}")

# Log first few elements for debugging
if hasattr(costmap_data, '__len__') and len(costmap_data) > 0:
    sample_size = min(3, len(costmap_data))
    self.get_logger().info(f"First {sample_size} data elements: {costmap_data[:sample_size]}")
```

### 3. Robust Data Format Handling
```python
# Handle different data formats
if isinstance(costmap_data, list):
    try:
        data = np.array(costmap_data, dtype=np.uint8)
    except (ValueError, TypeError) as e:
        # Try to extract numerical values if data contains dicts
        if len(costmap_data) > 0 and isinstance(costmap_data[0], dict):
            self.get_logger().info("Data appears to be list of dicts, attempting to extract values...")
            # Try common keys like 'value', 'cost', 'data'
            for key in ['value', 'cost', 'data', 'cell_value']:
                if key in costmap_data[0]:
                    try:
                        data = np.array([cell[key] for cell in costmap_data], dtype=np.uint8)
                        self.get_logger().info(f"Successfully extracted data using key '{key}'")
                        break
                    except Exception:
                        continue
            else:
                self.get_logger().error("Could not extract numerical data from dict format")
                return []
```

### 4. Variable Initialization
```python
def process_costmap(self, msg):
    free_contours_world = []  # Initialize at the start
    try:
        # ... processing code ...
        return free_contours_world
        
    except Exception as e:
        self.get_logger().error(f'Error processing costmap: {str(e)}')
        return []  # Return empty list on error
```

### 5. Enhanced Error Handling in Service Call
```python
def call_get_costmap_service(self):
    try:
        # ... service call code ...
        free_contours_world = self.process_costmap(data)
        
        # Check if we got valid contours
        if free_contours_world and len(free_contours_world) > 0:
            return free_contours_world[0]['world_coordinates']
        else:
            self.get_logger().warn('No free contours found in costmap')
            return None
    except Exception as e:
        self.get_logger().error(f'Error calling service: {str(e)}')
        return None
```

### 6. Data Validation
```python
# Reshape the data with validation
try:
    data = data.reshape((msg['metadata']['size_y'], msg['metadata']['size_x']))
except ValueError as e:
    self.get_logger().error(f"Cannot reshape data: {e}")
    self.get_logger().error(f"Data shape: {data.shape}, Expected: ({msg['metadata']['size_y']}, {msg['metadata']['size_x']})")
    return []
```

## Files Modified
- `opennav_coverage_demo/opennav_coverage_demo/demo_coverage_http.py`

## Testing
Run syntax check:
```bash
python3 -m py_compile opennav_coverage_demo/opennav_coverage_demo/demo_coverage_http.py
```

The fix now handles:
- HTTP service wrapper format (`{'code': 0, 'data': costmap}`)  
- Different costmap data formats (list of numbers vs list of dictionaries)
- Proper error handling and logging for debugging
- Variable initialization to prevent unbound variable errors
- Validation of data shapes and formats before processing

The system will now:
- Automatically detect and handle HTTP service wrapper responses
- Provide detailed debug information to identify exact costmap data formats
- Attempt to extract numerical values from various dictionary structures
- Process both `call_get_costmap_service()` and `crop_and_find_free_space()` with the same robust format handling