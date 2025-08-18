# Threading Fix for demo_coverage_http.py

## Problem
The original code had a race condition error where the ROS 2 action client was being accessed from multiple threads simultaneously:

```
rclpy._rclpy_pybind11.RCLError: Failed to get number of ready entities for action client: wait set index for status subscription is out of bounds
```

This occurred because:
1. One thread was running `rclpy.spin()` to handle ROS callbacks
2. Another thread was handling HTTP requests and trying to use the same action client
3. `spin_until_future_complete()` calls were creating unsafe concurrent access

## Solution
Applied several thread-safety fixes:

### 1. Use MultiThreadedExecutor
```python
# Before: 
ros_thread = threading.Thread(target=rclpy.spin, args=(navigator_server,))

# After:
from rclpy.executors import MultiThreadedExecutor
executor = MultiThreadedExecutor()
executor.add_node(navigator_server)
ros_thread = threading.Thread(target=executor.spin)
```

### 2. Replace spin_until_future_complete() calls
```python
# Before:
rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

# After:
timeout_count = 0
while not future.done() and timeout_count < 20:  # 2 second timeout
    time.sleep(0.1)
    timeout_count += 1

if future.done() and future.result() is not None:
    # Process result
```

### 3. Add Thread Safety Lock
```python
def __init__(self):
    # ... existing code ...
    self._action_lock = threading.Lock()  # Thread safety lock

def navigateCoverage(self, ...):
    with self._action_lock:  # Thread-safe access to action client
        # ... action client operations ...
```

### 4. Use future.done() instead of spin_until_future_complete()
```python
# Before:
if self.result_future.result():
    # Process result

# After:
if self.result_future.done():
    result = self.result_future.result()
    if result:
        # Process result
```

## Files Modified
- `opennav_coverage_demo/opennav_coverage_demo/demo_coverage_http.py`

## Testing
Run syntax check:
```bash
python3 -m py_compile opennav_coverage_demo/opennav_coverage_demo/demo_coverage_http.py
```

The fix ensures thread-safe access to ROS 2 action clients when using the HTTP API server.