# OpenNav Coverage Demo

This package contains the coverage navigator demo, using the Navigator, BT XML, BT Nodes, and Coverage Server to perform a simple coverage task.

## Simple Robot Parameters API

The HTTP server provides easy-to-use endpoints for common robot parameters. No complex file paths or dot notation required!

### Setup

First, set the configuration file path:
```bash
curl -X POST http://localhost:1235/config_file \
  -H "Content-Type: application/json" \
  -d '{"file_path": "/path/to/demo_params.yaml"}'
```

### Available Parameters

| Parameter | Description | Example Value |
|-----------|-------------|---------------|
| `robot_width` | Robot width in meters | `0.7` |
| `operation_width` | Operation width in meters | `0.7` |
| `min_turning_radius` | Minimum turning radius in meters | `0.35` |
| `headland_width` | Headland width in meters | `0.25` |
| `swath_angle` | Swath angle in radians | `3.14` |
| `allow_overlap` | Allow overlap (true/false) | `false` |

### API Endpoints

#### Get Parameter Value
```bash
curl http://localhost:1235/robot_width
curl http://localhost:1235/operation_width
curl http://localhost:1235/min_turning_radius
curl http://localhost:1235/headland_width
curl http://localhost:1235/swath_angle
curl http://localhost:1235/allow_overlap
```

**Response:**
```json
{
  "code": 0,
  "parameter": "robot_width",
  "value": 0.7
}
```

#### Set Parameter Value
```bash
curl -X POST http://localhost:1235/robot_width \
  -H "Content-Type: application/json" \
  -d '{"value": 0.8}'

curl -X POST http://localhost:1235/operation_width \
  -H "Content-Type: application/json" \
  -d '{"value": 0.8}'

curl -X POST http://localhost:1235/allow_overlap \
  -H "Content-Type: application/json" \
  -d '{"value": true}'
```

**Response:**
```json
{
  "code": 0,
  "message": "参数更新成功",
  "parameter": "robot_width",
  "old_value": 0.7,
  "new_value": 0.8
}
```

### Usage Examples

#### Using curl

**Set configuration file:**
```bash
curl -X POST http://localhost:1235/config_file \
  -H "Content-Type: application/json" \
  -d '{"file_path": "/path/to/demo_params.yaml"}'
```

**Get robot width:**
```bash
curl http://localhost:1235/robot_width
```

**Update robot width:**
```bash
curl -X POST http://localhost:1235/robot_width \
  -H "Content-Type: application/json" \
  -d '{"value": 0.8}'
```

#### Using Python requests

```python
import requests

base_url = "http://localhost:1235"

# Set config file
requests.post(f"{base_url}/config_file", 
              json={"file_path": "/path/to/demo_params.yaml"})

# Get current robot width
response = requests.get(f"{base_url}/robot_width")
print(f"Current robot width: {response.json()['value']}")

# Update robot width
requests.post(f"{base_url}/robot_width", json={"value": 0.8})

# Update multiple parameters
requests.post(f"{base_url}/operation_width", json={"value": 0.8})
requests.post(f"{base_url}/allow_overlap", json={"value": True})

print("Parameters updated!")
```

### Error Responses

All endpoints return error responses with `code: 1` when issues occur:

```json
{
  "code": 1,
  "error": "请先设置配置文件路径"
}
```

Common error scenarios:
- Config file not set (400)
- File not found (404)
- Invalid JSON format (400)
- Missing value field (400)
- Parameter not found (404)
