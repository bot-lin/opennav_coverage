# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Development Commands

### Building the Project
```bash
# Build all packages in the workspace
colcon build

# Build with debug symbols for development
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug

# Build specific packages only
colcon build --packages-select opennav_coverage opennav_coverage_msgs
```

### Testing
```bash
# Run all C++ tests
colcon test

# Run tests for specific packages
colcon test --packages-select opennav_coverage

# Run Python tests in demo package
cd opennav_coverage_demo && python -m pytest

# Run individual test files
python test_simple_api.py
python test_ros2_param_api.py
```

### Linting and Code Quality
```bash
# The project uses ament linting - tests are run automatically with colcon test
# Individual linting can be done with:
ament_flake8 opennav_coverage_demo/
ament_pep257 opennav_coverage_demo/
ament_copyright opennav_coverage_demo/
```

## Architecture Overview

This is a ROS 2 complete coverage planning system built on top of the Fields2Cover library. The system is modularized into 6 main packages:

### Core Packages
- **opennav_coverage**: Main coverage server for open-field polygon-based coverage planning. Uses Fields2Cover to generate swaths, routes, and paths from field polygons.
- **opennav_row_coverage**: Alternative coverage server for pre-annotated/irregular row-based coverage. Handles tree farms and similar applications with established rows.
- **opennav_coverage_msgs**: Action definitions and message types for the coverage system.

### Integration Packages  
- **opennav_coverage_navigator**: BT Navigator plugin exposing `NavigateCompleteCoverage` action (requires ROS 2 Iron+).
- **opennav_coverage_bt**: Behavior Tree nodes and XML files for coverage navigation tasks.
- **opennav_coverage_demo**: Demo launch files and Python scripts for testing and demonstration.

### Key Components Architecture
The main coverage server (`CoverageServer`) follows a modular factory pattern with separate generators:
- `HeadlandGenerator`: Removes headland boundaries from fields
- `SwathGenerator`: Computes coverage swaths using different objectives (LENGTH, COVERAGE, NUMBER)
- `RouteGenerator`: Orders swaths using patterns (BOUSTROPHEDON, SNAKE, SPIRAL, CUSTOM)
- `PathGenerator`: Connects ordered swaths with feasible paths (REEDS_SHEPP, DUBIN)
- `Visualizer`: Publishes visualization markers for debugging

All generators accept different modes and parameters, making the system highly configurable for different agricultural applications.

## Key Configuration Files

### Parameter Files
- `opennav_coverage_demo/params/demo_params.yaml`: Complete parameter examples for all servers and Nav2 integration
- Key parameters include robot dimensions, turning radius, coordinate frames, and algorithm-specific settings

### Launch Files
- `opennav_coverage_demo/launch/coverage_demo_launch.py`: Full coverage navigation demo
- `opennav_coverage_demo/launch/row_coverage_demo_launch.py`: Row-based coverage demo
- HTTP API variants available for remote operation

## Dependencies

### External Dependencies
- **Fields2Cover v1.2.1**: Core coverage planning library (v2.0.0 not supported)
- **Nav2**: Navigation framework integration
- Standard ROS 2 packages (geometry_msgs, nav_msgs, tf2_ros, etc.)

### Dependency Installation
Fields2Cover must be built from source:
```bash
# Clone Fields2Cover into your workspace
cd src/
git clone https://github.com/Fields2Cover/Fields2Cover.git -b v1.2.1-devel
cd ../
colcon build --packages-select Fields2Cover
```

## Development Notes

### Coordinate System Handling
- Supports both GPS and Cartesian coordinate inputs via `coordinates_in_cartesian_frame` parameter
- GML files can contain coordinate frame information, while polygon messages require explicit `frame_id`

### Testing and Field Files
- Test field polygons located in `opennav_coverage/test/`: `test_field.xml`, `cartesian_test_field.xml`, `irregular_test_field.xml`
- Demo utilities in `tester.py` scripts for standalone testing

### Key Interfaces
- `ComputeCoveragePath` action: Main interface for coverage path planning
- `NavigateCompleteCoverage` action: High-level navigation interface
- Custom `PathComponents` message type for detailed coverage path representation with `PathComponentsIterator` utility

### Code Standards  
- C++17 standard with strict compiler flags (-Wall -Wextra -Wpedantic -Werror)
- Uses Nav2 lifecycle node patterns
- Follows ROS 2 ament package structure
- Python packages use setuptools with pytest testing