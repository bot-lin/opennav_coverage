# Field Visualization for Debugging

## Overview
Added visual debugging capabilities to help understand and verify the field polygons being used for coverage planning. The system now generates annotated images showing the exact field boundaries being processed.

## New Features

### 1. Field Polygon Visualization
The `visualize_field_polygon()` method creates detailed images showing:
- **Field boundary** with light green fill and dark green border
- **Vertex coordinates** as red dots with coordinate labels
- **Area calculation** using shoelace formula
- **Bounding box information** 
- **Timestamp** for unique filenames

### 2. Automatic Visualization Calls
The system automatically generates visualization images for different field sources:

#### User-Provided Fields (`use_user_field=true`)
```
File: user_provided_field_{timestamp}.png
Log: "Using user-provided field: X vertices, area ≈ Y.Y m²"
```

#### Global Free Space (`use_user_field=false`)
```  
File: global_free_space_{timestamp}.png
Log: "Using global free space: X vertices, area ≈ Y.Y m²"
```

#### Cropped Free Space (fallback)
```
File: cropped_free_space_{timestamp}.png  
Log: "Using cropped free space: X vertices, area ≈ Y.Y m²"
```

## Image Features

### Visual Elements
- **Light green fill**: Shows the navigable field area
- **Dark green border**: Defines the exact field boundary  
- **Red dots**: Mark polygon vertices
- **Coordinate labels**: Show world coordinates (x, y) in meters
- **Title bar**: Shows polygon info and statistics
- **Area calculation**: Computed using precise shoelace formula

### Image Specifications  
- **Max resolution**: 800x600 pixels for reasonable file sizes
- **Auto-scaling**: Maintains aspect ratio and fits field bounds
- **Coordinate system**: World coordinates in meters (map frame)
- **File format**: PNG with timestamp for uniqueness

### Information Displayed
- Total number of vertices
- Calculated area in square meters  
- Bounding box coordinates (min/max x,y)
- First 8 vertex coordinates (to avoid visual clutter)

## Usage

### For Debugging Coverage Issues
1. **Compare field sources**: See difference between user fields vs. detected free space
2. **Verify coordinates**: Check that field boundaries match expectations  
3. **Validate area calculations**: Ensure coverage planning gets reasonable field sizes
4. **Debug costmap processing**: Visualize results of free space detection

### Log Messages
The system logs detailed information for each field:
```
[INFO] Creating field visualization: 640x480 pixels, scale=12.34 px/m
[INFO] Using global free space: 23 vertices, area ≈ 156.7 m²
[INFO] Saved field visualization: global_free_space_1755597080.png
[INFO] Field area: 156.7 m², 23 vertices
```

## Files Generated

### Image Files (saved to working directory)
- `user_provided_field_{timestamp}.png` - User-supplied field polygon
- `global_free_space_{timestamp}.png` - Largest free space from costmap  
- `cropped_free_space_{timestamp}.png` - Free space cropped to user polygon
- `global_costmap_*.png` - Various costmap analysis images (existing)

### Integration with Existing Visualizations
The field visualization works alongside existing costmap images:
- `global_costmap.png` - Raw costmap visualization
- `global_costmap_polygons.png` - All detected free space regions  
- `global_costmap_indexed_free_contours.png` - Numbered free space areas
- `cropped_costmap_analysis.png` - User polygon cropping results

## Debugging Workflow

1. **Send coverage request** with `use_user_field=false`
2. **Check logs** for field selection decisions and area calculations
3. **Review generated images**:
   - Compare `global_free_space_*.png` with `global_costmap_polygons.png`  
   - Verify field boundaries make sense for coverage
   - Check if fallback to cropped space was needed
4. **Validate coverage planning** uses appropriate field boundaries

## Benefits

- **Visual verification** of field polygon processing
- **Debug costmap analysis** effectiveness  
- **Compare field selection strategies** (global vs. cropped)
- **Validate coordinate transformations** from costmap to world frame
- **Troubleshoot coverage planning** with concrete visual evidence

This visualization system provides essential debugging tools for understanding how the coverage planner selects and processes field boundaries from costmap data.