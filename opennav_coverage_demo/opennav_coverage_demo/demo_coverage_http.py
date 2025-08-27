#! /usr/bin/env python3
# Copyright 2023 Open Navigation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from enum import Enum
import time
import threading
import json
import logging
import os
import requests
import yaml
from flask import Flask, request, jsonify, Response

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point32, Polygon
from lifecycle_msgs.srv import GetState
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rcl_interfaces.msg import Parameter
from rcl_interfaces.srv import SetParametersAtomically
import math

import cv2
import numpy as np
import fields2cover as f2c
from geometry_msgs.msg import Point32

# from opennav_coverage_demo.robot_nagivator import BasicNavigator
# 配置日志记录
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')


class CoverageNavigatorTester(Node):
    # 这里保留您现有的CoverageNavigatorTester类的代码
    def __init__(self):
        super().__init__(node_name='coverage_navigator_tester')
        self.current_costmap = None  # Initialize costmap storage   
    
    def call_get_costmap_service(self):
        try:
            url = "http://127.0.0.1:1234/get_global_costmap"
            self.get_logger().info('Calling GetCostmap service...')
            res = requests.get(url)
            if res.status_code == 200:
                self.get_logger().info('GetCostmap service call succeeded.')
                data = res.json()
                # Store the full response for processing
                self.current_costmap = data
                return True
            else:
                self.get_logger().error(f'GetCostmap service call failed with status code: {res.status_code}')
                return False
                  
        except Exception as e:
            self.get_logger().error(f'Error calling service: {str(e)}')
            return False

    def pixel_to_world(self, pixel_x, pixel_y, resolution, origin_x, origin_y):
        """
        Convert pixel coordinates to world coordinates
        
        Args:
            pixel_x: Column in image (x-axis in image coordinates)
            pixel_y: Row in image (y-axis in image coordinates)
            resolution: Meters per pixel
            origin_x: World X coordinate of pixel (0,0)
            origin_y: World Y coordinate of pixel (0,0)
            
        Returns:
            (world_x, world_y): World coordinates in meters
        """
        world_x = origin_x + (pixel_x * resolution)
        world_y = origin_y + (pixel_y * resolution)
        return world_x, world_y

    def convert_contour_to_world(self, contour, resolution, origin_x, origin_y):
        """
        Convert a single contour from pixel coordinates to world coordinates
        
        Args:
            contour: OpenCV contour in pixel coordinates
            resolution: Meters per pixel
            origin_x, origin_y: World coordinates of pixel (0,0)
            
        Returns:
            world_contour: List of (x, y) tuples in world coordinates
        """
        world_contour = []
        for point in contour:
            pixel_x, pixel_y = point[0][0], point[0][1]  # OpenCV contour format
            world_x, world_y = self.pixel_to_world(pixel_x, pixel_y, resolution, origin_x, origin_y)
            world_contour.append([world_x, world_y])
        return world_contour

    def process_costmap(self, msg):
        free_contours_world = []  # Initialize at the start
        try:
            # Debug: Print the structure of the received message
            self.get_logger().info(f"Costmap message type: {type(msg)}")
            self.get_logger().info(f"Costmap keys: {list(msg.keys()) if isinstance(msg, dict) else 'Not a dict'}")
            
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
            
            # Check if data is in the expected format
            if 'data' not in actual_costmap_data or 'metadata' not in actual_costmap_data:
                self.get_logger().error("Invalid costmap message format - missing 'data' or 'metadata' keys")
                self.get_logger().error(f"Available keys in costmap data: {list(actual_costmap_data.keys())}")
                return []
            
            # Use the actual costmap data for processing
            msg = actual_costmap_data
            
            costmap_data = msg['data']
            self.get_logger().info(f"Data type: {type(costmap_data)}, length: {len(costmap_data) if hasattr(costmap_data, '__len__') else 'Unknown'}")
            
            # Log first few elements for debugging
            if hasattr(costmap_data, '__len__') and len(costmap_data) > 0:
                sample_size = min(3, len(costmap_data))
                self.get_logger().info(f"First {sample_size} data elements: {costmap_data[:sample_size]}")
            
            # Handle different data formats
            if isinstance(costmap_data, list):
                # If data is a list of numbers
                try:
                    data = np.array(costmap_data, dtype=np.uint8)
                except (ValueError, TypeError) as e:
                    self.get_logger().error(f"Cannot convert costmap data to numpy array: {e}")
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
                    else:
                        return []
            else:
                self.get_logger().error(f"Unexpected data type: {type(costmap_data)}")
                return []
            
            # Reshape the data
            try:
                data = data.reshape((msg['metadata']['size_y'], msg['metadata']['size_x']))
            except ValueError as e:
                self.get_logger().error(f"Cannot reshape data: {e}")
                self.get_logger().error(f"Data shape: {data.shape}, Expected: ({msg['metadata']['size_y']}, {msg['metadata']['size_x']})")
                return []
            
            # Extract metadata for coordinate conversion
            resolution = msg['metadata']['resolution']
            origin_x = msg['metadata']['origin']['position']['x']
            origin_y = msg['metadata']['origin']['position']['y']

            self.get_logger().info(f'Costmap size: {msg["metadata"]["size_x"]} x {msg["metadata"]["size_y"]}')
            self.get_logger().info(f'Costmap resolution: {resolution} m/cell')
            self.get_logger().info(f'Costmap origin: ({origin_x}, {origin_y}) meters')

            # Create a uint8 image for visualization
            # Costmap values: 0 (free) -> 255 (occupied), so we need to invert for visualization
            image = np.zeros_like(data, dtype=np.uint8)
            
            # For costmap: 0 = free, 255 = occupied
            # For visualization: 0 = black (occupied), 255 = white (free)
            image = 255 - data  # Invert the costmap for better visualization
            
            # Alternative: Create a more detailed mapping if needed
            # image = np.zeros_like(data, dtype=np.uint8)
            # image[data == 0] = 255      # Free space -> white
            # image[data == 255] = 0      # Occupied -> black
            # image[(data > 0) & (data < 255)] = 255 - data[(data > 0) & (data < 255)]  # Interpolate

            # Save original costmap
            filename = 'global_costmap.png'
            cv2.imwrite(filename, image)
            self.get_logger().info(f'Saved costmap to {filename}')

            # Create better binary images for polygon detection
            # For costmap data (0-255), we need different thresholds
            
            # Free space: cells with low cost (0-50)
            free_space_binary = np.zeros_like(data, dtype=np.uint8)
            free_space_binary[data <= 50] = 255
            
            # Navigable space: cells with cost <= 100
            navigable_binary = np.zeros_like(data, dtype=np.uint8)
            navigable_binary[data <= 100] = 255
            
            # Obstacle boundaries: cells with cost >= 200
            obstacle_binary = np.zeros_like(data, dtype=np.uint8)
            obstacle_binary[data >= 200] = 255
            
            # Occupied space: cells with cost >= 250 (nearly occupied)
            occupied_binary = np.zeros_like(data, dtype=np.uint8)
            occupied_binary[data >= 250] = 255
            
            # Apply morphological operations to clean up the binary images
            kernel = np.ones((3,3), np.uint8)
            free_space_binary = cv2.morphologyEx(free_space_binary, cv2.MORPH_CLOSE, kernel)
            navigable_binary = cv2.morphologyEx(navigable_binary, cv2.MORPH_CLOSE, kernel)
            obstacle_binary = cv2.morphologyEx(obstacle_binary, cv2.MORPH_CLOSE, kernel)
            occupied_binary = cv2.morphologyEx(occupied_binary, cv2.MORPH_CLOSE, kernel)
            
            # Create a colored image for polygon visualization
            polygon_image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
            
            # Find and draw contours with different parameters for better fitting
            
            # 1. Free space polygons (low cost areas) with index labels
            free_contours, _ = cv2.findContours(free_space_binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Convert free contours to world coordinates
            free_contours_world = []
            free_contour_index = 0
            
            for contour in free_contours:
                if cv2.contourArea(contour) > 700:  # Reduced threshold
                    # Convert contour to world coordinates
                    world_contour = self.convert_contour_to_world(contour, resolution, origin_x, origin_y)
                    free_contours_world.append({
                        'index': free_contour_index,
                        'area_pixels': cv2.contourArea(contour),
                        'area_meters_squared': cv2.contourArea(contour) * (resolution ** 2),
                        'pixel_coordinates': contour.tolist(),
                        'world_coordinates': world_contour
                    })
                    
                    # Use more precise approximation for visualization
                    epsilon = 0.005 * cv2.arcLength(contour, True)  # Much more precise
                    approx = cv2.approxPolyDP(contour, epsilon, True)
                    cv2.drawContours(polygon_image, [approx], -1, (0, 255, 0), 1)  # Green, thinner line
                    
                    # Calculate centroid for text placement
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        
                        # Convert centroid to world coordinates
                        world_cx, world_cy = self.pixel_to_world(cx, cy, resolution, origin_x, origin_y)
                        
                        # Draw index number at centroid
                        cv2.putText(polygon_image, str(free_contour_index), (cx, cy), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)  # White text with border
                        cv2.putText(polygon_image, str(free_contour_index), (cx, cy), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 1)  # Black text
                        
                        # Log information about this contour
                        self.get_logger().info(f'Free contour {free_contour_index}: '
                                             f'Area={cv2.contourArea(contour):.1f} pixels '
                                             f'({cv2.contourArea(contour) * (resolution ** 2):.2f} m²), '
                                             f'Centroid=({world_cx:.2f}, {world_cy:.2f}) m')
                        
                        free_contour_index += 1
            
            # Save free contours as JSON file with world coordinates
            with open('free_contours_world_coordinates.json', 'w') as f:
                json.dump(free_contours_world, f, indent=2)
            self.get_logger().info(f'Saved {len(free_contours_world)} free contours with world coordinates to free_contours_world_coordinates.json')
            
            # 2. Navigable space polygons (broader navigable area)
            navigable_contours, _ = cv2.findContours(navigable_binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in navigable_contours:
                if cv2.contourArea(contour) > 100:
                    epsilon = 0.008 * cv2.arcLength(contour, True)
                    approx = cv2.approxPolyDP(contour, epsilon, True)
                    cv2.drawContours(polygon_image, [approx], -1, (0, 255, 255), 1)  # Yellow
            
            # 3. Obstacle boundaries
            obstacle_contours, _ = cv2.findContours(obstacle_binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in obstacle_contours:
                if cv2.contourArea(contour) > 25:  # Even smaller threshold for obstacles
                    epsilon = 0.005 * cv2.arcLength(contour, True)
                    approx = cv2.approxPolyDP(contour, epsilon, True)
                    cv2.drawContours(polygon_image, [approx], -1, (0, 0, 255), 1)  # Red
            
            # 4. Occupied space polygons (high cost areas)
            occupied_contours, _ = cv2.findContours(occupied_binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in occupied_contours:
                if cv2.contourArea(contour) > 10:  # Very small threshold for precise obstacles
                    epsilon = 0.003 * cv2.arcLength(contour, True)  # Most precise
                    approx = cv2.approxPolyDP(contour, epsilon, True)
                    cv2.drawContours(polygon_image, [approx], -1, (255, 0, 0), 2)  # Blue, thicker line
            
            # Save polygon image
            polygon_filename = 'global_costmap_polygons.png'
            cv2.imwrite(polygon_filename, polygon_image)
            self.get_logger().info(f'Saved polygon visualization to {polygon_filename}')
            
            # Create a detailed version with just the most important polygons and indexed free contours
            detailed_polygon_image = np.ones_like(polygon_image) * 255  # White background
            
            # Draw only the most precise contours with indices
            free_contour_index = 0
            for contour in free_contours:
                if cv2.contourArea(contour) > 700:
                    # Use the original contour points for maximum precision
                    cv2.drawContours(detailed_polygon_image, [contour], -1, (0, 200, 0), 2)
                    
                    # Calculate centroid for text placement
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        
                        # Draw index number at centroid (larger font for white background)
                        cv2.putText(detailed_polygon_image, str(free_contour_index), (cx, cy), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 0, 0), 3)  # Red text with thick border
                        cv2.putText(detailed_polygon_image, str(free_contour_index), (cx, cy), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 0), 2)  # Black text
                        
                        free_contour_index += 1
            
            for contour in occupied_contours:
                if cv2.contourArea(contour) > 25:
                    cv2.drawContours(detailed_polygon_image, [contour], -1, (0, 0, 200), 1)
            
            detailed_filename = 'global_costmap_detailed_polygons.png'
            cv2.imwrite(detailed_filename, detailed_polygon_image)
            self.get_logger().info(f'Saved detailed polygon visualization to {detailed_filename}')
            
            # Create a version with original contours (no approximation) with indices
            precise_polygon_image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
            
            # Draw exact contours without approximation
            cv2.drawContours(precise_polygon_image, free_contours, -1, (0, 255, 0), 1)
            cv2.drawContours(precise_polygon_image, occupied_contours, -1, (0, 0, 255), 1)
            
            # Add indices to free contours
            free_contour_index = 0
            for contour in free_contours:
                if cv2.contourArea(contour) > 50:
                    # Calculate centroid for text placement
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        
                        # Draw index number at centroid
                        cv2.putText(precise_polygon_image, str(free_contour_index), (cx, cy), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)  # White text with border
                        cv2.putText(precise_polygon_image, str(free_contour_index), (cx, cy), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)  # Black text
                        
                        free_contour_index += 1
            
            precise_filename = 'global_costmap_precise_contours.png'
            cv2.imwrite(precise_filename, precise_polygon_image)
            self.get_logger().info(f'Saved precise contour visualization to {precise_filename}')
            
            # Create a special version with ONLY indexed free contours for clarity
            indexed_free_image = np.ones_like(polygon_image) * 255  # White background
            
            free_contour_index = 0
            for contour in free_contours:
                if cv2.contourArea(contour) > 50:
                    # Draw contour in green
                    cv2.drawContours(indexed_free_image, [contour], -1, (0, 180, 0), 2)
                    
                    # Calculate centroid for text placement
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        
                        # Draw index number at centroid (large, clear text)
                        cv2.putText(indexed_free_image, str(free_contour_index), (cx, cy), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 4)  # Red text with thick border
                        cv2.putText(indexed_free_image, str(free_contour_index), (cx, cy), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 2)  # White text
                        
                        free_contour_index += 1
            
            indexed_filename = 'global_costmap_indexed_free_contours.png'
            cv2.imwrite(indexed_filename, indexed_free_image)
            self.get_logger().info(f'Saved indexed free contours to {indexed_filename}')
            
            # Log detailed statistics
            self.get_logger().info(f'Found {len(free_contours)} free space regions')
            self.get_logger().info(f'Found {len(navigable_contours)} navigable regions')
            self.get_logger().info(f'Found {len(obstacle_contours)} obstacle boundary regions')
            self.get_logger().info(f'Found {len(occupied_contours)} occupied regions')
            
            # Log unique costmap values for debugging
            unique_values = np.unique(data)
            self.get_logger().info(f'Unique costmap values: {unique_values}')
            
            return free_contours_world
            
        except Exception as e:
            self.get_logger().error(f'Error processing costmap: {str(e)}')
            return []

    def visualize_field_polygon(self, field_coords, filename_prefix="selected_field", obstacles_polygons=None):
        """
        Visualize the field polygon and obstacles for debugging purposes.
        
        Args:
            field_coords: List of [x, y] coordinates defining the field boundary
            filename_prefix: Prefix for the saved image file
            obstacles_polygons: List of obstacle polygons, each as a list of [x, y] coordinates
        """
        try:
            if not field_coords or len(field_coords) < 3:
                self.get_logger().warn("Invalid field coordinates for visualization")
                return
            
            # Convert to numpy array for easier processing
            coords = np.array(field_coords)
            
            # Calculate bounds with some padding
            min_x, min_y = coords.min(axis=0)
            max_x, max_y = coords.max(axis=0)
            padding = max(max_x - min_x, max_y - min_y) * 0.1  # 10% padding
            
            # Create image dimensions (scale to reasonable pixel size)
            width_m = max_x - min_x + 2 * padding
            height_m = max_y - min_y + 2 * padding
            scale = min(800 / width_m, 600 / height_m)  # Max 800x600 image
            
            img_width = int(width_m * scale)
            img_height = int(height_m * scale)
            
            self.get_logger().info(f"Creating field visualization: {img_width}x{img_height} pixels, scale={scale:.2f} px/m")
            
            # Create white background image
            image = np.ones((img_height, img_width, 3), dtype=np.uint8) * 255
            
            # Convert world coordinates to image coordinates
            def world_to_image(x, y):
                img_x = int((x - (min_x - padding)) * scale)
                img_y = int((max_y + padding - y) * scale)  # Flip Y axis
                return img_x, img_y
            
            # Convert field coordinates to image coordinates
            image_coords = []
            for coord in field_coords:
                img_x, img_y = world_to_image(coord[0], coord[1])
                image_coords.append([img_x, img_y])
            
            # Draw field polygon
            image_coords = np.array(image_coords, dtype=np.int32)
            cv2.fillPoly(image, [image_coords], (200, 255, 200))  # Light green fill
            cv2.polylines(image, [image_coords], True, (0, 150, 0), 3)  # Dark green border
            
            # Draw obstacles if provided
            if obstacles_polygons:
                self.get_logger().info(f"Drawing {len(obstacles_polygons)} obstacles")
                for i, obstacle in enumerate(obstacles_polygons):
                    # Convert obstacle coordinates to image coordinates
                    obstacle_image_coords = []
                    for coord in obstacle:
                        img_x, img_y = world_to_image(coord[0], coord[1])
                        obstacle_image_coords.append([img_x, img_y])
                    
                    obstacle_coords = np.array(obstacle_image_coords, dtype=np.int32)
                    if len(obstacle_coords) >= 3:  # Valid polygon
                        cv2.fillPoly(image, [obstacle_coords], (100, 100, 100))  # Dark gray fill for obstacles
                        cv2.polylines(image, [obstacle_coords], True, (50, 50, 50), 2)  # Darker gray border
                        
                        # Add obstacle index
                        if len(obstacle_coords) > 0:
                            center_x = int(np.mean(obstacle_coords[:, 0]))
                            center_y = int(np.mean(obstacle_coords[:, 1]))
                            cv2.putText(image, f"Obs{i}", (center_x - 10, center_y + 5), 
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
            
            # Add coordinate labels at vertices
            for i, coord in enumerate(field_coords[:8]):  # Limit to first 8 points to avoid clutter
                img_x, img_y = world_to_image(coord[0], coord[1])
                # Draw point
                cv2.circle(image, (img_x, img_y), 5, (255, 0, 0), -1)  # Red dot
                # Add coordinate text
                text = f"({coord[0]:.1f},{coord[1]:.1f})"
                font_scale = 0.4
                thickness = 1
                text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness)[0]
                # Position text to avoid overlap
                text_x = img_x + 8 if img_x < img_width - text_size[0] - 8 else img_x - text_size[0] - 8
                text_y = img_y - 8 if img_y > text_size[1] + 8 else img_y + text_size[1] + 8
                # White background for text
                cv2.rectangle(image, (text_x - 2, text_y - text_size[1] - 2), 
                            (text_x + text_size[0] + 2, text_y + 2), (255, 255, 255), -1)
                cv2.putText(image, text, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 
                           font_scale, (0, 0, 0), thickness)
            
            # Add title and info
            obstacle_count = len(obstacles_polygons) if obstacles_polygons else 0
            title = f"Field Polygon - {len(field_coords)} points, {obstacle_count} obstacles"
            area_m2 = self.calculate_polygon_area(field_coords)
            subtitle = f"Area: {area_m2:.1f} m² | Bounds: ({min_x:.1f},{min_y:.1f}) to ({max_x:.1f},{max_y:.1f})"
            
            cv2.putText(image, title, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
            cv2.putText(image, subtitle, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
            
            # Save image
            filename = f"{filename_prefix}.png"
            cv2.imwrite(filename, image)
            self.get_logger().info(f"Saved field visualization: {filename}")
            self.get_logger().info(f"Field area: {area_m2:.1f} m², {len(field_coords)} vertices")
            
        except Exception as e:
            self.get_logger().error(f"Error visualizing field polygon: {str(e)}")

    def calculate_polygon_area(self, coords):
        """Calculate the area of a polygon using the shoelace formula."""
        if len(coords) < 3:
            return 0.0
        
        area = 0.0
        n = len(coords)
        for i in range(n):
            j = (i + 1) % n
            area += coords[i][0] * coords[j][1]
            area -= coords[j][0] * coords[i][1]
        return abs(area) / 2.0

    def crop_and_find_free_space(self, user_polygon):
        """
        Crop the global costmap to the user's polygon and find the largest free space within it.
        
        Args:
            user_polygon: List of [x, y] coordinates defining the field boundary
            
        Returns:
            Tuple of (free_space_polygon, obstacles_polygons) where:
            - free_space_polygon: List of [x, y] coordinates of the largest free space polygon within the field, or None if none found
            - obstacles_polygons: List of obstacle polygons, each as a list of [x, y] coordinates
        """
        try:
            if not hasattr(self, 'current_costmap') or not self.current_costmap:
                self.get_logger().error('No costmap available. Call get costmap service first.')
                return None, []
            
            if not user_polygon or len(user_polygon) < 3:
                self.get_logger().error('Invalid user polygon provided')
                return None, []
            
            costmap_msg = self.current_costmap
            self.get_logger().info(f'Cropping costmap to user polygon: {user_polygon}')
            
            # Handle HTTP service wrapper format
            actual_costmap_data = costmap_msg
            if 'code' in costmap_msg and 'data' in costmap_msg:
                if costmap_msg['code'] == 0:
                    actual_costmap_data = costmap_msg['data']
                else:
                    self.get_logger().error(f"Costmap data has error code: {costmap_msg['code']}")
                    return None, []
            
            # Extract costmap data
            costmap_data = actual_costmap_data['data']
            # Handle different data formats (similar to process_costmap)
            if isinstance(costmap_data, list) and len(costmap_data) > 0:
                if isinstance(costmap_data[0], dict):
                    # Try to extract from dict format
                    for key in ['value', 'cost', 'data', 'cell_value']:
                        if key in costmap_data[0]:
                            try:
                                data = np.array([cell[key] for cell in costmap_data], dtype=np.uint8)
                                break
                            except Exception:
                                continue
                    else:
                        self.get_logger().error("Could not extract numerical data from dict format")
                        return None, []
                else:
                    data = np.array(costmap_data, dtype=np.uint8)
            else:
                self.get_logger().error(f"Unexpected costmap data format: {type(costmap_data)}")
                return None, []
            
            data = data.reshape((actual_costmap_data['metadata']['size_y'], actual_costmap_data['metadata']['size_x']))
            resolution = actual_costmap_data['metadata']['resolution']
            origin_x = actual_costmap_data['metadata']['origin']['position']['x']
            origin_y = actual_costmap_data['metadata']['origin']['position']['y']

            self.get_logger().info(f'Costmap resolution: {resolution} m/cell, origin: ({origin_x}, {origin_y})')
            
            # Convert user polygon to pixel coordinates
            user_polygon_pixels = []
            for point in user_polygon:
                pixel_x = int((point[0] - origin_x) / resolution)
                pixel_y = int((point[1] - origin_y) / resolution)
                # Clamp to valid pixel range
                pixel_x = max(0, min(pixel_x, actual_costmap_data['metadata']['size_x'] - 1))
                pixel_y = max(0, min(pixel_y, actual_costmap_data['metadata']['size_y'] - 1))
                user_polygon_pixels.append([pixel_x, pixel_y])
            
            self.get_logger().info(f'User polygon in pixels: {user_polygon_pixels}')
            
            # Create a mask for the user polygon
            mask = np.zeros((actual_costmap_data['metadata']['size_y'], actual_costmap_data['metadata']['size_x']), dtype=np.uint8)
            polygon_contour = np.array(user_polygon_pixels, dtype=np.int32)
            cv2.fillPoly(mask, [polygon_contour], 255)
            
            # Create binary image of free space (cost <= 50)
            free_space_binary = np.zeros_like(data, dtype=np.uint8)
            free_space_binary[data <= 50] = 255
            
            # Apply user polygon mask to only consider areas within the polygon
            cropped_free_space = cv2.bitwise_and(free_space_binary, mask)
            
            # Clean up the binary image
            kernel = np.ones((3,3), np.uint8)
            cropped_free_space = cv2.morphologyEx(cropped_free_space, cv2.MORPH_CLOSE, kernel)
            cropped_free_space = cv2.morphologyEx(cropped_free_space, cv2.MORPH_OPEN, kernel)
            
            # Find contours in the cropped free space
            contours, _ = cv2.findContours(cropped_free_space, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if not contours:
                self.get_logger().warn('No free space contours found within the user polygon')
                return None, []
            
            # Find the largest contour
            largest_contour = None
            largest_area_pixels = 0
            
            for contour in contours:
                area_pixels = cv2.contourArea(contour)
                if area_pixels > largest_area_pixels and area_pixels > 100:  # Minimum size threshold
                    largest_area_pixels = area_pixels
                    largest_contour = contour
            
            if largest_contour is None:
                self.get_logger().warn('No sufficiently large free space found within the user polygon')
                return None, []
            
            # Simplify the polygon to reduce number of points
            epsilon = 0.01 * cv2.arcLength(largest_contour, True)
            simplified_contour = cv2.approxPolyDP(largest_contour, epsilon, True)
            
            # Convert the simplified contour back to world coordinates
            world_polygon = []
            for point in simplified_contour:
                pixel_x, pixel_y = point[0][0], point[0][1]
                world_x = origin_x + (pixel_x * resolution)
                world_y = origin_y + (pixel_y * resolution)
                world_polygon.append([world_x, world_y])
            
            largest_area_meters = largest_area_pixels * (resolution ** 2)
            self.get_logger().info(f'Found largest free space: area={largest_area_meters:.2f} m², points={len(world_polygon)}')
            self.get_logger().info(f'Free space polygon: {world_polygon[:5]}...')  # First 5 points
            
            # Detect obstacles within the cropped area
            obstacle_binary = np.zeros_like(data, dtype=np.uint8)
            obstacle_binary[data >= 200] = 255  # High cost areas are obstacles
            
            # Apply user polygon mask to only consider obstacles within the polygon
            cropped_obstacles = cv2.bitwise_and(obstacle_binary, mask)
            
            # Clean up the obstacle binary image
            cropped_obstacles = cv2.morphologyEx(cropped_obstacles, cv2.MORPH_CLOSE, kernel)
            cropped_obstacles = cv2.morphologyEx(cropped_obstacles, cv2.MORPH_OPEN, kernel)
            
            # Find obstacle contours
            obstacle_contours, _ = cv2.findContours(cropped_obstacles, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Convert obstacle contours to world coordinates
            obstacle_polygons = []
            for obstacle_contour in obstacle_contours:
                if cv2.contourArea(obstacle_contour) > 25:  # Minimum obstacle size threshold
                    # Simplify the obstacle polygon
                    epsilon = 0.02 * cv2.arcLength(obstacle_contour, True)
                    simplified_obstacle = cv2.approxPolyDP(obstacle_contour, epsilon, True)
                    
                    # Convert to world coordinates
                    obstacle_world = []
                    for point in simplified_obstacle:
                        pixel_x, pixel_y = point[0][0], point[0][1]
                        world_x = origin_x + (pixel_x * resolution)
                        world_y = origin_y + (pixel_y * resolution)
                        obstacle_world.append([world_x, world_y])
                    
                    if len(obstacle_world) >= 3:  # Valid polygon needs at least 3 points
                        obstacle_polygons.append(obstacle_world)
            
            self.get_logger().info(f'Found {len(obstacle_polygons)} obstacles within the cropped area')
            
            # Save debug image with obstacles
            debug_image = cv2.cvtColor(cropped_free_space, cv2.COLOR_GRAY2BGR)
            cv2.drawContours(debug_image, [largest_contour], -1, (0, 255, 0), 2)  # Green for free space
            cv2.drawContours(debug_image, [polygon_contour], -1, (255, 0, 0), 2)  # Blue for user polygon
            cv2.drawContours(debug_image, obstacle_contours, -1, (0, 0, 255), 2)  # Red for obstacles
            cv2.imwrite('cropped_costmap_analysis.png', debug_image)
            self.get_logger().info('Saved debug image: cropped_costmap_analysis.png')
            
            return world_polygon, obstacle_polygons
            
        except Exception as e:
            self.get_logger().error(f'Error cropping costmap and finding free space: {str(e)}')
            return None, []




    def destroy_node(self):
        super().destroy_node()

    def sendTaskRequest(self, waypoints, wait_at_first_waypoint=False, path_planner='straight'):
        flask_ros_url = 'http://127.0.0.1:1234'
        ros_data = {
            "wps": [],
            "is_repeat": False,
            "task_uid": "test_task",
            "use_path_map": False
            }
        index = 0
        for waypoint in waypoints:
            actions = []
            precise_xy = 0.05
            if index == 0 and wait_at_first_waypoint:
                actions.append({
                    'name': 'wait_for_command',
                    'id': 13
                })
                precise_xy = 0.12
            wp = {
                'pose': {
                    'position': {
                        'x': waypoint.x,
                        'y': waypoint.y
                    },
                    'orientation': {
                        'z': 0.0,
                        'w': 1.0
                    }
                },
                'is_dest': True,
                'precise_xy': precise_xy,
                'precise_rad': 6.28,
                'nav_type': path_planner,
                'actions': actions,
                'is_reverse': False,
                'inflation_radius': 1.1,
                'uid': f'wp_{index}',
            }
            ros_data['wps'].append(wp)
            index += 1
        url = "{}/execute_task".format(flask_ros_url)
        
        response = requests.post(url, json=ros_data)
        return response.json()['code']

    def startup(self, node_name='bt_navigator'):
        # Waits for the node within the tester namespace to become active
        print(f'Waiting for {node_name} to become active..')
        node_service = f'{node_name}/get_state'
        state_client = self.create_client(GetState, node_service)
        while not state_client.wait_for_service(timeout_sec=1.0):
            print(f'{node_service} service not available, waiting...')

        req = GetState.Request()
        state = 'unknown'
        while state != 'active':
            print(f'Getting {node_name} state...')
            future = state_client.call_async(req)
            # Use a timeout and check if done rather than spin_until_future_complete
            timeout_count = 0
            while not future.done() and timeout_count < 50:  # 5 second timeout
                time.sleep(0.1)
                timeout_count += 1
            
            if future.done() and future.result() is not None:
                state = future.result().current_state.label
                print(f'Result of get_state: {state}')
            else:
                print(f'Timeout getting {node_name} state, retrying...')
            time.sleep(2)
        return

    def toPolygon(self, field, rings):
        points = []
        for coord in field:
            p = f2c.Point(coord[0], coord[1])
            points.append(p)
        cell = f2c.Cell(f2c.LinearRing(f2c.VectorPoint(points)))
        for ring in rings:
            r = f2c.LinearRing()
            for coord in ring:
                p = f2c.Point(coord[0], coord[1])
                r.addPoint(p)
            cell.addRing(r)
        cells = f2c.Cells(cell)
        return cells
    
    def generate_coverage_path(self, 
                               cells,
                               robot_width, 
                               robot_op_width,
                               robot_min_turning_radius=1e-8,
                               robot_max_diff_curvature=1e8,
                               robot_cruise_vel=0.5,
                               robot_turn_vel=0.5,
                               use_decomposition=True,
                               decomposition_type='Trapezoidal',
                               headland_width=1.0,
                               swath_mode='BRUTE_FORCE',
                               swath_step_angle=0.1,
                               swath_obj="LENGTH",
                               swath_allow_overlap=False,
                               swath_set_angle=0.0,
                               route_mode="AUTO",
                               route_spiral=2,
                               route_custom_order=[]
                               ):
        robot = f2c.Robot(robot_width, robot_op_width)
        robot.setMinTurningRadius(robot_min_turning_radius)
        robot.setMaxDiffCurv(robot_max_diff_curvature)
        robot.setCruiseVel(robot_cruise_vel)
        robot.setTurnVel(robot_turn_vel)
        const_hl = f2c.HG_Const_gen()
        
        obj = f2c.OBJ_NSwathModified()

        # Step 1: Decomposition
        if use_decomposition:
            if decomposition_type == 'Boustrophedon':
                decomp = f2c.DECOMP_BoustrophedonDecomp()
            else:
                decomp = f2c.DECOMP_TrapezoidalDecomp()
                decomp.setSplitAngle(0.5*math.pi)
            cells = decomp.decompose(cells) 

        # Step 2: Generate headlands 
        no_hl_decomp = const_hl.generateHeadlands(cells, headland_width)
        rem_area = f2c.OBJ_RemArea()
        self.get_logger().info("The remaining area is {}, and with sign is {}".format(rem_area.computeCost(cells, no_hl_decomp), rem_area.computeCostWithMinimizingSign(cells, no_hl_decomp)))

        # Step 3: Generate swaths
        bf = f2c.SG_BruteForce()
        bf.setAllowOverlap(swath_allow_overlap)
        coverage_width = robot.getCovWidth()
        if swath_mode == 'BRUTE_FORCE':
            bf.setStepAngle(swath_step_angle)
            if swath_obj == 'LENGTH':
                obj = f2c.OBJ_SwathLength()
            elif swath_obj == 'NUMBER':
                obj = f2c.OBJ_NSwath()
            elif swath_obj == 'NUMBER_MODIFIED':
                obj = f2c.OBJ_NSwathModified()
            elif swath_obj == 'COVERAGE':
                obj = f2c.OBJ_FieldCoverage()
       
            swaths = bf.generateBestSwaths(obj, coverage_width, no_hl_decomp)
        elif swath_mode == 'SET_ANGLE':
            swaths = bf.generateSwaths(swath_set_angle, coverage_width, no_hl_decomp)   
        self.get_logger().info(f'生成 {swaths.size()} 组覆盖路径')
        swath_length = f2c.OBJ_SwathLength()
        self.get_logger().info(f'覆盖路径总长度约为 {swath_length.computeCost(no_hl_decomp, swaths):.2f} 米')  

        # Step 4: Plan route
        if route_mode == "AUTO":    
            route_planner = f2c.RP_RoutePlannerBase()
            route = route_planner.genRoute(no_hl_decomp, swaths)
        elif route_mode == "BOUSTROPHEDON":
            route_planner = f2c.RP_Boustrophedon()
            route = route_planner.genSortedSwaths(swaths)
        elif route_mode == "SNAKE":
            route_planner = f2c.RP_Snake()
            route = route_planner.genSortedSwaths(swaths)
        elif route_mode == "SPIRAL":
            route_planner = f2c.RP_Spiral(route_spiral)
            route = route_planner.genSortedSwaths(swaths)
        elif route_mode == "CUSTOM":
            route_planner = f2c.RP_CustomOrder(route_custom_order)
            route = route_planner.genSortedSwaths(swaths)
        else:
            self.get_logger().error(f"未知的路线模式: {route_mode}, 使用自动模式")
            route_planner = f2c.RP_RoutePlannerBase()
            route = route_planner.genRoute(no_hl_decomp, swaths)
        length = route.length()
        self.get_logger().info(f'规划的路径总长度约为 {length:.2f} 米')
        vector_swaths = route.getVectorSwaths()
        size = vector_swaths.size()
        self.get_logger().info(f'路径包含 {size} 条覆盖路径段')
        self.current_waypoints = []
        for i in range(size):
            size_swaths = vector_swaths[i].size()
            swaths = vector_swaths[i]
            for j in range(size_swaths):
                swath = swaths.at(j)
                 # Log start and end points of each swath
                
                start_point = swath.startPoint()
                point = Point32(x=start_point.X(), y=start_point.Y(), z=0.0)
                self.current_waypoints.append(point)
                end_point = swath.endPoint()
                point = Point32(x=end_point.X(), y=end_point.Y(), z=0.0)
                self.current_waypoints.append(point)
                self.get_logger().info(f'Swath {i}-{j}: Start({start_point.X():.2f}, {start_point.Y():.2f}) End({end_point.X():.2f}, {end_point.Y():.2f})')
        # Step 5: Plan path



        f2c.Visualizer.figure()
        f2c.Visualizer.plot(cells)
        # f2c.Visualizer.plot(swaths)
        f2c.Visualizer.plot(route)

        f2c.Visualizer.save("Tutorial_image.png")




class CoverageNavigatorServer(CoverageNavigatorTester):
    """扩展CoverageNavigatorTester以提供HTTP API功能。"""
    
    def __init__(self):
        super().__init__()
        self.app = Flask(__name__)
        self.repeat_times = 1
        self.current_repeat = 0
        # self.robot_navigator = BasicNavigator()
        self.setup_routes()
        
    def setup_routes(self):
        """设置HTTP路由。"""
        @self.app.route('/navigate_coverage', methods=['POST'])
        def handle_navigate_coverage():
            try:
                logging.info(f"收到导航请求，内容类型: {request.content_type}")
                
                if not request.is_json:
                    logging.error(f"请求不是JSON格式: {request.data}")
                    return jsonify({"code": 1,"error": "请求必须是JSON格式"}), 400
                
                data = request.get_json(force=True)  # 使用force=True尝试强制解析JSON
                logging.info(f"解析的JSON数据: {data}")
                
                if 'field' not in data:
                    return jsonify({"code": 1,"error": "缺少'field'字段"}), 400
                    
                user_field = data['field']
                if not isinstance(user_field, list) or len(user_field) < 3:
                    return jsonify({"code": 1,"error": "'field'必须是至少包含3个坐标点的列表"}), 400
                
                use_user_field = True
                if 'use_user_field' in data:
                    use_user_field = data['use_user_field']
                field = None
                if use_user_field:
                    field = user_field
                    self.visualize_field_polygon(field, "user_provided_field")
                    self.get_logger().info(f"Using user-provided field: {len(field)} vertices, area ≈ {self.calculate_polygon_area(field):.1f} m²")
                else:
                    # Get the largest free space from costmap analysis
                    if self.call_get_costmap_service():
                        field, obstacles = self.crop_and_find_free_space(user_field)
                        if field:
                            self.visualize_field_polygon(field, "cropped_free_space", obstacles)
                            self.get_logger().info(f"Using cropped free space: {len(field)} vertices, area ≈ {self.calculate_polygon_area(field):.1f} m²")
                            self.get_logger().info(f"Found {len(obstacles)} obstacles within the field")
                            for i, obs in enumerate(obstacles):
                                self.get_logger().info(f"Obstacle {i}: {obs}")
                    
                if not field:
                    return jsonify({"code": 1,"error": "在用户指定区域内未找到可用的自由空间"}), 404
                
                # Ensure the polygon is closed (first point equals last point)
                if field[0] != field[-1]:
                    field.append(field[0])
                
                if "mode" in data:
                    mode = data['mode']
                    if mode not in ['SET_ANGLE', 'BRUTE_FORCE']:
                        return jsonify({"code": 1,"error": "'mode must be 'SET_ANGLE' or 'BRUTE_FORCE'"}), 400
                else:
                    mode = 'SET_ANGLE'
                
                if "best_angle" in data:
                    swath_angle = data['best_angle']
                    
                    if not isinstance(swath_angle, (int, float)):
                        return jsonify({"code": 1,"error": "'best_angle'必须是数字"}), 400
                    swath_angle = math.radians(swath_angle)  # 转换为弧度
                else:
                    swath_angle = 0.0

                if "step_angle" in data:
                    step_angle = data['step_angle']
                    if not isinstance(step_angle, (int, float)):
                        return jsonify({"code": 1,"error": "'step_angle'必须是数字"}), 400
                    step_angle = math.radians(step_angle)
                else:
                    step_angle = 0.2
                if "objective" in data:
                    objective = data['objective']
                    if objective not in ['LENGTH', 'NUMBER', 'COVERAGE']:
                        return jsonify({"code": 1,"error": "'objective'必须是'LENGTH', 'NUMBER'或'COVERAGE'"}), 400
                else:
                    objective = 'COVERAGE'
                
                if "repeat_times" in data:
                    repeat_times = data['repeat_times']
                    if not isinstance(repeat_times, int) or repeat_times < 1:
                        return jsonify({"code": 1,"error": "'repeat_times'必须是正整数"}), 400
                else:
                    repeat_times = 1


                cells = self.toPolygon(field, data['rings'])
                self.generate_coverage_path(
                    cells=cells,
                    robot_width= data.get('robot_width', 1.0),
                    robot_op_width= data.get('robot_op_width', 1.0),
                    robot_min_turning_radius= data.get('robot_min_turning_radius', 1e-8),
                    robot_max_diff_curvature= data.get('robot_max_diff_curvature', 1e8),
                    robot_cruise_vel= data.get('robot_cruise_vel', 0.5),
                    robot_turn_vel= data.get('robot_turn_vel', 0.5),
                    use_decomposition= data.get('use_decomposition', True),
                    decomposition_type= data.get('decomposition_type', 'Trapezoidal'),
                    headland_width= data.get('headland_width', 1.0),
                    swath_allow_overlap= data.get('swath_allow_overlap', False),
                    swath_mode= mode,
                    swath_set_angle= swath_angle,
                    swath_step_angle= step_angle,
                    swath_obj= objective,
                    route_mode= data.get('route_mode', 'AUTO'),
                    route_spiral= data.get('route_spiral', 2),
                    route_custom_order= data.get('route_custom_order', [])

                )

                    
                



                # for swaths in swaths_decomp:
                #     for swath in swaths:
                #         self.get_logger().info(f'Swath from ({swath.start.x:.2f}, {swath.start.y:.2f}) to ({swath.end.x:.2f}, {swath.end.y:.2f})')


                # no_hl_wo_decomp = const_hl.generateHeadlands(cells, 3.0 * r_w)

                    
                # 在新线程中启动导航任务
                # self.task_thread = threading.Thread(target=self._run_navigation_task, args=(field, swath_angle, repeat_times, mode, objective, step_angle))
                # self.task_thread.start()
                # path = self._run_navigation_task(field, swath_angle, repeat_times, mode, objective, step_angle)

                return jsonify({"code": 0, "path": None, "status": "导航任务已启动"}), 202
            except Exception as e:
                logging.exception("处理导航请求时出错:")
                return jsonify({"code": 1, "error": f"服务器处理请求时发生错误: {str(e)}"}), 500

        @self.app.route('/navigate_coverage', methods=['OPTIONS'])
        def handle_options():
            response = Response()
            response.headers.add("Access-Control-Allow-Origin", "*")
            response.headers.add("Access-Control-Allow-Headers", "Content-Type")
            response.headers.add("Access-Control-Allow-Methods", "POST, OPTIONS")
            return response


        @self.app.route('/start_navigation', methods=['POST'])
        def handle_start_navigation():
            data = request.get_json()
            wait_at_first_waypoint = data.get('wait_at_first_waypoint', False)
            path_planner = data.get('path_planner', 'straight')
            if self.current_waypoints is None or len(self.current_waypoints) == 0:
                return jsonify({
                    "code": 1,
                    "error": "没有可用的导航路径，请先调用 /navigate_coverage 接口"
                }), 400
            result = self.sendTaskRequest(self.current_waypoints, wait_at_first_waypoint=wait_at_first_waypoint, path_planner=path_planner)
            if result != 0:
                return jsonify({
                    "code": 1,
                    "error": "导航任务请求失败，可能是服务器错误或路径问题"
                }), 500
            self.current_waypoints = None  # 清除当前路径，防止重复提交
            return jsonify({
                "code": 0,
                "status": "导航任务已开始"
            }), 200



    def run_server(self, host='0.0.0.0', port=1235):  # 修改端口为1235
        """启动HTTP服务器。"""
        logging.info(f"开启HTTP服务器在 http://{host}:{port}/")
        # 启用CORS支持，设置线程模式
        self.app.run(host=host, port=port, debug=False, threaded=True)


def main():
    rclpy.init()
    
    # 创建ROS节点和服务器
    navigator_server = CoverageNavigatorServer()
    # navigator_server.startup()
    
    # Create executor for thread-safe ROS operations
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(navigator_server)
    
    # 在单独的线程中处理ROS循环，使用MultiThreadedExecutor
    ros_thread = threading.Thread(target=executor.spin)
    ros_thread.daemon = True
    ros_thread.start()
    
    # 启动HTTP服务器(在主线程中)
    try:
        navigator_server.run_server()
    except KeyboardInterrupt:
        logging.info("服务器正在关闭...")
    finally:
        executor.shutdown()
        navigator_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()