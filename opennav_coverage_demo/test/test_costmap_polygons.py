#!/usr/bin/env python3
# Copyright 2026 Open Navigation LLC
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

"""Exercise contour generation and HTTP rejection without starting ROS or moving a robot."""

import importlib.util
import json
import logging
from pathlib import Path
import unittest
from unittest import mock

import cv2
import numpy as np
from shapely.geometry import Polygon


FIXTURES = Path(__file__).parent / 'fixtures'
SQUARE = [[0, 0], [4, 0], [4, 4], [0, 4], [0, 0]]
BOWTIE = [[0, 0], [2, 2], [0, 2], [2, 0], [0, 0]]


class TestCostmapPolygons(unittest.TestCase):
    """Cover the production cropper, native conversion, and Flask request handler."""

    @classmethod
    def setUpClass(cls):
        """Load the source module with the normal ROS and Fields2Cover dependencies."""
        path = (Path(__file__).resolve().parents[1] / 'opennav_coverage_demo' /
                'demo_coverage_http.py')
        spec = importlib.util.spec_from_file_location('coverage_under_test', path)
        cls.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.module)
        cls.captured = json.loads((FIXTURES / 'invalid-polygon.json').read_text())

    def setUp(self):
        """Use the real methods without initializing a ROS node or writing images."""
        self.server = self.module.CoverageNavigatorTester.__new__(
            self.module.CoverageNavigatorTester)
        self.server.get_logger = lambda: logging.getLogger('polygon-regression')
        patch = mock.patch.object(self.module.cv2, 'imwrite', return_value=True)
        patch.start()
        self.addCleanup(patch.stop)

    def crop(self, mask):
        """Run the production raster-to-polygon path and check every returned ring."""
        height, width = mask.shape
        self.server.current_costmap = {'code': 0, 'data': {
            'data': np.where(mask > 0, 0, 254).astype(np.uint8).ravel().tolist(),
            'metadata': {
                'size_x': width, 'size_y': height, 'resolution': 1.0,
                'origin': {'position': {'x': 0.0, 'y': 0.0}},
            },
        }}
        field, obstacles = self.server.crop_and_find_free_space([
            [0, 0], [width - 1, 0], [width - 1, height - 1], [0, height - 1],
        ])
        self.assertIsNotNone(field)
        polygon = Polygon(field)
        self.assertTrue(polygon.is_valid)
        self.assertTrue(polygon.exterior.is_simple)
        self.assertGreater(polygon.area, 0)
        for obstacle in obstacles:
            self.assertTrue(Polygon(obstacle).is_valid)
        return polygon, obstacles

    def test_self_touching_raster(self):
        """The saved raster produced a self-intersecting polygon before the fix."""
        mask = cv2.imread(str(FIXTURES / 'crossing-regression-mask.png'), 0)
        self.assertIsNotNone(mask)
        self.crop(mask)

    def test_largest_disconnected_region(self):
        """Select the large component without joining it to a disconnected island."""
        mask = np.zeros((120, 160), np.uint8)
        cv2.rectangle(mask, (8, 8), (105, 105), 255, -1)
        cv2.rectangle(mask, (125, 15), (150, 40), 255, -1)
        cv2.rectangle(mask, (35, 35), (50, 55), 0, -1)
        polygon, obstacles = self.crop(mask)
        self.assertGreater(polygon.area, 8000)
        self.assertLess(polygon.bounds[2], 120)
        self.assertTrue(obstacles)

    def test_captured_contour_and_native_cost(self):
        """Replay the crash geometry through repair and the native crashing operation."""
        contour = np.asarray(self.captured['field'], np.float32).reshape(-1, 1, 2)
        parts = self.module._costmap_contour_polygons(contour, keep_inside=True)
        self.assertTrue(parts)
        self.assertTrue(all(p.is_valid and p.exterior.is_simple for p in parts))
        largest = max(parts, key=lambda part: part.area)
        cells = self.server.toPolygon(
            list(largest.exterior.coords),
            [list(ring.coords) for ring in largest.interiors])
        headlands = self.module.f2c.HG_Const_gen().generateHeadlands(cells, 0.8)
        cost = self.module.f2c.OBJ_RemArea().computeCost(cells, headlands)
        self.assertTrue(np.isfinite(cost))

    def test_conservative_simplification(self):
        """Keep narrow concavities without adding free space or shrinking obstacles."""
        contour = np.asarray([
            [0, 0], [20, 0], [20, 20], [11, 20],
            [11, 3], [9, 3], [9, 20], [0, 20],
        ], np.int32).reshape(-1, 1, 2)
        source = Polygon(contour[:, 0, :])
        for inside in (True, False):
            parts = self.module._costmap_contour_polygons(contour, keep_inside=inside)
            self.assertTrue(parts)
            for part in parts:
                self.assertTrue(part.is_valid)
                self.assertTrue(source.covers(part) if inside else part.covers(source))

    def test_degenerate_contour(self):
        """A collapsed line must not become a coverage polygon."""
        contour = np.asarray([[0, 0], [1, 1], [2, 2]], np.int32).reshape(-1, 1, 2)
        self.assertEqual(self.module._costmap_contour_polygons(contour), [])

    def test_deterministic_rasters(self):
        """Keep all generated rings valid across irregular raster boundaries."""
        rng = np.random.default_rng(785)
        for index in range(60):
            with self.subTest(index=index):
                mask = np.zeros((100, 100), np.uint8)
                for _ in range(10):
                    center = tuple(int(v) for v in rng.integers(12, 88, size=2))
                    axes = tuple(int(v) for v in rng.integers(4, 20, size=2))
                    angle = int(rng.integers(0, 180))
                    cv2.ellipse(mask, center, axes, angle, 0, 360, 255, -1)
                self.crop(mask)

    def test_http_rejection_and_valid_conversion(self):
        """Reject invalid geometry before writes or native calls; accept valid inputs."""
        server = self.module.CoverageNavigatorServer.__new__(
            self.module.CoverageNavigatorServer)
        server.app = self.module.Flask('polygon-regression')
        server.get_logger = self.server.get_logger
        server.current_routes = []
        server.visualize_field_polygon = lambda *args, **kwargs: None
        server.calculate_polygon_area = lambda field: 1.0
        server.call_get_costmap_service = lambda: True
        server.crop_and_find_free_space = lambda field: (self.captured['field'], [])
        server.update_move_forward_xml = mock.Mock()
        server.generate_coverage_path = mock.Mock()
        server.setup_routes()
        client = server.app.test_client()

        class ForbiddenNative:
            def __getattr__(self, name):
                raise AssertionError('Invalid polygon reached Fields2Cover: ' + name)

        invalid = [
            {'field': SQUARE, 'use_user_field': False},
            {'field': BOWTIE},
            {'field': SQUARE, 'rings': [BOWTIE]},
            {'field': [[0, 0], [1], [0, 2]]},
            {'field': [[0, 0], [float('inf'), 0], [0, 2]]},
            {'field': [[0, 0], [1, 1], [2, 2]]},
            {'field': SQUARE, 'rings': [SQUARE]},
        ]
        with mock.patch.object(self.module, 'f2c', ForbiddenNative()):
            for index, payload in enumerate(invalid):
                with self.subTest(invalid=index):
                    response = client.post('/navigate_coverage', json=payload)
                    self.assertEqual(response.status_code, 422)
                    self.assertEqual(response.json['error_type'], 'invalid_polygon')
                    self.assertEqual(response.json['code'], 1)
                    self.assertTrue(response.json['error'])
                    server.update_move_forward_xml.assert_not_called()
                    server.generate_coverage_path.assert_not_called()

        for payload in [
            {'field': SQUARE},
            {'field': SQUARE, 'rings': [[[1, 1], [2, 1], [2, 2], [1, 2], [1, 1]]]},
        ]:
            with self.subTest(valid=payload):
                server.update_move_forward_xml.reset_mock()
                server.generate_coverage_path.reset_mock()
                response = client.post('/navigate_coverage', json=payload)
                self.assertEqual(response.status_code, 202)
                server.update_move_forward_xml.assert_called_once()
                server.generate_coverage_path.assert_called_once()


if __name__ == '__main__':
    unittest.main()
