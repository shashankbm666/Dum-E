import unittest
import io
import cv2
import numpy as np
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from src.esp32_mock import create_mock_app
from src.stream_server import app as stream_app, generate_sample_workbench_image

class TestApiEndpoints(unittest.TestCase):
    def setUp(self):
        self.mock_client = create_mock_app().test_client()
        self.stream_client = stream_app.test_client()

    def test_mock_esp32_status(self):
        res = self.mock_client.get('/status')
        self.assertEqual(res.status_code, 200)
        data = res.get_json()
        self.assertEqual(data["status"], "online")

    def test_sample_image_endpoint(self):
        res = self.stream_client.get('/api/sample')
        self.assertEqual(res.status_code, 200)
        data = res.get_json()
        self.assertEqual(data["status"], "success")
        self.assertGreaterEqual(data["coins_found"], 1)
        self.assertGreaterEqual(data["bins_found"], 2)

    def test_set_base_tap_endpoint(self):
        # 1. Load an image first
        self.stream_client.get('/api/sample')

        # 2. Tap to set base point at (410, 520)
        tap_payload = {"pixel_x": 410, "pixel_y": 520}
        res = self.stream_client.post('/api/set_base_tap', json=tap_payload)
        self.assertEqual(res.status_code, 200)
        data = res.get_json()
        self.assertEqual(data["base_pixel"], {"x": 410, "y": 520})

    def test_detect_bins_refresh_endpoint(self):
        # 1. Load an image first
        self.stream_client.get('/api/sample')

        # 2. Trigger detect bins refresh
        res = self.stream_client.post('/api/detect_bins')
        self.assertEqual(res.status_code, 200)
        data = res.get_json()
        self.assertGreaterEqual(data["bins_found"], 2)

    def test_sort_to_box_endpoint(self):
        # Load sample with bins
        self.stream_client.get('/api/sample')

        # Dispatch to Box 1 (Green bin)
        res = self.stream_client.post('/api/sort_to_box', json={"box": 1, "esp_ip": "http://localhost:8080"})
        self.assertIn(res.status_code, [200, 502])

    def test_motion_status_endpoint(self):
        res = self.stream_client.get('/api/motion_status')
        self.assertEqual(res.status_code, 200)
        data = res.get_json()
        self.assertIn("phase", data)
        self.assertIn("progress", data)

    def test_pick_and_drop_endpoint(self):
        self.stream_client.get('/api/sample')
        res = self.stream_client.post('/api/pick_and_drop', json={"coin_id": 1, "box": 2, "esp_ip": "http://localhost:8080"})
        self.assertEqual(res.status_code, 200)
        data = res.get_json()
        self.assertEqual(data["status"], "success")

    def test_network_info_endpoint(self):
        res = self.stream_client.get('/api/network_info')
        self.assertEqual(res.status_code, 200)
        data = res.get_json()
        self.assertEqual(data["status"], "success")
        self.assertIn("mobile_url", data)
        self.assertIn("local_ip", data)
        self.assertIn("port", data)

if __name__ == '__main__':
    unittest.main()
