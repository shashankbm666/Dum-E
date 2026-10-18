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

    def test_detect_no_image_guard(self):
        import src.stream_server as srv
        # Reset state to clean initial condition
        srv.current_raw_image = None
        srv.last_processed_results = None
        srv.image_version = 0

        res = self.stream_client.get('/api/detect')
        self.assertEqual(res.status_code, 200)
        data = res.get_json()
        self.assertEqual(data["status"], "no_image")
        self.assertIsNone(srv.current_raw_image)

    def test_set_base_tap_no_image_returns_error(self):
        import src.stream_server as srv
        srv.current_raw_image = None
        srv.last_processed_results = None

        res = self.stream_client.post('/api/set_base_tap', json={"pixel_x": 100, "pixel_y": 100})
        self.assertEqual(res.status_code, 400)
        data = res.get_json()
        self.assertEqual(data["status"], "error")

    def test_image_upload_and_version_sync(self):
        import src.stream_server as srv
        srv.current_raw_image = None
        srv.last_processed_results = None
        srv.image_version = 0

        # Create a small valid test JPEG image in memory
        dummy = np.zeros((200, 200, 3), dtype=np.uint8)
        cv2.circle(dummy, (100, 100), 20, (200, 200, 200), -1)
        _, buf = cv2.imencode('.jpg', dummy)

        # Upload image
        data = {'image': (io.BytesIO(buf.tobytes()), 'test.jpg')}
        res = self.stream_client.post('/api/upload', data=data, content_type='multipart/form-data')
        self.assertEqual(res.status_code, 200)
        upload_data = res.get_json()
        self.assertEqual(upload_data["status"], "success")
        self.assertEqual(upload_data["version"], 1)

        # Tap base point -> version increments to 2
        tap_res = self.stream_client.post('/api/set_base_tap', json={"pixel_x": 100, "pixel_y": 190})
        self.assertEqual(tap_res.status_code, 200)
        tap_data = tap_res.get_json()
        self.assertEqual(tap_data["version"], 2)
        self.assertEqual(tap_data["base_pixel"], {"x": 100, "y": 190})

        # Detect bins refresh -> version increments to 3
        refresh_res = self.stream_client.post('/api/detect_bins')
        self.assertEqual(refresh_res.status_code, 200)
        refresh_data = refresh_res.get_json()
        self.assertEqual(refresh_data["version"], 3)

        # /api/detect returns version 3 with the uploaded image, NOT sample
        detect_res = self.stream_client.get('/api/detect')
        self.assertEqual(detect_res.status_code, 200)
        detect_data = detect_res.get_json()
        self.assertEqual(detect_data["version"], 3)
        self.assertEqual(detect_data["base_pixel"], {"x": 100, "y": 190})

if __name__ == '__main__':
    unittest.main()
