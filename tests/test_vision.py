import unittest
import numpy as np
import cv2
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from src.stream_server import analyze_and_annotate_image, generate_sample_workbench_image, detect_colored_bins

class TestVisionPipeline(unittest.TestCase):
    def test_sample_image_generation(self):
        img = generate_sample_workbench_image(width=800, height=600)
        self.assertEqual(img.shape, (600, 800, 3))

    def test_detect_colored_bins(self):
        # Generate sample image with Green, Red, and Black bins
        img = generate_sample_workbench_image(width=800, height=600)
        bins = detect_colored_bins(img)

        # Must detect at least 2 bins (Green and Red, and Black)
        self.assertGreaterEqual(len(bins), 2)
        bin_colors = [b["color"] for b in bins]
        self.assertTrue("GREEN" in bin_colors or "RED" in bin_colors)

    def test_image_analysis_pipeline_with_bins_and_coins(self):
        img = generate_sample_workbench_image(width=800, height=600)
        results = analyze_and_annotate_image(img, base_pixel=[400, 570])

        self.assertEqual(results["status"], "success")
        self.assertGreaterEqual(results["coins_found"], 1)
        self.assertGreaterEqual(results["bins_found"], 2)
        self.assertTrue(results["image_data"].startswith("data:image/jpeg;base64,"))
        self.assertEqual(results["base_pixel"], {"x": 400, "y": 570})

if __name__ == '__main__':
    unittest.main()
