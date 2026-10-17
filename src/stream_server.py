"""
Robotic Vision & Image Processing Server (OpenCV + Flask)
----------------------------------------------------------
Features:
  - Detects Circular Coins (Hough Circles).
  - Detects Rectangular Colored Bins (Green, Red, Black, Blue, Yellow).
  - Tappable Base Point: Tap anywhere on the image to set the robot's base reference.
  - Interactive Bin Refresh: Re-detect and refresh bins on demand.
  - Direct Dispatch to Bins: Moves arm to the exact physical coordinates of detected bins.
"""

import os
import sys
import time
import base64
import math
import cv2
import numpy as np
import requests
from flask import Flask, request, jsonify

# Add parent directory to path to allow importing inverse_kinematics
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from src.inverse_kinematics import ArmKinematics

app = Flask(__name__, static_folder="../static", template_folder="../templates")

# Configuration Defaults
ESP32_IP = os.environ.get("ESP32_IP", "http://localhost:8080")

# Vision Hough Circles Tuning Parameters
vision_config = {
    "dp": 1.2,
    "minDist": 35,
    "param1": 50,       # Canny higher threshold
    "param2": 28,       # Accumulator threshold for circle centers
    "minRadius": 15,
    "maxRadius": 80,
    "blurKernel": 5
}

ik_solver = ArmKinematics(l1=120.0, l2=120.0, base_height=50.0)

# In-memory session state
current_raw_image = None
current_base_pixel = None  # [x, y] in image coordinates
last_processed_results = None
last_detected_bins = []

def detect_colored_bins(img, base_pixel=None):
    """
    Detects colored rectangular bins (Green, Red, Black, Blue, etc.) in the image.
    Returns list of dicts with bounding boxes, center coordinates, and IK solutions.
    """
    h, w = img.shape[:2]
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

    color_ranges = [
        ('GREEN', (35, 60, 40), (88, 255, 255), (0, 200, 0)),
        ('RED', (0, 70, 50), (10, 255, 255), (0, 0, 220)),
        ('RED', (170, 70, 50), (180, 255, 255), (0, 0, 220)),
        ('BLUE', (90, 70, 50), (135, 255, 255), (220, 100, 0)),
        ('BLACK', (0, 0, 0), (180, 255, 55), (40, 40, 40)),
        ('YELLOW', (15, 70, 70), (35, 255, 255), (0, 220, 220))
    ]

    bins = []
    seen_centers = []

    for color_name, low, high, draw_color in color_ranges:
        mask = cv2.inRange(hsv, low, high)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for c in cnts:
            area = cv2.contourArea(c)
            # Filter for substantial rectangular areas
            if area > 1800:
                x, y, bw, bh = cv2.boundingRect(c)
                # Ignore background borders
                if bw < w * 0.92 and bh < h * 0.92:
                    cx, cy = x + bw // 2, y + bh // 2

                    # Deduplicate overlapping detections
                    is_dup = any(abs(prev_x - cx) < 35 and abs(prev_y - cy) < 35 for prev_x, prev_y in seen_centers)
                    if not is_dup:
                        seen_centers.append((cx, cy))
                        bin_entry = {
                            "color": color_name,
                            "rect": [x, y, bw, bh],
                            "center": [cx, cy],
                            "area": area,
                            "bgr": draw_color
                        }

                        # Compute IK for bin center relative to base
                        if base_pixel:
                            bin_ik = ik_solver.solve_from_base_pixel(cx, cy, base_pixel[0], base_pixel[1], target_z=30.0)
                        else:
                            bin_ik = ik_solver.coin_to_joint_targets(cx, cy, w, h, target_z=30.0)
                        bin_entry["ik"] = bin_ik

                        bins.append(bin_entry)

    # Sort bins from left to right for stable numbering: Box 1, Box 2, Box 3...
    bins.sort(key=lambda b: (b['center'][0]))
    for idx, b in enumerate(bins):
        b['id'] = idx + 1
        b['name'] = f"Box {idx + 1} ({b['color']})"

    return bins

def generate_sample_workbench_image(width=800, height=600):
    """
    Creates a realistic synthetic photo matching user workbench:
      - Green rectangular bin on left
      - Red rectangular bin on right
      - Black rectangular bin on top
      - Metallic circular coins in the center
    """
    img = np.full((height, width, 3), 245, dtype=np.uint8)  # White workbench surface

    # Subtle grid
    for x in range(0, width, 50):
        cv2.line(img, (x, 0), (x, height), (225, 225, 225), 1)
    for y in range(0, height, 50):
        cv2.line(img, (0, y), (width, y), (225, 225, 225), 1)

    cx, cy = width // 2, height // 2

    # 1. Top Black Bin
    cv2.rectangle(img, (cx - 160, 30), (cx + 160, 160), (20, 20, 20), -1)
    cv2.rectangle(img, (cx - 160, 30), (cx + 160, 160), (0, 0, 0), 3)

    # 2. Left Green Bin
    cv2.rectangle(img, (40, cy - 80), (220, cy + 180), (50, 180, 60), -1)
    cv2.rectangle(img, (40, cy - 80), (220, cy + 180), (20, 120, 30), 3)

    # 3. Right Red Bin
    cv2.rectangle(img, (width - 220, cy - 80), (width - 40, cy + 180), (40, 40, 220), -1)
    cv2.rectangle(img, (width - 220, cy - 80), (width - 40, cy + 180), (20, 20, 160), 3)

    # 4. Circular Coins in the center workspace
    coins = [
        {"x": cx - 15, "y": cy - 40, "r": 30, "color": (30, 190, 230)},
        {"x": cx + 10, "y": cy + 40, "r": 28, "color": (190, 190, 190)},
        {"x": cx - 5, "y": cy + 120, "r": 32, "color": (30, 190, 230)}
    ]

    for c in coins:
        cv2.circle(img, (c["x"], c["y"]), c["r"], c["color"], -1)
        cv2.circle(img, (c["x"], c["y"]), c["r"] - 4, (10, 10, 10), -1)
        cv2.circle(img, (c["x"], c["y"]), c["r"] - 8, c["color"], -1)
        cv2.circle(img, (c["x"] - 6, c["y"] - 6), 5, (255, 255, 255), -1)

    return img

def analyze_and_annotate_image(img, base_pixel=None):
    """
    Detects both coins and colored rectangular bins,
    calculates relative Inverse Kinematics from base_pixel,
    and returns an annotated image with structured metadata.
    """
    global current_base_pixel, last_detected_bins
    h, w = img.shape[:2]
    annotated = img.copy()

    # Default base pixel to bottom-center of image if not specified
    if base_pixel is None:
        if current_base_pixel is not None:
            base_pixel = current_base_pixel
        else:
            base_pixel = [w // 2, h - 30]
            current_base_pixel = base_pixel
    else:
        current_base_pixel = base_pixel

    # ----------------------------------------------------
    # 1. Detect Rectangular Colored Bins
    # ----------------------------------------------------
    detected_bins = detect_colored_bins(img, base_pixel=base_pixel)
    last_detected_bins = detected_bins

    for b in detected_bins:
        x, y, bw, bh = b["rect"]
        color = b["bgr"]
        # Draw bold colored boundary for bin
        cv2.rectangle(annotated, (x, y), (x + bw, y + bh), color, 4)
        # Center marker
        cx, cy = b["center"]
        cv2.drawMarker(annotated, (cx, cy), color, cv2.MARKER_CROSS, 20, 2)

        # Label tag
        lbl = b["name"]
        cv2.rectangle(annotated, (x, max(0, y - 26)), (x + len(lbl) * 11, y), color, -1)
        cv2.putText(annotated, lbl, (x + 4, max(18, y - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2, cv2.LINE_AA)

        if b.get("ik"):
            j = b["ik"]["joints"]
            ik_lbl = f"IK: B:{j['base']}* S:{j['shoulder']}* E:{j['elbow']}*"
            cv2.putText(annotated, ik_lbl, (x + 4, y + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)

    # ----------------------------------------------------
    # 2. Detect Circular Coins (Hough Circles)
    # ----------------------------------------------------
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    k = vision_config["blurKernel"]
    if k % 2 == 0:
        k += 1
    gray = cv2.medianBlur(gray, k)

    circles = cv2.HoughCircles(
        gray,
        cv2.HOUGH_GRADIENT,
        dp=float(vision_config["dp"]),
        minDist=float(vision_config["minDist"]),
        param1=float(vision_config["param1"]),
        param2=float(vision_config["param2"]),
        minRadius=int(vision_config["minRadius"]),
        maxRadius=int(vision_config["maxRadius"])
    )

    targets = []

    if circles is not None:
        circles = np.uint16(np.around(circles))
        for idx, i in enumerate(circles[0, :]):
            center = (int(i[0]), int(i[1]))
            radius = int(i[2])

            # Draw outer yellow ring & center marker
            cv2.circle(annotated, center, radius, (0, 240, 255), 3)
            cv2.circle(annotated, center, 4, (0, 255, 0), -1)

            # Solve Kinematics relative to tapped base pixel
            ik_solution = ik_solver.solve_from_base_pixel(
                center[0], center[1],
                base_pixel[0], base_pixel[1],
                target_z=10.0
            )

            target_entry = {
                "id": idx + 1,
                "pixel": {"x": center[0], "y": center[1], "radius": radius},
                "ik": ik_solution
            }
            targets.append(target_entry)

            # Visual badge
            badge_text = f"#{idx + 1}"
            cv2.putText(annotated, badge_text, (center[0] - 10, center[1] + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2, cv2.LINE_AA)
            cv2.putText(annotated, badge_text, (center[0] - 10, center[1] + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

            if ik_solution:
                c = ik_solution["world_coords"]
                j = ik_solution["joints"]
                lbl1 = f"#{idx + 1} ({c['x']}mm, {c['y']}mm)"
                lbl2 = f"B:{j['base']}* S:{j['shoulder']}* E:{j['elbow']}*"
                cv2.putText(annotated, lbl1, (center[0] - 55, center[1] - radius - 16),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 240, 255), 1, cv2.LINE_AA)
                cv2.putText(annotated, lbl2, (center[0] - 65, center[1] - radius - 2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.40, (0, 240, 255), 1, cv2.LINE_AA)

    # ----------------------------------------------------
    # 3. Draw Tapped Base Reference Marker
    # ----------------------------------------------------
    bx, by = base_pixel
    # Concentric target circles for Base
    cv2.circle(annotated, (bx, by), 16, (0, 240, 255), 2)
    cv2.circle(annotated, (bx, by), 8, (0, 0, 255), -1)
    cv2.drawMarker(annotated, (bx, by), (0, 240, 255), cv2.MARKER_CROSS, 36, 2)
    cv2.putText(annotated, "BASE REF (0, 0)", (bx - 60, by + 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 240, 255), 2, cv2.LINE_AA)

    # Encode to base64
    ret, buffer = cv2.imencode('.jpg', annotated, [cv2.IMWRITE_JPEG_QUALITY, 90])
    b64_image = base64.b64encode(buffer).decode('utf-8') if ret else ""

    result = {
        "status": "success",
        "coins_found": len(targets),
        "bins_found": len(detected_bins),
        "targets": targets,
        "bins": detected_bins,
        "base_pixel": {"x": bx, "y": by},
        "image_data": f"data:image/jpeg;base64,{b64_image}",
        "width": w,
        "height": h,
        "timestamp": time.time()
    }
    return result

# ==========================================
# Web Routes & API Endpoints
# ==========================================

@app.route('/')
def index():
    html_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "../index.html"))
    if os.path.exists(html_path):
        with open(html_path, "r", encoding="utf-8") as f:
            return f.read(), 200, {"Content-Type": "text/html"}
    return "<h1>Vision Server Active</h1>"

@app.route('/api/upload', methods=['POST'])
def upload_image():
    """Receives image file, processes it, and caches the raw image for re-detection/taps."""
    global current_raw_image, last_processed_results

    img = None
    if 'image' in request.files:
        file = request.files['image']
        file_bytes = np.frombuffer(file.read(), np.uint8)
        img = cv2.imdecode(file_bytes, cv2.IMREAD_COLOR)
    elif request.is_json:
        data = request.get_json(silent=True) or {}
        b64_str = data.get("image", "")
        if b64_str.startswith("data:image"):
            b64_str = b64_str.split(",", 1)[1]
        if b64_str:
            file_bytes = np.frombuffer(base64.b64decode(b64_str), np.uint8)
            img = cv2.imdecode(file_bytes, cv2.IMREAD_COLOR)
    elif request.data:
        file_bytes = np.frombuffer(request.data, np.uint8)
        img = cv2.imdecode(file_bytes, cv2.IMREAD_COLOR)

    if img is None:
        return jsonify({"status": "error", "error": "No valid image provided"}), 400

    current_raw_image = img
    results = analyze_and_annotate_image(img)
    last_processed_results = results
    return jsonify(results)

@app.route('/api/sample', methods=['GET'])
def get_sample():
    """Generates and processes a sample test image with colored bins and coins."""
    global current_raw_image, last_processed_results
    sample_img = generate_sample_workbench_image()
    current_raw_image = sample_img
    results = analyze_and_annotate_image(sample_img)
    last_processed_results = results
    return jsonify(results)

@app.route('/api/set_base_tap', methods=['POST'])
def set_base_tap():
    """Sets the robot arm base reference directly by tapping coordinates on the image."""
    global current_raw_image, current_base_pixel, last_processed_results
    if current_raw_image is None:
        return jsonify({"status": "error", "error": "No image loaded yet"}), 400

    data = request.get_json(force=True, silent=True) or {}
    px = int(data.get("pixel_x", current_raw_image.shape[1] // 2))
    py = int(data.get("pixel_y", current_raw_image.shape[0] - 30))

    current_base_pixel = [px, py]
    results = analyze_and_annotate_image(current_raw_image, base_pixel=current_base_pixel)
    last_processed_results = results
    return jsonify(results)

@app.route('/api/detect_bins', methods=['POST', 'GET'])
def trigger_detect_bins():
    """Re-runs and refreshes rectangular colored bin detection on the active image."""
    global current_raw_image, last_processed_results
    if current_raw_image is None:
        return jsonify({"status": "error", "error": "No image loaded yet"}), 400

    results = analyze_and_annotate_image(current_raw_image, base_pixel=current_base_pixel)
    last_processed_results = results
    return jsonify(results)

@app.route('/api/detect', methods=['GET'])
def get_latest_detection():
    """Returns the most recent image processing results."""
    if last_processed_results is None:
        return get_sample()
    return jsonify(last_processed_results)

@app.route('/api/sort_to_box', methods=['POST'])
def sort_to_box():
    """
    Dispatches robotic arm directly to the detected physical coordinates of Box 1, 2, or 3.
    """
    global last_detected_bins
    data = request.get_json(force=True, silent=True) or {}
    box_num = int(data.get("box", 1))
    target_esp = data.get("esp_ip") or os.environ.get("ESP32_IP", ESP32_IP)

    # 1. Try to find the detected bin
    chosen_joints = None
    for b in last_detected_bins:
        if b.get("id") == box_num and b.get("ik"):
            chosen_joints = b["ik"]["joints"]
            break

    # 2. Fallback positions if bin not detected
    if not chosen_joints:
        fallback_map = {
            1: {"base": 35, "shoulder": 40, "elbow": 90, "wrist": 90},
            2: {"base": 90, "shoulder": 35, "elbow": 90, "wrist": 90},
            3: {"base": 145, "shoulder": 40, "elbow": 90, "wrist": 90}
        }
        chosen_joints = fallback_map.get(box_num, fallback_map[1])

    try:
        # Move joints to bin location
        for j_idx, (j_name, angle) in enumerate(chosen_joints.items()):
            requests.post(f"{target_esp}/joint", json={"joint": j_idx, "angle": angle}, timeout=0.5)
        # Release electromagnet
        requests.post(f"{target_esp}/grip", json={"state": "open"}, timeout=0.5)
        return jsonify({"status": "success", "box": box_num, "joints": chosen_joints})
    except Exception as e:
        return jsonify({"status": "error", "error": str(e)}), 502

@app.route('/api/abort', methods=['POST'])
def abort_arm():
    """Emergency stop: releases magnet and immediately returns arm to neutral 90 degrees."""
    data = request.get_json(force=True, silent=True) or {}
    target_esp = data.get("esp_ip") or os.environ.get("ESP32_IP", ESP32_IP)
    try:
        requests.post(f"{target_esp}/grip", json={"state": "open"}, timeout=0.5)
        for j in range(4):
            requests.post(f"{target_esp}/joint", json={"joint": j, "angle": 90}, timeout=0.5)
        return jsonify({"status": "aborted", "joints": {"base": 90, "shoulder": 90, "elbow": 90, "wrist": 90}, "magnet": "off"})
    except Exception as e:
        return jsonify({"status": "error", "error": str(e)}), 502

@app.route('/api/config', methods=['GET', 'POST'])
def manage_config():
    if request.method == 'POST':
        data = request.get_json(force=True, silent=True) or {}
        for k in vision_config.keys():
            if k in data:
                vision_config[k] = type(vision_config[k])(data[k])
        return jsonify({"status": "updated", "config": vision_config})
    return jsonify(vision_config)

@app.route('/api/health', methods=['GET'])
def health():
    return jsonify({"status": "running", "timestamp": time.time()})

if __name__ == '__main__':
    port = int(os.environ.get("PORT", 8000))
    print("=" * 60)
    print(f" [VISION SERVER] Running on http://localhost:{port}")
    print(f" [DETECT BINS]   POST http://localhost:{port}/api/detect_bins")
    print(f" [TAP BASE]      POST http://localhost:{port}/api/set_base_tap")
    print("=" * 60)
    app.run(host='0.0.0.0', port=port, debug=False, threaded=True)
