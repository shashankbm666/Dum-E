"""
Dum-E Vision & Image Processing Server (OpenCV + Flask)
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
import threading
from flask import Flask, request, jsonify, make_response

# Add parent directory to path to allow importing inverse_kinematics
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from src.inverse_kinematics import ArmKinematics

app = Flask(__name__, static_folder="../static", template_folder="../templates")

# Configuration Defaults
ESP32_IP = os.environ.get("ESP32_IP", "http://10.34.233.78")

# Vision Hough Circles Tuning Parameters
vision_config = {
    "dp": 1.2,
    "minDist": 55,
    "param1": 50,       # Canny higher threshold
    "param2": 30,       # Accumulator threshold for circle centers
    "minRadius": 15,
    "maxRadius": 50,
    "blurKernel": 7
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

    Tuned to reject small dark coin-centers that previously triggered false BLACK bins.
    """
    h, w = img.shape[:2]
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

    # Tightened HSV ranges – BLACK upper V dropped from 55→45, S capped at 60
    color_ranges = [
        ('GREEN',  (35, 100, 80),  (88, 255, 255), (0, 200, 0)),
        ('RED',    (0,   100, 80), (10, 255, 255),  (0, 0, 220)),
        ('RED',    (170, 100, 80), (180, 255, 255), (0, 0, 220)),
        ('BLUE',   (90,  70,  50), (135, 255, 255), (220, 100, 0)),
        ('BLACK',  (0,   0,   0),  (180, 60,  45),  (40, 40, 40)),
        ('YELLOW', (15,  70,  70), (35, 255, 255),  (0, 220, 220))
    ]

    bins = []
    seen_centers = []

    for color_name, low, high, draw_color in color_ranges:
        mask = cv2.inRange(hsv, low, high)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for c in cnts:
            area = cv2.contourArea(c)
            # ── Area gate: true bins are large; coins are small ──
            if area >= 8000:
                x, y, bw, bh = cv2.boundingRect(c)
                # Ignore image-wide background borders
                if bw < w * 0.90 and bh < h * 0.90:
                    # ── Aspect-ratio gate: reject circles ──
                    # A coin's bounding box is roughly square (aspect ≈ 1.0).
                    # True rectangular bins have aspect > 1.3 OR area/bbox > 0.6
                    aspect = max(bw, bh) / max(min(bw, bh), 1)
                    bbox_fill = area / max(bw * bh, 1)
                    if aspect < 1.25 and bbox_fill < 0.65:
                        continue  # Looks circular – skip

                    cx, cy = x + bw // 2, y + bh // 2

                    # Deduplicate overlapping detections (wider 60 px tolerance)
                    is_dup = any(abs(prev_x - cx) < 60 and abs(prev_y - cy) < 60
                                for prev_x, prev_y in seen_centers)
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
                            bin_ik = ik_solver.solve_from_base_pixel(
                                cx, cy, base_pixel[0], base_pixel[1], target_z=30.0)
                        else:
                            bin_ik = ik_solver.coin_to_joint_targets(
                                cx, cy, w, h, target_z=30.0)
                        bin_entry["ik"] = bin_ik

                        bins.append(bin_entry)

    # Sort bins from left to right for stable numbering: Box 1, Box 2, Box 3…
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

def _draw_text_badge(img, text, org, font_scale=0.45, thickness=1,
                      bg_color=(255, 255, 255), fg_color=(0, 0, 0),
                      border_color=(0, 0, 0), padding=4):
    """
    Draws *text* at *org* (bottom-left) on a solid white pill background
    with a thin black border and pure black text for maximum contrast.
    """
    font = cv2.FONT_HERSHEY_SIMPLEX
    (tw, th), baseline = cv2.getTextSize(text, font, font_scale, thickness)
    x, y = int(org[0]), int(org[1])
    x1 = x - padding
    y1 = y - th - padding
    x2 = x + tw + padding
    y2 = y + baseline + padding
    # Clamp inside image
    x1 = max(0, x1)
    y1 = max(0, y1)
    cv2.rectangle(img, (x1, y1), (x2, y2), bg_color, -1)        # white fill
    cv2.rectangle(img, (x1, y1), (x2, y2), border_color, 1)     # black border
    cv2.putText(img, text, (x, y), font, font_scale, fg_color,
                thickness, cv2.LINE_AA)

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
    #    Mask out detected bin regions so coins inside bins
    #    are not double-counted.
    # ----------------------------------------------------
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Blank-out every detected bin rect in the grayscale image
    # so HoughCircles never triggers inside a bin
    bin_mask = np.ones(gray.shape, dtype=np.uint8) * 255
    for b in detected_bins:
        bx, by, bbw, bbh = b["rect"]
        pad = 8  # small padding around the bin
        x1 = max(0, bx - pad)
        y1 = max(0, by - pad)
        x2 = min(w, bx + bbw + pad)
        y2 = min(h, by + bbh + pad)
        bin_mask[y1:y2, x1:x2] = 0
    gray = cv2.bitwise_and(gray, bin_mask)

    # Gaussian blur for smoother edge gradients than median
    k = vision_config["blurKernel"]
    if k % 2 == 0:
        k += 1
    gray = cv2.GaussianBlur(gray, (k, k), 1.5)

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

            # ── Draw coin outline (cyan ring + green center dot) ──
            cv2.circle(annotated, center, radius, (255, 200, 0), 3)  # cyan
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

            # ── White-circle badge with black number at coin center ──
            badge_text = f"#{idx + 1}"
            cv2.circle(annotated, center, 14, (255, 255, 255), -1)   # white fill
            cv2.circle(annotated, center, 14, (0, 0, 0), 2)          # black border
            cv2.putText(annotated, badge_text, (center[0] - 10, center[1] + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 2, cv2.LINE_AA)

            # ── IK label above coin (black text on white pill) ──
            if ik_solution:
                c_ik = ik_solution["world_coords"]
                j = ik_solution["joints"]
                lbl1 = f"#{idx + 1} ({c_ik['x']}mm, {c_ik['y']}mm)"
                lbl2 = f"B:{j['base']}  S:{j['shoulder']}  E:{j['elbow']}"
                lbl_y = center[1] - radius - 6
                _draw_text_badge(annotated, lbl1, (center[0] - 55, lbl_y - 16))
                _draw_text_badge(annotated, lbl2, (center[0] - 55, lbl_y))

    # ----------------------------------------------------
    # 3. Draw Tapped Base Reference Marker
    # ----------------------------------------------------
    bx, by = base_pixel
    # Concentric target circles for Base
    cv2.circle(annotated, (bx, by), 16, (0, 200, 255), 2)
    cv2.circle(annotated, (bx, by), 8, (0, 0, 255), -1)
    cv2.drawMarker(annotated, (bx, by), (0, 200, 255), cv2.MARKER_CROSS, 36, 2)
    _draw_text_badge(annotated, "BASE REF (0, 0)", (bx - 60, by + 30), font_scale=0.55)

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
            resp = make_response(f.read())
            resp.headers["Content-Type"] = "text/html; charset=utf-8"
            resp.headers["Cache-Control"] = "no-cache, no-store, must-revalidate"
            resp.headers["Pragma"] = "no-cache"
            resp.headers["Expires"] = "0"
            return resp
    return "<h1>Vision Server Active</h1>"

@app.route('/api/joint', methods=['POST'])
def proxy_joint():
    """Proxy joint control to ESP32 without CORS/PNA issues on mobile browsers."""
    data = request.get_json(silent=True) or {}
    esp = data.get("esp_ip") or os.environ.get("ESP32_IP", ESP32_IP)
    joint = int(data.get("joint", 0))
    angle = int(data.get("angle", 90))
    try:
        r = requests.post(f"{esp}/joint", json={"joint": joint, "angle": angle}, timeout=0.8)
        return jsonify(r.json()), r.status_code
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/api/status', methods=['GET'])
@app.route('/api/esp_status', methods=['GET', 'POST'])
def proxy_status():
    """Proxy hardware status from ESP32."""
    esp = request.args.get("esp_ip") or os.environ.get("ESP32_IP", ESP32_IP)
    try:
        r = requests.get(f"{esp}/status", timeout=0.8)
        return jsonify(r.json()), r.status_code
    except Exception as e:
        return jsonify({"status": "offline", "error": str(e)}), 503

@app.route('/api/grip', methods=['POST'])
def proxy_grip():
    """Proxy electromagnet toggle to ESP32."""
    data = request.get_json(silent=True) or {}
    esp = data.get("esp_ip") or os.environ.get("ESP32_IP", ESP32_IP)
    state = data.get("state", "open")
    try:
        r = requests.post(f"{esp}/grip", json={"state": state}, timeout=0.8)
        return jsonify(r.json()), r.status_code
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

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

# ==========================================
# Smooth Motion Engine & Trajectory Tracking
# ==========================================

current_motion_status = {
    "active": False,
    "phase": "IDLE",
    "current_step": 0,
    "message": "Arm is ready in neutral position.",
    "progress": 100,
    "last_updated": time.time()
}

def map_to_physical_joints(joints):
    """
    Translates logical IK coordinates to physical ESP32 servo commands:
      - Joint 0 (Base): Direct [0, 180]
      - Joint 1 (Shoulder): Inverted [180 - angle] to match physical mounting (0° = Back, 180° = Reach Forward)
      - Joint 2 (Elbow): Direct [0, 180] (0° = Back, 180° = Reach Forward / Down)
    """
    mapped = {}
    if "base" in joints:
        mapped["base"] = max(0, min(180, int(joints["base"])))
    if "shoulder" in joints:
        # Invert shoulder angle to match physical servo mount
        mapped["shoulder"] = max(0, min(180, 180 - int(joints["shoulder"])))
    if "elbow" in joints:
        mapped["elbow"] = max(0, min(180, int(joints["elbow"])))
    return mapped

def send_smooth_move(esp_ip, joints, elbow_delay=45, transit_delay=20):
    """
    Commands ESP32 to execute a smooth, coordinated move for 3 joints (Base, Shoulder, Elbow).
    Applies hardware calibration mapping (inverting Shoulder).
    Uses /move endpoint with speed control, falling back to /joint.
    """
    phys = map_to_physical_joints(joints)
    payload = {
        "base": phys.get("base", 90),
        "shoulder": phys.get("shoulder", 90),
        "elbow": phys.get("elbow", 90),
        "elbowDelay": int(elbow_delay),
        "transitDelay": int(transit_delay)
    }
    try:
        res = requests.post(f"{esp_ip}/move", json=payload, timeout=0.8)
        if res.status_code == 200:
            return True
    except Exception:
        pass

    # Fallback to individual /joint calls (0=Base, 1=Shoulder, 2=Elbow)
    j_map = {"base": 0, "shoulder": 1, "elbow": 2}
    for name, idx in j_map.items():
        if name in phys:
            try:
                requests.post(f"{esp_ip}/joint", json={"joint": idx, "angle": phys[name]}, timeout=0.5)
            except Exception:
                pass
    return True

def get_primary_coin_joints():
    global last_processed_results
    if last_processed_results and last_processed_results.get("targets"):
        for t in last_processed_results["targets"]:
            if t.get("ik") and t["ik"].get("joints"):
                return t["ik"]["joints"]
    # Sensible default reach position near table front
    return {"base": 90, "shoulder": 45, "elbow": 90}

def get_box_joints(box_num):
    global last_detected_bins
    for b in last_detected_bins:
        if b.get("id") == box_num and b.get("ik"):
            return b["ik"]["joints"]
    fallback_map = {
        1: {"base": 35, "shoulder": 40, "elbow": 90},
        2: {"base": 90, "shoulder": 35, "elbow": 90},
        3: {"base": 145, "shoulder": 40, "elbow": 90}
    }
    return fallback_map.get(box_num, fallback_map[1])

def run_smooth_pick_and_drop_sequence(target_esp, coin_joints, bin_joints, box_num, coin_id=1):
    """
    Executes a multi-stage, jerk-free pick-and-place sequence:
      1. Lift to safe transit elevation.
      2. Smoothly rotate base & shoulder over coin target.
      3. Slowly lower elbow down to gently reach the coin.
      4. Energize electromagnet and pause for solid latch.
      5. Slowly lift elbow back up with coin.
      6. Smoothly and steadily rotate across to destination bin (at transit altitude).
      7. Slowly lower elbow over the bin.
      8. De-energize electromagnet and pause to let coin drop cleanly.
      9. Lift elbow and smoothly return to neutral home position.
    """
    global current_motion_status
    current_motion_status.update({"active": True, "current_step": 1, "progress": 5, "last_updated": time.time()})

    try:
        # Step 1: Raise to Safe Transit Elevation (Elbow ~65° to avoid dragging)
        current_motion_status.update({
            "phase": "TRANSIT_HOVER",
            "current_step": 1,
            "message": f"Hovering above Coin #{coin_id} at safe elevation...",
            "progress": 15,
            "last_updated": time.time()
        })
        hover_coin_pose = {
            "base": coin_joints["base"],
            "shoulder": max(30, coin_joints["shoulder"] - 15),
            "elbow": 65
        }
        send_smooth_move(target_esp, hover_coin_pose, elbow_delay=25, transit_delay=20)
        time.sleep(0.7)

        # Step 2: Slow Steady Elbow Descent to Pick Coin
        current_motion_status.update({
            "phase": "SLOW_DESCENT_PICK",
            "current_step": 2,
            "message": f"Lowering elbow slowly towards Coin #{coin_id}...",
            "progress": 35,
            "last_updated": time.time()
        })
        pick_pose = {
            "base": coin_joints["base"],
            "shoulder": coin_joints["shoulder"],
            "elbow": coin_joints["elbow"]
        }
        # Extra-slow descent: 45ms per degree for gentle coin contact
        send_smooth_move(target_esp, pick_pose, elbow_delay=45, transit_delay=20)
        time.sleep(1.2)

        # Step 3: Energize Electromagnet
        current_motion_status.update({
            "phase": "LATCHING_MAGNET",
            "current_step": 2,
            "message": f"Electromagnet ON. Latched to Coin #{coin_id}.",
            "progress": 45,
            "last_updated": time.time()
        })
        requests.post(f"{target_esp}/grip", json={"state": "close"}, timeout=0.5)
        time.sleep(0.4)

        # Step 4: Slow Steady Lift with Coin
        current_motion_status.update({
            "phase": "SLOW_ASCENT",
            "current_step": 3,
            "message": f"Lifting Coin #{coin_id} smoothly from table...",
            "progress": 55,
            "last_updated": time.time()
        })
        send_smooth_move(target_esp, hover_coin_pose, elbow_delay=30, transit_delay=20)
        time.sleep(0.8)

        # Step 5: Smooth and Steady Transit Across to Bin
        current_motion_status.update({
            "phase": "STEADY_TRANSIT",
            "current_step": 4,
            "message": f"Smooth steady transit across to Box {box_num}...",
            "progress": 70,
            "last_updated": time.time()
        })
        hover_bin_pose = {
            "base": bin_joints["base"],
            "shoulder": max(30, bin_joints["shoulder"] - 15),
            "elbow": 65
        }
        send_smooth_move(target_esp, hover_bin_pose, elbow_delay=25, transit_delay=22)
        time.sleep(1.2)

        # Step 6: Slow Steady Elbow Descent into Bin
        current_motion_status.update({
            "phase": "SLOW_DESCENT_DROP",
            "current_step": 5,
            "message": f"Lowering elbow slowly over Box {box_num}...",
            "progress": 85,
            "last_updated": time.time()
        })
        drop_pose = {
            "base": bin_joints["base"],
            "shoulder": bin_joints["shoulder"],
            "elbow": bin_joints["elbow"]
        }
        send_smooth_move(target_esp, drop_pose, elbow_delay=45, transit_delay=20)
        time.sleep(1.1)

        # Step 7: Release Coin into Bin
        current_motion_status.update({
            "phase": "RELEASING_COIN",
            "current_step": 5,
            "message": f"Releasing coin into Box {box_num}...",
            "progress": 92,
            "last_updated": time.time()
        })
        requests.post(f"{target_esp}/grip", json={"state": "open"}, timeout=0.5)
        time.sleep(0.35)

        # Step 8: Smooth Home Return
        current_motion_status.update({
            "phase": "HOME_RETURN",
            "current_step": 6,
            "message": "Returning smoothly to neutral home position...",
            "progress": 98,
            "last_updated": time.time()
        })
        home_pose = {"base": 90, "shoulder": 90, "elbow": 90}
        send_smooth_move(target_esp, home_pose, elbow_delay=25, transit_delay=20)
        time.sleep(0.8)

        current_motion_status.update({
            "active": False,
            "phase": "COMPLETED",
            "current_step": 6,
            "message": f"Coin #{coin_id} delivered successfully to Box {box_num}!",
            "progress": 100,
            "last_updated": time.time()
        })
    except Exception as e:
        current_motion_status.update({
            "active": False,
            "phase": "ERROR",
            "current_step": 0,
            "message": f"Motion error: {str(e)}",
            "progress": 0,
            "last_updated": time.time()
        })

# ==========================================
# Step-by-Step Individual Action Routes
# ==========================================

@app.route('/api/step/hover_coin', methods=['POST'])
def step_hover_coin():
    data = request.get_json(silent=True) or {}
    target_esp = data.get("esp_ip") or os.environ.get("ESP32_IP", ESP32_IP)
    coin_joints = get_primary_coin_joints()
    hover_pose = {
        "base": coin_joints["base"],
        "shoulder": max(30, coin_joints["shoulder"] - 15),
        "elbow": 65
    }
    current_motion_status.update({
        "active": True,
        "phase": "TRANSIT_HOVER",
        "current_step": 1,
        "message": "Hovering above coin at safe elevation...",
        "progress": 15,
        "last_updated": time.time()
    })
    send_smooth_move(target_esp, hover_pose, elbow_delay=25, transit_delay=20)
    current_motion_status.update({"active": False, "message": "Hovering above coin (Step 1 Complete)."})
    return jsonify({"status": "success", "step": 1, "phase": "HOVER_COIN", "joints": hover_pose})

@app.route('/api/step/pick_coin', methods=['POST'])
def step_pick_coin():
    data = request.get_json(silent=True) or {}
    target_esp = data.get("esp_ip") or os.environ.get("ESP32_IP", ESP32_IP)
    coin_joints = get_primary_coin_joints()
    pick_pose = {
        "base": coin_joints["base"],
        "shoulder": coin_joints["shoulder"],
        "elbow": coin_joints["elbow"]
    }
    current_motion_status.update({
        "active": True,
        "phase": "SLOW_DESCENT_PICK",
        "current_step": 2,
        "message": "Lowering slowly onto coin...",
        "progress": 35,
        "last_updated": time.time()
    })
    send_smooth_move(target_esp, pick_pose, elbow_delay=45, transit_delay=20)
    time.sleep(0.4)
    try:
        requests.post(f"{target_esp}/grip", json={"state": "close"}, timeout=0.5)
    except Exception:
        pass
    current_motion_status.update({
        "active": False,
        "phase": "LATCHING_MAGNET",
        "current_step": 2,
        "message": "Electromagnet ON. Coin latched! (Step 2 Complete)",
        "progress": 45,
        "last_updated": time.time()
    })
    return jsonify({"status": "success", "step": 2, "phase": "PICK_COIN", "joints": pick_pose, "magnet": "on"})

@app.route('/api/step/lift_coin', methods=['POST'])
def step_lift_coin():
    data = request.get_json(silent=True) or {}
    target_esp = data.get("esp_ip") or os.environ.get("ESP32_IP", ESP32_IP)
    coin_joints = get_primary_coin_joints()
    lift_pose = {
        "base": coin_joints["base"],
        "shoulder": max(30, coin_joints["shoulder"] - 15),
        "elbow": 65
    }
    current_motion_status.update({
        "active": True,
        "phase": "SLOW_ASCENT",
        "current_step": 3,
        "message": "Lifting coin smoothly from table...",
        "progress": 55,
        "last_updated": time.time()
    })
    send_smooth_move(target_esp, lift_pose, elbow_delay=30, transit_delay=20)
    current_motion_status.update({"active": False, "message": "Coin lifted to transit altitude (Step 3 Complete)."})
    return jsonify({"status": "success", "step": 3, "phase": "LIFT_COIN", "joints": lift_pose})

@app.route('/api/step/transit_box', methods=['POST'])
def step_transit_box():
    data = request.get_json(silent=True) or {}
    box_num = int(data.get("box", 1))
    target_esp = data.get("esp_ip") or os.environ.get("ESP32_IP", ESP32_IP)
    bin_joints = get_box_joints(box_num)
    hover_bin = {
        "base": bin_joints["base"],
        "shoulder": max(30, bin_joints["shoulder"] - 15),
        "elbow": 65
    }
    current_motion_status.update({
        "active": True,
        "phase": "STEADY_TRANSIT",
        "current_step": 4,
        "message": f"Smooth transit across to Box {box_num}...",
        "progress": 75,
        "last_updated": time.time()
    })
    send_smooth_move(target_esp, hover_bin, elbow_delay=25, transit_delay=22)
    current_motion_status.update({"active": False, "message": f"Hovering over Box {box_num} (Step 4 Complete)."})
    return jsonify({"status": "success", "step": 4, "phase": "TRANSIT_BOX", "box": box_num, "joints": hover_bin})

@app.route('/api/step/drop_coin', methods=['POST'])
def step_drop_coin():
    data = request.get_json(silent=True) or {}
    box_num = int(data.get("box", 1))
    target_esp = data.get("esp_ip") or os.environ.get("ESP32_IP", ESP32_IP)
    bin_joints = get_box_joints(box_num)
    drop_pose = {
        "base": bin_joints["base"],
        "shoulder": bin_joints["shoulder"],
        "elbow": bin_joints["elbow"]
    }
    current_motion_status.update({
        "active": True,
        "phase": "SLOW_DESCENT_DROP",
        "current_step": 5,
        "message": f"Lowering slowly into Box {box_num}...",
        "progress": 85,
        "last_updated": time.time()
    })
    send_smooth_move(target_esp, drop_pose, elbow_delay=45, transit_delay=20)
    time.sleep(0.4)
    try:
        requests.post(f"{target_esp}/grip", json={"state": "open"}, timeout=0.5)
    except Exception:
        pass
    lift_bin = {
        "base": bin_joints["base"],
        "shoulder": max(30, bin_joints["shoulder"] - 15),
        "elbow": 65
    }
    send_smooth_move(target_esp, lift_bin, elbow_delay=25, transit_delay=20)
    current_motion_status.update({
        "active": False,
        "phase": "RELEASING_COIN",
        "current_step": 5,
        "message": f"Coin dropped in Box {box_num}! (Step 5 Complete)",
        "progress": 92
    })
    return jsonify({"status": "success", "step": 5, "phase": "DROP_COIN", "box": box_num, "magnet": "off"})

@app.route('/api/step/home', methods=['POST'])
def step_home():
    data = request.get_json(silent=True) or {}
    target_esp = data.get("esp_ip") or os.environ.get("ESP32_IP", ESP32_IP)
    home_pose = {"base": 90, "shoulder": 90, "elbow": 90}
    current_motion_status.update({
        "active": True,
        "phase": "HOME_RETURN",
        "current_step": 6,
        "message": "Returning to neutral 90° home position...",
        "progress": 98,
        "last_updated": time.time()
    })
    send_smooth_move(target_esp, home_pose, elbow_delay=25, transit_delay=20)
    current_motion_status.update({
        "active": False,
        "phase": "COMPLETED",
        "current_step": 6,
        "message": "Arm centered at 90° neutral (Step 6 Complete).",
        "progress": 100
    })
    return jsonify({"status": "success", "step": 6, "phase": "HOME", "joints": home_pose})

@app.route('/api/sort_to_box', methods=['POST'])
def sort_to_box():
    """
    Dispatches robotic arm smoothly across to the detected physical coordinates of Box 1, 2, or 3.
    """
    global last_detected_bins, last_processed_results
    data = request.get_json(force=True, silent=True) or {}
    box_num = int(data.get("box", 1))
    target_esp = data.get("esp_ip") or os.environ.get("ESP32_IP", ESP32_IP)

    # 1. Find detected bin target
    chosen_joints = None
    for b in last_detected_bins:
        if b.get("id") == box_num and b.get("ik"):
            chosen_joints = b["ik"]["joints"]
            break

    # Fallback coordinates if bin not visually detected
    if not chosen_joints:
        fallback_map = {
            1: {"base": 35, "shoulder": 40, "elbow": 90},
            2: {"base": 90, "shoulder": 35, "elbow": 90},
            3: {"base": 145, "shoulder": 40, "elbow": 90}
        }
        chosen_joints = fallback_map.get(box_num, fallback_map[1])

    # Check if a primary coin exists to execute full pick-and-place
    coin_joints = None
    if last_processed_results and last_processed_results.get("targets"):
        t0 = last_processed_results["targets"][0]
        if t0.get("ik"):
            coin_joints = t0["ik"]["joints"]

    if coin_joints:
        # Launch full smooth pick-and-place in background thread
        thread = threading.Thread(
            target=run_smooth_pick_and_drop_sequence,
            args=(target_esp, coin_joints, chosen_joints, box_num, 1),
            daemon=True
        )
        thread.start()
        return jsonify({
            "status": "success",
            "mode": "smooth_pick_and_drop",
            "box": box_num,
            "bin_joints": chosen_joints,
            "coin_joints": coin_joints
        })
    else:
        # Move directly to bin with smooth transit & slow descent
        transit_pose = {"base": chosen_joints["base"], "shoulder": max(30, chosen_joints["shoulder"] - 15), "elbow": 65}
        send_smooth_move(target_esp, transit_pose, elbow_delay=25, transit_delay=20)
        time.sleep(0.4)
        send_smooth_move(target_esp, chosen_joints, elbow_delay=45, transit_delay=20)
        try:
            requests.post(f"{target_esp}/grip", json={"state": "open"}, timeout=0.5)
        except Exception:
            pass
        return jsonify({"status": "success", "mode": "smooth_bin_transit", "box": box_num, "joints": chosen_joints})

@app.route('/api/pick_and_drop', methods=['POST'])
def pick_and_drop_coin():
    """
    Picks a specific detected coin and smoothly drops it into the chosen box.
    """
    global last_detected_bins, last_processed_results
    data = request.get_json(force=True, silent=True) or {}
    coin_id = int(data.get("coin_id", 1))
    box_num = int(data.get("box", 1))
    target_esp = data.get("esp_ip") or os.environ.get("ESP32_IP", ESP32_IP)

    # 1. Locate coin IK
    coin_joints = None
    if last_processed_results and last_processed_results.get("targets"):
        for t in last_processed_results["targets"]:
            if t.get("id") == coin_id and t.get("ik"):
                coin_joints = t["ik"]["joints"]
                break

    if not coin_joints:
        return jsonify({"status": "error", "error": f"Coin #{coin_id} has no valid IK solution."}), 400

    # 2. Locate bin IK
    bin_joints = None
    for b in last_detected_bins:
        if b.get("id") == box_num and b.get("ik"):
            bin_joints = b["ik"]["joints"]
            break

    if not bin_joints:
        fallback_map = {
            1: {"base": 35, "shoulder": 40, "elbow": 90},
            2: {"base": 90, "shoulder": 35, "elbow": 90},
            3: {"base": 145, "shoulder": 40, "elbow": 90}
        }
        bin_joints = fallback_map.get(box_num, fallback_map[1])

    # Launch smooth sequence
    thread = threading.Thread(
        target=run_smooth_pick_and_drop_sequence,
        args=(target_esp, coin_joints, bin_joints, box_num, coin_id),
        daemon=True
    )
    thread.start()

    return jsonify({
        "status": "success",
        "coin_id": coin_id,
        "box": box_num,
        "message": f"Smooth pick & drop initiated for Coin #{coin_id} -> Box {box_num}"
    })

@app.route('/api/motion_status', methods=['GET'])
def get_motion_status():
    """Returns real-time telemetry of arm motion, phase, and progress."""
    global current_motion_status
    return jsonify(current_motion_status)

@app.route('/api/abort', methods=['POST'])
def abort_arm():
    """Emergency stop: releases magnet and immediately returns arm to neutral 90 degrees."""
    global current_motion_status
    data = request.get_json(force=True, silent=True) or {}
    target_esp = data.get("esp_ip") or os.environ.get("ESP32_IP", ESP32_IP)
    current_motion_status.update({"active": False, "phase": "ABORTED", "message": "Emergency abort triggered.", "progress": 0})
    try:
        requests.post(f"{target_esp}/grip", json={"state": "open"}, timeout=0.5)
        for j in range(3):
            requests.post(f"{target_esp}/joint", json={"joint": j, "angle": 90}, timeout=0.5)
        return jsonify({"status": "aborted", "joints": {"base": 90, "shoulder": 90, "elbow": 90}, "magnet": "off"})
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
    print(f" [DUM-E VISION] Running on http://localhost:{port}")
    print(f" [DUM-E BINS]   POST http://localhost:{port}/api/detect_bins")
    print(f" [DUM-E BASE]      POST http://localhost:{port}/api/set_base_tap")
    print("=" * 60)
    app.run(host='0.0.0.0', port=port, debug=False, threaded=True)
