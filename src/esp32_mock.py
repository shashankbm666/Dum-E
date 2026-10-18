"""
ESP32 Mock Hardware Server
---------------------------
Simulates the ESP32 robotic arm REST API for offline testing, UI verification,
and hardware-in-the-loop validation without physical ESP32 hardware connected.
"""

from flask import Flask, request, jsonify
from flask_cors import CORS
import logging

def create_mock_app():
    app = Flask(__name__)
    CORS(app)  # Enable CORS for all routes

    state = {
        "status": "online",
        "device": "ESP32-Robotic-Arm-Simulator",
        "base": 90,
        "shoulder": 90,
        "elbow": 90,
        "targetBase": 90,
        "targetShoulder": 90,
        "targetElbow": 90,
        "isMoving": False,
        "magnetOn": False,
        "autoMode": False
    }

    # Suppress verbose Flask logging for clean output
    log = logging.getLogger('werkzeug')
    log.setLevel(logging.WARNING)

    @app.route('/', methods=['GET'])
    def root():
        return jsonify(state)

    @app.route('/status', methods=['GET'])
    def get_status():
        return jsonify(state)

    @app.route('/move', methods=['POST'])
    def set_move():
        """Smooth multi-joint movement endpoint (3 Servos: Base, Shoulder, Elbow)."""
        if state["autoMode"]:
            return jsonify({"error": "Auto mode active"}), 403

        data = request.get_json(force=True, silent=True) or {}
        for j_name in ["base", "shoulder", "elbow"]:
            if j_name in data:
                val = max(0, min(180, int(data[j_name])))
                state[j_name] = val
                state[f"target{j_name.capitalize()}"] = val

        elbow_desc = " (SLOW DESCENT)" if data.get("elbowDelay", 0) >= 35 else ""
        print(f"[MOCK ESP32] Smooth Move -> Base:{state['base']}° Shoulder:{state['shoulder']}° Elbow:{state['elbow']}°{elbow_desc}")
        return jsonify({"status": "success", "state": state})

    @app.route('/joint', methods=['POST'])
    def set_joint():
        if state["autoMode"]:
            return jsonify({"error": "Auto mode active"}), 403

        data = request.get_json(force=True, silent=True) or {}
        joint = data.get("joint")
        angle = data.get("angle")

        if joint is None or angle is None:
            return jsonify({"error": "Missing 'joint' or 'angle'"}), 400

        try:
            joint = int(joint)
            angle = max(0, min(180, int(angle)))
        except ValueError:
            return jsonify({"error": "Invalid numbers"}), 400

        joint_names = {0: "base", 1: "shoulder", 2: "elbow"}
        name = joint_names.get(joint)
        if not name:
            return jsonify({"error": f"Invalid joint {joint} (valid: 0=Base, 1=Shoulder, 2=Elbow)"}), 400

        state[name] = angle
        state[f"target{name.capitalize()}"] = angle
        print(f"[MOCK ESP32] Joint {joint} ({name.upper()}) -> {angle}° (smooth glide)")
        return jsonify({"status": "success", "joint": joint, "angle": angle})

    @app.route('/grip', methods=['POST'])
    def set_grip():
        data = request.get_json(force=True, silent=True) or {}
        val = str(data.get("state", "")).lower()
        state["magnetOn"] = val in ["close", "on", "1", "true"]
        print(f"[MOCK ESP32] Electromagnet -> {'ON (HIGH)' if state['magnetOn'] else 'OFF (LOW)'}")
        return jsonify({"status": "success", "magnetOn": state["magnetOn"]})

    @app.route('/mode', methods=['POST'])
    def set_mode():
        data = request.get_json(force=True, silent=True) or {}
        val = str(data.get("mode", "")).lower()
        state["autoMode"] = (val == "auto")
        print(f"[MOCK ESP32] Control Mode -> {'AUTO' if state['autoMode'] else 'MANUAL'}")
        return jsonify({"status": "success", "autoMode": state["autoMode"]})

    return app

if __name__ == '__main__':
    port = 8080
    print(f"==================================================")
    print(f" [MOCK ESP32] Running Simulated Hardware on http://localhost:{port}")
    print(f"==================================================")
    app = create_mock_app()
    app.run(host='0.0.0.0', port=port, debug=False)
