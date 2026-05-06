"""
UAV Dashboard - Real Server (Raspberry Pi 5 + MAVLink)
"""
import os, time, threading, csv, math, cv2
from flask import Flask, request, jsonify, render_template, Response
from flask_cors import CORS
from pymavlink import mavutil

app = Flask(__name__)
CORS(app)

SERIAL_PORT = '/dev/ttyAMA0'
BAUD_RATE   = 57600

# ── GLOBAL STATE ──────────────────────────────────────────────────────────────
state = {
    "timestamp": "",
    "armed": False, "mode": "UNKNOWN", "battery": 0.0,
    "gps": {"lat": 0.0, "lon": 0.0, "alt": 0.0, "speed": 0.0, "heading": 0.0},
    "sensors": {"pm25": 0.0, "eco2": 0.0, "tvoc": 0.0, "temp": 0.0, "hum": 0.0, "aqi": 1},
    "mission": {"current": 0, "total": 0, "state": 0}
}

_master = None
_master_lock = threading.Lock()

def get_master():
    global _master
    if _master is None:
        try:
            _master = mavutil.mavlink_connection(SERIAL_PORT, baud=BAUD_RATE)
            _master.wait_heartbeat(timeout=5)
            print("[MAVLink] Connected!")
        except Exception as e:
            print(f"[MAVLink Error] {e}")
    return _master

# ── THREAD: Đọc MAVLink liên tục ──────────────────────────────────────────────
def mavlink_reader():
    m = get_master()
    while True:
        if m is None:
            time.sleep(1)
            m = get_master()
            continue
            
        msg = m.recv_match(blocking=True)
        if not msg: continue
        
        msg_type = msg.get_type()
        state["timestamp"] = time.strftime('%Y-%m-%d %H:%M:%S')

        if msg_type == 'GLOBAL_POSITION_INT':
            state["gps"]["lat"] = msg.lat / 1e7
            state["gps"]["lon"] = msg.lon / 1e7
            state["gps"]["alt"] = msg.relative_alt / 1000.0
        elif msg_type == 'VFR_HUD':
            state["gps"]["speed"] = msg.groundspeed
            state["gps"]["heading"] = msg.heading
        elif msg_type == 'BATTERY_STATUS':
            state["battery"] = msg.battery_remaining
        elif msg_type == 'HEARTBEAT':
            state["armed"] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            state["mode"] = mavutil.mode_string_v10(msg)
        elif msg_type == 'MISSION_CURRENT':
            state["mission"]["current"] = msg.seq
            state["mission"]["state"] = msg.mission_state
        elif msg_type == 'NAMED_VALUE_FLOAT':
            name = msg.name.rstrip('\x00')
            val = round(float(msg.value), 2)
            if name == 'PM25': state["sensors"]["pm25"] = val
            elif name == 'CO2': state["sensors"]["eco2"] = val
            elif name == 'TVOC': state["sensors"]["tvoc"] = val
            elif name == 'TEMP': state["sensors"]["temp"] = val
            elif name == 'HUM': state["sensors"]["hum"] = val
            elif name == 'AQI': state["sensors"]["aqi"] = int(val)

threading.Thread(target=mavlink_reader, daemon=True).start()

# ── THREAD: Backend CSV Logger ────────────────────────────────────────────────
def backend_csv_logger():
    if not os.path.exists('logs'): os.makedirs('logs')
    filename = f"logs/uav_flight_{time.strftime('%Y%m%d_%H%M')}.csv"
    
    with open(filename, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["Timestamp", "Lat", "Lon", "Alt(m)", "Speed(m/s)", "PM2.5", "eCO2", "TVOC", "Temp", "Hum", "AQI"])
        
    while True:
        time.sleep(2)
        if state["gps"]["lat"] != 0.0:
            with open(filename, mode='a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    state["timestamp"], state["gps"]["lat"], state["gps"]["lon"], state["gps"]["alt"], state["gps"]["speed"],
                    state["sensors"]["pm25"], state["sensors"]["eco2"], state["sensors"]["tvoc"],
                    state["sensors"]["temp"], state["sensors"]["hum"], state["sensors"]["aqi"]
                ])

threading.Thread(target=backend_csv_logger, daemon=True).start()

# ── API ENDPOINTS ─────────────────────────────────────────────────────────────
@app.route("/")
def home(): return render_template("index.html")

@app.route("/telemetry")
def telemetry(): return render_template("dashboard.html")

@app.route("/stream")
def stream(): return render_template("stream.html")

@app.route("/api/uav-state")
def uav_state():
    return jsonify({"success": True, "state": state})

# ── Camera ────────────────────────────────────────────────────────────────────
def gen_frames():
    try:
        from picamera2 import Picamera2
        cam = Picamera2()
        cam.configure(cam.create_preview_configuration(main={"size": (640, 480)}))
        cam.start()
        while True:
            frame = cam.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            # HUD Overlay Camera
            txt = f"AQI:{state['sensors']['aqi']} PM2.5:{state['sensors']['pm25']} Alt:{state['gps']['alt']:.1f}m"
            cv2.putText(frame, txt, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            _, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buf.tobytes() + b'\r\n')
    except:
        pass

@app.route("/video_feed")
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

# ── Controls ──────────────────────────────────────────────────────────────────
@app.route("/arm", methods=['POST'])
def arm():
    with _master_lock: 
        get_master().arducopter_arm()
    return jsonify({"success": True})

@app.route("/disarm", methods=['POST'])
def disarm():
    with _master_lock: 
        get_master().arducopter_disarm()
    return jsonify({"success": True})

@app.route("/set-mode", methods=['POST'])
def set_mode():
    mode = request.get_json().get('mode', 'RTL')
    with _master_lock: 
        get_master().set_mode(mode)
    return jsonify({"success": True})

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, threaded=True)