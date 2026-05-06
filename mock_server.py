"""
UAV Dashboard - Mock Server (Đã sửa lỗi chuyển waypoint)
"""
from flask import Flask, request, jsonify, render_template, Response, send_file
from flask_cors import CORS
import time
import threading
import math
import json
import csv
import os
import random
from datetime import datetime
from io import BytesIO
from PIL import Image, ImageDraw

app = Flask(__name__)
CORS(app)

WAYPOINT_RADIUS = 5.0
HOLD_TIME = 10.0
SAMPLE_COUNT = 10
SAMPLE_INTERVAL = 1.0
MAX_LOG_SIZE = 10 * 1024 * 1024

FLIGHT_SPEED = 0.00002
                           
FLIGHT_ALT_SPEED = 0.01

MAX_SPEED_MS = 3.0

UPDATE_INTERVAL = 0.1 

DATA_DIR = "uav_data"
CAMERA_DIR = "camera_captures"
os.makedirs(DATA_DIR, exist_ok=True)
os.makedirs(CAMERA_DIR, exist_ok=True)

# ==================== TRẠNG THÁI ====================
simulation_running = True
data_logging_enabled = True
current_waypoints = []
current_waypoint_index = 0
is_flying = False
is_holding = False
hold_start_time = 0
hold_samples_collected = 0
collected_readings = []

current_position = {
    "lat": 16.074353668716064,
    "lon": 108.15225143177362,
    "alt": 30.0,
    "heading": 0.0,
    "speed": 0.0
}

latest_sensor_data = {
    "pm25": 45.2,
    "eco2": 520,
    "tvoc": 180,
    "temp": 28.5,
    "hum": 65.0,
    "aqi": 2
}

collected_data_history = []
camera_recording = False
recorded_frames = []

data_lock = threading.Lock()

def append_to_event_log(message, type_="info"):
    timestamp = datetime.now().strftime("%H:%M:%S")
    print(f"[{timestamp}] [{type_.upper()}] {message}")

def save_reading_to_log(reading):
    csv_path = os.path.join(DATA_DIR, "sensor_readings.csv")
    file_exists = os.path.exists(csv_path)
    with open(csv_path, 'a', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=reading.keys())
        if not file_exists:
            writer.writeheader()
        writer.writerow(reading)
    
    json_path = os.path.join(DATA_DIR, "sensor_readings.jsonl")
    with open(json_path, 'a', encoding='utf-8') as f:
        f.write(json.dumps(reading, ensure_ascii=False) + '\n')

def calculate_distance(lat1, lon1, lat2, lon2):
    R = 6371000
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    a = math.sin(delta_phi/2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return R * c

def update_sensor_data():
    global latest_sensor_data
    alt_factor = max(0, min(1, current_position["alt"] / 100.0))
    with data_lock:
        latest_sensor_data["pm25"] = max(5, min(300, latest_sensor_data["pm25"] + random.uniform(-5, 8) - alt_factor * 3))
        latest_sensor_data["eco2"] = max(350, min(3000, latest_sensor_data["eco2"] + random.uniform(-20, 35) - alt_factor * 50))
        latest_sensor_data["tvoc"] = max(0, min(800, latest_sensor_data["tvoc"] + random.uniform(-15, 25) - alt_factor * 40))
        latest_sensor_data["temp"] = max(20, min(40, latest_sensor_data["temp"] + random.uniform(-0.3, 0.5)))
        latest_sensor_data["hum"] = max(30, min(85, latest_sensor_data["hum"] + random.uniform(-1, 1.5)))
        pm = latest_sensor_data["pm25"]
        if pm <= 25: aqi = 1
        elif pm <= 50: aqi = 2
        elif pm <= 100: aqi = 3
        elif pm <= 200: aqi = 4
        else: aqi = 5
        latest_sensor_data["aqi"] = aqi

def perform_sensor_reading_at_waypoint():
    global is_holding, hold_samples_collected, collected_readings, current_waypoint_index, is_flying
    
    if not is_holding:
        return
    
    # Nếu đã đọc đủ số lần
    if hold_samples_collected >= SAMPLE_COUNT:
        # Lưu dữ liệu trung bình
        if collected_readings and data_logging_enabled:
            avg_reading = {
                "lat": current_position["lat"],
                "lon": current_position["lon"],
                "alt": current_position["alt"],
                "time": datetime.now().isoformat(),
                "waypoint_index": current_waypoint_index,
                "pm25": sum(r["pm25"] for r in collected_readings) / len(collected_readings),
                "eco2": sum(r["eco2"] for r in collected_readings) / len(collected_readings),
                "tvoc": sum(r["tvoc"] for r in collected_readings) / len(collected_readings),
                "temp": sum(r["temp"] for r in collected_readings) / len(collected_readings),
                "hum": sum(r["hum"] for r in collected_readings) / len(collected_readings),
                "aqi": max(1, min(5, round(sum(r["aqi"] for r in collected_readings) / len(collected_readings)))),
                "sample_count": len(collected_readings)
            }
            with data_lock:
                collected_data_history.append(avg_reading)
                save_reading_to_log(avg_reading)
            append_to_event_log(f"✅ Waypoint {current_waypoint_index + 1}: Thu thập {len(collected_readings)} mẫu | PM2.5: {avg_reading['pm25']:.1f} | AQI: {avg_reading['aqi']}", "success")
        
        # KẾT THÚC HOLDING - CHUYỂN SANDPOINT TIẾP THEO
        is_holding = False
        hold_samples_collected = 0
        collected_readings = []
        
        # TĂNG CHỈ SỐ WAYPOINT
        current_waypoint_index += 1
        
        # KIỂM TRA XEM CÒN WAYPOINT KHÔNG
        if current_waypoint_index >= len(current_waypoints):
            is_flying = False
            current_position["speed"] = 0
            append_to_event_log("🏁 Hoàn thành tất cả waypoint! Mission kết thúc.", "success")
        else:
            append_to_event_log(f"✈️ Rời waypoint {current_waypoint_index}, di chuyển đến waypoint {current_waypoint_index + 1}", "info")
        return
    
    # Đọc cảm biến
    update_sensor_data()
    reading = {
        "timestamp": datetime.now().isoformat(),
        "lat": current_position["lat"],
        "lon": current_position["lon"],
        "alt": current_position["alt"],
        "waypoint_index": current_waypoint_index,
        **latest_sensor_data.copy()
    }
    collected_readings.append(reading)
    
    if data_logging_enabled:
        save_reading_to_log(reading)
        append_to_event_log(f"📊 Đọc {hold_samples_collected + 1}/{SAMPLE_COUNT} tại WP{current_waypoint_index + 1}: PM2.5={reading['pm25']:.1f}, eCO₂={reading['eco2']}, AQI={reading['aqi']}", "info")
    
    hold_samples_collected += 1

def simulation_loop():
    global current_position, current_waypoint_index, is_flying, is_holding, hold_start_time
    
    last_sensor_update = time.time()
    last_position_log = time.time()
    
    while simulation_running:
        current_time = time.time()
        
        # Cập nhật cảm biến định kỳ
        if current_time - last_sensor_update >= 2.0:
            update_sensor_data()
            last_sensor_update = current_time
        
        # XỬ LÝ TRẠNG THÁI HOLD (đang dừng đọc cảm biến)
        if is_holding:
            # Kiểm tra thời gian dừng
            if current_time - hold_start_time >= HOLD_TIME:
                # Đã hết thời gian, gọi hàm kết thúc (sẽ tự động tăng waypoint)
                perform_sensor_reading_at_waypoint()
            else:
                # Đang trong thời gian dừng - thực hiện đọc cảm biến theo chu kỳ
                sample_interval = HOLD_TIME / SAMPLE_COUNT
                expected_sample_index = int((current_time - hold_start_time) / sample_interval)
                
                if expected_sample_index > hold_samples_collected and hold_samples_collected < SAMPLE_COUNT:
                    perform_sensor_reading_at_waypoint()
                
                time.sleep(0.1)
            continue
        
        # XỬ LÝ BAY THEO WAYPOINT
        if is_flying and current_waypoints and current_waypoint_index < len(current_waypoints):
            target = current_waypoints[current_waypoint_index]
            distance = calculate_distance(
                current_position["lat"], current_position["lon"],
                target["lat"], target["lng"]
            )
            
            # Log khoảng cách mỗi 5 giây để debug
            if current_time - last_position_log > 5:
                append_to_event_log(f"📍 WP{current_waypoint_index + 1}: Còn {distance:.1f}m", "info")
                last_position_log = current_time
            
            # KIỂM TRA NẾU ĐẾN GẦN WAYPOINT
            if distance <= WAYPOINT_RADIUS and not is_holding:
                append_to_event_log(
                    f"📍 Đến waypoint {current_waypoint_index + 1} (cách {distance:.1f}m). "
                    f"Dừng {HOLD_TIME}s để thu thập dữ liệu...",
                    "success"
                )
                is_holding = True
                hold_start_time = current_time
                hold_samples_collected = 0
                collected_readings = []
                time.sleep(0.5)
                continue
            
            # DI CHUYỂN VỀ PHÍA WAYPOINT - SỬ DỤNG FLIGHT_SPEED MỚI
            if distance > 0.5:
                # Tính toán hướng di chuyển
                dx = target["lat"] - current_position["lat"]
                dy = target["lng"] - current_position["lon"]
                dist_deg = math.sqrt(dx*dx + dy*dy)
                
                if dist_deg > 0:
                    # SỬ DỤNG FLIGHT_SPEED ĐÃ ĐIỀU CHỈNH
                    step = min(FLIGHT_SPEED, dist_deg)
                    current_position["lat"] += dx / dist_deg * step
                    current_position["lon"] += dy / dist_deg * step
                    
                    # SỬ DỤNG MAX_SPEED_MS MỚI
                    current_position["speed"] = random.uniform(1, MAX_SPEED_MS)
                    
                    # Tính heading
                    current_position["heading"] = math.degrees(math.atan2(dy, dx))
                
                # Cập nhật độ cao - SỬ DỤNG FLIGHT_ALT_SPEED MỚI
                target_alt = target.get("alt", 30)
                alt_diff = target_alt - current_position["alt"]
                if abs(alt_diff) > 0.5:
                    current_position["alt"] += alt_diff * FLIGHT_ALT_SPEED
                else:
                    current_position["alt"] = target_alt
            else:
                # Đã đến rất gần nhưng chưa trong bán kính - tiếp tục di chuyển với tốc độ chậm
                pass
        
        time.sleep(UPDATE_INTERVAL)  # SỬ DỤNG UPDATE_INTERVAL MỚI
    
    append_to_event_log("⏹️ Simulation loop đã dừng", "warn")

# Khởi động simulation thread
sim_thread = threading.Thread(target=simulation_loop, daemon=True)
sim_thread.start()

# ==================== ROUTES ====================
@app.route("/")
def home():
    return render_template("index.html")

@app.route("/telemetry")
def telemetry():
    return render_template("dashboard.html")

@app.route("/stream")
def stream():
    return render_template("stream.html")

@app.route("/vehicle-position")
def vehicle_position():
    return jsonify({"success": True, "lat": current_position["lat"], "lon": current_position["lon"], "alt": current_position["alt"]})

@app.route("/vehicle-info")
def vehicle_info():
    return jsonify({
        "success": True,
        "alt": round(current_position["alt"], 1),
        "speed": round(current_position["speed"], 2),
        "heading": round(current_position["heading"], 1),
        "battery": round(random.uniform(65, 95), 1),
        "mode": "AUTO" if is_flying else ("HOLD" if is_holding else "GUIDED"),
        "armed": is_flying or is_holding
    })

@app.route("/api/telemetry")
def api_telemetry():
    with data_lock:
        return jsonify({
            "success": True,
            "telemetry": {
                "pm25": latest_sensor_data.get("pm25", 45.2),
                "pm10": latest_sensor_data.get("pm25", 45.2) * 1.5,
                "eco2": latest_sensor_data.get("eco2", 520),
                "co2": latest_sensor_data.get("eco2", 520),
                "tvoc": latest_sensor_data.get("tvoc", 180),
                "temp": latest_sensor_data.get("temp", 28.5),
                "hum": latest_sensor_data.get("hum", 65.0),
                "aqi": latest_sensor_data.get("aqi", 2),
                "alt": current_position["alt"],
                "speed": current_position["speed"],
                "heading": current_position["heading"],
                "armed": is_flying or is_holding,
                "mode": "AUTO" if is_flying else ("HOLD" if is_holding else "GUIDED"),
                "is_holding": is_holding,
                "hold_samples": hold_samples_collected if is_holding else 0,
                "hold_total": SAMPLE_COUNT if is_holding else 0,
                "current_wp": current_waypoint_index + 1 if current_waypoint_index < len(current_waypoints) else len(current_waypoints),
                "total_wp": len(current_waypoints)
            }
        })

@app.route("/api/uav-state")
def api_uav_state():
    with data_lock:
        return jsonify({
            "success": True,
            "battery": round(random.uniform(65, 95), 1),
            "altitude": current_position["alt"],
            "speed": current_position["speed"],
            "heading": current_position["heading"],
            "mode": "AUTO" if is_flying else ("HOLD" if is_holding else "GUIDED"),
            "armed": is_flying or is_holding,
            "latitude": current_position["lat"],
            "longitude": current_position["lon"],
            "satellites": random.randint(8, 14),
            "pm25": latest_sensor_data.get("pm25", 45.2),
            "pm10": latest_sensor_data.get("pm25", 45.2) * 1.5,
            "co2": latest_sensor_data.get("eco2", 520),
            "tvoc": latest_sensor_data.get("tvoc", 180),
            "temperature": latest_sensor_data.get("temp", 28.5),
            "humidity": latest_sensor_data.get("hum", 65.0),
            "aqi": latest_sensor_data.get("aqi", 2),
            "current_waypoint": current_waypoint_index,
            "total_waypoints": len(current_waypoints),
            "is_flying": is_flying,
            "is_holding": is_holding
        })

@app.route("/mission-progress")
def mission_progress():
    return jsonify({
        "success": True,
        "mission_total": len(current_waypoints),
        "mission_current": current_waypoint_index if current_waypoint_index <= len(current_waypoints) else len(current_waypoints),
        "mission_state": 3 if is_flying else (2 if is_holding else 5)
    })

@app.route("/upload-mission", methods=["POST"])
def upload_mission():
    global current_waypoints, current_waypoint_index, is_flying, is_holding, current_position
    
    data = request.get_json()
    if not data or "mission" not in data:
        return jsonify({"success": False, "message": "Missing mission data"}), 400
    
    with data_lock:
        current_waypoints = data["mission"]
        current_waypoint_index = 0
        is_flying = False
        is_holding = False
        
        # Đặt vị trí bắt đầu là waypoint đầu tiên
        if current_waypoints:
            current_position["lat"] = current_waypoints[0]["lat"]
            current_position["lon"] = current_waypoints[0]["lng"]
            current_position["alt"] = current_waypoints[0].get("alt", 30)
    
    append_to_event_log(f"📤 Upload {len(current_waypoints)} waypoints thành công", "success")
    return jsonify({"success": True, "message": f"Uploaded {len(current_waypoints)} waypoints"})

@app.route("/start-mission", methods=["POST"])
def start_mission():
    global is_flying, current_waypoint_index, is_holding
    
    if not current_waypoints:
        return jsonify({"success": False, "message": "No waypoints uploaded"}), 400
    
    with data_lock:
        is_flying = True
        is_holding = False
        current_waypoint_index = 0
        
        # Đặt lại vị trí về waypoint đầu tiên
        if current_waypoints:
            current_position["lat"] = current_waypoints[0]["lat"]
            current_position["lon"] = current_waypoints[0]["lng"]
            current_position["alt"] = current_waypoints[0].get("alt", 30)
    
    append_to_event_log("🚀 Bắt đầu mission! UAV đang di chuyển đến waypoint đầu tiên", "success")
    return jsonify({"success": True, "message": "Mission started"})

@app.route("/stop-mission", methods=["POST"])
def stop_mission():
    global is_flying, is_holding
    with data_lock:
        is_flying = False
        is_holding = False
    append_to_event_log("⏸️ Mission tạm dừng", "warn")
    return jsonify({"success": True, "message": "Mission stopped"})

@app.route("/clear-mission", methods=["POST"])
def clear_mission():
    global current_waypoints, current_waypoint_index, is_flying, is_holding
    with data_lock:
        current_waypoints = []
        current_waypoint_index = 0
        is_flying = False
        is_holding = False
    append_to_event_log("🗑️ Đã xóa toàn bộ waypoint", "info")
    return jsonify({"success": True, "message": "Mission cleared"})

@app.route("/get-mission")
def get_mission():
    return jsonify({"success": True, "mission": current_waypoints})

@app.route("/set-data-logging", methods=["POST"])
def set_data_logging():
    global data_logging_enabled
    data = request.get_json()
    data_logging_enabled = data.get("enabled", True)
    append_to_event_log(f"📝 Ghi dữ liệu cảm biến: {'BẬT' if data_logging_enabled else 'TẮT'}", "info")
    return jsonify({"success": True, "enabled": data_logging_enabled})

@app.route("/get-data-logging")
def get_data_logging():
    return jsonify({"success": True, "enabled": data_logging_enabled})

@app.route("/get-collected-data")
def get_collected_data():
    with data_lock:
        return jsonify({"success": True, "data": collected_data_history})

@app.route("/export-csv")
def export_csv():
    csv_path = os.path.join(DATA_DIR, "sensor_readings.csv")
    if not os.path.exists(csv_path):
        return jsonify({"success": False, "message": "No data available"}), 404
    return send_file(csv_path, as_attachment=True, download_name=f"uav_sensor_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")

@app.route("/export-json")
def export_json():
    json_path = os.path.join(DATA_DIR, "sensor_readings.jsonl")
    if not os.path.exists(json_path):
        return jsonify({"success": False, "message": "No data available"}), 404
    data = []
    with open(json_path, 'r', encoding='utf-8') as f:
        for line in f:
            if line.strip():
                data.append(json.loads(line))
    return jsonify({"success": True, "data": data})

@app.route("/video_feed")
def video_feed():
    def generate():
        frame_count = 0
        while True:
            img = Image.new('RGB', (640, 480), color=(30, 30, 50))
            draw = ImageDraw.Draw(img)
            draw.text((50, 50), f"UAV SIMULATION MODE", fill=(0, 255, 0))
            draw.text((50, 100), f"Lat: {current_position['lat']:.6f}", fill=(255, 255, 255))
            draw.text((50, 130), f"Lon: {current_position['lon']:.6f}", fill=(255, 255, 255))
            draw.text((50, 160), f"Alt: {current_position['alt']:.1f}m", fill=(255, 255, 255))
            draw.text((50, 190), f"Heading: {current_position['heading']:.0f}°", fill=(255, 255, 255))
            
            if is_flying:
                status = f"FLYING to WP{current_waypoint_index + 1}/{len(current_waypoints)}"
                status_color = (0, 255, 255)
            elif is_holding:
                status = f"HOLDING at WP{current_waypoint_index + 1} - Collecting data {hold_samples_collected}/{SAMPLE_COUNT}"
                status_color = (255, 255, 0)
            else:
                status = "IDLE"
                status_color = (255, 0, 0)
            
            draw.text((50, 230), f"Status: {status}", fill=status_color)
            draw.text((50, 440), datetime.now().strftime("%Y-%m-%d %H:%M:%S"), fill=(200, 200, 200))
            
            if camera_recording:
                draw.text((550, 50), "● REC", fill=(255, 0, 0))
            
            img_byte_arr = BytesIO()
            img.save(img_byte_arr, format='JPEG', quality=70)
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + img_byte_arr.getvalue() + b'\r\n')
            frame_count += 1
            time.sleep(0.033)
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route("/camera/snapshot", methods=["POST"])
def camera_snapshot():
    try:
        img = Image.new('RGB', (640, 480), color=(30, 30, 50))
        draw = ImageDraw.Draw(img)
        draw.text((50, 50), f"SNAPSHOT - {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}", fill=(0, 255, 0))
        draw.text((50, 100), f"Lat: {current_position['lat']:.6f} Lon: {current_position['lon']:.6f}", fill=(255, 255, 255))
        draw.text((50, 130), f"Alt: {current_position['alt']:.1f}m", fill=(255, 255, 255))
        draw.text((50, 160), f"PM2.5: {latest_sensor_data['pm25']:.1f} | AQI: {latest_sensor_data['aqi']}", fill=(255, 255, 255))
        filename = f"snapshot_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg"
        filepath = os.path.join(CAMERA_DIR, filename)
        img.save(filepath, 'JPEG', quality=85)
        return jsonify({"success": True, "filename": filename, "path": filepath})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)}), 500

@app.route("/camera/record/start", methods=["POST"])
def camera_record_start():
    global camera_recording, recorded_frames
    camera_recording = True
    recorded_frames = []
    return jsonify({"success": True, "recording": True})

@app.route("/camera/record/stop", methods=["POST"])
def camera_record_stop():
    global camera_recording, recorded_frames
    camera_recording = False
    if recorded_frames:
        try:
            filename = f"recording_{datetime.now().strftime('%Y%m%d_%H%M%S')}.gif"
            filepath = os.path.join(CAMERA_DIR, filename)
            if recorded_frames:
                recorded_frames[0].save(filepath, save_all=True, append_images=recorded_frames[1:20], duration=50, loop=0)
            recorded_frames = []
            return jsonify({"success": True, "filename": filename})
        except Exception as e:
            return jsonify({"success": False, "error": str(e)}), 500
    return jsonify({"success": True, "message": "No frames recorded"})

@app.route("/arm", methods=["POST"])
def arm():
    global is_flying
    if current_waypoints and not is_flying:
        is_flying = True
    return jsonify({"success": True, "message": "Armed"})

@app.route("/disarm", methods=["POST"])
def disarm():
    global is_flying, is_holding
    is_flying = False
    is_holding = False
    return jsonify({"success": True, "message": "Disarmed"})

@app.route("/rtl", methods=["POST"])
def rtl():
    global is_flying, is_holding
    is_flying = True
    is_holding = False
    return jsonify({"success": True, "message": "Return to Launch"})

@app.route("/set-mode", methods=["POST"])
def set_mode():
    return jsonify({"success": True, "mode": request.get_json().get("mode", "GUIDED")})

if __name__ == "__main__":
    print("=" * 60)
    print("  🚁 UAV Ground Control Station - MOCK SERVER")
    print("  📍 http://localhost:5000")
    print("  📊 Cấu hình test: Bán kính=5m, Dừng=10s, Số mẫu=10")
    print("=" * 60)
    app.run(host="0.0.0.0", port=5000, debug=True, threaded=True)