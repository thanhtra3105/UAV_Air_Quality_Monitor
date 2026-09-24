"""
UAV Dashboard - Realistic Ground Control Station Mock Server
Simulates full flight dynamics, telemetry, air quality sensors (ENS160 + AHT21 + Optical PM + Gas),
accurate waypoint navigation, holding/sampling states, and video feed with Cockpit HUD.
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
from PIL import Image, ImageDraw, ImageFont

app = Flask(__name__)
CORS(app)

# ==================== CẤU HÌNH THỜI GIAN & THÔNG SỐ BAY ====================
WAYPOINT_RADIUS = 5.0      # Bán kính đạt waypoint (mét)
HOLD_TIME = 10.0            # Thời gian dừng tại mỗi waypoint (giây)
SAMPLE_COUNT = 10          # Số lượng mẫu cảm biến thu thập trong lúc hold
SAMPLE_INTERVAL = HOLD_TIME / SAMPLE_COUNT
MAX_SPEED_MS = 4.5         # Vận tốc bay tối đa (m/s)
FLIGHT_SPEED_DEG = 0.000035 # Bước di chuyển tọa độ mỗi tick ~3-4 m/s
UPDATE_INTERVAL = 0.08     # Chu kỳ cập nhật simulation loop (12.5 Hz)
CRUISE_ALTITUDE = 35.0     # Độ cao hành trình tiêu chuẩn (35m khớp đồ thị mẫu)
TERRAIN_ALTITUDE = 7.0     # Độ cao địa hình mặt đất (7m khớp đồ thị mẫu)
TAKEOFF_CLIMB_RATE = 3.5   # Vận tốc leo cao khi cất cánh (m/s)
LANDING_DESCENT_RATE = 2.5 # Vận tốc hạ cánh (m/s)

DATA_DIR = "uav_data"
CAMERA_DIR = "camera_captures"
WP_DATA_DIR = "wp_data"
os.makedirs(DATA_DIR, exist_ok=True)
os.makedirs(CAMERA_DIR, exist_ok=True)
os.makedirs(WP_DATA_DIR, exist_ok=True)

# ==================== TRẠNG THÁI HỆ THỐNG ====================
data_lock = threading.Lock()
simulation_running = True
data_logging_enabled = True
camera_recording = False
recorded_frames = []

is_armed = False
current_mode = "MANUAL"     # MANUAL, GUIDED, AUTO, HOLD, RTL
flight_state = "IDLE"       # IDLE, TAKEOFF, RUNNING, HOLD, LANDING
is_flying = False
is_holding = False
hold_start_time = 0
hold_samples_collected = 0
flight_start_time = 0
total_flight_seconds = 0

current_waypoints = []
current_waypoint_index = 0
collected_readings = []
collected_data_history = []
wp_csv_files = []

# Tọa độ ban đầu (Khu vực ĐH Bách Khoa Đà Nẵng / Vịnh Đà Nẵng)
HOME_POSITION = {
    "lat": 16.0743537,
    "lon": 108.1522514,
    "alt": 0.0
}

current_position = {
    "lat": 16.0743537,
    "lon": 108.1522514,
    "alt": 0.0,
    "heading": 45.0,
    "speed": 0.0
}

attitude = {
    "pitch": 0.0,
    "roll": 0.0,
    "yaw": 45.0
}

# 4S LiPo Battery Model: 16.8V (100%) -> 14.8V (20%) -> 14.0V (0%)
battery_state = {
    "pct": 94.5,
    "voltage": 16.48,
    "current": 0.8,
    "mah_used": 150.0
}

latest_sensor_data = {
    "pm25": 38.4,
    "pm10": 57.6,
    "eco2": 490,
    "tvoc": 120,
    "co": 0.65,
    "no2": 22.0,
    "temp": 28.6,
    "hum": 68.2,
    "aqi": 77,
    "aqi_category": "Trung bình",
    "aqi_main_pollutant": "PM2.5",
    "aqi_components": {
        "pm25": 77,
        "pm10": 54,
        "eco2": 41,
        "tvoc": 35,
        "co": 7,
        "no2": 11,
        "level": 2,
        "color": "#eab308"
    }
}

# ==================== HÀM TIỆN ÍCH & AQI ====================
def calculate_distance(lat1, lon1, lat2, lon2):
    """Tính khoảng cách Haversine giữa 2 tọa độ GPS (mét)."""
    R = 6371000
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    a = math.sin(delta_phi/2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c

def calculate_bearing(lat1, lon1, lat2, lon2):
    """Tính góc phương vị từ điểm 1 đến điểm 2 (0..360 độ)."""
    y = math.sin(math.radians(lon2 - lon1)) * math.cos(math.radians(lat2))
    x = math.cos(math.radians(lat1)) * math.sin(math.radians(lat2)) - \
        math.sin(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.cos(math.radians(lon2 - lon1))
    bearing = (math.degrees(math.atan2(y, x)) + 360) % 360
    return bearing

def compute_vietnam_aqi(pm25, pm10, co, no2, eco2):
    """
    Tính chỉ số AQI và cấp chất lượng không khí chuẩn Việt Nam (VN_AQI 0-500)
    Quy định theo Hướng dẫn kỹ thuật tính toán và công bố chỉ số VN_AQI của Tổng cục Môi trường
    & QCVN 05:2023/BTNM.
    """
    def calc_sub_aqi(c, breakpoints):
        if c <= 0:
            return 0
        for b_low, b_high, i_low, i_high in breakpoints:
            if c <= b_high:
                return round(((i_high - i_low) / (b_high - b_low)) * (c - b_low) + i_low)
        return breakpoints[-1][3]

    # Breakpoints chuẩn Việt Nam (VN_AQI)
    # PM2.5 (µg/m³) -> I (0-500)
    bp_pm25 = [
        (0.0, 25.0, 0, 50),
        (25.0, 50.0, 50, 100),
        (50.0, 80.0, 100, 150),
        (80.0, 150.0, 150, 200),
        (150.0, 250.0, 200, 300),
        (250.0, 350.0, 300, 400),
        (350.0, 500.0, 400, 500)
    ]
    # PM10 (µg/m³)
    bp_pm10 = [
        (0.0, 50.0, 0, 50),
        (50.0, 150.0, 50, 100),
        (150.0, 250.0, 100, 150),
        (250.0, 350.0, 150, 200),
        (350.0, 420.0, 200, 300),
        (420.0, 500.0, 300, 400),
        (500.0, 600.0, 400, 500)
    ]
    # CO (mg/m³)
    bp_co = [
        (0.0, 5.0, 0, 50),
        (5.0, 10.0, 50, 100),
        (10.0, 25.0, 100, 150),
        (25.0, 45.0, 150, 200),
        (45.0, 60.0, 200, 300),
        (60.0, 90.0, 300, 400),
        (90.0, 120.0, 400, 500)
    ]
    # NO2 (µg/m³)
    bp_no2 = [
        (0.0, 100.0, 0, 50),
        (100.0, 200.0, 50, 100),
        (200.0, 700.0, 100, 150),
        (700.0, 1200.0, 150, 200),
        (1200.0, 2350.0, 200, 300)
    ]
    # eCO2 (mg/m³)
    bp_eco2 = [
        (0.0, 600.0, 0, 50),
        (600.0, 1000.0, 50, 100),
        (1000.0, 1500.0, 100, 150),
        (1500.0, 2000.0, 150, 200),
        (2000.0, 3000.0, 200, 300)
    ]

    i_pm25 = calc_sub_aqi(pm25, bp_pm25)
    i_pm10 = calc_sub_aqi(pm10, bp_pm10)
    i_co = calc_sub_aqi(co, bp_co)
    i_no2 = calc_sub_aqi(no2, bp_no2)
    i_eco2 = calc_sub_aqi(eco2, bp_eco2)

    sub_dict = {
        "PM2.5": i_pm25,
        "PM10": i_pm10,
        "CO": i_co,
        "NO2": i_no2,
        "CO2": i_eco2
    }

    # Chất ô nhiễm chủ đạo
    main_pol = max(sub_dict, key=sub_dict.get)
    overall_aqi = max(0, min(500, sub_dict[main_pol]))

    # Phân cấp chất lượng không khí chuẩn Việt Nam:
    if overall_aqi <= 50:
        cat = "Tốt"
        level = 1
        color = "#157a3a"
    elif overall_aqi <= 100:
        cat = "Trung bình"
        level = 2
        color = "#eab308"
    elif overall_aqi <= 150:
        cat = "Kém"
        level = 3
        color = "#f97316"
    elif overall_aqi <= 200:
        cat = "Xấu"
        level = 4
        color = "#ef4444"
    elif overall_aqi <= 300:
        cat = "Rất xấu"
        level = 5
        color = "#a855f7"
    else:
        cat = "Nguy hại"
        level = 6
        color = "#7e0023"

    return overall_aqi, cat, main_pol, {
        "pm25": i_pm25,
        "pm10": i_pm10,
        "co": i_co,
        "no2": i_no2,
        "eco2": i_eco2,
        "level": level,
        "color": color
    }

def append_to_event_log(message, type_="info"):
    timestamp = datetime.now().strftime("%H:%M:%S")
    print(f"[{timestamp}] [{type_.upper()}] {message}")

def save_reading_to_log(reading):
    """Lưu dữ liệu sensor vào CSV & JSONL."""
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

SUMMARY_CSV_PATH = os.path.join(WP_DATA_DIR, "waypoint_history_summary.csv")
HISTORY_JSON_PATH = os.path.join(WP_DATA_DIR, "waypoint_history.json")

def save_to_summary_history_csv(entry):
    """Ghi dữ liệu tóm tắt của một điểm đo vào file CSV tổng hợp với mã hóa UTF-8-sig (hỗ trợ Excel tiếng Việt)."""
    headers = [
        "STT", "Điểm đo", "Thời gian", "Vĩ độ (Lat)", "Kinh độ (Lon)", "Độ cao (m)", "Số mẫu",
        "PM2.5 (µg/m³)", "PM10 (µg/m³)", "CO₂ (mg/m³)", "TVOC (ppb)",
        "CO (mg/m³)", "NO₂ (µg/m³)", "Nhiệt độ (°C)", "Độ ẩm (%)", "Chỉ số VN_AQI", "Chất lượng không khí", "Tập tin chi tiết"
    ]
    file_exists = os.path.exists(SUMMARY_CSV_PATH) and os.path.getsize(SUMMARY_CSV_PATH) > 0
    with open(SUMMARY_CSV_PATH, "a", newline="", encoding="utf-8-sig") as f:
        writer = csv.writer(f)
        if not file_exists:
            writer.writerow(headers)
        writer.writerow([
            len(wp_csv_files),
            f"Điểm #{entry['wp']}",
            entry["time"],
            entry["lat"],
            entry["lon"],
            entry["alt"],
            entry["samples"],
            entry.get("pm25", 0),
            entry.get("pm10", 0),
            entry.get("eco2", 0),
            entry.get("tvoc", 0),
            entry.get("co", 0),
            entry.get("no2", 0),
            entry.get("temp", 0),
            entry.get("hum", 0),
            entry.get("aqi", 1),
            entry.get("aqi_category", "Tốt"),
            entry.get("filename", "")
        ])

def save_to_history_json():
    """Lưu trữ lịch sử ra JSON để duy trì dữ liệu qua các lần khởi động server."""
    try:
        with open(HISTORY_JSON_PATH, "w", encoding="utf-8") as f:
            json.dump({
                "collected_data_history": collected_data_history,
                "wp_csv_files": wp_csv_files
            }, f, ensure_ascii=False, indent=2)
    except Exception as e:
        print("[STORAGE] Lỗi ghi JSON lịch sử:", e)

def load_history_from_disk():
    """Tự động khôi phục dữ liệu đã đo đạc khi khởi động server (từ JSON hoặc quét file CSV)."""
    global collected_data_history, wp_csv_files
    if os.path.exists(HISTORY_JSON_PATH):
        try:
            with open(HISTORY_JSON_PATH, "r", encoding="utf-8") as f:
                d = json.load(f)
                collected_data_history = d.get("collected_data_history", [])
                wp_csv_files = d.get("wp_csv_files", [])
                if collected_data_history:
                    print(f"[STORAGE] ✅ Đã khôi phục {len(collected_data_history)} điểm đo lịch sử từ JSON.")
                    return
        except Exception as e:
            print("[STORAGE] Lỗi đọc JSON lịch sử:", e)

    # Nếu chưa có file JSON, tự động khôi phục từ các file CSV hiện có trong wp_data/
    if os.path.exists(WP_DATA_DIR):
        csv_files = sorted([f for f in os.listdir(WP_DATA_DIR) if f.startswith("wp") and f.endswith(".csv") and not f.endswith("summary.csv")])
        if csv_files:
            rebuilt_history = []
            rebuilt_wp_files = []
            for fn in csv_files:
                fp = os.path.join(WP_DATA_DIR, fn)
                try:
                    with open(fp, "r", encoding="utf-8-sig", errors="ignore") as f:
                        reader = csv.DictReader(f)
                        rows = list(reader)
                        if not rows:
                            continue
                        
                        wp_num = 1
                        try:
                            wp_num = int(fn[2:4])
                        except Exception:
                            pass
                        
                        count = len(rows)
                        pm25_vals = [float(r["pm25"]) for r in rows if r.get("pm25")]
                        pm10_vals = [float(r["pm10"]) for r in rows if r.get("pm10")]
                        eco2_vals = [float(r["eco2"]) for r in rows if r.get("eco2")]
                        tvoc_vals = [float(r["tvoc"]) for r in rows if r.get("tvoc")]
                        co_vals = [float(r["co"]) for r in rows if r.get("co")]
                        no2_vals = [float(r["no2"]) for r in rows if r.get("no2")]
                        temp_vals = [float(r["temp"]) for r in rows if r.get("temp")]
                        hum_vals = [float(r["hum"]) for r in rows if r.get("hum")]
                        aqi_vals = [int(float(r["aqi"])) for r in rows if r.get("aqi")]
                        
                        lat = float(rows[-1].get("lat", 16.074))
                        lon = float(rows[-1].get("lon", 108.152))
                        alt = float(rows[-1].get("alt", 35.0))
                        time_str = rows[-1].get("timestamp", datetime.now().isoformat())
                        
                        avg_pm25 = round(sum(pm25_vals) / len(pm25_vals), 1) if pm25_vals else 0
                        avg_pm10 = round(sum(pm10_vals) / len(pm10_vals), 1) if pm10_vals else 0
                        avg_eco2 = round(sum(eco2_vals) / len(eco2_vals), 0) if eco2_vals else 0
                        avg_tvoc = round(sum(tvoc_vals) / len(tvoc_vals), 0) if tvoc_vals else 0
                        avg_co = round(sum(co_vals) / len(co_vals), 2) if co_vals else 0
                        avg_no2 = round(sum(no2_vals) / len(no2_vals), 1) if no2_vals else 0
                        avg_temp = round(sum(temp_vals) / len(temp_vals), 1) if temp_vals else 28.5
                        avg_hum = round(sum(hum_vals) / len(hum_vals), 1) if hum_vals else 68.0
                        avg_aqi = int(round(sum(aqi_vals) / len(aqi_vals))) if aqi_vals else 1
                        
                        _, cat, _, info = compute_vietnam_aqi(avg_pm25, avg_pm10, avg_co, avg_no2, avg_eco2)
                        
                        item = {
                            "lat": lat,
                            "lon": lon,
                            "alt": alt,
                            "time": time_str,
                            "waypoint_index": wp_num - 1,
                            "pm25": avg_pm25,
                            "pm10": avg_pm10,
                            "eco2": avg_eco2,
                            "tvoc": avg_tvoc,
                            "co": avg_co,
                            "no2": avg_no2,
                            "temp": avg_temp,
                            "hum": avg_hum,
                            "aqi": avg_aqi,
                            "aqi_category": cat,
                            "aqi_color": info.get("color", "#157a3a"),
                            "sample_count": count,
                            "filename": fn
                        }
                        rebuilt_history.append(item)
                        rebuilt_wp_files.append({
                            "wp": wp_num,
                            "filename": fn,
                            "samples": count,
                            "lat": round(lat, 6),
                            "lon": round(lon, 6),
                            "alt": round(alt, 1),
                            "time": time_str.replace("T", " ")[:19],
                            "pm25": avg_pm25,
                            "pm10": avg_pm10,
                            "eco2": avg_eco2,
                            "tvoc": avg_tvoc,
                            "co": avg_co,
                            "no2": avg_no2,
                            "temp": avg_temp,
                            "hum": avg_hum,
                            "aqi": avg_aqi,
                            "aqi_category": cat,
                            "aqi_color": info.get("color", "#157a3a")
                        })
                except Exception as e:
                    print(f"[STORAGE] Lỗi đọc {fn}:", e)
            
            if rebuilt_history:
                collected_data_history = rebuilt_history
                wp_csv_files = rebuilt_wp_files
                save_to_history_json()
                for entry in wp_csv_files:
                    save_to_summary_history_csv(entry)
                print(f"[STORAGE] ✅ Tự động nạp lại {len(rebuilt_history)} điểm đo từ các file CSV hiện có.")

# Khôi phục dữ liệu đã lưu từ các phiên chạy trước
load_history_from_disk()

def save_wp_raw_csv(wp_index, readings, lat, lon, alt, avg_reading=None):
    """Lưu 10 raw samples tại waypoint thành file CSV riêng trong wp_data/ và lưu vào bảng tổng hợp."""
    ts_now = datetime.now()
    ts_str = ts_now.strftime("%Y%m%d_%H%M%S")
    time_display = ts_now.strftime("%Y-%m-%d %H:%M:%S")
    filename = f"wp{wp_index + 1:02d}_{ts_str}.csv"
    filepath = os.path.join(WP_DATA_DIR, filename)
    fieldnames = ['sample', 'timestamp', 'lat', 'lon', 'alt', 'pm25', 'pm10', 'eco2', 'tvoc', 'temp', 'hum', 'aqi', 'co', 'no2']
    with open(filepath, "w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for i, r in enumerate(readings, 1):
            writer.writerow({
                "sample": i,
                "timestamp": r.get("timestamp", datetime.now().isoformat()),
                "lat": round(lat, 7),
                "lon": round(lon, 7),
                "alt": round(alt, 2),
                "pm25": round(r.get("pm25", 0), 2),
                "pm10": round(r.get("pm10", 0), 2),
                "eco2": round(r.get("eco2", 0), 1),
                "tvoc": round(r.get("tvoc", 0), 1),
                "temp": round(r.get("temp", 0), 2),
                "hum": round(r.get("hum", 0), 2),
                "aqi": int(r.get("aqi", 1)),
                "co": round(r.get("co", 0), 2),
                "no2": round(r.get("no2", 0), 1)
            })

    if avg_reading:
        avg_reading["filename"] = filename

    file_info = {
        "wp": wp_index + 1,
        "filename": filename,
        "samples": len(readings),
        "lat": round(lat, 6),
        "lon": round(lon, 6),
        "alt": round(alt, 1),
        "time": time_display,
        "pm25": round(avg_reading.get("pm25", 0), 1) if avg_reading else 0,
        "pm10": round(avg_reading.get("pm10", 0), 1) if avg_reading else 0,
        "eco2": round(avg_reading.get("eco2", 0), 0) if avg_reading else 0,
        "tvoc": round(avg_reading.get("tvoc", 0), 0) if avg_reading else 0,
        "co": round(avg_reading.get("co", 0), 2) if avg_reading else 0,
        "no2": round(avg_reading.get("no2", 0), 1) if avg_reading else 0,
        "temp": round(avg_reading.get("temp", 0), 1) if avg_reading else 0,
        "hum": round(avg_reading.get("hum", 0), 1) if avg_reading else 0,
        "aqi": avg_reading.get("aqi", 1) if avg_reading else 1,
        "aqi_category": avg_reading.get("aqi_category", "Tốt") if avg_reading else "Tốt",
        "aqi_color": avg_reading.get("aqi_color", "#157a3a") if avg_reading else "#157a3a"
    }
    wp_csv_files.append(file_info)
    save_to_summary_history_csv(file_info)
    save_to_history_json()

def update_sensor_data():
    """Mô phỏng sensor thay đổi mượt mà theo độ cao và vị trí."""
    global latest_sensor_data
    alt = current_position["alt"]
    alt_factor = max(0.0, min(1.0, alt / 80.0))

    with data_lock:
        # Nhiệt độ giảm nhẹ khi lên cao, độ ẩm thay đổi
        temp_target = 29.5 - alt_factor * 2.5 + random.uniform(-0.1, 0.1)
        hum_target = 65.0 + alt_factor * 8.0 + random.uniform(-0.5, 0.5)

        # Bụi mịn và khí giảm nhẹ theo độ cao
        pm25 = max(8.0, min(240.0, latest_sensor_data["pm25"] + random.uniform(-2.0, 3.2) - alt_factor * 0.8))
        pm10 = pm25 * random.uniform(1.4, 1.6)
        eco2 = max(380.0, min(1800.0, latest_sensor_data["eco2"] + random.uniform(-10.0, 15.0) - alt_factor * 5.0))
        tvoc = max(20.0, min(600.0, latest_sensor_data["tvoc"] + random.uniform(-8.0, 10.0) - alt_factor * 3.0))
        co = max(0.1, min(15.0, latest_sensor_data["co"] + random.uniform(-0.04, 0.05)))
        no2 = max(5.0, min(80.0, latest_sensor_data["no2"] + random.uniform(-1.0, 1.2)))

        aqi, aqi_cat, main_pol, components = compute_vietnam_aqi(pm25, pm10, co, no2, eco2)

        latest_sensor_data = {
            "pm25": round(pm25, 1),
            "pm10": round(pm10, 1),
            "eco2": round(eco2, 0),
            "tvoc": round(tvoc, 0),
            "co": round(co, 2),
            "no2": round(no2, 1),
            "temp": round(temp_target, 1),
            "hum": round(hum_target, 1),
            "aqi": aqi,
            "aqi_category": aqi_cat,
            "aqi_main_pollutant": main_pol,
            "aqi_components": components
        }

def perform_sensor_reading_at_waypoint():
    """Thu thập mẫu cảm biến khi đang Hold tại waypoint."""
    global is_holding, hold_samples_collected, collected_readings, current_waypoint_index, is_flying, current_mode, flight_state

    if not is_holding:
        return

    update_sensor_data()
    sample = {
        "timestamp": datetime.now().isoformat(),
        "lat": current_position["lat"],
        "lon": current_position["lon"],
        "alt": current_position["alt"],
        "waypoint_index": current_waypoint_index,
        **latest_sensor_data
    }
    collected_readings.append(sample)
    hold_samples_collected += 1

    if data_logging_enabled:
        save_reading_to_log(sample)

    append_to_event_log(
        f"📊 Mẫu {hold_samples_collected}/{SAMPLE_COUNT} tại WP{current_waypoint_index + 1}: "
        f"PM2.5={sample['pm25']} µg/m³ | eCO₂={sample['eco2']:.0f} mg/m³ | AQI={sample['aqi']} ({sample['aqi_category']})",
        "info"
    )

    # Đã thu thập đủ số lượng mẫu
    if hold_samples_collected >= SAMPLE_COUNT:
        if collected_readings:
            avg_aqi = int(round(sum(r["aqi"] for r in collected_readings) / len(collected_readings)))
            avg_reading = {
                "lat": current_position["lat"],
                "lon": current_position["lon"],
                "alt": current_position["alt"],
                "time": datetime.now().isoformat(),
                "waypoint_index": current_waypoint_index,
                "pm25": sum(r["pm25"] for r in collected_readings) / len(collected_readings),
                "pm10": sum(r["pm10"] for r in collected_readings) / len(collected_readings),
                "eco2": sum(r["eco2"] for r in collected_readings) / len(collected_readings),
                "tvoc": sum(r["tvoc"] for r in collected_readings) / len(collected_readings),
                "co": sum(r["co"] for r in collected_readings) / len(collected_readings),
                "no2": sum(r["no2"] for r in collected_readings) / len(collected_readings),
                "temp": sum(r["temp"] for r in collected_readings) / len(collected_readings),
                "hum": sum(r["hum"] for r in collected_readings) / len(collected_readings),
                "aqi": avg_aqi,
                "sample_count": len(collected_readings)
            }
            # Tính lại phân loại chất lượng không khí Việt Nam cho giá trị trung bình
            _, avg_cat, _, avg_info = compute_vietnam_aqi(
                avg_reading["pm25"], avg_reading["pm10"], avg_reading["co"], avg_reading["no2"], avg_reading["eco2"]
            )
            avg_reading["aqi_category"] = avg_cat
            avg_reading["aqi_color"] = avg_info.get("color", "#157a3a")

            with data_lock:
                wp_save_idx = len(wp_csv_files)
                avg_reading["point_number"] = wp_save_idx + 1
                save_wp_raw_csv(wp_save_idx, collected_readings, current_position["lat"], current_position["lon"], current_position["alt"], avg_reading)
                collected_data_history.append(avg_reading)

            append_to_event_log(
                f"✅ Hoàn tất điểm đo #{wp_save_idx + 1} (WP{current_waypoint_index + 1}): Đã lưu {len(collected_readings)} mẫu | PM2.5: {avg_reading['pm25']:.1f} µg/m³ | AQI: {avg_aqi} ({avg_cat})",
                "success"
            )

        # Chuyển waypoint tiếp theo
        is_holding = False
        hold_samples_collected = 0
        collected_readings = []
        current_waypoint_index += 1

        if current_waypoint_index >= len(current_waypoints):
            flight_state = "LANDING"
            is_flying = False
            append_to_event_log("🏁 Đã hoàn thành tất cả điểm đo! UAV bắt đầu hạ cánh (LANDING)...", "warn")
        else:
            flight_state = "RUNNING"
            is_flying = True
            current_mode = "AUTO"
            current_position["speed"] = 1.0
            append_to_event_log(f"✈️ Rời điểm #{current_waypoint_index}, tiếp tục bay đến điểm #{current_waypoint_index + 1}...", "info")

# ==================== SIMULATION THREAD ====================
def simulation_loop():
    global current_position, attitude, battery_state, current_waypoint_index
    global is_flying, is_holding, hold_start_time, current_mode, total_flight_seconds
    global flight_state, is_armed

    last_tick = time.time()
    last_sensor_tick = time.time()
    last_sample_time = 0.0

    while simulation_running:
        now = time.time()
        dt = now - last_tick
        last_tick = now

        # Cập nhật sensor định kỳ mỗi 1.5s
        if now - last_sensor_tick >= 1.5:
            update_sensor_data()
            last_sensor_tick = now

        # Tiêu hao pin thực tế
        if is_flying or flight_state in ["TAKEOFF", "RUNNING", "HOLD", "LANDING"]:
            total_flight_seconds += dt
            battery_state["current"] = 12.5 + current_position["speed"] * 1.8 + random.uniform(-0.4, 0.4)
            battery_state["pct"] = max(5.0, battery_state["pct"] - (0.012 * dt))
        elif is_armed:
            battery_state["current"] = 2.2 + random.uniform(-0.1, 0.1)
            battery_state["pct"] = max(5.0, battery_state["pct"] - (0.002 * dt))
        else:
            battery_state["current"] = 0.6 + random.uniform(-0.05, 0.05)
            battery_state["pct"] = max(5.0, battery_state["pct"] - (0.0005 * dt))

        # Tính điện áp 4S LiPo tương ứng phần trăm
        battery_state["voltage"] = round(14.2 + (battery_state["pct"] / 100.0) * 2.6, 2)

        # 1. Trạng thái TAKEOFF (Cất cánh thẳng đứng leo lên độ cao hành trình)
        if flight_state == "TAKEOFF":
            current_position["alt"] += TAKEOFF_CLIMB_RATE * dt
            current_position["speed"] = min(1.2, current_position["speed"] + 0.6 * dt)
            attitude["pitch"] = 3.5
            attitude["roll"] = 0.0

            # Định hướng mũi UAV về phía điểm đo đầu tiên ngay khi cất cánh
            if current_waypoints and len(current_waypoints) > 0:
                first_wp = current_waypoints[0]
                t_lon = first_wp.get("lng", first_wp.get("lon"))
                bearing = calculate_bearing(
                    current_position["lat"], current_position["lon"],
                    first_wp["lat"], t_lon
                )
                current_hdg = current_position["heading"]
                diff = (bearing - current_hdg + 180) % 360 - 180
                turn_step = max(-45.0 * dt, min(45.0 * dt, diff * 3.5 * dt))
                current_position["heading"] = (current_hdg + turn_step) % 360
                attitude["yaw"] = current_position["heading"]

            if current_position["alt"] >= CRUISE_ALTITUDE:
                current_position["alt"] = CRUISE_ALTITUDE
                if current_waypoints and len(current_waypoints) > 0 and current_waypoint_index < len(current_waypoints):
                    flight_state = "RUNNING"
                    is_flying = True
                    current_mode = "AUTO"
                    append_to_event_log(f"🚀 Đạt độ cao hành trình {CRUISE_ALTITUDE}m! Bắt đầu bay đến điểm đo #{current_waypoint_index + 1}", "success")
                else:
                    flight_state = "IDLE"
                    is_flying = False
                    current_mode = "LOITER"
                    append_to_event_log(f"🚀 Đạt độ cao hành trình {CRUISE_ALTITUDE}m! Đang giữ vị trí trên không (LOITER).", "info")
            time.sleep(UPDATE_INTERVAL)
            continue

        # 2. Trạng thái LANDING (Hạ cánh giảm dần độ cao xuống mặt đất)
        if flight_state == "LANDING":
            current_position["speed"] = max(0.0, current_position["speed"] - 2.0 * dt)
            current_position["alt"] = max(0.0, current_position["alt"] - LANDING_DESCENT_RATE * dt)
            attitude["pitch"] = -2.0
            attitude["roll"] = 0.0
            if current_position["alt"] <= 0.1:
                current_position["alt"] = 0.0
                current_position["speed"] = 0.0
                is_flying = False
                is_armed = False
                flight_state = "IDLE"
                current_mode = "MANUAL"
                append_to_event_log("🛬 UAV đã hạ cánh an toàn xuống mặt đất! Trạng thái IDLE (Disarmed).", "success")
            time.sleep(UPDATE_INTERVAL)
            continue

        # 3. Trạng thái HOLD (dừng đo tại waypoint)
        if flight_state == "HOLD" or is_holding:
            current_position["speed"] = 0.0
            attitude["pitch"] = 0.0
            attitude["roll"] = 0.0

            # Lấy mẫu đúng chu kỳ 1.0 giây một lần
            if now - last_sample_time >= 1.0:
                last_sample_time = now
                perform_sensor_reading_at_waypoint()

            time.sleep(UPDATE_INTERVAL)
            continue

        # 4. Trạng thái RUNNING (Bay theo Waypoint)
        if (is_flying or flight_state == "RUNNING") and current_waypoints and current_waypoint_index < len(current_waypoints):
            target = current_waypoints[current_waypoint_index]
            target_lat = target["lat"]
            target_lon = target.get("lng", target.get("lon"))
            target_alt = CRUISE_ALTITUDE

            dist_m = calculate_distance(current_position["lat"], current_position["lon"], target_lat, target_lon)

            # Đạt bán kính waypoint -> bắt đầu Hold
            if dist_m <= WAYPOINT_RADIUS and not is_holding:
                append_to_event_log(f"🎯 Đã tiếp cận điểm #{current_waypoint_index + 1} (còn {dist_m:.1f}m). Giữ vị trí (HOLD) và thu thập dữ liệu...", "success")
                current_position["lat"] = target_lat
                current_position["lon"] = target_lon
                current_position["speed"] = 0.0
                flight_state = "HOLD"
                is_holding = True
                current_mode = "HOLD"
                hold_start_time = now
                last_sample_time = now - 0.95
                hold_samples_collected = 0
                collected_readings = []
                time.sleep(UPDATE_INTERVAL)
                continue

            # Di chuyển về đích
            bearing = calculate_bearing(current_position["lat"], current_position["lon"], target_lat, target_lon)

            # Xoay heading mượt mà
            current_hdg = current_position["heading"]
            diff = (bearing - current_hdg + 180) % 360 - 180
            turn_step = max(-45.0 * dt, min(45.0 * dt, diff * 3.5 * dt))
            current_position["heading"] = (current_hdg + turn_step) % 360
            attitude["yaw"] = current_position["heading"]

            # Độ nghiêng Roll khi vào cua (Banking)
            attitude["roll"] = max(-25.0, min(25.0, -diff * 0.45))

            # Tăng tốc dần tới tốc độ bay tối đa
            current_position["speed"] = min(MAX_SPEED_MS, current_position["speed"] + 1.2 * dt)

            # Giữ độ cao ổn định tại CRUISE_ALTITUDE
            alt_diff = target_alt - current_position["alt"]
            attitude["pitch"] = max(-15.0, min(15.0, -current_position["speed"] * 1.5 + alt_diff * 0.3))

            # Bước di chuyển tọa độ GPS theo vận tốc thực tế (m/s) và dt (giây)
            rad = math.radians(bearing)
            dist_step_m = current_position["speed"] * dt
            d_lat = (dist_step_m * math.cos(rad)) / 111139.0
            d_lon = (dist_step_m * math.sin(rad)) / (111139.0 * math.cos(math.radians(current_position["lat"])))
            current_position["lat"] += d_lat
            current_position["lon"] += d_lon

            if abs(alt_diff) > 0.2:
                current_position["alt"] += math.copysign(min(abs(alt_diff), 2.5 * dt), alt_diff)
            else:
                current_position["alt"] = target_alt

        else:
            # Không bay: giảm tốc độ về 0
            if current_position["speed"] > 0:
                current_position["speed"] = max(0.0, current_position["speed"] - 2.0 * dt)
            attitude["pitch"] = attitude["pitch"] * 0.8
            attitude["roll"] = attitude["roll"] * 0.8

        time.sleep(UPDATE_INTERVAL)

# Bật simulation thread
sim_thread = threading.Thread(target=simulation_loop, daemon=True)
sim_thread.start()

# ==================== TẠO FRAME VIDEO CAMERA: HIỂN THỊ CHƯA KẾT NỐI ====================
def generate_camera_hud_frame():
    """Tạo frame camera thông báo chưa kết nối phần cứng theo chuẩn Cockpit GCS."""
    w, h = 640, 480
    img = Image.new('RGB', (w, h), color=(15, 20, 28))
    draw = ImageDraw.Draw(img)

    # Lưới grid kỹ thuật mờ
    for x in range(0, w, 40):
        draw.line([(x, 0), (x, h)], fill=(24, 32, 46), width=1)
    for y in range(0, h, 40):
        draw.line([(0, y), (w, y)], fill=(24, 32, 46), width=1)

    # Box cảnh báo trung tâm
    bw, bh = 440, 160
    bx1 = (w - bw) // 2
    by1 = (h - bh) // 2 - 20
    bx2 = bx1 + bw
    by2 = by1 + bh

    draw.rectangle([(bx1, by1), (bx2, by2)], fill=(20, 26, 38), outline=(234, 179, 8), width=2)
    draw.rectangle([(bx1, by1), (bx2, by1 + 32)], fill=(48, 38, 12))
    draw.text((bx1 + 14, by1 + 8), "⚠️  CAMERA STATUS: NO SIGNAL", fill=(234, 179, 8))

    draw.text((bx1 + 24, by1 + 50), "CAMERA CHƯA KẾT NỐI", fill=(255, 255, 255))
    draw.text((bx1 + 24, by1 + 78), "• Luồng video RTSP/UVC: DISCONNECTED", fill=(200, 215, 230))
    draw.text((bx1 + 24, by1 + 102), "• Phần cứng camera chưa được kích hoạt trên UAV", fill=(148, 163, 184))
    draw.text((bx1 + 24, by1 + 126), f"• Trạng thái bay: {flight_state} | Viễn thám: HOẠT ĐỘNG", fill=(0, 229, 255))

    # Thanh trạng thái đáy
    draw.rectangle([(15, h - 50), (w - 15, h - 15)], fill=(10, 14, 22), outline=(50, 65, 85))
    draw.text((25, h - 42), f"STATE: {flight_state} | ALT: {current_position['alt']:.1f}m | SPD: {current_position['speed']:.1f}m/s | BAT: {battery_state['voltage']}V ({battery_state['pct']:.0f}%)", fill=(255, 255, 255))
    draw.text((w - 180, h - 42), datetime.now().strftime("%Y-%m-%d %H:%M:%S"), fill=(0, 229, 255))

    return img

# ==================== ROUTES TRANG WEB ====================
@app.route("/")
def home():
    return render_template("index.html")

@app.route("/telemetry")
def telemetry_page():
    return render_template("dashboard.html")

@app.route("/stream")
def stream_page():
    return render_template("stream.html")

# ==================== API TELEMETRY & STATE ====================
@app.route("/api/uav-state")
def api_uav_state():
    with data_lock:
        dist_to_wp = 0.0
        if current_waypoints and current_waypoint_index < len(current_waypoints):
            twp = current_waypoints[current_waypoint_index]
            twp_lon = twp.get("lng", twp.get("lon"))
            dist_to_wp = calculate_distance(
                current_position["lat"], current_position["lon"],
                twp["lat"], twp_lon
            )

        state_obj = {
            "timestamp": datetime.now().isoformat(),
            "armed": is_armed,
            "mode": current_mode,
            "flight_state": flight_state,
            "target_altitude": CRUISE_ALTITUDE,
            "terrain_altitude": TERRAIN_ALTITUDE,
            "battery": round(battery_state["pct"], 1),
            "voltage": round(battery_state["voltage"], 2),
            "current": round(battery_state["current"], 1),
            "satellites": 18,
            "link_rssi": 98,
            "gps": {
                "lat": current_position["lat"],
                "lon": current_position["lon"],
                "alt": round(current_position["alt"], 1),
                "speed": round(current_position["speed"], 2),
                "heading": round(current_position["heading"], 1),
                "satellites": 18,
                "fix_type": "3D FIX",
                "hdop": 0.8
            },
            "attitude": {
                "pitch": round(attitude["pitch"], 1),
                "roll": round(attitude["roll"], 1),
                "yaw": round(attitude["yaw"], 1)
            },
            "sensors": latest_sensor_data,
            "flight": {
                "flight_time": int(total_flight_seconds),
                "flight_state": flight_state,
                "is_flying": is_flying or flight_state in ["TAKEOFF", "RUNNING", "LANDING"],
                "is_holding": is_holding or flight_state == "HOLD",
                "hold_samples": hold_samples_collected,
                "hold_total": SAMPLE_COUNT,
                "current_waypoint": current_waypoint_index,
                "total_waypoints": len(current_waypoints),
                "distance_to_wp": round(dist_to_wp, 1)
            },
            "collected_data": collected_data_history
        }

        # Trả về cả `state` chuẩn và các trường phẳng để tương thích 100% với cả mã cũ và mới
        return jsonify({
            "success": True,
            "state": state_obj,
            "collected_data": collected_data_history,
            "distance_to_wp": round(dist_to_wp, 1),
            "flight_state": flight_state,
            "target_altitude": CRUISE_ALTITUDE,
            "terrain_altitude": TERRAIN_ALTITUDE,
            "battery": round(battery_state["pct"], 1),
            "voltage": round(battery_state["voltage"], 2),
            "altitude": round(current_position["alt"], 1),
            "speed": round(current_position["speed"], 2),
            "heading": round(current_position["heading"], 1),
            "mode": current_mode,
            "armed": is_armed,
            "latitude": current_position["lat"],
            "longitude": current_position["lon"],
            "satellites": 18,
            "pitch": round(attitude["pitch"], 1),
            "roll": round(attitude["roll"], 1),
            "pm25": latest_sensor_data["pm25"],
            "pm10": latest_sensor_data["pm10"],
            "eco2": latest_sensor_data["eco2"],
            "co2": latest_sensor_data["eco2"],
            "tvoc": latest_sensor_data["tvoc"],
            "co": latest_sensor_data["co"],
            "no2": latest_sensor_data["no2"],
            "temperature": latest_sensor_data["temp"],
            "humidity": latest_sensor_data["hum"],
            "aqi": latest_sensor_data["aqi"],
            "aqi_category": latest_sensor_data["aqi_category"],
            "current_waypoint": current_waypoint_index,
            "total_waypoints": len(current_waypoints),
            "is_flying": is_flying or flight_state in ["TAKEOFF", "RUNNING", "LANDING"],
            "is_holding": is_holding or flight_state == "HOLD"
        })

@app.route("/api/telemetry")
def api_telemetry():
    with data_lock:
        return jsonify({
            "success": True,
            "telemetry": {
                "pm25": latest_sensor_data["pm25"],
                "pm10": latest_sensor_data["pm10"],
                "eco2": latest_sensor_data["eco2"],
                "co2": latest_sensor_data["eco2"],
                "tvoc": latest_sensor_data["tvoc"],
                "temp": latest_sensor_data["temp"],
                "hum": latest_sensor_data["hum"],
                "aqi": latest_sensor_data["aqi"],
                "aqi_category": latest_sensor_data["aqi_category"],
                "aqi_main_pollutant": latest_sensor_data["aqi_main_pollutant"],
                "aqi_components": latest_sensor_data["aqi_components"],
                "co": latest_sensor_data["co"],
                "no2": latest_sensor_data["no2"],
                "alt": round(current_position["alt"], 1),
                "target_alt": CRUISE_ALTITUDE,
                "terrain_alt": TERRAIN_ALTITUDE,
                "speed": round(current_position["speed"], 2),
                "heading": round(current_position["heading"], 1),
                "pitch": round(attitude["pitch"], 1),
                "roll": round(attitude["roll"], 1),
                "armed": is_armed,
                "battery": round(battery_state["pct"], 1),
                "voltage": round(battery_state["voltage"], 2),
                "current": round(battery_state["current"], 1),
                "satellites": 18,
                "mode": current_mode,
                "flight_state": flight_state,
                "is_holding": is_holding or flight_state == "HOLD",
                "is_flying": is_flying or flight_state in ["TAKEOFF", "RUNNING", "LANDING"],
                "hold_samples": hold_samples_collected if is_holding else 0,
                "hold_total": SAMPLE_COUNT if is_holding else 0,
                "current_wp": current_waypoint_index + 1 if current_waypoint_index < len(current_waypoints) else len(current_waypoints),
                "total_wp": len(current_waypoints)
            }
        })

@app.route("/vehicle-position")
def vehicle_position():
    return jsonify({
        "success": True,
        "lat": current_position["lat"],
        "lon": current_position["lon"],
        "alt": round(current_position["alt"], 1),
        "target_alt": CRUISE_ALTITUDE,
        "terrain_alt": TERRAIN_ALTITUDE,
        "flight_state": flight_state
    })

@app.route("/vehicle-info")
def vehicle_info():
    return jsonify({
        "success": True,
        "alt": round(current_position["alt"], 1),
        "speed": round(current_position["speed"], 2),
        "heading": round(current_position["heading"], 1),
        "pitch": round(attitude["pitch"], 1),
        "roll": round(attitude["roll"], 1),
        "battery": round(battery_state["pct"], 1),
        "voltage": round(battery_state["voltage"], 2),
        "mode": current_mode,
        "flight_state": flight_state,
        "armed": is_armed,
        "satellites": 18
    })

@app.route("/mission-progress")
def mission_progress():
    total_wp = len(current_waypoints)
    current_wp = current_waypoint_index if current_waypoint_index <= total_wp else total_wp
    # State: 3 = flying, 2 = holding/sampling, 5 = idle
    state_code = 3 if is_flying else (2 if is_holding else 5)
    return jsonify({
        "success": True,
        "mission_total": total_wp,
        "mission_current": current_wp,
        "mission_state": state_code,
        "flight_state": flight_state
    })

# ==================== MISSION & CONTROL ENDPOINTS ====================
@app.route("/upload-mission", methods=["POST"])
def upload_mission():
    global current_waypoints, current_waypoint_index, is_flying, is_holding, current_mode, flight_state

    data = request.get_json()
    if not data or "mission" not in data:
        return jsonify({"success": False, "message": "Missing mission data"}), 400

    raw_mission = data["mission"]
    with data_lock:
        # CHỈ LẤY VỊ TRÍ TỌA ĐỘ (lat, lng/lon), KHÔNG CẦN VÀ KHÔNG DÙNG ĐỘ CAO TỪ CLIENT
        current_waypoints = []
        for wp in raw_mission:
            lat = float(wp["lat"])
            lon = float(wp.get("lng", wp.get("lon", 0.0)))
            current_waypoints.append({
                "lat": lat,
                "lng": lon,
                "lon": lon,
                "alt": CRUISE_ALTITUDE
            })

        current_waypoint_index = 0
        is_flying = False
        is_holding = False
        flight_state = "IDLE"
        current_mode = "GUIDED"

    append_to_event_log(f"📤 Nạp {len(current_waypoints)} điểm đo vào hệ thống thành công (Độ cao hành trình: {CRUISE_ALTITUDE}m)!", "success")
    return jsonify({"success": True, "message": f"Uploaded {len(current_waypoints)} waypoints", "count": len(current_waypoints), "target_altitude": CRUISE_ALTITUDE})

@app.route("/set-altitude", methods=["POST"])
def set_altitude():
    global CRUISE_ALTITUDE
    data = request.get_json(silent=True) or {}
    alt_val = data.get("altitude", data.get("alt"))
    if alt_val is not None:
        try:
            val = float(alt_val)
            val = max(5.0, min(150.0, val))
            with data_lock:
                CRUISE_ALTITUDE = val
                for w in current_waypoints:
                    w["alt"] = CRUISE_ALTITUDE
            append_to_event_log(f"📐 Thiết lập độ cao cất cánh & hành trình: {CRUISE_ALTITUDE}m", "info")
            return jsonify({"success": True, "target_altitude": CRUISE_ALTITUDE, "message": f"Đã thiết lập độ cao {CRUISE_ALTITUDE}m"})
        except Exception as e:
            return jsonify({"success": False, "message": str(e)}), 400
    return jsonify({"success": False, "message": "Thiếu thông số altitude"}), 400

@app.route("/start-mission", methods=["POST"])
def start_mission():
    global is_flying, is_holding, is_armed, current_mode, current_waypoint_index, flight_state, CRUISE_ALTITUDE

    if not current_waypoints:
        return jsonify({"success": False, "message": "Chưa có danh sách waypoint!"}), 400

    data = request.get_json(silent=True) or {}
    if "alt" in data or "altitude" in data:
        try:
            val = float(data.get("alt", data.get("altitude")))
            val = max(5.0, min(150.0, val))
            with data_lock:
                CRUISE_ALTITUDE = val
                for w in current_waypoints:
                    w["alt"] = CRUISE_ALTITUDE
        except Exception:
            pass

    with data_lock:
        is_armed = True
        is_holding = False
        current_mode = "AUTO"
        current_waypoint_index = 0

        # Nếu UAV đang ở mặt đất hoặc độ cao thấp (< 5m) -> Bật motor cất cánh lên CRUISE_ALTITUDE
        if current_position["alt"] < 5.0:
            current_position["alt"] = 0.0
            flight_state = "TAKEOFF"
            is_flying = False
            append_to_event_log(f"🚀 Khởi động hành trình: UAV đã bật motor và cất cánh lên {CRUISE_ALTITUDE}m (TAKEOFF)...", "success")
        else:
            # Nếu UAV đang ở trên không -> Duy trì độ cao CRUISE_ALTITUDE và bay thẳng tới waypoint #1
            flight_state = "RUNNING"
            is_flying = True
            current_position["speed"] = 1.0
            append_to_event_log(f"🚀 Bắt đầu hành trình: UAV giữ độ cao {CRUISE_ALTITUDE}m và bay đến điểm đo #1...", "success")

    return jsonify({"success": True, "message": "Mission started", "flight_state": flight_state, "target_altitude": CRUISE_ALTITUDE})

@app.route("/takeoff", methods=["POST"])
def takeoff():
    global is_armed, is_flying, is_holding, current_mode, flight_state, CRUISE_ALTITUDE
    data = request.get_json(silent=True) or {}
    if "alt" in data or "altitude" in data:
        try:
            val = float(data.get("alt", data.get("altitude")))
            val = max(5.0, min(150.0, val))
            with data_lock:
                CRUISE_ALTITUDE = val
                for w in current_waypoints:
                    w["alt"] = CRUISE_ALTITUDE
        except Exception:
            pass

    with data_lock:
        is_armed = True
        flight_state = "TAKEOFF"
        is_flying = False
        is_holding = False
        current_mode = "GUIDED"
    append_to_event_log(f"🛫 Lệnh cất cánh: UAV đã kích hoạt động cơ và bắt đầu cất cánh lên độ cao {CRUISE_ALTITUDE}m", "success")
    return jsonify({"success": True, "message": f"UAV đang cất cánh lên {CRUISE_ALTITUDE}m", "flight_state": flight_state, "target_altitude": CRUISE_ALTITUDE})

@app.route("/save-current-waypoint", methods=["POST"])
def save_current_waypoint():
    """Lưu thông số hiện tại của vị trí và cảm biến thành một điểm đo lịch sử."""
    with data_lock:
        wp_idx = len(wp_csv_files)
        lat = current_position["lat"]
        lon = current_position["lon"]
        alt = current_position["alt"]
        now_iso = datetime.now().isoformat()
        readings = []
        for i in range(10):
            readings.append({
                "timestamp": now_iso,
                "pm25": latest_sensor_data["pm25"],
                "pm10": latest_sensor_data["pm10"],
                "eco2": latest_sensor_data["eco2"],
                "tvoc": latest_sensor_data["tvoc"],
                "temp": latest_sensor_data["temp"],
                "hum": latest_sensor_data["hum"],
                "aqi": latest_sensor_data["aqi"],
                "co": latest_sensor_data["co"],
                "no2": latest_sensor_data["no2"]
            })
        
        avg_reading = {
            "lat": lat,
            "lon": lon,
            "alt": alt,
            "time": now_iso,
            "waypoint_index": wp_idx,
            "pm25": latest_sensor_data["pm25"],
            "pm10": latest_sensor_data["pm10"],
            "eco2": latest_sensor_data["eco2"],
            "tvoc": latest_sensor_data["tvoc"],
            "temp": latest_sensor_data["temp"],
            "hum": latest_sensor_data["hum"],
            "aqi": latest_sensor_data["aqi"],
            "aqi_category": latest_sensor_data["aqi_category"],
            "aqi_color": latest_sensor_data.get("aqi_components", {}).get("color", "#157a3a"),
            "co": latest_sensor_data["co"],
            "no2": latest_sensor_data["no2"],
            "sample_count": 10
        }
        collected_data_history.append(avg_reading)
        save_wp_raw_csv(wp_idx, readings, lat, lon, alt, avg_reading=avg_reading)
    
    append_to_event_log(f"💾 Đã lưu điểm đo #{wp_idx + 1} vào lịch sử quan trắc", "success")
    return jsonify({"success": True, "message": f"Đã lưu điểm đo #{wp_idx + 1}", "data": avg_reading})

@app.route("/stop-mission", methods=["POST"])
def stop_mission():
    global is_flying, is_holding, current_mode, flight_state
    with data_lock:
        is_flying = False
        is_holding = False
        flight_state = "HOLD"
        current_mode = "HOLD"
        current_position["speed"] = 0.0
    append_to_event_log("⏸️ Mission tạm dừng. UAV giữ vị trí trên không (HOLD).", "warn")
    return jsonify({"success": True, "message": "Mission stopped (HOLD)", "flight_state": flight_state})

@app.route("/clear-mission", methods=["POST"])
def clear_mission():
    global current_waypoints, current_waypoint_index, is_flying, is_holding, current_mode, flight_state
    with data_lock:
        current_waypoints = []
        current_waypoint_index = 0
        is_flying = False
        is_holding = False
        flight_state = "IDLE"
        current_mode = "MANUAL"
    append_to_event_log("🗑️ Đã xóa sạch toàn bộ danh sách waypoint", "info")
    return jsonify({"success": True, "message": "Mission cleared", "flight_state": flight_state})

@app.route("/get-mission")
def get_mission():
    return jsonify({"success": True, "mission": current_waypoints})

@app.route("/arm", methods=["POST"])
def arm():
    global is_armed
    with data_lock:
        is_armed = True
    append_to_event_log("⚡ Đã kích hoạt vũ trang (ARMED)", "warn")
    return jsonify({"success": True, "message": "Armed"})

@app.route("/disarm", methods=["POST"])
def disarm():
    global is_armed, is_flying, is_holding, current_mode, flight_state
    with data_lock:
        is_armed = False
        is_flying = False
        is_holding = False
        flight_state = "IDLE"
        current_mode = "MANUAL"
        current_position["speed"] = 0.0
        current_position["alt"] = 0.0
    append_to_event_log("🔒 Đã giải giáp động cơ (DISARMED) - Trạng thái IDLE", "info")
    return jsonify({"success": True, "message": "Disarmed", "flight_state": flight_state})

@app.route("/rtl", methods=["POST"])
def rtl():
    global is_flying, is_holding, current_mode, flight_state
    with data_lock:
        current_mode = "RTL"
        is_holding = False
        flight_state = "LANDING"
        is_flying = False
    append_to_event_log("🛬 Lệnh Hạ cánh khẩn cấp / Trở về bãi đáp được kích hoạt!", "warn")
    return jsonify({"success": True, "message": "Landing triggered", "flight_state": flight_state})

@app.route("/set-mode", methods=["POST"])
def set_mode():
    global current_mode
    req = request.get_json() or {}
    new_mode = req.get("mode", "GUIDED").upper()
    with data_lock:
        current_mode = new_mode
    append_to_event_log(f"🔄 Chuyển chế độ bay: {current_mode}", "info")
    return jsonify({"success": True, "mode": current_mode})

# ==================== DATA LOGGING & EXPORT ====================
@app.route("/set-data-logging", methods=["POST"])
def set_data_logging():
    global data_logging_enabled
    data = request.get_json() or {}
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

@app.route("/export-waypoint-csv")
@app.route("/download-csv")
@app.route("/export-csv")
def export_csv():
    # 1. Ưu tiên xuất file tóm tắt lịch sử waypoint nếu có
    if os.path.exists(SUMMARY_CSV_PATH) and os.path.getsize(SUMMARY_CSV_PATH) > 0:
        return send_file(
            SUMMARY_CSV_PATH,
            mimetype="text/csv; charset=utf-8",
            as_attachment=True,
            download_name=f"uav_waypoint_history_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        )
    elif collected_data_history:
        buf = BytesIO()
        buf.write(b'\xef\xbb\xbf') # UTF-8 BOM cho Excel
        headers = [
            "STT", "Điểm đo", "Thời gian", "Vĩ độ (Lat)", "Kinh độ (Lon)", "Độ cao (m)", "Số mẫu",
            "PM2.5 (µg/m³)", "PM10 (µg/m³)", "CO₂ (mg/m³)", "TVOC (ppb)",
            "CO (mg/m³)", "NO₂ (µg/m³)", "Nhiệt độ (°C)", "Độ ẩm (%)", "Chỉ số VN_AQI", "Chất lượng không khí"
        ]
        csv_text = ",".join(f'"{h}"' for h in headers) + "\n"
        for i, item in enumerate(collected_data_history, 1):
            wp_num = item.get("waypoint_index", 0) + 1
            csv_text += f'{i},"Điểm #{wp_num}","{item.get("time","")}","{item.get("lat",0)}","{item.get("lon",0)}","{item.get("alt",0)}","{item.get("sample_count",10)}","{item.get("pm25",0):.1f}","{item.get("pm10",0):.1f}","{item.get("eco2",0):.0f}","{item.get("tvoc",0):.0f}","{item.get("co",0):.2f}","{item.get("no2",0):.1f}","{item.get("temp",0):.1f}","{item.get("hum",0):.1f}","{item.get("aqi",1)}","{item.get("aqi_category","")}"\n'
        buf.write(csv_text.encode('utf-8'))
        buf.seek(0)
        return send_file(
            buf,
            mimetype="text/csv; charset=utf-8",
            as_attachment=True,
            download_name=f"uav_waypoint_history_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        )
    else:
        raw_csv = os.path.join(DATA_DIR, "sensor_readings.csv")
        if os.path.exists(raw_csv):
            return send_file(raw_csv, as_attachment=True, download_name=f"uav_telemetry_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
        return jsonify({"success": False, "message": "Chưa có dữ liệu đo đạc nào được ghi lại"}), 404

@app.route("/export-json")
def export_json():
    json_path = os.path.join(DATA_DIR, "sensor_readings.jsonl")
    if not os.path.exists(json_path):
        return jsonify({"success": False, "message": "Chưa có dữ liệu"}), 404
    data = []
    with open(json_path, 'r', encoding='utf-8') as f:
        for line in f:
            if line.strip():
                data.append(json.loads(line))
    return jsonify({"success": True, "data": data})

# ==================== WAYPOINT DATA & CSV LOGGING ====================
@app.route("/wp-data/list")
def wp_data_list():
    return jsonify({"success": True, "files": wp_csv_files, "total": len(wp_csv_files)})

@app.route("/wp-data/download/<filename>")
def wp_data_download(filename):
    safe = os.path.basename(filename)
    filepath = os.path.join(WP_DATA_DIR, safe)
    if not os.path.exists(filepath):
        return jsonify({"success": False, "message": "File not found"}), 404
    return send_file(filepath, mimetype="text/csv; charset=utf-8", as_attachment=True, download_name=safe)

@app.route("/wp-data/download-all")
def wp_data_download_all():
    import zipfile
    if not wp_csv_files and not (os.path.exists(SUMMARY_CSV_PATH) and os.path.getsize(SUMMARY_CSV_PATH) > 0):
        return jsonify({"success": False, "message": "Chưa có dữ liệu để tải"}), 404
    buf = BytesIO()
    with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as zf:
        if os.path.exists(SUMMARY_CSV_PATH) and os.path.getsize(SUMMARY_CSV_PATH) > 0:
            zf.write(SUMMARY_CSV_PATH, "waypoint_history_summary.csv")
        for entry in wp_csv_files:
            fp = os.path.join(WP_DATA_DIR, entry["filename"])
            if os.path.exists(fp):
                zf.write(fp, entry["filename"])
    buf.seek(0)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    return send_file(buf, mimetype="application/zip", as_attachment=True, download_name=f"uav_mission_data_{ts}.zip")

@app.route("/clear-collected-data", methods=["POST", "GET"])
@app.route("/wp-data/clear", methods=["POST", "GET"])
def wp_data_clear():
    import shutil
    global wp_csv_files, collected_data_history
    with data_lock:
        if os.path.exists(WP_DATA_DIR):
            shutil.rmtree(WP_DATA_DIR)
            os.makedirs(WP_DATA_DIR, exist_ok=True)
        wp_csv_files = []
        collected_data_history = []
    append_to_event_log("🗑️ Đã xóa toàn bộ dữ liệu lịch sử các điểm đo", "info")
    return jsonify({"success": True, "message": "Đã xóa toàn bộ dữ liệu lịch sử waypoint"})

@app.route("/wp-data/delete", methods=["POST", "DELETE", "GET"])
@app.route("/wp-data/delete/<path:filename>", methods=["POST", "DELETE", "GET"])
def wp_data_delete_single(filename=None):
    global wp_csv_files, collected_data_history
    req = request.get_json(silent=True) or {}
    target_fn = filename or req.get("filename") or request.args.get("filename")
    target_wp = req.get("waypoint_index") if req.get("waypoint_index") is not None else request.args.get("waypoint_index")
    if target_wp is not None:
        try:
            target_wp = int(target_wp)
        except Exception:
            target_wp = None

    deleted = False
    with data_lock:
        if target_fn:
            safe = os.path.basename(target_fn)
            fp = os.path.join(WP_DATA_DIR, safe)
            if os.path.exists(fp):
                try:
                    os.remove(fp)
                    deleted = True
                except Exception as e:
                    print("Lỗi xóa file:", e)
            
            wp_csv_files = [f for f in wp_csv_files if f.get("filename") != safe]
            collected_data_history = [c for c in collected_data_history if c.get("filename") != safe]

        if target_wp is not None:
            collected_data_history = [c for c in collected_data_history if c.get("waypoint_index") != target_wp]
            wp_csv_files = [f for f in wp_csv_files if f.get("wp") != (target_wp + 1)]
            deleted = True

        save_to_history_json()
        if os.path.exists(SUMMARY_CSV_PATH):
            try:
                os.remove(SUMMARY_CSV_PATH)
            except Exception:
                pass
        for entry in wp_csv_files:
            save_to_summary_history_csv(entry)

    append_to_event_log(f"🗑️ Đã xóa dữ liệu điểm đo (file: {target_fn}, wp: {target_wp})", "info")
    return jsonify({"success": True, "message": "Đã xóa điểm đo thành công", "deleted": deleted})

csv_logging_active = False
csv_samples_collected = 0
csv_max_samples = 30
csv_filename = "sensor_data.csv"
csv_position = "default_position"

@app.route("/start-csv-logging", methods=["POST"])
def start_csv_logging_route():
    global csv_logging_active, csv_samples_collected, csv_max_samples, csv_filename, csv_position
    data = request.get_json() or {}
    csv_position = data.get("position", "unknown")
    csv_max_samples = data.get("max_samples", 30)
    csv_filename = data.get("filename", f"sensor_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
    csv_logging_active = True
    csv_samples_collected = 0
    return jsonify({
        "success": True,
        "message": f"Started logging {csv_max_samples} samples to {csv_filename}",
        "status": {"active": csv_logging_active, "samples": csv_samples_collected, "max": csv_max_samples}
    })

@app.route("/stop-csv-logging", methods=["POST"])
def stop_csv_logging_route():
    global csv_logging_active
    csv_logging_active = False
    return jsonify({
        "success": True,
        "message": "CSV logging stopped",
        "status": {"active": False, "samples": csv_samples_collected, "max": csv_max_samples}
    })

@app.route("/csv-logging-status")
def csv_logging_status_route():
    return jsonify({
        "success": True,
        "status": {"active": csv_logging_active, "samples": csv_samples_collected, "max": csv_max_samples}
    })

# ==================== VIDEO STREAM & CAMERA CONTROLS ====================
@app.route("/video_feed")
def video_feed():
    def generate():
        while True:
            img = generate_camera_hud_frame()
            if camera_recording and len(recorded_frames) < 150:
                recorded_frames.append(img.copy())

            img_byte_arr = BytesIO()
            img.save(img_byte_arr, format='JPEG', quality=75)
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + img_byte_arr.getvalue() + b'\r\n')
            time.sleep(0.04) # ~25 FPS
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route("/camera/snapshot", methods=["POST"])
def camera_snapshot():
    try:
        img = generate_camera_hud_frame()
        filename = f"snapshot_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg"
        filepath = os.path.join(CAMERA_DIR, filename)
        img.save(filepath, 'JPEG', quality=90)
        append_to_event_log(f"📸 Chụp ảnh Snapshot thành công: {filename}", "success")
        return jsonify({"success": True, "filename": filename, "path": filepath})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)}), 500

@app.route("/camera/record/start", methods=["POST"])
def camera_record_start():
    global camera_recording, recorded_frames
    camera_recording = True
    recorded_frames = []
    append_to_event_log("🔴 Bắt đầu ghi hình video camera...", "warn")
    return jsonify({"success": True, "recording": True})

@app.route("/camera/record/stop", methods=["POST"])
def camera_record_stop():
    global camera_recording, recorded_frames
    camera_recording = False
    if recorded_frames:
        try:
            filename = f"recording_{datetime.now().strftime('%Y%m%d_%H%M%S')}.gif"
            filepath = os.path.join(CAMERA_DIR, filename)
            frames_to_save = recorded_frames[::2] # Giảm fps để lưu gif nhẹ
            if frames_to_save:
                frames_to_save[0].save(filepath, save_all=True, append_images=frames_to_save[1:], duration=80, loop=0)
            count = len(recorded_frames)
            recorded_frames = []
            append_to_event_log(f"💾 Đã lưu video clip ({count} frames): {filename}", "success")
            return jsonify({"success": True, "filename": filename, "frames": count})
        except Exception as e:
            return jsonify({"success": False, "error": str(e)}), 500
    return jsonify({"success": True, "message": "No frames recorded"})

if __name__ == "__main__":
    print("=" * 70)
    print("  🚁 UAV Ground Control Station (GCS) - COCKPIT MOCK SERVER")
    print("  📍 Local GCS URL: http://localhost:5000")
    print("  🌐 Telemetry & Air Quality Sensors: ENS160 + AHT21 + Optical PM + Gas")
    print("  🛰️ Flight Dynamics: 18 Sats 3D FIX, 4S LiPo Battery Model, Horizon HUD")
    print("=" * 70)
    app.run(host="0.0.0.0", port=5000, debug=True, threaded=True)