"""
UAV Ground Control Station - Real Hardware Server (Raspberry Pi 5 / Linux)
Air Quality Monitor & GCS Cockpit Backend

Sensors:
  - CAN Bus (MCP2515 / SocketCAN can0):
      Frame 0x555 (8 bytes): Temperature, Humidity, TVOC, eCO2
      Frame 0x556 (8 bytes): CO, NO2, PM2.5, AQI
  - Local I2C Fallback: ENS160 + AHT21
Camera:
  - Picamera2 with fallback to Disconnected Cockpit HUD Feed
Navigation & State Machine:
  - Flight states: IDLE, TAKEOFF, RUNNING, HOLD, LANDING
  - Vietnam VN_AQI (QCVN 05:2023 0-500 scale)
  - Coordinate-only waypoint upload (lat, lng only, no alt required)
  - Altitude telemetry (target_altitude=35m, terrain_altitude=7m)
"""
import os
import sys
import time
import threading
import csv
import math
import random
import json

# Ensure UTF-8 console encoding on Windows to prevent UnicodeEncodeError
try:
    if sys.stdout and hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    if sys.stderr and hasattr(sys.stderr, 'reconfigure'):
        sys.stderr.reconfigure(encoding='utf-8', errors='replace')
except Exception:
    pass
from datetime import datetime
from flask import Flask, request, jsonify, render_template, Response, send_file, send_from_directory
from flask_cors import CORS

# Optional imports with graceful fallbacks
try:
    import numpy as np
    import cv2
except ImportError:
    np = None
    cv2 = None

try:
    from PIL import Image, ImageDraw, ImageFont
    from io import BytesIO
except ImportError:
    Image = None
    ImageDraw = None
    ImageFont = None
    BytesIO = None

try:
    import can
except ImportError:
    can = None

try:
    import serial
except ImportError:
    serial = None

try:
    from smbus2 import SMBus, i2c_msg
except ImportError:
    SMBus = None
    i2c_msg = None

app = Flask(__name__)
CORS(app)

# ==================== CONSTANTS & FLIGHT CONFIG ====================
WAYPOINT_RADIUS = 5.0        # Mét đạt waypoint
HOLD_TIME = 10.0             # Thời gian dừng tại mỗi waypoint (giây)
SAMPLE_COUNT = 10            # Số lượng mẫu lấy tại mỗi waypoint
SAMPLE_INTERVAL = HOLD_TIME / SAMPLE_COUNT
MAX_SPEED_MS = 4.5           # Vận tốc tối đa (m/s)
FLIGHT_SPEED_DEG = 0.000035  # Bước dịch chuyển GPS mỗi chu kỳ
UPDATE_INTERVAL = 0.08       # 12.5 Hz loop
CRUISE_ALTITUDE = 35.0       # Độ cao hành trình tiêu chuẩn (khớp đồ thị mẫu 35m)
TERRAIN_ALTITUDE = 7.0       # Độ cao mặt đất (7m)
TAKEOFF_CLIMB_RATE = 3.5     # Tốc độ cất cánh leo cao (m/s)
LANDING_DESCENT_RATE = 2.5   # Tốc độ hạ cánh (m/s)

CAN_ID_ENV = 0x555           # Frame 1: Temp, Hum, TVOC, eCO2
CAN_ID_GAS = 0x556           # Frame 2: CO, NO2, PM2.5, AQI
CAN_TIMEOUT = 10.0           # Timeout mất tín hiệu CAN (giây)

ENS160_ADDR = 0x53
AHT21_ADDR = 0x38

DATA_DIR = "uav_data"
CAMERA_DIR = "camera_captures"
WP_DATA_DIR = "wp_data"
LOGS_DIR = "logs"
for d in [DATA_DIR, CAMERA_DIR, WP_DATA_DIR, LOGS_DIR]:
    os.makedirs(d, exist_ok=True)

# ==================== TRẠNG THÁI TOÀN CỤC ====================
data_lock = threading.Lock()
simulation_running = True
data_logging_enabled = True

flight_state = "IDLE"        # IDLE, TAKEOFF, RUNNING, HOLD, LANDING
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

# ==================== CAN & UART TELEMETRY TRACKING ====================
last_can_update = 0.0

UART_PORT = os.environ.get("UART_PORT", "/dev/serial0")
UART_BAUD = int(os.environ.get("UART_BAUD", 115200))
uart_serial = None
uart_lock = threading.Lock()
last_uart_rx_time = 0.0
has_real_telemetry = False

SUMMARY_CSV_PATH = os.path.join(WP_DATA_DIR, "waypoint_history_summary.csv")
HISTORY_JSON_PATH = os.path.join(WP_DATA_DIR, "waypoint_history.json")

HOME_POSITION = {
    "lat": 16.0743537,
    "lon": 108.1522514,
    "alt": 0.0
}

state = {
    "timestamp": "",
    "armed": False,
    "mode": "MANUAL",
    "flight_state": "IDLE",
    "battery": 95.0,
    "voltage": 16.48,
    "target_altitude": CRUISE_ALTITUDE,
    "terrain_altitude": TERRAIN_ALTITUDE,
    "gps": {
        "lat": 16.0743537,
        "lon": 108.1522514,
        "alt": 0.0,
        "speed": 0.0,
        "heading": 45.0,
        "satellites": 18
    },
    "attitude": {
        "pitch": 0.0,
        "roll": 0.0,
        "yaw": 45.0
    },
    "sensors": {
        "pm25": 0.0,
        "pm10": 0.0,
        "eco2": 0,
        "co2": 0,
        "tvoc": 0,
        "temp": 0.0,
        "hum": 0.0,
        "co": 0.0,
        "no2": 0.0,
        "aqi": 1,
        "aqi_category": "Tốt",
        "aqi_main_pollutant": "PM2.5",
        "aqi_components": {
            "pm25": 1,
            "pm10": 1,
            "co": 1,
            "no2": 1,
            "eco2": 1,
            "level": 1,
            "color": "#157a3a"
        },
        "firmware_aqi": 0
    }
}

# ==================== CAN BUS INITIALIZATION ====================
can_bus = None

def init_can():
    global can_bus
    if can is None:
        print("[CAN] ⚠️ 'python-can' chưa được cài đặt. CAN reader tạm tắt.")
        return None
    try:
        bus = can.interface.Bus(channel="can0", interface="socketcan")
        print("[CAN] ✅ Đã kết nối giao tiếp SocketCAN 'can0' thành công")
        return bus
    except Exception as e:
        print(f"[CAN] ⚠️ Chưa thể mở giao tiếp socketcan can0: {e}")
        return None

can_bus = init_can()

# ==================== I2C FALLBACK INITIALIZATION ====================
i2c_bus = None
if SMBus is not None:
    try:
        i2c_bus = SMBus(1)
        print("[I2C] ✅ I2C bus 1 initialized")
        try:
            i2c_bus.write_byte_data(ENS160_ADDR, 0x10, 0x02)
            print("[I2C] ✅ ENS160 initialized in standard mode")
        except Exception as e:
            print(f"[I2C] ⚠️ ENS160 init warning: {e}")
    except Exception as e:
        print(f"[I2C] ⚠️ I2C bus not available: {e}")
        i2c_bus = None

# ==================== AQI VIỆT NAM (QCVN 05:2023 / VN_AQI 0-500) ====================
def compute_vietnam_aqi(pm25, pm10, co, no2, eco2):
    """
    Tính chỉ số AQI và cấp chất lượng không khí chuẩn Việt Nam (VN_AQI 0-500)
    Quy định theo Hướng dẫn kỹ thuật tính toán và công bố chỉ số VN_AQI của Tổng cục Môi trường
    & QCVN 05:2023/BTNM.
    """
    def calc_sub_aqi(c, breakpoints):
        if c is None or c <= 0:
            return 0
        for b_low, b_high, i_low, i_high in breakpoints:
            if c <= b_high:
                return round(((i_high - i_low) / (b_high - b_low)) * (c - b_low) + i_low)
        return breakpoints[-1][3]

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
    # eCO2 (ppm / mg/m³)
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

    main_pol = max(sub_dict, key=sub_dict.get)
    overall_aqi = max(0, min(500, sub_dict[main_pol]))

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

def recompute_aqi():
    """Hàm tính toán lại AQI từ dữ liệu hiện thời của sensors"""
    s = state["sensors"]
    aqi, cat, pol, comps = compute_vietnam_aqi(
        s.get("pm25", 0.0),
        s.get("pm10", 0.0),
        s.get("co", 0.0),
        s.get("no2", 0.0),
        s.get("eco2", 0)
    )
    fw_aqi = s.get("firmware_aqi", 0)
    if fw_aqi > 0 and fw_aqi > aqi:
        aqi = fw_aqi

    s["aqi"] = aqi
    s["aqi_category"] = cat
    s["aqi_main_pollutant"] = pol
    s["aqi_components"] = comps

# ==================== UART SERIAL (TELEMETRY & MISSION) ====================
def init_uart():
    global uart_serial
    if serial is None:
        print("[UART] ⚠️ 'pyserial' chưa được cài đặt. UART Serial tạm tắt.")
        return None
    try:
        s = serial.Serial(UART_PORT, UART_BAUD, timeout=1.0)
        print(f"[UART] ✅ Đã kết nối cổng Serial UART '{UART_PORT}' ({UART_BAUD} baud)")
        return s
    except Exception as e:
        print(f"[UART] ⚠️ Chưa thể mở cổng UART '{UART_PORT}': {e}")
        return None

uart_serial = init_uart()

def send_uart_command(cmd_dict):
    """
    Gửi lệnh định dạng JSON qua UART tới Flight Controller của UAV.
    Các lệnh bao gồm: upload_mission, start_mission, stop_mission, clear_mission, arm, disarm, rtl, set_mode.
    """
    global uart_serial
    payload_str = json.dumps(cmd_dict, ensure_ascii=False) + "\n"
    with uart_lock:
        if uart_serial and uart_serial.is_open:
            try:
                uart_serial.write(payload_str.encode('utf-8'))
                uart_serial.flush()
                print(f"[UART TX] 📤 Gửi lệnh: {payload_str.strip()}")
                return True
            except Exception as e:
                print(f"[UART TX] ⚠️ Lỗi gửi lệnh UART: {e}")
                try:
                    uart_serial.close()
                except Exception:
                    pass
                uart_serial = None
        else:
            print(f"[UART TX (Mô phỏng / Chưa nối cổng)] 📤 Lệnh: {payload_str.strip()}")
    return False

def uart_reader_thread():
    """
    Luồng tiếp nhận viễn trắc (Telemetry) từ UAV qua UART (/dev/serial0 hoặc cổng cấu hình):
    - Đọc từng dòng JSON kết thúc bằng ký tự newline '\\n'
    - Cập nhật thời gian thực vào global state:
      + GPS: lat, lon, alt, speed, heading, satellites
      + Attitude: pitch, roll, yaw
      + Battery: battery, voltage
      + Control: mode, armed, flight_state
      + Tiến độ nhiệm vụ: current_wp
    """
    global uart_serial, last_uart_rx_time, has_real_telemetry, flight_state, current_waypoint_index
    print(f"[UART RX] 🚀 Bắt đầu luồng đọc UART ({UART_PORT} @ {UART_BAUD})")

    while True:
        if uart_serial is None or not uart_serial.is_open:
            time.sleep(3.0)
            uart_serial = init_uart()
            continue

        try:
            line_bytes = uart_serial.readline()
            if not line_bytes:
                continue

            raw_line = line_bytes.decode('utf-8', errors='ignore').strip()
            if not raw_line:
                continue

            try:
                msg = json.loads(raw_line)
            except Exception:
                continue

            now = time.time()
            with data_lock:
                last_uart_rx_time = now
                has_real_telemetry = True

                telemetry_data = msg.get("telemetry", msg)

                # 1. Tọa độ GPS & Vận tốc
                if "lat" in telemetry_data:
                    state["gps"]["lat"] = float(telemetry_data["lat"])
                if "lon" in telemetry_data:
                    state["gps"]["lon"] = float(telemetry_data["lon"])
                elif "lng" in telemetry_data:
                    state["gps"]["lon"] = float(telemetry_data["lng"])
                if "alt" in telemetry_data:
                    state["gps"]["alt"] = float(telemetry_data["alt"])
                if "speed" in telemetry_data:
                    state["gps"]["speed"] = float(telemetry_data["speed"])
                if "heading" in telemetry_data:
                    state["gps"]["heading"] = float(telemetry_data["heading"])
                if "satellites" in telemetry_data:
                    state["gps"]["satellites"] = int(telemetry_data["satellites"])

                # 2. Góc nghiêng (Attitude)
                if "pitch" in telemetry_data:
                    state["attitude"]["pitch"] = float(telemetry_data["pitch"])
                if "roll" in telemetry_data:
                    state["attitude"]["roll"] = float(telemetry_data["roll"])
                if "yaw" in telemetry_data:
                    state["attitude"]["yaw"] = float(telemetry_data["yaw"])
                elif "heading" in telemetry_data:
                    state["attitude"]["yaw"] = float(telemetry_data["heading"])

                # 3. Pin & Chế độ bay
                if "battery" in telemetry_data:
                    state["battery"] = float(telemetry_data["battery"])
                if "voltage" in telemetry_data:
                    state["voltage"] = float(telemetry_data["voltage"])
                if "armed" in telemetry_data:
                    state["armed"] = bool(telemetry_data["armed"])
                if "mode" in telemetry_data:
                    state["mode"] = str(telemetry_data["mode"])
                if "flight_state" in telemetry_data:
                    flight_state = str(telemetry_data["flight_state"]).upper()

                # 4. Tiến trình Waypoint
                if "current_wp" in telemetry_data:
                    wp_num = int(telemetry_data["current_wp"])
                    if wp_num > 0 and wp_num <= len(current_waypoints):
                        current_waypoint_index = wp_num - 1
                    else:
                        current_waypoint_index = wp_num
                elif "waypoint_index" in telemetry_data:
                    current_waypoint_index = int(telemetry_data["waypoint_index"])

        except Exception as e:
            print(f"[UART RX] ⚠️ Lỗi đọc UART: {e}")
            time.sleep(1.0)

# ==================== LƯU TRỮ VÀ KHÔI PHỤC DỮ LIỆU ĐO ĐẠC ====================
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

    # Nếu chưa có JSON, khôi phục từ CSV
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

# Tự động nạp lịch sử khi khởi động
load_history_from_disk()

# ==================== CAN BUS RECEIVER THREAD ====================
def can_reader_thread():
    """
    Lắng nghe nhận 2 frame CAN theo đúng protocol C/C++:
    Frame 1: ID = 0x555 (Temp + Hum + TVOC + CO2)
      - data[0:2]: int16_t tempData = temperature * 100.0 (big-endian signed)
      - data[2:4]: uint16_t humData = humidity * 100.0 (big-endian unsigned)
      - data[4:6]: uint16_t tvoc (big-endian unsigned)
      - data[6:8]: uint16_t eco2 (big-endian unsigned)
    Frame 2: ID = 0x556 (CO, NO2, PM2.5, AQI)
      - data[0:2]: uint16_t coData = co * 10.0 (big-endian unsigned)
      - data[2:4]: uint16_t no2Data = no2 * 10.0 (big-endian unsigned)
      - data[4:6]: uint16_t pmData = pm25 * 100.0 (big-endian unsigned)
      - data[6]: uint8_t aqi (0-255)
      - data[7]: 0 (reserved)
    """
    global last_can_update, can_bus
    print("[CAN] 🚀 CAN Reader thread started")

    while True:
        if can_bus is None:
            time.sleep(5)
            can_bus = init_can()
            continue

        try:
            msg = can_bus.recv(timeout=1.0)
            if msg is None:
                continue

            data = bytes(msg.data)
            now = time.time()

            # --- FRAME 1: 0x555 (Temperature, Humidity, TVOC, eCO2) ---
            if msg.arbitration_id == CAN_ID_ENV and len(data) >= 8:
                temp_raw = int.from_bytes(data[0:2], byteorder='big', signed=True)
                temperature = round(temp_raw / 100.0, 2)

                hum_raw = int.from_bytes(data[2:4], byteorder='big', signed=False)
                humidity = round(hum_raw / 100.0, 2)

                tvoc = int.from_bytes(data[4:6], byteorder='big', signed=False)
                eco2 = int.from_bytes(data[6:8], byteorder='big', signed=False)

                with data_lock:
                    state["sensors"]["temp"] = temperature
                    state["sensors"]["hum"] = max(0.0, min(100.0, humidity))
                    state["sensors"]["tvoc"] = tvoc
                    state["sensors"]["eco2"] = eco2
                    state["sensors"]["co2"] = eco2
                    last_can_update = now
                    recompute_aqi()

            # --- FRAME 2: 0x556 (CO, NO2, PM2.5, AQI) ---
            elif msg.arbitration_id == CAN_ID_GAS and len(data) >= 7:
                co_raw = int.from_bytes(data[0:2], byteorder='big', signed=False)
                co = round(co_raw / 10.0, 2)

                no2_raw = int.from_bytes(data[2:4], byteorder='big', signed=False)
                no2 = round(no2_raw / 10.0, 2)

                pm_raw = int.from_bytes(data[4:6], byteorder='big', signed=False)
                pm25 = round(pm_raw / 100.0, 2)

                firmware_aqi = int(data[6])

                with data_lock:
                    state["sensors"]["co"] = co
                    state["sensors"]["no2"] = no2
                    state["sensors"]["pm25"] = pm25
                    state["sensors"]["pm10"] = round(pm25 * 1.5, 2)
                    state["sensors"]["firmware_aqi"] = firmware_aqi
                    last_can_update = now
                    recompute_aqi()

        except Exception as e:
            print(f"[CAN] Error receiving message: {e}")
            time.sleep(1)

# ==================== I2C SENSOR FALLBACK (ENS160 + AHT21) ====================
def read_u16_i2c(addr, reg):
    if i2c_bus is None:
        return 0
    try:
        low = i2c_bus.read_byte_data(addr, reg)
        high = i2c_bus.read_byte_data(addr, reg + 1)
        return (high << 8) | low
    except Exception:
        return 0

def read_aht21_i2c():
    if i2c_bus is None or i2c_msg is None:
        return None, None
    try:
        i2c_bus.write_i2c_block_data(AHT21_ADDR, 0xAC, [0x33, 0x00])
        time.sleep(0.08)
        msg = i2c_msg.read(AHT21_ADDR, 6)
        i2c_bus.i2c_rdwr(msg)
        data = list(msg)
        if len(data) >= 6 and (data[0] & 0x80) == 0:
            raw_hum = ((data[1] & 0xFF) << 12) | ((data[2] & 0xFF) << 4) | ((data[3] & 0xF0) >> 4)
            raw_temp = ((data[3] & 0x0F) << 16) | ((data[4] & 0xFF) << 8) | (data[5] & 0xFF)
            hum = round(raw_hum * 100.0 / 1048576.0, 2)
            temp = round(raw_temp * 200.0 / 1048576.0 - 50.0, 2)
            return temp, max(0.0, min(100.0, hum))
    except Exception:
        pass
    return None, None

def i2c_sensor_fallback_thread():
    """Chỉ đọc I2C cục bộ nếu tín hiệu CAN không hoạt động (quá timeout)"""
    print("[I2C Fallback] Thread started")
    while True:
        try:
            if time.time() - last_can_update < CAN_TIMEOUT:
                time.sleep(1)
                continue

            if i2c_bus is not None:
                tvoc = read_u16_i2c(ENS160_ADDR, 0x24)
                eco2 = read_u16_i2c(ENS160_ADDR, 0x22)
                temp, hum = read_aht21_i2c()

                with data_lock:
                    if temp is not None and hum is not None:
                        state["sensors"]["temp"] = temp
                        state["sensors"]["hum"] = hum
                    if tvoc > 0:
                        state["sensors"]["tvoc"] = tvoc
                    if eco2 > 0:
                        state["sensors"]["eco2"] = eco2
                        state["sensors"]["co2"] = eco2
                    recompute_aqi()
        except Exception:
            pass
        time.sleep(2)

# ==================== ĐỊNH TUYẾN & ĐIỀU HÀNH BAY ====================
def calculate_distance(lat1, lon1, lat2, lon2):
    R = 6371000
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    a = math.sin(delta_phi/2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return R * c

def calculate_bearing(lat1, lon1, lat2, lon2):
    y = math.sin(math.radians(lon2 - lon1)) * math.cos(math.radians(lat2))
    x = math.cos(math.radians(lat1)) * math.sin(math.radians(lat2)) - \
        math.sin(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.cos(math.radians(lon2 - lon1))
    bearing = (math.degrees(math.atan2(y, x)) + 360) % 360
    return bearing

def save_wp_raw_csv(wp_index, readings, lat, lon, alt, avg_reading=None):
    """Ghi dữ liệu lấy mẫu tại một waypoint ra file CSV riêng trong wp_data/ và lưu vào bảng tổng hợp."""
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
                "pm25": round(r.get("pm25", 0.0), 2),
                "pm10": round(r.get("pm10", 0.0), 2),
                "eco2": round(r.get("eco2", 0), 1),
                "tvoc": round(r.get("tvoc", 0), 1),
                "temp": round(r.get("temp", 0.0), 2),
                "hum": round(r.get("hum", 0.0), 2),
                "aqi": int(r.get("aqi", 1)),
                "co": round(r.get("co", 0.0), 2) if r.get("co") is not None else '',
                "no2": round(r.get("no2", 0.0), 1) if r.get("no2") is not None else ''
            })

    if avg_reading:
        avg_pm25 = avg_reading.get("pm25", 0)
        avg_pm10 = avg_reading.get("pm10", round(avg_pm25 * 1.5, 1))
        avg_eco2 = avg_reading.get("eco2", 0)
        avg_tvoc = avg_reading.get("tvoc", 0)
        avg_co = avg_reading.get("co", 0)
        avg_no2 = avg_reading.get("no2", 0)
        avg_temp = avg_reading.get("temp", 0)
        avg_hum = avg_reading.get("hum", 0)
        avg_aqi = avg_reading.get("aqi", 1)
        aqi_cat = avg_reading.get("aqi_category", "Tốt")
        aqi_color = avg_reading.get("aqi_color", "#157a3a")
    else:
        avg_pm25 = round(sum(r.get("pm25", 0) for r in readings) / len(readings), 1) if readings else 0
        avg_pm10 = round(sum(r.get("pm10", 0) for r in readings) / len(readings), 1) if readings else 0
        avg_eco2 = round(sum(r.get("eco2", 0) for r in readings) / len(readings), 0) if readings else 0
        avg_tvoc = round(sum(r.get("tvoc", 0) for r in readings) / len(readings), 0) if readings else 0
        avg_co = round(sum(r.get("co", 0) for r in readings) / len(readings), 2) if readings else 0
        avg_no2 = round(sum(r.get("no2", 0) for r in readings) / len(readings), 1) if readings else 0
        avg_temp = round(sum(r.get("temp", 0) for r in readings) / len(readings), 1) if readings else 0
        avg_hum = round(sum(r.get("hum", 0) for r in readings) / len(readings), 1) if readings else 0
        avg_aqi = int(round(sum(r.get("aqi", 1) for r in readings) / len(readings))) if readings else 1
        _, aqi_cat, _, info = compute_vietnam_aqi(avg_pm25, avg_pm10, avg_co, avg_no2, avg_eco2)
        aqi_color = info.get("color", "#157a3a")

    summary_entry = {
        "wp": wp_index + 1,
        "filename": filename,
        "samples": len(readings),
        "lat": round(lat, 6),
        "lon": round(lon, 6),
        "alt": round(alt, 1),
        "time": time_display,
        "pm25": avg_pm25,
        "pm10": avg_pm10,
        "eco2": avg_eco2,
        "tvoc": avg_tvoc,
        "co": avg_co,
        "no2": avg_no2,
        "temp": avg_temp,
        "hum": avg_hum,
        "aqi": avg_aqi,
        "aqi_category": aqi_cat,
        "aqi_color": aqi_color
    }
    wp_csv_files.append(summary_entry)
    save_to_summary_history_csv(summary_entry)
    save_to_history_json()
    print(f"[WP-CSV] ✅ WP{wp_index + 1} -> {filepath} ({len(readings)} mẫu)")

# Alias tương thích ngược
write_wp_csv = save_wp_raw_csv

def flight_navigation_loop():
    """
    Vòng lặp điều hướng & quản lý bay tự động:
    - Nếu CÓ telemetry thực tế từ UAV truyền qua UART (has_real_telemetry = True):
        Sử dụng GPS, độ cao, vận tốc thực tế của UAV.
        Khi UAV bay tới phạm vi Waypoint (hoặc nhận trạng thái HOLD), kích hoạt lấy mẫu khí (10 mẫu/10s).
        Gửi lệnh UART tương ứng (hold, next_waypoint, rtl).
    - Nếu KHÔNG CÓ UART (chạy test trên máy tính hoặc chưa nối phần cứng):
        Tự động mô phỏng bay mượt mà theo đúng các trạng thái IDLE -> TAKEOFF -> RUNNING -> HOLD -> LANDING.
    """
    global flight_state, is_flying, is_holding, current_waypoint_index
    global hold_start_time, hold_samples_collected, collected_readings
    global flight_start_time, total_flight_seconds

    last_tick = time.time()

    while simulation_running:
        now = time.time()
        dt = now - last_tick
        last_tick = now

        with data_lock:
            uart_alive = (now - last_uart_rx_time < 3.0)

            if state["armed"] and flight_state != "IDLE":
                if flight_start_time == 0:
                    flight_start_time = now
                total_flight_seconds = int(now - flight_start_time)
            else:
                flight_start_time = 0

            # Chỉ tự động trừ pin nếu không có telemetry thực
            if not uart_alive and state["armed"]:
                state["battery"] = max(10.0, state["battery"] - (dt * 0.015))
                state["voltage"] = round(14.2 + (state["battery"] / 100.0) * 2.6, 2)

            # ================= 1. TRẠNG THÁI IDLE =================
            if flight_state == "IDLE":
                if not uart_alive:
                    state["gps"]["speed"] = 0.0
                    state["attitude"]["pitch"] = 0.0
                    state["attitude"]["roll"] = 0.0
                time.sleep(UPDATE_INTERVAL)
                continue

            # ================= 2. TRẠNG THÁI TAKEOFF =================
            if flight_state == "TAKEOFF":
                if not uart_alive:
                    state["gps"]["speed"] = 1.0
                    state["attitude"]["pitch"] = 5.0
                    state["attitude"]["roll"] = 0.0
                    state["gps"]["alt"] += TAKEOFF_CLIMB_RATE * dt

                    if state["gps"]["alt"] >= CRUISE_ALTITUDE:
                        state["gps"]["alt"] = CRUISE_ALTITUDE
                        state["attitude"]["pitch"] = 0.0
                        flight_state = "RUNNING"
                        is_flying = True
                        print(f"[Flight] 🛫 Đã đạt độ cao hành trình {CRUISE_ALTITUDE}m -> Chuyển RUNNING")
                else:
                    if state["gps"]["alt"] >= (CRUISE_ALTITUDE - 2.0):
                        flight_state = "RUNNING"
                        is_flying = True

                time.sleep(UPDATE_INTERVAL)
                continue

            # ================= 3. TRẠNG THÁI LANDING =================
            if flight_state == "LANDING":
                if not uart_alive:
                    state["gps"]["speed"] = max(0.5, state["gps"]["speed"] - 1.0 * dt)
                    state["attitude"]["pitch"] = -3.0
                    state["attitude"]["roll"] = 0.0
                    state["gps"]["alt"] = max(0.0, state["gps"]["alt"] - LANDING_DESCENT_RATE * dt)

                    if state["gps"]["alt"] <= 0.2:
                        state["gps"]["alt"] = 0.0
                        state["gps"]["speed"] = 0.0
                        state["attitude"]["pitch"] = 0.0
                        state["armed"] = False
                        flight_state = "IDLE"
                        is_flying = False
                        is_holding = False
                        state["mode"] = "MANUAL"
                        print("[Flight] 🛬 Đã hạ cánh an toàn xuống mặt đất -> Chuyển IDLE")
                else:
                    if not state["armed"] or state["gps"]["alt"] <= 0.5:
                        flight_state = "IDLE"
                        is_flying = False
                        is_holding = False

                time.sleep(UPDATE_INTERVAL)
                continue

            # ================= 4. TRẠNG THÁI HOLD (Lấy mẫu tại Waypoint) =================
            if flight_state == "HOLD" or is_holding:
                if not uart_alive:
                    state["gps"]["speed"] = 0.0
                    state["attitude"]["pitch"] = 0.0
                    state["attitude"]["roll"] = 0.0

                elapsed_hold = now - hold_start_time
                expected_sample = int(elapsed_hold / SAMPLE_INTERVAL)

                if expected_sample >= hold_samples_collected and hold_samples_collected < SAMPLE_COUNT:
                    s_copy = dict(state["sensors"])
                    reading = {
                        "timestamp": datetime.now().isoformat(),
                        "lat": state["gps"]["lat"],
                        "lon": state["gps"]["lon"],
                        "alt": state["gps"]["alt"],
                        "waypoint_index": current_waypoint_index,
                        "pm25": s_copy.get("pm25", 0.0),
                        "pm10": s_copy.get("pm10", 0.0),
                        "eco2": s_copy.get("eco2", 0),
                        "tvoc": s_copy.get("tvoc", 0),
                        "temp": s_copy.get("temp", 0.0),
                        "hum": s_copy.get("hum", 0.0),
                        "aqi": s_copy.get("aqi", 1),
                        "co": s_copy.get("co", 0.0),
                        "no2": s_copy.get("no2", 0.0)
                    }
                    collected_readings.append(reading)
                    hold_samples_collected += 1
                    print(f"[Mission] 📊 Thu thập mẫu {hold_samples_collected}/{SAMPLE_COUNT} tại WP{current_waypoint_index + 1}")

                if elapsed_hold >= HOLD_TIME:
                    if collected_readings and data_logging_enabled:
                        avg_pm25 = sum(r["pm25"] for r in collected_readings) / len(collected_readings)
                        avg_eco2 = sum(r["eco2"] for r in collected_readings) / len(collected_readings)
                        avg_aqi = max(r["aqi"] for r in collected_readings)

                        avg_reading = {
                            "lat": state["gps"]["lat"],
                            "lon": state["gps"]["lon"],
                            "alt": state["gps"]["alt"],
                            "time": datetime.now().isoformat(),
                            "waypoint_index": current_waypoint_index,
                            "pm25": round(avg_pm25, 1),
                            "pm10": round(sum(r.get("pm10", 0) for r in collected_readings) / len(collected_readings), 1),
                            "eco2": round(avg_eco2, 1),
                            "tvoc": round(sum(r["tvoc"] for r in collected_readings) / len(collected_readings), 1),
                            "temp": round(sum(r["temp"] for r in collected_readings) / len(collected_readings), 1),
                            "hum": round(sum(r["hum"] for r in collected_readings) / len(collected_readings), 1),
                            "aqi": int(avg_aqi),
                            "aqi_category": state["sensors"]["aqi_category"],
                            "aqi_color": state["sensors"].get("aqi_components", {}).get("color", "#157a3a"),
                            "co": round(sum(r["co"] for r in collected_readings) / len(collected_readings), 2),
                            "no2": round(sum(r["no2"] for r in collected_readings) / len(collected_readings), 1),
                            "sample_count": len(collected_readings)
                        }
                        collected_data_history.append(avg_reading)

                        try:
                            save_wp_raw_csv(
                                current_waypoint_index,
                                collected_readings,
                                state["gps"]["lat"],
                                state["gps"]["lon"],
                                state["gps"]["alt"],
                                avg_reading=avg_reading
                            )
                        except Exception as e:
                            print(f"[Mission] ⚠️ Lỗi ghi CSV WP{current_waypoint_index + 1}: {e}")

                    is_holding = False
                    hold_samples_collected = 0
                    collected_readings = []
                    current_waypoint_index += 1

                    if current_waypoint_index >= len(current_waypoints):
                        print("[Mission] ✅ Đã hoàn thành toàn bộ lộ trình Waypoints -> Chuyển LANDING")
                        flight_state = "LANDING"
                        is_flying = False
                        state["mode"] = "GUIDED"
                        send_uart_command({"cmd": "mission_complete"})
                    else:
                        flight_state = "RUNNING"
                        is_flying = True
                        state["mode"] = "AUTO"
                        send_uart_command({"cmd": "next_waypoint", "waypoint": current_waypoint_index + 1})
                        print(f"[Mission] ✈️ Bay tiếp tới Waypoint #{current_waypoint_index + 1}")

                time.sleep(UPDATE_INTERVAL)
                continue

            # ================= 5. TRẠNG THÁI RUNNING =================
            if flight_state == "RUNNING" and current_waypoints and current_waypoint_index < len(current_waypoints):
                target = current_waypoints[current_waypoint_index]
                target_lat = target["lat"]
                target_lon = target.get("lng", target.get("lon"))

                dist = calculate_distance(
                    state["gps"]["lat"], state["gps"]["lon"],
                    target_lat, target_lon
                )

                if not uart_alive:
                    alt_diff = CRUISE_ALTITUDE - state["gps"]["alt"]
                    if abs(alt_diff) > 0.5:
                        state["gps"]["alt"] += math.copysign(min(abs(alt_diff), TAKEOFF_CLIMB_RATE * dt), alt_diff)

                if dist <= WAYPOINT_RADIUS:
                    if not data_logging_enabled:
                        flight_state = "IDLE"
                        is_flying = False
                        state["mode"] = "GUIDED"
                        continue

                    print(f"[Mission] 📍 Đạt Waypoint #{current_waypoint_index + 1}, chuyển HOLD {HOLD_TIME}s")
                    flight_state = "HOLD"
                    is_holding = True
                    is_flying = False
                    hold_start_time = now
                    hold_samples_collected = 0
                    collected_readings = []
                    state["mode"] = "HOLD"
                    send_uart_command({"cmd": "hold", "waypoint": current_waypoint_index + 1})
                    continue

                bearing = calculate_bearing(
                    state["gps"]["lat"], state["gps"]["lon"],
                    target_lat, target_lon
                )

                if not uart_alive:
                    state["gps"]["heading"] = round(bearing, 1)
                    state["attitude"]["yaw"] = round(bearing, 1)

                    step_meters = MAX_SPEED_MS * dt
                    if dist > 0:
                        fraction = min(1.0, step_meters / dist)
                        state["gps"]["lat"] += (target_lat - state["gps"]["lat"]) * fraction
                        state["gps"]["lon"] += (target_lon - state["gps"]["lon"]) * fraction

                    state["gps"]["speed"] = round(min(MAX_SPEED_MS, dist / 2.0 + 1.0), 2)
                    state["attitude"]["pitch"] = -3.5
                    state["attitude"]["roll"] = 1.0 * math.sin(now)

        time.sleep(UPDATE_INTERVAL)

# ==================== CAMERA FEED (PICAMERA2 / HUD OVERLAY) ====================
_camera = None
_camera_lock = threading.Lock()
_camera_initialized = False

def rotate_frame(frame, angle=180):
    if cv2 is None or frame is None:
        return frame
    if angle == 180:
        return cv2.rotate(frame, cv2.ROTATE_180)
    elif angle == 90:
        return cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    elif angle == 270:
        return cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    return frame

def init_camera():
    global _camera, _camera_initialized
    if _camera_initialized:
        return
    _camera_initialized = True
    print("[Camera] Đang kiểm tra Picamera2 phần cứng...")

    try:
        from picamera2 import Picamera2
        cam = Picamera2()
        config = cam.create_preview_configuration(main={"size": (640, 480), "format": "RGB888"})
        cam.configure(config)
        cam.start()
        time.sleep(1)
        test_frame = cam.capture_array()
        if test_frame is not None:
            _camera = cam
            print("[Camera] ✅ Picamera2 phần cứng hoạt động tốt")
            return
        cam.stop()
    except Exception as e:
        print(f"[Camera] ⚠️ Không tìm thấy Picamera2 hoặc camera bận ({e}). Sử dụng chế độ HUD ngắt kết nối.")
    _camera = None

def get_disconnected_frame_bytes():
    """Tạo frame JPEG thể hiện trạng thái Camera Chưa Kết Nối (bằng cv2 hoặc PIL)"""
    g = state["gps"]
    s = state["sensors"]

    if cv2 is not None and np is not None:
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        frame[:, :] = [18, 14, 10]

        for y in range(0, 480, 40):
            cv2.line(frame, (0, y), (640, y), (30, 24, 18), 1)
        for x in range(0, 640, 40):
            cv2.line(frame, (x, 0), (x, 480), (30, 24, 18), 1)

        cv2.rectangle(frame, (80, 160), (560, 300), (35, 28, 20), -1)
        cv2.rectangle(frame, (80, 160), (560, 300), (0, 140, 255), 2)

        cv2.putText(frame, "[ ! ] CAMERA CHUA KET NOI", (135, 210),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 200, 255), 2)
        cv2.putText(frame, "OPTICAL SENSOR DISCONNECTED / NO SIGNAL", (125, 245),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (160, 160, 160), 1)
        cv2.putText(frame, "HE THONG GIAM SAT KHI VAN HOAT DONG QUA CAN BUS", (110, 275),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 220, 100), 1)

        L_len = 25
        cv2.line(frame, (20, 20), (20 + L_len, 20), (0, 220, 255), 2)
        cv2.line(frame, (20, 20), (20, 20 + L_len), (0, 220, 255), 2)
        cv2.line(frame, (620, 20), (620 - L_len, 20), (0, 220, 255), 2)
        cv2.line(frame, (620, 20), (620, 20 + L_len), (0, 220, 255), 2)
        cv2.line(frame, (20, 460), (20 + L_len, 460), (0, 220, 255), 2)
        cv2.line(frame, (20, 460), (20, 460 - L_len), (0, 220, 255), 2)
        cv2.line(frame, (620, 460), (620 - L_len, 460), (0, 220, 255), 2)
        cv2.line(frame, (620, 460), (620 - L_len, 460), (0, 220, 255), 2)

        cv2.putText(frame, f"ALT: {g['alt']:.1f}m  SPD: {g['speed']:.1f}m/s  HDG: {g['heading']:.0f}'",
                    (25, 435), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 220, 255), 1)
        cv2.putText(frame, f"PM2.5: {s['pm25']:.1f}  CO2: {s['eco2']}  AQI: {s['aqi']} ({s['aqi_category']})",
                    (25, 455), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 120), 1)
        cv2.putText(frame, f"STATE: {flight_state}", (520, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

        _, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        return buf.tobytes()

    elif Image is not None and BytesIO is not None:
        img = Image.new("RGB", (640, 480), color=(18, 14, 10))
        draw = ImageDraw.Draw(img)

        for y in range(0, 480, 40):
            draw.line([(0, y), (640, y)], fill=(30, 24, 18), width=1)
        for x in range(0, 640, 40):
            draw.line([(x, 0), (x, 480)], fill=(30, 24, 18), width=1)

        draw.rectangle([80, 160, 560, 300], fill=(35, 28, 20), outline=(255, 140, 0), width=2)
        draw.text((140, 190), "[ ! ] CAMERA CHUA KET NOI", fill=(0, 220, 255))
        draw.text((120, 225), "OPTICAL SENSOR DISCONNECTED / NO SIGNAL", fill=(160, 160, 160))
        draw.text((105, 255), "HE THONG GIAM SAT KHI VAN HOAT DONG QUA CAN BUS", fill=(0, 220, 100))

        draw.text((25, 430), f"ALT: {g['alt']:.1f}m  SPD: {g['speed']:.1f}m/s  HDG: {g['heading']:.0f}'", fill=(0, 220, 255))
        draw.text((25, 450), f"PM2.5: {s['pm25']:.1f}  CO2: {s['eco2']}  AQI: {s['aqi']} ({s['aqi_category']})", fill=(0, 255, 120))
        draw.text((510, 25), f"STATE: {flight_state}", fill=(0, 255, 255))

        buf = BytesIO()
        img.save(buf, format="JPEG", quality=80)
        return buf.getvalue()

    return b""

def generate_video_stream():
    """Tạo luồng video MJPEG: Real camera nếu có, hoặc Cockpit HUD với cảnh báo CHƯA KẾT NỐI"""
    init_camera()

    while True:
        try:
            if _camera is not None and cv2 is not None:
                with _camera_lock:
                    frame = _camera.capture_array()
                frame = rotate_frame(frame, 180)
                _, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                jpeg_bytes = buf.tobytes()
            else:
                jpeg_bytes = get_disconnected_frame_bytes()

            if jpeg_bytes:
                yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpeg_bytes + b'\r\n'
            time.sleep(0.04)
        except Exception:
            time.sleep(0.1)

# ==================== FLASK ROUTES & API ====================
@app.route("/")
def home():
    return render_template("index.html")

@app.route("/telemetry")
def telemetry():
    return render_template("dashboard.html")

@app.route("/stream")
def stream_page():
    return render_template("stream.html")

@app.route("/video_feed")
def video_feed():
    return Response(generate_video_stream(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route("/api/uav-state")
def uav_state():
    """Endpoint chính cung cấp viễn trắc tổng hợp thời gian thực cho toàn bộ Frontend"""
    with data_lock:
        target_lat, target_lon = None, None
        distance_to_wp = 0.0
        if current_waypoints and current_waypoint_index < len(current_waypoints):
            twp = current_waypoints[current_waypoint_index]
            target_lat = twp["lat"]
            target_lon = twp.get("lng", twp.get("lon"))
            distance_to_wp = calculate_distance(
                state["gps"]["lat"], state["gps"]["lon"],
                target_lat, target_lon
            )

        now = time.time()
        uart_connected = (now - last_uart_rx_time < 3.0)
        can_connected = (now - last_can_update < CAN_TIMEOUT)

        state_payload = {
            "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "armed": state["armed"],
            "mode": state["mode"],
            "flight_state": flight_state,
            "battery": round(state["battery"], 1),
            "voltage": round(state.get("voltage", 16.48), 2),
            "target_altitude": CRUISE_ALTITUDE,
            "terrain_altitude": TERRAIN_ALTITUDE,
            "gps": {
                "lat": round(state["gps"]["lat"], 7),
                "lon": round(state["gps"]["lon"], 7),
                "alt": round(state["gps"]["alt"], 2),
                "speed": round(state["gps"]["speed"], 2),
                "heading": round(state["gps"]["heading"], 1),
                "satellites": state["gps"].get("satellites", 18)
            },
            "attitude": {
                "pitch": round(state["attitude"]["pitch"], 1),
                "roll": round(state["attitude"]["roll"], 1),
                "yaw": round(state["attitude"]["yaw"], 1)
            },
            "sensors": {
                "pm25": round(state["sensors"]["pm25"], 1),
                "pm10": round(state["sensors"]["pm10"], 1),
                "eco2": int(state["sensors"]["eco2"]),
                "co2": int(state["sensors"]["eco2"]),
                "tvoc": int(state["sensors"]["tvoc"]),
                "co": round(state["sensors"]["co"], 2),
                "no2": round(state["sensors"]["no2"], 1),
                "temp": round(state["sensors"]["temp"], 1),
                "hum": round(state["sensors"]["hum"], 1),
                "aqi": int(state["sensors"]["aqi"]),
                "aqi_category": state["sensors"]["aqi_category"],
                "aqi_main_pollutant": state["sensors"]["aqi_main_pollutant"],
                "aqi_components": state["sensors"]["aqi_components"]
            },
            "flight": {
                "flight_state": flight_state,
                "is_flying": is_flying,
                "is_holding": is_holding,
                "hold_samples": hold_samples_collected if is_holding else 0,
                "hold_total": SAMPLE_COUNT if is_holding else 0,
                "current_waypoint": current_waypoint_index,
                "total_waypoints": len(current_waypoints),
                "distance_to_wp": round(distance_to_wp, 1),
                "flight_time": total_flight_seconds
            },
            "hardware": {
                "uart_connected": uart_connected,
                "can_connected": can_connected,
                "uart_port": UART_PORT,
                "uart_baud": UART_BAUD
            },
            "collected_data": collected_data_history
        }

        # Trả về cả state bọc và các trường phẳng để tương thích hoàn hảo với giao diện
        payload = {
            "success": True,
            "state": state_payload,
            "collected_data": collected_data_history,
            "distance_to_wp": round(distance_to_wp, 1),
            "flight_state": flight_state,
            "target_altitude": CRUISE_ALTITUDE,
            "terrain_altitude": TERRAIN_ALTITUDE,
            "battery": round(state["battery"], 1),
            "voltage": round(state.get("voltage", 16.48), 2),
            "altitude": round(state["gps"]["alt"], 1),
            "speed": round(state["gps"]["speed"], 2),
            "heading": round(state["gps"]["heading"], 1),
            "mode": state["mode"],
            "armed": state["armed"],
            "latitude": state["gps"]["lat"],
            "longitude": state["gps"]["lon"],
            "satellites": state["gps"].get("satellites", 18),
            "pitch": round(state["attitude"]["pitch"], 1),
            "roll": round(state["attitude"]["roll"], 1),
            "pm25": state["sensors"]["pm25"],
            "pm10": state["sensors"]["pm10"],
            "eco2": state["sensors"]["eco2"],
            "co2": state["sensors"]["eco2"],
            "tvoc": state["sensors"]["tvoc"],
            "co": state["sensors"]["co"],
            "no2": state["sensors"]["no2"],
            "temperature": state["sensors"]["temp"],
            "humidity": state["sensors"]["hum"],
            "aqi": state["sensors"]["aqi"],
            "aqi_category": state["sensors"]["aqi_category"],
            "current_waypoint": current_waypoint_index,
            "total_waypoints": len(current_waypoints),
            "is_flying": is_flying or flight_state in ["TAKEOFF", "RUNNING", "LANDING"],
            "is_holding": is_holding or flight_state == "HOLD",
            "uart_connected": uart_connected,
            "can_connected": can_connected
        }
    return jsonify(payload)

@app.route("/vehicle-position")
def vehicle_position():
    with data_lock:
        return jsonify({
            "success": True,
            "lat": state["gps"]["lat"],
            "lon": state["gps"]["lon"],
            "alt": state["gps"]["alt"],
            "speed": state["gps"]["speed"],
            "heading": state["gps"]["heading"],
            "flight_state": flight_state
        })

@app.route("/vehicle-info")
def vehicle_info():
    with data_lock:
        return jsonify({
            "success": True,
            "alt": round(state["gps"]["alt"], 1),
            "speed": round(state["gps"]["speed"], 2),
            "heading": round(state["gps"]["heading"], 1),
            "battery": round(state["battery"], 1),
            "voltage": round(state["voltage"], 2),
            "mode": state["mode"],
            "flight_state": flight_state,
            "armed": state["armed"]
        })

@app.route("/api/telemetry")
def api_telemetry():
    with data_lock:
        s = state["sensors"]
        return jsonify({
            "success": True,
            "telemetry": {
                "pm25": s["pm25"],
                "pm10": s["pm10"],
                "eco2": s["eco2"],
                "co2": s["eco2"],
                "tvoc": s["tvoc"],
                "temp": s["temp"],
                "hum": s["hum"],
                "co": s["co"],
                "no2": s["no2"],
                "aqi": s["aqi"],
                "aqi_category": s["aqi_category"],
                "aqi_main_pollutant": s["aqi_main_pollutant"],
                "aqi_components": s["aqi_components"],
                "alt": state["gps"]["alt"],
                "target_altitude": CRUISE_ALTITUDE,
                "terrain_altitude": TERRAIN_ALTITUDE,
                "speed": state["gps"]["speed"],
                "heading": state["gps"]["heading"],
                "armed": state["armed"],
                "battery": round(state["battery"], 1),
                "voltage": round(state["voltage"], 2),
                "mode": state["mode"],
                "flight_state": flight_state,
                "is_holding": is_holding,
                "hold_samples": hold_samples_collected if is_holding else 0,
                "hold_total": SAMPLE_COUNT if is_holding else 0,
                "current_wp": current_waypoint_index + 1 if current_waypoint_index < len(current_waypoints) else len(current_waypoints),
                "total_wp": len(current_waypoints)
            }
        })

@app.route("/upload-mission", methods=["POST"])
def upload_mission():
    """
    Nhận danh sách Waypoint từ GCS:
    QUY TẮC BẮT BUỘC: Chỉ gửi vị trí tọa độ (lat, lng), không gửi độ cao (alt) sang UAV!
    Độ cao hành trình được tự động gán theo tiêu chuẩn CRUISE_ALTITUDE (35m).
    Đồng thời gửi packet JSON qua UART tới Flight Controller của UAV.
    """
    global current_waypoints, current_waypoint_index, is_flying, is_holding
    data = request.get_json()
    if not data or "mission" not in data:
        return jsonify({"success": False, "message": "Thiếu dữ liệu mission"}), 400

    mission_list = []
    uart_waypoints = []
    for w in data["mission"]:
        lat_val = float(w["lat"])
        lng_val = float(w.get("lng", w.get("lon")))
        mission_list.append({
            "lat": lat_val,
            "lng": lng_val,
            "alt": CRUISE_ALTITUDE
        })
        # Chỉ gửi tọa độ vị trí thuần túy, không gửi trường alt
        uart_waypoints.append({
            "lat": round(lat_val, 7),
            "lng": round(lng_val, 7)
        })

    with data_lock:
        current_waypoints = mission_list
        current_waypoint_index = 0
        is_flying = False
        is_holding = False

    # Gửi qua UART tới UAV
    uart_sent = send_uart_command({
        "cmd": "upload_mission",
        "count": len(uart_waypoints),
        "waypoints": uart_waypoints
    })

    print(f"[Mission] ✅ Đã nạp {len(current_waypoints)} Waypoints (chỉ tọa độ) vào hệ thống! UART TX: {uart_sent}")
    return jsonify({
        "success": True,
        "message": f"Đã nạp {len(current_waypoints)} waypoints",
        "uart_transmitted": uart_sent
    })

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
                state["target_altitude"] = CRUISE_ALTITUDE
                for w in current_waypoints:
                    w["alt"] = CRUISE_ALTITUDE
            print(f"[Mission] 📐 Đã cài đặt độ cao cất cánh & hành trình: {CRUISE_ALTITUDE}m")
            return jsonify({"success": True, "target_altitude": CRUISE_ALTITUDE, "message": f"Đã thiết lập độ cao {CRUISE_ALTITUDE}m"})
        except Exception as e:
            return jsonify({"success": False, "message": str(e)}), 400
    return jsonify({"success": False, "message": "Thiếu thông số altitude"}), 400

@app.route("/start-mission", methods=["POST"])
def start_mission():
    global is_flying, is_holding, current_waypoint_index, flight_state, CRUISE_ALTITUDE
    if not current_waypoints:
        return jsonify({"success": False, "message": "Chưa có Waypoint nào được tải lên!"}), 400

    data = request.get_json(silent=True) or {}
    if "alt" in data or "altitude" in data:
        try:
            val = float(data.get("alt", data.get("altitude")))
            val = max(5.0, min(150.0, val))
            with data_lock:
                CRUISE_ALTITUDE = val
                state["target_altitude"] = CRUISE_ALTITUDE
                for w in current_waypoints:
                    w["alt"] = CRUISE_ALTITUDE
        except Exception:
            pass

    with data_lock:
        state["armed"] = True
        current_waypoint_index = 0
        is_holding = False
        state["mode"] = "AUTO"

        if state["gps"]["alt"] < 5.0:
            flight_state = "TAKEOFF"
            is_flying = False
            print(f"[Mission] 🚀 Cất cánh (TAKEOFF) bắt đầu nhiệm vụ lên {CRUISE_ALTITUDE}m")
        else:
            flight_state = "RUNNING"
            is_flying = True
            print(f"[Mission] 🚀 Bắt đầu bay hành trình (RUNNING) giữ độ cao {CRUISE_ALTITUDE}m")

    send_uart_command({"cmd": "start_mission", "alt": CRUISE_ALTITUDE})
    return jsonify({"success": True, "message": "Nhiệm vụ bắt đầu thành công", "target_altitude": CRUISE_ALTITUDE})

@app.route("/takeoff", methods=["POST"])
def takeoff():
    global flight_state, is_flying, is_holding, CRUISE_ALTITUDE
    data = request.get_json(silent=True) or {}
    if "alt" in data or "altitude" in data:
        try:
            val = float(data.get("alt", data.get("altitude")))
            val = max(5.0, min(150.0, val))
            with data_lock:
                CRUISE_ALTITUDE = val
                state["target_altitude"] = CRUISE_ALTITUDE
                for w in current_waypoints:
                    w["alt"] = CRUISE_ALTITUDE
        except Exception:
            pass

    with data_lock:
        state["armed"] = True
        flight_state = "TAKEOFF"
        is_flying = False
        is_holding = False
        state["mode"] = "GUIDED"
    send_uart_command({"cmd": "takeoff", "alt": CRUISE_ALTITUDE})
    print(f"[Control] 🛫 Lệnh cất cánh (TAKEOFF lên {CRUISE_ALTITUDE}m) đã gửi qua UART")
    return jsonify({"success": True, "message": f"UAV đang cất cánh lên {CRUISE_ALTITUDE}m", "flight_state": flight_state, "target_altitude": CRUISE_ALTITUDE})

@app.route("/save-current-waypoint", methods=["POST"])
def save_current_waypoint():
    """Lưu thông số hiện tại của vị trí và cảm biến thành một điểm đo lịch sử."""
    with data_lock:
        wp_idx = len(wp_csv_files)
        lat = state["gps"]["lat"]
        lon = state["gps"]["lon"]
        alt = state["gps"]["alt"]
        now_iso = datetime.now().isoformat()
        s_copy = dict(state["sensors"])
        readings = []
        for i in range(10):
            readings.append({
                "timestamp": now_iso,
                "pm25": s_copy.get("pm25", 0.0),
                "pm10": s_copy.get("pm10", 0.0),
                "eco2": s_copy.get("eco2", 0),
                "tvoc": s_copy.get("tvoc", 0),
                "temp": s_copy.get("temp", 0.0),
                "hum": s_copy.get("hum", 0.0),
                "aqi": s_copy.get("aqi", 1),
                "co": s_copy.get("co", 0.0),
                "no2": s_copy.get("no2", 0.0)
            })
        
        avg_reading = {
            "lat": lat,
            "lon": lon,
            "alt": alt,
            "time": now_iso,
            "waypoint_index": wp_idx,
            "pm25": s_copy.get("pm25", 0.0),
            "pm10": s_copy.get("pm10", 0.0),
            "eco2": s_copy.get("eco2", 0),
            "tvoc": s_copy.get("tvoc", 0),
            "temp": s_copy.get("temp", 0.0),
            "hum": s_copy.get("hum", 0.0),
            "aqi": s_copy.get("aqi", 1),
            "aqi_category": s_copy.get("aqi_category", "Tốt"),
            "aqi_color": s_copy.get("aqi_components", {}).get("color", "#157a3a"),
            "co": s_copy.get("co", 0.0),
            "no2": s_copy.get("no2", 0.0),
            "sample_count": 10
        }
        collected_data_history.append(avg_reading)
        save_wp_raw_csv(wp_idx, readings, lat, lon, alt, avg_reading=avg_reading)
    
    print(f"[Mission] 💾 Đã lưu điểm đo #{wp_idx + 1} vào lịch sử quan trắc")
    return jsonify({"success": True, "message": f"Đã lưu điểm đo #{wp_idx + 1}", "data": avg_reading})

@app.route("/stop-mission", methods=["POST"])
def stop_mission():
    global is_flying, is_holding, flight_state
    with data_lock:
        is_flying = False
        is_holding = False
        flight_state = "IDLE" if state["gps"]["alt"] < 1.0 else "HOLD"
        state["mode"] = "GUIDED"
    send_uart_command({"cmd": "stop_mission"})
    print("[Mission] ⏸️ Tạm dừng nhiệm vụ")
    return jsonify({"success": True, "message": "Đã dừng nhiệm vụ"})

@app.route("/clear-mission", methods=["POST"])
def clear_mission():
    global current_waypoints, current_waypoint_index, is_flying, is_holding, flight_state
    with data_lock:
        current_waypoints = []
        current_waypoint_index = 0
        is_flying = False
        is_holding = False
        if state["gps"]["alt"] < 1.0:
            flight_state = "IDLE"
    send_uart_command({"cmd": "clear_mission"})
    return jsonify({"success": True, "message": "Đã xóa toàn bộ Waypoint"})

@app.route("/get-mission")
def get_mission():
    return jsonify({"success": True, "mission": current_waypoints})

@app.route("/arm", methods=['POST'])
def arm():
    global flight_state, is_flying
    with data_lock:
        state["armed"] = True
        if state["gps"]["alt"] < 1.0:
            flight_state = "TAKEOFF"
        else:
            flight_state = "RUNNING"
            is_flying = True
    send_uart_command({"cmd": "arm"})
    print("[Control] ⚡ ARMED")
    return jsonify({"success": True, "message": "Armed"})

@app.route("/disarm", methods=['POST'])
def disarm():
    global flight_state, is_flying, is_holding
    with data_lock:
        state["armed"] = False
        is_flying = False
        is_holding = False
        flight_state = "IDLE"
        state["mode"] = "MANUAL"
    send_uart_command({"cmd": "disarm"})
    print("[Control] 🛑 DISARMED")
    return jsonify({"success": True, "message": "Disarmed"})

@app.route("/rtl", methods=['POST'])
def rtl():
    global flight_state, is_flying, is_holding
    with data_lock:
        state["mode"] = "RTL"
        is_holding = False
        flight_state = "LANDING"
    send_uart_command({"cmd": "rtl"})
    print("[Control] 🏠 Return To Launch (RTL) -> LANDING")
    return jsonify({"success": True, "message": "Return to Launch"})

@app.route("/set-mode", methods=['POST'])
def set_mode():
    data = request.get_json() or {}
    mode = data.get("mode", "GUIDED")
    with data_lock:
        state["mode"] = mode
    send_uart_command({"cmd": "set_mode", "mode": mode})
    return jsonify({"success": True, "mode": state["mode"]})

@app.route("/set-data-logging", methods=["POST"])
def set_data_logging():
    global data_logging_enabled
    data = request.get_json()
    data_logging_enabled = data.get("enabled", True)
    return jsonify({"success": True, "enabled": data_logging_enabled})

@app.route("/get-data-logging")
def get_data_logging():
    return jsonify({"success": True, "enabled": data_logging_enabled})

@app.route("/get-collected-data")
def get_collected_data():
    return jsonify({"success": True, "data": collected_data_history})

@app.route("/download-csv")
def download_csv():
    """Xuất file CSV tổng hợp tất cả các điểm đo (hỗ trợ tiếng Việt Excel qua UTF-8-sig)."""
    if os.path.exists(SUMMARY_CSV_PATH) and os.path.getsize(SUMMARY_CSV_PATH) > 0:
        return send_file(
            SUMMARY_CSV_PATH,
            mimetype="text/csv; charset=utf-8",
            as_attachment=True,
            download_name=f"uav_waypoint_history_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        )
    elif collected_data_history:
        import io
        output = io.StringIO()
        writer = csv.writer(output)
        writer.writerow([
            "STT", "Điểm đo", "Thời gian", "Vĩ độ (Lat)", "Kinh độ (Lon)", "Độ cao (m)", "Số mẫu",
            "PM2.5 (µg/m³)", "PM10 (µg/m³)", "CO₂ (mg/m³)", "TVOC (ppb)",
            "CO (mg/m³)", "NO₂ (µg/m³)", "Nhiệt độ (°C)", "Độ ẩm (%)", "Chỉ số VN_AQI", "Chất lượng không khí"
        ])
        for idx, d in enumerate(collected_data_history, 1):
            writer.writerow([
                idx, f"Điểm #{d.get('waypoint_index', idx-1) + 1}", d.get("time", ""),
                d.get("lat", ""), d.get("lon", ""), d.get("alt", ""), d.get("sample_count", 10),
                d.get("pm25", ""), d.get("pm10", ""), d.get("eco2", ""), d.get("tvoc", ""),
                d.get("co", ""), d.get("no2", ""), d.get("temp", ""), d.get("hum", ""),
                d.get("aqi", 1), d.get("aqi_category", "Tốt")
            ])
        csv_bytes = ('\ufeff' + output.getvalue()).encode('utf-8')
        from io import BytesIO
        return send_file(
            BytesIO(csv_bytes),
            mimetype="text/csv; charset=utf-8",
            as_attachment=True,
            download_name=f"uav_waypoint_history_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        )
    return jsonify({"success": False, "message": "Chưa có dữ liệu trạm để xuất"}), 404

@app.route("/export-csv")
def export_csv():
    return download_csv()

@app.route("/export-json")
def export_json():
    return jsonify({"success": True, "data": collected_data_history})

# ==================== WAYPOINT CSV DATA DOWNLOAD ENDPOINTS ====================
@app.route("/wp-data/list")
def wp_data_list():
    return jsonify({"success": True, "files": wp_csv_files, "total": len(wp_csv_files)})

@app.route("/wp-data/download/<filename>")
def wp_data_download(filename):
    safe = os.path.basename(filename)
    filepath = os.path.join(WP_DATA_DIR, safe)
    if not os.path.exists(filepath):
        return jsonify({"success": False, "message": "File không tồn tại"}), 404
    return send_file(filepath, mimetype="text/csv; charset=utf-8", as_attachment=True, download_name=safe)

@app.route("/wp-data/download-all")
def wp_data_download_all():
    import zipfile
    if not wp_csv_files and not (os.path.exists(SUMMARY_CSV_PATH) and os.path.getsize(SUMMARY_CSV_PATH) > 0):
        return jsonify({"success": False, "message": "Chưa có file CSV"}), 404
    from io import BytesIO
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
    return send_file(buf, mimetype="application/zip", as_attachment=True, download_name=f"wp_datasets_{ts}.zip")

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
    print("[STORAGE] 🗑️ Đã xóa toàn bộ lịch sử điểm đo")
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

    print(f"[STORAGE] 🗑️ Đã xóa dữ liệu điểm đo (file: {target_fn}, wp: {target_wp})")
    return jsonify({"success": True, "message": "Đã xóa điểm đo thành công", "deleted": deleted})

# ==================== CAMERA CAPTURE / RECORD ENDPOINTS ====================
_recorder = None
_recorder_lock = threading.Lock()
_recording_filename = None

@app.route("/camera/snapshot", methods=["POST"])
def camera_snapshot():
    try:
        filename = f"snapshot_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg"
        filepath = os.path.join(CAMERA_DIR, filename)

        if _camera and cv2 is not None:
            with _camera_lock:
                frame = _camera.capture_array()
            frame = rotate_frame(frame, 180)
            cv2.imwrite(filepath, frame)
        else:
            jpeg_bytes = get_disconnected_frame_bytes()
            if jpeg_bytes:
                with open(filepath, "wb") as f:
                    f.write(jpeg_bytes)
            elif Image is not None:
                img = Image.new("RGB", (640, 480), color=(18, 14, 10))
                draw = ImageDraw.Draw(img)
                draw.text((50, 240), f"SNAPSHOT (NO CAMERA) {datetime.now().strftime('%H:%M:%S')}", fill=(0, 200, 255))
                img.save(filepath)

        return jsonify({"success": True, "filename": filename, "path": filepath})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)}), 500

@app.route("/camera_captures/<filename>")
def serve_capture(filename):
    return send_from_directory(os.path.abspath(CAMERA_DIR), filename)

@app.route("/camera/record/start", methods=["POST"])
def camera_record_start():
    global _recorder, _recording_filename
    if cv2 is None:
        return jsonify({"success": False, "error": "OpenCV chưa được cài đặt để ghi hình MP4"}), 501

    with _recorder_lock:
        if _recorder is not None:
            return jsonify({"success": False, "error": "Đang trong quá trình ghi hình"}), 400
        _recording_filename = f"video_{datetime.now().strftime('%Y%m%d_%H%M%S')}.mp4"
        filepath = os.path.join(CAMERA_DIR, _recording_filename)
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        _recorder = cv2.VideoWriter(filepath, fourcc, 20.0, (640, 480))
        if not _recorder.isOpened():
            _recorder = None
            return jsonify({"success": False, "error": "Không thể mở VideoWriter"}), 500

    def record_loop():
        global _recorder
        while True:
            with _recorder_lock:
                if _recorder is None:
                    break
            if _camera:
                with _camera_lock:
                    frame = _camera.capture_array()
                bgr = rotate_frame(frame, 180)
            else:
                bgr = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(bgr, f"REC (NO CAMERA) {datetime.now().strftime('%H:%M:%S')}", (20, 240),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            with _recorder_lock:
                if _recorder is not None:
                    _recorder.write(bgr)
            time.sleep(1 / 20)

    threading.Thread(target=record_loop, daemon=True).start()
    return jsonify({"success": True, "filename": _recording_filename})

@app.route("/camera/record/stop", methods=["POST"])
def camera_record_stop():
    global _recorder, _recording_filename
    with _recorder_lock:
        if _recorder is None:
            return jsonify({"success": False, "error": "Không có phiên ghi hình nào đang chạy"}), 400
        _recorder.release()
        _recorder = None
        fn = _recording_filename
        _recording_filename = None
    return jsonify({"success": True, "filename": fn})

@app.route("/camera_captures/video/<filename>")
def serve_video(filename):
    return send_from_directory(os.path.abspath(CAMERA_DIR), filename)

# ==================== LOGGING VÀ FLIGHT RECORDER THREAD ====================
def flight_recorder_thread():
    fn = os.path.join(LOGS_DIR, f"telemetry_{time.strftime('%Y%m%d_%H%M%S')}.csv")
    with open(fn, 'w', newline='', encoding='utf-8') as f:
        csv.writer(f).writerow([
            "Timestamp", "State", "Lat", "Lon", "Alt(m)", "Speed(m/s)", "Heading",
            "PM2.5", "PM10", "eCO2", "TVOC", "CO", "NO2", "Temp", "Hum", "AQI"
        ])
    while simulation_running:
        time.sleep(3)
        try:
            with data_lock:
                g, s = state["gps"], state["sensors"]
                with open(fn, 'a', newline='', encoding='utf-8') as f:
                    csv.writer(f).writerow([
                        datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                        flight_state,
                        g["lat"], g["lon"], g["alt"], g["speed"], g["heading"],
                        s["pm25"], s["pm10"], s["eco2"], s["tvoc"], s["co"], s["no2"],
                        s["temp"], s["hum"], s["aqi"]
                    ])
        except Exception:
            pass

# ==================== MAIN SERVER ENTRYPOINT ====================
if __name__ == "__main__":
    print("=" * 65)
    print("  🚁 UAV GROUND CONTROL STATION - REAL HARDWARE BACKEND")
    print("  📍 GCS UI: http://localhost:8000")
    print("  📡 CAN Bus (0x555 / 0x556): " + ("✅ can0 online" if can_bus else "⚠️ SocketCAN can0 fallback/waiting"))
    print("  📶 UART Serial (Telemetry/Mission): " + (f"✅ {UART_PORT} @ {UART_BAUD}" if (uart_serial and uart_serial.is_open) else f"⚠️ {UART_PORT} @ {UART_BAUD} (waiting connection)"))
    print("  🔌 I2C Sensors: " + ("✅ ENS160+AHT21 online" if i2c_bus else "⚠️ I2C not attached"))
    print("  📸 Camera: " + ("✅ Picamera2 online" if _camera else "⚠️ Disconnected HUD fallback"))
    print("  🇻🇳 AQI: QCVN 05:2023 / Vietnam AQI standard (0-500 scale)")
    print("  🗺️ Waypoints: Coordinate-only mission upload active (lat, lng only)")
    print("=" * 65)

    # Khởi động các luồng xử lý nền
    threading.Thread(target=uart_reader_thread, daemon=True).start()
    threading.Thread(target=can_reader_thread, daemon=True).start()
    threading.Thread(target=i2c_sensor_fallback_thread, daemon=True).start()
    threading.Thread(target=flight_navigation_loop, daemon=True).start()
    threading.Thread(target=flight_recorder_thread, daemon=True).start()

    app.run(host="0.0.0.0", port=8000, threaded=True)