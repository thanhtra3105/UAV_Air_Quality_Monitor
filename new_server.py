"""
UAV Dashboard - Pi 5 Server
Sensors: ENS160 + AHT21
Air sensor link: CAN MCP2515 from ESP32
Camera: Picamera2
"""
import os, time, threading, csv, cv2, math, random
import numpy as np
from flask import Flask, request, jsonify, render_template, Response
from flask_cors import CORS
from datetime import datetime
from smbus2 import SMBus, i2c_msg
import can
import pandas as pd
import json


app = Flask(__name__)
CORS(app)

ENS160_ADDR = 0x53
AHT21_ADDR = 0x38

# ==================== CAN CONFIG ====================
# ESP32 gửi JSON đã chia thành nhiều frame CAN classic 8 byte.
# Format mỗi frame:
#   byte0: sequence number
#   byte1: total JSON length
#   byte2..7: 6 byte payload JSON
CAN_ID_JSON = 0x555
CAN_TIMEOUT = 10
# True: dữ liệu cảm biến không khí lấy từ ESP32 qua CAN, không để I2C fallback ghi đè.
USE_CAN_SENSOR_DATA = True
last_can_update = time.time()

try:
    can_bus = can.interface.Bus(channel="can0", interface="socketcan")
    print("[CAN] ✅ can0 initialized")
except Exception as e:
    print(f"[CAN] ⚠️ CAN init error: {e}")
    can_bus = None

try:
    bus = SMBus(1)
    print("[Sensors] ✅ I2C bus initialized")
    
    bus.write_byte_data(ENS160_ADDR, 0x10, 0x02)
    print("[Sensors] ✅ ENS160 initialized (active mode)")
    time.sleep(1)
    
except Exception as e:
    print(f"[Sensors] ⚠️ I2C error: {e}")
    bus = None


def can_reader_thread():
    """Nhận JSON từ ESP32 qua CAN MCP2515 và cập nhật state.

    Protocol CAN classic:
      ID 0x555
      data[0] = seq
      data[1] = total_len JSON
      data[2:8] = payload ASCII/UTF-8, tối đa 6 byte/frame
    """
    global last_can_update

    print("[CAN] Thread started")

    buffer = bytearray()
    expected_len = None
    last_packet_time = time.time()

    while True:
        try:
            if can_bus is None:
                time.sleep(1)
                continue

            msg = can_bus.recv(timeout=1.0)
            if msg is None:
                continue

            if msg.arbitration_id != CAN_ID_JSON:
                continue

            frame = bytes(msg.data)
            if len(frame) != 8:
                continue

            seq = frame[0]
            total_len = frame[1]
            payload = frame[2:8]

            # Frame đầu của gói JSON mới
            if seq == 0:
                buffer = bytearray()
                expected_len = total_len
                last_packet_time = time.time()

            if expected_len is None:
                continue

            # Nếu mất frame hoặc quá lâu chưa đủ gói thì reset buffer
            if time.time() - last_packet_time > 2:
                print("[CAN] Packet timeout, reset buffer")
                buffer = bytearray()
                expected_len = None
                continue

            buffer.extend(payload)
            buffer = buffer[:expected_len]

            if len(buffer) >= expected_len:
                try:
                    json_str = buffer.decode("utf-8")
                    data = json.loads(json_str)

                    # ESP32 gửi JSON gồm:
                    # {"battery":...,"pm25":...,"co":...,"no2":...,"co2":...,"tvoc":...,"temp":...,"hum":...}
                    # Quy ước giữ tương thích code UART cũ:
                    #   pm25 gửi dạng mg/m3  -> Pi nhân 1000 thành µg/m3
                    #   co   gửi dạng x1000 -> Pi chia 1000 thành mg/m3
                    #   no2, co2, tvoc, temp, hum giữ nguyên theo đơn vị hiển thị/dashboard
                    state["battery"] = data.get("battery", state["battery"])

                    if "pm25" in data:
                        state["sensors"]["pm25"] = data.get("pm25", state["sensors"].get("pm25", 0)) * 1000

                    if "co" in data:
                        state["sensors"]["co"] = data.get("co", state["sensors"].get("co", 0)) / 1000

                    if "no2" in data:
                        state["sensors"]["no2"] = data.get("no2", state["sensors"].get("no2", 0))

                    if "co2" in data:
                        # Dashboard đang dùng key eco2/co2 lấy từ state["sensors"]["eco2"]
                        state["sensors"]["eco2"] = data.get("co2", state["sensors"].get("eco2", 0))

                    if "tvoc" in data:
                        state["sensors"]["tvoc"] = data.get("tvoc", state["sensors"].get("tvoc", 0))

                    if "temp" in data:
                        state["sensors"]["temp"] = data.get("temp", state["sensors"].get("temp", 0))

                    if "hum" in data:
                        state["sensors"]["hum"] = data.get("hum", state["sensors"].get("hum", 0))

                    last_can_update = time.time()
                    print(f"[CAN] JSON OK: {json_str}")
                    print(
                        "[CAN] Sensors => "
                        f"bat={state['battery']:.1f}%, "
                        f"PM2.5={state['sensors']['pm25']:.2f} µg/m³, "
                        f"CO={state['sensors']['co']:.2f} mg/m³, "
                        f"NO2={state['sensors']['no2']:.1f} µg/m³, "
                        f"CO2={state['sensors']['eco2']:.1f}, "
                        f"TVOC={state['sensors']['tvoc']:.1f}, "
                        f"Temp={state['sensors']['temp']:.1f} °C, "
                        f"Hum={state['sensors']['hum']:.1f}%"
                    )

                except Exception as e:
                    print(f"[CAN] JSON parse error: {e}")
                    print(f"[CAN] Raw buffer: {buffer}")

                buffer = bytearray()
                expected_len = None

        except Exception as e:
            print(f"[CAN] Error: {e}")
            time.sleep(1)

def read_u16(addr, reg):
    """Đọc 16-bit little-endian từ ENS160"""
    if bus is None:
        return 0
    try:
        low = bus.read_byte_data(addr, reg)
        high = bus.read_byte_data(addr, reg + 1)
        return (high << 8) | low
    except Exception as e:
        print(f"[read_u16] ERROR addr={hex(addr)} reg={hex(reg)}: {e}")
        return 0


def aht21_direct_read(length):
    """AHT21 đọc direct read, không đọc theo register 0x00"""
    try:
        msg = i2c_msg.read(AHT21_ADDR, length)
        bus.i2c_rdwr(msg)
        return list(msg)
    except Exception as e:
        print(f"[AHT21] Direct read error: {e}")
        return None


def reset_aht21():
    """Reset mềm AHT21"""
    try:
        bus.write_byte(AHT21_ADDR, 0xBA)
        time.sleep(0.15)
    except Exception as e:
        print(f"[AHT21] Reset error: {e}")


def init_aht21():
    """Khởi tạo/calibrate AHT21"""
    try:
        time.sleep(0.1)

        data = aht21_direct_read(1)
        if data is None:
            return False

        status = data[0]
        print("[AHT21] Init status:", hex(status))

        # Bit 3 = calibrated
        if (status & 0x08) == 0:
            bus.write_i2c_block_data(AHT21_ADDR, 0xBE, [0x08, 0x00])
            time.sleep(0.1)

        return True

    except Exception as e:
        print(f"[AHT21] Init error: {e}")
        return False


def read_aht21():
    """Đọc nhiệt độ và độ ẩm từ AHT21"""
    try:
        bus.write_i2c_block_data(AHT21_ADDR, 0xAC, [0x33, 0x00])
        time.sleep(0.12)
        data = None
        for _ in range(20):
            data = aht21_direct_read(6)

            if data is None or len(data) < 6:
                time.sleep(0.02)
                continue

            status = data[0]

            if (status & 0x80) == 0:
                break

            time.sleep(0.03)

        if data is None or len(data) < 6:
            print("[AHT21] No valid data")
            return None, None

        status = data[0]

        if status & 0x80:
            print("[AHT21] Sensor still busy after retry")
            print("AHT21 raw data:", data)
            print("AHT21 status:", hex(status))
            return None, None

        if (status & 0x08) == 0:
            print("[AHT21] Not calibrated")
            return None, None

        raw_humidity = (
            ((data[1] & 0xFF) << 12) |
            ((data[2] & 0xFF) << 4) |
            ((data[3] & 0xF0) >> 4)
        )

        raw_temp = (
            ((data[3] & 0x0F) << 16) |
            ((data[4] & 0xFF) << 8) |
            (data[5] & 0xFF)
        )

        humidity = raw_humidity * 100.0 / 1048576.0
        temperature = raw_temp * 200.0 / 1048576.0 - 52.0

        humidity = max(0.0, min(100.0, humidity))

        print("AHT21 raw data:", data)
        print("AHT21 status:", hex(status))
        print("Raw Humidity:", raw_humidity)
        print("Humidity:", humidity)
        print("Raw Temp:", raw_temp)
        print("Temperature:", temperature)

        return temperature, humidity

    except Exception as e:
        print(f"[AHT21] Read error: {e}")
        return None, None


def read_sensors():
    """Đọc ENS160 + AHT21"""
    if bus is None:
        return None

    try:
        ens160_aqi = bus.read_byte_data(ENS160_ADDR, 0x21)
        tvoc = read_u16(ENS160_ADDR, 0x22)
        eco2 = read_u16(ENS160_ADDR, 0x24)

        temperature, humidity = read_aht21()

        if temperature is None or humidity is None:
            print("[Sensors] Retry AHT21 with reset...")
            reset_aht21()
            init_aht21()
            temperature, humidity = read_aht21()

        if temperature is None or humidity is None:
            print("[Sensors] AHT21 read failed")
            return None

        return {
            "ens160_aqi": ens160_aqi,
            "tvoc": tvoc,
            "eco2": eco2,
            "temp": round(temperature, 2),
            "hum": round(humidity, 2),
        }

    except Exception as e:
        print(f"[Sensors] ERROR: {e}")
        return None


state = {
    "timestamp": "",
    "armed": False,
    "mode": "MANUAL",
    "battery": 0.0,
    "gps": {"lat": 16.07570, "lon": 108.15338, "alt": 30.0, "speed": 0.0, "heading": 0.0},
    "sensors": {
        "pm25": 0.0, "eco2": 0.0, "tvoc": 0.0, "temp": 0.0, "hum": 0.0, "aqi": 0,
        "aqi_category": "--", "aqi_main_pollutant": None, "aqi_components": {},
        "co": 0.0, "no2": 0.0
    },
}

current_waypoints = []
current_waypoint_index = 0
is_flying = False
is_holding = False
hold_start_time = 0
hold_samples_collected = 0
collected_readings = []
collected_data_history = []
data_logging_enabled = True
wp_csv_files = []  # {"wp", "filename", "samples", "lat", "lon", "alt", "time"}

WAYPOINT_RADIUS = 5.0
HOLD_TIME = 60.0
SAMPLE_COUNT = 30
FLIGHT_SPEED = 0.00002
FLIGHT_ALT_SPEED = 0.01
MAX_SPEED_MS = 3.0
UPDATE_INTERVAL = 0.1

csv_logging_active = False
csv_samples_collected = 0
csv_max_samples = 30
csv_filename = "sensor_data.csv"
csv_position = "default_position"
csv_file_initialized = False

def write_wp_csv(wp_index, readings, lat, lon, alt):
    """Ghi 30 raw samples của một waypoint ra file CSV riêng trong thư mục wp_data/."""
    os.makedirs("wp_data", exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"wp{wp_index + 1:02d}_{ts}.csv"
    filepath = os.path.join("wp_data", filename)
    fieldnames = ['sample', 'timestamp', 'lat', 'lon', 'alt', 'pm25', 'eco2', 'tvoc', 'temp', 'hum', 'aqi', 'co', 'no2']
    with open(filepath, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for i, r in enumerate(readings, 1):
            _co  = r.get("co")
            _no2 = r.get("no2")
            writer.writerow({
                "sample":    i,
                "timestamp": r.get("timestamp", ""),
                "lat":       round(lat, 7),
                "lon":       round(lon, 7),
                "alt":       round(alt, 2),
                "pm25":      safe_round(r.get("pm25"), 2),
                "eco2":      safe_round(r.get("eco2"), 1),
                "tvoc":      safe_round(r.get("tvoc"), 1),
                "temp":      safe_round(r.get("temp"), 2),
                "hum":       safe_round(r.get("hum"), 2),
                "aqi":       int(round(safe_number(r.get("aqi"), 0))),
                "co":        safe_round(_co, 2)  if _co  is not None else '',
                "no2":       safe_round(_no2, 1) if _no2 is not None else '',
            })
    wp_csv_files.append({
        "wp":       wp_index + 1,
        "filename": filename,
        "samples":  len(readings),
        "lat":      round(lat, 6),
        "lon":      round(lon, 6),
        "alt":      round(alt, 1),
        "time":     datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
    })
    print(f"[WP-CSV] ✅ WP{wp_index + 1} → {filepath} ({len(readings)} mẫu)")
    return filename

def calculate_distance(lat1, lon1, lat2, lon2):
    R = 6371000
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    a = math.sin(delta_phi/2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return R * c

def safe_number(value, default=0.0):
    """Ép giá trị sensor về số an toàn để mission thread không bị crash khi gặp None/NaN."""
    try:
        if value is None:
            return default
        value = float(value)
        if math.isnan(value) or math.isinf(value):
            return default
        return value
    except (TypeError, ValueError):
        return default

def safe_round(value, digits=2, default=0.0):
    return round(safe_number(value, default), digits)

def average_readings(readings, key, default=0.0):
    values = [safe_number(r.get(key), None) for r in readings]
    values = [v for v in values if v is not None]
    if not values:
        return default
    return sum(values) / len(values)

simulation_running = True

def simulation_loop():
    global current_waypoint_index, is_flying, is_holding, hold_start_time
    global hold_samples_collected, collected_readings

    while simulation_running:
        try:
            current_time = time.time()

            if is_holding:
                if current_time - hold_start_time >= HOLD_TIME:
                    if collected_readings and data_logging_enabled:
                        avg_reading = {
                            "lat": state["gps"]["lat"],
                            "lon": state["gps"]["lon"],
                            "alt": state["gps"]["alt"],
                            "time": datetime.now().isoformat(),
                            "waypoint_index": current_waypoint_index,
                            "pm25": average_readings(collected_readings, "pm25"),
                            "eco2": average_readings(collected_readings, "eco2"),
                            "tvoc": average_readings(collected_readings, "tvoc"),
                            "temp": average_readings(collected_readings, "temp"),
                            "hum": average_readings(collected_readings, "hum"),
                            "aqi": int(round(average_readings(collected_readings, "aqi"))),
                            "co": average_readings(collected_readings, "co") if any(r.get("co") is not None for r in collected_readings) else None,
                            "no2": average_readings(collected_readings, "no2") if any(r.get("no2") is not None for r in collected_readings) else None,
                            "sample_count": len(collected_readings)
                        }

                        collected_data_history.append(avg_reading)
                        try:
                            write_wp_csv(
                                current_waypoint_index,
                                collected_readings,
                                state["gps"]["lat"],
                                state["gps"]["lon"],
                                state["gps"]["alt"],
                            )
                            print(f"[Mission] ✅ Saved data at WP{current_waypoint_index + 1}")
                        except Exception as e:
                            print(f"[Mission] ⚠️ Failed to save WP{current_waypoint_index + 1} CSV: {e}")

                    # Luôn thoát HOLD và chuyển waypoint, kể cả khi CSV lỗi hoặc mẫu bị thiếu.
                    is_holding = False
                    hold_samples_collected = 0
                    collected_readings = []
                    current_waypoint_index += 1

                    if current_waypoint_index >= len(current_waypoints):
                        is_flying = False
                        state["gps"]["speed"] = 0
                        state["mode"] = "GUIDED"
                        print("[Mission] ✅ Completed all waypoints!")
                    else:
                        state["mode"] = "AUTO"
                        print(f"[Mission] ✈️ Moving to waypoint {current_waypoint_index + 1}")
                else:
                    sample_interval = HOLD_TIME / SAMPLE_COUNT
                    expected_sample_index = int((current_time - hold_start_time) / sample_interval)

                    if expected_sample_index >= hold_samples_collected and hold_samples_collected < SAMPLE_COUNT:
                        reading = {
                            "timestamp": datetime.now().isoformat(),
                            "lat": state["gps"]["lat"],
                            "lon": state["gps"]["lon"],
                            "alt": state["gps"]["alt"],
                            "waypoint_index": current_waypoint_index,
                            "pm25": safe_number(state["sensors"].get("pm25"), 0.0),
                            "eco2": safe_number(state["sensors"].get("eco2"), 0.0),
                            "tvoc": safe_number(state["sensors"].get("tvoc"), 0.0),
                            "temp": safe_number(state["sensors"].get("temp"), 0.0),
                            "hum": safe_number(state["sensors"].get("hum"), 0.0),
                            "aqi": int(round(safe_number(state["sensors"].get("aqi"), 0))),
                            "co": safe_number(state["sensors"].get("co"), 0.0),
                            "no2": safe_number(state["sensors"].get("no2"), 0.0)
                        }
                        collected_readings.append(reading)
                        hold_samples_collected += 1
                        print(f"[Mission] 📊 Sample {hold_samples_collected}/{SAMPLE_COUNT} at WP{current_waypoint_index + 1}")

                time.sleep(0.1)
                continue

            if is_flying and current_waypoints and current_waypoint_index < len(current_waypoints):
                target = current_waypoints[current_waypoint_index]
                distance = calculate_distance(
                    state["gps"]["lat"], state["gps"]["lon"],
                    target["lat"], target["lng"]
                )

                if distance <= WAYPOINT_RADIUS and not is_holding:
                    if not data_logging_enabled:
                        # Chế độ "fly-to-point": tới đích thì dừng, không thu thập, không bay tiếp
                        is_flying = False
                        state["gps"]["speed"] = 0
                        state["mode"] = "GUIDED"
                        print(f"[Mission] 📍 Reached WP{current_waypoint_index + 1}, hovering (auto-collect OFF)")
                        continue
                    print(f"[Mission] 📍 Reached waypoint {current_waypoint_index + 1}, holding {HOLD_TIME}s to collect data...")
                    is_holding = True
                    hold_start_time = current_time
                    hold_samples_collected = 0
                    collected_readings = []
                    state["mode"] = "HOLD"
                    continue

                if distance > 0.5:
                    dx = target["lat"] - state["gps"]["lat"]
                    dy = target["lng"] - state["gps"]["lon"]
                    dist_deg = math.sqrt(dx*dx + dy*dy)

                    if dist_deg > 0:
                        step = min(FLIGHT_SPEED, dist_deg)
                        state["gps"]["lat"] += dx / dist_deg * step
                        state["gps"]["lon"] += dy / dist_deg * step
                        state["gps"]["speed"] = random.uniform(1, MAX_SPEED_MS)
                        state["gps"]["heading"] = math.degrees(math.atan2(dy, dx))

                    target_alt = target.get("alt", 30)
                    alt_diff = target_alt - state["gps"]["alt"]
                    if abs(alt_diff) > 0.5:
                        state["gps"]["alt"] += alt_diff * FLIGHT_ALT_SPEED
                    else:
                        state["gps"]["alt"] = target_alt

            time.sleep(UPDATE_INTERVAL)

        except Exception as e:
            # Không để một lỗi nhỏ trong sensor/CSV làm chết thread mission.
            print(f"[Mission] ERROR in simulation loop: {e}")
            time.sleep(0.5)

# ==================== CSV LOGGING FUNCTIONS ====================
def init_csv_file():
    """Initialize CSV file with headers"""
    global csv_file_initialized
    with open(csv_filename, 'w', newline='', encoding='utf-8') as csvfile:
        fieldnames = ['timestep', 'timestamp', 'position', 'aqi', 'tvoc', 'eco2', 'temp', 'hum', 'pm25', 'co', 'no2']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
    csv_file_initialized = True
    print(f"[CSV] File initialized: {csv_filename}")

def append_to_csv(timestep, position, sensor_data):
    """Append one row of sensor data to CSV"""
    with open(csv_filename, 'a', newline='', encoding='utf-8') as csvfile:
        fieldnames = ['timestep', 'timestamp', 'position', 'aqi', 'tvoc', 'eco2', 'temp', 'hum', 'pm25', 'co', 'no2']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        
        _co  = sensor_data.get('co')
        _no2 = sensor_data.get('no2')
        row = {
            'timestep': timestep,
            'timestamp': datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            'position': position,
            'aqi': sensor_data.get('aqi', ''),
            'tvoc': sensor_data.get('tvoc', ''),
            'eco2': sensor_data.get('eco2', ''),
            'temp': sensor_data.get('temp', ''),
            'hum': sensor_data.get('hum', ''),
            'pm25': sensor_data.get('pm25', ''),
            'co':  round(_co, 2)  if _co  is not None else '',
            'no2': round(_no2, 1) if _no2 is not None else '',
        }
        writer.writerow(row)

def start_csv_logging(position_name, max_samples=30, filename="sensor_data.csv"):
    """Start logging sensor data to CSV"""
    global csv_logging_active, csv_samples_collected, csv_max_samples, csv_filename, csv_position
    csv_logging_active = True
    csv_samples_collected = 0
    csv_max_samples = max_samples
    csv_filename = filename
    csv_position = position_name
    init_csv_file()
    print(f"[CSV] Started logging {max_samples} samples to {filename} at position '{position_name}'")

def stop_csv_logging():
    """Stop logging sensor data to CSV"""
    global csv_logging_active
    csv_logging_active = False
    print(f"[CSV] Logging stopped. Collected {csv_samples_collected} samples.")

def get_csv_logging_status():
    """Get current CSV logging status"""
    return {
        "active": csv_logging_active,
        "collected": csv_samples_collected,
        "max_samples": csv_max_samples,
        "filename": csv_filename,
        "position": csv_position
    }

def ppm_to_ugm3(ppm, mw):
    return ppm * (mw * 1000) / 24.45

def ppb_to_ugm3(ppb, mw):
    return ppb * mw / 24.45

AQI_RANGES = [
    (0, 50),
    (51, 100),
    (101, 150),
    (151, 200),
    (201, 300),
    (301, 500),
]

VN_AQI_BREAKPOINTS = {
    # µg/m³, trung bình 24 giờ
    "pm25": [(0, 25), (25, 50), (50, 80), (80, 150), (150, 250), (250, 500)],
    # mg/m³
    "co":   [(0, 10), (10, 30), (30, 45), (45, 60), (60, 90), (90, 150)],
    # µg/m³
    "no2":  [(0, 100), (100, 200), (200, 700), (700, 1200), (1200, 2340), (2340, 3840)],
}

def _safe_float(value):
    try:
        if value is None:
            return None
        value = float(value)
        if math.isnan(value) or math.isinf(value) or value < 0:
            return None
        return value
    except (TypeError, ValueError):
        return None

def calculate_sub_aqi(concentration, breakpoints):
    concentration = _safe_float(concentration)
    if concentration is None:
        return None

    for (bp_lo, bp_hi), (i_lo, i_hi) in zip(breakpoints, AQI_RANGES):
        if bp_lo <= concentration <= bp_hi:
            if bp_hi == bp_lo:
                return int(round(i_hi))
            value = ((i_hi - i_lo) / (bp_hi - bp_lo)) * (concentration - bp_lo) + i_lo
            return int(round(value))

    # Vượt ngưỡng cao nhất thì giới hạn ở 500 để đúng thang hiển thị AQI.
    if concentration > breakpoints[-1][1]:
        return 500
    return 0

def calculate_vietnam_aqi(pm25=None, co=None, no2=None, eco2=None, tvoc=None, temp=None, hum=None):
    sub_indices = {}
    values = {
        "pm25": _safe_float(pm25),
        "co": _safe_float(co),
        "no2": _safe_float(no2),
    }

    for key, value in values.items():
        sub_aqi = calculate_sub_aqi(value, VN_AQI_BREAKPOINTS[key])
        if sub_aqi is not None:
            sub_indices[key] = sub_aqi

    if not sub_indices:
        return 0, None, sub_indices

    main_pollutant = max(sub_indices, key=sub_indices.get)
    return max(sub_indices.values()), main_pollutant, sub_indices

def get_aqi_category(aqi):
    aqi = int(round(_safe_float(aqi) or 0))
    if aqi <= 50:
        return "Tốt"
    if aqi <= 100:
        return "Trung bình"
    if aqi <= 150:
        return "Kém"
    if aqi <= 200:
        return "Xấu"
    if aqi <= 300:
        return "Rất xấu"
    return "Nguy hại"

def _build_current_sensor_data():
    """Lấy snapshot sensor hiện tại từ state để tính AQI/CSV."""
    return {
        'tvoc': safe_number(state['sensors'].get('tvoc'), 0.0),
        'eco2': safe_number(state['sensors'].get('eco2'), 0.0),
        'temp': safe_number(state['sensors'].get('temp'), 0.0),
        'hum': safe_number(state['sensors'].get('hum'), 0.0),
        'pm25': safe_number(state['sensors'].get('pm25'), 0.0),
        'co': safe_number(state['sensors'].get('co'), 0.0),
        'no2': safe_number(state['sensors'].get('no2'), 0.0),
    }


def _update_aqi_and_csv_from_state(prefix="[CAN-Sensors]"):
    """Tính AQI từ state hiện tại, in log, và ghi CSV nếu đang bật."""
    global csv_samples_collected, csv_logging_active

    sensor_data = _build_current_sensor_data()

    aqi_value, main_pollutant, aqi_components = calculate_vietnam_aqi(
        pm25=sensor_data['pm25'],
        co=sensor_data['co'],
        no2=sensor_data['no2'],
        eco2=sensor_data['eco2'],
        tvoc=sensor_data['tvoc'],
        temp=sensor_data['temp'],
        hum=sensor_data['hum'],
    )

    state["sensors"]["aqi"] = aqi_value
    state["sensors"]["aqi_category"] = get_aqi_category(aqi_value)
    state["sensors"]["aqi_main_pollutant"] = main_pollutant
    state["sensors"]["aqi_components"] = aqi_components

    print("=" * 40)
    print(f"{prefix} AQI Việt Nam : {aqi_value} ({state['sensors']['aqi_category']}) - chính: {main_pollutant or 'N/A'}")
    print(f"TVOC        : {sensor_data['tvoc']:.1f}")
    print(f"CO2/eCO2    : {sensor_data['eco2']:.1f}")
    print(f"Temperature : {sensor_data['temp']:.1f} °C")
    print(f"Humidity    : {sensor_data['hum']:.1f} %")
    print(f"PM2.5       : {sensor_data['pm25']:.2f} µg/m³")
    print(f"CO          : {sensor_data['co']:.2f} mg/m³")
    print(f"NO2         : {sensor_data['no2']:.1f} µg/m³")

    csv_sensor_data = {
        'aqi': aqi_value,
        'tvoc': sensor_data['tvoc'],
        'eco2': sensor_data['eco2'],
        'temp': sensor_data['temp'],
        'hum': sensor_data['hum'],
        'pm25': sensor_data['pm25'],
        'co': sensor_data['co'],
        'no2': sensor_data['no2'],
    }

    if csv_logging_active and csv_samples_collected < csv_max_samples:
        csv_samples_collected += 1
        append_to_csv(csv_samples_collected, csv_position, csv_sensor_data)
        print(f"[CSV] ✅ Saved sample {csv_samples_collected}/{csv_max_samples}")

        if csv_samples_collected >= csv_max_samples:
            stop_csv_logging()


def sensor_reader():
    """Thread cập nhật AQI/CSV.

    Khi USE_CAN_SENSOR_DATA=True:
      - Không đọc ENS160/AHT21 trên Pi nữa.
      - Dữ liệu sensor lấy từ can_reader_thread() do ESP32 gửi qua CAN.
      - Hàm này chỉ tính AQI, in log, và ghi CSV.

    Khi USE_CAN_SENSOR_DATA=False:
      - Giữ logic cũ: đọc ENS160 + AHT21 trên Pi, còn pm25/co/no2 lấy từ CAN.
    """
    global csv_samples_collected, csv_logging_active

    while True:
        if USE_CAN_SENSOR_DATA:
            if time.time() - last_can_update > CAN_TIMEOUT:
                state["battery"] = 0
                state["sensors"]["pm25"] = 0
                state["sensors"]["co"] = 0
                state["sensors"]["no2"] = 0
                state["sensors"]["eco2"] = 0
                state["sensors"]["tvoc"] = 0
                state["sensors"]["temp"] = 0
                state["sensors"]["hum"] = 0
                print("[CAN-Sensors] ⚠️ CAN timeout, reset sensor values")

            _update_aqi_and_csv_from_state(prefix="[CAN-Sensors]")
            time.sleep(2)
            continue

        # ==================== LEGACY MODE: đọc ENS160 + AHT21 trên Pi ====================
        if time.time() - last_can_update > CAN_TIMEOUT:
            state["battery"] = 0
            state["sensors"]["pm25"] = 0
            state["sensors"]["co"] = 0
            state["sensors"]["no2"] = 0

        if bus is not None:
            sensor_data = read_sensors()
            if sensor_data:
                sensor_data["tvoc"] = ppb_to_ugm3(sensor_data["tvoc"], 56.1)
                sensor_data["eco2"] = ppm_to_ugm3(sensor_data["eco2"], 44.01) / 1000

                co_val = state['sensors']['co']
                no2_val = state['sensors']['no2']
                pm25_val = state['sensors']['pm25']
                aqi_value, main_pollutant, aqi_components = calculate_vietnam_aqi(
                    pm25=pm25_val,
                    co=co_val,
                    no2=no2_val,
                    eco2=sensor_data["eco2"],
                    tvoc=sensor_data["tvoc"],
                    temp=sensor_data["temp"],
                    hum=sensor_data["hum"],
                )

                state["sensors"]["tvoc"] = sensor_data["tvoc"]
                state["sensors"]["eco2"] = sensor_data["eco2"]
                state["sensors"]["aqi"] = aqi_value
                state["sensors"]["aqi_category"] = get_aqi_category(aqi_value)
                state["sensors"]["aqi_main_pollutant"] = main_pollutant
                state["sensors"]["aqi_components"] = aqi_components
                state["sensors"]["temp"] = sensor_data["temp"]
                state["sensors"]["hum"] = sensor_data["hum"]

                print("=" * 40)
                print(f"AQI Việt Nam : {aqi_value} ({state['sensors']['aqi_category']}) - chính: {main_pollutant or 'N/A'}")
                print(f"TVOC        : {sensor_data['tvoc']} µg/m³")
                print(f"CO2         : {sensor_data['eco2']} µg/m³")
                print(f"Temperature : {sensor_data['temp']:.2f} °C")
                print(f"Humidity    : {sensor_data['hum']:.2f} %")
                print(f"PM2.5       : {state['sensors']['pm25']:.2f} µg/m³")
                print(f"CO          : {co_val:.2f} mg/m³" if co_val is not None else "CO          : N/A")
                print(f"NO2         : {no2_val:.1f} µg/m³" if no2_val is not None else "NO2         : N/A")

                csv_sensor_data = {
                    'aqi': aqi_value,
                    'tvoc': sensor_data['tvoc'],
                    'eco2': sensor_data['eco2'],
                    'temp': sensor_data['temp'],
                    'hum': sensor_data['hum'],
                    'pm25': state['sensors']['pm25'],
                    'co': state['sensors']['co'],
                    'no2': state['sensors']['no2'],
                }

                if csv_logging_active and csv_samples_collected < csv_max_samples:
                    csv_samples_collected += 1
                    append_to_csv(csv_samples_collected, csv_position, csv_sensor_data)
                    print(f"[CSV] ✅ Saved sample {csv_samples_collected}/{csv_max_samples}")

                    if csv_samples_collected >= csv_max_samples:
                        stop_csv_logging()
            else:
                _update_aqi_and_csv_from_state(prefix="[Sensors-Fallback]")
        else:
            _update_aqi_and_csv_from_state(prefix="[Sensors-Fallback]")

        time.sleep(2)  # Đọc mỗi 2 giây như code mẫu

# ==================== START THREADS ====================
# Start CAN thread
can_thread = threading.Thread(target=can_reader_thread, daemon=True)
can_thread.start()

# Start simulation thread
sim_thread = threading.Thread(target=simulation_loop, daemon=True)
sim_thread.start()

# Start sensor thread
sensor_thread = threading.Thread(target=sensor_reader, daemon=True)
sensor_thread.start()

# Timestamp update thread
def _timestamp_loop():
    while True:
        state["timestamp"] = time.strftime('%Y-%m-%d %H:%M:%S')
        time.sleep(1)

threading.Thread(target=_timestamp_loop, daemon=True).start()

# CSV logger for mission data
def csv_logger():
    os.makedirs('logs', exist_ok=True)
    fn = f"logs/flight_{time.strftime('%Y%m%d_%H%M')}.csv"
    with open(fn, 'w', newline='') as f:
        csv.writer(f).writerow([
            "Timestamp", "Lat", "Lon", "Alt(m)", "Speed(m/s)",
            "PM2.5", "eCO2", "TVOC", "Temp", "Hum", "AQI"
        ])
    while True:
        time.sleep(5)
        with open(fn, 'a', newline='') as f:
            g, s = state["gps"], state["sensors"]
            csv.writer(f).writerow([
                state["timestamp"],
                g["lat"], g["lon"], g["alt"], g["speed"],
                s["pm25"], s["eco2"], s["tvoc"], s["temp"], s["hum"], s["aqi"],
            ])

threading.Thread(target=csv_logger, daemon=True).start()

_camera = None
_camera_lock = threading.Lock()
_camera_initialized = False


def rotate_frame(frame, angle=180):
    """Xoay frame 180 độ"""
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
    print("[Camera] Initializing camera...")

    try:
        import subprocess
        subprocess.run(['systemctl', '--user', 'stop', 'pipewire', 'pipewire-pulse', 'wireplumber'], 
                       capture_output=True, timeout=5)
        subprocess.run(['systemctl', '--user', 'stop', 'pipewire.socket', 'pipewire-pulse.socket'], 
                       capture_output=True, timeout=5)
        subprocess.run(['pkill', '-f', 'libcamera'], capture_output=True)
        subprocess.run(['pkill', '-f', 'rpicam'], capture_output=True)
        time.sleep(2)
    except Exception as e:
        print(f"[Camera] Service stop error: {e}")
    
    try:
        from picamera2 import Picamera2
        cam = Picamera2()
        config = cam.create_preview_configuration(
            main={"size": (640, 480), "format": "RGB888"}
        )
        cam.configure(config)
        cam.start()
        time.sleep(1)
        
        test_frame = cam.capture_array()
        if test_frame is not None:
            _camera = cam
            print("[Camera] ✅ Picamera2 OK")
            return
        else:
            cam.stop()
    except Exception as e:
        print(f"[Camera Error] {e}")
    
    print("[Camera] ⚠️ Using Mock mode")
    _camera = None

print("[Camera] Initializing...")
init_camera()

def gen_frames():
    if _camera is None:
        while True:
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
            for i in range(480):
                frame[i, :] = [int(40 + i * 0.05), int(40 + i * 0.03), int(60 + i * 0.02)]
            frame = rotate_frame(frame, 180)
            
            g, s = state["gps"], state["sensors"]
            ts = time.strftime('%Y-%m-%d %H:%M:%S')

            cv2.rectangle(frame, (0, 0), (640, 50), (0, 0, 0), -1)
            cv2.putText(frame, "UAV GROUND CONTROL STATION", (180, 32),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            cv2.rectangle(frame, (20, 70), (620, 200), (0, 0, 0), -1)
            cv2.rectangle(frame, (20, 70), (620, 200), (100, 100, 100), 1)

            cv2.putText(frame, f"LAT: {g['lat']:.6f}   LON: {g['lon']:.6f}", (40, 95),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
            cv2.putText(frame, f"ALT: {g['alt']:.1f}m   SPD: {g['speed']:.1f}m/s   HDG: {g['heading']:.0f}deg", 
                        (40, 125), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
            cv2.putText(frame, f"TEMP: {s['temp']:.1f}°C   HUM: {s['hum']:.1f}%   AQI: {s['aqi']}", 
                        (40, 155), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
            cv2.putText(frame, f"eCO2: {s['eco2']:.0f}ppm   TVOC: {s['tvoc']:.0f}ppb", 
                        (40, 185), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
            
            if is_flying:
                status = f"FLYING to WP {current_waypoint_index + 1}/{len(current_waypoints)}"
                cv2.putText(frame, status, (40, 215), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
            elif is_holding:
                status = f"HOLDING at WP {current_waypoint_index + 1} - Data: {hold_samples_collected}/{SAMPLE_COUNT}"
                cv2.putText(frame, status, (40, 215), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 2)
            
            cv2.rectangle(frame, (0, 440), (640, 480), (0, 0, 0), -1)
            cv2.putText(frame, f"UAV GCS - {ts}", (20, 465),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1)
            
            _, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buf.tobytes() + b'\r\n'
            time.sleep(0.033)
        return
    
    while True:
        try:
            with _camera_lock:
                frame = _camera.capture_array()
            
            frame = rotate_frame(frame, 180)
            
            g, s = state["gps"], state["sensors"]
            
            _, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buf.tobytes() + b'\r\n'
        except Exception as e:
            print(f"[Frame Error] {e}")
            time.sleep(0.1)


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
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route("/api/uav-state")
def uav_state():
    return jsonify({"success": True, "state": state})

@app.route("/vehicle-position")
def vehicle_position():
    return jsonify({
        "success": True,
        "lat": state["gps"]["lat"],
        "lon": state["gps"]["lon"],
        "alt": state["gps"]["alt"]
    })

@app.route("/vehicle-info")
def vehicle_info():
    return jsonify({
        "success": True,
        "alt": round(state["gps"]["alt"], 1),
        "speed": round(state["gps"]["speed"], 2),
        "heading": round(state["gps"]["heading"], 1),
        "battery": round(state["battery"], 1),
        "mode": "AUTO" if is_flying else ("HOLD" if is_holding else "GUIDED"),
        "armed": is_flying or is_holding
    })

@app.route("/api/telemetry")
def api_telemetry():
    s = state["sensors"]
    return jsonify({
        "success": True,
        "telemetry": {
            "pm25": s["pm25"],
            "pm10": round(s["pm25"] * 1.5, 2),
            "eco2": s["eco2"],
            "co2": s["eco2"],
            "tvoc": s["tvoc"],
            "temp": s["temp"],
            "hum": s["hum"],
            "aqi": s["aqi"],
            "aqi_category": s.get("aqi_category", "--"),
            "aqi_main_pollutant": s.get("aqi_main_pollutant"),
            "aqi_components": s.get("aqi_components", {}),
            "co": s.get("co", 0),
            "no2": s.get("no2", 0),
            "alt": state["gps"]["alt"],
            "speed": state["gps"]["speed"],
            "heading": state["gps"]["heading"],
            "armed": is_flying or is_holding,
            "battery": round(state["battery"], 1),
            "mode": "AUTO" if is_flying else ("HOLD" if is_holding else "GUIDED"),
            "is_holding": is_holding,
            "hold_samples": hold_samples_collected if is_holding else 0,
            "hold_total": SAMPLE_COUNT if is_holding else 0,
            "current_wp": current_waypoint_index + 1 if current_waypoint_index < len(current_waypoints) else len(current_waypoints),
            "total_wp": len(current_waypoints)
        }
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
    global current_waypoints, current_waypoint_index, is_flying, is_holding
    data = request.get_json()
    if not data or "mission" not in data:
        return jsonify({"success": False, "message": "Missing mission data"}), 400
    
    current_waypoints = data["mission"]
    current_waypoint_index = 0
    is_flying = False
    is_holding = False
    
    print(f"[Mission] Uploaded {len(current_waypoints)} waypoints")
    return jsonify({"success": True, "message": f"Uploaded {len(current_waypoints)} waypoints"})

@app.route("/start-mission", methods=["POST"])
def start_mission():
    global is_flying, current_waypoint_index, is_holding
    if not current_waypoints:
        return jsonify({"success": False, "message": "No waypoints uploaded"}), 400
    
    is_flying = True
    is_holding = False
    current_waypoint_index = 0
    # KHÔNG reset wp_csv_files ở đây nữa
    state["mode"] = "AUTO"
    
    print("[Mission] 🚀 Mission started")
    return jsonify({"success": True, "message": "Mission started"})

@app.route("/stop-mission", methods=["POST"])
def stop_mission():
    global is_flying, is_holding
    is_flying = False
    is_holding = False
    state["mode"] = "GUIDED"
    print("[Mission] ⏸️ Mission stopped")
    return jsonify({"success": True, "message": "Mission stopped"})

@app.route("/clear-mission", methods=["POST"])
def clear_mission():
    global current_waypoints, current_waypoint_index, is_flying, is_holding
    current_waypoints = []
    current_waypoint_index = 0
    is_flying = False
    is_holding = False
    return jsonify({"success": True, "message": "Mission cleared"})

@app.route("/get-mission")
def get_mission():
    return jsonify({"success": True, "mission": current_waypoints})

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

@app.route("/export-csv")
def export_csv():
    if not collected_data_history:
        return jsonify({"success": False, "message": "No data to export"})
    
    import io
    output = io.StringIO()
    writer = csv.writer(output)
    writer.writerow(["Time", "Lat", "Lon", "Alt", "Waypoint", "PM2.5", "eCO2", "TVOC", "Temp", "Hum", "AQI", "CO", "NO2", "SampleCount"])
    
    for data in collected_data_history:
        writer.writerow([
            data["time"], data["lat"], data["lon"], data["alt"],
            data["waypoint_index"] + 1, data["pm25"], data["eco2"],
            data["tvoc"], data["temp"], data["hum"], data["aqi"],
            data.get("co", ""), data.get("no2", ""),
            data["sample_count"]
        ])
    
    return Response(output.getvalue(), mimetype="text/csv",
                   headers={"Content-Disposition": "attachment;filename=mission_data.csv"})

@app.route("/export-json")
def export_json():
    return jsonify({"success": True, "data": collected_data_history})

@app.route("/arm", methods=['POST'])
def arm():
    global is_flying
    if current_waypoints and not is_flying:
        is_flying = True
    state["armed"] = True
    return jsonify({"success": True, "message": "Armed"})

@app.route("/disarm", methods=['POST'])
def disarm():
    global is_flying, is_holding
    is_flying = False
    is_holding = False
    state["armed"] = False
    return jsonify({"success": True, "message": "Disarmed"})

@app.route("/rtl", methods=['POST'])
def rtl():
    global is_flying, is_holding
    is_flying = True
    is_holding = False
    return jsonify({"success": True, "message": "Return to Launch"})

@app.route("/set-mode", methods=['POST'])
def set_mode():
    data = request.get_json()
    state["mode"] = data.get("mode", "GUIDED")
    return jsonify({"success": True, "mode": state["mode"]})

@app.route("/camera/snapshot", methods=["POST"])
def camera_snapshot():
    try:
        os.makedirs("camera_captures", exist_ok=True)
        filename = f"snapshot_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg"
        filepath = os.path.join("camera_captures", filename)
        
        if _camera:
            with _camera_lock:
                frame = _camera.capture_array()
            # Xoay ảnh 180 độ
            frame = rotate_frame(frame, 180)
            cv2.imwrite(filepath, frame)
        else:
            blank = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(blank, f"SNAPSHOT {datetime.now()}", (50, 240),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imwrite(filepath, blank)
        
        return jsonify({"success": True, "filename": filename, "path": filepath})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)}), 500

@app.route("/camera_captures/<filename>")
def serve_capture(filename):
    from flask import send_from_directory
    return send_from_directory(os.path.abspath("camera_captures"), filename)

# ── RECORDING STATE ────────────────────────────────────────────────────────────
_recorder = None
_recorder_lock = threading.Lock()
_recording_filename = None

@app.route("/camera/record/start", methods=["POST"])
def camera_record_start():
    global _recorder, _recording_filename
    with _recorder_lock:
        if _recorder is not None:
            return jsonify({"success": False, "error": "Already recording"}), 400
        os.makedirs("camera_captures", exist_ok=True)
        _recording_filename = f"video_{datetime.now().strftime('%Y%m%d_%H%M%S')}.mp4"
        filepath = os.path.join("camera_captures", _recording_filename)
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        _recorder = cv2.VideoWriter(filepath, fourcc, 20.0, (640, 480))
        if not _recorder.isOpened():
            _recorder = None
            return jsonify({"success": False, "error": "Failed to open VideoWriter"}), 500

    def record_loop():
        global _recorder
        while True:
            with _recorder_lock:
                if _recorder is None:
                    break
            if _camera:
                with _camera_lock:
                    frame = _camera.capture_array()
                # Xoay frame 180 độ khi ghi hình
                bgr = rotate_frame(frame, 180)
            else:
                bgr = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(bgr, f"REC {datetime.now().strftime('%H:%M:%S')}", (20, 240),
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
            return jsonify({"success": False, "error": "Not recording"}), 400
        _recorder.release()
        _recorder = None
        filename = _recording_filename
        _recording_filename = None
    return jsonify({"success": True, "filename": filename})

@app.route("/camera_captures/video/<filename>")
def serve_video(filename):
    from flask import send_from_directory
    return send_from_directory(os.path.abspath("camera_captures"), filename)
def wp_data_list():
    return jsonify({"success": True, "files": wp_csv_files, "total": len(wp_csv_files)})

@app.route("/wp-data/list")
def wp_data_list():
    """API để lấy danh sách các file CSV waypoint"""
    return jsonify({"success": True, "files": wp_csv_files, "total": len(wp_csv_files)})

@app.route("/wp-data/download/<filename>")
def wp_data_download(filename):
    from flask import send_file
    safe = os.path.basename(filename)
    filepath = os.path.join("wp_data", safe)
    if not os.path.exists(filepath):
        return jsonify({"success": False, "message": "File not found"}), 404
    return send_file(filepath, mimetype="text/csv",
                     as_attachment=True, download_name=safe)

@app.route("/wp-data/download-all")
def wp_data_download_all():
    import io, zipfile
    from flask import send_file
    if not wp_csv_files:
        return jsonify({"success": False, "message": "No data yet"}), 404
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as zf:
        for entry in wp_csv_files:
            fp = os.path.join("wp_data", entry["filename"])
            if os.path.exists(fp):
                zf.write(fp, entry["filename"])
    buf.seek(0)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    return send_file(buf, mimetype="application/zip",
                     as_attachment=True, download_name=f"mission_data_{ts}.zip")

@app.route("/wp-data/clear", methods=["POST"])
def wp_data_clear():
    import shutil
    global wp_csv_files
    if os.path.exists("wp_data"):
        shutil.rmtree("wp_data")
        os.makedirs("wp_data", exist_ok=True)
    wp_csv_files = []
    return jsonify({"success": True})

@app.route("/test-sensors")
def test_sensors():
    """Test endpoint để kiểm tra cảm biến"""
    if bus is None:
        return jsonify({"success": False, "message": "I2C bus not available"})
    
    sensor_data = read_sensors()
    if sensor_data:
        return jsonify({"success": True, "data": sensor_data})
    else:
        return jsonify({"success": False, "message": "Failed to read sensors"})


@app.route("/start-csv-logging", methods=["POST"])
def start_csv_logging_route():
    """Start logging sensor data to CSV"""
    data = request.get_json()
    position = data.get("position", "unknown")
    max_samples = data.get("max_samples", 30)
    filename = data.get("filename", f"sensor_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
    
    # Reset CSV logging state
    global csv_logging_active, csv_samples_collected, csv_max_samples, csv_filename, csv_position, csv_file_initialized
    csv_logging_active = True
    csv_samples_collected = 0
    csv_max_samples = max_samples
    csv_filename = filename
    csv_position = position
    csv_file_initialized = False  # Will be recreated on first write
    
    # Initialize the CSV file with headers
    init_csv_file()
    
    print(f"[CSV] Started logging {max_samples} samples to {filename} at position '{position}'")
    return jsonify({
        "success": True, 
        "message": f"Started logging {max_samples} samples to {filename}",
        "status": get_csv_logging_status()
    })

@app.route("/stop-csv-logging", methods=["POST"])
def stop_csv_logging_route():
    """Stop logging sensor data to CSV"""
    stop_csv_logging()
    return jsonify({
        "success": True, 
        "message": "CSV logging stopped",
        "status": get_csv_logging_status()
    })

@app.route("/csv-logging-status")
def csv_logging_status_route():
    """Get current CSV logging status"""
    return jsonify({
        "success": True,
        "status": get_csv_logging_status()
    })

@app.route("/download-csv")
def download_csv():
    """Download the CSV file - fixed to handle missing file"""
    if not os.path.exists(csv_filename):
        # Try to find the most recent CSV file in current directory
        csv_files = [f for f in os.listdir('.') if f.startswith('sensor_data_') and f.endswith('.csv')]
        if csv_files:
            latest_csv = max(csv_files, key=os.path.getctime)
            csv_path = latest_csv
        else:
            return jsonify({"success": False, "message": "No CSV file found"}), 404
    else:
        csv_path = csv_filename
    
    try:
        with open(csv_path, 'r', encoding='utf-8') as f:
            csv_content = f.read()
        
        return Response(
            csv_content,
            mimetype="text/csv",
            headers={"Content-Disposition": f"attachment;filename={os.path.basename(csv_path)}"}
        )
    except Exception as e:
        return jsonify({"success": False, "message": f"Error reading file: {str(e)}"}), 500

# ==================== MAIN ====================
if __name__ == "__main__":
    print("=" * 60)
    print("  🚁 UAV GROUND CONTROL STATION - REAL SENSORS")
    print("  📍 http://localhost:5000")
    print("  📡 Sensors: " + ("✅ ENS160 + AHT21" if bus else "⚠️ Fallback mode"))
    print("  📸 Camera: " + ("✅ Real camera" if _camera else "⚠️ Mock mode"))
    print("  🗺️  Map: Click to add waypoints")
    print("  ✈️  Mission: Waypoint navigation active")
    print("  📊 CSV Logging: Use /start-csv-logging endpoint")
    print("=" * 60)
    app.run(host="0.0.0.0", port=8000, threaded=True)
