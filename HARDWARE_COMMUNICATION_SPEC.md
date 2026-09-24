# BÁO CÁO ĐẶC TẢ GIAO TIẾP PHẦN CỨNG: UART & CAN BUS
## DỰ ÁN: TRẠM ĐIỀU KHIỂN & QUAN TRẮC MÔI TRƯỜNG KHÔNG KHÍ UAV (UAV AIR QUALITY GCS)
**Phiên bản:** 2.1 | **Ngày phát hành:** 2026-09-24  
**Thiết bị trung tâm (Companion Computer):** Raspberry Pi 5 (chạy hệ thống `real_server.py`)  
**Thiết bị ngoại vi (Hardware/Firmware bên UAV):** Bo đo môi trường (Sensor Node MCU) & Bo điều khiển bay (Flight Controller - FC)

---

## 1. TỔNG QUAN KIẾN TRÚC KẾT NỐI

Hệ thống trạm mặt đất GCS kết nối với bo nhúng máy bay UAV thông qua 2 kênh vật lý độc lập nhằm đảm bảo độ tin cậy cao, cách ly tín hiệu điều khiển bay và tín hiệu quan trắc:

```
[ CẢM BIẾN MÔI TRƯỜNG ]
(Nhiệt ẩm, Khí, Bụi)
         │
         ▼
┌──────────────────┐               CAN Bus (500 kbps)               ┌─────────────────────────────────┐
│ Bo Vi điều khiển │ ─────────────────────────────────────────────► │        Raspberry Pi 5           │
│   Sensor Node    │    Frame 0x555: Nhiệt độ, Độ ẩm, TVOC, eCO2   │   (Chạy real_server.py / Linux) │
└──────────────────┘    Frame 0x556: CO, NO2, PM2.5, AQI            │                                 │
                                                                    │  • CAN Reader: SocketCAN can0   │
┌──────────────────┐          UART Serial (115200 baud)             │  • UART Reader: /dev/serial0    │
│ Flight Controller│ ◄════════════════════════════════════════════► │  • Web GCS Server: Flask / WS  │
│(Pixhawk / Cube / │   TX (UAV->Pi): Chuỗi JSON Telemetry (2-5Hz)   │                                 │
│  STM32 / ESP32)  │   RX (Pi->UAV): Chuỗi JSON Lệnh điều khiển     └─────────────────────────────────┘
└──────────────────┘
```

---

## 2. GIAO TIẾP CAN BUS (TRUYỀN DỮ LIỆU CẢM BIẾN MÔI TRƯỜNG)

### 2.1. Thông số vật lý & Cấu hình phần cứng
- **Chuẩn CAN:** CAN 2.0A (Standard Identifier 11-bit).
- **Tốc độ truyền (Bitrate):** **500 kbps** (500,000 bps).
- **Trở phối hợp đầu cuối:** Bắt buộc có trở **120 Ω** tại 2 đầu của đường bus CAN (CAN_H và CAN_L).
- **Phía Raspberry Pi 5:** Sử dụng module SPI CAN **MCP2515 + TJA1050** (hoặc Waveshare 2-CH CAN HAT) giao tiếp qua SPI0.
- **Sơ đồ đấu dây Raspberry Pi 5 với module MCP2515:**

| Chân MCP2515 | Tên chân Raspberry Pi 5 | Vị trí Pin Header RPi | Ghi chú |
| :--- | :--- | :--- | :--- |
| **VCC** | 3.3V hoặc 5V | Pin 1 (3.3V) hoặc Pin 2 (5V) | TJA1050 dùng 5V; nếu module chạy 5V cần level shift chân SO về 3.3V |
| **GND** | Ground | Pin 6, 9, 14, 20 hoặc 25 | Nối đất chung |
| **CS** | GPIO 8 (SPI0_CE0_N) | Pin 24 | Chip Select |
| **SO (MISO)**| GPIO 9 (SPI0_MISO) | Pin 21 | Master In Slave Out |
| **SI (MOSI)**| GPIO 10 (SPI0_MOSI) | Pin 19 | Master Out Slave In |
| **SCK** | GPIO 11 (SPI0_SCLK) | Pin 23 | Xung nhịp SPI |
| **INT** | GPIO 25 | Pin 22 | Ngắt thu dữ liệu |

- **Cấu hình trên Raspberry Pi OS (`/boot/firmware/config.txt`):**
  ```ini
  dtparam=spi=on
  dtoverlay=mcp2515-can0,oscillator=8000000,interrupt=25
  # Nếu thạch anh trên module là 16MHz thì sửa oscillator=16000000
  ```
- **Lệnh kích hoạt interface CAN trên Pi:**
  ```bash
  sudo ip link set can0 up type can bitrate 500000
  ```

---

### 2.2. Định dạng đóng gói dữ liệu CAN (CAN Protocol Specification)
Hệ thống sử dụng **2 CAN Frames** có ID cố định. Tất cả các số nguyên 2-byte (`int16_t`, `uint16_t`) đều được đóng gói theo định dạng **Big-Endian** (Byte trọng số cao đi trước - MSB First).

#### Frame 1: Dữ liệu Khí hậu & Môi trường cơ bản
- **CAN Arbitration ID:** `0x555` (CAN_ID_ENV)
- **Độ dài dữ liệu (DLC):** 8 Bytes
- **Cấu trúc byte:**

| Byte Offset | Tên trường | Kiểu dữ liệu | Hệ số nhân | Đơn vị | Dải giá trị | Công thức giải mã tại Pi |
| :---: | :--- | :---: | :---: | :---: | :---: | :--- |
| **Byte 0 - 1** | `tempData` | `int16_t` (có dấu, Big-Endian) | $\times 100$ | °C | -40.00 đến +85.00 | $T = \text{int16}(\text{data}[0..1]) / 100.0$ |
| **Byte 2 - 3** | `humData` | `uint16_t` (không dấu, Big-Endian)| $\times 100$ | % | 0.00 đến 100.00 | $H = \text{uint16}(\text{data}[2..3]) / 100.0$ |
| **Byte 4 - 5** | `tvoc` | `uint16_t` (không dấu, Big-Endian)| $1$ | ppb | 0 đến 65,535 | $TVOC = \text{uint16}(\text{data}[4..5])$ |
| **Byte 6 - 7** | `eco2` | `uint16_t` (không dấu, Big-Endian)| $1$ | ppm | 400 đến 65,000 | $eCO_2 = \text{uint16}(\text{data}[6..7])$ |

*Ví dụ gói tin:*  
`Nhiệt độ = 28.45°C` (2845 = `0x0B1D`), `Độ ẩm = 65.50%` (6550 = `0x1996`), `TVOC = 45 ppb` (`0x002D`), `eCO2 = 480 ppm` (`0x01E0`)  
$\to$ Byte mảng: `[0x0B, 0x1D, 0x19, 0x96, 0x00, 0x2D, 0x01, 0xE0]`

---

#### Frame 2: Dữ liệu Khí độc hại, Bụi mịn & Chỉ số AQI
- **CAN Arbitration ID:** `0x556` (CAN_ID_GAS)
- **Độ dài dữ liệu (DLC):** 8 Bytes (tối thiểu 7 bytes)
- **Cấu trúc byte:**

| Byte Offset | Tên trường | Kiểu dữ liệu | Hệ số nhân | Đơn vị | Dải giá trị | Công thức giải mã tại Pi |
| :---: | :--- | :---: | :---: | :---: | :---: | :--- |
| **Byte 0 - 1** | `coData` | `uint16_t` (không dấu, Big-Endian)| $\times 10$ | mg/m³ | 0.0 đến 6500.0 | $CO = \text{uint16}(\text{data}[0..1]) / 10.0$ |
| **Byte 2 - 3** | `no2Data` | `uint16_t` (không dấu, Big-Endian)| $\times 10$ | µg/m³ | 0.0 đến 6500.0 | $NO_2 = \text{uint16}(\text{data}[2..3]) / 10.0$ |
| **Byte 4 - 5** | `pmData` | `uint16_t` (không dấu, Big-Endian)| $\times 100$| µg/m³ | 0.00 đến 650.00 | $PM_{2.5} = \text{uint16}(\text{data}[4..5]) / 100.0$ |
| **Byte 6** | `firmware_aqi` | `uint8_t` (không dấu) | $1$ | Điểm | 1 đến 255 | $AQI = \text{uint8}(\text{data}[6])$ |
| **Byte 7** | `reserved` | `uint8_t` (không dấu) | - | - | 0 | Đặt mặc định `0x00` |

*Ví dụ gói tin:*  
`CO = 3.2 mg/m³` (32 = `0x0020`), `NO2 = 45.6 µg/m³` (456 = `0x01C8`), `PM2.5 = 35.80 µg/m³` (3580 = `0x0DFC`), `AQI = 68` (`0x44`), `Reserved = 0` (`0x00`)  
$\to$ Byte mảng: `[0x00, 0x20, 0x01, 0xC8, 0x0D, 0xFC, 0x44, 0x00]`

---

### 2.3. Tần suất gửi khuyến nghị
- Cặp 2 Frame (`0x555` và `0x556`) nên được gửi liên tiếp cách nhau **5ms - 10ms**.
- Chu kỳ gửi lặp lại: **1 Hz đến 5 Hz** (khuyến nghị chuẩn: **2 Hz** - tức 500ms phát một lần).

---

### 2.4. Code mẫu C/C++ cho bo phần cứng Sensor (Arduino / STM32)

```cpp
#include <SPI.h>
#include <mcp_can.h>

const int SPI_CS_PIN = 10;
MCP_CAN CAN0(SPI_CS_PIN);

void setup() {
    Serial.begin(115200);
    // Khởi tạo MCP2515 với thạch anh 8MHz và tốc độ 500kbps
    if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
        Serial.println("CAN Init OK!");
        CAN0.setMode(MCP_NORMAL);
    } else {
        Serial.println("CAN Init Failed!");
    }
}

void sendSensorDataToPi(float temp, float hum, uint16_t tvoc, uint16_t eco2,
                        float co, float no2, float pm25, uint8_t aqi) {
    // 1. Đóng gói Frame 1 (0x555) - Big Endian
    byte buf1[8];
    int16_t t_raw = (int16_t)round(temp * 100.0f);
    uint16_t h_raw = (uint16_t)round(hum * 100.0f);
    
    buf1[0] = (byte)((t_raw >> 8) & 0xFF);
    buf1[1] = (byte)(t_raw & 0xFF);
    buf1[2] = (byte)((h_raw >> 8) & 0xFF);
    buf1[3] = (byte)(h_raw & 0xFF);
    buf1[4] = (byte)((tvoc >> 8) & 0xFF);
    buf1[5] = (byte)(tvoc & 0xFF);
    buf1[6] = (byte)((eco2 >> 8) & 0xFF);
    buf1[7] = (byte)(eco2 & 0xFF);

    CAN0.sendMsgBuf(0x555, 0, 8, buf1);
    delay(5); // Giãn cách ngắn giữa 2 frame

    // 2. Đóng gói Frame 2 (0x556) - Big Endian
    byte buf2[8];
    uint16_t co_raw = (uint16_t)round(co * 10.0f);
    uint16_t no2_raw = (uint16_t)round(no2 * 10.0f);
    uint16_t pm_raw = (uint16_t)round(pm25 * 100.0f);

    buf2[0] = (byte)((co_raw >> 8) & 0xFF);
    buf2[1] = (byte)(co_raw & 0xFF);
    buf2[2] = (byte)((no2_raw >> 8) & 0xFF);
    buf2[3] = (byte)(no2_raw & 0xFF);
    buf2[4] = (byte)((pm_raw >> 8) & 0xFF);
    buf2[5] = (byte)(pm_raw & 0xFF);
    buf2[6] = aqi;
    buf2[7] = 0x00; // Reserved

    CAN0.sendMsgBuf(0x556, 0, 8, buf2);
}
```

---

## 3. GIAO TIẾP UART SERIAL (TELEMETRY & FLIGHT CONTROL)

### 3.1. Thông số vật lý & Cấu hình phần cứng
- **Cổng giao tiếp trên Raspberry Pi 5:** `/dev/serial0` (hoặc `/dev/ttyAMA0`).
- **Tốc độ truyền (Baudrate):** **115200 bps**.
- **Cấu hình khung truyền:** **8-N-1** (8 Data bits, No parity, 1 Stop bit).
- **Mức điện áp Logic:** **3.3V TTL**.  
  ⚠️ **CẢNH BÁO:** Chân GPIO Raspberry Pi chỉ chịu mức điện áp 3.3V. Nếu Flight Controller dùng chuẩn 5V TTL hoặc RS232, **bắt buộc phải qua mạch chuyển đổi mức (Logic Level Shifter)** để không làm hỏng vi xử lý Pi 5!
- **Sơ đồ đấu nối giữa Flight Controller và Raspberry Pi 5:**

| Chân Flight Controller (TELEM UART) | Chân Raspberry Pi 5 Header | Vị trí Pin Header RPi | Ghi chú |
| :--- | :--- | :--- | :--- |
| **FC TX** (Truyền Telemetry) | **GPIO 15 (UART_RXD0)** | **Pin 10** | Đấu chéo (TX FC $\to$ RX Pi) |
| **FC RX** (Nhận Lệnh điều khiển) | **GPIO 14 (UART_TXD0)** | **Pin 8** | Đấu chéo (RX FC $\leftarrow$ TX Pi) |
| **GND** | **Ground** | **Pin 6, 9 hoặc 14** | **Nối đất chung bắt buộc** |

---

### 3.2. Cấu trúc khung truyền (Data Framing)
- Sử dụng giao thức văn bản **Newline-Delimited JSON (NDJSON)**:
  - Mỗi gói tin là một chuỗi JSON hợp lệ kết thúc chính xác bằng ký tự xuống dòng `\n` (ASCII `0x0A`).
  - Mã hóa ký tự: **UTF-8**.

---

### 3.3. Chiều FC $\to$ Raspberry Pi: Viễn trắc bay (Telemetry Stream)
Bo điều khiển bay (FC) định kỳ truyền gói tin viễn trắc lên Raspberry Pi để server hiển thị trên HUD Cockpit, bản đồ số và bảng thông số.

- **Tần suất truyền:** **2 Hz đến 5 Hz** (200ms - 500ms mỗi gói).
- **Cú pháp JSON:**
```json
{
  "telemetry": {
    "lat": 16.0743537,
    "lon": 108.1522514,
    "alt": 35.0,
    "speed": 4.2,
    "heading": 85.0,
    "satellites": 18,
    "pitch": 2.1,
    "roll": -1.2,
    "yaw": 85.0,
    "battery": 92.5,
    "voltage": 16.2,
    "armed": true,
    "mode": "AUTO",
    "flight_state": "RUNNING",
    "current_wp": 1
  }
}
```

- **Mô tả chi tiết các trường Telemetry:**

| Tên trường | Kiểu dữ liệu | Đơn vị | Ý nghĩa & Dải giá trị |
| :--- | :---: | :---: | :--- |
| `lat` | Float | Độ (deg) | Vĩ độ GPS (Ví dụ: `16.0743537`) |
| `lon` (hoặc `lng`) | Float | Độ (deg) | Kinh độ GPS (Ví dụ: `108.1522514`) |
| `alt` | Float | Mét (m) | Độ cao tương đối so với điểm cất cánh (AGL) |
| `speed` | Float | m/s | Vận tốc di chuyển mặt đất (Ground speed) |
| `heading` | Float | Độ (deg) | Hướng mũi bay theo la bàn từ trường (0.0° - 359.9°) |
| `satellites` | Integer | Số lượng | Số lượng vệ tinh GPS khóa được (khuyến nghị $\ge 12$) |
| `pitch` | Float | Độ (deg) | Góc ngửa/chúi trục Pitch (-90° đến +90°) |
| `roll` | Float | Độ (deg) | Góc nghiêng cánh trục Roll (-180° đến +180°) |
| `yaw` | Float | Độ (deg) | Góc xoay thân trục Yaw (0° đến 360°) |
| `battery` | Float | % | Phần trăm dung lượng pin còn lại (0.0 - 100.0%) |
| `voltage` | Float | V | Điện áp pack pin hiện tại (Ví dụ: `16.2` V cho pin 4S) |
| `armed` | Boolean | `true`/`false`| Trạng thái mở khóa động cơ (`true` = Đang quay/Sẵn sàng bay) |
| `mode` | String | - | Chế độ bay: `"AUTO"`, `"GUIDED"`, `"LOITER"`, `"RTL"`, `"MANUAL"` |
| `flight_state` | String | - | Trạng thái nhiệm vụ: `"IDLE"`, `"TAKEOFF"`, `"RUNNING"`, `"HOLD"`, `"LANDING"` |
| `current_wp` | Integer | Số thứ tự | Chỉ số waypoint UAV đang bay tới (bắt đầu từ 1 đến N) |

---

### 3.4. Chiều Raspberry Pi $\to$ FC: Lệnh điều khiển & Nhiệm vụ bay (Commands)
Raspberry Pi sẽ gửi lệnh khi người vận hành thao tác trên giao diện Cockpit GCS. FC cần lắng nghe trên cổng UART và thực thi:

#### 1. Lệnh Cất cánh (Takeoff)
- **Kích hoạt khi:** Bấm nút **"Cất Cánh"** trên bảng điều khiển.
- **Payload JSON:**
  ```json
  {"cmd": "takeoff", "alt": 35.0}
  ```
- **Hành động FC:** Chuyển sang chế độ GUIDED/TAKEOFF, Arm động cơ và nâng độ cao lên 35.0 mét, sau đó giữ vị trí (Hold).

#### 2. Lệnh Nạp danh sách Waypoint (Upload Mission)
- **Kích hoạt khi:** Bấm nút **"Nạp Waypoint"** trên giao diện lập nhiệm vụ.
- **Quy tắc thiết kế hệ thống:** Chỉ gửi danh sách tọa độ phẳng `lat`, `lng`. Độ cao bay được giữ cố định theo tiêu chuẩn 35m của trạm.
- **Payload JSON:**
  ```json
  {
    "cmd": "upload_mission",
    "count": 3,
    "waypoints": [
      {"lat": 16.0743537, "lng": 108.1522514},
      {"lat": 16.0751200, "lng": 108.1534000},
      {"lat": 16.0763000, "lng": 108.1542000}
    ]
  }
  ```
- **Hành động FC:** Xóa danh sách waypoint cũ, nạp danh sách waypoint mới vào bộ nhớ đệm chuyến bay, sẵn sàng cất cánh hoặc bay tiếp.

#### 3. Lệnh Bắt đầu bay nhiệm vụ (Start Mission)
- **Kích hoạt khi:** Bấm nút **"Bắt Đầu Nhiệm Vụ"**.
- **Payload JSON:**
  ```json
  {"cmd": "start_mission"}
  ```
- **Hành động FC:** Chuyển sang chế độ `AUTO` và bắt đầu dẫn đường từ Waypoint 1.

#### 4. Lệnh Tạm dừng nhiệm vụ (Stop Mission)
- **Kích hoạt khi:** Bấm nút **"Tạm Dừng"** hoặc dừng khẩn cấp.
- **Payload JSON:**
  ```json
  {"cmd": "stop_mission"}
  ```
- **Hành động FC:** Chuyển chế độ sang `LOITER` / `HOLD` để giữ nguyên vị trí và độ cao hiện tại trên không trung.

#### 5. Lệnh Xóa toàn bộ Waypoint (Clear Mission)
- **Kích hoạt khi:** Bấm nút **"Xóa Điểm"** trên GCS.
- **Payload JSON:**
  ```json
  {"cmd": "clear_mission"}
  ```

#### 6. Lệnh Mở khóa / Khóa động cơ (Arm / Disarm)
- **Arm:** `{"cmd": "arm"}\n`
- **Disarm:** `{"cmd": "disarm"}\n`

#### 7. Lệnh Quay về điểm xuất phát (RTL - Return To Launch)
- **Kích hoạt khi:** Bấm nút **"RTL"**.
- **Payload JSON:**
  ```json
  {"cmd": "rtl"}
  ```
- **Hành động FC:** Nâng lên độ cao an toàn, bay thẳng về tọa độ Home và tự động hạ cánh (`LANDING`).

#### 8. Lệnh Chuyển chế độ bay (Set Mode)
- **Payload JSON:**
  ```json
  {"cmd": "set_mode", "mode": "AUTO"}
  ```

---

### 3.5. Code mẫu C/C++ phía FC (Nhận lệnh & Truyền Telemetry)

```cpp
#include <ArduinoJson.h>

// Hàm gửi Telemetry lên Pi định kỳ 200ms (5Hz)
void sendTelemetryToPi(float lat, float lon, float alt, float speed, float heading,
                       float pitch, float roll, float yaw, float battery, float voltage,
                       bool armed, const char* mode, const char* flight_state, int current_wp) {
    StaticJsonDocument<512> doc;
    JsonObject tel = doc.createNestedObject("telemetry");
    tel["lat"] = lat;
    tel["lon"] = lon;
    tel["alt"] = alt;
    tel["speed"] = speed;
    tel["heading"] = heading;
    tel["satellites"] = 18;
    tel["pitch"] = pitch;
    tel["roll"] = roll;
    tel["yaw"] = yaw;
    tel["battery"] = battery;
    tel["voltage"] = voltage;
    tel["armed"] = armed;
    tel["mode"] = mode;
    tel["flight_state"] = flight_state;
    tel["current_wp"] = current_wp;

    // Gửi chuỗi JSON kết thúc bằng '\n' qua cổng Serial kết nối với Raspberry Pi
    serializeJson(doc, Serial1);
    Serial1.write('\n');
}

// Hàm tiếp nhận và phân tích lệnh từ Pi
void processUartCommandFromPi() {
    if (!Serial1.available()) return;
    
    String line = Serial1.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;

    StaticJsonDocument<1024> doc;
    DeserializationError error = deserializeJson(doc, line);
    if (error) {
        Serial.print("Lỗi parse JSON lệnh: ");
        Serial.println(error.c_str());
        return;
    }

    const char* cmd = doc["cmd"];
    if (strcmp(cmd, "takeoff") == 0) {
        float alt = doc["alt"] | 35.0f;
        // Thực thi lệnh cất cánh lên độ cao alt
    } else if (strcmp(cmd, "upload_mission") == 0) {
        int count = doc["count"];
        JsonArray waypoints = doc["waypoints"];
        for (JsonObject wp : waypoints) {
            float w_lat = wp["lat"];
            float w_lng = wp["lng"];
            // Lưu tọa độ vào flight plan của FC
        }
    } else if (strcmp(cmd, "start_mission") == 0) {
        // Kích hoạt bay AUTO theo danh sách waypoint
    } else if (strcmp(cmd, "stop_mission") == 0) {
        // Chuyển chế độ LOITER giữ vị trí
    } else if (strcmp(cmd, "rtl") == 0) {
        // Quay về vị trí cất cánh
    } else if (strcmp(cmd, "arm") == 0) {
        // Arm động cơ
    } else if (strcmp(cmd, "disarm") == 0) {
        // Disarm động cơ
    }
}
```

---

## 4. HƯỚNG DẪN KIỂM TRA CHẨN ĐOÁN TRÊN THỰC ĐỊA

Kỹ sư phần cứng có thể kết nối SSH vào Raspberry Pi 5 để kiểm tra tín hiệu trực tiếp bằng các công cụ dòng lệnh:

### 4.1. Kiểm tra tín hiệu CAN Bus
```bash
# 1. Kiểm tra interface can0 đã khởi tạo chưa:
ifconfig can0

# 2. Lắng nghe gói tin thực tế truyền từ UAV qua CAN:
candump can0
# Kết quả mong đợi hiển thị liên tục:
# can0  555   [8]  0B 1D 19 96 00 2D 01 E0
# can0  556   [8]  00 20 01 C8 0D FC 44 00

# 3. Thử bắn gói test từ Pi sang bo cảm biến:
cansend can0 555#0B1D199600100190
```

### 4.2. Kiểm tra tín hiệu UART Serial
```bash
# 1. Xem dữ liệu Telemetry truyền từ FC lên Pi theo thời gian thực:
cat /dev/serial0

# 2. Dùng minicom hoặc screen để tương tác trực tiếp:
minicom -b 115200 -o -D /dev/serial0

# 3. Thử gửi lệnh test từ Pi sang FC qua terminal:
echo '{"cmd":"takeoff","alt":35.0}' > /dev/serial0
```

---

## 5. TÓM TẮT CÁC LỖI THƯỜNG GẶP VÀ CÁCH KHẮC PHỤC

1. **Pi không nhận được dữ liệu UART:**
   - Kiểm tra chân GND giữa FC và Pi đã được nối chung chưa (Ground Loop).
   - Kiểm tra chân TX/RX: TX của FC phải nối vào RX của Pi (Pin 10), RX của FC nối vào TX của Pi (Pin 8).
   - Kiểm tra quyền truy cập cổng: Chạy `sudo usermod -a -G dialout $USER`.
   - Kiểm tra serial console trong `/boot/firmware/cmdline.txt` đã tắt chế độ `console=serial0,115200` chưa để giải phóng cổng.

2. **Giao tiếp CAN Bus bị lỗi Error-Passive / Bus-Off:**
   - Kiểm tra 2 điện trở đầu cuối **120 Ω** ở 2 đầu bus CAN_H và CAN_L. Tổng trở kháng đo nguội giữa CAN_H và CAN_L khi tắt nguồn phải đạt khoảng **60 Ω**.
   - Kiểm tra thạch anh của module MCP2515: Nếu bo phần cứng dùng thạch anh **8 MHz** mà cấu hình `oscillator=16000000` (hoặc ngược lại) thì bitrate sẽ bị lệch 50%, dẫn đến không đọc được dữ liệu.
   - Kiểm tra mức điện áp VCC của module MCP2515: Chip CAN Transceiver TJA1050 yêu cầu 5V để phân cực đúng vi sai trên CAN_H và CAN_L.
