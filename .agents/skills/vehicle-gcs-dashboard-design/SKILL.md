---
name: vehicle-gcs-dashboard-design
description: |
  Comprehensive design system and frontend architecture guide for building industrial Ground Control Station (GCS), UAV/drone telemetry, robotics, and vehicle dashboard web applications.

  Relevant when any of the following conditions are true:
    1. Designing, building, styling, or refactoring UI/UX for vehicle, UAV, drone, autonomous robot, or IoT ground control stations (GCS).
    2. Implementing cockpit telemetry widgets (artificial horizon, heading tape, battery/link pills, sensor grids, mission planners, camera PiP).
    3. Creating interactive Leaflet/map components with vehicle tracks, custom needle waypoint pins, satellite/street layer switching, and sovereign territory markers.
    4. Enforcing the high-contrast technical cockpit aesthetic, typography (Be Vietnam Pro, Montserrat, JetBrains Mono), and CSS token standards.
license: Apache-2.0
metadata:
  version: v1
  publisher: vietdai-bk
---

# Vehicle & GCS Dashboard Design System

Thiết kế giao diện bảng điều khiển trạm mặt đất (Ground Control Station - GCS), hệ thống giám sát phương tiện không người lái (UAV/Drone), robot tự hành và IoT theo phong cách **Technical Cockpit / Industrial High-Density**.

Giao diện tập trung vào tính **chắc chắn, đĩnh đạc, độ tương phản cao, thông tin mật độ cao nhưng ngăn nắp, phản hồi thời gian thực và vận hành mượt mà 60 FPS**.

---

## 1. Triết lý Thiết kế & Thẩm mỹ Visual

- **Phong cách Cockpit Công nghiệp**: Nền sáng nhẹ kỹ thuật (`#eef0f3`), bề mặt thẻ trắng sứ (`#ffffff`), đường phân cách sắc sảo (`#d3d8e0`), điểm nhấn xanh hải quân sâu (`#1a4d8f`).
- **Mật độ Thông tin Cao (High-Density & Scannable)**: Người điều khiển có thể nắm bắt toàn bộ trạng thái phương tiện (Pin, GPS, Vận tốc, Tọa độ, Trạng thái Vũ trang, Lỗi kết nối) trong vòng 2 giây.
- **Micro-Interactions Dứt khoát**:
  - Trạng thái hover tăng nhẹ độ sáng hoặc phóng to tỉ lệ nhỏ (`scale(1.05)`).
  - Đèn LED trạng thái có hiệu ứng thở nhịp tim (`pulse` 2s).
  - Không sử dụng hiệu ứng thừa thãi làm chậm hoặc gián đoạn thao tác điều khiển.
- **Khẳng định Chủ quyền Lãnh thổ**: Trên bản đồ Việt Nam, luôn đánh dấu trang trọng Quần đảo Hoàng Sa và Quần đảo Trường Sa bằng cờ đỏ sao vàng chuẩn tỷ lệ 2:3 với khả năng co giãn theo độ zoom bản đồ.

---

## 2. Hệ Thống Màu Sắc & CSS Design Tokens

Tất cả màu sắc và kích thước bắt buộc đi qua biến CSS trong `:root` để đảm bảo tính nhất quán và dễ dàng mở rộng sang Dark Mode:

```css
:root {
  /* Surface & Background */
  --bg: #eef0f3;
  --surface: #ffffff;
  --surface-2: #f6f7f9;
  --surface-hover: #edf2f9;
  --line: #d3d8e0;
  --line-strong: #aeb6c2;

  /* Typography Colors */
  --text: #14181f;       /* Đen than: chữ chính, tiêu đề */
  --text-2: #4d5766;     /* Xám thép: nhãn, phụ đề */
  --text-3: #7d8795;     /* Xám nhạt: ghi chú, đơn vị đo */

  /* Primary Accent: Navy Hải quân chỉ huy */
  --primary: #1a4d8f;
  --primary-hover: #153e73;
  --primary-ink: #ffffff;

  /* Semantic Status Colors (Theo chuẩn hàng không / quân sự) */
  --ok: #157a3a;          /* Xanh lục: Hoạt động bình thường, GPS Fix, Đã qua WP */
  --ok-bg: #e2f3e8;
  --warn: #b25e00;        /* Cam cháy: Cảnh báo, ARMED, Waypoint hiện tại */
  --warn-bg: #fff1dc;
  --danger: #b42318;      /* Đỏ đô: Khẩn cấp, Failsafe, Mất tín hiệu */
  --danger-bg: #fde7e5;
  --info: #0284c7;        /* Xanh da trời: Lộ trình, Điểm rẽ, Thông tin */
  --info-bg: #e0f2fe;

  /* Lớp bản đồ Vệ tinh: Màu neon tương phản cao trên nền ảnh vệ tinh tối */
  --sat-route: #00e5ff;
  --sat-street-route: #38bdf8;
  --sat-track: #00e676;

  /* Typography Tokens */
  --font: "Be Vietnam Pro", "Segoe UI", system-ui, -apple-system, sans-serif;
  --font-display: "Montserrat", "Be Vietnam Pro", sans-serif;
  --mono: "JetBrains Mono", "SFMono-Regular", Consolas, monospace;

  /* Layout Dimensions */
  --sidebar-w: 210px;
  --topbar-h: 48px;
  --radius: 4px;
  --shadow-sm: 0 1px 2px rgba(20, 24, 31, 0.06);
  --shadow-md: 0 4px 12px rgba(20, 24, 31, 0.12);
}
```

---

## 3. Tiêu Chuẩn Typography ("Chắc Chắn, Đậm Nét & Vững Chãi")

1. **Google Fonts bắt buộc tải trong `index.html`**:
   ```html
   <link rel="preconnect" href="https://fonts.googleapis.com">
   <link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
   <link href="https://fonts.googleapis.com/css2?family=Be+Vietnam+Pro:ital,wght@0,400;0,500;0,600;0,700;0,800;0,900;1,700&family=Montserrat:wght@600;700;800;900&family=JetBrains+Mono:wght@500;600;700;800&display=swap" rel="stylesheet">
   ```

2. **Quy tắc sử dụng font theo ngữ cảnh**:
   - **Tiêu đề, Nhãn Cờ, Huy hiệu, Tên trạm**: Dùng `Montserrat` (Weight `800` hoặc `900`), `text-transform: uppercase`, `letter-spacing: 0.04em` đến `0.06em`. Tạo cảm giác hình khối vuông vắn, cứng cáp và kỷ luật.
   - **Văn bản tiếng Việt, Nhãn đo, Hướng dẫn**: Dùng `Be Vietnam Pro` (Weight `500`, `600`, `700`). Đảm bảo dấu thanh tiếng Việt cân đối tuyệt đối, không bị dính nét, sắc cạnh ở cỡ chữ nhỏ (11px – 13px).
   - **Số liệu Telemetry, Tọa độ, Góc quay, Pin, Bộ đếm giờ**: Bắt buộc dùng `JetBrains Mono` với thuộc tính `font-variant-numeric: tabular-nums` để số nhảy thời gian thực không làm rung lắc bề rộng giao diện.

---

## 4. Kiến Trúc Bố Cục Cockpit GCS

Bố cục chuẩn là hệ thống **Grid 3 vùng chuyên biệt**:
```
+--------------------------------------------------------------------------+
| TopBar (48px): Brand | Trạng thái Kết nối | ARMED/DISARMED | Pin | Đồng hồ|
+-------------------+------------------------------------------------------+
| Sidebar (210px)   | Main Canvas                                          |
| - Giám sát Map    | - Bản đồ Leaflet Interactive (Chính)                 |
| - Telemetry số đo | - Bảng điều khiển Waypoint / Route Planning          |
| - Lịch sử chuyến bay| - Khung Camera PiP (Picture-in-Picture)            |
| - Cấu hình hệ thống| - HUD / Đồng hồ chân trời nhân tạo                  |
+-------------------+------------------------------------------------------+
```

### Quy tắc thiết kế TopBar:
- Luôn ghim cố định ở đỉnh màn hình (`height: 48px; border-bottom: 1px solid var(--line);`).
- **Brand Mark**: Biểu tượng hình vuông bo góc viền sắc nét với logo viết tắt (ví dụ: `GCS`, `UAV`).
- **Status Pills**:
  - `ARMED`: Nền cam, chữ in hoa đậm nét, viền nổi.
  - `GPS 3D FIX`: Hiển thị số lượng vệ tinh (ví dụ: `18 Sats`).
  - `BATTERY`: Thanh pin kèm điện áp chính xác `15.8V · 84%`.
  - `CONNECTION`: Chấm tròn xanh lục nhấp nháy 2s (`ping`/`pulse`).

---

## 5. Tiêu Chuẩn Bản Đồ Tương Tác (Leaflet GCS)

### A. Chuyển đổi Lớp Bản Đồ (Layer Switching)
Hỗ trợ 3 lớp hiển thị:
1. **Bản đồ đường (Street OSM)**: Thích hợp xem khu dân cư, đường xá.
2. **Vệ tinh (Esri World Imagery)**: Thích hợp quan sát địa hình thực tế biển đảo, đồi núi, bãi phóng.
3. **Vệ tinh nhãn (Google Hybrid)**: Có sẵn tên địa danh và đường sá trên nền ảnh vệ tinh.

### B. Mũi Ghim Waypoint (Custom Precision Pin)
Ghim waypoint phải cắm mũi kim chính xác 100% vào tọa độ:
```html
<div class="wp-pin-container ${territoryType ? 'has-vn-flag' : ''}">
  <!-- Lá cờ Tổ quốc nếu nằm tại Hoàng Sa hoặc Trường Sa -->
  ${territoryType ? `<div class="wp-vn-flag-overlay"><div class="wp-vn-mini-flag">${VIETNAM_FLAG_SVG}</div></div>` : ''}
  <!-- Nhãn tên waypoint phía trên -->
  <div class="wp-name-badge ${cls}">${wpTitle}</div>
  <!-- Biểu tượng tròn chứa số thứ tự -->
  <div class="wp-icon ${cls}">${index + 1}</div>
  <!-- Mũi nhọn tam giác chỉ đúng tọa độ -->
  <div class="wp-pin-tip ${cls}"></div>
</div>
```

### C. Đánh Dấu Chủ Quyền Biển Đảo với Cơ Chế Co Giãn Theo Zoom
Hai quần đảo Hoàng Sa và Trường Sa bắt buộc hiển thị mốc chủ quyền lá cờ Việt Nam.
Kích thước cờ được điều khiển mượt mà theo cấp số nhân dựa trên biến CSS trên container bản đồ:

```typescript
// Tính toán kích thước cờ theo zoom (zoom 3..18)
const updateZoomScale = () => {
  if (!map || !container) return;
  const zoom = map.getZoom();
  // Zoom 3 (toàn cảnh cả nước): ~26px
  // Zoom 7 (toàn cảnh Biển Đông): ~52px
  // Zoom 12 (khu vực quần đảo): ~110px
  // Zoom 16..18 (chi tiết đảo): ~210..260px
  const flagW = Math.round(Math.max(26, Math.min(260, 20 * Math.pow(1.18, Math.max(0, zoom - 3)))));
  const flagH = Math.round((flagW * 2) / 3);
  const fontSize = Math.max(9, Math.min(16, Math.round(flagW * 0.15)));
  
  container.style.setProperty("--vn-flag-w", `${flagW}px`);
  container.style.setProperty("--vn-flag-h", `${flagH}px`);
  container.style.setProperty("--vn-flag-font", `${fontSize}px`);
};

map.on("zoom", updateZoomScale);
```

### D. Phương Tiện Di Chuyển (Vehicle Marker)
- Sử dụng biểu tượng SVG máy bay / xe hình phi tiêu (`arrow-head`).
- Xoay mượt mà theo góc phương vị Heading: `el.style.transform = \`rotate(${heading}deg)\``.
- Vẽ vết đường đi thực tế (`actual track`) bằng nét liền xanh lá `#157a3a` (hoặc neon `#00e676` trên vệ tinh), bo tròn khớp nối (`lineCap: 'round', lineJoin: 'round'`).

---

## 6. Widget Telemetry & HUD Cockpit

### A. Thẻ Đo Đạc Cảm Biến (Telemetry Metric Card)
- Tiêu đề nhãn: `font-size: 11px; text-transform: uppercase; color: var(--text-2); font-weight: 700;`.
- Con số hiển thị: `font-family: var(--mono); font-size: 20px; font-weight: 800; color: var(--text);`.
- Đơn vị đo: `font-size: 11px; color: var(--text-3); margin-left: 4px; font-weight: 600;`.

### B. Chân Trời Nhân Tạo (Artificial Horizon / HUD)
- Vạch hiển thị góc Pitch & Roll với độ tương phản cao (nền trời xanh dương nhạt / nền đất nâu xám hoặc HUD neon tối giản).
- Thang đo Heading Tape nằm ngang hiển thị các hướng chính: `N (0°)`, `E (90°)`, `S (180°)`, `W (270°)`.

### C. Khung Video Camera PiP (Picture-in-Picture)
- Khung video dạng nổi góc dưới bản đồ, có nút Swap (hoán đổi vị trí toàn màn hình giữa Bản đồ và Camera).
- Khung viền kính mờ `backdrop-filter: blur(8px); border: 1px solid var(--line); border-radius: 6px;`.
- Tự động hiển thị huy hiệu trạng thái FPS và độ trễ luồng `30 FPS · 45ms`.

---

## 7. Quy Chuẩn Khi Viết Code Giao Diện

1. **Không dùng Tailwind CSS nếu không được yêu cầu**: Sử dụng Vanilla CSS với biến CSS tokens trong file `global.css`.
2. **Luôn xử lý dọn dẹp Map (Cleanup)**: Leaflet map bắt buộc gọi `map.remove()` trong hàm return của `useEffect` khi unmount để tránh lỗi `Map container is already initialized`.
3. **Đáp ứng Thay đổi Kích thước (ResizeObserver)**:
   ```typescript
   const ro = new ResizeObserver(() => map.invalidateSize());
   ro.observe(containerEl);
   return () => ro.disconnect();
   ```
4. **Build & Kiểm thử**: Sau mỗi lần thay đổi giao diện, luôn chạy `npm run build` để kiểm tra lỗi kiểu dữ liệu TypeScript và biên dịch asset tĩnh sẵn sàng triển khai.
