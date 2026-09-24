/**
 * UAV Ground Control Station (GCS) - Main Cockpit Script
 * Technical Cockpit / Industrial High-Density Standard
 * Complies with vehicle-gcs-dashboard-design SKILL.md
 */

let map;
let vehicleMarker = null;
let vehicleTrail = null;
let plannedRouteLine = null;
const waypoints = [];
const wpMarkers = [];
let dataPointMarkers = [];
let visitedWaypointData = {};
let latestTelemetryState = null;

let currentHeading = 0;
let currentPitch = 0;
let currentRoll = 0;
let isCameraSwapped = false;
let currentLayerName = 'roadmap';

// Map tile layers: Street (OSM), Satellite (Esri), Hybrid (Google)
const mapLayers = {
  roadmap: L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
    maxZoom: 19,
    attribution: '© OpenStreetMap'
  }),
  satellite: L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', {
    maxZoom: 19,
    attribution: '© Esri World Imagery'
  }),
  hybrid: L.tileLayer('https://mt1.google.com/vt/lyrs=y&x={x}&y={y}&z={z}', {
    maxZoom: 20,
    attribution: '© Google Satellite Hybrid'
  })
};

// ==================== ICONS & VISUAL STYLING ====================

/**
 * Custom Precision Needle Waypoint Pin
 * Mũi kim tam giác cắm chính xác 100% vào tọa độ
 */
function getWpIcon(index, isActive) {
  const cls = isActive ? 'active-target' : '';
  return L.divIcon({
    className: 'wp-pin-wrapper',
    html: `
      <div class="wp-pin-container ${cls}">
        <div class="wp-name-badge">WP${index + 1}</div>
        <div class="wp-icon">${index + 1}</div>
        <div class="wp-pin-tip"></div>
      </div>
    `,
    iconSize: [40, 50],
    iconAnchor: [20, 50], // Mũi nhọn cắm đúng tâm tọa độ GPS
    popupAnchor: [0, -50]
  });
}

/**
 * UAV Vehicle Marker Icon (Technical Jet / Arrowhead)
 */
function getVehicleIcon(heading) {
  const svg = `
    <svg viewBox="0 0 44 44" width="44" height="44" xmlns="http://www.w3.org/2000/svg">
      <defs>
        <filter id="uav-glow" x="-20%" y="-20%" width="140%" height="140%">
          <feGaussianBlur stdDeviation="2" result="blur" />
          <feMerge>
            <feMergeNode in="blur" />
            <feMergeNode in="SourceGraphic" />
          </feMerge>
        </filter>
      </defs>
      <!-- Outer Target Ring -->
      <circle cx="22" cy="22" r="18" fill="none" stroke="#1a4d8f" stroke-width="1.2" stroke-dasharray="3,3" opacity="0.6"/>
      <!-- Drone Body -->
      <path d="M22 6 L32 34 L22 28 L12 34 Z" fill="#1a4d8f" stroke="#00e5ff" stroke-width="2" stroke-linejoin="round" filter="url(#uav-glow)"/>
      <!-- Center Sensor Dome -->
      <circle cx="22" cy="22" r="3.5" fill="#ffffff" stroke="#1a4d8f" stroke-width="1.5"/>
    </svg>
  `;

  return L.divIcon({
    className: 'vehicle-marker-wrap',
    html: `<div id="vehicle-rotator" style="transform: rotate(${heading}deg); transition: transform 0.25s linear; width: 44px; height: 44px;">${svg}</div>`,
    iconSize: [44, 44],
    iconAnchor: [22, 22]
  });
}

/**
 * Cột mốc chủ quyền Quần đảo Hoàng Sa & Quần đảo Trường Sa
 */
function createSovereignMarker(lat, lon, title, isTruongSa = false) {
  const markerIcon = L.divIcon({
    className: 'sovereign-marker-container',
    html: `
      <div class="sovereign-territory-marker">
        <div class="sovereign-flag-box">
          <img src="/static/assets/flag_vn.png" alt="Cờ Tổ Quốc Việt Nam">
        </div>
        <div class="sovereign-badge ${isTruongSa ? 'truong-sa' : ''}">
          🇻🇳 ${title}
        </div>
      </div>
    `,
    iconSize: [160, 70],
    iconAnchor: [80, 35],
    popupAnchor: [0, -35]
  });

  const marker = L.marker([lat, lon], { icon: markerIcon, zIndexOffset: 800 }).addTo(map);
  marker.bindPopup(`
    <div style="text-align: center; padding: 6px 10px; font-family: var(--font);">
      <strong style="color: #b42318; font-size: 13px; font-family: var(--font-display);">🇻🇳 ${title}</strong>
      <div style="color: #157a3a; font-size: 11px; font-weight: 700; margin-top: 4px;">CHỦ QUYỀN KHÔNG THỂ TRANH CÃI CỦA VIỆT NAM</div>
    </div>
  `);
  return marker;
}

// ==================== BẢN ĐỒ KHỞI TẠO ====================
async function initMap() {
  let defaultCenter = [16.0743537, 108.1522514]; // Vịnh Đà Nẵng
  try {
    const res = await fetch('/vehicle-position');
    const d = await res.json();
    if (d.success && d.lat && d.lon) {
      defaultCenter = [d.lat, d.lon];
    }
  } catch (_) {}

  map = L.map('map', {
    zoomControl: false,
    attributionControl: false
  }).setView(defaultCenter, 16);

  // Mặc định lớp đường phố
  mapLayers.roadmap.addTo(map);

  // Điều khiển zoom góc dưới phải
  L.control.zoom({ position: 'bottomright' }).addTo(map);

  // Polyline vệt bay thực tế (Actual track) nét liền màu xanh
  vehicleTrail = L.polyline([], {
    color: '#157a3a',
    weight: 3.5,
    opacity: 0.9,
    lineCap: 'round',
    lineJoin: 'round'
  }).addTo(map);

  // Polyline lộ trình dự kiến (Planned route)
  plannedRouteLine = L.polyline([], {
    color: '#0284c7',
    weight: 2.5,
    opacity: 0.8,
    dashArray: '6, 8'
  }).addTo(map);

  // Vehicle Marker
  vehicleMarker = L.marker(defaultCenter, {
    icon: getVehicleIcon(currentHeading),
    zIndexOffset: 1000
  }).addTo(map);

  // Nhấp chuột vào UAV hiển thị trạng thái hiện tại (đang bay hay đang đo)
  vehicleMarker.on('click', function(e) {
    L.DomEvent.stopPropagation(e);
    const s = latestTelemetryState || {};
    const flight = s.flight || s;
    const currentWpNum = (flight.current_waypoint !== undefined ? flight.current_waypoint : (flight.current_wp ? flight.current_wp - 1 : 0)) + 1;
    
    let statusText = 'CHỜ BAY';
    let statusBg = '#f1f5f9';
    let statusColor = '#475569';
    let statusBorder = '#cbd5e1';

    const state = (flight.flight_state || flight.status || '').toUpperCase();

    if (state === 'HOLD' || flight.is_holding) {
      statusText = `ĐANG ĐO TẠI ĐIỂM #${currentWpNum}`;
      statusBg = '#fef3c7';
      statusColor = '#92400e';
      statusBorder = '#fde68a';
    } else if (state === 'RUNNING' || flight.is_flying) {
      statusText = `ĐANG BAY ĐẾN ĐIỂM #${currentWpNum}`;
      statusBg = '#e0f2fe';
      statusColor = '#0369a1';
      statusBorder = '#bae6fd';
    } else if (state === 'TAKEOFF') {
      statusText = 'ĐANG CẤT CÁNH';
      statusBg = '#e0f2fe';
      statusColor = '#0369a1';
      statusBorder = '#bae6fd';
    } else if (state === 'LANDING') {
      statusText = 'ĐANG HẠ CÁNH';
      statusBg = '#fee2e2';
      statusColor = '#b91c1c';
      statusBorder = '#fecaca';
    }

    const popupHtml = `
      <div style="font-family: var(--font), sans-serif; text-align: center; padding: 4px 6px; min-width: 170px;">
        <div style="font-size: 11px; font-weight: 700; color: #64748b; letter-spacing: 0.5px; margin-bottom: 5px;">TRẠNG THÁI UAV</div>
        <div style="display: inline-block; padding: 5px 12px; border-radius: 4px; background: ${statusBg}; color: ${statusColor}; border: 1px solid ${statusBorder}; font-weight: 800; font-size: 12px;">
          ${statusText}
        </div>
      </div>
    `;

    vehicleMarker.unbindPopup();
    vehicleMarker.bindPopup(popupHtml, { minWidth: 170, className: 'custom-wp-popup' }).openPopup();
  });

  // Đánh dấu Quần Đảo Hoàng Sa & Trường Sa
  createSovereignMarker(16.82847, 112.35718, 'QUẦN ĐẢO HOÀNG SA');
  createSovereignMarker(9.51058, 112.89551, 'QUẦN ĐẢO TRƯỜNG SA', true);

  // Cơ chế co giãn cờ Tổ quốc theo cấp số nhân zoom bản đồ
  const updateZoomScale = () => {
    if (!map) return;
    const zoom = map.getZoom();
    const flagW = Math.round(Math.max(28, Math.min(260, 22 * Math.pow(1.18, Math.max(0, zoom - 3)))));
    const flagH = Math.round((flagW * 2) / 3);
    const fontSize = Math.max(9, Math.min(16, Math.round(flagW * 0.15)));

    const container = document.getElementById('map');
    if (container) {
      container.style.setProperty("--vn-flag-w", `${flagW}px`);
      container.style.setProperty("--vn-flag-h", `${flagH}px`);
      container.style.setProperty("--vn-flag-font", `${fontSize}px`);
    }
  };
  map.on("zoom", updateZoomScale);
  updateZoomScale();

  // Click bản đồ để thêm Waypoint
  map.on('click', e => {
    addWaypoint(e.latlng.lat, e.latlng.lng);
  });

  // Đổi lớp bản đồ (Roadmap, Satellite, Hybrid)
  setupMapLayerSwitching();

  // Khởi động đồng bộ dữ liệu
  loadCollectedData();
  loadCurrentMission();

  // Lắng nghe sự kiện viễn trắc tổng
  window.addEventListener('uav-telemetry-packet', handleTelemetryPacket);
}

// Chuyển đổi lớp bản đồ
function setupMapLayerSwitching() {
  const btns = document.querySelectorAll('.layer-btn');
  btns.forEach(btn => {
    btn.addEventListener('click', function() {
      btns.forEach(b => b.classList.remove('active'));
      this.classList.add('active');
      const targetLayer = this.getAttribute('data-layer');
      if (mapLayers[targetLayer] && targetLayer !== currentLayerName) {
        map.removeLayer(mapLayers[currentLayerName]);
        mapLayers[targetLayer].addTo(map);
        currentLayerName = targetLayer;

        // Cập nhật màu sắc vệt bay tương phản trên vệ tinh
        if (targetLayer === 'satellite' || targetLayer === 'hybrid') {
          if (vehicleTrail) vehicleTrail.setStyle({ color: '#00e676' });
          if (plannedRouteLine) plannedRouteLine.setStyle({ color: '#00e5ff' });
        } else {
          if (vehicleTrail) vehicleTrail.setStyle({ color: '#157a3a' });
          if (plannedRouteLine) plannedRouteLine.setStyle({ color: '#0284c7' });
        }
      }
    });
  });
}

// ==================== XỬ LÝ VIỄN TRẮC & HUD ====================
function handleTelemetryPacket(event) {
  const s = event.detail.state;
  if (!s) return;
  latestTelemetryState = s;

  // 1. Cập nhật vị trí & xoay UAV
  const lat = s.gps ? s.gps.lat : s.latitude;
  const lon = s.gps ? s.gps.lon : s.longitude;
  const alt = s.gps ? s.gps.alt : (s.altitude || 0);
  const spd = s.gps ? s.gps.speed : (s.speed || 0);
  const hdg = s.gps ? s.gps.heading : (s.heading || 0);
  const pitch = s.attitude ? s.attitude.pitch : (s.pitch || 0);
  const roll = s.attitude ? s.attitude.roll : (s.roll || 0);

  if (lat && lon && vehicleMarker) {
    const latlng = [lat, lon];
    vehicleMarker.setLatLng(latlng);

    // Xoay SVG marker
    const rot = document.getElementById('vehicle-rotator');
    if (rot) {
      rot.style.transform = `rotate(${hdg}deg)`;
    }

    // Thêm điểm vào vệt bay
    if (s.armed || (s.flight && s.flight.is_flying)) {
      vehicleTrail.addLatLng(latlng);
    }
  }

  // 2. Cập nhật Artificial Horizon & Heading Tape
  updateCockpitHUD(pitch, roll, hdg);

  // 3. Cập nhật các chỉ số đo cảm biến (2 giây scannable)
  updateSensorMetrics(s);

  // 4. Cập nhật tiến độ Holding nếu UAV đang dừng lấy mẫu
  updateHoldingProgress(s);

  // 5. Cập nhật chỉ số waypoint hiện tại
  updateWaypointStatus(s);

  // 6. Ghi nhận dữ liệu và vẽ đồ thị độ cao thời gian thực (Altitude Chart)
  recordAltitudePoint(alt, s.target_altitude || 35.0, s.terrain_altitude || 7.0);

  // 7. Đồng bộ tức thời dữ liệu đo đạc tại các điểm khi nhận packet viễn trắc
  const collectedList = s.collected_data || (event.detail.raw && event.detail.raw.collected_data);
  if (collectedList && Array.isArray(collectedList) && collectedList.length > 0) {
    const updated = matchCollectedDataToWaypoints(collectedList);
    const tableBody = document.getElementById('collected-data-table-body');
    const isTableEmpty = tableBody && (tableBody.children.length <= 1 && tableBody.textContent.includes('Chưa có'));
    if (updated || isTableEmpty) {
      renderCollectedDataTable(collectedList);
      renderDataPointsOnMap(collectedList);
      renderWaypointList();
    }
  }
}

function updateCockpitHUD(pitch, roll, hdg) {
  currentPitch = pitch;
  currentRoll = roll;
  currentHeading = hdg;

  // 1. Attitude Indicator Gauge (Pitch translateY & Roll rotate)
  const sphere = document.getElementById('horizon-sphere');
  if (sphere) {
    const translateY = Math.max(-26, Math.min(26, pitch * 1.3));
    sphere.style.transform = `translateY(${translateY}px) rotate(${-roll}deg)`;
  }
  const pitchEl = document.getElementById('val-pitch');
  const rollEl = document.getElementById('val-roll');
  if (pitchEl) pitchEl.textContent = `${pitch >= 0 ? '+' : ''}${pitch.toFixed(1)}`;
  if (rollEl) rollEl.textContent = `${roll >= 0 ? '+' : ''}${roll.toFixed(1)}`;

  // 2. Compass Dial Gauge (Needle rotates to heading)
  const needle = document.getElementById('compass-needle');
  if (needle) {
    needle.style.transform = `rotate(${hdg}deg)`;
  }
  const hdgEl = document.getElementById('val-heading');
  if (hdgEl) {
    hdgEl.textContent = String(hdg.toFixed(1)).padStart(5, '0');
  }
}

function updateSensorMetrics(s) {
  const sens = s.sensors || s;

  // Cập nhật số liệu cảm biến dạng chip gọn gàng
  setVal('val-pm25', sens.pm25 !== undefined ? sens.pm25.toFixed(1) : '--');
  setVal('val-pm10', sens.pm10 !== undefined ? sens.pm10.toFixed(1) : '--');
  setVal('val-eco2', sens.eco2 !== undefined ? Math.round(sens.eco2) : (sens.co2 ? Math.round(sens.co2) : '--'));
  setVal('val-tvoc', sens.tvoc !== undefined ? Math.round(sens.tvoc) : '--');
  setVal('val-co', sens.co !== undefined ? sens.co.toFixed(2) : '--');
  setVal('val-no2', sens.no2 !== undefined ? sens.no2.toFixed(1) : '--');
  setVal('val-temp', sens.temp !== undefined ? sens.temp.toFixed(1) : (sens.temperature ? sens.temperature.toFixed(1) : '--'));
  setVal('val-hum', sens.hum !== undefined ? sens.hum.toFixed(1) : (sens.humidity ? sens.humidity.toFixed(1) : '--'));

  // Flight mini status bar
  const alt = s.gps ? s.gps.alt : (s.altitude || 0);
  const spd = s.gps ? s.gps.speed : (s.speed || 0);
  setVal('val-alt', `${alt.toFixed(1)}m`);
  setVal('val-spd', `${spd.toFixed(1)}m/s`);
  setVal('val-dist', (s.flight && s.flight.distance_to_wp) ? `${s.flight.distance_to_wp.toFixed(0)}m` : '--');

  const batPct = s.battery !== undefined ? s.battery : 0;
  const voltage = (s.voltage !== undefined ? Number(s.voltage) : (14.2 + (batPct / 100) * 2.6)).toFixed(1);
  const batEl = document.getElementById('val-bat');
  if (batEl) {
    batEl.textContent = `${Math.round(batPct)}% · ${voltage}V`;
    batEl.style.color = batPct > 50 ? 'var(--ok)' : (batPct > 20 ? 'var(--warn)' : 'var(--danger)');
  }

  // AQI Compact Badge (Thang đo Việt Nam VN_AQI 0-500)
  const aqi = sens.aqi !== undefined ? Math.round(sens.aqi) : 1;
  const aqiCat = sens.aqi_category || (aqi <= 50 ? 'Tốt' : (aqi <= 100 ? 'Trung bình' : (aqi <= 150 ? 'Kém' : (aqi <= 200 ? 'Xấu' : (aqi <= 300 ? 'Rất xấu' : 'Nguy hại')))));
  setVal('aqi-score-num', aqi);
  setVal('aqi-cat-name', aqiCat);

  const aqiBadge = document.getElementById('aqi-badge');
  if (aqiBadge) {
    let border = '#157a3a', bg = '#e2f3e8', text = '#157a3a';
    if (aqi <= 50) {
      border = '#157a3a'; bg = '#e2f3e8'; text = '#157a3a';
    } else if (aqi <= 100) {
      border = '#b25e00'; bg = '#fff1dc'; text = '#b25e00';
    } else if (aqi <= 150) {
      border = '#e65100'; bg = '#ffe0b2'; text = '#e65100';
    } else if (aqi <= 200) {
      border = '#b42318'; bg = '#fde7e5'; text = '#b42318';
    } else if (aqi <= 300) {
      border = '#7b1fa2'; bg = '#f3e5f5'; text = '#7b1fa2';
    } else {
      border = '#4a148c'; bg = '#ede7f6'; text = '#4a148c';
    }
    aqiBadge.style.borderColor = border;
    aqiBadge.style.background = bg;
    aqiBadge.style.color = text;
  }
}

function setVal(id, text) {
  const el = document.getElementById(id);
  if (el) el.textContent = text;
}

function updateHoldingProgress(s) {
  const flight = s.flight || s;
  const isHolding = flight.is_holding;
  const wrap = document.getElementById('holding-wrap');
  const bar = document.getElementById('holding-fill');
  const ratio = document.getElementById('holding-ratio');

  if (!wrap || !bar || !ratio) return;

  if (isHolding) {
    wrap.classList.add('active');
    const samples = flight.hold_samples || 0;
    const total = flight.hold_total || 10;
    const pct = Math.min(100, Math.round((samples / total) * 100));
    bar.style.width = `${pct}%`;
    ratio.textContent = `${samples}/${total} (${pct}%)`;
  } else {
    wrap.classList.remove('active');
  }
}

function getDistanceFromLatLng(lat1, lon1, lat2, lon2) {
  if (lat1 === undefined || lon1 === undefined || lat2 === undefined || lon2 === undefined) return 999999;
  if (typeof L !== 'undefined' && L.latLng) {
    try {
      return L.latLng(lat1, lon1).distanceTo(L.latLng(lat2, lon2));
    } catch (_) {}
  }
  const R = 6371000;
  const dLat = (lat2 - lat1) * Math.PI / 180;
  const dLon = (lon2 - lon1) * Math.PI / 180;
  const a = Math.sin(dLat / 2) * Math.sin(dLat / 2) +
            Math.cos(lat1 * Math.PI / 180) * Math.cos(lat2 * Math.PI / 180) *
            Math.sin(dLon / 2) * Math.sin(dLon / 2);
  return R * (2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a)));
}

function matchCollectedDataToWaypoints(collectedList) {
  if (!collectedList || !Array.isArray(collectedList) || waypoints.length === 0) return false;
  let updated = false;

  waypoints.forEach((wp) => {
    let bestMatch = null;
    let minDistance = 35; // Chỉ gán dữ liệu đo nếu vị trí đo thực tế nằm gần điểm đo này (< 35m)

    collectedList.forEach(item => {
      const itemLat = item.lat;
      const itemLon = item.lon !== undefined ? item.lon : item.lng;
      const d = getDistanceFromLatLng(wp.lat, wp.lng, itemLat, itemLon);
      if (d < minDistance) {
        minDistance = d;
        bestMatch = item;
      }
    });

    if (bestMatch && wp.measuredData !== bestMatch) {
      wp.measuredData = bestMatch;
      updated = true;
    }
  });

  return updated;
}

function showWaypointAirQualityPopup(wpIndex, marker) {
  if (wpIndex < 0 || wpIndex >= waypoints.length) return;
  const wp = waypoints[wpIndex];
  const targetMarker = marker || wpMarkers[wpIndex];
  if (!targetMarker) return;

  // CHỈ lấy dữ liệu nếu điểm này ĐÃ ĐƯỢC ĐO tại vị trí thực tế trong lộ trình
  const data = wp.measuredData;
  const s = latestTelemetryState || {};
  const flight = s.flight || s;
  const currentIdx = flight.current_waypoint !== undefined ? flight.current_waypoint : (flight.current_wp ? flight.current_wp - 1 : 0);
  const isHoldingHere = (wpIndex === currentIdx && flight.is_holding);

  let contentHtml = `
    <div class="wp-measurement-popup">
      <div class="wp-popup-title-bar">
        <span class="wp-popup-name">ĐIỂM ĐO #${wpIndex + 1}</span>
  `;

  if (data) {
    const aqi = data.aqi || 1;
    let aqiColor = '#15803d';
    let aqiBg = '#dcfce7';
    let aqiCat = data.aqi_category || 'Tốt';

    if (aqi <= 50) {
      aqiColor = '#15803d'; aqiBg = '#dcfce7'; aqiCat = 'Tốt';
    } else if (aqi <= 100) {
      aqiColor = '#b45309'; aqiBg = '#fef3c7'; aqiCat = 'Trung bình';
    } else if (aqi <= 150) {
      aqiColor = '#c2410c'; aqiBg = '#ffedd5'; aqiCat = 'Kém';
    } else if (aqi <= 200) {
      aqiColor = '#b91c1c'; aqiBg = '#fee2e2'; aqiCat = 'Xấu';
    } else {
      aqiColor = '#7e0023'; aqiBg = '#fce7f3'; aqiCat = 'Rất xấu';
    }

    contentHtml += `
        <span class="wp-badge-status done">ĐÃ HOÀN TẤT</span>
      </div>
      <div class="wp-popup-aqi-box" style="background:${aqiBg}; border-color:${aqiColor}40;">
        <span class="wp-aqi-text" style="color:${aqiColor};">VN_AQI: <b>${aqi}</b> (${aqiCat})</span>
        <span style="font-size: 9.5px; color:${aqiColor}; font-weight: 700;">${data.sample_count || 10} mẫu</span>
      </div>
      <div class="wp-popup-metrics-table">
        <div class="wp-metric-row"><span class="lbl">Bụi mịn PM2.5:</span><span class="val">${(data.pm25 || 0).toFixed(1)} µg/m³</span></div>
        <div class="wp-metric-row"><span class="lbl">Bụi mịn PM10:</span><span class="val">${(data.pm10 || (data.pm25 * 1.5) || 0).toFixed(1)} µg/m³</span></div>
        <div class="wp-metric-row"><span class="lbl">Khí CO₂ (eCO₂):</span><span class="val">${Math.round(data.eco2 || 0)} mg/m³</span></div>
        <div class="wp-metric-row"><span class="lbl">Hợp chất TVOC:</span><span class="val">${Math.round(data.tvoc || 0)} ppb</span></div>
        <div class="wp-metric-row"><span class="lbl">Khí CO:</span><span class="val">${(data.co || 0).toFixed(2)} mg/m³</span></div>
        <div class="wp-metric-row"><span class="lbl">Khí NO₂:</span><span class="val">${(data.no2 || 0).toFixed(1)} µg/m³</span></div>
        <div class="wp-metric-row"><span class="lbl">Nhiệt độ / Độ ẩm:</span><span class="val">${(data.temp || data.temperature || 29).toFixed(1)}°C | ${(data.hum || data.humidity || 65).toFixed(1)}%</span></div>
      </div>
      <div class="wp-popup-meta">
        <div>Tọa độ: ${wp.lat.toFixed(5)}, ${wp.lng.toFixed(5)}</div>
        <div>Thời gian đo: ${new Date(data.time).toLocaleTimeString('vi-VN')}</div>
      </div>
    `;
  } else if (isHoldingHere) {
    const samples = flight.hold_samples || 0;
    const total = flight.hold_total || 10;
    contentHtml += `
        <span class="wp-badge-status measuring">ĐANG ĐO</span>
      </div>
      <div style="background:#fef3c7;border:1px solid #fde68a;border-radius:4px;padding:8px 10px;font-size:11px;color:#92400e;margin-bottom:6px;">
        <div style="font-weight:700;">Đang lấy mẫu chất lượng không khí...</div>
        <div style="margin-top:3px;font-size:10.5px;">Tiến độ: <b>${samples}/${total}</b> mẫu (${Math.round(samples/total*100)}%)</div>
      </div>
      <div style="font-size:10px;color:#64748b;margin-bottom:6px;line-height:1.4;">
        Số liệu đo đạc sẽ hiển thị đầy đủ sau khi UAV hoàn thành 10 mẫu đo.
      </div>
      <div class="wp-popup-meta">
        <div>Tọa độ: ${wp.lat.toFixed(5)}, ${wp.lng.toFixed(5)}</div>
      </div>
    `;
  } else {
    contentHtml += `
        <span class="wp-badge-status pending">CHƯA ĐO</span>
      </div>
      <div style="background:#f8fafc;border:1px solid #e2e8f0;border-radius:4px;padding:8px 10px;font-size:11px;color:#64748b;margin-bottom:6px;line-height:1.4;">
        Điểm đo này chưa có dữ liệu. UAV sẽ tiến hành đo sau khi bay tới vị trí này.
      </div>
      <div class="wp-popup-meta">
        <div>Tọa độ: ${wp.lat.toFixed(5)}, ${wp.lng.toFixed(5)}</div>
      </div>
    `;
  }

  contentHtml += `</div>`;

  targetMarker.unbindPopup();
  targetMarker.bindPopup(contentHtml, {
    maxWidth: 280,
    minWidth: 230,
    className: 'custom-wp-popup'
  }).openPopup();
}

function openWaypointPopup(idx) {
  if (idx < 0 || idx >= waypoints.length) return;
  const wp = waypoints[idx];
  const marker = wpMarkers[idx];
  if (map && wp) {
    map.setView([wp.lat, wp.lng], Math.max(16, map.getZoom()), { animate: true });
    if (marker) {
      showWaypointAirQualityPopup(idx, marker);
    }
  }
}

let lastKnownWpIdx = -1;
let lastKnownHolding = false;
let lastKnownHoldSamples = -1;

function updateWaypointStatus(s) {
  const flight = s.flight || s;
  const currentIdx = flight.current_waypoint !== undefined ? flight.current_waypoint : (flight.current_wp ? flight.current_wp - 1 : 0);
  const isHolding = !!flight.is_holding;
  const holdSamples = flight.hold_samples || 0;

  // Cập nhật lại danh sách nếu chuyển waypoint hoặc đổi trạng thái đo hoặc thay đổi số mẫu
  if (currentIdx !== lastKnownWpIdx || isHolding !== lastKnownHolding || (isHolding && holdSamples !== lastKnownHoldSamples)) {
    lastKnownWpIdx = currentIdx;
    lastKnownHolding = isHolding;
    lastKnownHoldSamples = holdSamples;
    renderWaypointList();
  }

  // Cập nhật highlight cho danh sách Waypoint
  const items = document.querySelectorAll('.wp-compact-item');
  items.forEach((item, idx) => {
    item.classList.remove('current-target', 'visited');
    const w = waypoints[idx];
    if (w && w.measuredData) {
      item.classList.add('visited');
    } else if (idx === currentIdx && (flight.is_flying || flight.is_holding)) {
      item.classList.add('current-target');
    }
  });

  // Cập nhật icon trên bản đồ
  wpMarkers.forEach((m, idx) => {
    const isActive = (idx === currentIdx && (flight.is_flying || flight.is_holding));
    m.setIcon(getWpIcon(idx, isActive));
  });
}

// ==================== QUẢN LÝ WAYPOINT & MISSION ====================
function addWaypoint(lat, lng) {
  // Chỉ lưu trữ và hiển thị tọa độ GPS (lat, lng), không lưu độ cao, ban đầu chưa có dữ liệu đo
  const wp = { lat: Number(lat), lng: Number(lng), measuredData: null };
  waypoints.push(wp);
  const currentIdx = waypoints.length - 1;

  const marker = L.marker([lat, lng], {
    icon: getWpIcon(currentIdx, false),
    draggable: true
  }).addTo(map);

  // Bấm vào waypoint trên bản đồ để xem chi tiết chất lượng không khí
  marker.on('click', function(e) {
    L.DomEvent.stopPropagation(e);
    const idx = wpMarkers.indexOf(marker);
    if (idx !== -1) {
      showWaypointAirQualityPopup(idx, marker);
    }
  });

  marker.on('dragend', function(e) {
    const pos = e.target.getLatLng();
    const idx = wpMarkers.indexOf(marker);
    if (idx !== -1) {
      waypoints[idx].lat = pos.lat;
      waypoints[idx].lng = pos.lng;
      waypoints[idx].measuredData = null; // Tọa độ thay đổi -> reset dữ liệu đo
      updateRouteLine();
      renderWaypointList();
    }
  });

  wpMarkers.push(marker);
  updateRouteLine();
  renderWaypointList();
}

function updateRouteLine() {
  if (!plannedRouteLine) return;
  const pts = waypoints.map(w => [w.lat, w.lng]);
  plannedRouteLine.setLatLngs(pts);
}

function renderWaypointList() {
  const listEl = document.getElementById('wp-list-container');
  const countEl = document.getElementById('wp-count');
  if (!listEl) return;

  if (countEl) countEl.textContent = `${waypoints.length}`;

  if (waypoints.length === 0) {
    listEl.innerHTML = `<div style="text-align:center;color:var(--text-3);padding:10px;font-size:10.5px;">Chưa có điểm đo. Nhấp chuột lên bản đồ để thêm điểm.</div>`;
    return;
  }

  const s = latestTelemetryState || {};
  const flight = s.flight || s;
  const currentIdx = flight.current_waypoint !== undefined ? flight.current_waypoint : (flight.current_wp ? flight.current_wp - 1 : 0);

  // Hiển thị danh sách điểm đo kèm trạng thái và nút xem chi tiết
  listEl.innerHTML = waypoints.map((w, i) => {
    const hasData = !!w.measuredData;
    const isTarget = (i === currentIdx && (flight.is_flying || flight.is_holding));

    let statusChip = '';
    if (hasData) {
      const wpAqi = w.measuredData.aqi || 1;
      statusChip = `<span class="wp-status-chip done" title="Bấm để xem số liệu đo">Đã đo (AQI ${wpAqi})</span>`;
    } else if (i === currentIdx && flight.is_holding) {
      const samples = flight.hold_samples || 0;
      statusChip = `<span class="wp-status-chip active" style="background:#fef3c7;color:#92400e;border:1px solid #fde68a;">Đang đo (${samples}/10)</span>`;
    } else if (i === currentIdx && (flight.is_flying || flight.flight_state === 'RUNNING')) {
      statusChip = `<span class="wp-status-chip" style="background:#e0f2fe;color:#0369a1;border:1px solid #bae6fd;">Đang bay tới</span>`;
    } else {
      statusChip = `<span class="wp-status-chip pending">Chờ bay</span>`;
    }

    return `
      <div class="wp-compact-item ${hasData ? 'visited' : ''} ${isTarget ? 'current-target' : ''}" id="wp-row-${i}" onclick="openWaypointPopup(${i})" title="Bấm để xem chi tiết điểm đo #${i + 1}">
        <div style="display:flex;align-items:center;gap:6px;">
          <span style="font-family:var(--mono);font-weight:800;color:var(--primary);">#${i + 1}</span>
          <span style="font-family:var(--mono);font-size:10px;color:var(--text);font-weight:600;">${w.lat.toFixed(5)}, ${w.lng.toFixed(5)}</span>
          ${statusChip}
        </div>
        <button class="wp-del-btn" onclick="event.stopPropagation(); removeWaypoint(${i});" title="Xóa điểm này">×</button>
      </div>
    `;
  }).join('');
}

function removeWaypoint(idx) {
  if (idx < 0 || idx >= waypoints.length) return;
  map.removeLayer(wpMarkers[idx]);
  waypoints.splice(idx, 1);
  wpMarkers.splice(idx, 1);

  // Cập nhật lại số thứ tự icon
  wpMarkers.forEach((m, i) => m.setIcon(getWpIcon(i, false)));
  updateRouteLine();
  renderWaypointList();
}

function clearLocalWaypointsOnly() {
  wpMarkers.forEach(m => map.removeLayer(m));
  waypoints.length = 0;
  wpMarkers.length = 0;
  updateRouteLine();
  renderWaypointList();
}

function clearAllWaypoints() {
  if (!confirm("⚠️ XÁC NHẬN XÓA LỘ TRÌNH\n\nBạn có chắc chắn muốn xóa toàn bộ điểm đo trên bản đồ và hủy nhiệm vụ hiện tại?")) {
    return;
  }
  clearLocalWaypointsOnly();
  fetch('/clear-mission', { method: 'POST' }).catch(() => {});
}

async function uploadMissionToServer() {
  if (waypoints.length === 0) {
    alert("Vui lòng nhấp chuột trên bản đồ để thêm ít nhất 1 điểm đo!");
    return;
  }

  // Reset toàn bộ dữ liệu đo của các waypoint mới để tuyệt đối không gán dữ liệu lịch sử cũ
  waypoints.forEach(w => {
    w.measuredData = null;
  });
  renderWaypointList();

  // QUY TẮC: GỬI WAYPOINT CHỈ GỬI VỊ TRÍ TỌA ĐỘ (lat, lng), KHÔNG GỬI ĐỘ CAO (alt) SANG
  const missionPayload = waypoints.map(w => ({
    lat: Number(w.lat.toFixed(7)),
    lng: Number(w.lng.toFixed(7))
  }));

  try {
    const res = await fetch('/upload-mission', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ mission: missionPayload, alt: configuredAltitude })
    });
    const d = await res.json();
    if (d.success) {
      alert(`Đã nạp thành công ${waypoints.length} điểm đo vào hệ thống tự hành (Độ cao bay: ${configuredAltitude}m)! Bấm 'Bắt đầu bay' để cất cánh hoặc tiếp tục hành trình.`);
    } else {
      alert("Lỗi: " + d.message);
    }
  } catch (e) {
    alert("Lỗi khi gửi lộ trình lên máy chủ: " + e.message);
  }
}

// ==================== ĐỒ THỊ ĐỘ CAO THEO THỜI GIAN (ALTITUDE REALTIME CHART) ====================
let altitudeHistory = [];
let altitudeStartTime = null;

function recordAltitudePoint(actualAlt, targetAlt = 35.0, terrainAlt = 7.0) {
  if (!altitudeStartTime) {
    altitudeStartTime = Date.now();
  }
  const elapsedSec = (Date.now() - altitudeStartTime) / 1000;
  altitudeHistory.push({
    time: Number(elapsedSec.toFixed(1)),
    actual: Number(actualAlt),
    target: Number(targetAlt),
    terrain: Number(terrainAlt)
  });

  // Lưu trữ tối đa 150 điểm gần nhất
  if (altitudeHistory.length > 150) {
    altitudeHistory.shift();
  }

  // Cập nhật số điểm (e.g. 40 pts)
  const ptsEl = document.getElementById('alt-pts-counter');
  if (ptsEl) {
    ptsEl.textContent = `${altitudeHistory.length} pts`;
  }

  renderAltitudeChart();
}

function renderAltitudeChart() {
  const canvas = document.getElementById('altitude-canvas');
  if (!canvas) return;

  const rect = canvas.parentElement.getBoundingClientRect();
  if (rect.width === 0) return;

  const dpr = window.devicePixelRatio || 1;
  const w = rect.width;
  const h = 120;

  if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
    canvas.width = Math.round(w * dpr);
    canvas.height = Math.round(h * dpr);
  }

  const ctx = canvas.getContext('2d');
  ctx.save();
  ctx.scale(dpr, dpr);
  ctx.clearRect(0, 0, w, h);

  // Khoảng đệm khung vẽ
  const padLeft = 36;
  const padRight = 14;
  const padTop = 10;
  const padBottom = 20;

  const plotW = Math.max(10, w - padLeft - padRight);
  const plotH = Math.max(10, h - padTop - padBottom);

  // Thang đo trục Y động theo độ cao cài đặt: tối thiểu 35m, tự động mở rộng nếu độ cao cài đặt lớn hơn
  const targetAlt = (typeof configuredAltitude !== 'undefined') ? configuredAltitude : 35.0;
  const minY = 0.0;
  const maxY = Math.max(35.0, Math.ceil(targetAlt / 7.0) * 7.0);

  function toY(altVal) {
    const clamped = Math.max(minY, Math.min(maxY, altVal));
    return padTop + plotH - ((clamped - minY) / (maxY - minY)) * plotH;
  }

  // 1. Vẽ các đường chia ngang & nhãn Y
  const yTicks = [];
  const yStep = maxY > 70 ? 14 : 7;
  for (let y = 0; y <= maxY; y += yStep) {
    yTicks.push(y);
  }
  ctx.font = "9px 'JetBrains Mono', monospace";
  ctx.textAlign = "right";
  ctx.textBaseline = "middle";

  yTicks.forEach(yVal => {
    const yPx = toY(yVal);

    // Lưới chấm ngang
    ctx.beginPath();
    ctx.setLineDash([2, 3]);
    ctx.strokeStyle = "#e2e8f0";
    ctx.lineWidth = 1;
    ctx.moveTo(padLeft, yPx);
    ctx.lineTo(padLeft + plotW, yPx);
    ctx.stroke();

    // Nhãn trục Y
    ctx.fillStyle = "#94a3b8";
    ctx.fillText(`${yVal}m`, padLeft - 6, yPx);
  });

  // 2. Phạm vi thời gian trục X
  const n = altitudeHistory.length;
  let minT = 0;
  let maxT = 4.0; // Tối thiểu hiển thị cửa sổ 4.0s như ảnh

  if (n > 0) {
    const lastT = altitudeHistory[n - 1].time;
    if (lastT > 4.0) {
      maxT = Math.ceil(lastT * 10) / 10;
      minT = Math.max(0, Math.round((maxT - 15) * 10) / 10);
    }
  }

  function toX(tVal) {
    return padLeft + ((tVal - minT) / Math.max(0.1, maxT - minT)) * plotW;
  }

  // Vẽ các vạch chia dọc & nhãn thời gian X: 0.1s - 0.5s - 1s...
  const rangeT = maxT - minT;
  const stepT = rangeT <= 5 ? 0.5 : (rangeT <= 15 ? 1 : 2);
  ctx.textAlign = "center";
  ctx.textBaseline = "top";

  for (let t = Math.ceil(minT / stepT) * stepT; t <= maxT + 0.01; t += stepT) {
    const tRounded = Number(t.toFixed(1));
    const xPx = toX(tRounded);
    if (xPx >= padLeft && xPx <= padLeft + plotW) {
      // Vạch chấm dọc
      ctx.beginPath();
      ctx.setLineDash([2, 3]);
      ctx.strokeStyle = "#f1f5f9";
      ctx.moveTo(xPx, padTop);
      ctx.lineTo(xPx, padTop + plotH);
      ctx.stroke();

      // Nhãn thời gian trục X
      ctx.fillStyle = "#94a3b8";
      ctx.fillText(`${tRounded}s`, xPx, padTop + plotH + 5);
    }
  }

  // 3. Đường TERRAIN (Mặt đất / Địa hình - nét liền màu hổ phách #d97706)
  const terrainAlt = 7.0; // 7m theo ảnh mẫu
  const terrainY = toY(terrainAlt);
  ctx.beginPath();
  ctx.setLineDash([]);
  ctx.strokeStyle = "#d97706";
  ctx.lineWidth = 1.8;
  ctx.moveTo(padLeft, terrainY);
  ctx.lineTo(padLeft + plotW, terrainY);
  ctx.stroke();

  // 4. Đường TARGET (Độ cao mục tiêu - nét đứt màu xanh dương #0284c7)
  const targetAltVal = (typeof configuredAltitude !== 'undefined') ? configuredAltitude : 35.0;
  const targetY = toY(targetAltVal);
  ctx.beginPath();
  ctx.setLineDash([5, 4]);
  ctx.strokeStyle = "#0284c7";
  ctx.lineWidth = 1.8;
  ctx.moveTo(padLeft, targetY);
  ctx.lineTo(padLeft + plotW, targetY);
  ctx.stroke();

  // 5. Đường ACTUAL (Độ cao thực tế UAV - nét liền màu xanh lá #157a3a)
  if (n > 0) {
    ctx.beginPath();
    ctx.setLineDash([]);
    ctx.strokeStyle = "#157a3a";
    ctx.lineWidth = 2.0;
    ctx.lineCap = "round";
    ctx.lineJoin = "round";

    let started = false;
    for (let i = 0; i < n; i++) {
      const pt = altitudeHistory[i];
      if (pt.time >= minT - 0.2) {
        const x = toX(pt.time);
        const y = toY(pt.actual);
        if (!started) {
          ctx.moveTo(x, y);
          started = true;
        } else {
          ctx.lineTo(x, y);
        }
      }
    }
    ctx.stroke();
  }

  ctx.restore();
}

function toggleAltitudeChart() {
  const panel = document.getElementById('altitude-chart-panel');
  const icon = document.getElementById('alt-toggle-icon');
  const txt = document.getElementById('alt-toggle-text');
  if (!panel) return;

  const isCollapsed = panel.classList.toggle('collapsed');
  if (icon && txt) {
    icon.textContent = isCollapsed ? '^' : 'v';
    txt.textContent = isCollapsed ? 'SHOW' : 'HIDE';
  }

  // Trigger leaflet redraw khi co giãn
  setTimeout(() => {
    if (map) map.invalidateSize();
    if (!isCollapsed) renderAltitudeChart();
  }, 260);
}

// Bắt sự kiện chuyển tab trong đồ thị độ cao (ALT, PROFILE, ERR, V/S)
document.addEventListener('DOMContentLoaded', () => {
  const tabBtns = document.querySelectorAll('.alt-tab-btn');
  tabBtns.forEach(btn => {
    btn.addEventListener('click', function() {
      tabBtns.forEach(b => b.classList.remove('active'));
      this.classList.add('active');
    });
  });
});

// ==================== CÀI ĐẶT ĐỘ CAO CẤT CÁNH & HÀNH TRÌNH ====================
let configuredAltitude = 35.0;

function setTakeoffAltitude(val) {
  let num = parseFloat(val);
  if (isNaN(num)) num = 35.0;
  num = Math.max(5.0, Math.min(150.0, Math.round(num)));
  configuredAltitude = num;

  const input = document.getElementById('takeoff-alt-input');
  if (input) input.value = configuredAltitude;

  const display = document.getElementById('takeoff-alt-display');
  if (display) display.textContent = configuredAltitude;

  const legend = document.getElementById('legend-target-alt');
  if (legend) legend.textContent = `${configuredAltitude}m`;

  // Cập nhật trạng thái active cho các nút preset
  document.querySelectorAll('.alt-preset-chip').forEach(chip => {
    const chipAlt = parseFloat(chip.getAttribute('data-alt'));
    if (chipAlt === configuredAltitude) {
      chip.classList.add('active');
    } else {
      chip.classList.remove('active');
    }
  });

  // Cập nhật lại độ cao của các waypoint hiện có nếu chưa bay
  waypoints.forEach(w => {
    w.alt = configuredAltitude;
  });

  // Vẽ lại đồ thị độ cao với đường mục tiêu mới
  renderAltitudeChart();

  // Gửi thiết lập độ cao lên máy chủ
  fetch('/set-altitude', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ altitude: configuredAltitude })
  }).catch(() => {});
}

function stepAltitude(delta) {
  setTakeoffAltitude(configuredAltitude + delta);
}

async function startMission() {
  if (!waypoints || waypoints.length === 0) {
    alert("Chưa có điểm lộ trình nào! Vui lòng chọn ít nhất 1 điểm trên bản đồ và nhấn 'Nạp lộ trình' trước khi bắt đầu.");
    return;
  }
  if (!confirm(`⚠️ XÁC NHẬN BẮT ĐẦU BAY\n\nBạn có chắc chắn muốn UAV bắt đầu bay thực hiện nhiệm vụ ở độ cao thiết lập (${configuredAltitude}m)?\n\nUAV sẽ cất cánh hoặc duy trì độ cao ${configuredAltitude}m trong suốt hành trình qua các waypoint.`)) {
    return;
  }
  try {
    const res = await fetch('/start-mission', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ alt: configuredAltitude })
    });
    const d = await res.json();
    if (!d.success) alert(d.message);
  } catch (e) {
    console.error(e);
  }
}

async function triggerTakeoff() {
  if (!confirm(`⚠️ XÁC NHẬN CẤT CÁNH\n\nBạn có chắc chắn muốn UAV cất cánh lên độ cao thiết lập (${configuredAltitude}m)?`)) {
    return;
  }
  try {
    const res = await fetch('/takeoff', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ alt: configuredAltitude })
    });
    const d = await res.json();
    if (!d.success && d.message) alert(d.message);
  } catch (e) {
    console.error(e);
  }
}

async function stopMission() {
  try {
    await fetch('/stop-mission', { method: 'POST' });
  } catch (e) {
    console.error(e);
  }
}

async function armUAV() {
  try {
    await fetch('/arm', { method: 'POST' });
  } catch (e) {
    console.error(e);
  }
}

async function disarmUAV() {
  try {
    await fetch('/disarm', { method: 'POST' });
  } catch (e) {
    console.error(e);
  }
}

async function triggerRTL() {
  if (!confirm("⚠️ XÁC NHẬN HẠ CÁNH (RTL)\n\nBạn có chắc chắn muốn kích hoạt hạ cánh an toàn / quay về điểm xuất phát?")) {
    return;
  }
  try {
    const res = await fetch('/rtl', { method: 'POST' });
    const d = await res.json();
    if (!d.success && d.message) alert(d.message);
  } catch (e) {
    console.error(e);
  }
}

async function loadCurrentMission() {
  try {
    const res = await fetch('/get-mission');
    const d = await res.json();
    if (d.success && d.mission && d.mission.length > 0) {
      clearLocalWaypointsOnly();
      d.mission.forEach(w => addWaypoint(w.lat, w.lng || w.lon));
    }
  } catch (_) {}
}

// ==================== DỮ LIỆU ĐÃ THU THẬP & CSV ====================
async function loadCollectedData() {
  try {
    const res = await fetch('/get-collected-data');
    const d = await res.json();
    if (!d.success || !d.data) return;

    // Chỉ gán dữ liệu vào waypoint nếu tọa độ đo thực tế trùng khớp với waypoint hiện tại
    matchCollectedDataToWaypoints(d.data);

    renderCollectedDataTable(d.data);
    renderDataPointsOnMap(d.data);
    renderWaypointList();
  } catch (_) {}
}

async function clearCollectedHistory() {
  if (!confirm('Bạn có chắc chắn muốn xóa toàn bộ lịch sử các điểm đã đo?')) return;
  try {
    const res = await fetch('/wp-data/clear', { method: 'POST' });
    const d = await res.json();
    if (d.success) {
      waypoints.forEach(w => { w.measuredData = null; });
      dataPointMarkers.forEach(m => map.removeLayer(m));
      dataPointMarkers = [];
      renderCollectedDataTable([]);
      renderWaypointList();
    }
  } catch (err) {
    console.error('Lỗi khi xóa lịch sử:', err);
  }
}

async function deleteSingleCollectedPoint(filename, wpIdx) {
  if (!confirm(`Bạn có chắc chắn muốn xóa dữ liệu đo của điểm #${wpIdx + 1}?`)) return;
  try {
    const res = await fetch('/wp-data/delete', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ filename: filename, waypoint_index: wpIdx })
    });
    const d = await res.json();
    if (d.success) {
      waypoints.forEach(w => {
        if (w.measuredData && (w.measuredData.filename === filename || w.measuredData.waypoint_index === wpIdx)) {
          w.measuredData = null;
        }
      });
      loadCollectedData();
    }
  } catch (err) {
    console.error('Lỗi khi xóa điểm đo:', err);
  }
}

function renderCollectedDataTable(dataList) {
  const container = document.getElementById('collected-data-table-body');
  if (!container) return;

  if (dataList.length === 0) {
    container.innerHTML = `<tr><td colspan="6" style="text-align:center;color:var(--text-3);padding:10px;font-size:10px;">Chưa có dữ liệu đo đạc tại các điểm.</td></tr>`;
    return;
  }

  container.innerHTML = dataList.slice().reverse().map((r, i) => {
    const wpIdx = r.waypoint_index !== undefined ? r.waypoint_index : 0;
    const aqi = r.aqi || 1;
    let badgeBg = '#dcfce7';
    let badgeColor = '#15803d';
    if (aqi > 200) { badgeBg = '#fce7f3'; badgeColor = '#7e0023'; }
    else if (aqi > 150) { badgeBg = '#fee2e2'; badgeColor = '#b91c1c'; }
    else if (aqi > 100) { badgeBg = '#fee2e2'; badgeColor = '#b91c1c'; }
    else if (aqi > 50) { badgeBg = '#fef3c7'; badgeColor = '#b45309'; }

    // Rút gọn giờ sang HH:mm để tiết kiệm diện tích (ví dụ 11:28)
    const timeFormatted = r.time ? new Date(r.time).toLocaleTimeString('vi-VN', { hour: '2-digit', minute: '2-digit' }) : '--:--';

    return `
      <tr style="border-bottom:1px solid var(--line);">
        <td style="font-family:var(--mono);font-weight:700;padding:5px 2px;text-align:center;">#${wpIdx + 1}</td>
        <td class="tabular-num" style="padding:5px 2px;color:var(--text-2);font-size:9.5px;white-space:nowrap;">${timeFormatted}</td>
        <td class="tabular-num" style="padding:5px 2px;color:var(--danger);font-weight:700;white-space:nowrap;">${(r.pm25 || 0).toFixed(1)}</td>
        <td class="tabular-num" style="padding:5px 2px;white-space:nowrap;">${Math.round(r.eco2 || 0)}</td>
        <td style="padding:5px 2px;text-align:center;white-space:nowrap;">
          <span style="display:inline-block;white-space:nowrap;background:${badgeBg};color:${badgeColor};font-size:9px;padding:1px 3px;font-weight:800;border-radius:3px;border:1px solid ${badgeColor}30;letter-spacing:-0.2px;">
            AQI ${aqi}
          </span>
        </td>
        <td style="padding:5px 2px;text-align:center;white-space:nowrap;">
          <div style="display:inline-flex;align-items:center;justify-content:center;gap:3px;">
            <button class="btn-cockpit btn-outline" style="height:19px;padding:0 4px;font-size:8.5px;" onclick="panToPoint(${r.lat}, ${r.lon}, ${wpIdx})" title="Xem vị trí điểm đo">Xem</button>
            ${r.filename ? `<a href="/wp-data/download/${r.filename}" class="btn-cockpit btn-primary" style="height:19px;padding:1px 4px;font-size:8.5px;text-decoration:none;" title="Tải CSV điểm #${wpIdx + 1}">CSV</a>` : ''}
            <button onclick="deleteSingleCollectedPoint('${r.filename || ''}', ${wpIdx})" style="width:17px;height:17px;line-height:17px;font-size:13px;font-weight:700;color:var(--danger);background:transparent;border:none;cursor:pointer;padding:0;display:inline-flex;align-items:center;justify-content:center;border-radius:3px;" title="Xóa điểm đo này">×</button>
          </div>
        </td>
      </tr>
    `;
  }).join('');
}

function renderDataPointsOnMap(dataList) {
  // Xóa markers cũ
  dataPointMarkers.forEach(m => map.removeLayer(m));
  dataPointMarkers = [];

  dataList.forEach(pt => {
    const aqi = pt.aqi || 1;
    const color = aqi <= 50 ? '#157a3a' : (aqi <= 100 ? '#b25e00' : '#b42318');
    const wpIdx = pt.waypoint_index !== undefined ? pt.waypoint_index : 0;

    const icon = L.divIcon({
      className: 'data-point-node',
      html: `<div style="width:12px;height:12px;border-radius:50%;background:${color};border:2px solid #fff;box-shadow:0 1px 4px rgba(0,0,0,0.3);cursor:pointer;" title="Điểm đo #${wpIdx + 1} - Bấm xem chi tiết"></div>`,
      iconSize: [12, 12],
      iconAnchor: [6, 6]
    });

    const m = L.marker([pt.lat, pt.lon], { icon: icon }).addTo(map);
    m.on('click', (e) => {
      L.DomEvent.stopPropagation(e);
      if (wpMarkers[wpIdx]) {
        showWaypointAirQualityPopup(wpIdx, wpMarkers[wpIdx]);
      }
    });

    dataPointMarkers.push(m);
  });
}

function panToPoint(lat, lon, wpIdx) {
  if (map && lat && lon) {
    map.setView([lat, lon], 17, { animate: true });
    if (wpIdx !== undefined && wpMarkers[wpIdx]) {
      showWaypointAirQualityPopup(wpIdx, wpMarkers[wpIdx]);
    }
  }
}

// Khởi chạy khi DOM sẵn sàng
document.addEventListener('DOMContentLoaded', () => {
  initMap();
  loadCollectedData();
  setInterval(loadCollectedData, 5000);
});