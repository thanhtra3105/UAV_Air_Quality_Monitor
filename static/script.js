let map, vehicleMarker;
const waypoints = [];
const wpMarkers = [];
let pathLine = null;
let vehicleTrail = null;
let currentHeading = 0;
let dataPointsMarkers = []; // Lưu các marker điểm dữ liệu
let pollingInterval = null;

const mapLayers = {
  roadmap: L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { maxZoom: 19, attribution: '© OpenStreetMap' }),
  satellite: L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', { maxZoom: 19, attribution: '© Esri' }),
  terrain: L.tileLayer('https://{s}.tile.opentopomap.org/{z}/{x}/{y}.png', { maxZoom: 17, attribution: '© OpenTopoMap' })
};
let currentLayerName = 'roadmap';

function log(msg, type = '') {
  const box = document.getElementById('log');
  if (!box) return;
  const el = document.createElement('div');
  el.className = 'log-line ' + type;
  const ts = new Date().toLocaleTimeString('vi-VN', { hour: '2-digit', minute: '2-digit', second: '2-digit' });
  el.textContent = `[${ts}] ${msg}`;
  box.prepend(el);
  while (box.children.length > 50) box.removeChild(box.lastChild);
}

function getDroneIcon(heading) {
  // Tạo SVG cho drone với hướng quay
  const svgHtml = `
    <svg viewBox="0 0 40 40" width="40" height="40" xmlns="http://www.w3.org/2000/svg">
      <defs>
        <filter id="glow" x="-20%" y="-20%" width="140%" height="140%">
          <feGaussianBlur stdDeviation="2" result="blur" />
          <feMerge>
            <feMergeNode in="blur" />
            <feMergeNode in="SourceGraphic" />
          </feMerge>
        </filter>
      </defs>
      <!-- Thân chính -->
      <circle cx="20" cy="20" r="8" fill="#3B82F6" stroke="#fff" stroke-width="1.5" filter="url(#glow)"/>
      <!-- Cánh trước -->
      <line x1="20" y1="8" x2="20" y2="2" stroke="#3B82F6" stroke-width="2.5" stroke-linecap="round"/>
      <line x1="20" y1="8" x2="26" y2="5" stroke="#3B82F6" stroke-width="2" stroke-linecap="round"/>
      <line x1="20" y1="8" x2="14" y2="5" stroke="#3B82F6" stroke-width="2" stroke-linecap="round"/>
      <!-- Cánh sau -->
      <line x1="20" y1="32" x2="20" y2="38" stroke="#3B82F6" stroke-width="2.5" stroke-linecap="round"/>
      <line x1="20" y1="32" x2="26" y2="35" stroke="#3B82F6" stroke-width="2" stroke-linecap="round"/>
      <line x1="20" y1="32" x2="14" y2="35" stroke="#3B82F6" stroke-width="2" stroke-linecap="round"/>
      <!-- Cánh trái -->
      <line x1="8" y1="20" x2="2" y2="20" stroke="#3B82F6" stroke-width="2.5" stroke-linecap="round"/>
      <line x1="8" y1="20" x2="5" y2="14" stroke="#3B82F6" stroke-width="2" stroke-linecap="round"/>
      <line x1="8" y1="20" x2="5" y2="26" stroke="#3B82F6" stroke-width="2" stroke-linecap="round"/>
      <!-- Cánh phải -->
      <line x1="32" y1="20" x2="38" y2="20" stroke="#3B82F6" stroke-width="2.5" stroke-linecap="round"/>
      <line x1="32" y1="20" x2="35" y2="14" stroke="#3B82F6" stroke-width="2" stroke-linecap="round"/>
      <line x1="32" y1="20" x2="35" y2="26" stroke="#3B82F6" stroke-width="2" stroke-linecap="round"/>
      <!-- Chấm trung tâm -->
      <circle cx="20" cy="20" r="2" fill="#fff"/>
    </svg>
  `;
  
  return L.divIcon({
    className: 'drone-icon',
    html: `<div style="transform: rotate(${heading}deg); transition: transform 0.2s ease; width: 40px; height: 40px;">${svgHtml}</div>`,
    iconSize: [40, 40],
    iconAnchor: [20, 20],
    popupAnchor: [0, -20]
  });
}

function getWpIcon(num, isActive) {
  const bg = isActive ? '#F59E0B' : '#3B82F6';
  return L.divIcon({
    className: 'wp-icon-wrap',
    html: `<div style="background: ${bg}; color: #fff; width: 26px; height: 26px; border-radius: 50%; border: 2px solid #fff; display: flex; align-items: center; justify-content: center; font-family: 'Roboto Mono', monospace; font-size: 13px; font-weight: bold; box-shadow: 0 2px 5px rgba(0,0,0,0.2); transition: background 0.3s;">
            ${num}
           </div>`,
    iconSize: [26, 26], iconAnchor: [13, 13]
  });
}

function getDataPointIcon(aqi) {
  let color = '#10B981';
  if (aqi === 2) color = '#84CC16';
  else if (aqi === 3) color = '#F59E0B';
  else if (aqi === 4) color = '#F97316';
  else if (aqi === 5) color = '#EF4444';
  
  return L.divIcon({
    className: 'data-point-marker',
    html: `<div style="background: ${color}; width: 14px; height: 14px; border-radius: 50%; border: 2px solid white; box-shadow: 0 2px 4px rgba(0,0,0,0.3);"></div>`,
    iconSize: [14, 14], iconAnchor: [7, 7]
  });
}

async function initMap() {
  let center = [16.074353668716064, 108.15225143177362];
  try {
    const pos = await fetch('/vehicle-position').then(r => r.json());
    if (pos.success) center = [pos.lat, pos.lon];
  } catch (_) { }

  map = L.map('map', { zoomControl: false }).setView(center, 17);
  mapLayers.roadmap.addTo(map);
  L.control.zoom({ position: 'bottomright' }).addTo(map);
  
  // Tạo marker UAV với icon drone
  vehicleMarker = L.marker(center, { 
    icon: getDroneIcon(0), 
    zIndexOffset: 1000,
    title: 'UAV Position'
  }).addTo(map);
  
  // Thêm popup cho UAV
  vehicleMarker.bindPopup('<b>🚁 UAV</b><br>Đang hoạt động', { offset: [0, -20] });
  
  vehicleTrail = L.polyline([], { 
    color: '#3B82F6', 
    weight: 3, 
    opacity: 0.6, 
    dashArray: '5, 8',
    lineCap: 'round'
  }).addTo(map);

  map.on('click', e => addWaypoint(e.latlng.lat, e.latlng.lng));

  initDataLoggingToggle();
  loadCollectedData();
  monitorHoldStatus();

  setInterval(updatePosition, 1000);
  setInterval(updateVehicleInfo, 1500);
  setInterval(updateMissionProgress, 1700);
  setInterval(loadCollectedData, 5000);
}

function initDataLoggingToggle() {
  const toggle = document.getElementById('data-logging-toggle');
  if (toggle) {
    toggle.addEventListener('change', async function() {
      try {
        const response = await fetch('/set-data-logging', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ enabled: this.checked })
        });
        const data = await response.json();
        log(`Ghi dữ liệu cảm biến: ${data.enabled ? 'BẬT' : 'TẮT'}`, data.enabled ? 'ok' : 'warn');
      } catch (err) {
        console.error(err);
      }
    });
    
    // Lấy trạng thái hiện tại
    fetch('/get-data-logging')
      .then(r => r.json())
      .then(data => {
        toggle.checked = data.enabled;
      });
  }
}

async function monitorHoldStatus() {
  setInterval(async () => {
    try {
      const tele = await fetch('/api/telemetry').then(r => r.json());
      if (tele.success && tele.telemetry.is_holding) {
        const holdDiv = document.getElementById('hold-progress');
        const holdBar = document.getElementById('hold-bar');
        const holdCounter = document.getElementById('hold-counter');
        const holdStatus = document.getElementById('hold-status');
        
        holdDiv.classList.add('active');
        const samples = tele.telemetry.hold_samples || 0;
        const total = tele.telemetry.hold_total || 25;
        const percent = (samples / total) * 100;
        holdBar.style.width = percent + '%';
        holdCounter.textContent = `${samples}/${total}`;
        holdStatus.textContent = `Đang thu thập dữ liệu không khí...`;
        
        if (samples >= total) {
          setTimeout(() => holdDiv.classList.remove('active'), 3000);
        }
      } else {
        document.getElementById('hold-progress').classList.remove('active');
      }
    } catch (_) {}
  }, 1000);
}

async function loadCollectedData() {
  try {
    const response = await fetch('/get-collected-data');
    const data = await response.json();
    
    if (data.success && data.data) {
      // Cập nhật danh sách hiển thị
      updateDataList(data.data);
      
      // Cập nhật marker trên bản đồ
      updateDataMarkers(data.data);
      
      // Cập nhật số lượng
      document.getElementById('data-count').textContent = `(${data.data.length})`;
    }
  } catch (err) {
    console.error('Lỗi load dữ liệu:', err);
  }
}

function updateDataList(dataPoints) {
  const container = document.getElementById('data-list');
  if (!container) return;
  
  if (!dataPoints || dataPoints.length === 0) {
    container.innerHTML = '<div style="text-align:center;color:var(--txt3);padding:20px;">Chưa có dữ liệu</div>';
    return;
  }
  
  container.innerHTML = dataPoints.slice().reverse().map(point => {
    const date = new Date(point.time);
    const timeStr = date.toLocaleTimeString('vi-VN');
    const aqiClass = `aqi-${point.aqi || 3}`;
    
    return `
      <div class="data-item ${aqiClass}" onclick="flyToDataPoint(${point.lat}, ${point.lon})">
        <div style="display: flex; justify-content: space-between;">
          <strong>WP${point.waypoint_index + 1}</strong>
          <span style="font-size: 0.65rem;">${timeStr}</span>
        </div>
        <div style="display: flex; gap: 12px; margin-top: 4px; font-size: 0.7rem;">
          <span>📍 ${point.lat.toFixed(5)}, ${point.lon.toFixed(5)}</span>
          <span>⬆️ ${point.alt.toFixed(0)}m</span>
        </div>
        <div style="display: flex; gap: 8px; margin-top: 4px; flex-wrap: wrap;">
          <span style="background: #EF4444; padding: 2px 6px; border-radius: 4px;">PM2.5: ${point.pm25.toFixed(1)}</span>
          <span style="background: #F59E0B; padding: 2px 6px; border-radius: 4px;">CO₂: ${point.eco2.toFixed(0)}</span>
          <span style="background: #8B5CF6; padding: 2px 6px; border-radius: 4px;">AQI: ${point.aqi}</span>
        </div>
        <div style="display: flex; gap: 8px; margin-top: 4px; font-size: 0.65rem; color: var(--txt3);">
          <span>🌡️ ${point.temp.toFixed(1)}°C</span>
          <span>💧 ${point.hum.toFixed(1)}%</span>
          <span>📊 ${point.sample_count || 0} mẫu</span>
        </div>
      </div>
    `;
  }).join('');
}

function updateDataMarkers(dataPoints) {
  // Xóa marker cũ
  dataPointsMarkers.forEach(marker => map.removeLayer(marker));
  dataPointsMarkers = [];
  
  // Thêm marker mới
  dataPoints.forEach(point => {
    const popupContent = `
      <div style="font-family: 'Inter', sans-serif; min-width: 200px;">
        <strong>📍 Điểm đo #${point.waypoint_index + 1}</strong><br>
        <small>${new Date(point.time).toLocaleString('vi-VN')}</small><br>
        <hr style="margin: 8px 0;">
        <table style="width: 100%; font-size: 12px;">
          <tr><td>🌍 Vị trí:</td><td>${point.lat.toFixed(6)}, ${point.lon.toFixed(6)}</td></tr>
          <tr><td>⬆️ Độ cao:</td><td>${point.alt.toFixed(1)} m</td></tr>
          <tr><td>🌫️ PM2.5:</td><td><b>${point.pm25.toFixed(1)} µg/m³</b></td></tr>
          <tr><td>💨 eCO₂:</td><td><b>${point.eco2.toFixed(0)} ppm</b></td></tr>
          <tr><td>🧪 TVOC:</td><td><b>${point.tvoc.toFixed(0)} ppb</b></td></tr>
          <tr><td>🌡️ Nhiệt độ:</td><td>${point.temp.toFixed(1)} °C</td></tr>
          <tr><td>💧 Độ ẩm:</td><td>${point.hum.toFixed(1)} %</td></tr>
          <tr><td>📊 AQI:</td><td><b style="color: ${point.aqi <= 2 ? '#10B981' : (point.aqi <= 3 ? '#F59E0B' : '#EF4444')}">${point.aqi}</b></td></tr>
        </table>
        <hr style="margin: 8px 0;">
        <small>📊 ${point.sample_count || 0} lần đo</small>
      </div>
    `;
    
    const marker = L.marker([point.lat, point.lon], {
      icon: getDataPointIcon(point.aqi),
      zIndexOffset: 500
    }).bindPopup(popupContent);
    
    marker.addTo(map);
    dataPointsMarkers.push(marker);
  });
}

function flyToDataPoint(lat, lon) {
  map.flyTo([lat, lon], 18, { duration: 1.5 });
  log(`Đã di chuyển đến điểm dữ liệu tại (${lat.toFixed(5)}, ${lon.toFixed(5)})`, 'info');
}

function centerToUAV() {
  if (vehicleMarker) {
    const currentLatLng = vehicleMarker.getLatLng();
    map.flyTo(currentLatLng, 18, {
      animate: true,
      duration: 1.0,
      easeLinearity: 0.5
    });
    log('🎯 Đã căn giữa bản đồ vào vị trí UAV', 'ok');
  } else {
    log('⚠️ Chưa có dữ liệu vị trí UAV', 'warn');
  }
}

function setMapType(type, btn) {
  map.removeLayer(mapLayers[currentLayerName]);
  mapLayers[type].addTo(map);
  currentLayerName = type;
  document.querySelectorAll('.map-type-btn').forEach(b => b.classList.remove('active'));
  btn.classList.add('active');
}

async function updatePosition() {
  try {
    const d = await fetch('/vehicle-position').then(r => r.json());
    if (!d.success) return;
    const latlng = [d.lat, d.lon];
    vehicleMarker.setLatLng(latlng);
    vehicleTrail.addLatLng(latlng);
    if (vehicleTrail.getLatLngs().length > 100) {
      const arr = vehicleTrail.getLatLngs(); arr.shift(); vehicleTrail.setLatLngs(arr);
    }
  } catch (_) { }
}

async function updateVehicleInfo() {
  try {
    const d = await fetch('/vehicle-info').then(r => r.json());
    if (!d.success) return;
    document.getElementById('spd-val').textContent = d.speed.toFixed(1) + ' m/s';
    document.getElementById('hdg-val').textContent = Math.round(d.heading) + '°';
    document.getElementById('alt-val').textContent = d.alt.toFixed(1) + ' m';
    currentHeading = d.heading;
    // Cập nhật icon drone với hướng mới
    vehicleMarker.setIcon(getDroneIcon(currentHeading));
  } catch (_) { }
}

async function updateMissionProgress() {
  try {
    const d = await fetch('/mission-progress').then(r => r.json());
    if (!d.success) return;
    const total = d.mission_total || 0;
    const cur   = d.mission_current || 0;
    document.getElementById('ms-text').textContent = total ? `WP ${cur} / ${total}` : '-- / --';
    document.getElementById('ms-dot').className = 'ms-dot ' + (d.mission_state === 3 ? 'running' : d.mission_state === 5 ? 'done' : 'idle');

    wpMarkers.forEach((m, i) => m.setIcon(getWpIcon(i + 1, (i === cur && d.mission_state === 3))));
  } catch (_) { }
}

function addWaypoint(lat, lng) {
  const idx = waypoints.length;
  waypoints.push({ lat, lng, hold_time: 3, alt: 30 });
  
  const marker = L.marker([lat, lng], { icon: getWpIcon(idx + 1, false) }).addTo(map);
  wpMarkers.push(marker);

  updatePathLine();
  renderWpList();
  log(`WP ${idx + 1} added`, 'ok');
}

function updatePathLine() {
  if (pathLine) map.removeLayer(pathLine);
  if (waypoints.length < 2) return;
  const latlngs = waypoints.map(w => [w.lat, w.lng]);
  pathLine = L.polyline(latlngs, { color: '#F59E0B', weight: 2, dashArray: '8, 6', opacity: 0.8 }).addTo(map);
}

function renderWpList() {
  const wrap = document.getElementById('wp-list-wrap');
  wrap.querySelectorAll('.wp-item').forEach(el => el.remove());
  document.getElementById('wp-empty').style.display = waypoints.length ? 'none' : 'block';

  waypoints.forEach((wp, i) => {
    const el = document.createElement('div');
    el.className = 'wp-item';
    el.innerHTML = `
      <span class="wp-num">${i + 1}</span>
      <span class="wp-coords">${wp.lat.toFixed(5)}<br>${wp.lng.toFixed(5)}</span>
      <input type="number" class="wp-alt" value="${wp.alt || 30}" style="width: 55px; background: var(--surface); border: 1px solid var(--border); border-radius: 4px; padding: 4px;" placeholder="Alt m" onchange="updateWaypointAlt(${i}, this.value)">
      <button class="wp-remove" onclick="removeWaypoint(${i})" title="Xóa">×</button>
    `;
    wrap.appendChild(el);
  });
}

function updateWaypointAlt(idx, value) {
  waypoints[idx].alt = parseFloat(value) || 30;
  log(`WP${idx + 1} độ cao: ${waypoints[idx].alt}m`, 'info');
}

function removeWaypoint(idx) {
  waypoints.splice(idx, 1);
  map.removeLayer(wpMarkers[idx]);
  wpMarkers.splice(idx, 1);
  wpMarkers.forEach((m, i) => m.setIcon(getWpIcon(i + 1, false)));
  updatePathLine();
  renderWpList();
}

async function uploadMission() {
  if (!waypoints.length) {
    log('❌ Không có waypoint để upload', 'err');
    return;
  }
  
  const bar = document.getElementById('upload-bar');
  bar.style.width = '0%';
  bar.className = 'progress-bar';
  
  log(`📤 Đang upload ${waypoints.length} waypoints lên UAV...`, 'info');
  
  let prog = 0;
  const interval = setInterval(() => {
    prog = Math.min(prog + 4, 85);
    bar.style.width = prog + '%';
  }, 80);
  
  try {
    const response = await fetch('/upload-mission', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ mission: waypoints })
    });
    const data = await response.json();
    
    clearInterval(interval);
    
    if (data.success) {
      bar.style.width = '100%';
      bar.className = 'progress-bar success';
      log(`✅ ${data.message || 'Upload thành công!'}`, 'ok');
      setTimeout(() => { bar.style.width = '0%'; }, 2000);
    } else {
      bar.className = 'progress-bar fail';
      log(`❌ Upload thất bại: ${data.message || 'Lỗi không xác định'}`, 'err');
    }
  } catch (error) {
    clearInterval(interval);
    bar.className = 'progress-bar fail';
    log(`❌ Lỗi kết nối: ${error.message}`, 'err');
    console.error('Upload error:', error);
  }
}

async function startMission() {
  if (!current_waypoints.length) {
    log('❌ Chưa có waypoint nào được upload! Hãy upload waypoint trước.', 'err');
    return;
  }
  
  log('🚀 Đang gửi lệnh bắt đầu mission...', 'info');
  
  try {
    const response = await fetch('/start-mission', { method: 'POST' });
    const data = await response.json();
    
    if (data.success) {
      log(`✅ ${data.message || 'Mission đã bắt đầu! UAV đang di chuyển đến waypoint đầu tiên.'}`, 'ok');
    } else {
      log(`❌ ${data.message || 'Không thể bắt đầu mission'}`, 'err');
    }
  } catch (error) {
    log(`❌ Lỗi: ${error.message}`, 'err');
    console.error('Start mission error:', error);
  }
}

async function stopMission() {
  log('⏸️ Đang dừng mission...', 'info');
  
  try {
    const response = await fetch('/stop-mission', { method: 'POST' });
    const data = await response.json();
    
    if (data.success) {
      log(`✅ ${data.message || 'Mission đã dừng'}`);
    } else {
      log(`❌ ${data.message || 'Không thể dừng mission'}`, 'err');
    }
  } catch (error) {
    log(`❌ Lỗi: ${error.message}`, 'err');
  }
}

function clearWaypoints() {
  waypoints.length = 0;
  wpMarkers.forEach(m => map.removeLayer(m));
  wpMarkers.length = 0;
  if (pathLine) { map.removeLayer(pathLine); pathLine = null; }
  renderWpList();
  log('Đã xóa toàn bộ Waypoints', 'warn');
}

async function uploadMission() {
  if (!waypoints.length) return log('Không có WP để upload', 'warn');
  const bar = document.getElementById('upload-bar');
  bar.style.width = '0%'; bar.className = 'progress-bar';
  let prog = 0; const interval = setInterval(() => { prog = Math.min(prog + 4, 85); bar.style.width = prog + '%'; }, 80);
  try {
    const r = await fetch('/upload-mission', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ mission: waypoints }) });
    const d = await r.json();
    clearInterval(interval); bar.style.width = '100%'; bar.className = 'progress-bar ' + (d.success ? 'success' : 'fail');
    log(d.message || (d.success ? 'Upload OK' : 'Failed'), d.success ? 'ok' : 'err');
    setTimeout(() => { bar.style.width = '0%'; }, 2000);
  } catch (e) { clearInterval(interval); bar.className = 'progress-bar fail'; log('Error', 'err'); }
}

async function startMission() { 
  const r = await fetch('/start-mission', { method: 'POST' });
  const d = await r.json();
  log(d.message || 'Start Mission Request Sent', 'ok');
}

async function stopMission() {
  const r = await fetch('/stop-mission', { method: 'POST' });
  const d = await r.json();
  log(d.message || 'Mission Stopped', 'warn');
}

async function loadMissionFromVehicle() { }

async function armVehicle() { 
  const r = await fetch('/arm', { method: 'POST' });
  log('Arming...', 'info');
}

async function disarmVehicle() { 
  const r = await fetch('/disarm', { method: 'POST' });
  log('Disarming...', 'info');
}

async function rtl() { 
  const r = await fetch('/rtl', { method: 'POST' });
  log('Return To Launch...', 'info');
}

async function exportDataCSV() {
  window.open('/export-csv', '_blank');
  log('Đang xuất file CSV...', 'info');
}

async function exportDataJSON() {
  try {
    const r = await fetch('/export-json');
    const d = await r.json();
    if (d.success) {
      const dataStr = JSON.stringify(d.data, null, 2);
      const blob = new Blob([dataStr], { type: 'application/json' });
      const url = URL.createObjectURL(blob);
      const a = document.createElement('a');
      a.href = url;
      a.download = `uav_data_${new Date().toISOString().slice(0,19)}.json`;
      a.click();
      URL.revokeObjectURL(url);
      log('Xuất JSON thành công', 'ok');
    }
  } catch (err) {
    log('Lỗi xuất JSON', 'err');
  }
}

// Hàm toàn cục cho onclick
window.flyToDataPoint = flyToDataPoint;
window.updateWaypointAlt = updateWaypointAlt;
window.removeWaypoint = removeWaypoint;
window.clearWaypoints = clearWaypoints;
window.uploadMission = uploadMission;
window.startMission = startMission;
window.stopMission = stopMission;
window.armVehicle = armVehicle;
window.disarmVehicle = disarmVehicle;
window.rtl = rtl;
window.setMapType = setMapType;
window.centerToUAV = centerToUAV;
window.exportDataCSV = exportDataCSV;
window.exportDataJSON = exportDataJSON;