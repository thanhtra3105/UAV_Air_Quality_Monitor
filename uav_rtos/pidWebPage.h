#ifndef PID_WEB_PAGE_H
#define PID_WEB_PAGE_H

#include <WebServer.h>
#include "types.h"
// ===== EXTERN OBJECTS =====
// WebServer server — defined in uav_rtos.ino
extern WebServer server;
extern SemaphoreHandle_t xFlightMutex;
extern SemaphoreHandle_t xGainsMutex;
extern FlightState gFlight;
extern PidGains gains;

// Forward declarations (tránh lỗi "not declared in this scope")
void handleRoot();
void handleSetPID();
void handleData();

// ===== HTML ROW =====
String pidRow(const char* label, const char* name, float value, int precision = 4) {
  String s = "<tr><td>" + String(label) + "</td>";
  s += "<td><input type='text' name='" + String(name) + "' value='" + String(value, precision) + "' style='width:80px'></td></tr>";
  return s;
}
void web_setup() {
  server.on("/", handleRoot);
  server.on("/setPID", handleSetPID);
  server.on("/data", handleData);
  server.begin();
}
// ===== WEB PAGE =====
void handleRoot() {
  String page = "<html><body><h2>PID Tuning</h2>";
  page += "<form action='/setPID'>";

  page += "<h3>ROLL</h3><table>";
  page += pidRow("KP", "kp_r", gains.kp_r, 4);
  page += pidRow("KI", "ki_r", gains.ki_r, 4);
  page += pidRow("KD", "kd_r", gains.kd_r, 4);
  page += "</table>";

  page += "<h3>PITCH</h3><table>";
  page += pidRow("KP", "kp_p", gains.kp_p, 4);
  page += pidRow("KI", "ki_p", gains.ki_p, 4);
  page += pidRow("KD", "kd_p", gains.kd_p, 4);
  page += "</table>";

  page += "<h3>YAW</h3><table>";
  page += pidRow("KP", "kp_y", gains.kp_y, 4);
  page += pidRow("KI", "ki_y", gains.ki_y, 4);
  page += pidRow("KD", "kd_y", gains.kd_y, 4);
  page += "</table>";

  page += "<h3>ANGLE</h3><table>";
  page += pidRow("KP", "kp_angle", gains.kp_angle, 4);
  page += pidRow("KI", "ki_angle", gains.ki_angle, 3);
  page += pidRow("KD", "kd_angle", gains.kd_angle, 3);
  page += "</table>";

  page += "<h3>VEL Z</h3><table>";
  page += pidRow("KP", "kp_vel_z", gains.kp_vel_z, 3);
  page += pidRow("KI", "ki_vel_z", gains.ki_vel_z, 3);
  page += pidRow("KD", "kd_vel_z", gains.kd_vel_z, 3);
  page += "</table>";
  
  page += "<h3>TARGET</h3><table>";
  page += pidRow("Target Pitch", "target_pitch", gFlight.target_pitch, 0);
  page += pidRow("Target Roll", "target_roll", gFlight.target_roll, 0);
  page += "</table>";

  page += "<br><input type='submit' value='Update'></form>";

  page += "<h3>Realtime Data</h3>";
  page += "<pre id='data'></pre>";

  page += R"rawliteral(
<script>
setInterval(()=>{
  fetch('/data')
    .then(r=>r.text())
    .then(t=>document.getElementById('data').innerText = t);
}, 500);
</script>
)rawliteral";

  page += "</body></html>";

  server.send(200, "text/html", page);
}

// ===== HANDLE SET PID =====
void handleSetPID() {
  PidGains newGains;

  xSemaphoreTake(xGainsMutex, portMAX_DELAY);
  newGains = gains;
  xSemaphoreGive(xGainsMutex);

  if (server.hasArg("kp_r")) newGains.kp_r = server.arg("kp_r").toFloat();
  if (server.hasArg("ki_r")) newGains.ki_r = server.arg("ki_r").toFloat();
  if (server.hasArg("kd_r")) newGains.kd_r = server.arg("kd_r").toFloat();

  if (server.hasArg("kp_p")) newGains.kp_p = server.arg("kp_p").toFloat();
  if (server.hasArg("ki_p")) newGains.ki_p = server.arg("ki_p").toFloat();
  if (server.hasArg("kd_p")) newGains.kd_p = server.arg("kd_p").toFloat();

  if (server.hasArg("kp_y")) newGains.kp_y = server.arg("kp_y").toFloat();
  if (server.hasArg("ki_y")) newGains.ki_y = server.arg("ki_y").toFloat();
  if (server.hasArg("kd_y")) newGains.kd_y = server.arg("kd_y").toFloat();

  if (server.hasArg("kp_angle")) newGains.kp_angle = server.arg("kp_angle").toFloat();
  if (server.hasArg("ki_angle")) newGains.ki_angle = server.arg("ki_angle").toFloat();
  if (server.hasArg("kd_angle")) newGains.kd_angle = server.arg("kd_angle").toFloat();

  if (server.hasArg("kp_vel_z")) newGains.kp_vel_z = server.arg("kp_vel_z").toFloat();
  if (server.hasArg("ki_vel_z")) newGains.ki_vel_z = server.arg("ki_vel_z").toFloat();
  if (server.hasArg("kd_vel_z")) newGains.kd_vel_z = server.arg("kd_vel_z").toFloat();

  xSemaphoreTake(xGainsMutex, portMAX_DELAY);
  gains = newGains;
  xSemaphoreGive(xGainsMutex);

  xSemaphoreTake(xFlightMutex, portMAX_DELAY);
  if (server.hasArg("target_pitch")) gFlight.target_pitch = server.arg("target_pitch").toFloat();
  if (server.hasArg("target_roll"))  gFlight.target_roll  = server.arg("target_roll").toFloat();
  xSemaphoreGive(xFlightMutex);

  server.sendHeader("Location", "/");
  server.send(303);
}
// ===== REALTIME DATA =====
void handleData() {
  FlightState fs;

  xSemaphoreTake(xFlightMutex, portMAX_DELAY);
  fs = gFlight;
  xSemaphoreGive(xFlightMutex);

  String json = "{";
  json += "\"pitch\":" + String(fs.pitch) + ",";
  json += "\"roll\":" + String(fs.roll) + ",";
  json += "\"yaw\":" + String(fs.yaw) + ",";
  json += "\"throttle\":" + String(fs.throttle) + ",";
  json += "\"alt\":" + String(fs.AltitudeKalman) + ",";
  json += "\"vel_z\":" + String(fs.VelocityVerticalKalman) + ",";
  json += "\"pid_vel\":" + String(fs.pid_vel) + ",";
  json += "\"acc_z\":" + String(fs.acc_z_inertial);
  json += "}";

  server.send(200, "application/json", json);
}

#endif