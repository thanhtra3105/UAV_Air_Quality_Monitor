#include <WiFi.h>
#include <WebServer.h>
#include "pid_control.h"
WebServer server(80);

extern float kp_r, kp_p, kp_yaw;
extern float ki_r, ki_p, ki_yaw;
extern float kd_r, kd_p, kd_yaw;
extern float kp_angle, ki_angle, kd_angle;
extern float target_pitch, target_roll;
extern float danh_lai;
extern int throttle;
extern float AltitudeKalman;
extern float VelocityVerticalKalman;
extern float kp_alt_hold;
extern float ki_alt_hold;
extern float kd_alt_hold;
extern PIDController altPID;

String pidPage();
void handleRoot();
void handleSetPID();
void handleData();

void web_setup() {
  server.on("/", handleRoot);
  server.on("/set", handleSetPID);
  server.on("/data", handleData);
  server.begin();
}

String pidPage() {
  String page = "<html><body>";
  page += "<h2>DRONE PID TUNING</h2>";

  // ===== JavaScript =====
  page += "<script>";
  page += "function inc(id, step, fix){";
  page += "  var e=document.getElementById(id);";
  page += "  e.value=(parseFloat(e.value)+step).toFixed(fix);";
  page += "}";
  page += "function dec(id, step, fix){";
  page += "  var e=document.getElementById(id);";
  page += "  e.value=(parseFloat(e.value)-step).toFixed(fix);";
  page += "}";

  page += "function updateData(){";
  page += " fetch('/data').then(r=>r.json()).then(d=>{";
  page += "   document.getElementById('thr').innerText=d.throttle;";
  page += "   document.getElementById('alt').innerText=d.alt.toFixed(2);";
  page += "   document.getElementById('vel').innerText=d.vel.toFixed(2);";
  page += " });";
  page += "}";

  page += "setInterval(updateData,200);";

  page += "</script>";

  page += "<h3>LIVE DATA</h3>";
  page += "Throttle: <span id='thr'>0</span><br>";
  page += "Altitude: <span id='alt'>0</span><br>";
  page += "Velocity Z: <span id='vel'>0</span><br><br>";

  page += "<form action='/set' method='GET'>";
  // ===== PITCH =====
  page += "<h3>PITCH</h3>";

  page += "KP_P ";
  page += "<button type='button' onclick='dec(\"kp_p\",0.01,2)'>-</button>";
  page += "<input id='kp_p' name='kp_p' value='" + String(kp_p, 2) + "' size='6'>";
  page += "<button type='button' onclick='inc(\"kp_p\",0.01,2)'>+</button><br>";

  page += "KI_P ";
  page += "<button type='button' onclick='dec(\"ki_p\",0.001,3)'>-</button>";
  page += "<input id='ki_p' name='ki_p' value='" + String(ki_p, 4) + "' size='6'>";
  page += "<button type='button' onclick='inc(\"ki_p\",0.001,4)'>+</button><br>";

  page += "KD_P ";
  page += "<button type='button' onclick='dec(\"kd_p\",0.001,4)'>-</button>";
  page += "<input id='kd_p' name='kd_p' value='" + String(kd_p, 4) + "' size='6'>";
  page += "<button type='button' onclick='inc(\"kd_p\",0.001,4)'>+</button><br>";

  // ===== ROLL =====
  page += "<h3>ROLL</h3>";

  page += "KP_R ";
  page += "<button type='button' onclick='dec(\"kp_r\",0.01,2)'>-</button>";
  page += "<input id='kp_r' name='kp_r' value='" + String(kp_r, 2) + "' size='6'>";
  page += "<button type='button' onclick='inc(\"kp_r\",0.01,2)'>+</button><br>";

  page += "KI_R ";
  page += "<button type='button' onclick='dec(\"ki_r\",0.001,4)'>-</button>";
  page += "<input id='ki_r' name='ki_r' value='" + String(ki_r, 4) + "' size='6'>";
  page += "<button type='button' onclick='inc(\"ki_r\",0.001,4)'>+</button><br>";

  page += "KD_R ";
  page += "<button type='button' onclick='dec(\"kd_r\",0.001,4)'>-</button>";
  page += "<input id='kd_r' name='kd_r' value='" + String(kd_r, 4) + "' size='6'>";
  page += "<button type='button' onclick='inc(\"kd_r\",0.001,4)'>+</button><br>";

  // ===== YAW =====
  page += "<h3>YAW</h3>";

  page += "KP_Y ";
  page += "<button type='button' onclick='dec(\"kp_yaw\",0.01,2)'>-</button>";
  page += "<input id='kp_yaw' name='kp_yaw' value='" + String(kp_yaw, 2) + "' size='6'>";
  page += "<button type='button' onclick='inc(\"kp_yaw\",0.01,2)'>+</button><br>";

  page += "KI_Y ";
  page += "<button type='button' onclick='dec(\"ki_yaw\",0.001,3)'>-</button>";
  page += "<input id='ki_yaw' name='ki_yaw' value='" + String(ki_yaw, 3) + "' size='6'>";
  page += "<button type='button' onclick='inc(\"ki_yaw\",0.001,3)'>+</button><br>";

  page += "KD_Y ";
  page += "<button type='button' onclick='dec(\"kd_yaw\",0.001,3)'>-</button>";
  page += "<input id='kd_yaw' name='kd_yaw' value='" + String(kd_yaw, 3) + "' size='6'>";
  page += "<button type='button' onclick='inc(\"kd_yaw\",0.001,3)'>+</button><br><br>";

  page += "<h3>ALT HOLD PID</h3>";

  page += "KP_ALT ";
  page += "<button type='button' onclick='dec(\"kp_alt\",0.1,2)'>-</button>";
  page += "<input id='kp_alt' name='kp_alt' value='" + String(kp_alt_hold, 2) + "' size='6'>";
  page += "<button type='button' onclick='inc(\"kp_alt\",0.1,2)'>+</button><br>";

  page += "KI_ALT ";
  page += "<button type='button' onclick='dec(\"ki_alt\",0.01,3)'>-</button>";
  page += "<input id='ki_alt' name='ki_alt' value='" + String(ki_alt_hold, 3) + "' size='6'>";
  page += "<button type='button' onclick='inc(\"ki_alt\",0.01,3)'>+</button><br>";

  page += "KD_ALT ";
  page += "<button type='button' onclick='dec(\"kd_alt\",0.01,3)'>-</button>";
  page += "<input id='kd_alt' name='kd_alt' value='" + String(kd_alt_hold, 3) + "' size='6'>";
  page += "<button type='button' onclick='inc(\"kd_alt\",0.01,3)'>+</button><br><br>";

  page += "<b>TUNING KP ANGLE </b><br>";
  page += "<button type='button' onclick='dec(\"kp_angle\",0.01,2)'>-</button>";
  page += "<input id='kp_angle' name='kp_angle' value='" + String(kp_angle, 2) + "' size='6'>";
  page += "<button type='button' onclick='inc(\"kp_angle\",0.01,2)'>+</button><br><br>";

  page += "<b>TUNING KI ANGLE </b><br>";
  page += "<button type='button' onclick='dec(\"ki_angle\",0.001,3)'>-</button>";
  page += "<input id='ki_angle' name='ki_angle' value='" + String(ki_angle, 3) + "' size='6'>";
  page += "<button type='button' onclick='inc(\"ki_angle\",0.001,3)'>+</button><br><br>";

  page += "<b>TUNING KD ANGLE </b><br>";
  page += "<button type='button' onclick='dec(\"kd_angle\",0.001,3)'>-</button>";
  page += "<input id='kd_angle' name='kd_angle' value='" + String(kd_angle, 3) + "' size='6'>";
  page += "<button type='button' onclick='inc(\"kd_angle\",0.001,3)'>+</button><br><br>";

  page += "<b>TARGET PITCH</b><br>";
  page += "<button type='button' onclick='dec(\"target_pitch\",1.0,1)'>-</button>";
  page += "<input id='target_pitch' name='target_pitch' value='" + String(target_pitch, 1) + "' size='6'>";
  page += "<button type='button' onclick='inc(\"target_pitch\",1.0,1)'>+</button><br><br>";

  page += "<b>TARGET ROLL</b><br>";
  page += "<button type='button' onclick='dec(\"target_roll\",1.0,1)'>-</button>";
  page += "<input id='target_roll' name='target_roll' value='" + String(target_roll, 1) + "' size='6'>";
  page += "<button type='button' onclick='inc(\"target_roll\",1.0,1)'>+</button><br><br>";

  // ===== DANH LAI =====
  page += "<b>DANH LAI</b><br>";
  page += "<button type='button' onclick='dec(\"danh_lai\",1,1)'>-</button>";
  page += "<input id='danh_lai' name='danh_lai' value='" + String(danh_lai, 2) + "' size='6'>";
  page += "<button type='button' onclick='inc(\"danh_lai\",1,1)'>+</button><br><br>";

  page += "<input type='submit' value='UPDATE PID'>";
  page += "</form></body></html>";

  return page;
}


void handleRoot() {
  server.send(200, "text/html", pidPage());
}

void handleSetPID() {
  if (server.hasArg("kp_p")) kp_p = server.arg("kp_p").toFloat();
  if (server.hasArg("ki_p")) ki_p = server.arg("ki_p").toFloat();
  if (server.hasArg("kd_p")) kd_p = server.arg("kd_p").toFloat();

  if (server.hasArg("kp_r")) kp_r = server.arg("kp_r").toFloat();
  if (server.hasArg("ki_r")) ki_r = server.arg("ki_r").toFloat();
  if (server.hasArg("kd_r")) kd_r = server.arg("kd_r").toFloat();

  if (server.hasArg("kp_yaw")) kp_yaw = server.arg("kp_yaw").toFloat();
  if (server.hasArg("ki_yaw")) ki_yaw = server.arg("ki_yaw").toFloat();
  if (server.hasArg("kd_yaw")) kd_yaw = server.arg("kd_yaw").toFloat();

  if (server.hasArg("kp_angle")) kp_angle = server.arg("kp_angle").toFloat();
  if (server.hasArg("ki_angle")) ki_angle = server.arg("ki_angle").toFloat();
  // if (server.hasArg("kd_yaw")) kd_yaw = server.arg("kd_yaw").toFloat();

  if (server.hasArg("target_pitch")) target_pitch = server.arg("target_pitch").toFloat();
  if (server.hasArg("target_roll")) target_roll = server.arg("target_roll").toFloat();

  if (server.hasArg("kp_alt"))
    kp_alt_hold = server.arg("kp_alt").toFloat();

  if (server.hasArg("ki_alt"))
    ki_alt_hold = server.arg("ki_alt").toFloat();

  if (server.hasArg("kd_alt"))
    kd_alt_hold = server.arg("kd_alt").toFloat();

  if (server.hasArg("danh_lai")) {
    danh_lai = server.arg("danh_lai").toFloat();
  }

  altPID.set(
      kp_alt_hold,
      ki_alt_hold,
      kd_alt_hold
  );

  server.send(200, "text/html",
              "<h3>PID Updated!</h3><a href='/'>Back</a>");
  Serial.println("Updated PID value");
  Serial.println("Kp_r = " + String(kp_r, 3));
  Serial.println("Ki_r = " + String(ki_r, 3));
  Serial.println("Kd_r = " + String(kd_r, 3));
  Serial.println("Kp_p = " + String(kp_p, 3));
  Serial.println("Ki_p = " + String(ki_p, 3));
  Serial.println("Kd_p = " + String(kd_p, 3));
  Serial.println("Kp_y = " + String(kp_yaw, 3));
  Serial.println("Ki_y = " + String(ki_yaw, 3));
  Serial.println("Kd_y = " + String(kd_yaw, 3));
  Serial.println("Kp_angle = " + String(kp_angle, 3));
  Serial.println("Ki_angle = " + String(ki_angle, 3));
  Serial.println("danh lai = " + String(danh_lai));
}

void handleData() {
  String json = "{";
  json += "\"throttle\":" + String(throttle) + ",";
  json += "\"alt\":" + String(AltitudeKalman, 2) + ",";
  json += "\"vel\":" + String(VelocityVerticalKalman, 2);
  json += "}";

  server.send(200, "application/json", json);
}