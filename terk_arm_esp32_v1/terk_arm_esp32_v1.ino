#include <queue>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <AccelStepper.h>
#include <ESP32Servo.h>
#include <math.h>  // สำหรับฟังก์ชันทางคณิตศาสตร์
#include "RobotArmIK.h"
#include <EEPROM.h>
const char* ssid = "Terk_2.4GHz";      // ชื่อ Wi-Fi ของคุณ
const char* password = "08110171188";  // รหัสผ่าน Wi-Fi ของคุณ

struct Command {
  String type;
  int theta1;
  int theta2;
  int theta3;
  int speed;
  int acceleration;
};

// สร้างคิวสำหรับจัดเก็บคำสั่ง
std::queue<Command> commandQueue;
// สร้างวัตถุ WebServer (ใช้พอร์ต 80)
WebServer server(80);
int len = 0;
int wifi_connecy_count = 0;
// สร้างวัตถุมอเตอร์และเซอร์โวเหมือนในโค้ดเดิม
AccelStepper stepper1(1, 17, 16);  // ข้อต่อ 1 หมุนรอบแกน Z
AccelStepper stepper2(1, 18, 5);   // ข้อต่อ 2
AccelStepper stepper3(1, 21, 19);  // ข้อต่อ 3
Servo gripperServo;
// กำหนด Limit Switch
#define limitSwitch1 26    // Limit switch สำหรับมอเตอร์ 1
#define limitSwitch2 25    // Limit switch สำหรับมอเตอร์ 2
#define limitSwitch3 33    // Limit switch สำหรับมอเตอร์ 3
#define DEBOUNCE_DELAY 10  // เวลาในการหน่วง debounce (50 มิลลิวินาที)

// ประกาศ Timer
hw_timer_t* timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
#define MAX_STEPS 1  // จำนวนขั้นตอนสูงสุดที่ต้องการสำหรับ trajectory

// ตัวแปรจัดเก็บตำแหน่งมุมของแต่ละขั้นตอน
int theta1Steps[MAX_STEPS];
int theta2Steps[MAX_STEPS];
int theta3Steps[MAX_STEPS];

int currentStep = 0;    // ขั้นตอนปัจจุบัน
int totalSteps = 0;     // จำนวนขั้นตอนทั้งหมด
bool isMoving = false;  // สถานะของการเคลื่อนที่

// ความยาวของข้อต่อ (สมมุติ)
double L1 = 135;          // ความยาวของข้อต่อแรก L1 = 135 มม.
double L2 = 157;          // ความยาวของข้อต่อที่สอง L2 = 147 มม.
double baseHeight = 156;  // ความสูงจากพื้น 156 มม.

int stepper1Position, stepper2Position, stepper3Position;
float theta1AngleToSteps = 2.8778;
float theta2AngleToSteps = 2.8778;
float phiAngleToSteps = 23.22 ; 

// ตัวแปรเก็บค่าที่ถูกส่งไปล่าสุด
int lastTheta1 = 250;
int lastTheta2 = 210;
int lastTheta3 = 50;

int lastTheta1_joint = 0;
int lastTheta2_joint = 0;
int lastTheta3_joint = 0;

int targetTheta1_i = 0;
int targetTheta2_i = 0;
int targetTheta3_i = 0;

int lastTheta1_home = 5;
int lastTheta2_home = 22;
int lastTheta3_home = 158;
int lastGripper = 180;
int lastSpeed = 1000;
int lastAcceleration = 500;

// ตัวแปรสำหรับตำแหน่งปลายแขนกล (forward kinematics result)
double endEffectorX = 0;
double endEffectorY = 0;
double endEffectorZ = 0;  // เพิ่มแกน Z
g_Code cmd;
RobotArmIK robotArmIK(L1, L2, baseHeight, 125, 125, 420, 50);

String Arm_mode = "JOINT";
String savedSSID = "";
String savedPassword = "";
String html = "";
String content = "";
String state = "";
bool isFirstTime = false;  // เช็คว่าคือการเริ่มต้นครั้งแรกหรือไม่
char data[32];

unsigned char k;



String ip = "";
String gateway = "";
String subnet = "";
String urlMain = "http://183.88.230.236:12000";
int delay_time;

// ตั้งค่า flag เพื่อตรวจสอบการทำงานของ Timer
volatile bool timerFlag = false;

// ฟังก์ชัน Timer ISR
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  timerFlag = true;
  portEXIT_CRITICAL_ISR(&timerMux);
}
// ฟังก์ชัน debounce สำหรับ Limit Switch
bool debounce(int pin) {
  bool state = digitalRead(pin);
  delay(DEBOUNCE_DELAY);  // รอช่วงเวลาหน่วง
  bool newState = digitalRead(pin);
  return state == newState ? state : HIGH;  // ตรวจสอบว่าค่าที่อ่านได้เป็นค่าเดิมหรือไม่
}

void setStepperSpeedAndAcceleration(int speed, int acceleration) {
  stepper1.setMaxSpeed(speed);
  stepper1.setAcceleration(acceleration);
  stepper2.setMaxSpeed(speed);
  stepper2.setAcceleration(acceleration);
  stepper3.setMaxSpeed(speed);
  stepper3.setAcceleration(acceleration);
}
// ฟังก์ชันสำหรับการตั้งค่าตำแหน่งเริ่มต้นโดยใช้ Limit Switch พร้อม debounce
void setZero() {

  // ตั้งค่าความเร็วและความเร่งที่ได้รับจากเว็บ
  stepper1.setMaxSpeed(1000);
  stepper1.setAcceleration(500);
  stepper2.setMaxSpeed(1000);
  stepper2.setAcceleration(500);
  stepper3.setMaxSpeed(1000);
  stepper3.setAcceleration(500);
  // กำหนดทิศทางการเคลื่อนที่ให้เคลื่อนไปยัง Limit Switch
  stepper1.setSpeed(-2000);  // เคลื่อนที่ไปทิศทางตรงข้ามเพื่อหาจุดเริ่มต้น
  // stepper1.setAcceleration(500);
  stepper2.setSpeed(-2000);
  // stepper2.setAcceleration(500);
  stepper3.setSpeed(-2000);
  // stepper3.setAcceleration(500);
  while (debounce(limitSwitch2) == HIGH) {
    stepper2.runSpeed();
  }
  stepper2.setCurrentPosition(0);  // ตั้งค่าเป็นศูนย์เมื่อถึง Limit Switch
  // รันมอเตอร์จนกว่าจะถึง Limit Switch (พร้อม debounce)
  while (debounce(limitSwitch1) == HIGH) {
    stepper1.runSpeed();
  }
  stepper1.setCurrentPosition(0);  // ตั้งค่าเป็นศูนย์เมื่อถึง Limit Switch

  while (debounce(limitSwitch3) == HIGH) {
    stepper3.runSpeed();
  }
  stepper3.setCurrentPosition(0);  // ตั้งค่าเป็นศูนย์เมื่อถึง Limit Switch

  Serial.println("Set Zero: มอเตอร์ทั้งหมดถูกตั้งค่าที่ตำแหน่ง 0");
}
// ฟังก์ชันสำหรับตรวจสอบสถานะ Limit Switch
// ฟังก์ชันสำหรับตรวจสอบสถานะ Limit Switch
bool checkLimitSwitch() {
  bool limit1 = digitalRead(limitSwitch1) == LOW;  // LOW เมื่อถูกกด
  bool limit2 = digitalRead(limitSwitch2) == LOW;
  bool limit3 = digitalRead(limitSwitch3) == LOW;

  // หากมี Limit Switch ใดถูกกด, ให้หยุดการเคลื่อนที่
  if (limit1 || limit2 || limit3) {
    Serial.println("Limit switch triggered! Motion stopped.");
    return true;  // มี Limit Switch ถูกกด
  }
  return false;  // ไม่มี Limit Switch ถูกกด
}

float lerp(float start, float end, float t) {
  return start + (end - start) * t;
}
// ฟังก์ชัน Trajectory Planning
void trajectoryPlanning(int targetTheta1, int targetTheta2, int targetTheta3, int duration) {

  // เรียกใช้ฟังก์ชัน runIK เพื่อคำนวณตำแหน่ง x, y, z ที่ต้องการ
  g_Code stepCmd = robotArmIK.runIK(targetTheta1, targetTheta2, targetTheta3, cmd);


  Command cmd;
  // Serial.print("x = ");
  // Serial.println(targetTheta1);
  // Serial.print("y = ");
  // Serial.println(targetTheta2);
  // Serial.print("z = ");
  // Serial.println(targetTheta3);
  // Serial.print("b = ");
  // Serial.println(stepCmd.x);
  // Serial.print("L1 = ");
  // Serial.println(stepCmd.y);
  // Serial.print("L2 = ");
  // Serial.println(stepCmd.z);

  if (stepCmd.y > 110) {
    stepCmd.y = 110;
  }

  if (stepCmd.z > 110) {
    stepCmd.z = 110;
  }
  cmd.type = "move";
  cmd.theta1 = stepCmd.z * theta1AngleToSteps;
  ;
  cmd.theta2 = stepCmd.y * theta2AngleToSteps;
  cmd.theta3 = stepCmd.x * phiAngleToSteps;
  cmd.speed = lastSpeed;
  cmd.acceleration = lastAcceleration;
  commandQueue.push(cmd);


  // ตั้งค่าเริ่มต้นสำหรับการเคลื่อนที่
  currentStep = 0;
  isMoving = true;
  lastTheta1 = targetTheta1;
  lastTheta2 = targetTheta2;
  lastTheta3 = targetTheta3;
}

void Planning(int targetTheta1, int targetTheta2, int targetTheta3, int duration) {

  Command cmd;
  cmd.type = "move";
  cmd.theta1 = targetTheta1 * theta1AngleToSteps;
  cmd.theta2 = targetTheta2 * theta2AngleToSteps;
  cmd.theta3 = targetTheta3 * phiAngleToSteps;
  cmd.speed = lastSpeed;
  cmd.acceleration = lastAcceleration;
  commandQueue.push(cmd);


  // ตั้งค่าเริ่มต้นสำหรับการเคลื่อนที่
  currentStep = 0;
  isMoving = true;
  lastTheta1_joint = targetTheta1;
  lastTheta2_joint = targetTheta2;
  lastTheta3_joint = targetTheta3;
}
void handleSetup() {

  // แสดงหน้าเว็บสำหรับการตั้งค่า Wi-Fi
  html = "<html><body>";
  html += "<h1>WiFi and Network Setup</h1>";
  html += "<form action=\"/save-setup\" method=\"POST\">";
  html += "SSID: <input type=\"text\" name=\"ssid\"><br>";
  html += "Password: <input type=\"password\" name=\"password\"><br>";
  html += "Static IP Address: <input type=\"text\" name=\"ip\" value=\"" + readStringFromEEPROM(64) + "\"><br>";
  html += "Gateway: <input type=\"text\" name=\"gateway\" value=\"" + readStringFromEEPROM(96) + "\"><br>";
  html += "Subnet Mask: <input type=\"text\" name=\"subnet\" value=\"" + readStringFromEEPROM(128) + "\"><br>";
  html += "<input type=\"submit\" value=\"Save\">";
  html += "</form>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

void executeCommand(Command cmd) {
  // Set speed and acceleration
  stepper1.setMaxSpeed(cmd.speed);
  stepper1.setAcceleration(cmd.acceleration);
  stepper2.setMaxSpeed(cmd.speed);
  stepper2.setAcceleration(cmd.acceleration);
  stepper3.setMaxSpeed(cmd.speed);
  stepper3.setAcceleration(cmd.acceleration);

  // Move to target positions
  stepper1.moveTo(cmd.theta1);
  stepper2.moveTo(cmd.theta2);
  stepper3.moveTo(cmd.theta3);

  // Run motors
  while (stepper1.isRunning() || stepper2.isRunning() || stepper3.isRunning()) {
    stepper1.run();
    stepper2.run();
    stepper3.run();
  }
}
void loadSettingsFromEEPROM() {
  theta1AngleToSteps = EEPROM.readFloat(0);
  theta2AngleToSteps = EEPROM.readFloat(4);
  phiAngleToSteps = EEPROM.readFloat(8);
  L1 = EEPROM.readFloat(12);
  L2 = EEPROM.readFloat(16);
  baseHeight = EEPROM.readFloat(20);

  // ตรวจสอบว่ามีการตั้งค่าไว้แล้วหรือไม่ ถ้าไม่ให้ใช้ค่าที่ตั้งไว้ในโค้ด
  if (theta1AngleToSteps == 0)
    theta1AngleToSteps = 77.222222;
  if (theta2AngleToSteps == 0)
    theta2AngleToSteps = 77.222222;
  if (phiAngleToSteps == 0)
    phiAngleToSteps = 23.222222;
  if (L1 == 0)
    L1 = 135;
  if (L2 == 0)
    L2 = 157;
  if (baseHeight == 0)
    baseHeight = 156;
}
void handleSaveWiFi() {
  savedSSID = server.arg("ssid");
  savedPassword = server.arg("password");
  ip = server.arg("ip");
  gateway = server.arg("gateway");
  subnet = server.arg("subnet");

  if (savedSSID.length() > 0 && savedPassword.length() > 0) {
    // บันทึก SSID และ Password
    writeStringToEEPROM(0, savedSSID);
    writeStringToEEPROM(32, savedPassword);
    EEPROM.commit();

    // บันทึก IP Address, Gateway และ Subnet
    writeStringToEEPROM(64, ip);
    writeStringToEEPROM(96, gateway);
    writeStringToEEPROM(128, subnet);
    EEPROM.commit();

    server.send(200, "text/html", "Settings saved. Restarting...");
    delay(2000);
    ESP.restart();
  } else {
    server.send(200, "text/html", "Please provide all required fields.");
  }
}
// ฟังก์ชันสำหรับแสดงหน้าเว็บพร้อมค่าสุดท้ายที่ผู้ใช้ส่ง
void handleRoot() {
  html = "<!DOCTYPE html>";
  html += "<html lang=\"en\">";
  html += "<head>";
  html += "<meta charset=\"UTF-8\">";
  html += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">";
  html += "<title>Robot Arm Control</title>";
  html += "<style>";
  html += "body { background: #f0f4f8; font-family: 'Segoe UI', Arial, sans-serif; margin: 0; padding: 0; }";
  html += ".container { background: #fff; max-width: 420px; margin: 40px auto; border-radius: 12px; box-shadow: 0 4px 24px rgba(0,0,0,0.08); padding: 32px 24px; }";
  html += "h1 { color: #2d3a4b; margin-bottom: 18px; }";
  html += "form { margin-bottom: 24px; }";
  html += ".form-group { margin-bottom: 14px; text-align: left; }";
  html += ".form-group label { display: block; margin-bottom: 4px; color: #444; font-weight: 500; }";
  html += ".form-group input { width: 100%; padding: 8px 10px; border: 1px solid #cfd8dc; border-radius: 5px; font-size: 15px; }";
  html += ".form-group input:focus { outline: none; border-color: #4CAF50; }";
  html += "button, input[type='submit'] { background: #4CAF50; color: #fff; border: none; border-radius: 5px; padding: 10px 22px; font-size: 16px; cursor: pointer; transition: background 0.2s; }";
  html += "button:hover, input[type='submit']:hover { background: #388e3c; }";
  html += ".effector { background: #f7fafc; border-radius: 8px; padding: 12px 0 12px 0; margin-bottom: 18px; }";
  html += ".effector span { display: inline-block; min-width: 80px; font-weight: 500; color: #333; }";
  html += "canvas { display: block; margin: 0 auto 18px auto; border: 1px solid #b0bec5; border-radius: 6px; background: #fff; }";
  html += "@media (max-width: 500px) { .container { padding: 12px 2vw; } }";
  html += "</style>";
  html += "</head>";
  html += "<body>";
  html += "<div class='container'>";
  html += "<h1>Robot Arm Control</h1>";
  html += "<form action='/move' method='GET'>";
  html += "<div class='form-group'><label for='theta1'>Joint 1 Angle</label><input type='number' id='theta1' name='theta1' value='" + String(lastTheta1) + "'></div>";
  html += "<div class='form-group'><label for='theta2'>Joint 2 Angle</label><input type='number' id='theta2' name='theta2' value='" + String(lastTheta2) + "'></div>";
  html += "<div class='form-group'><label for='theta3'>Joint 3 Angle</label><input type='number' id='theta3' name='theta3' value='" + String(lastTheta3) + "'></div>";
  html += "<div class='form-group'><label for='gripper'>Gripper</label><input type='number' id='gripper' name='gripper' value='" + String(lastGripper) + "'></div>";
  html += "<div class='form-group'><label for='speed'>Speed</label><input type='number' id='speed' name='speed' value='" + String(lastSpeed) + "'></div>";
  html += "<div class='form-group'><label for='acceleration'>Acceleration</label><input type='number' id='acceleration' name='acceleration' value='" + String(lastAcceleration) + "'></div>";
  html += "<input type='submit' value='Move'>";
  html += "</form>";
  html += "<div class='effector'>";
  html += "<h2 style='margin:0 0 10px 0;color:#2d3a4b;'>End Effector Position</h2>";
  html += "<span>X: " + String(endEffectorX) + " mm</span> ";
  html += "<span>Y: " + String(endEffectorY) + " mm</span> ";
  html += "<span>Z: " + String(endEffectorZ) + " mm</span>";
  html += "</div>";
  html += "<canvas id='myChart' width='340' height='340'></canvas>";
  html += "<script>";
  html += "function drawChart() {";
  html += "  var canvas = document.getElementById('myChart');";
  html += "  var ctx = canvas.getContext('2d');";
  html += "  ctx.clearRect(0, 0, canvas.width, canvas.height);";
  html += "  var scale = 1.5;";
  html += "  var x = " + String(endEffectorX) + " * scale + 170;";
  html += "  var y = " + String(endEffectorY) + " * scale + 170;";
  html += "  ctx.beginPath();";
  html += "  ctx.arc(x, y, 7, 0, 2 * Math.PI);";
  html += "  ctx.fillStyle = '#e53935';";
  html += "  ctx.fill();";
  html += "  ctx.font = '13px Segoe UI, Arial';";
  html += "  ctx.fillStyle = '#333';";
  html += "  ctx.fillText('X: ' + Math.round(" + String(endEffectorX) + ") + ' mm', x + 12, y);";
  html += "  ctx.fillText('Y: ' + Math.round(" + String(endEffectorY) + ") + ' mm', x + 12, y + 18);";
  html += "  ctx.fillText('Z: ' + Math.round(" + String(endEffectorZ) + ") + ' mm', x + 12, y + 36);";
  html += "}";
  html += "drawChart();";
  html += "</script>";
  html += "</div>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}


void handlejoint() {
  html = "<!DOCTYPE html>";
  html += "<html lang=\"en\">";
  html += "<head>";
  html += "<meta charset=\"UTF-8\">";
  html += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">";
  html += "<title>Robot Arm Joint Control</title>";
  html += "<style>";
  html += "body { background: #f0f4f8; font-family: 'Segoe UI', Arial, sans-serif; margin: 0; padding: 0; }";
  html += ".container { background: #fff; max-width: 440px; margin: 40px auto; border-radius: 14px; box-shadow: 0 4px 24px rgba(0,0,0,0.10); padding: 32px 24px; }";
  html += "h1 { color: #2d3a4b; margin-bottom: 18px; }";
  html += "form { margin-bottom: 20px; }";
  html += ".form-group { margin-bottom: 14px; text-align: left; }";
  html += ".form-group label { display: block; margin-bottom: 4px; color: #444; font-weight: 500; }";
  html += ".form-group input { width: 100%; padding: 8px 10px; border: 1px solid #cfd8dc; border-radius: 5px; font-size: 15px; }";
  html += ".form-group input:focus { outline: none; border-color: #4CAF50; }";
  html += ".btn-row { display: flex; gap: 10px; margin-bottom: 18px; justify-content: center; }";
  html += ".btn-row button { flex: 1; background: #1976d2; color: #fff; border: none; border-radius: 5px; padding: 10px 0; font-size: 15px; font-weight: 500; cursor: pointer; transition: background 0.2s; }";
  html += ".btn-row button:hover { background: #0d47a1; }";
  html += "button, input[type='submit'] { background: #4CAF50; color: #fff; border: none; border-radius: 5px; padding: 10px 22px; font-size: 16px; cursor: pointer; transition: background 0.2s; }";
  html += "button:hover, input[type='submit']:hover { background: #388e3c; }";
  html += ".effector { background: #f7fafc; border-radius: 8px; padding: 12px 0 12px 0; margin-bottom: 18px; }";
  html += ".effector span { display: inline-block; min-width: 80px; font-weight: 500; color: #333; }";
  html += ".status { margin-bottom: 10px; color: #1976d2; font-size: 15px; min-height: 20px; text-align: center; }";
  html += "@media (max-width: 500px) { .container { padding: 12px 2vw; } .btn-row button { font-size: 13px; } }";
  html += "</style>";
  html += "<script>function sendAction(action) { fetch('/gohomevalue', {method: 'GET'}).then(()=>location.reload()); document.getElementById('theta1').value = 0; document.getElementById('theta2').value = 0;document.getElementById('theta3').value = 0;}</script>";
  html += "<script>function callSetHomeValue() { fetch('/sethomevalue', {method: 'GET'}).then(()=>location.reload()); }</script>";
  html += "</head>";
  html += "<body>";
  html += "<div class='container'>";
  html += "<h1>Robot Arm Joint Control</h1>";
  html += "<div class='btn-row'>";
  html += "<button onclick=\"sendAction('home')\">Home</button>";

  html += "<button onclick=\"callSetHomeValue()\" style='background:#ff9800;'>Set Home Value</button>";
  html += "</div>";
  html += "<div class='status' id='status'></div>";
  html += "<form action='/movejoint' method='GET'>";
  html += "<div class='form-group'><label for='theta1'>Joint(ข้อต่อที่ 2)  Angle</label><input type='number' id='theta1' name='theta1' value='" + String(lastTheta1_joint) + "'></div>";
  html += "<div class='form-group'><label for='theta2'>Joint(ข้อต่อที่ 1)  Angle</label><input type='number' id='theta2' name='theta2' value='" + String(lastTheta2_joint) + "'></div>";
  html += "<div class='form-group'><label for='theta3'>Joint(ฐาน)) 3 Angle</label><input type='number' id='theta3' name='theta3' value='" + String(lastTheta3_joint) + "'></div>";
  html += "<div class='form-group'><label for='gripper'>Gripper</label><input type='number' id='gripper' name='gripper' value='" + String(lastGripper) + "'></div>";
  html += "<div class='form-group'><label for='speed'>Speed</label><input type='number' id='speed' name='speed' value='" + String(lastSpeed) + "'></div>";
  html += "<div class='form-group'><label for='acceleration'>Acceleration</label><input type='number' id='acceleration' name='acceleration' value='" + String(lastAcceleration) + "'></div>";
  html += "<input type='submit' value='Move Joint'>";
  html += "</form>";
  html += "<div class='effector'>";
  html += "<h2 style='margin:0 0 10px 0;color:#2d3a4b;'>End Effector Position</h2>";
  html += "<span>X: " + String(endEffectorX) + " mm</span> ";
  html += "<span>Y: " + String(endEffectorY) + " mm</span> ";
  html += "<span>Z: " + String(endEffectorZ) + " mm</span>";
  html += "</div>";
  html += "</div>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}
void handleGcodePage() {
  html = "<!DOCTYPE html>";
  html += "<html lang=\"en\">";
  html += "<head>";
  html += "<meta charset=\"UTF-8\">";
  html += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">";
  html += "<title>G-code Input</title>";
  html += "<style>";
  html += "body { background: #f0f4f8; font-family: 'Segoe UI', Arial, sans-serif; margin: 0; padding: 0; }";
  html += ".container { background: #fff; max-width: 480px; margin: 40px auto; border-radius: 12px; box-shadow: 0 4px 24px rgba(0,0,0,0.08); padding: 32px 24px; }";
  html += "h1 { color: #2d3a4b; margin-bottom: 18px; }";
  html += "form { margin-bottom: 24px; }";
  html += ".form-group { margin-bottom: 14px; text-align: left; }";
  html += ".form-group label { display: block; margin-bottom: 4px; color: #444; font-weight: 500; }";
  html += ".form-group textarea { width: 100%; min-height: 120px; padding: 8px 10px; border: 1px solid #cfd8dc; border-radius: 5px; font-size: 15px; resize: vertical; }";
  html += ".form-group textarea:focus { outline: none; border-color: #4CAF50; }";
  html += "input[type='submit'] { background: #4CAF50; color: #fff; border: none; border-radius: 5px; padding: 10px 22px; font-size: 16px; cursor: pointer; transition: background 0.2s; margin-top: 10px; }";
  html += "input[type='submit']:hover { background: #388e3c; }";
  html += "</style>";
  html += "</head>";
  html += "<body>";
  html += "<div class='container'>";
  html += "<h1>G-code Input</h1>";
  html += "<form action='/gcode' method='POST'>";
  html += "<div class='form-group'><label for='gcode'>Paste your G-code here:</label><textarea id='gcode' name='gcode'></textarea></div>";
  html += "<input type='submit' value='Send G-code'>";
  html += "</form>";
  html += "</div>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

// Handler สำหรับรับข้อมูล G-code ที่ส่งมาจากฟอร์ม
void handleGcodeSubmit() {
  String gcode = server.arg("gcode");
  Serial.println("------G-code Received------");
  Serial.println(gcode);
  // รองรับหลายบรรทัด X100Y100Z100
  String resultHtml = "<html><body><h1>G-code Received</h1><pre>" + gcode + "</pre><hr><ul>";
  int lineStart = 0;
  while (lineStart < gcode.length()) {
    int lineEnd = gcode.indexOf('\n', lineStart);
    if (lineEnd == -1) lineEnd = gcode.length();
    String line = gcode.substring(lineStart, lineEnd);
    line.trim();
    if (line.length() > 0) {
      String type;
      int xVal = 0, yVal = 0, zVal = 0, delayVal = 0;
      parseGcodeLine(line, type, xVal, yVal, zVal, delayVal);

      if (type == "XYZ") {
        trajectoryPlanning(xVal, yVal, zVal, delayVal);
        Serial.print("X: ");
        Serial.print(xVal);
        Serial.print(", Y: ");
        Serial.print(yVal);
        Serial.print(", Z: ");
        Serial.println(zVal);
        resultHtml += "<li>" + line + " &rarr; X: " + String(xVal) + ", Y: " + String(yVal) + ", Z: " + String(zVal) + "</li>";
      } else if (type == "GPON") {
        Command cmd;
        cmd.type = "Gripper";
        cmd.theta1 = 0;
        cmd.theta2 = 0;
        cmd.theta3 = 0;
        cmd.speed = 0;
        cmd.acceleration = 0;
        commandQueue.push(cmd);
        Serial.println("Gripper ON");
        resultHtml += "<li>" + line + " &rarr; Gripper ON</li>";
      } else if (type == "GPOFF") {
        Command cmd;
        cmd.type = "Gripper";
        cmd.theta1 = 180;
        cmd.theta2 = 0;
        cmd.theta3 = 0;
        cmd.speed = 0;
        cmd.acceleration = 0;
        commandQueue.push(cmd);
        Serial.println("Gripper OFF");
        resultHtml += "<li>" + line + " &rarr; Gripper OFF</li>";
      } else if (type == "DELAY") {
        Command cmd;
        cmd.type = "delay";
        cmd.theta1 = delayVal;
        cmd.theta2 = 0;
        cmd.theta3 = 0;
        cmd.speed = 0;
        cmd.acceleration = 0;
        commandQueue.push(cmd);
        Serial.print("Delay: ");
        Serial.println(delayVal);
        resultHtml += "<li>" + line + " &rarr; Delay " + String(delayVal) + " ms</li>";
      } else {
        resultHtml += "<li>" + line + " &rarr; Unknown</li>";
      }
      // ฟังก์ชันแยกประเภทคำสั่ง G-code: XYZ, GPON, GPOFF, DELAY
    }
    lineStart = lineEnd + 1;
  }
  resultHtml += "</ul><a href='/gcode'>Back</a></body></html>";
  server.send(200, "text/html", resultHtml);
}


void serial() {
  if (Serial.available()) {
    content = Serial.readString();  // อ่านข้อมูลจาก Processing
    Serial.println(content);
    // รองรับหลายบรรทัด G-code เช่นเดียวกับหน้าเว็บ
    int lineStart = 0;
    while (lineStart < content.length()) {
      int lineEnd = content.indexOf('\n', lineStart);
      if (lineEnd == -1) lineEnd = content.length();
      String line = content.substring(lineStart, lineEnd);
      line.trim();
      if (line.length() > 0) {
        String type;
        int xVal = 0, yVal = 0, zVal = 0, delayVal = 0;
        parseGcodeLine(line, type, xVal, yVal, zVal, delayVal);

        if (type == "XYZ") {
          Serial.print("[SERIAL] X: ");
          Serial.print(xVal);
          Serial.print(", Y: ");
          Serial.print(yVal);
          Serial.print(", Z: ");
          Serial.println(zVal);
          trajectoryPlanning(xVal, yVal, zVal, delayVal);
        } else if (type == "GPON") {
          Command cmd;
          cmd.type = "Gripper";
          cmd.theta1 = 0;
          cmd.theta2 = 0;
          cmd.theta3 = 0;
          cmd.speed = 0;
          cmd.acceleration = 0;
          commandQueue.push(cmd);
          Serial.println("[SERIAL] Gripper ON");
        } else if (type == "GPOFF") {
          Command cmd;
          cmd.type = "Gripper";
          cmd.theta1 = 180;
          cmd.theta2 = 0;
          cmd.theta3 = 0;
          cmd.speed = 0;
          cmd.acceleration = 0;
          commandQueue.push(cmd);
          Serial.println("[SERIAL] Gripper OFF");
        } else if (type == "DELAY") {
          Command cmd;
          cmd.type = "delay";
          cmd.theta1 = delayVal;
          cmd.theta2 = 0;
          cmd.theta3 = 0;
          cmd.speed = 0;
          cmd.acceleration = 0;
          commandQueue.push(cmd);
          Serial.print("[SERIAL] Delay: ");
          Serial.println(delayVal);
        } else {
          Serial.println("[SERIAL] Unknown command");
        }
      }
      lineStart = lineEnd + 1;
    }
  }
}
void parseGcodeLine(const String& str, String& type, int& x, int& y, int& z, int& delayVal) {
  String s = str;
  s.trim();
  x = y = z = delayVal = 0;
  if (s.startsWith("X") && s.indexOf('Y') != -1 && s.indexOf('Z') != -1) {
    int xIdx = s.indexOf('X');
    int yIdx = s.indexOf('Y');
    int zIdx = s.indexOf('Z');
    if (xIdx != -1 && yIdx != -1 && zIdx != -1 && xIdx < yIdx && yIdx < zIdx) {
      String xStr = s.substring(xIdx + 1, yIdx);
      String yStr = s.substring(yIdx + 1, zIdx);
      String zStr = s.substring(zIdx + 1);
      x = xStr.toInt();
      y = yStr.toInt();
      z = zStr.toInt();
      type = "XYZ";
      return;
    }
  }
  if (s.equalsIgnoreCase("GPON")) {
    type = "GPON";
    return;
  }
  if (s.equalsIgnoreCase("GPOFF")) {
    type = "GPOFF";
    return;
  }
  if (s.startsWith("DELAY")) {
    String d = s.substring(5);
    d.trim();
    delayVal = d.toInt();
    type = "DELAY";
    return;
  }
  type = "UNKNOWN";
}

void parseXYZString(const String& str, int& x, int& y, int& z) {
  int xIdx = str.indexOf('X');
  int yIdx = str.indexOf('Y');
  int zIdx = str.indexOf('Z');
  if (xIdx != -1 && yIdx != -1 && zIdx != -1 && xIdx < yIdx && yIdx < zIdx) {
    String xStr = str.substring(xIdx + 1, yIdx);
    String yStr = str.substring(yIdx + 1, zIdx);
    String zStr = str.substring(zIdx + 1);
    x = xStr.toInt();
    y = yStr.toInt();
    z = zStr.toInt();
  } else {
    x = y = z = 0;
  }
}
void handleMain() {
  html = "<!DOCTYPE html>";
  html += "<html lang=\"th\">";
  html += "<head>";
  html += "<meta charset=\"UTF-8\">";
  html += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">";
  html += "<title>Robot Arm Learning Platform</title>";
  html += "<style>";
  html += "body { background: #f0f4f8; font-family: 'Segoe UI', Arial, sans-serif; margin: 0; padding: 0; }";
  html += ".container { background: #fff; max-width: 420px; margin: 40px auto; border-radius: 12px; box-shadow: 0 4px 24px rgba(0,0,0,0.08); padding: 32px 24px; text-align: center; }";
  html += "h1 { color: #2d3a4b; margin-bottom: 18px; }";
  html += "h3 { color: #4CAF50; margin-top: 24px; margin-bottom: 12px; }";
  html += ".menu { display: flex; flex-direction: column; gap: 12px; margin-bottom: 18px; }";
  html += ".menu a { text-decoration: none; background: #4CAF50; color: #fff; border-radius: 6px; padding: 12px 0; font-size: 17px; font-weight: 500; transition: background 0.2s; box-shadow: 0 2px 8px rgba(76,175,80,0.07); }";
  html += ".menu a:hover { background: #388e3c; }";
  html += "@media (max-width: 500px) { .container { padding: 12px 2vw; } .menu a { font-size: 15px; } }";
  html += "</style>";
  html += "</head>";
  html += "<body>";
  html += "<div class='container'>";
  html += "<h1>ยินดีต้อนรับสู่แพลตฟอร์มการเรียนรู้แขนหุ่นยนต์</h1>";
  html += "<h3>เลือกบทเรียน</h3>";
  html += "<div class='menu'>";
  html += "<a href='joint'>การควบคุม joint</a>";
  html += "<a href='index'>การควบคุม x y z</a>";
  html += "<a href='gcode'>G code</a>";
  html += "<a href='" + urlMain + "/blockly?ip=" + ip + "'>Blockly</a>";
  html += "<a href='" + urlMain + "'>เนื้อหาการสร้างแขนกล</a>";
  html += "</div>";
  html += "<h3>ควบคุมแขนกล</h3>";
  html += "<div class='menu'>";
  html += "<a href='setup'>ตั้งค่า WiFi</a>";
  html += "<a href='setuparm'>ตั้งค่าแขนกล</a>";
  html += "<a href='ota'>อัปเดตเฟิร์มแวร์</a>";
  html += "</div>";
  html += "</div>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}
// ฟังก์ชันการแสดงหน้าเว็บสำหรับตั้งค่า
void handleSetupPage() {
  html = "<!DOCTYPE html>";
  html += "<html lang=\"en\">";
  html += "<head>";
  html += "<meta charset=\"UTF-8\">";
  html += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">";
  html += "<title>Robot Arm Configuration</title>";
  html += "<style>";
  html += "body { background: #f0f4f8; font-family: 'Segoe UI', Arial, sans-serif; margin: 0; padding: 0; }";
  html += ".container { background: #fff; max-width: 420px; margin: 40px auto; border-radius: 12px; box-shadow: 0 4px 24px rgba(0,0,0,0.08); padding: 32px 24px; }";
  html += "h1 { color: #2d3a4b; margin-bottom: 18px; }";
  html += "form { margin-top: 18px; }";
  html += ".form-group { margin-bottom: 14px; text-align: left; }";
  html += ".form-group label { display: block; margin-bottom: 4px; color: #444; font-weight: 500; }";
  html += ".form-group input { width: 100%; padding: 8px 10px; border: 1px solid #cfd8dc; border-radius: 5px; font-size: 15px; }";
  html += ".form-group input:focus { outline: none; border-color: #4CAF50; }";
  html += "input[type='submit'] { background: #4CAF50; color: #fff; border: none; border-radius: 5px; padding: 10px 22px; font-size: 16px; cursor: pointer; transition: background 0.2s; margin-top: 10px; }";
  html += "input[type='submit']:hover { background: #388e3c; }";
  html += "@media (max-width: 500px) { .container { padding: 12px 2vw; } }";
  html += "</style>";
  html += "</head>";
  html += "<body>";
  html += "<div class='container'>";
  html += "<h1>Robot Arm Configuration</h1>";
  html += "<form action='/save-param' method='POST'>";
  html += "<div class='form-group'><label for='theta1AngleToSteps'>Theta1 AngleToSteps</label><input type='number' step='0.1' id='theta1AngleToSteps' name='theta1AngleToSteps' value='" + String(theta1AngleToSteps) + "'></div>";
  html += "<div class='form-group'><label for='theta2AngleToSteps'>Theta2 AngleToSteps</label><input type='number' step='0.1' id='theta2AngleToSteps' name='theta2AngleToSteps' value='" + String(theta2AngleToSteps) + "'></div>";
  html += "<div class='form-group'><label for='phiAngleToSteps'>Phi AngleToSteps</label><input type='number' step='0.1' id='phiAngleToSteps' name='phiAngleToSteps' value='" + String(phiAngleToSteps) + "'></div>";
  html += "<div class='form-group'><label for='L1'>L1 (mm)</label><input type='number' step='0.1' id='L1' name='L1' value='" + String(L1) + "'></div>";
  html += "<div class='form-group'><label for='L2'>L2 (mm)</label><input type='number' step='0.1' id='L2' name='L2' value='" + String(L2) + "'></div>";
  html += "<div class='form-group'><label for='baseHeight'>Base Height (mm)</label><input type='number' step='0.1' id='baseHeight' name='baseHeight' value='" + String(baseHeight) + "'></div>";
  html += "<div class='form-group'><label for='ip'>Static IP Address</label><input type='text' id='ip' name='ip' value='" + readStringFromEEPROM(64) + "'></div>";
  html += "<div class='form-group'><label for='gateway'>Gateway</label><input type='text' id='gateway' name='gateway' value='" + readStringFromEEPROM(96) + "'></div>";
  html += "<div class='form-group'><label for='subnet'>Subnet Mask</label><input type='text' id='subnet' name='subnet' value='" + readStringFromEEPROM(128) + "'></div>";
  html += "<input type='submit' value='Save'>";
  html += "</form>";
  html += "</div>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}


void handleOta() {
  html = "<script src='https://ajax.googleapis.com/ajax/libs/jquery/3.2.1/jquery.min.js'></script>";
  html += "<form method='POST' action='#' enctype='multipart/form-data' id='upload_form'>";
  html += "<input type='file' name='update'>";
  html += "<input type='submit' value='Update'>";
  html += "</form>";
  html += "<div id='prg'>progress: 0%</div>";
  html += "<script>";
  html += "$('form').submit(function(e){";
  html += "e.preventDefault();";
  html += "var form = $('#upload_form')[0];";
  html += "var data = new FormData(form);";
  html += " $.ajax({";
  html += "url: '/update',";
  html += "type: 'POST',";
  html += "data: data,";
  html += "contentType: false,";
  html += "processData:false,";
  html += "xhr: function() {";
  html += "var xhr = new window.XMLHttpRequest();";
  html += "xhr.upload.addEventListener('progress', function(evt) {";
  html += "if (evt.lengthComputable) {";
  html += "var per = evt.loaded / evt.total;";
  html += "$('#prg').html('progress: ' + Math.round(per*100) + '%');";
  html += "}";
  html += "}, false);";
  html += "return xhr;";
  html += "},";
  html += "success:function(d, s) {";
  html += "console.log('success!')";
  html += "},";
  html += "error: function (a, b, c) {";
  html += "}";
  html += "});";
  html += "});";
  html += "</script>";
  server.send(200, "text/html", html);
}
void handleSaveSetupArm() {
  // อ่านค่าจากฟอร์ม
  theta1AngleToSteps = server.arg("theta1AngleToSteps").toFloat();
  theta2AngleToSteps = server.arg("theta2AngleToSteps").toFloat();
  phiAngleToSteps = server.arg("phiAngleToSteps").toFloat();
  L1 = server.arg("L1").toFloat();
  L2 = server.arg("L2").toFloat();
  baseHeight = server.arg("baseHeight").toFloat();
  String ip = server.arg("ip");
  String gateway = server.arg("gateway");
  String subnet = server.arg("subnet");

  // บันทึกค่าลงใน EEPROM
  EEPROM.writeFloat(0, theta1AngleToSteps);
  EEPROM.writeFloat(4, theta2AngleToSteps);
  EEPROM.writeFloat(8, phiAngleToSteps);
  EEPROM.writeFloat(12, L1);
  EEPROM.writeFloat(16, L2);
  EEPROM.writeFloat(20, baseHeight);
  writeStringToEEPROM(64, ip);
  writeStringToEEPROM(96, gateway);
  writeStringToEEPROM(128, subnet);
  EEPROM.commit();

  server.send(200, "text/html", "<html><body><h1>Settings Saved</h1><a href=\"/setup\">Go Back</a></body></html>");
}

void setHomePosition() {
  // คำนวณตำแหน่งปลายแขนกลด้วย Forward Kinematics
  // calculateForwardKinematics(lastTheta1, lastTheta2, lastTheta3);

  // ตั้งค่าความเร็วและความเร่งที่ได้รับจากเว็บ

  setStepperSpeedAndAcceleration(3000, 500);
  // แปลงมุมเป็นสเต็ปและเคลื่อนมอเตอร์
  stepper1.moveTo(lastTheta1_home * theta1AngleToSteps);  // theta1AngleToSteps
  stepper2.moveTo(lastTheta2_home * theta2AngleToSteps);  // theta2AngleToSteps
  stepper3.moveTo(lastTheta3_home * phiAngleToSteps);     // phiAngleToSteps

  // เคลื่อนที่ทั้งหมด
  while (stepper1.isRunning() || stepper2.isRunning() || stepper3.isRunning()) {

    stepper1.run();
    stepper2.run();
    stepper3.run();
  }
  stepper1.setCurrentPosition(0);  // ตั้งค่าเป็นศูนย์เมื่อถึง Limit Switch
  stepper2.setCurrentPosition(0);  // ตั้งค่าเป็นศูนย์เมื่อถึง Limit Switch
  stepper3.setCurrentPosition(0);  // ตั้งค่าเป็นศูนย์เมื่อถึง Limit Switch
  // ควบคุม Gripper
  gripperServo.write(lastGripper);
}


void setHomeValue() {
  stepper1.setCurrentPosition(0);  // ตั้งค่าเป็นศูนย์เมื่อถึง Limit Switch
  stepper2.setCurrentPosition(0);  // ตั้งค่าเป็นศูนย์เมื่อถึง Limit Switch
  stepper3.setCurrentPosition(0);  // ตั้งค่าเป็นศูนย์เมื่อถึง Limit Switch
  lastTheta1_joint = 0;
  lastTheta2_joint = 0;
  lastTheta3_joint = 0;
  server.send(200, "text/plain", "Set home value success");
}

void goHomeValue() {
  Planning(0, 0, 0, 0);
  lastTheta1_joint = 0;
  lastTheta2_joint = 0;
  lastTheta3_joint = 0;
  server.send(200, "text/plain", "Set home value success");
}
void setHomeArm() {
  // คำนวณตำแหน่งปลายแขนกลด้วย Forward Kinematics
  // calculateForwardKinematics(lastTheta1, lastTheta2, lastTheta3);

  // ตั้งค่าความเร็วและความเร่งที่ได้รับจากเว็บ

  setStepperSpeedAndAcceleration(3000, 500);
  // แปลงมุมเป็นสเต็ปและเคลื่อนมอเตอร์
  stepper1.moveTo(0);  // theta1AngleToSteps
  stepper2.moveTo(0);  // theta2AngleToSteps
  stepper3.moveTo(0);  // phiAngleToSteps

  // เคลื่อนที่ทั้งหมด
  while (stepper1.isRunning() || stepper2.isRunning() || stepper3.isRunning()) {

    stepper1.run();
    stepper2.run();
    stepper3.run();
  }
}
// ฟังก์ชันสำหรับรับคำสั่งการเคลื่อนที่จากเว็บเบราว์เซอร์
void handleMove() {
  // รับค่าจากฟอร์ม
  targetTheta1_i = server.arg("theta1").toInt();
  targetTheta2_i = server.arg("theta2").toInt();
  targetTheta3_i = server.arg("theta3").toInt();
  lastGripper = server.arg("gripper").toInt();
  lastSpeed = server.arg("speed").toInt();
  lastAcceleration = server.arg("acceleration").toInt();
  Arm_mode = String(server.arg("mode"));
  Serial.println("------handleMove-------");
  Serial.println(targetTheta1_i);
  Serial.println(targetTheta2_i);
  Serial.println(targetTheta3_i);
  Serial.println(lastGripper);
  Serial.println(lastSpeed);
  Serial.println(lastAcceleration);
  Serial.println(Arm_mode);
  Serial.println("-------------");
  // คำนวณตำแหน่งปลายแขนกลด้วย Forward Kinematics
  // calculateForwardKinematics(lastTheta1, lastTheta2, lastTheta3);

  // ตั้งค่าความเร็วและความเร่งที่ได้รับจากเว็บ
  setStepperSpeedAndAcceleration(lastSpeed, lastAcceleration);

  trajectoryPlanning(targetTheta1_i, targetTheta2_i, targetTheta3_i, 7);

  // ควบคุม Gripper
  gripperServo.write(lastGripper);

  // ทำการ Redirect กลับไปที่หน้าแรก (ฟอร์มควบคุม)
  server.sendHeader("Location", "/index");  // กลับไปที่ URL "/"
  server.send(303);                         // 303: See Other (Redirect to GET)
}


void movejoint() {
  // รับค่าจากฟอร์ม
  targetTheta1_i = server.arg("theta1").toInt();
  targetTheta2_i = server.arg("theta2").toInt();
  targetTheta3_i = server.arg("theta3").toInt();
  lastGripper = server.arg("gripper").toInt();
  lastSpeed = server.arg("speed").toInt();
  lastAcceleration = server.arg("acceleration").toInt();
  Arm_mode = String(server.arg("mode"));
  Serial.println("------handleMove-------");
  Serial.println(targetTheta1_i);
  Serial.println(targetTheta2_i);
  Serial.println(targetTheta3_i);
  Serial.println(lastGripper);
  Serial.println(lastSpeed);
  Serial.println(lastAcceleration);
  Serial.println(Arm_mode);
  Serial.println("-------------");
  // คำนวณตำแหน่งปลายแขนกลด้วย Forward Kinematics
  // calculateForwardKinematics(lastTheta1, lastTheta2, lastTheta3);

  // ตั้งค่าความเร็วและความเร่งที่ได้รับจากเว็บ
  setStepperSpeedAndAcceleration(lastSpeed, lastAcceleration);

  Planning(targetTheta1_i, targetTheta2_i, targetTheta3_i, 7);

  // ควบคุม Gripper
  gripperServo.write(lastGripper);

  // ทำการ Redirect กลับไปที่หน้าแรก (ฟอร์มควบคุม)
  server.sendHeader("Location", "/joint");  // กลับไปที่ URL "/"
  server.send(303);                         // 303: See Other (Redirect to GET)
}

void handleOutput() {
  // รับค่าจากฟอร์ม
  targetTheta1_i = server.arg("io").toInt();
  state = server.arg("state");
  Serial.println("------Output-------");
  Serial.println(targetTheta1_i);
  Serial.println(state);

  Command cmd;
  cmd.type = "output";
  cmd.theta1 = targetTheta1_i;
  cmd.theta2 = 0;
  cmd.theta3 = 0;
  cmd.speed = 0;
  cmd.acceleration = 0;
  commandQueue.push(cmd);
  // ทำการ Redirect กลับไปที่หน้าแรก (ฟอร์มควบคุม)
  server.sendHeader("Location", "/");  // กลับไปที่ URL "/"
  server.send(303);                    // 303: See Other (Redirect to GET)
}
void handleIf() {
  // รับค่าจากฟอร์ม
  targetTheta1_i = server.arg("io").toInt();
  state = server.arg("state");
  Serial.println("------if-------");
  Serial.println(targetTheta1_i);
  Serial.println(state);

  Command cmd;
  cmd.type = "if";
  cmd.theta1 = targetTheta1_i;
  cmd.theta2 = 0;
  cmd.theta3 = 0;
  cmd.speed = 0;
  cmd.acceleration = 0;
  commandQueue.push(cmd);
  // ทำการ Redirect กลับไปที่หน้าแรก (ฟอร์มควบคุม)
  server.sendHeader("Location", "/");  // กลับไปที่ URL "/"
  server.send(303);                    // 303: See Other (Redirect to GET)
}
void handleGripper() {
  // รับค่าจากฟอร์ม
  lastGripper = server.arg("gripper").toInt();


  Serial.println("------Gripper-------");

  Serial.println(lastGripper);
  // ควบคุม Gripper
  //
  Command cmd;
  cmd.type = "Gripper";
  cmd.theta1 = lastGripper;
  cmd.theta2 = 0;
  cmd.theta3 = 0;
  cmd.speed = 0;
  cmd.acceleration = 0;
  commandQueue.push(cmd);
  // ทำการ Redirect กลับไปที่หน้าแรก (ฟอร์มควบคุม)
  server.sendHeader("Location", "/");  // กลับไปที่ URL "/"
  server.send(303);                    // 303: See Other (Redirect to GET)
}
void handleDelay() {
  // รับค่าจากฟอร์ม
  delay_time = server.arg("time").toInt();



  Serial.println("------delay_time-------");
  Serial.println(delay_time);
  Command cmd;
  cmd.type = "delay";
  cmd.theta1 = delay_time;
  cmd.theta2 = 0;
  cmd.theta3 = 0;
  cmd.speed = 0;
  cmd.acceleration = 0;
  commandQueue.push(cmd);
  // ควบคุม Gripper

  // ทำการ Redirect กลับไปที่หน้าแรก (ฟอร์มควบคุม)
  server.sendHeader("Location", "/");  // กลับไปที่ URL "/"
  server.send(303);                    // 303: See Other (Redirect to GET)
}

// ฟังก์ชันสำหรับรับคำสั่งการเคลื่อนที่จากเว็บเบราว์เซอร์
void handleFor() {
  // รับค่าจากฟอร์ม
  targetTheta1_i = server.arg("BY").toInt();
  targetTheta2_i = server.arg("FROM").toInt();
  targetTheta3_i = server.arg("TO").toInt();

  Serial.println("------for-------");
  Serial.println(targetTheta1_i);
  Serial.println(targetTheta2_i);
  Serial.println(targetTheta3_i);


  Command cmd;
  cmd.type = "for";
  cmd.theta1 = targetTheta1_i;
  cmd.theta2 = targetTheta2_i;
  cmd.theta3 = targetTheta3_i;
  cmd.speed = 0;
  cmd.acceleration = 0;
  commandQueue.push(cmd);
  // ทำการ Redirect กลับไปที่หน้าแรก (ฟอร์มควบคุม)
  server.sendHeader("Location", "/");  // กลับไปที่ URL "/"
  server.send(303);                    // 303: See Other (Redirect to GET)
}
void handleHome() {
  Serial.println("------home-------");

  Command cmd;
  cmd.type = "home";
  cmd.theta1 = 0;
  cmd.theta2 = 0;
  cmd.theta3 = 0;
  cmd.speed = 0;
  cmd.acceleration = 0;
  commandQueue.push(cmd);

  server.sendHeader("Location", "/");  // กลับไปที่ URL "/"
  server.send(303);                    // 303: See Other (Redirect to GET)
}
void handleZero() {
  Serial.println("------zero-------");

  Command cmd;
  cmd.type = "zero";
  cmd.theta1 = 0;
  cmd.theta2 = 0;
  cmd.theta3 = 0;
  cmd.speed = 0;
  cmd.acceleration = 0;
  commandQueue.push(cmd);

  server.sendHeader("Location", "/");  // กลับไปที่ URL "/"
  server.send(303);                    // 303: See Other (Redirect to GET)
}
void powerOffMotors() {
  stepper1.disableOutputs();
  stepper2.disableOutputs();
  stepper3.disableOutputs();
}
// ฟังก์ชันสำหรับบันทึกข้อความลงใน EEPROM
void writeStringToEEPROM(int addr, const String& data) {
  len = data.length();
  for (int i = 0; i < len; i++) {
    EEPROM.write(addr + i, data[i]);
  }
  EEPROM.write(addr + len, 0);  // Null terminator
  EEPROM.commit();              // บันทึกข้อมูลลง EEPROM จริงๆ
}

// ฟังก์ชันสำหรับอ่านข้อความจาก EEPROM
String readStringFromEEPROM(int addr) {
  memset(data, 0, sizeof(data));

  len = 0;

  k = EEPROM.read(addr);
  Serial.println("-------------");
  while (k != 0 && len < 32) {
    data[len] = k;  // เก็บค่าที่อ่านได้ในตัวแปร data
    len++;
    k = EEPROM.read(addr + len);  // อ่านตัวอักษรถัดไปจาก EEPROM
    Serial.println(k);            // แสดงค่าที่อ่านจาก EEPROM เพื่อวิเคราะห์
  }
  Serial.println("-------------");
  data[len] = '\0';  // สิ้นสุด string ด้วย null terminator

  return String(data);  // แปลง char array เป็น String และส่งคืน
}
void setup() {
  Serial.begin(115200);

  EEPROM.begin(500);

  // ตรวจสอบว่าเคยบันทึก SSID และ Password หรือไม่

  // ตั้งค่า Limit Switch เป็น INPUT_PULLUP
  pinMode(limitSwitch1, INPUT_PULLUP);
  pinMode(limitSwitch2, INPUT_PULLUP);
  pinMode(limitSwitch3, INPUT_PULLUP);
  // เชื่อมต่อ Wi-Fi
  // เชื่อมต่อ Wi-Fi
  // WiFi.begin(ssid, password);
  // while (WiFi.status() != WL_CONNECTED) {
  //   delay(1000);
  //   Serial.println("Connecting to WiFi...");
  // }

  // // แสดง IP Address บน Serial Monitor
  // Serial.println("Connected to WiFi");
  // Serial.print("IP Address: ");
  // Serial.println(WiFi.localIP());  // แสดง IP Address ของ ESP32
  // writeStringToEEPROM(0, "0");
  // writeStringToEEPROM(32 ,"0");
  savedSSID = readStringFromEEPROM(0);
  savedPassword = readStringFromEEPROM(32);
  ip = readStringFromEEPROM(64);
  gateway = readStringFromEEPROM(96);
  subnet = readStringFromEEPROM(128);

  Serial.println(savedSSID);
  Serial.println(savedPassword);

  if (savedSSID.length() < 2) {
    Serial.println("Starting in AP mode...");
    isFirstTime = true;
    WiFi.softAP("RobotArmAP", "12345678");

    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);
  } else {
    // ตรวจสอบและตั้งค่า IP แบบคงที่จากข้อมูลที่บันทึก
    if (ip.length() > 0 && gateway.length() > 0 && subnet.length() > 0) {
      IPAddress local_IP, local_gateway, local_subnet;
      local_IP.fromString(ip);
      local_gateway.fromString(gateway);
      local_subnet.fromString(subnet);

      if (!WiFi.config(local_IP, local_gateway, local_subnet)) {
        Serial.println("STA Failed to configure");
      }
    }

    WiFi.begin(savedSSID.c_str(), savedPassword.c_str());

    while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      Serial.print(".");
      wifi_connecy_count++;
      if (wifi_connecy_count > 20) {
        writeStringToEEPROM(0, "0");
        writeStringToEEPROM(32, "0");
        ESP.restart();
      }
    }

    Serial.println("");
    Serial.println("Connected to WiFi");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  }

  // เริ่มต้นการตั้งค่าเซอร์โวและมอเตอร์
  gripperServo.attach(13);
  gripperServo.write(180);
  setStepperSpeedAndAcceleration(lastSpeed, lastAcceleration);

  // setZero();
  //setHomePosition();
  server.enableCORS();
  // เริ่มต้น Web Server
  server.on("/", handleMain);          // หน้าเว็บหลัก
  server.on("/index", handleRoot);     // หน้าเว็บหลัก
  server.on("/joint", handlejoint);    // หน้าเว็บหลัก
  server.on("/move", handleMove);      // รับคำสั่งการเคลื่อนที่
  server.on("/movejoint", movejoint);  // รับคำสั่งการเคลื่อนที่
  server.on("/sethomevalue", setHomeValue);
  server.on("/gohomevalue", goHomeValue);
  server.on("/gcode", HTTP_GET, handleGcodePage);     // หน้า input G-code
  server.on("/gcode", HTTP_POST, handleGcodeSubmit);  // รับข้อมูล G-code
  server.on("/setup", handleSetup);                   // หน้าเว็บหลัก
  server.on("/setuparm", handleSetupPage);            // หน้าเว็บหลัก
  server.on("/save-setup", handleSaveWiFi);
  server.on("/save-param", handleSaveSetupArm);
  server.on("/controlGripper", handleGripper);
  server.on("/delay", handleDelay);
  server.on("/for", handleFor);
  server.on("/home", handleHome);
  server.on("/zero", handleZero);
  server.on("/ota", handleOta);  //
  server.on("/output", handleOutput);
  server.on("/if", handleIf);
  /*handling uploading firmware file */
  server.on(
    "/update", HTTP_POST, []() {
      server.sendHeader("Connection", "close");
      server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
      ESP.restart();
    },
    []() {
      HTTPUpload& upload = server.upload();
      if (upload.status == UPLOAD_FILE_START) {
        Serial.printf("Update: %s\n", upload.filename.c_str());
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {  //start with max available size
          Update.printError(Serial);
        }
      } else if (upload.status == UPLOAD_FILE_WRITE) {
        /* flashing firmware to ESP*/
        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
          Update.printError(Serial);
        }
      } else if (upload.status == UPLOAD_FILE_END) {
        if (Update.end(true)) {  //true to set the size to the current progress
          Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
        } else {
          Update.printError(Serial);
        }
      }
    });
  server.begin();
  Serial.println("Web server started");
  // ตั้งค่า Timer ให้เรียกใช้ `onTimer` ทุกๆ 1 ms
  timer = timerBegin(0, 80, true);  // Timer 0, Prescaler 80, count up
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 1000, true);  // เรียกทุกๆ 1000 ticks = 1 ms
  timerAlarmEnable(timer);             // เปิดใช้งาน Timer
}

void loop() {
  serial();
  if (!commandQueue.empty()) {
    Command cmd = commandQueue.front();
    if (!commandQueue.empty()) {

      if (cmd.type == "move") {
        executeCommand(cmd);
      } else if (cmd.type == "delay") {
        delay(cmd.theta1);
      } else if (cmd.type == "Gripper") {
        gripperServo.write(cmd.theta1);
      } else if (cmd.type == "home") {
        setHomeArm();
      } else if (cmd.type == "zero") {
        setZero();
      } else if (cmd.type == "output") {

      } else {
        // Handle other cases if needed
      }
    }
    commandQueue.pop();
  }



  // จัดการคำร้องขอ HTTP
  server.handleClient();
}
