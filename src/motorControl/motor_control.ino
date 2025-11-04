#include <Arduino.h>
#include <ArduinoJson.h>
#include <ModbusMaster.h>

//================================================================
// 1. Modbus 寄存器地址 (根據手冊)
//================================================================
// 寫入地址
const int ADDR_RESET_FAULT      = 2000;
const int ADDR_MOTOR_A_STATE    = 2002;
const int ADDR_MOTOR_B_STATE    = 2003;
const int ADDR_MOTOR_A_DIR      = 2004;
const int ADDR_MOTOR_B_DIR      = 2005;
const int ADDR_MOTOR_A_SPEED_SP = 2006; // A電機轉速設定點
const int ADDR_MOTOR_B_SPEED_SP = 2007; // B電機轉速設定點

// 讀取地址
const int ADDR_MOTOR_A_SPEED_PV = 1002; // A電機轉速實際值
const int ADDR_MOTOR_B_SPEED_PV = 1003; // B電機轉速實際值
const int ADDR_FAULT_CODE       = 1005;

//================================================================
// 2. 機器人與驅動器參數
//================================================================
const float WHEEL_RADIUS      = 0.065;  // 輪半徑 (m) - 請根據您的機器人修改
const float WHEEL_SEPARATION  = 0.381;  // 輪距 (m) - 請根據您的機器人修改
const float MIN_RPM           = 100.0;
const float MAX_RPM           = 3000.0;
const uint8_t SLAVE_ID        = 1;

// *** 新增：定義 DE/RE 控制接腳 ***
const int DE_RE_PIN = 4;

//================================================================
// 3. 全域變數
//================================================================
ModbusMaster node;
String serial_buffer = "";

// 里程計變數
float odom_x = 0.0;
float odom_y = 0.0;
float odom_theta = 0.0;
unsigned long last_odom_update_time = 0;

// 儲存最後一次命令的方向，用於里程計計算
int last_dir_a = 0; // 0 for fwd, 1 for rev
int last_dir_b = 0; // 0 for fwd, 1 for rev

//================================================================
// *** 新增：Modbus 發送前後的回呼函式 ***
//================================================================
void preTransmission() {
  digitalWrite(DE_RE_PIN, HIGH);
}

void postTransmission() {
  delayMicroseconds(100); // 確保數據發送完成
  digitalWrite(DE_RE_PIN, LOW);
}

//================================================================
// 4. Setup
//================================================================
void processRosCommand(String& json_str);
void calculateAndSendOdometry();
void setup() {
  // 初始化與 ROS (Python) 的通訊
  Serial.begin(115200);

  // *** 新增：初始化 DE/RE 接腳 ***
  pinMode(DE_RE_PIN, OUTPUT);
  digitalWrite(DE_RE_PIN, LOW); // 預設為接收模式

  // 初始化與 RS-485 馬達驅動器的通訊
  Serial2.begin(115200, SERIAL_8N1, 16, 17);
  
  node.begin(SLAVE_ID, Serial2);

  // *** 新增：設定回呼函式 ***
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);

  // 啟用馬達
  node.writeSingleRegister(ADDR_MOTOR_A_STATE, 1);
  node.writeSingleRegister(ADDR_MOTOR_B_STATE, 1);

  last_odom_update_time = millis();
  Serial.println("ESP32 Motor Controller Initialized and Ready (with DE/RE control).");
}

//================================================================
// 5. Loop 
//================================================================
void loop() {
  // 處理來自 ROS 的指令
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      processRosCommand(serial_buffer);
      serial_buffer = "";
    } else {
      serial_buffer += c;
    }
  }

  // 定期讀取速度並發送里程計數據 (每 50ms)
  if (millis() - last_odom_update_time > 50) {
    calculateAndSendOdometry();
    last_odom_update_time = millis();
  }
}

//================================================================
// 6. 功能函數
//================================================================

/**
 * @brief 處理從 ROS 來的 JSON 指令
 */
void processRosCommand(String& json_str) {
  StaticJsonDocument<200> doc;
  // 移除 JSON 字串前後的標記
  json_str.replace("::", "");
  DeserializationError error = deserializeJson(doc, json_str);

  if (error) {
    Serial.print("JSON deserialize failed: ");
    Serial.println(error.c_str());
    return;
  }

  if (doc["type"] == "motor_cmd") {
    float left_vel_ms = doc["left_vel"];  // 單位: m/s
    float right_vel_ms = doc["right_vel"]; // 單位: m/s

    // --- 控制 A 電機 (左輪) ---
    last_dir_a = (left_vel_ms >= 0) ? 0 : 1; // 0=正轉, 1=反轉
    float abs_rpm_a = abs(left_vel_ms) / (2 * PI * WHEEL_RADIUS) * 60.0;
    uint16_t target_rpm_a = 0;
    if (abs_rpm_a >= MIN_RPM) {
      target_rpm_a = constrain(abs_rpm_a, MIN_RPM, MAX_RPM);
    }
    
    node.writeSingleRegister(ADDR_MOTOR_A_DIR, last_dir_a);
    node.writeSingleRegister(ADDR_MOTOR_A_SPEED_SP, target_rpm_a);

    // --- 控制 B 電機 (右輪) ---
    last_dir_b = (right_vel_ms >= 0) ? 0 : 1; // 0=正轉, 1=反轉
    float abs_rpm_b = abs(right_vel_ms) / (2 * PI * WHEEL_RADIUS) * 60.0;
    uint16_t target_rpm_b = 0;
    if (abs_rpm_b >= MIN_RPM) {
      target_rpm_b = constrain(abs_rpm_b, MIN_RPM, MAX_RPM);
    }

    node.writeSingleRegister(ADDR_MOTOR_B_DIR, last_dir_b);
    node.writeSingleRegister(ADDR_MOTOR_B_SPEED_SP, target_rpm_b);
  }
}

/**
 * @brief 計算里程計並回傳給 ROS
 */
void calculateAndSendOdometry() {
  uint8_t result;
  float rpm_a = 0;
  float rpm_b = 0;

  // 讀取兩個馬達的實際轉速
  result = node.readInputRegisters(ADDR_MOTOR_A_SPEED_PV, 2);
  if (result == node.ku8MBSuccess) {
    rpm_a = node.getResponseBuffer(0);
    rpm_b = node.getResponseBuffer(1);
  } else {
    // 讀取失敗，直接返回
    return;
  }

  // 將 RPM 轉換回 m/s，並根據上次的方向指令加上符號
  float vel_a_ms = (rpm_a / 60.0) * (2 * PI * WHEEL_RADIUS);
  float vel_b_ms = (rpm_b / 60.0) * (2 * PI * WHEEL_RADIUS);
  if (last_dir_a == 1) vel_a_ms = -vel_a_ms;
  if (last_dir_b == 1) vel_b_ms = -vel_b_ms;

  // 計算機器人線速度 (vx) 和角速度 (vth)
  float vx = (vel_a_ms + vel_b_ms) / 2.0;
  float vth = (vel_b_ms - vel_a_ms) / WHEEL_SEPARATION;

  // 透過積分更新里程計位置和姿態
  float dt = (millis() - last_odom_update_time) / 1000.0;
  odom_x += vx * cos(odom_theta) * dt;
  odom_y += vx * sin(odom_theta) * dt;
  odom_theta += vth * dt;

  // 準備 JSON 回傳給 Python
  StaticJsonDocument<300> odom_doc;
  odom_doc["type"] = "odometry";
  odom_doc["x"] = odom_x;
  odom_doc["y"] = odom_y;
  odom_doc["theta"] = odom_theta;
  odom_doc["vx"] = vx;
  odom_doc["vy"] = 0.0; // 差速驅動 vy 為 0
  odom_doc["vth"] = vth;

  String output;
  serializeJson(odom_doc, output);

  // 發送數據，並用 "::" 包圍
  Serial.print("::");
  Serial.print(output);
  Serial.println("::");
}