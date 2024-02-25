// LCD 2004A
#include <LiquidCrystal_I2C.h>
#define I2C_SDA 33
#define I2C_SCL 32
LiquidCrystal_I2C lcd(0x27, 20, 4);

#include <Wire.h>
#include <Adafruit_INA219.h>

// 伺服馬達
static const int servoPin = 2;
const int pwmChanneServo = 8; // PWM 信道
const int freq_servo = 50; // PWM 频率設為 50 Hz
const int resolution_servo = 8; // PWM 分辨率（8 位）

// ESP32：https://dl.espressif.com/dl/package_esp32_index.json
#include <BluetoothSerial.h>
BluetoothSerial SerialBT;
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
boolean confirmRequestPending = false;

// 宣告任務Task1
TaskHandle_t Task1;

// L298N設定
const int motorPinA1 = 4;
const int motorPinA2 = 16;
const int motorPinAP = 17;
const int motorPinB1 = 5;
const int motorPinB2 = 18;
const int motorPinBP = 19;
const int motorPinC1 = 21;
const int motorPinC2 = 22;
const int motorPinCP = 23;
const int freq = 10000; // PWM 频率
const int pwmChannelA = 2; // PWM 信道 A
const int pwmChannelB = 4; // PWM 信道 B
const int pwmChannelC = 6; // PWM 信道 C
const int resolution = 8; // PWM 分辨率（8 位）

// 限位開關設定
const int resSpeed = 250; // 復位速度設定
const int stopSpeed = 180; // 固定用設定
const int limitSwitchSpeed = 10; // 減速速度設定
const int limitSwitchUp = 35; // 上限位開關(碰撞為低態)
const int limitSwitchDn = 34; // 下限位開關(碰撞為低態)

// 蜂鳴器
const int buzzerHz[6] = {1580, 1970, 2360, 1970, 2360, 9098};
const int buzzerPin = 27;
const int pwmChanneBuzzer = 0; // PWM 信道
const int freq_Buzzer = 2000; // PWM 频率
const int resolution_Buzzer = 8; // PWM 分辨率（8 位）

// 繼電器
const int relay_wind = 26; // 風力能煞車
const int relay_lm298 = 25; // 超級電容動力系統

// 電源監控模組
Adafruit_INA219 ina219_sun(0x40);
Adafruit_INA219 ina219_wind(0x41);
Adafruit_INA219 ina219_rc(0x44);
Adafruit_INA219 ina219_bat(0x45);

// 藍芽發送資料
/*
   藍芽資料發送格式："00.0,0.000,00,00.0,0.000,00,00.000,0.00,00,00.000,0.00,00,0000,65"
                  "電壓(太陽能),電流(太陽能),狀態(太陽能),電壓(風力),電流(風力),狀態(風力),電壓(電池),電流(電池),狀態(電池),電壓(電容),電流(電容),狀態(電容),系統狀態,驗算碼"
   藍芽資料接收格式："0,000,000,0,0,16"
                  "前/後,速度,轉向角度,上/下,始能,驗算碼"
*/
// 發送
String bleWireData[14] = {"0"};
// 接收
int bleReadData[6] = {0};

// 狀態變數
int lockerState = 0; // 儲物倉馬達狀態 0未知, 1上升(停), 2下降(停), 3上升(動), 4下降(動)
long previousTime1 = 0; // 多工時間
long previousTime2 = 0; // 多工時間
long previousTime3 = 0; // 多工時間
bool bleConnect = false;
bool servoPower = true;

void setup() {
  Serial.begin(115200);
  Serial.println("裝置初始化中");

  // 初始化 ESP32 多核心執行續
  xTaskCreatePinnedToCore(Task1_senddata, "Task1", 10000, NULL, 0, &Task1, 0);

  // 初始化蜂鳴器
  ledcSetup(pwmChanneBuzzer, freq_Buzzer, resolution_Buzzer);
  ledcAttachPin(buzzerPin, pwmChanneBuzzer);
  for (int i = 0; i < 6; i++) {
    ledcWriteTone(0, buzzerHz[i]);  // 輸出聲音
    delay(150);
    ledcWriteTone(0, 0);  // 關閉聲音
    delay(50);
  }

  // 初始化 LCD
  Wire.begin(I2C_SDA, I2C_SCL);
  lcd.begin();
  lcd.setCursor(0, 0);
  lcd.print("===ASME-SPDC-2023===");
  lcd.setCursor(0, 1);
  lcd.print("BlueTooth:Wait...");
  lcd.setCursor(0, 2);
  lcd.print("System:Prepare");

  // 初始化繼電器
  pinMode(relay_wind, OUTPUT);
  pinMode(relay_lm298, OUTPUT);
  digitalWrite(relay_wind, HIGH);
  digitalWrite(relay_lm298, HIGH);

  // 初始化伺服馬達
  ledcSetup(pwmChanneServo, freq_servo, resolution_servo);
  ledcAttachPin(servoPin, pwmChanneServo);
  delay(90);
  ledcWrite(pwmChanneServo, calculatePWM(90));

  // 初始化 L298N 引腳 A 左後馬達, B 右後馬達, C 儲物倉馬達
  pinMode(motorPinA1, OUTPUT);
  pinMode(motorPinA2, OUTPUT);
  pinMode(motorPinB1, OUTPUT);
  pinMode(motorPinB2, OUTPUT);
  pinMode(motorPinC1, OUTPUT);
  pinMode(motorPinC2, OUTPUT);
  digitalWrite(motorPinA1, LOW);
  digitalWrite(motorPinA2, LOW);
  digitalWrite(motorPinB1, LOW);
  digitalWrite(motorPinB2, LOW);
  digitalWrite(motorPinC1, LOW);
  digitalWrite(motorPinC2, LOW);

  // 初始化 LEDC 對象並綁定到L298N輸出引腳上
  ledcSetup(pwmChannelA, freq, resolution);
  ledcAttachPin(motorPinAP, pwmChannelA);
  ledcSetup(pwmChannelB, freq, resolution);
  ledcAttachPin(motorPinBP, pwmChannelB);
  ledcSetup(pwmChannelC, freq, resolution);
  ledcAttachPin(motorPinCP, pwmChannelC);

  // 初始化感測器
  pinMode(limitSwitchUp, INPUT_PULLUP);
  pinMode(limitSwitchDn, INPUT_PULLUP);

  // 初始化儲物倉
  lockerReset();

  // 初始化每個 INA219
  ina219_sun.begin();
  ina219_wind.begin();
  ina219_rc.begin();
  ina219_bat.begin();

  // 設置 INA219 的電壓電流範圍
  ina219_sun.setCalibration_32V_2A();
  ina219_wind.setCalibration_32V_2A();
  ina219_rc.setCalibration_32V_2A();
  ina219_bat.setCalibration_32V_2A();

  // 初始化藍芽
  SerialBT.enableSSP();
  SerialBT.onConfirmRequest(BTConfirmRequestCallback);
  SerialBT.onAuthComplete(BTAuthCompleteCallback);
  SerialBT.begin("ESP32_BT_MSG"); // 藍芽裝置名稱
  SerialBT.setPin("594666"); // 設置 SSP 安全層級
  SerialBT.setTimeout(20); // 緩衝區等待時間
  SerialBT.register_callback(callback); // 建立callback事件

  ledcWriteTone(0, 1970);  // 輸出聲音
  delay(100);
  ledcWriteTone(0, 0);  // 關閉聲音
  delay(50);
  ledcWriteTone(0, 1970);  // 輸出聲音
  delay(100);
  ledcWriteTone(0, 0);  // 關閉聲音
  delay(50);
  lcd.setCursor(0, 1);
  lcd.print("BlueTooth:Disconnect");
  lcd.setCursor(0, 2);
  lcd.print("System:Ok           ");
  Serial.println("初始化完成");
}

void loop() {

  unsigned long currentTime = millis();  // 將當前的時間存入變數currentTime
  if (currentTime - previousTime1 > 500) { // 如果當前時間扣除前一次保留時間超過間隔時間(ms)就進入執行程式
    ina219 ();
    previousTime1 = currentTime;
  }

  if (currentTime - previousTime2 > 1100) { // 如果當前時間扣除前一次保留時間超過間隔時間(ms)就進入執行程式
    if (bleConnect) {
      lcd.setCursor(0, 1);
      lcd.print("BlueTooth:Connect   ");
    } else if (confirmRequestPending) {
    } else {
      lcd.setCursor(0, 1);
      lcd.print("BlueTooth:Disconnect");
      lcd.setCursor(0, 3);
      lcd.print("                    ");
    }
    previousTime2 = currentTime;
  }

  if (currentTime - previousTime3 > 5400) { // 如果當前時間扣除前一次保留時間超過間隔時間(ms)就進入執行程式
    ina219 ();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("===ASME-SPDC-2023===");
    previousTime3 = currentTime;
  }

  if (confirmRequestPending)
  {
    Serial.println("裝置配對中");
    int i = 0;
    while (i < 5000) {
      if (!digitalRead(limitSwitchDn)) {
        SerialBT.confirmReply(true);
        lcd.setCursor(0, 1);
        lcd.print("BlueTooth:Perfect!!  ");
        Serial.println("配對成功");
        break;
      }
      i++;
      delay(1);
    }
    if (i > 5000) {
      SerialBT.confirmReply(false);
      lcd.setCursor(0, 1);
      lcd.print("BlueTooth:Failed... ");
      Serial.println("配對失敗");
      confirmRequestPending = false;
    }
  }

  // 藍芽訊息接收
  if (SerialBT.available()) {
    String msg = SerialBT.readString();
    if (msg.length() == 16) {
      // Serial.println(msg);
      parseIntArray(msg, 6);
      lcd.setCursor(0, 3);
      lcd.print("DT:" + msg);
    }
  }

  // 檢測到狀態更新
  if (bleReadData[4] == 1) {
    bleReadData[4] = 0;
    if (servoPower) {
      // 伺服馬達角度變更
      if (bleReadData[2] >= 0 && bleReadData[2] <= 180) // 限制輸入角度
        ledcWrite(pwmChanneServo, calculatePWM(180 - bleReadData[2]));
      else
        Serial.println("角度數據異常");
    }
    /*
      L298N
      差速控制
      計算左右馬達速度
    */
    int leftSpeed = 0, rightSpeed = 0; // 左右輪速度
    if (bleReadData[1] != 0) {
      int motorSpeed = bleReadData[1]; // 原始速度
      int angle = bleReadData[2]; // 轉向角度
      if (angle < 80) {
        // 左轉，調整右馬達速度
        rightSpeed = map(angle, 0, 87, 0, motorSpeed);
        leftSpeed = motorSpeed;
      } else if (angle > 100) {
        // 右轉，調整左馬達速度
        leftSpeed = map(angle, 93, 180, 0, motorSpeed);
        rightSpeed = motorSpeed;
      } else {
        // 直行
        leftSpeed = rightSpeed = motorSpeed;
      }
    }

    /*
      L298N
      進退控制
    */
    switch (bleReadData[0]) {
      // 前進
      case 1 :
        digitalWrite(motorPinA1, HIGH);
        digitalWrite(motorPinA2, LOW);
        // 左馬達速度
        ledcWrite(pwmChannelA, map(leftSpeed, 0, 100, 0, 255));
        digitalWrite(motorPinB1, HIGH);
        digitalWrite(motorPinB2, LOW);
        // 右馬達速度
        ledcWrite(pwmChannelB, map(rightSpeed, 0, 100, 0, 255));
        break;
      // 後退
      case 2 :
        digitalWrite(motorPinA1, LOW);
        digitalWrite(motorPinA2, HIGH);
        // 左馬達速度
        ledcWrite(pwmChannelA, map(leftSpeed, 0, 100, 0, 255));
        digitalWrite(motorPinB1, LOW);
        digitalWrite(motorPinB2, HIGH);
        // 右馬達速度
        ledcWrite(pwmChannelB, map(rightSpeed, 0, 100, 0, 255));
        break;
      // 停止
      default :
        digitalWrite(motorPinA1, LOW);
        digitalWrite(motorPinA2, LOW);
        // 左馬達速度
        ledcWrite(pwmChannelA, 0);
        digitalWrite(motorPinB1, LOW);
        digitalWrite(motorPinB2, LOW);
        // 右馬達速度
        ledcWrite(pwmChannelB, 0);
        break;
    }

    /*
      儲物倉
      上下控制
      lockerState 儲物倉馬達狀態 0未知, 1上升(停), 2下降(停), 3上升(動), 4下降(動)
    */
    switch (bleReadData[3]) {
      case 1:
        if (lockerState == 2) {
          // 向上轉，直到觸發上限位開關
          digitalWrite(motorPinC1, HIGH);
          digitalWrite(motorPinC2, LOW);
          // 速度設定
          ledcWrite(pwmChannelC, resSpeed);
          int i = 0; // 計數
          while (digitalRead(limitSwitchUp)) {
            lockerState = 1;
            if (i > 3000) {
              Serial.println("儲物倉異常，上限位開關不觸發");
              lcd.setCursor(0, 2);
              lcd.print("System:LockerERROR U");
              lockerState = 0;
              break;
            }
            i++;
            delay(1);
          }
          ledcWrite(pwmChannelC, stopSpeed);
        } else if (lockerState == 0) {
          // 初始化儲物倉
          lockerReset();
        }
        break;
      case 2:
        if (lockerState == 1) {
          // 向下轉，直到觸發下限位開關
          digitalWrite(motorPinC1, HIGH);
          digitalWrite(motorPinC2, LOW);
          // 速度設定
          ledcWrite(pwmChannelC, limitSwitchSpeed);
          int i = 0;
          while (digitalRead(limitSwitchDn)) {
            lockerState = 2;
            if (i > 3000) {
              Serial.println("儲物倉異常，下限位開關不觸發");
              lcd.setCursor(0, 2);
              lcd.print("System:LockerERROR D");
              lockerState = 0;
              break;
            }
            i++;
            delay(1);
          }
          digitalWrite(motorPinC1, LOW);
          digitalWrite(motorPinC2, LOW);
          ledcWrite(pwmChannelC, 0);
        } else if (lockerState == 0) {
          // 初始化儲物倉
          lockerReset();
        }
        break;
    }
  }
  // delay(20);


}

// 多核心執行續
void Task1_senddata(void * pvParameters ) {
  for (;;) {
    String msg = bleWireData[0];
    for (int i = 1; i < 14 - 2; i++) msg = msg + "," + bleWireData[i];
    msg = msg + "," + "0000" + "," + "65";
    SerialBT.print(msg);
    delay(1000);
  }
}

// INA219 傳感器
void ina219 () {
  // 讀取每個傳感器狀態
  // 太陽能
  float shuntVoltage_sun = ina219_sun.getShuntVoltage_mV(); // 分流電阻電壓
  float busVoltage_sun = ina219_sun.getBusVoltage_V(); // 電源電壓
  float current_sun = ina219_sun.getCurrent_mA(); // 電路電流
  float loadVoltage_sun = busVoltage_sun + (shuntVoltage_sun / 1000); // 負載電壓
  // 風力能
  float shuntVoltage_wind = ina219_wind.getShuntVoltage_mV(); // 分流電阻電壓
  float busVoltage_wind = ina219_wind.getBusVoltage_V(); // 電源電壓
  float current_wind = ina219_wind.getCurrent_mA(); // 電路電流
  float loadVoltage_wind = busVoltage_wind + (shuntVoltage_wind / 1000); // 負載電壓
  // 超級電容
  float shuntVoltage_rc = ina219_rc.getShuntVoltage_mV(); // 分流電阻電壓
  float busVoltage_rc = ina219_rc.getBusVoltage_V(); // 電源電壓
  float current_rc = ina219_rc.getCurrent_mA(); // 電路電流
  float loadVoltage_rc = busVoltage_rc + (shuntVoltage_rc / 1000); // 負載電壓
  // 鋰電電池
  float shuntVoltage_bat = ina219_bat.getShuntVoltage_mV(); // 分流電阻電壓
  float busVoltage_bat = ina219_bat.getBusVoltage_V(); // 電源電壓
  float current_bat = ina219_bat.getCurrent_mA(); // 電路電流
  float loadVoltage_bat = busVoltage_bat + (shuntVoltage_bat / 1000); // 負載電壓

  // 將數值存入陣列
  // 太陽能 電壓
  char loadVoltage_sun_Str[8];
  sprintf(loadVoltage_sun_Str, "%04.1f", abs(loadVoltage_sun));
  bleWireData[0] = String(loadVoltage_sun_Str);
  // 太陽能 電流
  char current_sun_Str[11];
  sprintf(current_sun_Str, "%05.3f", abs(current_sun) * 0.001);
  bleWireData[1] = String(current_sun_Str);
  // 判斷是否加上負號
  if (current_sun < 0) {
    if (bleWireData[2] == "00") bleWireData[2] = "10";
    else if (bleWireData[2] == "01") bleWireData[2] = "11";
    else bleWireData[2] = "10";
  } else {
    if (bleWireData[2] == "10") bleWireData[2] = "00";
    else if (bleWireData[2] == "11") bleWireData[2] = "01";
    else bleWireData[2] = "00";
  }
  // 風力能 電壓
  char loadVoltage_wind_Str[8];
  sprintf(loadVoltage_wind_Str, "%04.1f", abs(loadVoltage_wind));
  bleWireData[3] = String(loadVoltage_wind_Str);
  // 風力能 電流
  char current_wind_Str[11];
  sprintf(current_wind_Str, "%05.3f", abs(current_wind) * 0.001);
  bleWireData[4] = String(current_wind_Str);
  // 判斷是否加上負號
  if (current_wind < 0) {
    if (bleWireData[5] == "00") bleWireData[5] = "10";
    else if (bleWireData[5] == "01") bleWireData[5] = "11";
    else bleWireData[5] = "10";
  } else {
    if (bleWireData[5] == "10") bleWireData[5] = "00";
    else if (bleWireData[5] == "11") bleWireData[5] = "01";
    else bleWireData[5] = "00";
  }
  // 超級電容 電壓
  char loadVoltage_rc_Str[12];
  sprintf(loadVoltage_rc_Str, "%06.3f", abs(loadVoltage_rc));
  bleWireData[9] = String(loadVoltage_rc_Str);
  // 超級電容 電流
  char current_rc_Str[9];
  sprintf(current_rc_Str, "%04.2f", abs(current_rc) * 0.001);
  bleWireData[10] = String(current_rc_Str);
  // 判斷是否加上負號
  if (current_rc < 0) {
    if (loadVoltage_rc > 0.1)
      if (abs(current_rc) * 0.001 > 0.05) {
        lcd.setCursor(0, 2);
        lcd.print("System:<RC>Charging ");
      }
    if (bleWireData[11] == "00") bleWireData[11] = "10";
    else if (bleWireData[11] == "01") bleWireData[11] = "11";
    else bleWireData[11] = "10";
  } else {
    lcd.setCursor(0, 2);
    lcd.print("System:Standby      ");
    if (bleWireData[11] == "10") bleWireData[11] = "00";
    else if (bleWireData[11] == "11") bleWireData[11] = "01";
    else bleWireData[11] = "00";
  }
  // 鋰電電池 電壓
  char loadVoltage_bat_Str[12];
  sprintf(loadVoltage_bat_Str, "%06.3f", abs(loadVoltage_bat));
  bleWireData[6] = String(loadVoltage_bat_Str);
  // 鋰電電池 電流
  char current_bat_Str[9];
  sprintf(current_bat_Str, "%04.2f", abs(current_bat) * 0.001);
  bleWireData[7] = String(current_bat_Str);
  // 判斷是否加上負號
  if (current_bat < 0) {
    if (bleWireData[8] == "00") bleWireData[8] = "10";
    else if (bleWireData[8] == "01") bleWireData[8] = "11";
    else bleWireData[8] = "10";
  } else {
    if (bleWireData[8] == "10") bleWireData[8] = "00";
    else if (bleWireData[8] == "11") bleWireData[8] = "01";
    else bleWireData[8] = "00";
  }

  // 風力能煞車設定
  if (loadVoltage_wind > 9) {
    ledcWriteTone(0, 2000);  // 輸出聲音
    digitalWrite(relay_wind, LOW);
  }
  else if (busVoltage_rc < 8) {
    ledcWriteTone(0, 0);  // 關閉聲音
    digitalWrite(relay_wind, HIGH);
  }

  // 超級電容動力系統設定
  if (loadVoltage_rc >= 3) {
    digitalWrite(relay_lm298, LOW);
    servoPower = true;
  }
  else {
    digitalWrite(relay_lm298, HIGH);
    servoPower = false;
  }

}

// 伺服馬達控制
int calculatePWM(int degree) // 輸出 PWM 佔空比
{
  float deadZone = 6.4;
  float maxs = 32;

  if (degree < 0)
    degree = 0;
  if (degree > 200)
    degree = 180;
  return (int)(((maxs - deadZone) / 180) * degree + deadZone);
}

// 儲物倉歸位
void lockerReset() {
  // 檢測儲物倉狀態
  Serial.println("儲物倉正在復位");
  if (!digitalRead(limitSwitchUp) && digitalRead(limitSwitchDn)) { // 若上限位已觸發，進行復位
    Serial.println("上限位觸發");
    // 向下轉，直到離開限位開關
    digitalWrite(motorPinC1, LOW);
    digitalWrite(motorPinC2, HIGH);
    // 復位速度設定
    ledcWrite(pwmChannelC, resSpeed);
    int i = 0;
    while (!digitalRead(limitSwitchUp) && !(Serial.available() > 0)) {
      if (i < 5000) {
        i++;
      } else if (i == 5000) {
        Serial.println("儲物倉異常，上限位開關或馬達故障");
        digitalWrite(motorPinC1, LOW);
        digitalWrite(motorPinC2, LOW);
        ledcWrite(pwmChannelC, 0);
        ledcWriteTone(0, 1580);  // 輸出聲音
        lcd.setCursor(0, 2);
        lcd.print("System:LockerERROR U");
        i++;
      }
      delay(1);
    }
    ledcWriteTone(0, 0);  // 關閉聲音
    i = 0;
    // 向上轉，直到觸發限位開關
    digitalWrite(motorPinC1, HIGH);
    digitalWrite(motorPinC2, LOW);
    // 復位速度設定
    ledcWrite(pwmChannelC, resSpeed);
    while (digitalRead(limitSwitchUp) && !(Serial.available() > 0)) {
      if (i < 5000) {
        i++;
      } else if (i == 5000) {
        Serial.println("儲物倉異常，上限位開關或馬達故障");
        digitalWrite(motorPinC1, LOW);
        digitalWrite(motorPinC2, LOW);
        ledcWrite(pwmChannelC, 0);
        ledcWriteTone(0, 1580);  // 輸出聲音
        lcd.setCursor(0, 2);
        lcd.print("System:LockerERROR U");
        i++;
      }
      delay(1);
    }
    ledcWriteTone(0, 0);  // 關閉聲音
    ledcWrite(pwmChannelC, stopSpeed);
  } else if (digitalRead(limitSwitchUp) && !digitalRead(limitSwitchDn)) { // 若下限位已觸發，進行復位
    Serial.println("下限位觸發");
    // 向上轉，直到觸發上限位開關
    digitalWrite(motorPinC1, HIGH);
    digitalWrite(motorPinC2, LOW);
    // 復位速度設定
    ledcWrite(pwmChannelC, resSpeed);
    int i = 0;
    while (digitalRead(limitSwitchUp) && !(Serial.available() > 0)) {
      if (i < 5000) {
        i++;
      } else if (i == 5000) {
        Serial.println("儲物倉異常，上限位開關或馬達故障");
        digitalWrite(motorPinC1, LOW);
        digitalWrite(motorPinC2, LOW);
        ledcWrite(pwmChannelC, 0);
        ledcWriteTone(0, 1580);  // 輸出聲音
        lcd.setCursor(0, 2);
        lcd.print("System:LockerERROR U");
        i++;
      }
      delay(1);
    }
    ledcWriteTone(0, 0);  // 關閉聲音
    ledcWrite(pwmChannelC, stopSpeed);
  } else if (digitalRead(limitSwitchUp) && digitalRead(limitSwitchDn)) { // 上下限位皆未觸發，進行復位
    Serial.println("空位觸發");
    // 向下轉，直到觸發下限位開關
    digitalWrite(motorPinC1, LOW);
    digitalWrite(motorPinC2, HIGH);
    ledcWrite(pwmChannelC, resSpeed);
    delay(5);
    digitalWrite(motorPinC1, HIGH);
    digitalWrite(motorPinC2, LOW);
    // 復位速度設定
    ledcWrite(pwmChannelC, limitSwitchSpeed);
    int i = 0;
    while (digitalRead(limitSwitchDn) && !(Serial.available() > 0)) {
      if (i < 5000) {
        i++;
      } else if (i == 5000) {
        Serial.println("儲物倉異常，下限位開關或馬達故障");
        digitalWrite(motorPinC1, LOW);
        digitalWrite(motorPinC2, LOW);
        ledcWrite(pwmChannelC, 0);
        ledcWriteTone(0, 1580);  // 輸出聲音
        lcd.setCursor(0, 2);
        lcd.print("System:LockerERROR D");
        i++;
      }
      delay(1);
    }
    ledcWriteTone(0, 0);  // 關閉聲音
    i = 0;
    // 向上轉，直到觸發限位開關
    digitalWrite(motorPinC1, HIGH);
    digitalWrite(motorPinC2, LOW);
    // 復位速度設定
    ledcWrite(pwmChannelC, resSpeed);
    while (digitalRead(limitSwitchUp) && !(Serial.available() > 0)) {
      if (i < 5000) {
        i++;
      } else if (i == 5000) {
        Serial.println("儲物倉異常，上限位開關或馬達故障");
        digitalWrite(motorPinC1, LOW);
        digitalWrite(motorPinC2, LOW);
        ledcWrite(pwmChannelC, 0);
        ledcWriteTone(0, 1580);  // 輸出聲音
        lcd.setCursor(0, 2);
        lcd.print("System:LockerERROR U");
        i++;
      }
      delay(1);
    }
    ledcWriteTone(0, 0);  // 關閉聲音
    ledcWrite(pwmChannelC, stopSpeed);
  }
  lockerState = 1; // 設定儲物倉馬達狀態為 1上升(停)
}

// 將字串轉為整數存到陣列
void parseIntArray(String str, int arraySize) {
  int index = 0; // 初始化整數陣列的索引
  char *pch; // 宣告一個 char 指標
  pch = strtok((char*)str.c_str(), ","); // 將字串 str 以逗號為分隔符進行拆分，pch 指向分隔符前的子字串
  while (pch != NULL && index < arraySize) { // 循環處理分隔符分割的子字串，並檢查陣列是否已滿
    bleReadData[index] = atoi(pch); // 將子字串轉換為整數並存儲在整數陣列中
    index++; // 將整數陣列的索引加 1
    pch = strtok(NULL, ","); // 繼續處理下一個分隔符分割的子字串，pch 指向分隔符前的子字串
  }
  delete pch;
}

// 藍芽SPP安全配對
// 配對碼
void BTConfirmRequestCallback(uint32_t numVal)
{
  confirmRequestPending = true;
  Serial.println((String)"BT:" + numVal);
  lcd.setCursor(0, 1);
  lcd.print((String)"BlueTooth:" + numVal + (String)"     ");
}

// 配對結果
void BTAuthCompleteCallback(boolean success)
{
  confirmRequestPending = false;
  if (success)
  {
    Serial.println("配對成功");
  }
  else
  {
    Serial.println("用戶拒絕配對&配對失敗");
  }
}

void callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
  // 藍芽連線成功
  if (event == ESP_SPP_SRV_OPEN_EVT) {
    bleConnect = true;
    ledcWriteTone(0, 1580);  // 輸出聲音
    delay(100);
    ledcWriteTone(0, 0);  // 關閉聲音
    delay(50);
    ledcWriteTone(0, 2360);  // 輸出聲音
    delay(100);
    ledcWriteTone(0, 0);  // 關閉聲音
    delay(50);
    Serial.println("裝置連線成功");
  }
  // 藍芽連線中斷
  if (event == ESP_SPP_CLOSE_EVT ) {
    bleConnect = false;
    ledcWriteTone(0, 2360);  // 輸出聲音
    delay(100);
    ledcWriteTone(0, 0);  // 關閉聲音
    delay(50);
    ledcWriteTone(0, 1580);  // 輸出聲音
    delay(100);
    ledcWriteTone(0, 0);  // 關閉聲音
    delay(50);
    Serial.println("裝置連線中斷");
  }
}
