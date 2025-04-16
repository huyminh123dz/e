#include <Arduino.h>
#include <BluetoothSerial.h>
#include <ESP32Servo.h>

// Bluetooth
BluetoothSerial SerialBT;

// === Khai báo motor dùng PWM & chiều ===
#define DIR_1_pin 23
#define PWM_1_pin 16

#define DIR_2_pin 22
#define PWM_2_pin 4

#define DIR_3_pin 18
#define PWM_3_pin 0

#define DIR_4_pin 17
#define PWM_4_pin 15

#define PWM_FREQ 1000
#define PWM_RES 8

int motor_speed = 255;
#define spd 125

// === Khai báo cảm biến dò line ===
#define SENSOR_1 27
#define SENSOR_2 26
#define SENSOR_3 25
#define SENSOR_4 33
#define SENSOR_5 32

const int threshold = 1000; // <-- đo thực tế rồi chỉnh
int sensor_1, sensor_2, sensor_3, sensor_4, sensor_5;
int vitri;
bool lineFollowing = false;

// === Servo ===
#define SERVO1_PIN 5
#define SERVO2_PIN 13
#define SERVO3_PIN 12

Servo servo1, servo2, servo3;
bool servo1State = false, servo2State = false, servo3State = false;

///////////////////////////////* các hàm con điều khiển */////////////////////////////////////////////////////////
void tien() {
    digitalWrite(DIR_1_pin, HIGH); analogWrite(PWM_1_pin,motor_speed);
    digitalWrite(DIR_2_pin, LOW);  analogWrite(PWM_2_pin, motor_speed);
    digitalWrite(DIR_3_pin, HIGH); analogWrite(PWM_3_pin, motor_speed);
    digitalWrite(DIR_4_pin, LOW);  analogWrite(PWM_4_pin, motor_speed);
}

void lui() {
    digitalWrite(DIR_1_pin, LOW);  analogWrite(PWM_1_pin, motor_speed);
    digitalWrite(DIR_2_pin, HIGH); analogWrite(PWM_2_pin, motor_speed);
    digitalWrite(DIR_3_pin, LOW);  analogWrite(PWM_3_pin, motor_speed);
    digitalWrite(DIR_4_pin, HIGH); analogWrite(PWM_4_pin, motor_speed);
}

void retrai() {
    digitalWrite(DIR_1_pin, HIGH); analogWrite(PWM_1_pin, motor_speed);
    digitalWrite(DIR_2_pin, LOW);  analogWrite(PWM_2_pin, motor_speed);
    digitalWrite(DIR_3_pin, HIGH); analogWrite(PWM_3_pin, motor_speed);
    digitalWrite(DIR_4_pin, LOW);  analogWrite(PWM_4_pin, motor_speed);
}

void rephai() {
    digitalWrite(DIR_1_pin, LOW);  analogWrite(PWM_1_pin, motor_speed);
    digitalWrite(DIR_2_pin, HIGH); analogWrite(PWM_2_pin, motor_speed);
    digitalWrite(DIR_3_pin, LOW);  analogWrite(PWM_3_pin, motor_speed);
    digitalWrite(DIR_4_pin, HIGH); analogWrite(PWM_4_pin, motor_speed);
}

void dung() {
    analogWrite(PWM_1_pin, 0);
    analogWrite(PWM_2_pin, 0);
    analogWrite(PWM_3_pin, 0);
    analogWrite(PWM_4_pin, 0);
}





void tien_1() {
  digitalWrite(Drv_In1_pin, 0);
  ledcWrite(Drv_In2_chn, spd-50);
  digitalWrite(Drv_In3_pin, 0);
  ledcWrite(Drv_In4_chn, spd-50);
}
void rephai_1() {
  digitalWrite(Drv_In1_pin, 0);
  ledcWrite(Drv_In2_chn, spd);
  digitalWrite(Drv_In3_pin, 0);
  ledcWrite(Drv_In4_chn, spd-30);
}
void retrai_1() {
  digitalWrite(Drv_In1_pin, 0);
  ledcWrite(Drv_In2_chn, spd-30);
  digitalWrite(Drv_In3_pin, 0);
  ledcWrite(Drv_In4_chn, spd);
}
void rephai1() {
  digitalWrite(Drv_In1_pin, 0);
  ledcWrite(Drv_In2_chn, spd);
  digitalWrite(Drv_In3_pin, 0);
  ledcWrite(Drv_In4_chn, spd-50);
}
void retrai1() {
  digitalWrite(Drv_In1_pin, 0);
  ledcWrite(Drv_In2_chn, spd-50);
  digitalWrite(Drv_In3_pin, 0);
  ledcWrite(Drv_In4_chn, spd);
}
void rephai2() {
  digitalWrite(Drv_In1_pin, 0);
  ledcWrite(Drv_In2_chn, spd);
  digitalWrite(Drv_In3_pin, 1);
  ledcWrite(Drv_In4_chn, spd);
}
void retrai2() {
  digitalWrite(Drv_In1_pin, 1);
  ledcWrite(Drv_In2_chn, spd);
  digitalWrite(Drv_In3_pin, 0);
  ledcWrite(Drv_In4_chn, spd);
}




// === Đọc cảm biến và xử lý dò line ===
void readSensor() {
  sensor_1 = (analogRead(SENSOR_1) >= threshold) ? 1 : 0;
  sensor_2 = (analogRead(SENSOR_2) >= threshold) ? 1 : 0;
  sensor_3 = (analogRead(SENSOR_3) >= threshold) ? 1 : 0;
  sensor_4 = (analogRead(SENSOR_4) >= threshold) ? 1 : 0;
  sensor_5 = (analogRead(SENSOR_5) >= threshold) ? 1 : 0;
}

void do_line() {
    readSensor();
    vitri = 10000 * sensor_1 + 1000 * sensor_2 + 100 * sensor_3 + 10 * sensor_4 + sensor_5;
    
    switch (vitri) {
        case 0:
            lui();
            Serial.println("LUI");
            break;
        case 1: case 11: case 111: case 1111:
            rephai_1();
            Serial.println("Re phải");
            break;
        case 10000: case 11000: case 11100: case 11110:
            retrai_1();
            Serial.println("Re trái");
            break;
        case 100: case 1110: case 11111:
            tien_1();
            Serial.println("Tiến");
            break;
        case 110: case 10:
            rephai1();
            Serial.println("Re phải 1");
            break;
        case 1000: case 1100:
            Serial.println("Re trái 1");
            retrai1();
            break;
    }
}

// === Servo điều khiển ===
void toggleServo1() { servo1.write(servo1State ? 0 : 180); servo1State = !servo1State; }
void toggleServo2() { servo2.write(servo2State ? 0 : 180); servo2State = !servo2State; }
void toggleServo3() { servo3.write(servo3State ? 0 : 90);  servo3State = !servo3State; }

// === Bluetooth command handler ===
void controlCar(char cmd) {
  if (lineFollowing && cmd != 'Y') return;  // Nếu đang dò line, chỉ cho phép tắt

  switch (cmd) {
    case 'A': tien(); break;
    case 'E': lui(); break;
    case 'C': retrai(); break;
    case 'U': rephai(); break;
    case 'V': retrai_1(); break;  // trái trước
    case 'B': rephai_1(); break; // phải trước
    case 'T': retrai_2(); break; // chéo trái
    case 'D': rephai_2(); break;  // chéo phải
    case 'S': dung(); break;

    case 'F': toggleServo1(); break;
    case 'L': toggleServo2(); break;
    case 'R': toggleServo3(); break;

    case 'M': lineFollowing = true; break;
    case 'N': lineFollowing = false; dung(); break;
  }
}

// === Setup ===
void setup() {
  Serial.begin(115200);
  SerialBT.begin("TN1_BANANANA");

  // Motor PWM setup
  pinMode(Drv_In1_pin, OUTPUT); ledcSetup(Drv_In2_chn, PWM_FREQ, PWM_RES); ledcAttachPin(Drv_In2_pin, Drv_In2_chn);
  pinMode(Drv_In3_pin, OUTPUT); ledcSetup(Drv_In4_chn, PWM_FREQ, PWM_RES); ledcAttachPin(Drv_In4_pin, Drv_In4_chn);

  // Line sensor
  pinMode(SENSOR_1, INPUT); pinMode(SENSOR_2, INPUT); pinMode(SENSOR_3, INPUT);
  pinMode(SENSOR_4, INPUT); pinMode(SENSOR_5, INPUT);

  // Servo
  servo1.attach(SERVO1_PIN); servo2.attach(SERVO2_PIN); servo3.attach(SERVO3_PIN);
  servo1.write(0); servo2.write(0); servo3.write(0);
}

// === Loop ===
void loop() {
  if (SerialBT.available()) {
    char command = SerialBT.read();
    Serial.println(command);
    controlCar(command);
  }

  if (lineFollowing) {
    do_line();
  }

  delay(100);
}
