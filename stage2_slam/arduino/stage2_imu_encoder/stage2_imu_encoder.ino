#include "tb6612_motor.h"
#include <Wire.h>
#include "MPU9250.h"  // MPU6500과 호환 가능

// 모터 정의: 왼쪽(M1), 오른쪽(M2)
SPDMotor* motorL = new SPDMotor(18, 31, true, 12, 35, 34);  // M1
SPDMotor* motorR = new SPDMotor(19, 38, false, 8, 37, 36);  // M2

// IMU 객체
MPU9250 mpu;

// 명령 정의
#define GO_FORWARD  1
#define GO_BACKWARD 2
#define TURN_LEFT   3
#define TURN_RIGHT  4
#define STOP        5

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(2000);

  if (!mpu.setup(0x68)) {
    while (1); // IMU 연결 실패 시 정지
  }
}

// 모터 제어 함수
void go_forward(int speed)  { motorL->speed(speed);  motorR->speed(speed); }
void go_backward(int speed) { motorL->speed(-speed); motorR->speed(-speed); }
void turn_left(int speed)   { motorL->speed(-speed); motorR->speed(speed); }
void turn_right(int speed)  { motorL->speed(speed);  motorR->speed(-speed); }
void stop_motors()          { motorL->hardStop();    motorR->hardStop(); }

unsigned long last_send = 0;

void loop() {
  // 모터 제어 패킷 수신
  if (Serial.available() >= 5) {
    byte header1 = Serial.read();
    byte header2 = Serial.read();
    byte mode = Serial.read();
    byte direction = Serial.read();
    byte speed = Serial.read();

    if (header1 == 0xF5 && header2 == 0xF5 && mode == 0x51) {
      int mapped_speed = map(speed, 0, 255, 0, 100);
      switch (direction) {
        case GO_FORWARD:  go_forward(mapped_speed); break;
        case GO_BACKWARD: go_backward(mapped_speed); break;
        case TURN_LEFT:   turn_left(mapped_speed); break;
        case TURN_RIGHT:  turn_right(mapped_speed); break;
        case STOP:        stop_motors(); break;
      }
    }
  }

  // 센서 데이터 전송 (50ms마다)
  if (millis() - last_send >= 50) {
    last_send = millis();
    if (mpu.update()) {
      float ax = mpu.getAccX();
      float ay = mpu.getAccY();
      float az = mpu.getAccZ();

      float gx = mpu.getGyroX();
      float gy = mpu.getGyroY();
      float gz = mpu.getGyroZ();

      // 예: F5 F5 S1 ax ay az gx gy gz (10바이트)
      Serial.write(0xF5); Serial.write(0xF5);
      Serial.write(0xA1);  // 센서 헤더 (가상 정의)

      // 가속도 값 전송 (int16으로 축소)
      Serial.write((int16_t)(ax * 1000) >> 8); Serial.write((int16_t)(ax * 1000) & 0xFF);
      Serial.write((int16_t)(ay * 1000) >> 8); Serial.write((int16_t)(ay * 1000) & 0xFF);
      Serial.write((int16_t)(az * 1000) >> 8); Serial.write((int16_t)(az * 1000) & 0xFF);

      Serial.write((int16_t)(gx * 1000) >> 8); Serial.write((int16_t)(gx * 1000) & 0xFF);
      Serial.write((int16_t)(gy * 1000) >> 8); Serial.write((int16_t)(gy * 1000) & 0xFF);
      Serial.write((int16_t)(gz * 1000) >> 8); Serial.write((int16_t)(gz * 1000) & 0xFF);
    }
  }
}
