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

// Endian 변환용 함수 (Big Endian 전송)
void sendInt16BE(int16_t value) {
  Serial.write((uint8_t)((value >> 8) & 0xFF));  // MSB
  Serial.write((uint8_t)(value & 0xFF));         // LSB
}

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

  // IMU 센서 패킷 전송 (50ms마다)
  if (millis() - last_send >= 50) {
    last_send = millis();

    if (mpu.update()) {
      // 센서값 읽기 및 단위 변환 (m/s^2 → mm/s^2, rad/s → mrad/s)
      int16_t ax = mpu.getAccX() * 1000;
      int16_t ay = mpu.getAccY() * 1000;
      int16_t az = mpu.getAccZ() * 1000;
      int16_t gx = mpu.getGyroX() * 1000;
      int16_t gy = mpu.getGyroY() * 1000;
      int16_t gz = mpu.getGyroZ() * 1000;

      // 아직 안 쓰는 값은 0으로 채움
      int16_t mx = 0;
      int16_t my = 0;
      int16_t mz = 0;
      int16_t enc_l = 0;
      int16_t enc_r = 0;

      // 패킷 전송
      Serial.write(0xF5);  // 헤더1
      Serial.write(0xF5);  // 헤더2

      sendInt16BE(ax); sendInt16BE(ay); sendInt16BE(az);
      sendInt16BE(gx); sendInt16BE(gy); sendInt16BE(gz);
      sendInt16BE(mx); sendInt16BE(my); sendInt16BE(mz);
      sendInt16BE(enc_l); sendInt16BE(enc_r);
    }
  }
}

