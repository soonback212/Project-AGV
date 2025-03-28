#include "tb6612_motor.h"

// 왼쪽 모터 (M1), 오른쪽 모터 (M2)
SPDMotor* motorL = new SPDMotor(18, 31, true, 12, 35, 34);  // M1
SPDMotor* motorR = new SPDMotor(19, 38, false, 8, 37, 36);  // M2

// 명령 정의
#define GO_FORWARD  1
#define GO_BACKWARD 2
#define TURN_LEFT   3
#define TURN_RIGHT  4
#define STOP        5

void setup() {
  Serial.begin(115200);
}

// 직진/후진/회전 속도 명령
void go_forward(int speed) {
  motorL->speed(speed);
  motorR->speed(speed);
}

void go_backward(int speed) {
  motorL->speed(-speed);
  motorR->speed(-speed);
}

void turn_left(int speed) {
  motorL->speed(-speed);
  motorR->speed(speed);
}

void turn_right(int speed) {
  motorL->speed(speed);
  motorR->speed(-speed);
}

void stop_motors() {
  motorL->hardStop();
  motorR->hardStop();
}

void loop() {
  // 명령은 5바이트로 고정
  if (Serial.available() >= 5) {
    byte header1 = Serial.read();
    byte header2 = Serial.read();
    byte mode = Serial.read();      // 현재는 항상 0x51
    byte direction = Serial.read(); // 1~5
    byte speed = Serial.read();     // 0~255

    if (header1 == 0xF5 && header2 == 0xF5 && mode == 0x51) {
      switch (direction) {
        case GO_FORWARD:
          go_forward(map(speed, 0, 255, 0, 100));
          break;
        case GO_BACKWARD:
          go_backward(map(speed, 0, 255, 0, 100));
          break;
        case TURN_LEFT:
          turn_left(map(speed, 0, 255, 0, 100));
          break;
        case TURN_RIGHT:
          turn_right(map(speed, 0, 255, 0, 100));
          break;
        case STOP:
        default:
          stop_motors();
          break;
      }
    }
  }
}
