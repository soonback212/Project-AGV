#include "tb6612_motor.h"

// 왼쪽 모터: M1, 오른쪽 모터: M2
SPDMotor* motorL = new SPDMotor(18, 31, true, 12, 35, 34);  // M1
SPDMotor* motorR = new SPDMotor(19, 38, false, 8, 37, 36);  // M2

String inputString = "";
bool stringComplete = false;

void setup() {
  Serial.begin(115200);
}

void loop() {
  if (stringComplete) {
    int l_val = 0, r_val = 0;
    if (parseCommand(inputString, l_val, r_val)) {
      motorL->speed(l_val);
      motorR->speed(r_val);
    }
    inputString = "";
    stringComplete = false;
  }
}

bool parseCommand(String input, int &left, int &right) {
  int l_index = input.indexOf('L');
  int r_index = input.indexOf('R');

  if (l_index != -1 && r_index != -1) {
    left = input.substring(l_index + 1, r_index).toInt();
    right = input.substring(r_index + 1).toInt();
    return true;
  }
  return false;
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') stringComplete = true;
  }
}
