#include <Stepper.h>
#include <string.h>

enum MODE {CONTINUIOUS, BOUNDED};

const int stepsPerRevolution = 2038;
int curPos = 0;
int dir = 1;
int speed = 5;
MODE mode = CONTINUIOUS;

Stepper stepper(stepsPerRevolution, 10,8,11,9);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  stepper.setSpeed(5);
}

void continuiousRot(int nextPos) {
  int steps = dir * (nextPos - curPos);
  if (steps < 0) {
    steps += stepsPerRevolution;
  }
  curPos = nextPos;
  stepper.step(dir*steps);
}

void boundedRot(int nextPos) {
  int steps = (nextPos - curPos);
  if (steps > stepsPerRevolution/2-1 || steps < -stepsPerRevolution/2)
    return;
  stepper.step(steps);
}

void loop() {
  // put your main code here, to run repeatedly:
  while (Serial.available() == 0) {}
  String serialStr = Serial.readString();
  serialStr.trim();
  serialStr.toLowerCase();
  switch(serialStr[0]) {
    case 's': // Set the speed
      speed = serialStr.substring(1).toInt();
      stepper.setSpeed(speed);
      Serial.println(speed);
      break;
    
    case 'r': // reverse rotation
      dir = -1 * dir;
      Serial.println(dir);
      break;
    
    case 'd': // set direction
      speed = serialStr.substring(1).toInt();
      Serial.println(dir);
      break;
    
    case 'm': // swap mode
      if (serialStr.substring(1)[0] == 'c') {
        mode = CONTINUIOUS;
      } else if (serialStr.substring(1)[0] == 'b') {
        mode = BOUNDED;
      }
      if (mode == CONTINUIOUS) {
        Serial.println("continuious");
      } else {
        Serial.println("Bounded");
      }
      break;

    case 'z': // zero out rotation
      Serial.println(curPos);
      curPos = 0;
      break;
    
    default:
      int nextPos = serialStr.toInt();
      switch(mode) {
        case CONTINUIOUS:
          continuiousRot(nextPos);
          break;
        case BOUNDED:
          boundedRot(nextPos);
          break;
        default:;
      }
      Serial.println(curPos);
  }

}
