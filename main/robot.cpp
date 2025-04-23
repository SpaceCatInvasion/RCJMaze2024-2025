#include "robot.h"

int rampTilesForward = 0;
bool incline = false;

bool samePoint(Point p1, Point p2) {
  return (p1.x == p2.x && p1.y == p2.y);
}
Point nextPoint(Point p, Direction d, int mag) {
  Point temp = p;
  switch (d) {
    case NORTH:
      temp.y += mag;
      break;
    case SOUTH:
      temp.y -= mag;
      break;
    case EAST:
      temp.x += mag;
      break;
    case WEST:
      temp.x -= mag;
      break;
  }
  return temp;
}

int directionAngle(Direction d) {
  switch (d) {
    case NORTH:
      return 0;
    case SOUTH:
      return 180;
    case EAST:
      return 90;
    case WEST:
      return 270;
  }
  return -1;
}

// Don't pay attention to this function it doesn't work well
ReturnError Robot::wallTrace(int cm, int speed) {
  enc = 0;
  double prevErr = 0, dist = 0, err = 0;
  double leftDist, rightDist;
  while (encToCm(enc) < cm) {
    leftDist = readTOF(LEFT_TOF);
    rightDist = readTOF(RIGHT_TOF);
    Serial.print("Left: ");
    Serial.print(leftDist);
    Serial.print("; Right: ");
    Serial.print(rightDist);
    if (leftDist < 15)
      dist = leftDist;
    else if (rightDist < 15)
      dist = 30 - WIDTH - rightDist;
    else {
      forward(speed);
      continue;
    }

    err = (30 - WIDTH) / 2 - dist;
    lmotors(speed + err * kP + (err - prevErr) * kD);
    rmotors(speed - err * kP - (err - prevErr) * kD);
    prevErr = err;
  }
  return GOOD;
}

// Radians to Angle
double rToA(double r) {
  return r * 180 / PI;
}
// Angle to Radians
double aToR(double a) {
  return a * PI / 180;
}

/*
 * Find angle to turn to in order to be aligned with the next tile
 *
 * @param None
 * @return Acute angle from the y-axis
 */
double Robot::sideAlignment() {
  double dist = readTOF(LEFT_TOF) + 2;  //bias - positive = goes closer to wall
  //dist*=dist/3; //exagerates movement - (dist/b)^a decrease a to exagerate less, b is focus of exageration
  dist = dist < 0 ? 0.2 : dist;
  if (dist < 15 && dist > 0.1) {
    double theta = atan(60 / (2 * dist - 30 + TOF_WIDTH)) >= 0 ? rToA(atan(60 / (2 * dist - 30 + TOF_WIDTH))) : rToA(atan(60 / (2 * dist - 30 + TOF_WIDTH))) + 180;  // math
    turn_to(roundAngle(getBNO()) - 90 + theta);
    stop_motors();
    delay(10);
    return theta;
  }
  dist = readTOF(RIGHT_TOF) + 2;  //bias - positive = goes closer to wall
  //dist*=dist/3; //exagerates movement - (dist/b)^a decrease a to exagerate less, b is focus of exageration
  dist = dist < 0 ? 0.2 : dist;
  if (dist < 15 && dist > 0.1) {
    double theta = atan(60 / (2 * dist - 30 + TOF_WIDTH)) >= 0 ? rToA(atan(60 / (2 * dist - 30 + TOF_WIDTH))) : rToA(atan(60 / (2 * dist - 30 + TOF_WIDTH))) + 180;  //math
    turn_to(roundAngle(getBNO()) + 90 - theta);
    stop_motors();
    delay(10);
    return theta;
  }
  return 90;
}

/*
 * Move the robot forward while checking color sensor
 *
 * @param None
 * @return Whether or not the robot finds a special case (e.g. ramp, black tile)
 */
ReturnError Robot::robotForward(double cm) {
  Serial.print("CM:");
  Serial.println(cm);
  enc = 0;
  int colorIter = 0;
  bool obstacle = false;
  // Serial.print("enc: ");
  // Serial.println(enc);
  while (enc < cmToEnc(cm)) {
    Serial.print("goal: ");
    Serial.println(cmToEnc(cm));
    // Serial.print("enc: ");
    // Serial.println(enc);
    // if(digitalRead(LEFT_LIMIT_SWITCH_PIN) == LOW || digitalRead(RIGHT_LIMIT_SWITCH_PIN) == LOW) {
    //   obstacle = true;
    // }

#ifdef CAM_ON
    if (interrupted) interruptFunc();
#endif
#ifdef RAMP_ON
    // Serial.print("Ramp Tilt: "); Serial.println(abs(getTilt()));
    if (abs(getTilt()) > RAMP_TILT_THRESH) {

      if (getTilt() > 0) {
        Serial.println("Going up ramp...");
        incline = true;
      } else {
        Serial.println("Going down ramp...");
        incline = false;
      }
      // stop_motors(); delay(500);// while(digitalRead(20)==HIGH);
      enc = 0;
      double distForward = 0;  // x dist forward
      int prevEnc = 0;
      float angle = getTilt();
      int calcIter = 0;
      while ((abs(getTilt())) > RAMP_TILT_THRESH) {
#ifdef CAM_ON
        if (interrupted) interruptFunc();
#endif
        if (!(calcIter++ % 10)) {
          forward(RAMP_MOVE_SPEED * (1 + 0.01 * (angle - 20)));  // PID
          distForward += encToCm(enc - prevEnc) * cos((aToR(angle = getTilt())));
          Serial.print("Enc: ");
          Serial.print(enc);
          Serial.print(" Prev: ");
          Serial.print(prevEnc);
          Serial.print(" Angle: ");
          Serial.print(angle);
          Serial.print(" Cos: ");
          Serial.println(cos(aToR(angle)));
          prevEnc = enc;
        }
      }
      distForward *= 1.07;  // tested error
      if (!incline) distForward += 10;
      Serial.print("Dist forward: ");
      Serial.println(distForward);
      rampTilesForward = distForward / 30;
      if ((int)distForward % TILE_LENGTH > 15) rampTilesForward++;
      while (abs(getTilt()) > 5) {
        forward(RAMP_MOVE_SPEED * (.4 + 0.04 * (-angle)));
      }
      forwardCm(30, 5);
      stop_motors();
      delay(500);
      return RAMP;
    }
#endif
    if (!(colorIter++ % 25)) {
      switch (getColor()) {
        case BLACK:
          stop_motors();
          delay(200);
          if (getColor() == BLACK) {
            backwardCm(FORWARD_MOVE_SPEED, encToCm(enc) + 1);
            return BLACKTILE;
          }
          break;
        case RED:
          if (status == TRAVERSING) {
            stop_motors();
            delay(200);
            if (getColor() == RED) {
              backwardCm(FORWARD_MOVE_SPEED, encToCm(enc) + 1);
              return REDTILE;
            }
            break;
          }
        case WHITE:
        default:
          break;
      }
    }
    forward(FORWARD_MOVE_SPEED);
  }
  // if(obstacle) {
  //   stop_motors();
  //   delay(500);
  //   backtrack();
  //   stop_motors();
  //   delay(100);
  // }
  enc = 0;
  stop_motors();
  delay(10);
  return GOOD;
}

/*
 * Align the robot with the front TOF
 *
 * @param None
 * @return None
 */
void Robot::frontAlign() {
  Serial.println("Front Aligning");
  int dist = readTOF(FRONT_TOF);
  while (dist < 15 && dist > (30 - FRONTBACKTOF) / 2) {
    forward(30);
    dist = readTOF(FRONT_TOF);
  }
  while (dist < 15 && dist < (30 - FRONTBACKTOF) / 2) {
    backward(30);
    dist = readTOF(FRONT_TOF);
  }
  Serial.println("Front Aligning Done");
}
/*
 * Align the robot with the back TOF
 *
 * @param None
 * @return None
 */
void Robot::backAlign() {
  Serial.println("Back Aligning");
  int dist = readTOF(BACK_TOF);
  while (dist < 15 && dist > (30 - FRONTBACKTOF) / 2) {
    backward(30);
    dist = readTOF(BACK_TOF);
  }
  while (dist < 15 && dist < (30 - FRONTBACKTOF) / 2) {
    forward(30);
    dist = readTOF(BACK_TOF);
  }
  Serial.println("Back Aligning Done");
}


Robot::Robot() {
  pos.x = 0;
  pos.y = 0;
  pos.z = 0;
  facing = NORTH;
  status = TRAVERSING;
}

/*
 * Move the robot in the direction dir
 *
 * @param Direction dir to move towards
 * @return Error status of movement (e.g. black tile, ramp)
 */
ReturnError Robot::moveRobot(Direction dir) {
  facing = dir;
  stop_motors();
  delay(200);

  turn_to(directionAngle(dir));
  stop_motors();
  delay(200);
  frontAlign();
  backAlign();
  turn_to(directionAngle(dir));
  stop_motors();
  delay(200);
  // Serial.println("rcj done");
  // Serial.println(abs(enc));
  // delay(10);
  switch (robotForward(TILE_MOVE_DIST / sin(aToR(sideAlignment())))) {
      // Serial.println(abs(enc));
      // Serial.println(abs(enc));
    case RAMP:
      stop_motors();
      delay(500);
      return RAMP;
    case BLACKTILE:
      stop_motors();
      delay(500);
      return BLACKTILE;
    case REDTILE:
      stop_motors();
      delay(500);
      return REDTILE;
    case GOOD:
    default:
      stop_motors();
      delay(200);
      turn_to(directionAngle(dir));
      stop_motors();
      delay(200);
      frontAlign();
      backAlign();
      return GOOD;
  }
}
/*
 * Move robot on the path denoted by the vector directions
 *
 * @param Vector of the directions to move in
 * @return Error status of movement (e.g. black tile, no moves left)
 */
ReturnError Robot::moveDirections(std::vector<Direction> directions) {
  if (directions.empty()) return NOMOVES;
  for (Direction d : directions) {
    printDir(d);
    pos = nextPoint(pos, d);
    ReturnError moveStatus = moveRobot(d);
    stop_motors();
    delay(200);
    switch (moveStatus) {
      case RAMP:
      case REDTILE:
      case BLACKTILE:
        return moveStatus;
    }
  }
  return GOOD;
}

/*
 * PID turn the robot to specified degrees clockwise from North
 *
 * @param Degrees clockwise from North to turn to
 * @return None
 */

 /*
void Robot::turn_to(int deg) {

  /*
lmotors(-80);
delay(175);
stop_motors();
delay(100);
rmotors(75);
delay(175);
stop_motors();
forward(60);
delay(20);
stop_motors();
delay(300);
  lmotors(-60);
  rmotors(-60);
  delay(50);
  stop_motors();
  int i = 0;
  deg %= 360;
  if (deg < 0) deg += 360;
  double err = deg - getBNO();
  if (err > 180) err -= 360;
  if (err < -180) err += 360;
  while (err > 2 || err < -2) {
#ifdef CAM_ON
    if (interrupted) interruptFunc();
#endif
    if (err > 0) {

      if (i == 0) {
        lmotors(-80);
        rmotors(0);
        delay(175);
      }
      if (i == 1) {
        stop_motors();
        delay(100);
      }

      if (i == 2) {
        rmotors(75);
        lmotors(0);
        delay(175);
      }

      if (i == 3) {
        stop_motors();
        forward(60);
        delay(35);
      }

      if (i == 4) {
        stop_motors();
        delay(300);
      }
    } else if (err <= 0) {
      if (i == 0) {
        lmotors(0);
        rmotors(-75);
        delay(175);
      }
      if (i == 1) {
        stop_motors();
        delay(100);
      }

      if (i == 2) {
        rmotors(0);
        lmotors(80);
        delay(175);
      }

      if (i == 3) {
        stop_motors();
        forward(60);
        delay(35);
      }

      if (i == 4) {
        stop_motors();
        delay(300);
      }
    }
    err = deg - getBNO();
    i++;
    if (i == 5) {
      i = 0;
    }
    if (err > 180) err -= 360;
    if (err < -180) err += 360;
    //Serial.print("Error: "); Serial.println(err);
  }
  //Serial.println("DONE");
}

*/

void Robot::turn_to(int deg) {
  deg %= 360;
  if (deg < 0) deg += 360;
  double err = deg - getBNO();
  if (err > 180) err -= 360;
  if (err < -180) err += 360;
  while (err > 2 || err < -2) {
#ifdef CAM_ON
    if (interrupted) interruptFunc();
#endif
    lmotors((err > 0 ? BASE_TURN_SPEED : -BASE_TURN_SPEED));
    rmotors((err > 0 ? -BASE_TURN_SPEED : BASE_TURN_SPEED));
    err = deg - getBNO();
    if (err > 180) err -= 360;
    if (err < -180) err += 360;
    //Serial.print("Error: "); Serial.println(err);
  }
  //Serial.println("DONE");
}




/*
 * Turn specified degrees clockwise from current angle
 *
 * @param Degrees to turn clockwise
 * @return None
 */
void Robot::turn(int deg) {

  turn_to(deg + getBNO());
}

void Robot::print() {
  switch (status) {
    case TRAVERSING: Serial.print("Traversing... "); break;
    case DANGERZONE: Serial.print("In Danger Zone... "); break;
    case BACKTRACKING: Serial.print("Backtracking... "); break;
    case FINISH: Serial.print("Finished! "); break;
    case END: Serial.print("Run over... "); break;
  }
  Serial.print("Robot at (");
  Serial.print(pos.x);
  Serial.print(",");
  Serial.print(pos.y);
  Serial.print(",");
  Serial.print(pos.z);
  Serial.print(") facing ");
  switch (facing) {
    case NORTH: Serial.print("North"); break;
    case SOUTH: Serial.print("South"); break;
    case EAST: Serial.print("East"); break;
    case WEST: Serial.print("West"); break;
  }
  Serial.println();
}

void printPoint(Point p) {
  Serial.print("(");
  Serial.print((int)p.x);
  Serial.print(",");
  Serial.print((int)p.y);
  Serial.print(",");
  Serial.print((int)p.z);
  Serial.println(")");
}

void printDir(Direction d) {
  switch (d) {
    case NORTH: Serial.println("North"); break;
    case SOUTH: Serial.println("South"); break;
    case EAST: Serial.println("East"); break;
    case WEST: Serial.println("West"); break;
  }
}