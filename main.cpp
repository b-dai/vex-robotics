/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       1274B                                                     */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  2021-22 Season 1274B Robot                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// fLDrive_mtr          motor         17              
// fRDrive_mtr          motor         18              
// bRDrive_mtr          motor         7               
// lift_mtr             motor         11              
// intake_mtr           motor         13              
// imu_sensor           inertial      20              
// bLDrive_mtr          motor         15              
// claw_pneu            digital_out   D               
// mLDrive_mtr          motor         16              
// mRDrive_mtr          motor         19              
// gps_sensor           gps           8               
// tilt_pneu            digital_out   H               
// tran_pneu            digital_out   C               
// cover_pneu           digital_out   G               
// claw_sensor          optical       5               
// back_sensor          vision        21              
// backRange_sensor     sonar         E, F            
// lift_sensor          rotation      2               
// lift_pneu            digital_out   B               
// ---- END VEXCODE CONFIGURED DEVICES ----

using namespace vex;

// A global instance of competition
competition Competition;

// variables
bool settingStartingRobot = true;

// double xRobotPos = 0;
// double yRobotPos = 0;
// double prevHorizontalTrackDeg = 0;
// double prevVerticalTrackDeg = 0;
// double prevHeadingDeg = 0;

// double trackWheelCircumIn = 8.6394;
/*
* Diagram for vertical measurements below:
*   ___________
*  |           |
* -[]\         |
*  |  \ z      |
* y|   \       |
* -|-----C     |
*  |  x  |     |
*  |           |
*  |___________|
*
* (C = center point of drivetrain)
* ([] = vertical tracking wheel)
* (All variables are absolute values)
* x = verticalTrackAxisFromCenterIn
* y = verticalTrackWheelFromCenterAxisIn
* z = verticalTrackWheelFromCenterIn
*/
// double horizontalTrackAxisFromCenterIn = 2.50;
// double horizontalTrackWheelFromCenterAxisIn = 1.50;
// double horizontalTrackWheelFromCenterIn = sqrt(horizontalTrackAxisFromCenterIn * horizontalTrackAxisFromCenterIn + horizontalTrackWheelFromCenterAxisIn * horizontalTrackWheelFromCenterAxisIn);
// double horizontalTrackWheelAngleOffsetRad = atan(horizontalTrackWheelFromCenterAxisIn / horizontalTrackAxisFromCenterIn);
// double verticalTrackAxisFromCenterIn = 3.47;
// double verticalTrackWheelFromCenterAxisIn = 1.87;
// double verticalTrackWheelFromCenterIn = sqrt(verticalTrackAxisFromCenterIn * verticalTrackAxisFromCenterIn + verticalTrackWheelFromCenterAxisIn * verticalTrackWheelFromCenterAxisIn);
// double verticalTrackWheelAngleOffsetRad = atan(verticalTrackWheelFromCenterAxisIn / verticalTrackAxisFromCenterIn);
double wheelDiameterIn = 3.25;
double driveGearRatio = 5.0 / 3.0; // multiply by this whenever moving distance for drive

bool isInAuto = false;
bool isSkillsAuto = false;

bool calibrated = false;
bool isBotInit = false;
bool intakingIn = false;
bool intakingOut = false;
bool isClawLocked = true;
bool isCoverDeployed = false;
bool isBackUntilted = false;
bool isLiftLocked = true;
bool isOnRampMode = false;

double liftSpeedVolt = 12;
double intakeSpeedPCT = 100;
double maxDriveSpeedVolt = 12;

double liftDeadzoneDeg = 20;
double liftAutoWaitDeg = 3;
double liftCanIntakeDeg = 5;
bool usingAxis2 = false;

bool is6MtrDrive = false;
brakeType startAutonLiftIntakeType = brake;

bool isDetectingFrontMogo = false;
bool isLiftingFrontMogo = false;

double maxIntakeThresholdDeg = 30;
double maxIntakeAutoWaitDeg = 3;
bool isUnjamming = false;

double currAxis3;
double currAxis4;
double currAxis2;
double prevAxis3 = 0;
double prevAxis4 = 0;
double prevAxis2 = 0;

// functions

void calibrateSensors() {
	imu_sensor.startCalibration();
	wait(2000, msec);
	while (imu_sensor.isCalibrating()) {
		wait(15, msec);
	}
  wait(250, msec);
  gps_sensor.startCalibration();
  wait(2000, msec);
  while (gps_sensor.isCalibrating()) {
    wait(15, msec);
  }
	wait(250, msec);
	Controller1.rumble("-");
	Brain.Screen.printAt(0, 15, "Pre-calibration complete!");
  calibrated = true;
}

double getX()
{
  return (gps_sensor.xPosition(inches) + 72.0);
}

double getY()
{
  return (gps_sensor.yPosition(inches) + 72.0);
}

double doubleMathAbs(double num)
{
  if (num < 0.0)
  {
    return num * -1.0;
  }
  else
  {
    return num;
  }
}

/*
* Stops all wheels; non-blocking.
*/
void stopDrive()
{
  fLDrive_mtr.stop();
  fRDrive_mtr.stop();
  mLDrive_mtr.stop();
  mRDrive_mtr.stop();
  bLDrive_mtr.stop();
  bRDrive_mtr.stop();
}

void toggleLiftLock()
{
  if (!isLiftLocked)
  {
    isLiftLocked = true;
    lift_pneu.set(true);
  }
  else
  {
    isLiftLocked = false;
    lift_pneu.set(false);
  }
}

void setLiftLock(bool locked)
{
  isLiftLocked = locked;
  lift_pneu.set(locked);
}

void turnIntakeOff()
{
  intakingIn = false;
  intakingOut = false;
  intake_mtr.stop();
}

void autonToggleIntakeIn()
{
  if (!intakingIn)
  {
    intakingIn = true;
    intakingOut = false;
    intake_mtr.spin(fwd, intakeSpeedPCT, pct);
  }
  else
  {
    turnIntakeOff();
  }
}

void autonToggleIntakeOut()
{
  if (!intakingOut)
  {
    intakingOut = true;
    intakingIn = false;
    intake_mtr.spin(fwd, -intakeSpeedPCT, pct);
  }
  else
  {
    turnIntakeOff();
  }
}

void toggleIntakeIn()
{
  if (is6MtrDrive && !intakingIn)
  {
    intakingIn = true;
    intakingOut = false;
    if (!usingAxis2 && lift_sensor.position(deg) < liftDeadzoneDeg)
    {
      lift_mtr.spin(fwd, -liftSpeedVolt, volt);
      intake_mtr.spin(fwd, -liftSpeedVolt, volt);
      while (is6MtrDrive && !usingAxis2 && lift_sensor.position(deg) < (liftDeadzoneDeg - liftAutoWaitDeg))
      {
        wait(5, msec);
      }
    }
    else if (!usingAxis2 && lift_sensor.position(deg) > (110.0 - maxIntakeThresholdDeg))
    {
      lift_mtr.spin(fwd, liftSpeedVolt, volt);
      intake_mtr.spin(fwd, liftSpeedVolt, volt);
      while (is6MtrDrive && !usingAxis2 && lift_sensor.position(deg) > (110.0 - maxIntakeThresholdDeg + maxIntakeAutoWaitDeg))
      {
        wait(5, msec);
      }
    }
    if (is6MtrDrive && intakingIn && lift_sensor.position(deg) > (liftDeadzoneDeg - liftCanIntakeDeg))
    {
      intake_mtr.spin(fwd, intakeSpeedPCT, pct);
    }
    else if (is6MtrDrive && intakingOut && lift_sensor.position(deg) > (liftDeadzoneDeg - liftCanIntakeDeg))
    {
      intake_mtr.spin(fwd, -intakeSpeedPCT, pct);
    }
    else if (is6MtrDrive)
    {
      turnIntakeOff();
    }
  }
  else if (is6MtrDrive)
  {
    turnIntakeOff();
  }
}

void toggleIntakeOut()
{
  if (is6MtrDrive && !intakingOut)
  {
    intakingOut = true;
    intakingIn = false;
    if (!usingAxis2 && lift_sensor.position(deg) < liftDeadzoneDeg)
    {
      lift_mtr.spin(fwd, -liftSpeedVolt, volt);
      intake_mtr.spin(fwd, -liftSpeedVolt, volt);
      while (is6MtrDrive && !usingAxis2 && lift_sensor.position(deg) < (liftDeadzoneDeg - liftAutoWaitDeg))
      {
        wait(5, msec);
      }
    }
    if (is6MtrDrive && intakingIn && lift_sensor.position(deg) > (liftDeadzoneDeg - liftCanIntakeDeg))
    {
      intake_mtr.spin(fwd, intakeSpeedPCT, pct);
    }
    else if (is6MtrDrive && intakingOut && lift_sensor.position(deg) > (liftDeadzoneDeg - liftCanIntakeDeg))
    {
      intake_mtr.spin(fwd, -intakeSpeedPCT, pct);
    }
    else if (is6MtrDrive)
    {
      turnIntakeOff();
    }
  }
  else if (is6MtrDrive)
  {
    turnIntakeOff();
  }
}

void toggleClawLock()
{
  if (!isClawLocked)
  {
    isClawLocked = true;
    claw_pneu.set(true);
  }
  else
  {
    isClawLocked = false;
    claw_pneu.set(false);
  }
}

void setClawLock(bool locked)
{
  isClawLocked = locked;
  claw_pneu.set(locked);
}

void toggleBackTilt()
{
  if (!isBackUntilted)
  {
    isBackUntilted = true;
    tilt_pneu.set(true);
  }
  else
  {
    isBackUntilted = false;
    tilt_pneu.set(false);
  }
}

void setBackTilt(bool untilted)
{
  isBackUntilted = untilted;
  tilt_pneu.set(untilted);
}

void toggleGoalCover()
{
  if (!isCoverDeployed)
  {
    isCoverDeployed = true;
    cover_pneu.set(true);
  }
  else
  {
    isCoverDeployed = false;
    cover_pneu.set(false);
  }
}

void setGoalCover(bool deployed)
{
  isCoverDeployed = deployed;
  cover_pneu.set(deployed);
}

void toggleTrans()
{
  turnIntakeOff();
  lift_mtr.setStopping(coast);
  intake_mtr.setStopping(coast);
  lift_mtr.stop();
  intake_mtr.stop();
  if (!is6MtrDrive)
  {
    is6MtrDrive = true;
    tran_pneu.set(true);
  }
  else
  {
    is6MtrDrive = false;
    tran_pneu.set(false);
    Controller1.rumble("-");
  }
}

void setTrans(bool tranState)
{
  turnIntakeOff();
  lift_mtr.setStopping(coast);
  intake_mtr.setStopping(coast);
  lift_mtr.stop();
  intake_mtr.stop();
  is6MtrDrive = tranState;
  tran_pneu.set(tranState);
}

void initiateBot()
{
  if (!isBotInit)
  {
    isBotInit = true;
    setClawLock(isClawLocked);
    setBackTilt(isBackUntilted);
    setGoalCover(isCoverDeployed);
    setTrans(is6MtrDrive);
    setLiftLock(isLiftLocked);
    isBotInit = false;
  }
}

void intakeJamThread()
{
  while (true)
  {
    if (!isInAuto && intakingIn && !usingAxis2 && lift_sensor.position(deg) > (liftDeadzoneDeg - liftCanIntakeDeg) && lift_sensor.position(deg) < (110.0 - maxIntakeThresholdDeg + maxIntakeAutoWaitDeg))
    {
      wait(500, msec);
      while (intakingIn && !usingAxis2 && lift_sensor.position(deg) > (liftDeadzoneDeg - liftCanIntakeDeg) && lift_sensor.position(deg) < (110.0 - maxIntakeThresholdDeg + maxIntakeAutoWaitDeg))
      {
        if (doubleMathAbs(intake_mtr.velocity(pct)) < 5.0)
        {
          isUnjamming = true;
          setLiftLock(false);
          lift_mtr.spin(fwd, -liftSpeedVolt, volt);
          intake_mtr.spin(fwd, -liftSpeedVolt, volt);
          int timeCount = 0;
          while (timeCount < 250)
          {
            if (!intakingIn || usingAxis2)
              break;
            wait(5, msec);
            timeCount += 5;
          }
          if (intakingIn && !usingAxis2)
          {
            lift_mtr.spin(fwd, liftSpeedVolt, volt);
            intake_mtr.spin(fwd, liftSpeedVolt, volt);
          }
          timeCount = 0;
          while (timeCount < 250)
          {
            if (!intakingIn || usingAxis2)
              break;
            wait(5, msec);
            timeCount += 5;
          }
          if (intakingIn && !usingAxis2)
          {
            if (lift_sensor.position(deg) > (liftDeadzoneDeg - liftCanIntakeDeg) && lift_sensor.position(deg) < (110.0 - maxIntakeThresholdDeg + maxIntakeAutoWaitDeg))
            {
              intake_mtr.spin(fwd, intakeSpeedPCT, pct);
            }
            else
            {
              intake_mtr.stop();
            }
          }
          isUnjamming = false;
        }
        wait(20, msec);
      }
    }
    wait(20, msec);
  }
}

void liftSpinToPosition(double targetDeg)
{
  lift_mtr.setStopping(hold);
  intake_mtr.setStopping(hold);
  double amt = doubleMathAbs(targetDeg - lift_sensor.position(deg));
  if (lift_sensor.position(deg) > targetDeg)
  {
    lift_mtr.spinFor(fwd, amt * 25.0, deg, false);
    intake_mtr.spinFor(fwd, amt * 25.0, deg, false);
  }
  else
  {
    lift_mtr.spinFor(fwd, -amt * 25.0, deg, false);
    intake_mtr.spinFor(fwd, -amt * 25.0, deg, false);
  }
}

void onClawSensorDetected()
{
  if (isDetectingFrontMogo)
  {
    setClawLock(true);
    if (isLiftingFrontMogo)
    {
      liftSpinToPosition(70);
    }
  }
}

/*
* Calculate the degree difference between two angles.
* It takes care of crossing from 359.99 degrees back to 0.00.
* Always returns the absolute difference in degrees.
* Note: The IMU sensor goes from 0 to 360 in clockwise direction.
*/
double getIMUDegDiff(double deg1, double deg2)
{
  if (deg1 <= deg2) // if first is smaller than second
  {
    // returns smaller of two possible differences in angles (there are always two)
    if (deg2 - deg1 <= deg1 + (360 - deg2)) // if the difference between first and second is smaller when not crossing the 0 degree line
    {
      return deg2 - deg1;
    }
    else // if the difference between first and second is smaller when crossing the 0 degree line
    {
      return deg1 + (360 - deg2);
    }
  }
  else
  {
    if (deg1 - deg2 <= deg2 + (360 - deg1)) // if the difference between second and first is smaller when not crossing the 0 degree line
    {
      return deg1 - deg2;
    }
    else // if the difference between second and first is smaller when crossing the 0 degree line
    {
      return deg2 + (360 - deg1);
    }
  }
}

/*
* Returns the absolute degree difference between newDeg and oldDeg, depending on which turning
* direction is to be taken.
* If isDirRight is true, the direction is turning right.
* The newDeg is the "target," and the oldDeg is the "start."
* Note: The IMU sensor goes from 0 to 360 in clockwise direction.
*/
double getIMUDegDiffFromDir(double newDeg, double oldDeg, bool isDirRight) {
	double returnVal = 0;
	if (isDirRight && newDeg >= oldDeg) {
		returnVal = newDeg - oldDeg;
	} else if (isDirRight && newDeg <= oldDeg) {
		returnVal = (360 - oldDeg) + newDeg;
	} else if (!isDirRight && newDeg <= oldDeg) {
		returnVal = oldDeg - newDeg;
	} else if (!isDirRight && newDeg >= oldDeg) {
		returnVal = oldDeg + (360 - newDeg);
	}
	return returnVal;
}

/*
* Gets any degree value from negative infinity to positive infinity.
* Returns degree value that corresponds to deg that is within 0 to 360 degrees.
*/
double getDegWithin360(double deg)
{
  double returnDeg = deg - ((int)(deg / 360.0) * 360.0);
  if (returnDeg < 0.0)
    returnDeg += 360.0;
  return returnDeg;
}

/*
* Gets a degree value in terms of either a default math coordinate grid or
* an IMU sensor direction.
* Translates the given degree value into a corresponding value on an IMU sensor
* degree grid if given a coordinate degree, or into a coordinate degree if given
* an IMU sensor degree (all after ensuring given degree value is within 360).
* Returns translated degree value.
* Works for both ways:
*    coord -> IMU
*    IMU -> coord
*/
double convertIMUAndCoordDeg(double deg)
{
  double translatedDeg = getDegWithin360(deg);
  if (translatedDeg <= 90.0)
    translatedDeg = translatedDeg * -1.0 + 90.0;
  else
    translatedDeg = translatedDeg * -1.0 + 450.0;
  return translatedDeg;
}

/*
* Convert between wheel distance and wheel degrees.
*/
double convertWheelInAndDeg(bool isGivenDeg, double value, double diameter)
{
  if (isGivenDeg)
  {
    return ((value / 360.0) * (diameter * M_PI));
  }
  else
  {
    return (value / (M_PI * diameter)) * 360.0;
  }
}

/*
* Return constant multiplier for speed curves using PD.
* MUST TAKE INCHES LENGTH, RETURNS FOR INCHES
*/
double getCurveKSM(double PDDistIn, double kP)
{
  return (PDDistIn - kP) / (100.0 * pow((PDDistIn - kP) / 100.0, 0.52));
}

/*
* Odometry Tracking (Horizontal/Vertical Tracking Wheels)
*/
// void updateXYRobotPos()
// {

//   double newHorizontalDeg = horizontalTrackWheel.position(deg); // read new degree pos in tracking wheels
//   double newVerticalDeg = -1.0 * verticalTrackWheel.position(deg);

//   double currHeadingDeg = imu_sensor.heading(deg); // get current robot heading
//   double dHeading = getIMUDegDiff(currHeadingDeg, prevHeadingDeg); // get change in robot heading since last iteration
//   double horizontalTurnError = 2.0 * M_PI * horizontalTrackWheelFromCenterIn * dHeading / 360.0; // calculate horizontal and vertical wheel dist along dHeading arc
//   double verticalTurnError = 2.0 * M_PI * verticalTrackWheelFromCenterIn * dHeading / 360.0;
//   horizontalTurnError *= cos(horizontalTrackWheelAngleOffsetRad); // project turning arcs onto raw orientation of tracking wheels based on wheel angle offsets
//   verticalTurnError *= cos(verticalTrackWheelAngleOffsetRad);

//   double horizontalDist = trackWheelCircumIn * (newHorizontalDeg - prevHorizontalTrackDeg) / 360.0; // calculate horizontal and vertical wheel raw dist from change in degrees of wheels
//   double verticalDist = trackWheelCircumIn * (newVerticalDeg - prevVerticalTrackDeg) / 360.0;

//   /*if (xDist <= 0 && xDist + xTurnError <= 0)
//     xDist += xTurnError;
//   else if (xDist > 0 && xDist - xTurnError > 0)
//     xDist -= xTurnError;
//   else
//     xDist = 0;
//   if (yDist <= 0 && yDist + yTurnError <= 0)
//     yDist += yTurnError;
//   else if (yDist > 0 && yDist - yTurnError > 0)
//     yDist -= yTurnError;
//   else
//     yDist = 0;*/
//   if (getIMUDegDiffFromDir(currHeadingDeg, prevHeadingDeg, true) < getIMUDegDiffFromDir(currHeadingDeg, prevHeadingDeg, false)) // if robot turned right (clockwise) from prevHeadingDeg
//   {
//     horizontalDist += horizontalTurnError;
//     verticalDist -= verticalTurnError;
//   }
//   else // if robot turned left (counterclockwise) from prevHeadingDeg
//   {
//     horizontalDist -= horizontalTurnError;
//     verticalDist += verticalTurnError;
//   }

//   xRobotPos += horizontalDist * sin((prevHeadingDeg + 90.0) * (M_PI / 180.0)) + verticalDist * sin(prevHeadingDeg * (M_PI / 180.0)); // update robotPos using adjusted horizontalDist and verticalDist
//   yRobotPos += verticalDist * cos(prevHeadingDeg * (M_PI / 180.0)) + horizontalDist * cos((prevHeadingDeg + 90.0) * (M_PI / 180.0));

//   prevHorizontalTrackDeg = newHorizontalDeg; // update values for next iteration use
//   prevVerticalTrackDeg = newVerticalDeg;
//   prevHeadingDeg = currHeadingDeg;

// }

/*
* Drive to a specific point on the field (from any location)
* Can choose to drive forward/backward
* Includes motion profiling (see driveForDistance below)
* Note: Motion profiling does not include an acceleration phase.
* Also: Has a simpler signature below without the last few arguments (refer below for uses).
*/
void driveToPoint(double targetX, double targetY, double maxSpeedPCT, bool goingForward, double absErrorThreshold, double maxAddTurnAdjustPCT, double decelRangeIn, double requireStopRangeIn)
{

  // int count = 0;
  // double xTotal = 0.0;
  // double yTotal = 0.0;

  double xError = targetX - getX(); // how much x is off
  double yError = targetY - getY(); // how much y is off

  double absAngleThreshold = 0.5; // degrees; max angle difference before needing turn adjust
  double absTurnAdjustMaxAngleThreshold = 180.0; // degrees; scaling of adjusting drive based on degree off
  double baseTurnAdjustPCT = 0.0; // percent mtr power; baseline adjust required
  double baseDrivePCT = 0.0; // percent mtr power; baseline straight drive required

  double currDistToPoint = sqrt(xError * xError + yError * yError); // distance left to point (positive only)
  double adjustedSpeedPCT; // adjusted straight drive pct
  double driveAdjustPCT; // adjusted turning drive pct
  double currAngle; // current angle heading of robot on field
  double targetAngle; // current angle from robot to point
  double rearAngle; // current angle heading of robot rear on field (180 + currAngle)

  while (currDistToPoint > absErrorThreshold)
  {

    targetAngle = atan(yError / xError) * (180.0 / M_PI); // angle from robot to point in degrees (from this formula, can only be -90 to 90 deg)
    if (xError < 0.0) // if angle is on left quadrants of math graph, project angle onto other side of math graph with equivalent tangent
      targetAngle += 180.0;
    targetAngle = convertIMUAndCoordDeg(targetAngle); // project angle from robot to point from math graph to inertial sensor angle system
    currAngle = gps_sensor.heading(deg); // actual heading of robot
    driveAdjustPCT = 0.0; // amount (in percent mtr power) to adjust drivetrain
    adjustedSpeedPCT = (currDistToPoint / decelRangeIn) * maxSpeedPCT + baseDrivePCT; // adjust speed for wheels based on dist (motion profiling)
    if (adjustedSpeedPCT > maxSpeedPCT) // limit the adjustment to no more than maxSpeedPCT given
      adjustedSpeedPCT = maxSpeedPCT;

    if (goingForward) // if driving forward
    {
      if (getIMUDegDiff(targetAngle, currAngle) > absAngleThreshold) // if smallest angle from target to robot is greater than acceptable threshold
      {
        if (currDistToPoint <= requireStopRangeIn) // remove any speed percent that is not for turn adjusting if within specified range
          adjustedSpeedPCT = 0.0;
        if (getIMUDegDiffFromDir(targetAngle, currAngle, true) <= getIMUDegDiffFromDir(targetAngle, currAngle, false)) // if turning robot right is closer than turning left to face the point
        {
          driveAdjustPCT = (getIMUDegDiffFromDir(targetAngle, currAngle, true) / absTurnAdjustMaxAngleThreshold) * maxAddTurnAdjustPCT + baseTurnAdjustPCT;
          //driveAdjustPCT = pow(getIMUDegDiffFromDir(targetAngle, currAngle, true) / absTurnAdjustMaxAngleThreshold, turnCurveExponent) * maxAddTurnAdjustPCT + baseTurnAdjustPCT;
          if (driveAdjustPCT > maxAddTurnAdjustPCT + baseTurnAdjustPCT) // limit driveAdjustPCT to max + base (in the case that robot angle to point is > specified threshold in formula)
            driveAdjustPCT = maxAddTurnAdjustPCT + baseTurnAdjustPCT;
        }
        else // if turning robot left is closer than turning right to face the point
        {
          driveAdjustPCT = -1.0 * (getIMUDegDiffFromDir(targetAngle, currAngle, false) / absTurnAdjustMaxAngleThreshold) * maxAddTurnAdjustPCT - baseTurnAdjustPCT;
          //driveAdjustPCT = -1.0 * pow(getIMUDegDiffFromDir(targetAngle, currAngle, false) / absTurnAdjustMaxAngleThreshold, turnCurveExponent) * maxAddTurnAdjustPCT - baseTurnAdjustPCT;
          if (driveAdjustPCT < -maxAddTurnAdjustPCT - baseTurnAdjustPCT) // limit driveAdjustPCT to -max - base (in the case that robot angle to point is > specified threshold in formula)
            driveAdjustPCT = -maxAddTurnAdjustPCT - baseTurnAdjustPCT;
        }
      }
      fLDrive_mtr.spin(fwd, adjustedSpeedPCT + driveAdjustPCT, pct);
      fRDrive_mtr.spin(fwd, adjustedSpeedPCT - driveAdjustPCT, pct);
      mLDrive_mtr.spin(fwd, adjustedSpeedPCT + driveAdjustPCT, pct);
      mRDrive_mtr.spin(fwd, adjustedSpeedPCT - driveAdjustPCT, pct);
      bLDrive_mtr.spin(fwd, adjustedSpeedPCT + driveAdjustPCT, pct);
      bRDrive_mtr.spin(fwd, adjustedSpeedPCT - driveAdjustPCT, pct);
    }
    else // if driving backward
    {
      rearAngle = currAngle + 180.0;
      if (rearAngle >= 360.0) // ensure rear angle is within 0 to 360 deg (IMU range)
      {
        rearAngle -= 360.0;
      }
      if (getIMUDegDiff(targetAngle, rearAngle) > absAngleThreshold) // if smallest angle from target to robot rear is greater than acceptable threshold
      {
        if (currDistToPoint <= requireStopRangeIn) // remove any speed percent that is not for turn adjusting if within specified range
          adjustedSpeedPCT = 0.0;
        if (getIMUDegDiffFromDir(targetAngle, rearAngle, true) <= getIMUDegDiffFromDir(targetAngle, rearAngle, false)) // if turning robot left on back is closer than turning right to face the point
        {
          driveAdjustPCT = (getIMUDegDiffFromDir(targetAngle, rearAngle, true) / absTurnAdjustMaxAngleThreshold) * maxAddTurnAdjustPCT + baseTurnAdjustPCT;
          //driveAdjustPCT = pow(getIMUDegDiffFromDir(targetAngle, currAngle, true) / absTurnAdjustMaxAngleThreshold, turnCurveExponent) * maxAddTurnAdjustPCT + baseTurnAdjustPCT;
          if (driveAdjustPCT > maxAddTurnAdjustPCT + baseTurnAdjustPCT) // limit driveAdjustPCT to max + base (in the case that robot angle to point is > specified threshold in formula)
            driveAdjustPCT = maxAddTurnAdjustPCT + baseTurnAdjustPCT;
        }
        else // if turning robot right on back is closer than turning left to face the point
        {
          driveAdjustPCT = -1.0 * (getIMUDegDiffFromDir(targetAngle, rearAngle, false) / absTurnAdjustMaxAngleThreshold) * maxAddTurnAdjustPCT - baseTurnAdjustPCT;
          //driveAdjustPCT = -1.0 * pow(getIMUDegDiffFromDir(targetAngle, currAngle, false) / absTurnAdjustMaxAngleThreshold, turnCurveExponent) * maxAddTurnAdjustPCT - baseTurnAdjustPCT;
          if (driveAdjustPCT < -maxAddTurnAdjustPCT - baseTurnAdjustPCT) // limit driveAdjustPCT to -max - base (in the case that robot angle to point is > specified threshold in formula)
            driveAdjustPCT = -maxAddTurnAdjustPCT - baseTurnAdjustPCT;
        }
      }
      fLDrive_mtr.spin(fwd, -adjustedSpeedPCT + driveAdjustPCT, pct);
      fRDrive_mtr.spin(fwd, -adjustedSpeedPCT - driveAdjustPCT, pct);
      mLDrive_mtr.spin(fwd, -adjustedSpeedPCT + driveAdjustPCT, pct);
      mRDrive_mtr.spin(fwd, -adjustedSpeedPCT - driveAdjustPCT, pct);
      bLDrive_mtr.spin(fwd, -adjustedSpeedPCT + driveAdjustPCT, pct);
      bRDrive_mtr.spin(fwd, -adjustedSpeedPCT - driveAdjustPCT, pct);
    }

    wait(5, msec);
    
    // count++;
    // xTotal += getX();
    // yTotal += getY();
    // if (count == 3)
    // {
    //   count = 0;
    //   xError = targetX - (xTotal / 3.0); // update values for next iteration
    //   yError = targetY - (yTotal / 3.0);
    currDistToPoint = sqrt(xError * xError + yError * yError);
    //   xTotal = 0.0;
    //   yTotal = 0.0;
    // }
    xError = targetX - getX();
    yError = targetY - getY();

  }

  stopDrive();

}

/*
* Version of above method driveToPoint with default values for the arguments after goingForward.
* Call this method when using driveToPoint with the default values for:
* absErrorThreshold, absAngleThreshold, decelRangeIn, requireStopRangeIn
*/
void driveToPoint(double targetX, double targetY, double maxSpeedPCT, bool goingForward)
{

  double absErrorThreshold = 5.0; // inches; xy allowance by end
  double maxAddTurnAdjustPCT = 150.0; // percent mtr power; max additional adjust to drive
  //double turnCurveExponent = 1.0; // decimal power; to infinity is less harsh turn, to 0 is harsher turn
  double decelRangeIn = 8.0; // inches; max range to start decelerating to point
  double requireStopRangeIn = 0.0; // inches; max range to start overriding straight drive to turn to point

  driveToPoint(targetX, targetY, maxSpeedPCT, goingForward, absErrorThreshold, maxAddTurnAdjustPCT, decelRangeIn, requireStopRangeIn);

}

/*
* Drive straight forward/backward (motion profiling).
* Motion profiling = motion achieved by acceleration, maxSpeed, and deceleration phases
* isPressureTracking == true, then exit driveForDistance when wheels have gotten "stuck" along drive
* Online references:
* http://georgegillard.com/programming-guides/introduction_to_pid_controllers_ed2-pdf?format=raw
* file:///C:/Users/MonkeyKing/Downloads/introduction_to_pid_controllers_ed2.pdf
*/
void driveForDistance(double dist, double maxSpeed, bool isPressureTracking)
{

  //double kP = 0.050; // 0.000 to 1.000
  double kP = 1.0; // 0 to inf INCHES
  double kDS = -0.450; // 0.000 to -inf
  double kDP = 0.100; // 0.000 to inf
  bool isInKP = false;

  double accelRangeIn = 2;
	double origin = fLDrive_mtr.position(deg);
	bool isForward = true;
	double absDist = dist;
	double absPosition = 0;
  int initialTimePassed = 0;
  bool leftSideDisable = false;
  bool rightSideDisable = false;
	if (absDist < 0.0) {
		isForward = false;
		absDist *= -1.0;
	}
	if (absDist < (accelRangeIn * 2.0))
		accelRangeIn = absDist / 2.0;
  absDist *= driveGearRatio;
  accelRangeIn *= driveGearRatio;
  kP *= driveGearRatio;
	double accelDistInDeg = convertWheelInAndDeg(false, accelRangeIn, wheelDiameterIn);
  double absDistInDeg = convertWheelInAndDeg(false, absDist, wheelDiameterIn);
  double PDDistInDeg = absDistInDeg - accelDistInDeg;
  double kSM = getCurveKSM(convertWheelInAndDeg(true, PDDistInDeg, wheelDiameterIn), kP);

	// accelerate period
	double error = accelDistInDeg;
	double absTargetPos = accelDistInDeg;
	double speedPCT;
  int totalErrorArrayLength = 3;
  double totalErrorArray[totalErrorArrayLength];
  for (int e = 0; e < totalErrorArrayLength; e++)
    totalErrorArray[e] = absDistInDeg;
  int i = 0;
  double totalErrorSlope;
  double derivativeMinSpeedPCT = 0.0;

	while (error > 0.0) {
  
    if (isPressureTracking && (bLDrive_mtr.velocity(pct) < 0.1) && (bLDrive_mtr.velocity(pct) > -0.1) && (initialTimePassed > 500))
      leftSideDisable = true;
    if (isPressureTracking && (bRDrive_mtr.velocity(pct) < 0.1) && (bRDrive_mtr.velocity(pct) > -0.1) && (initialTimePassed > 500))
      rightSideDisable = true;
    if (leftSideDisable && rightSideDisable)
      break;
		speedPCT = ((accelDistInDeg - error) / accelDistInDeg) * maxSpeed;
		if (speedPCT > maxSpeed)
			speedPCT = maxSpeed;
    else if (speedPCT < 15.0)
      speedPCT = 15.0;
		if (isForward) {
      if (!leftSideDisable)
      {
        fLDrive_mtr.spin(fwd, speedPCT, pct);
        mLDrive_mtr.spin(fwd, speedPCT, pct);
        bLDrive_mtr.spin(fwd, speedPCT, pct);
      }
      if (!rightSideDisable)
      {
        fRDrive_mtr.spin(fwd, speedPCT, pct);
        mRDrive_mtr.spin(fwd, speedPCT, pct);
		  	bRDrive_mtr.spin(fwd, speedPCT, pct);
      }
		}
    else
    {
			if (!leftSideDisable)
      {
        fLDrive_mtr.spin(fwd, -speedPCT, pct);
        mLDrive_mtr.spin(fwd, -speedPCT, pct);
        bLDrive_mtr.spin(fwd, -speedPCT, pct);
      }
      if (!rightSideDisable)
      {
        fRDrive_mtr.spin(fwd, -speedPCT, pct);
        mRDrive_mtr.spin(fwd, -speedPCT, pct);
		  	bRDrive_mtr.spin(fwd, -speedPCT, pct);
      }
		}
  
		wait(5, msec);
  
    initialTimePassed += 5;
    absPosition = doubleMathAbs(fLDrive_mtr.position(deg) - origin);
		error = absTargetPos - absPosition;
    totalErrorArray[i] = absDistInDeg - absPosition;
    i++;
    if (i >= totalErrorArrayLength)
      i = 0;
    
	}

	// period of PD motion (rest of drive)
  double mathPowerParameter;
	absTargetPos = absDistInDeg;
  error = absTargetPos - absPosition;

	while (error > 0.0) {
  
    if (convertWheelInAndDeg(true, error, wheelDiameterIn) <= kP)
      isInKP = true;

    if (isPressureTracking && (bLDrive_mtr.velocity(pct) < 0.1) && (bLDrive_mtr.velocity(pct) > -0.1))
      leftSideDisable = true;
    if (isPressureTracking && (bRDrive_mtr.velocity(pct) < 0.1) && (bRDrive_mtr.velocity(pct) > -0.1))
      rightSideDisable = true;
    if (leftSideDisable && rightSideDisable)
      break;
    mathPowerParameter = (convertWheelInAndDeg(true, error, wheelDiameterIn) - kP) / 100.0;
    if (mathPowerParameter > 0.0)
    {
      speedPCT = (pow(mathPowerParameter, 0.52) * 100.0)
                  / (convertWheelInAndDeg(true, PDDistInDeg, wheelDiameterIn) - kP)
                  * (maxSpeed * kSM);
    }
    else
    {
      speedPCT = 0.0;
    }
		if ((speedPCT + derivativeMinSpeedPCT) > maxSpeed)
			speedPCT = maxSpeed;
    else if (speedPCT < 0.0)
      speedPCT = derivativeMinSpeedPCT;
    else
      speedPCT += derivativeMinSpeedPCT;

    if (isInKP && speedPCT > 30.0)
    {
      speedPCT = 30.0;
    }

		if (isForward) {
      if (!leftSideDisable)
      {
        fLDrive_mtr.spin(fwd, speedPCT, pct);
        mLDrive_mtr.spin(fwd, speedPCT, pct);
        bLDrive_mtr.spin(fwd, speedPCT, pct);
      }
      if (!rightSideDisable)
      {
        fRDrive_mtr.spin(fwd, speedPCT, pct);
        mRDrive_mtr.spin(fwd, speedPCT, pct);
		  	bRDrive_mtr.spin(fwd, speedPCT, pct);
      }
		}
    else
    {
			if (!leftSideDisable)
      {
        fLDrive_mtr.spin(fwd, -speedPCT, pct);
        mLDrive_mtr.spin(fwd, -speedPCT, pct);
        bLDrive_mtr.spin(fwd, -speedPCT, pct);
      }
      if (!rightSideDisable)
      {
        fRDrive_mtr.spin(fwd, -speedPCT, pct);
        mRDrive_mtr.spin(fwd, -speedPCT, pct);
		  	bRDrive_mtr.spin(fwd, -speedPCT, pct);
      }
		}
  
		wait(5, msec);
  
    absPosition = doubleMathAbs(fLDrive_mtr.position(deg) - origin);
		error = absTargetPos - absPosition;
    totalErrorArray[i] = absDistInDeg - absPosition;
    if ((i + 1) >= totalErrorArrayLength)
      totalErrorSlope = (totalErrorArray[i] - totalErrorArray[0]) / ((totalErrorArrayLength - 1) * 5.0);
    else
      totalErrorSlope = (totalErrorArray[i] - totalErrorArray[i + 1]) / ((totalErrorArrayLength - 1) * 5.0);
    if (totalErrorSlope >= kDS)
      derivativeMinSpeedPCT += kDP;
    else
      derivativeMinSpeedPCT -= kDP;
    if (derivativeMinSpeedPCT < 0.0)
      derivativeMinSpeedPCT = 0.0;
    i++;
    if (i >= totalErrorArrayLength)
      i = 0;
  
	}

	stopDrive();

}

// use for driving into platform
void driveForDistanceSSPT(double dist, double maxSpeed)
{

  //double kP = 0.050; // 0.000 to 1.000
  double kP = 1.0; // 0 to inf INCHES
  double kDS = -0.450; // 0.000 to -inf
  double kDP = 0.100; // 0.000 to inf
  bool isInKP = false;

  double accelRangeIn = 2;
	double origin = fLDrive_mtr.position(deg);
	bool isForward = true;
	double absDist = dist;
	double absPosition = 0;
  int initialTimePassed = 0;
  bool leftSideDisable = false;
  bool rightSideDisable = false;
	if (absDist < 0.0) {
		isForward = false;
		absDist *= -1.0;
	}
	if (absDist < (accelRangeIn * 2.0))
		accelRangeIn = absDist / 2.0;
  absDist *= driveGearRatio;
  accelRangeIn *= driveGearRatio;
  kP *= driveGearRatio;
	double accelDistInDeg = convertWheelInAndDeg(false, accelRangeIn, wheelDiameterIn);
  double absDistInDeg = convertWheelInAndDeg(false, absDist, wheelDiameterIn);
  double PDDistInDeg = absDistInDeg - accelDistInDeg;
  double kSM = getCurveKSM(convertWheelInAndDeg(true, PDDistInDeg, wheelDiameterIn), kP);

	// accelerate period
	double error = accelDistInDeg;
	double absTargetPos = accelDistInDeg;
	double speedPCT;
  int totalErrorArrayLength = 3;
  double totalErrorArray[totalErrorArrayLength];
  for (int e = 0; e < totalErrorArrayLength; e++)
    totalErrorArray[e] = absDistInDeg;
  int i = 0;
  double totalErrorSlope;
  double derivativeMinSpeedPCT = 0.0;

	while (error > 0.0) {
  
    if ((bLDrive_mtr.velocity(pct) < 0.1) && (bLDrive_mtr.velocity(pct) > -0.1) && (initialTimePassed > 500))
      leftSideDisable = true;
    if ((bRDrive_mtr.velocity(pct) < 0.1) && (bRDrive_mtr.velocity(pct) > -0.1) && (initialTimePassed > 500))
      rightSideDisable = true;
    if (leftSideDisable || rightSideDisable)
      break;
		speedPCT = ((accelDistInDeg - error) / accelDistInDeg) * maxSpeed;
		if (speedPCT > maxSpeed)
			speedPCT = maxSpeed;
    else if (speedPCT < 15.0)
      speedPCT = 15.0;
		if (isForward) {
      if (!leftSideDisable)
      {
        fLDrive_mtr.spin(fwd, speedPCT, pct);
        mLDrive_mtr.spin(fwd, speedPCT, pct);
        bLDrive_mtr.spin(fwd, speedPCT, pct);
      }
      if (!rightSideDisable)
      {
        fRDrive_mtr.spin(fwd, speedPCT, pct);
        mRDrive_mtr.spin(fwd, speedPCT, pct);
		  	bRDrive_mtr.spin(fwd, speedPCT, pct);
      }
		}
    else
    {
			if (!leftSideDisable)
      {
        fLDrive_mtr.spin(fwd, -speedPCT, pct);
        mLDrive_mtr.spin(fwd, -speedPCT, pct);
        bLDrive_mtr.spin(fwd, -speedPCT, pct);
      }
      if (!rightSideDisable)
      {
        fRDrive_mtr.spin(fwd, -speedPCT, pct);
        mRDrive_mtr.spin(fwd, -speedPCT, pct);
		  	bRDrive_mtr.spin(fwd, -speedPCT, pct);
      }
		}
  
		wait(5, msec);
  
    initialTimePassed += 5;
    absPosition = doubleMathAbs(fLDrive_mtr.position(deg) - origin);
		error = absTargetPos - absPosition;
    totalErrorArray[i] = absDistInDeg - absPosition;
    i++;
    if (i >= totalErrorArrayLength)
      i = 0;
    
	}

	// period of PD motion (rest of drive)
  double mathPowerParameter;
	absTargetPos = absDistInDeg;
  error = absTargetPos - absPosition;

	while (error > 0.0) {
  
    if (convertWheelInAndDeg(true, error, wheelDiameterIn) <= kP)
      isInKP = true;

    if ((bLDrive_mtr.velocity(pct) < 0.1) && (bLDrive_mtr.velocity(pct) > -0.1))
      leftSideDisable = true;
    if ((bRDrive_mtr.velocity(pct) < 0.1) && (bRDrive_mtr.velocity(pct) > -0.1))
      rightSideDisable = true;
    if (leftSideDisable || rightSideDisable)
      break;
    mathPowerParameter = (convertWheelInAndDeg(true, error, wheelDiameterIn) - kP) / 100.0;
    if (mathPowerParameter > 0.0)
    {
      speedPCT = (pow(mathPowerParameter, 0.52) * 100.0)
                  / (convertWheelInAndDeg(true, PDDistInDeg, wheelDiameterIn) - kP)
                  * (maxSpeed * kSM);
    }
    else
    {
      speedPCT = 0.0;
    }
		if ((speedPCT + derivativeMinSpeedPCT) > maxSpeed)
			speedPCT = maxSpeed;
    else if (speedPCT < 0.0)
      speedPCT = derivativeMinSpeedPCT;
    else
      speedPCT += derivativeMinSpeedPCT;

    if (isInKP && speedPCT > 30.0)
    {
      speedPCT = 30.0;
    }

		if (isForward) {
      if (!leftSideDisable)
      {
        fLDrive_mtr.spin(fwd, speedPCT, pct);
        mLDrive_mtr.spin(fwd, speedPCT, pct);
        bLDrive_mtr.spin(fwd, speedPCT, pct);
      }
      if (!rightSideDisable)
      {
        fRDrive_mtr.spin(fwd, speedPCT, pct);
        mRDrive_mtr.spin(fwd, speedPCT, pct);
		  	bRDrive_mtr.spin(fwd, speedPCT, pct);
      }
		}
    else
    {
			if (!leftSideDisable)
      {
        fLDrive_mtr.spin(fwd, -speedPCT, pct);
        mLDrive_mtr.spin(fwd, -speedPCT, pct);
        bLDrive_mtr.spin(fwd, -speedPCT, pct);
      }
      if (!rightSideDisable)
      {
        fRDrive_mtr.spin(fwd, -speedPCT, pct);
        mRDrive_mtr.spin(fwd, -speedPCT, pct);
		  	bRDrive_mtr.spin(fwd, -speedPCT, pct);
      }
		}
  
		wait(5, msec);
  
    absPosition = doubleMathAbs(fLDrive_mtr.position(deg) - origin);
		error = absTargetPos - absPosition;
    totalErrorArray[i] = absDistInDeg - absPosition;
    if ((i + 1) >= totalErrorArrayLength)
      totalErrorSlope = (totalErrorArray[i] - totalErrorArray[0]) / ((totalErrorArrayLength - 1) * 5.0);
    else
      totalErrorSlope = (totalErrorArray[i] - totalErrorArray[i + 1]) / ((totalErrorArrayLength - 1) * 5.0);
    if (totalErrorSlope >= kDS)
      derivativeMinSpeedPCT += kDP;
    else
      derivativeMinSpeedPCT -= kDP;
    if (derivativeMinSpeedPCT < 0.0)
      derivativeMinSpeedPCT = 0.0;
    i++;
    if (i >= totalErrorArrayLength)
      i = 0;
  
	}

	stopDrive();

}

// use for driving back to alliance side w/ middle mogos
void driveBackToSide(double wallThresh, double targetHeading, double maxSpeed)
{

  double accelRangeIn = 2;
	double origin = fLDrive_mtr.position(deg);
	double absPosition = 0;
  accelRangeIn *= driveGearRatio;
	double accelDistInDeg = convertWheelInAndDeg(false, accelRangeIn, wheelDiameterIn);

  double left = 0;
  double right = 0;
  double headingThresh = 0.5;

	// accelerate period
	double error = accelDistInDeg;
	double absTargetPos = accelDistInDeg;
	double speedPCT;

	while (error > 0.0) {

    if (backRange_sensor.distance(inches) < wallThresh)
    {
      break;
    }

    if (getIMUDegDiff(targetHeading, imu_sensor.heading(deg)) > headingThresh)
    {
      if (getIMUDegDiffFromDir(targetHeading, imu_sensor.heading(deg), true) < getIMUDegDiffFromDir(targetHeading, imu_sensor.heading(deg), false))
      {
        left = 40;
        right = -40;
      }
      else
      {
        left = -40;
        right = 40;
      }
    }
    else
    {
      left = 0;
      right = 0;
    }
  
		speedPCT = ((accelDistInDeg - error) / accelDistInDeg) * maxSpeed;
		if (speedPCT > maxSpeed)
			speedPCT = maxSpeed;
    else if (speedPCT < 65.0)
      speedPCT = 65.0;
        fLDrive_mtr.spin(fwd, -speedPCT + left, pct);
        mLDrive_mtr.spin(fwd, -speedPCT + left, pct);
        bLDrive_mtr.spin(fwd, -speedPCT + left, pct);
        fRDrive_mtr.spin(fwd, -speedPCT + right, pct);
        mRDrive_mtr.spin(fwd, -speedPCT + right, pct);
		  	bRDrive_mtr.spin(fwd, -speedPCT + right, pct);
    if (!is6MtrDrive)
    {
      lift_mtr.spin(fwd, -speedPCT + right, pct);
      intake_mtr.spin(fwd, -speedPCT + left, pct);
    }
  
		wait(5, msec);
  
    absPosition = doubleMathAbs(fLDrive_mtr.position(deg) - origin);
		error = absTargetPos - absPosition;
    
	}

	// period of sonar drive

	while (backRange_sensor.distance(inches) > wallThresh) {

    if (getIMUDegDiff(targetHeading, imu_sensor.heading(deg)) > headingThresh)
    {
      if (getIMUDegDiffFromDir(targetHeading, imu_sensor.heading(deg), true) < getIMUDegDiffFromDir(targetHeading, imu_sensor.heading(deg), false))
      {
        left = 40;
        right = -40;
      }
      else
      {
        left = -40;
        right = 40;
      }
    }
    else
    {
      left = 0;
      right = 0;
    }
  
    speedPCT = maxSpeed;

        fLDrive_mtr.spin(fwd, -speedPCT + left, pct);
        mLDrive_mtr.spin(fwd, -speedPCT + left, pct);
        bLDrive_mtr.spin(fwd, -speedPCT + left, pct);
        fRDrive_mtr.spin(fwd, -speedPCT + right, pct);
        mRDrive_mtr.spin(fwd, -speedPCT + right, pct);
		  	bRDrive_mtr.spin(fwd, -speedPCT + right, pct);

        if (!is6MtrDrive)
        {
          lift_mtr.spin(fwd, -speedPCT + right, pct);
          intake_mtr.spin(fwd, -speedPCT + left, pct);
        }
      
  
		wait(5, msec);
  
	}

	stopDrive();

}

/*
* use for getting middle mogos
*/
void driveToMiddleMogos(double dist, double maxSpeed, double accelBaseSpeed)
{

  double accelRangeIn = 2;
	double origin = fLDrive_mtr.position(deg);
	bool isForward = true;
	double absDist = dist;
	double absPosition = 0;
	if (absDist < 0.0) {
		isForward = false;
		absDist *= -1.0;
	}
	if (absDist < (accelRangeIn * 2.0))
		accelRangeIn = absDist / 2.0;
  absDist *= driveGearRatio;
  accelRangeIn *= driveGearRatio;
	double accelDistInDeg = convertWheelInAndDeg(false, accelRangeIn, wheelDiameterIn);
  double absDistInDeg = convertWheelInAndDeg(false, absDist, wheelDiameterIn);

	// accelerate period
	double error = accelDistInDeg;
	double absTargetPos = accelDistInDeg;
	double speedPCT;

	while (error > 0.0) {
  
		speedPCT = ((accelDistInDeg - error) / accelDistInDeg) * 100.0;
		if (speedPCT > maxSpeed)
			speedPCT = maxSpeed;
    else if (speedPCT < accelBaseSpeed)
      speedPCT = accelBaseSpeed;
		if (isForward) {
      fLDrive_mtr.spin(fwd, speedPCT, pct);
      mLDrive_mtr.spin(fwd, speedPCT, pct);
      bLDrive_mtr.spin(fwd, speedPCT, pct);
      fRDrive_mtr.spin(fwd, speedPCT, pct);
      mRDrive_mtr.spin(fwd, speedPCT, pct);
      bRDrive_mtr.spin(fwd, speedPCT, pct);
      if (!is6MtrDrive)
      {
        lift_mtr.spin(fwd, speedPCT, pct);
        intake_mtr.spin(fwd, speedPCT, pct);
      }
		}
    else
    {
      fLDrive_mtr.spin(fwd, -speedPCT, pct);
      mLDrive_mtr.spin(fwd, -speedPCT, pct);
      bLDrive_mtr.spin(fwd, -speedPCT, pct);
      fRDrive_mtr.spin(fwd, -speedPCT, pct);
      mRDrive_mtr.spin(fwd, -speedPCT, pct);
      bRDrive_mtr.spin(fwd, -speedPCT, pct);
      if (!is6MtrDrive)
      {
        lift_mtr.spin(fwd, -speedPCT, pct);
        intake_mtr.spin(fwd, -speedPCT, pct);
      }
		}
  
		wait(5, msec);
  
    absPosition = doubleMathAbs(fLDrive_mtr.position(deg) - origin);
		error = absTargetPos - absPosition;
    
	}

  absTargetPos = absDistInDeg;
  error = absTargetPos - absPosition;

	while (error > 0.0) {

    if (claw_sensor.isNearObject())
    {
      break;
    }

    speedPCT = maxSpeed;

		if (isForward) {
      fLDrive_mtr.spin(fwd, speedPCT, pct);
      mLDrive_mtr.spin(fwd, speedPCT, pct);
      bLDrive_mtr.spin(fwd, speedPCT, pct);
      fRDrive_mtr.spin(fwd, speedPCT, pct);
      mRDrive_mtr.spin(fwd, speedPCT, pct);
      bRDrive_mtr.spin(fwd, speedPCT, pct);
      if (!is6MtrDrive)
      {
        lift_mtr.spin(fwd, speedPCT, pct);
        intake_mtr.spin(fwd, speedPCT, pct);
      }
		}
    else
    {
      fLDrive_mtr.spin(fwd, -speedPCT, pct);
      mLDrive_mtr.spin(fwd, -speedPCT, pct);
      bLDrive_mtr.spin(fwd, -speedPCT, pct);
      fRDrive_mtr.spin(fwd, -speedPCT, pct);
      mRDrive_mtr.spin(fwd, -speedPCT, pct);
      bRDrive_mtr.spin(fwd, -speedPCT, pct);
      if (!is6MtrDrive)
      {
        lift_mtr.spin(fwd, -speedPCT, pct);
        intake_mtr.spin(fwd, -speedPCT, pct);
      }
		}
  
		wait(5, msec);
  
    absPosition = doubleMathAbs(fLDrive_mtr.position(deg) - origin);
		error = absTargetPos - absPosition;
  
	}

	stopDrive();

}

/*
* Turn to specified degree angle from any current angle
* NOTE: Turns in direction of least degrees to travel
* Also: Does not include an acceleration profile to minimize time spent on task
*/
void turnToDegree(double targetDeg, double maxSpeedPCT)
{

  double kP = 0.005; // 0.000 to 1.000
  double kDS = -0.040; // 0.000 to -inf
  double kDP = 0.050; // 0.000 to inf

  double absAngleThreshold = 0.75; // degrees
  double decelDeg = 180.0; // degrees
  //double baseSpeedPCT = 0.0; // percent mtr power
  double speedPCT;
	double currDeg = imu_sensor.heading(deg);
  double degDiff = getIMUDegDiff(targetDeg, currDeg);
  /*if (degDiff < 80.0)
    baseSpeedPCT = 5.0; // percent mtr power (when smaller turns)*/
  bool turningRight = false;
  // decide if turning left or right (whichever is faster)
  if (getIMUDegDiffFromDir(targetDeg, currDeg, true) < getIMUDegDiffFromDir(targetDeg, currDeg, false))
    turningRight = true;

  int totalErrorArrayLength = 5;
  double totalErrorArray[totalErrorArrayLength];
  for (int e = 0; e < totalErrorArrayLength; e++)
    totalErrorArray[e] = degDiff;
  int i = 0;
  double totalErrorSlope;
  double derivativeMinSpeedPCT = 0.0;
  double mathPowerParameter;
  
  while (degDiff > absAngleThreshold)
  {
    
    mathPowerParameter = (degDiff - (kP * decelDeg)) / 100.0;
    if (mathPowerParameter > 0.0)
    {
      speedPCT = ((pow(mathPowerParameter, 1.0) * 100.0)
                  / (decelDeg * (1.0 - kP)))
                  * maxSpeedPCT;
    }
    else
    {
      speedPCT = 0.0;
    }
    if ((speedPCT + derivativeMinSpeedPCT) > maxSpeedPCT)
      speedPCT = maxSpeedPCT;
    else if (speedPCT < 0.0)
      speedPCT = derivativeMinSpeedPCT;
    else
      speedPCT += derivativeMinSpeedPCT;
    if (turningRight)
    {
      fLDrive_mtr.spin(fwd, speedPCT, pct);
      fRDrive_mtr.spin(fwd, -speedPCT, pct);
      mLDrive_mtr.spin(fwd, speedPCT, pct);
      mRDrive_mtr.spin(fwd, -speedPCT, pct);
      bLDrive_mtr.spin(fwd, speedPCT, pct);
      bRDrive_mtr.spin(fwd, -speedPCT, pct);
    }
    else
    {
      fLDrive_mtr.spin(fwd, -speedPCT, pct);
      fRDrive_mtr.spin(fwd, speedPCT, pct);
      mLDrive_mtr.spin(fwd, -speedPCT, pct);
      mRDrive_mtr.spin(fwd, speedPCT, pct);
      bLDrive_mtr.spin(fwd, -speedPCT, pct);
      bRDrive_mtr.spin(fwd, speedPCT, pct);
    }

    wait(5, msec);

    currDeg = imu_sensor.heading(deg);
    degDiff = getIMUDegDiff(targetDeg, currDeg);
    if (getIMUDegDiffFromDir(targetDeg, currDeg, true) < getIMUDegDiffFromDir(targetDeg, currDeg, false))
      turningRight = true;
    else
      turningRight = false;

    totalErrorArray[i] = degDiff;
    if ((i + 1) >= totalErrorArrayLength)
      totalErrorSlope = (totalErrorArray[i] - totalErrorArray[0]) / ((totalErrorArrayLength - 1) * 5.0);
    else
      totalErrorSlope = (totalErrorArray[i] - totalErrorArray[i + 1]) / ((totalErrorArrayLength - 1) * 5.0);
    if (totalErrorSlope >= kDS)
      derivativeMinSpeedPCT += kDP;
    else
      derivativeMinSpeedPCT -= kDP;
    if (derivativeMinSpeedPCT < 0.0)
      derivativeMinSpeedPCT = 0.0;
    i++;
    if (i >= totalErrorArrayLength)
      i = 0;

  }

	stopDrive();

}

void turnToDegreeGPS(double targetDeg, double maxSpeedPCT)
{

  double kP = 0.005; // 0.000 to 1.000
  double kDS = -0.040; // 0.000 to -inf
  double kDP = 0.050; // 0.000 to inf

  double absAngleThreshold = 0.75; // degrees
  double decelDeg = 180.0; // degrees
  //double baseSpeedPCT = 0.0; // percent mtr power
  double speedPCT;
	double currDeg = gps_sensor.heading(deg);
  double degDiff = getIMUDegDiff(targetDeg, currDeg);
  /*if (degDiff < 80.0)
    baseSpeedPCT = 5.0; // percent mtr power (when smaller turns)*/
  bool turningRight = false;
  // decide if turning left or right (whichever is faster)
  if (getIMUDegDiffFromDir(targetDeg, currDeg, true) < getIMUDegDiffFromDir(targetDeg, currDeg, false))
    turningRight = true;

  int totalErrorArrayLength = 5;
  double totalErrorArray[totalErrorArrayLength];
  for (int e = 0; e < totalErrorArrayLength; e++)
    totalErrorArray[e] = degDiff;
  int i = 0;
  double totalErrorSlope;
  double derivativeMinSpeedPCT = 0.0;
  double mathPowerParameter;
  
  while (degDiff > absAngleThreshold)
  {
    
    mathPowerParameter = (degDiff - (kP * decelDeg)) / 100.0;
    if (mathPowerParameter > 0.0)
    {
      speedPCT = ((pow(mathPowerParameter, 1.0) * 100.0)
                  / (decelDeg * (1.0 - kP)))
                  * maxSpeedPCT;
    }
    else
    {
      speedPCT = 0.0;
    }
    if ((speedPCT + derivativeMinSpeedPCT) > maxSpeedPCT)
      speedPCT = maxSpeedPCT;
    else if (speedPCT < 0.0)
      speedPCT = derivativeMinSpeedPCT;
    else
      speedPCT += derivativeMinSpeedPCT;
    if (turningRight)
    {
      fLDrive_mtr.spin(fwd, speedPCT, pct);
      fRDrive_mtr.spin(fwd, -speedPCT, pct);
      mLDrive_mtr.spin(fwd, speedPCT, pct);
      mRDrive_mtr.spin(fwd, -speedPCT, pct);
      bLDrive_mtr.spin(fwd, speedPCT, pct);
      bRDrive_mtr.spin(fwd, -speedPCT, pct);
    }
    else
    {
      fLDrive_mtr.spin(fwd, -speedPCT, pct);
      fRDrive_mtr.spin(fwd, speedPCT, pct);
      mLDrive_mtr.spin(fwd, -speedPCT, pct);
      mRDrive_mtr.spin(fwd, speedPCT, pct);
      bLDrive_mtr.spin(fwd, -speedPCT, pct);
      bRDrive_mtr.spin(fwd, speedPCT, pct);
    }

    wait(5, msec);

    currDeg = gps_sensor.heading(deg);
    degDiff = getIMUDegDiff(targetDeg, currDeg);
    if (getIMUDegDiffFromDir(targetDeg, currDeg, true) < getIMUDegDiffFromDir(targetDeg, currDeg, false))
      turningRight = true;
    else
      turningRight = false;

    totalErrorArray[i] = degDiff;
    if ((i + 1) >= totalErrorArrayLength)
      totalErrorSlope = (totalErrorArray[i] - totalErrorArray[0]) / ((totalErrorArrayLength - 1) * 5.0);
    else
      totalErrorSlope = (totalErrorArray[i] - totalErrorArray[i + 1]) / ((totalErrorArrayLength - 1) * 5.0);
    if (totalErrorSlope >= kDS)
      derivativeMinSpeedPCT += kDP;
    else
      derivativeMinSpeedPCT -= kDP;
    if (derivativeMinSpeedPCT < 0.0)
      derivativeMinSpeedPCT = 0.0;
    i++;
    if (i >= totalErrorArrayLength)
      i = 0;

  }

	stopDrive();

}

/*
* Returns a double representing distance from robot to target position in inches.
*/
double getDistanceToPoint(double targetX, double targetY)
{
  double xError = targetX - getX();
  double yError = targetY - getY();
  return sqrt(xError * xError + yError * yError);
}

double getAngleToPoint(double targetX, double targetY)
{
  double xError = targetX - getX();
  double yError = targetY - getY();
  double targetAngle = atan(yError / xError) * (180.0 / M_PI);
  if (xError < 0.0)
    targetAngle += 180.0;
  targetAngle = convertIMUAndCoordDeg(targetAngle);
  return targetAngle;
}

/*
* Makes robot turn in place to the target position.
*/
void turnToPoint(double targetX, double targetY, double maxSpeedPCT, bool faceFwd)
{

  double kP = 0.005; // 0.000 to 1.000
  double kDS = -0.040; // 0.000 to -inf
  double kDP = 0.050; // 0.000 to inf

  double absAngleThreshold = 0.75; // degrees
  double decelDeg = 180.0; // degrees
  //double baseSpeedPCT = 0.0; // percent mtr power
  double speedPCT;
  double xError = targetX - getX();
  double yError = targetY - getY();
  double targetDeg = atan(yError / xError) * (180 / M_PI);
  if (xError < 0.0)
    targetDeg += 180.0;
  targetDeg = convertIMUAndCoordDeg(targetDeg);
  double currDeg = gps_sensor.heading(deg);
  if (!faceFwd)
  {
	  currDeg += 180.0;
    currDeg = getDegWithin360(currDeg);
  }
  double degDiff = getIMUDegDiff(targetDeg, currDeg);
  /*if (degDiff < 80.0)
    baseSpeedPCT = 5.0;*/
  bool turningRight = false;
  // decide if turning left or right (whichever is faster)
  if (getIMUDegDiffFromDir(targetDeg, currDeg, true) < getIMUDegDiffFromDir(targetDeg, currDeg, false))
    turningRight = true;

  int totalErrorArrayLength = 5;
  double totalErrorArray[totalErrorArrayLength];
  for (int e = 0; e < totalErrorArrayLength; e++)
    totalErrorArray[e] = degDiff;
  int i = 0;
  double totalErrorSlope;
  double derivativeMinSpeedPCT = 0.0;
  double mathPowerParameter;

  while (degDiff > absAngleThreshold)
  {
  
    mathPowerParameter = (degDiff - (kP * decelDeg)) / 100.0;
    if (mathPowerParameter > 0.0)
    {
      speedPCT = ((pow(mathPowerParameter, 1.0) * 100.0)
                  / (decelDeg * (1.0 - kP)))
                  * maxSpeedPCT;
    }
    else
    {
      speedPCT = 0.0;
    }
    if ((speedPCT + derivativeMinSpeedPCT) > maxSpeedPCT)
      speedPCT = maxSpeedPCT;
    else if (speedPCT < 0.0)
      speedPCT = derivativeMinSpeedPCT;
    else
      speedPCT += derivativeMinSpeedPCT;
    if (turningRight)
    {
      fLDrive_mtr.spin(fwd, speedPCT, pct);
      fRDrive_mtr.spin(fwd, -speedPCT, pct);
      mLDrive_mtr.spin(fwd, speedPCT, pct);
      mRDrive_mtr.spin(fwd, -speedPCT, pct);
      bLDrive_mtr.spin(fwd, speedPCT, pct);
      bRDrive_mtr.spin(fwd, -speedPCT, pct);
    }
    else
    {
      fLDrive_mtr.spin(fwd, -speedPCT, pct);
      fRDrive_mtr.spin(fwd, speedPCT, pct);
      mLDrive_mtr.spin(fwd, -speedPCT, pct);
      mRDrive_mtr.spin(fwd, speedPCT, pct);
      bLDrive_mtr.spin(fwd, -speedPCT, pct);
      bRDrive_mtr.spin(fwd, speedPCT, pct);
    }
  
    wait(5, msec);
  
    xError = targetX - getX();
    yError = targetY - getY();
    targetDeg = atan(yError / xError) * (180.0 / M_PI);
    if (xError < 0.0)
      targetDeg += 180.0;
    targetDeg = convertIMUAndCoordDeg(targetDeg);
    currDeg = gps_sensor.heading(deg);
    if (!faceFwd)
    {
      currDeg += 180.0;
      currDeg = getDegWithin360(currDeg);
    }
    degDiff = getIMUDegDiff(targetDeg, currDeg);
    if (getIMUDegDiffFromDir(targetDeg, currDeg, true) < getIMUDegDiffFromDir(targetDeg, currDeg, false))
      turningRight = true;
    else
      turningRight = false;
    
    totalErrorArray[i] = degDiff;
    if ((i + 1) >= totalErrorArrayLength)
      totalErrorSlope = (totalErrorArray[i] - totalErrorArray[0]) / ((totalErrorArrayLength - 1) * 5.0);
    else
      totalErrorSlope = (totalErrorArray[i] - totalErrorArray[i + 1]) / ((totalErrorArrayLength - 1) * 5.0);
    if (totalErrorSlope >= kDS)
      derivativeMinSpeedPCT += kDP;
    else
      derivativeMinSpeedPCT -= kDP;
    if (derivativeMinSpeedPCT < 0.0)
      derivativeMinSpeedPCT = 0.0;
    i++;
    if (i >= totalErrorArrayLength)
      i = 0;
    
  }

	stopDrive();

}

void takeBackSnapshot(int targetColor)
{
  if (targetColor == 0)
    back_sensor.takeSnapshot(back_sensor__BLUE_ALLIANCE);
  else if (targetColor == 1)
    back_sensor.takeSnapshot(back_sensor__RED_ALLIANCE);
  else
    back_sensor.takeSnapshot(back_sensor__YELLOW_NEUTRAL);
}

// targetColor = 0, 1, 2; blue, red, yellow respectively
void turnToBackMogo(int targetColor, double maxSpeedPCT, bool isDefaultRight)
{

  double absPixelThreshold = 5; // pixels
  double pixelTurnScale = 160; // pixels
  double minSpeedPCT = 2.5; // motor pwr pct
  double speedPCT;
  double pixelError;
  bool isFacing = false;
  vision::object targetMogo;
  
  while (!isFacing)
  {

    takeBackSnapshot(targetColor);
    targetMogo = back_sensor.largestObject;
    if (targetMogo.exists)
    {
      pixelError = (double)targetMogo.centerX - 119.0;
      if (doubleMathAbs(pixelError) < absPixelThreshold)
        isFacing = true;
      speedPCT = pixelError / pixelTurnScale * maxSpeedPCT;
      if (doubleMathAbs(speedPCT) < minSpeedPCT)
        speedPCT = speedPCT / doubleMathAbs(speedPCT) * minSpeedPCT;
      fLDrive_mtr.spin(fwd, speedPCT, pct);
      fRDrive_mtr.spin(fwd, -speedPCT, pct);
      mLDrive_mtr.spin(fwd, speedPCT, pct);
      mRDrive_mtr.spin(fwd, -speedPCT, pct);
      bLDrive_mtr.spin(fwd, speedPCT, pct);
      bRDrive_mtr.spin(fwd, -speedPCT, pct);
    }
    else
    {
      if (isDefaultRight)
      {
        fLDrive_mtr.spin(fwd, maxSpeedPCT, pct);
        fRDrive_mtr.spin(fwd, -maxSpeedPCT, pct);
        mLDrive_mtr.spin(fwd, maxSpeedPCT, pct);
        mRDrive_mtr.spin(fwd, -maxSpeedPCT, pct);
        bLDrive_mtr.spin(fwd, maxSpeedPCT, pct);
        bRDrive_mtr.spin(fwd, -maxSpeedPCT, pct);
      }
      else
      {
        fLDrive_mtr.spin(fwd, -maxSpeedPCT, pct);
        fRDrive_mtr.spin(fwd, maxSpeedPCT, pct);
        mLDrive_mtr.spin(fwd, -maxSpeedPCT, pct);
        mRDrive_mtr.spin(fwd, maxSpeedPCT, pct);
        bLDrive_mtr.spin(fwd, -maxSpeedPCT, pct);
        bRDrive_mtr.spin(fwd, maxSpeedPCT, pct);
      }
    }

    wait(5, msec);

  }

	stopDrive();

}

void turnToBackMogoWithDeg(int targetColor, double maxSpeedPCT, bool isDefaultRight, double defaultDeg)
{

  double absPixelThreshold = 5; // pixels
  double pixelTurnScale = 160; // pixels
  double minSpeedPCT = 2.5; // motor pwr pct
  double degTravelled = 0;
  double prevDeg = imu_sensor.heading(deg);
  double currDeg;
  double speedPCT;
  double pixelError;
  bool isFacing = false;
  vision::object targetMogo;
  
  while (!isFacing)
  {

    takeBackSnapshot(targetColor);
    targetMogo = back_sensor.largestObject;
    if (targetMogo.exists)
    {
      pixelError = (double)targetMogo.centerX - 119.0;
      if (doubleMathAbs(pixelError) < absPixelThreshold)
        isFacing = true;
      speedPCT = pixelError / pixelTurnScale * maxSpeedPCT;
      if (doubleMathAbs(speedPCT) < minSpeedPCT)
        speedPCT = speedPCT / doubleMathAbs(speedPCT) * minSpeedPCT;
      fLDrive_mtr.spin(fwd, speedPCT, pct);
      fRDrive_mtr.spin(fwd, -speedPCT, pct);
      mLDrive_mtr.spin(fwd, speedPCT, pct);
      mRDrive_mtr.spin(fwd, -speedPCT, pct);
      bLDrive_mtr.spin(fwd, speedPCT, pct);
      bRDrive_mtr.spin(fwd, -speedPCT, pct);
    }
    else
    {
      currDeg = imu_sensor.heading(deg);
      degTravelled += getIMUDegDiff(currDeg, prevDeg);
      prevDeg = currDeg;
      if (degTravelled > 360.0)
      {
        isFacing = true;
        turnToDegree(defaultDeg, 100);
        break;
      }
      else if (isDefaultRight)
      {
        fLDrive_mtr.spin(fwd, maxSpeedPCT, pct);
        fRDrive_mtr.spin(fwd, -maxSpeedPCT, pct);
        mLDrive_mtr.spin(fwd, maxSpeedPCT, pct);
        mRDrive_mtr.spin(fwd, -maxSpeedPCT, pct);
        bLDrive_mtr.spin(fwd, maxSpeedPCT, pct);
        bRDrive_mtr.spin(fwd, -maxSpeedPCT, pct);
      }
      else
      {
        fLDrive_mtr.spin(fwd, -maxSpeedPCT, pct);
        fRDrive_mtr.spin(fwd, maxSpeedPCT, pct);
        mLDrive_mtr.spin(fwd, -maxSpeedPCT, pct);
        mRDrive_mtr.spin(fwd, maxSpeedPCT, pct);
        bLDrive_mtr.spin(fwd, -maxSpeedPCT, pct);
        bRDrive_mtr.spin(fwd, maxSpeedPCT, pct);
      }
    }

    wait(5, msec);

  }

	stopDrive();

}

void customSpinDriveBySide(double leftPower, double rightPower)
{
  fLDrive_mtr.spin(fwd, leftPower, pct);
  fRDrive_mtr.spin(fwd, rightPower, pct);
  mLDrive_mtr.spin(fwd, leftPower, pct);
  mRDrive_mtr.spin(fwd, rightPower, pct);
  bLDrive_mtr.spin(fwd, leftPower, pct);
  bRDrive_mtr.spin(fwd, rightPower, pct);
}

void driveOntoPlatform()
{

  double accelRangeIn = 2;
  double maxSpeedPCT = 85;
	double origin = fLDrive_mtr.position(deg);
	double absPosition = 0;
  accelRangeIn *= driveGearRatio;
	double accelDistInDeg = convertWheelInAndDeg(false, accelRangeIn, wheelDiameterIn);

	// accelerate period
	double error = accelDistInDeg;
	double absTargetPos = accelDistInDeg;
	double speedPCT;

	while (error > 0.0) {
  
		speedPCT = ((accelDistInDeg - error) / accelDistInDeg) * maxSpeedPCT;
		if (speedPCT > maxSpeedPCT)
			speedPCT = maxSpeedPCT;
    else if (speedPCT < 65.0)
      speedPCT = 65.0;
    fLDrive_mtr.spin(fwd, speedPCT, pct);
    mLDrive_mtr.spin(fwd, speedPCT, pct);
    bLDrive_mtr.spin(fwd, speedPCT, pct);
    fRDrive_mtr.spin(fwd, speedPCT, pct);
    mRDrive_mtr.spin(fwd, speedPCT, pct);
    bRDrive_mtr.spin(fwd, speedPCT, pct);
  
		wait(5, msec);
  
    absPosition = doubleMathAbs(fLDrive_mtr.position(deg) - origin);
		error = absTargetPos - absPosition;
    
	}

  // period of waiting for inertial pitch change

  while (imu_sensor.pitch(deg) < 20.0)
  {

    speedPCT = maxSpeedPCT;
    fLDrive_mtr.spin(fwd, speedPCT, pct);
    mLDrive_mtr.spin(fwd, speedPCT, pct);
    bLDrive_mtr.spin(fwd, speedPCT, pct);
    fRDrive_mtr.spin(fwd, speedPCT, pct);
    mRDrive_mtr.spin(fwd, speedPCT, pct);
    bRDrive_mtr.spin(fwd, speedPCT, pct);

    wait(5, msec);

    absPosition = doubleMathAbs(fLDrive_mtr.position(deg) - origin);

  }

	// period of P motion (rest of drive)
  double distIn = 27;
  distIn *= driveGearRatio;
  double distInDeg = convertWheelInAndDeg(false, distIn, wheelDiameterIn);
	absTargetPos = distInDeg + absPosition;
  error = absTargetPos - absPosition;

	while (error > 0.0) {

		speedPCT = (error / distInDeg) * maxSpeedPCT;
		if (speedPCT > maxSpeedPCT)
			speedPCT = maxSpeedPCT;
    else if (speedPCT < 35.0)
      speedPCT = 35.0;

    fLDrive_mtr.spin(fwd, speedPCT, pct);
    mLDrive_mtr.spin(fwd, speedPCT, pct);
    bLDrive_mtr.spin(fwd, speedPCT, pct);
    fRDrive_mtr.spin(fwd, speedPCT, pct);
    mRDrive_mtr.spin(fwd, speedPCT, pct);
    bRDrive_mtr.spin(fwd, speedPCT, pct);
  
		wait(5, msec);
  
    absPosition = doubleMathAbs(fLDrive_mtr.position(deg) - origin);
		error = absTargetPos - absPosition;
  
	}

	stopDrive();

}

double autonTimeElapsedMSec = 0.0;
void autonTimeThread()
{
  while (!isInAuto)
  {
    wait(20, msec);
  }
  while (autonTimeElapsedMSec < (15.0 * 1000.0))
  {
    wait(20, msec);
    autonTimeElapsedMSec += 20.0;
  }
}

void autonBackThread()
{
  while (true)
  {
    if (!isSkillsAuto && autonTimeElapsedMSec > (14.5 * 1000.0))
    {
      setBackTilt(true);
      break;
    }
    wait(20, msec);
  }
}

void setDriveStopping(brakeType type)
{
  fLDrive_mtr.setStopping(type);
	fRDrive_mtr.setStopping(type);
  mLDrive_mtr.setStopping(type);
  mRDrive_mtr.setStopping(type);
	bLDrive_mtr.setStopping(type);
	bRDrive_mtr.setStopping(type);
}

void toggleRampMode()
{
  if (!isOnRampMode)
  {
    isOnRampMode = true;
    maxDriveSpeedVolt = 10;
    Controller1.rumble("-");
  }
  else
  {
    isOnRampMode = false;
    maxDriveSpeedVolt = 12;
    Controller1.rumble("--");
  }
}

void autonSettings(double x, double y, double head)
{
  // xRobotPos = x;
  // yRobotPos = y;
  imu_sensor.setHeading(head, deg);
  // prevHeadingDeg = imu_sensor.heading(deg);
  // prevHorizontalTrackDeg = horizontalTrackWheel.position(deg);
  // prevVerticalTrackDeg = -1.0 * verticalTrackWheel.position(deg);
  settingStartingRobot = false;
  // wait(50, msec);
}

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // driver controls
  Controller1.ButtonL1.pressed(toggleIntakeIn);
  Controller1.ButtonL2.pressed(toggleGoalCover);
  Controller1.ButtonR1.pressed(toggleClawLock);
  Controller1.ButtonR2.pressed(toggleBackTilt);
  Controller1.ButtonY.pressed(toggleRampMode);
  Controller1.ButtonA.pressed(toggleTrans);
  Controller1.ButtonUp.pressed(toggleLiftLock);

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
  lift_sensor.resetPosition();
  claw_sensor.setLight(ledState::on);
  claw_sensor.setLightPower(100, pct);
  //horizontalTrackWheel.setPosition(0, deg);
  //verticalTrackWheel.setPosition(0, deg);
  wait(1000, msec);
  calibrateSensors();

  wait(500, msec);

  setClawLock(isClawLocked);
  setBackTilt(isBackUntilted);
  setGoalCover(isCoverDeployed);
  setTrans(is6MtrDrive);
  setLiftLock(isLiftLocked);

}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

int numAuton = 11; // keep on 11 so that startAutonLiftIntakeType stays correct
int teamColor = 0; // 0 is blue, 1 is red

void autonomous(void) {

  isInAuto = true;

  int oppositeColor = (teamColor == 0) ? 1 : 0;
  
  setDriveStopping(brake);
  lift_mtr.setStopping(startAutonLiftIntakeType);
  intake_mtr.setStopping(startAutonLiftIntakeType);

  lift_mtr.spin(fwd, liftSpeedVolt, volt);
  lift_sensor.resetPosition();
  
  lift_mtr.setVelocity(liftSpeedVolt / 12.0 * 100.0, pct);
  intake_mtr.setVelocity(liftSpeedVolt / 12.0 * 100.0, pct);

  if (!calibrated)
    calibrateSensors();

  initiateBot();

  if (numAuton == 0) // test auton
  {
    autonSettings(0, 0, 0);

    // empty

  }
  else if (numAuton == 1) // win pt auton (from left)
  {
    autonSettings(0, 0, 90);

    toggleBackTilt();
    toggleGoalCover();
    wait(250, msec);
    driveForDistance(-5, 65, false);
    toggleGoalCover();
    wait(50, msec);
    turnToDegree(0, 100);
    wait(50, msec);
    driveForDistance(15, 85, false);
    turnToDegree(270, 100);
    wait(50, msec);
    turnToDegree(270, 100);
    wait(50, msec);
    driveForDistance(-55, 125, false);
    turnToDegree(268, 100);
    wait(50, msec);
    driveForDistanceSSPT(-58, 100);
    toggleBackTilt();
    toggleLiftLock();
    liftSpinToPosition(30);
    autonToggleIntakeIn();
    driveForDistance(22, 100, false);
    turnToDegree(312.5, 100);
    toggleBackTilt();
    wait(50, msec);
    turnIntakeOff();
    liftSpinToPosition(-25);
    toggleClawLock();
    driveToMiddleMogos(50, 65, 50);
    toggleClawLock();
    liftSpinToPosition(20);
    turnToDegree(312.5, 100);
    wait(50, msec);
    driveForDistance(-70, 250, false);

  }
  else if (numAuton == 2) // right middle tug
  {
    autonSettings(0, 0, 0);

    toggleClawLock();
    toggleGoalCover();
    driveToMiddleMogos(45, 100, 100);
    toggleClawLock();
    customSpinDriveBySide(-100, -100);
    lift_mtr.spin(fwd, -100, pct);
    intake_mtr.spin(fwd, -100, pct);

  }
  else if (numAuton == 3) // left middle then middle middle
  {
    autonSettings(0, 0, 7.5);

    toggleClawLock();
    toggleGoalCover();
    driveToMiddleMogos(48, 100, 100);
    toggleClawLock();
    driveBackToSide(20, 15, 100);
    toggleTrans();
    turnToDegree(0, 100);
    wait(50, msec);
    driveForDistance(20, 100, false);
    toggleGoalCover();
    turnToDegree(195, 100);
    toggleClawLock();
    wait(100, msec);
    turnToDegree(270, 100);
    wait(50, msec);
    driveForDistance(-62, 100, false);
    driveForDistance(7, 65, false);
    turnToDegree(180, 125);
    turnToBackMogo(2, 15, true);
    turnToDegree(getDegWithin360(imu_sensor.heading() + 180.0), 100);
    wait(50, msec);
    driveForDistance(27, 65, true);
    toggleClawLock();
    driveForDistance(-50, 85, false);

  }
  else if (numAuton == 4) // middle middle tug
  {
    autonSettings(0, 0, 330);

    toggleClawLock();
    toggleGoalCover();
    driveToMiddleMogos(62, 100, 100);
    toggleClawLock();
    customSpinDriveBySide(-100, -100);
    lift_mtr.spin(fwd, -100, pct);
    intake_mtr.spin(fwd, -100, pct);

  }
  else if (numAuton == 5) // left drop
  {
    autonSettings(0, 0, 90);
    
    toggleLiftLock();
    toggleBackTilt();
    lift_mtr.spinFor(fwd, -20 * 25.0, deg, false);
    intake_mtr.spinFor(fwd, -20 * 25.0, deg);
    driveForDistance(3, 35, false);
    toggleClawLock();
    wait(500, msec);
    liftSpinToPosition(70);
    driveForDistance(-3, 35, false);
    wait(50, msec);
    turnToDegree(0, 100);
    liftSpinToPosition(-25);

  }
  else if (numAuton == 6) // skills auton
  {
    isSkillsAuto = true;
    autonSettings(0, 0, 180);

    toggleClawLock();
    toggleBackTilt();
    driveForDistance(-10, 35, false);
    toggleBackTilt();
    turnToDegree(270, 100);
    wait(50, msec);
    // isDetectingFrontMogo = true;
    // isLiftingFrontMogo = true;
    driveToMiddleMogos(50, 85, 65);
    setClawLock(true);
    toggleLiftLock();
    wait(50, msec);
    lift_mtr.spin(fwd, -liftSpeedVolt, volt);
    intake_mtr.spin(fwd, -liftSpeedVolt, volt);
    //autonToggleIntakeIn();
    driveToPoint(36, 69, 65, true, 5, 250, 16, 0);
    lift_mtr.spin(fwd, -liftSpeedVolt, volt);
    intake_mtr.spin(fwd, -liftSpeedVolt, volt);
    // isDetectingFrontMogo = false;
    // isLiftingFrontMogo = false;
    turnToPoint(12, 70, 100, true);
    wait(50, msec);
    driveForDistanceSSPT(12, 50);
    liftSpinToPosition(-25);
    wait(600, msec);
    lift_mtr.stop();
    intake_mtr.stop();
    lift_mtr.setStopping(coast);
    intake_mtr.setStopping(coast);
    wait(600, msec);
    setClawLock(false); // 1
    wait(100, msec);
    lift_mtr.setStopping(hold);
    intake_mtr.setStopping(hold);
    liftSpinToPosition(70);
    turnToDegree(247, 100);
    wait(50, msec);
    turnIntakeOff();
    driveForDistance(-19, 85, false);
    liftSpinToPosition(-25);
    driveForDistance(3, 100, false);
    toggleBackTilt();
    driveForDistance(3, 100, true);
    turnToDegree(getDegWithin360(imu_sensor.heading(deg) + 180.5), 100);
    // isDetectingFrontMogo = true;
    // isLiftingFrontMogo = true;
    driveForDistance(8, 65, false);
    setClawLock(true);
    liftSpinToPosition(70);
    wait(300, msec);
    // isDetectingFrontMogo = false;
    // isLiftingFrontMogo = false;
    turnToDegree(270, 100);wait(50, msec);turnToDegree(263, 100);
    wait(50, msec);
    driveForDistanceSSPT(24, 85);
    liftSpinToPosition(-25);
    wait(300, msec);
    lift_mtr.stop();
    intake_mtr.stop();
    lift_mtr.setStopping(coast);
    intake_mtr.setStopping(coast);
    wait(300, msec);
    setClawLock(false); // 2
    wait(100, msec);
    lift_mtr.setStopping(hold);
    intake_mtr.setStopping(hold);
    liftSpinToPosition(70);
    wait(150, msec);
    driveForDistance(-5, 65, false);
    turnToDegree(180, 120);
    liftSpinToPosition(-25);
    toggleBackTilt();
    wait(50, msec);
    driveForDistance(-35, 135, true);
    turnToDegree(90, 100);
    // isDetectingFrontMogo = true;
    // isLiftingFrontMogo = false;
    driveToPoint(74, 108, 50, true, 5, 150, 0.01, 0);
    setClawLock(true);
    driveToPoint(110, 108, 65, true, 5, 150, 16, 0);
    liftSpinToPosition(5);
    // isDetectingFrontMogo = false;
    // isLiftingFrontMogo = false;
    turnToDegree(180, 100);
    turnToBackMogo(oppositeColor, 15, true);
    toggleBackTilt();
    driveForDistance(-20, 35, true);
    liftSpinToPosition(50);
    toggleBackTilt();
    //autonToggleIntakeIn();
    customSpinDriveBySide(65, 65);
    wait(150, msec);
    turnToDegree(210, 100);
    driveToPoint(36, 69, 85, true, 5, 250, 24, 0);
    turnToDegree(270, 100);wait(50, msec);turnToPoint(12, 72, 100, true);
    wait(50, msec);
    driveForDistanceSSPT(16, 65);
    liftSpinToPosition(-25);
    wait(300, msec);
    //lift_mtr.stop();
    lift_mtr.setStopping(coast);
    intake_mtr.setStopping(coast);
    wait(300, msec);
    setClawLock(false); // 3
    wait(100, msec);
    lift_mtr.setStopping(hold);
    intake_mtr.setStopping(hold);
    liftSpinToPosition(70);
    turnToDegree(247, 100);
    wait(50, msec);
    turnIntakeOff();
    driveForDistance(-19, 85, false);
    liftSpinToPosition(-25);
    driveForDistance(3, 100, false);
    toggleBackTilt();
    driveForDistance(3, 100, true);
    turnToDegree(getDegWithin360(imu_sensor.heading(deg) + 180.5), 100);
    // isDetectingFrontMogo = true;
    // isLiftingFrontMogo = true;
    driveForDistance(8, 65, false);
    setClawLock(true);
    liftSpinToPosition(70);
    wait(300, msec);
    // isDetectingFrontMogo = false;
    // isLiftingFrontMogo = false;
    turnToDegree(270, 100);wait(50, msec);turnToDegree(255, 100);
    wait(50, msec);
    driveForDistanceSSPT(24, 65);
    liftSpinToPosition(-25);
    wait(300, msec);
    lift_mtr.stop();
    intake_mtr.stop();
    lift_mtr.setStopping(coast);
    intake_mtr.setStopping(coast);
    wait(300, msec);
    setClawLock(false); // 4
    wait(100, msec);
    lift_mtr.setStopping(hold);
    intake_mtr.setStopping(hold);
    liftSpinToPosition(70);
    wait(150, msec);
    driveForDistance(-9, 65, false);
    turnToDegree(0, 150);
    liftSpinToPosition(-25);
    toggleBackTilt();
    wait(50, msec);
    driveForDistance(-38, 150, false);
    turnToBackMogo(teamColor, 15, false);
    toggleBackTilt();
    driveForDistanceSSPT(-30, 35);//driveForDistance(-26, 35, true); // 5
    toggleBackTilt();
    customSpinDriveBySide(65, 65);
    wait(150, msec);
    turnToDegree(30, 100);
    wait(50, msec);
    // isDetectingFrontMogo = true;
    // isLiftingFrontMogo = true;
    driveToPoint(74, 74, 65, true, 5, 100, 0.01, 0); // 6
    lift_mtr.stop();
    intake_mtr.stop();
    lift_mtr.setStopping(coast);
    intake_mtr.setStopping(coast);
    setClawLock(true);
    driveToPoint(108, 115, 85, true, 7.5, 250, 16, 0);
    // isDetectingFrontMogo = false;
    // isLiftingFrontMogo = false;
    lift_mtr.setStopping(hold);
    intake_mtr.setStopping(hold);
    liftSpinToPosition(70);
    turnToDegree(90, 100);
    wait(50, msec);
    turnToDegree(90, 100);
    wait(50, msec);
    driveForDistance(52, 85, true);
    driveForDistance(-5, 65, false);
    turnToDegree(180, 100);
    liftSpinToPosition(-25);
    wait(50, msec);
    turnToDegree(180, 100);
    wait(500, msec);
    liftSpinToPosition(-25);
    driveOntoPlatform(); // park
    setDriveStopping(hold);
    lift_mtr.setStopping(coast);
    intake_mtr.setStopping(coast);
    toggleBackTilt();
  
  }
  else if (numAuton == 7) // left drop then left middle
  {
    autonSettings(0, 0, 90);
    
    toggleLiftLock();
    lift_mtr.spinFor(fwd, -20 * 25.0, deg, false);
    intake_mtr.spinFor(fwd, -20 * 25.0, deg);
    driveForDistance(3, 35, false);
    toggleClawLock();
    liftSpinToPosition(35);
    wait(500, msec);
    turnToDegree(11, 100);
    wait(50, msec);
    liftSpinToPosition(-25);
    driveToMiddleMogos(50, 100, 100);
    toggleClawLock();
    customSpinDriveBySide(-100, -100);

  }
  else if (numAuton == 8) // middle middle then alliance mogo
  {
    autonSettings(0, 0, 330);

    toggleClawLock();
    toggleGoalCover();
    driveToMiddleMogos(62, 100, 100);
    toggleClawLock();
    toggleLiftLock();
    toggleTrans();
    turnToDegree(330, 100);
    wait(50, msec);
    driveForDistance(-66, 150, false);
    if (autonTimeElapsedMSec < 4000.0)
    {
      turnToDegree(0, 100);
      wait(50, msec);
      driveForDistance(11, 85, false);
      toggleBackTilt();
      turnToDegree(270, 100);
      wait(50, msec);
      turnToBackMogo(teamColor, 15, true);
      wait(50, msec);
      liftSpinToPosition(70);
      driveForDistanceSSPT(-24, 50);
      toggleBackTilt();
      autonToggleIntakeIn();
      toggleLiftLock();
      turnToDegree(180, 100);
      wait(50, msec);
      turnToDegree(180, 100);
      wait(50, msec);
      driveForDistance(36, 50, true);
      driveForDistance(-6, 65, false);
      toggleLiftLock();
      turnToDegree(90, 100);
      wait(50, msec);
      driveForDistance(-5, 100, false);
    }
    else
    {
      turnToDegree(0, 100);
      wait(50, msec);
      driveForDistance(11, 85, false);
      toggleBackTilt();
      turnToDegree(270, 100);
      wait(50, msec);
      turnToBackMogo(teamColor, 15, true);
      wait(50, msec);
      liftSpinToPosition(70);
      driveForDistanceSSPT(-24, 50);
      toggleBackTilt();
      autonToggleIntakeIn();
      driveForDistance(25, 65, false);
    }

  }
  else if (numAuton == 9) // right middle then alliance mogo
  {
    autonSettings(0, 0, 0);

    toggleClawLock();
    toggleGoalCover();
    driveToMiddleMogos(45, 100, 100);
    toggleClawLock();
    toggleLiftLock();
    driveBackToSide(20, 0, 100);
    toggleTrans();
    if (autonTimeElapsedMSec < 3250.0)
    {
      turnToDegree(0, 100);
      wait(50, msec);
      driveForDistance(-24, 150, true);
      wait(100, msec);
      driveForDistance(25, 85, false);
      toggleBackTilt();
      turnToDegree(270, 100);
      wait(50, msec);
      turnToDegree(270, 100);
      wait(50, msec);
      liftSpinToPosition(70);
      driveForDistanceSSPT(-32, 50);
      toggleBackTilt();
      autonToggleIntakeIn();
      toggleLiftLock();
      driveForDistance(2.5, 50, false);
      turnToDegree(0, 100);
      wait(50, msec);
      turnToDegree(0, 100);
      wait(50, msec);
      driveForDistance(39, 50, true);
      toggleLiftLock();
      driveForDistance(-48, 200, false);
    }
    else
    {
      turnToDegree(0, 100);
      wait(50, msec);
      driveForDistance(-24, 150, true);
      wait(100, msec);
      driveForDistance(25, 85, false);
      toggleBackTilt();
      turnToDegree(270, 100);
      wait(50, msec);
      turnToDegree(270, 100);
      wait(50, msec);
      liftSpinToPosition(70);
      driveForDistanceSSPT(-32, 50);
      toggleBackTilt();
      autonToggleIntakeIn();
      driveForDistance(25, 65, false);
    }

  }
  else if (numAuton == 10) // right middle then middle middle
  {
    autonSettings(0, 0, 0);

    toggleClawLock();
    toggleGoalCover();
    driveToMiddleMogos(45, 100, 100);
    toggleClawLock();
    //driveBackToSide(20, 0, 100);
    toggleTrans();
    turnToDegree(0, 100);
    wait(50, msec);
    driveForDistance(-44, 125, false);
    toggleGoalCover();
    turnToDegree(135, 100);
    toggleClawLock();
    wait(50, msec);
    driveForDistance(-20, 100, false);
    liftSpinToPosition(-25);
    turnToBackMogo(2, 15, true);
    turnToDegree(getDegWithin360(imu_sensor.heading() + 180.0), 100);
    wait(50, msec);
    driveForDistance(32, 65, true);
    toggleClawLock();
    driveForDistance(-70, 85, false);

  }
  else if (numAuton == 11) // left middle then alliance mogo
  {
    autonSettings(0, 0, 7.5);

    toggleClawLock();
    toggleGoalCover();
    driveToMiddleMogos(48, 100, 100);
    toggleClawLock();
    toggleLiftLock();
    //driveBackToSide(20, 15, 100);
    toggleTrans();
    turnToDegree(15, 100);
    wait(50, msec);
    driveForDistance(-53, 125, true);
    if (autonTimeElapsedMSec < 3750.0)
    {
      turnToDegree(0, 100);
      wait(50, msec);
      driveForDistance(-24, 150, true);
      wait(100, msec);
      //driveForDistance(1, 65, false);
      toggleBackTilt();
      turnToDegree(270, 35);
      wait(50, msec);
      turnToDegree(270, 100);
      wait(50, msec);
      liftSpinToPosition(70);
      driveForDistanceSSPT(-28, 50);
      toggleBackTilt();
      turnToDegree(270, 100);
      wait(50, msec);
      autonToggleIntakeIn();
      toggleLiftLock();
      driveForDistance(50, 35, true);
      toggleLiftLock();
      driveForDistance(-15, 150, false);
    }
    else
    {
      turnToDegree(0, 100);
      wait(50, msec);
      driveForDistance(-24, 150, true);
      wait(100, msec);
      //driveForDistance(1, 65, false);
      toggleBackTilt();
      turnToDegree(270, 35);
      wait(50, msec);
      turnToDegree(270, 100);
      wait(50, msec);
      liftSpinToPosition(70);
      driveForDistanceSSPT(-28, 50);
      toggleBackTilt();
      turnToDegree(270, 100);
      wait(50, msec);
      autonToggleIntakeIn();
      driveForDistance(25, 65, false);
    }

  }

}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {

  settingStartingRobot = false;

  intakingIn = false;
  intakingOut = false;
  isLiftLocked = false;

  setDriveStopping(coast);
  lift_mtr.setStopping(coast);
  intake_mtr.setStopping(coast);

  initiateBot();

  while (true) {

    bool movingLift = false;
    double liftPower = 0;
    if (Controller1.Axis2.position(pct) > 20.0)
    {
      usingAxis2 = true;
      if (is6MtrDrive)
      {
        movingLift = true;
        lift_mtr.setStopping(hold);
        intake_mtr.setStopping(hold);
        // lift_mtr.spin(fwd, -liftSpeedVolt, volt);
        // intake_mtr.spin(fwd, -liftSpeedVolt, volt);
        liftPower = -liftSpeedVolt;

        if (lift_sensor.position(deg) > (110.0 - maxIntakeThresholdDeg))
          intakingIn = false;
      }
    }
    else if (Controller1.Axis2.position(pct) < -20.0)
    {
      usingAxis2 = true;
      if (is6MtrDrive)
      {
        movingLift = true;
        lift_mtr.setStopping(hold);
        intake_mtr.setStopping(hold);
        if ((intakingIn || intakingOut) && (lift_sensor.position(deg) < liftDeadzoneDeg))
          turnIntakeOff();
        // lift_mtr.spin(fwd, liftSpeedVolt, volt);
        // intake_mtr.spin(fwd, liftSpeedVolt, volt);
        liftPower = liftSpeedVolt;
      }
    }
    else
    {
      usingAxis2 = false;
      if (is6MtrDrive)
      {
        if (!intakingIn && !intakingOut)
        {
          movingLift = true;
          // lift_mtr.stop();
          // intake_mtr.stop();
        }
        else if (lift_sensor.position(deg) > liftDeadzoneDeg && lift_sensor.position(deg) < (110.0 - maxIntakeThresholdDeg))
        {
          if (!isUnjamming)
          {
            movingLift = true;
            //lift_mtr.stop();
            if (intakingIn)
            {
              intake_mtr.spin(fwd, intakeSpeedPCT, pct);
            }
            else
            {
              intake_mtr.spin(fwd, -intakeSpeedPCT, pct);
            }
          }
        }
      }
    }

    currAxis2 = liftPower;
    if (currAxis2 - prevAxis2 > 0.7)
    {
      liftPower = prevAxis2 + 0.7;
      prevAxis2 = liftPower;
    }
    else if (currAxis2 - prevAxis2 < -0.7)
    {
      liftPower = prevAxis2 - 0.7;
      prevAxis2 = liftPower;
    }
    else
    {
      prevAxis2 = liftPower;
    }
    if (movingLift)
    {
      lift_mtr.spin(fwd, liftPower, volt);
      if (usingAxis2)
      {
        intake_mtr.spin(fwd, liftPower, volt);
      }
      else if (!intakingIn && !intakingOut)
      {
        intake_mtr.spin(fwd, liftPower, volt);
      }
      else
      {
        if (intakingIn)
        {
          intake_mtr.spin(fwd, intakeSpeedPCT, pct);
        }
        else
        {
          intake_mtr.spin(fwd, -intakeSpeedPCT, pct);
        }
      }
    }

    double straightPower = Controller1.Axis3.position(pct);
    double middleAxisThresh = 10; // no side-to-side power deadzone along Controller1.Axis4
    double sidePower = Controller1.Axis4.position(pct);
    if (sidePower >= middleAxisThresh)
    {
      sidePower = ((sidePower - middleAxisThresh) / (100.0 - middleAxisThresh)) * 100.0;
    }
    else if (sidePower <= -1.0 * middleAxisThresh)
    {
      sidePower = ((sidePower + middleAxisThresh) / (100.0 - middleAxisThresh)) * 100.0;
    }
    else
    {
      sidePower = 0.0;
    }
    sidePower = (sidePower / 100.0) * 12.0;
    straightPower = (straightPower / 100.0) * maxDriveSpeedVolt; // max drive power

    // slew rate control
    currAxis3 = straightPower;
    currAxis4 = sidePower;
    if (!isOnRampMode)
    {
      if (currAxis3 - prevAxis3 > 0.96)
      {
        straightPower = prevAxis3 + 0.96;
        prevAxis3 = straightPower;
      }
      else if (currAxis3 - prevAxis3 < -0.96)
      {
        straightPower = prevAxis3 - 0.96;
        prevAxis3 = straightPower;
      }
      else
      {
        prevAxis3 = straightPower;
      }
      if (currAxis4 - prevAxis4 > 2.4)
      {
        sidePower = prevAxis4 + 2.4;
        prevAxis4 = sidePower;
      }
      else if (currAxis4 - prevAxis4 < -2.4)
      {
        sidePower = prevAxis4 - 2.4;
        prevAxis4 = sidePower;
      }
      else
      {
        prevAxis4 = sidePower;
      }
      setDriveStopping(coast);
      fLDrive_mtr.spin(fwd, straightPower + sidePower, volt);
      fRDrive_mtr.spin(fwd, straightPower - sidePower, volt);
      mLDrive_mtr.spin(fwd, straightPower + sidePower, volt);
      mRDrive_mtr.spin(fwd, straightPower - sidePower, volt);
      bLDrive_mtr.spin(fwd, straightPower + sidePower, volt);
      bRDrive_mtr.spin(fwd, straightPower - sidePower, volt);
      if (!is6MtrDrive)
      {
        lift_mtr.setStopping(coast);
        intake_mtr.setStopping(coast);
        lift_mtr.spin(fwd, straightPower - sidePower, volt);
        intake_mtr.spin(fwd, straightPower + sidePower, volt);
      }
    }
    else
    {
      prevAxis3 = straightPower;
      prevAxis4 = sidePower;
      setDriveStopping(hold);
      if (doubleMathAbs(straightPower + sidePower) < 1.2 && doubleMathAbs(straightPower - sidePower) < 1.2)
      {
        stopDrive();
        if (!is6MtrDrive)
        {
          lift_mtr.setStopping(hold);
          intake_mtr.setStopping(hold);
          lift_mtr.stop();
          intake_mtr.stop();
        }
      }
      else
      {
        fLDrive_mtr.spin(fwd, straightPower + sidePower, volt);
        fRDrive_mtr.spin(fwd, straightPower - sidePower, volt);
        mLDrive_mtr.spin(fwd, straightPower + sidePower, volt);
        mRDrive_mtr.spin(fwd, straightPower - sidePower, volt);
        bLDrive_mtr.spin(fwd, straightPower + sidePower, volt);
        bRDrive_mtr.spin(fwd, straightPower - sidePower, volt);
        if (!is6MtrDrive)
        {
          lift_mtr.setStopping(hold);
          intake_mtr.setStopping(hold);
          lift_mtr.spin(fwd, straightPower - sidePower, volt);
          intake_mtr.spin(fwd, straightPower + sidePower, volt);
        }
      }
    }

    wait(5, msec);

  }
}

void changeNumAutonAndTeam()
{
  if (Brain.Screen.xPosition() < 90 && Brain.Screen.yPosition() > 60 && Brain.Screen.yPosition() < 180)
  {
    numAuton--;
  }
  else if (Brain.Screen.xPosition() > 120 && Brain.Screen.xPosition() < 240 && Brain.Screen.yPosition() > 60 && Brain.Screen.yPosition() < 180)
  {
    numAuton++;
  }
  else if (Brain.Screen.xPosition() > 270 && Brain.Screen.xPosition() < 390 && Brain.Screen.yPosition() > 60 && Brain.Screen.yPosition() < 180)
  {
    if (teamColor == 0)
    {
      teamColor = 1;
    }
    else
    {
      teamColor = 0;
    }
  }
  if (numAuton == 1) // manually update this when changing auton starting transmission positions
  {
    setTrans(true);
    startAutonLiftIntakeType = hold;
  }
  else if (numAuton == 2)
  {
    setTrans(false);
    startAutonLiftIntakeType = brake;
  }
  else if (numAuton == 3)
  {
    setTrans(false);
    startAutonLiftIntakeType = brake;
  }
  else if (numAuton == 4)
  {
    setTrans(false);
    startAutonLiftIntakeType = brake;
  }
  else if (numAuton == 5)
  {
    setTrans(true);
    startAutonLiftIntakeType = hold;
  }
  else if (numAuton == 6)
  {
    setTrans(true);
    startAutonLiftIntakeType = hold;
  }
  else if (numAuton == 7)
  {
    setTrans(true);
    startAutonLiftIntakeType = hold;
  }
  else if (numAuton == 8)
  {
    setTrans(false);
    startAutonLiftIntakeType = brake;
  }
  else if (numAuton == 9)
  {
    setTrans(false);
    startAutonLiftIntakeType = brake;
  }
  else if (numAuton == 10)
  {
    setTrans(false);
    startAutonLiftIntakeType = brake;
  }
  else if (numAuton == 11)
  {
    setTrans(false);
    startAutonLiftIntakeType = brake;
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // callbacks
  claw_sensor.objectDetected(onClawSensorDetected);

  // threads
  thread intakeSafety = thread(intakeJamThread);
  thread autonTimer = thread(autonTimeThread);
  thread autoBackUnlatch = thread(autonBackThread);

  // auton selector
  Brain.Screen.drawRectangle(0, 90, 60, 60);
  Brain.Screen.drawRectangle(60, 90, 90, 60);
  Brain.Screen.drawRectangle(150, 90, 60, 60);
  Brain.Screen.printAt(20, 122, "<<");
  Brain.Screen.printAt(170, 122, ">>");
  Brain.Screen.drawRectangle(300, 90, 60, 60);
  Brain.Screen.printAt(320, 122, "TC");

  Brain.Screen.pressed(changeNumAutonAndTeam);

  // Prevent main from exiting with an infinite loop.
  while (true) {
    Brain.Screen.printAt(0, 30, "X: %-10f; Y: %-10f", getX(), getY());
    Brain.Screen.printAt(0, 45, "Heading (imu): %f; Heading (gps): %f", imu_sensor.heading(deg), gps_sensor.heading(deg));
    Brain.Screen.printAt(0, 60, "lift_sensor position (deg): %f", lift_sensor.position(deg));
    Brain.Screen.printAt(0, 75, "usingAxis2: %d intakingIn: %d intakingOut: %d", usingAxis2, intakingIn, intakingOut);
    wait(50, msec);
    Brain.Screen.printAt(82, 122, "%3d", numAuton);
    Brain.Screen.printAt(90, 137, (teamColor == 0)?"BLUE":"RED ");

    Brain.Screen.printAt(0, 180, "backRange_sensor dist: %f", backRange_sensor.distance(inches));
    Brain.Screen.printAt(0, 195, "is6MtrDrive: %d", is6MtrDrive);
  }
}
// autonomous routines
/*
*    #        Auton
*    1     win pt auton (from left)
*    2     right middle tug
*    3     left middle then middle middle
*    4     middle middle tug
*    5     left drop
*    6     skills auton
*    7     left drop then left middle
*    8     middle middle then alliance mogo
*    9     right middle then alliance mogo
*    10    right middle then middle middle
*    11    left middle then alliance mogo
*/