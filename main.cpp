/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       maddie                                                    */
/*    Created:      5/23/2025, 5:38:26 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/
#pragma region VEXcode Generated Robot Configuration
// Make sure all required headers are included.
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>


#include "vex.h"

using namespace vex;

// Brain should be defined by default
brain Brain;


// START V5 MACROS
#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)
// END V5 MACROS


// Robot configuration code.
motor RightMotor = motor(PORT2, ratio18_1, true);

motor LeftMotor = motor(PORT1, ratio18_1, false);

motor MoveClaw = motor(PORT18, ratio18_1, false);

motor OpenClaw = motor(PORT3, ratio18_1, false);

inertial IMU = inertial(PORT10);

distance frontDistance = distance(PORT12);
distance rightDistance = distance(PORT20);
distance leftDistance = distance(PORT11);
// AI Vision Color Descriptions
// AI Vision Code Descriptions
vex::aivision Camera(PORT4);



// generating and setting random seed
void initializeRandomSeed(){
  int systemTime = Brain.Timer.systemHighResolution();
  double batteryCurrent = Brain.Battery.current();
  double batteryVoltage = Brain.Battery.voltage(voltageUnits::mV);

  // Combine these values into a single integer
  int seed = int(batteryVoltage + batteryCurrent * 100) + systemTime;

  // Set the seed
  srand(seed);
}



void vexcodeInit() {

  //Initializing random seed.
  initializeRandomSeed();
}


// Helper to make playing sounds from the V5 in VEXcode easier and
// keeps the code cleaner by making it clear what is happening.
void playVexcodeSound(const char *soundName) {
  printf("VEXPlaySound:%s\n", soundName);
  wait(5, msec);
}

#pragma endregion VEXcode Generated Robot Configuration

// ---PORT SETUP---

// motor setup
motor RightMotor = motor(PORT2, ratio18_1, true);
motor LeftMotor = motor(PORT1, ratio18_1, false);
motor MoveClaw = motor(PORT18, ratio18_1, false);
motor OpenClaw = motor(PORT3, ratio18_1, false);
inertial IMU = inertial(PORT10);

//distance sensor setup
distance frontDistance = distance(PORT12);
distance rightDistance = distance(PORT20);
distance leftDistance = distance(PORT11);

//AI Vision Setup
vex::aivision Camera(PORT4);
aivision::colordesc tubeColor = {1, 88, 28, 108, 10.0, 0.20}; //tube target color

// --- GLOBAL VARIABLE DEFINITIONS ---

// --- Global PID Variables and Constants ---
// --- PID Constants for turnToHeading ---
const double HEADING_TURN_KP = 0.8;  // Proportional gain for turning (tune this)
const double HEADING_TURN_KI = 0.001; // Integral gain for turning (tune this, usually small)
const double HEADING_TURN_KD = 0.05; // Derivative gain for turning (tune this)
const double HEADING_ANGLE_DEADBAND = 2.0; // Degrees within target to consider aligned
const int HEADING_MIN_TURN_SPEED = 5; // Minimum speed to overcome friction (percent)
const int HEADING_MAX_TURN_SPEED = 50; // Maximum turn speed (percent)
double heading_integral = 0.0;
double heading_prevError = 0.0;

// --- PID Constants for general distance driving (moveForwardSmallDistance and main navigation) ---
const double DIST_DRIVE_KP = 0.5; // Proportional gain for distance (tune this)
const double DIST_DRIVE_KI = 0.0001; // Integral gain for distance (tune this)
const double DIST_DRIVE_KD = 0.05; // Derivative gain for distance (tune this)
const double DIST_DRIVE_DEADBAND = 1.0; // Inches within target distance to consider reached
const int DIST_DRIVE_MIN_SPEED = 10; // Minimum speed for driving
// const int DIST_DRIVE_MAX_SPEED = 60; // Maximum speed for driving
const int DIST_DRIVE_MAX_SPEED = 30; // Maximum speed for driving
double dist_drive_integral = 0.0;
double dist_drive_prevError = 0.0;

// --- PID Constants for obstacleMoveBackward (Reverse Wall Following) ---
// Note: This function is still present but not used in the new obstacle avoidance strategy.
// It could be used for other scenarios or removed if no longer needed.
const double AVOID_KP = 2.0; // Proportional gain for avoidance (tune this)
const double AVOID_KI = 0.01; // Integral gain for avoidance
const double AVOID_KD = 0.1; // Derivative gain for avoidance
const double AVOID_DIST_DEADBAND = 1.0; // Inches within target distance to consider aligned
const int AVOID_MIN_SPEED = 10; // Minimum reverse speed
const int AVOID_MAX_SPEED = 40; // Max reverse speed
double avoid_integral = 0.0;
double avoid_prevError = 0.0;

// --- PID Constants for Color Tracking (compute_tracking_speed_color) ---
const double COLOR_TURN_KP = 0.15; // Proportional gain for turning towards color
const double COLOR_TURN_KI = 0.0005; // Integral gain for turning
const double COLOR_TURN_KD = 0.01; // Derivative gain for turning
const double COLOR_FWD_KP = 0.1; // Proportional gain for moving towards color
const double COLOR_FWD_KI = 0.0005; // Integral gain for moving
const double COLOR_FWD_KD = 0.01; // Derivative gain for moving
const double COLOR_POS_DEADBAND = 10.0; // Pixels from center to consider aligned
const double COLOR_WIDTH_DEADBAND = 5.0; // Pixels from target width to consider aligned
const int COLOR_MIN_SPEED = 5; // Min speed for color tracking
const int COLOR_MAX_SPEED = 30; // Max speed for color tracking
double color_turn_integral = 0.0;
double color_turn_prevError = 0.0;
double color_fwd_integral = 0.0;
double color_fwd_prevError = 0.0;

// --- PID Constants for AprilTag Tracking (compute_tracking_turn_speed) ---
const double TAG_TURN_KP = 0.1; // Proportional gain for turning towards tag
const double TAG_TURN_KI = 0.0005; // Integral gain for turning
const double TAG_TURN_KD = 0.01; // Derivative gain for turning
const double TAG_FWD_KP = 0.1; // Proportional gain for moving towards tag
const double TAG_FWD_KI = 0.0005; // Integral gain for moving
const double TAG_FWD_KD = 0.01; // Derivative gain for moving
const double TAG_POS_DEADBAND = 10.0; // Pixels from center to consider aligned
const double TAG_HEIGHT_DEADBAND = 5.0; // Pixels from target height to consider aligned
const int TAG_MIN_SPEED = 5; // Min speed for tag tracking
const int TAG_MAX_SPEED = 30; // Max speed for tag tracking
double tag_turn_integral = 0.0;
double tag_turn_prevError = 0.0;
double tag_fwd_integral = 0.0;
double tag_fwd_prevError = 0.0;

// turnLeftOrRight: 0: no preference, 1: turn robot's rear to the left (pivot counter-clockwise), 2: turn robot's rear to the right (pivot clockwise)
int turnLeftOrRight = 0;

// Odometry variables
double INITIAL_X = -120;
double INITIAL_Y = 156;
double xx = INITIAL_X;          // Robot's X position (in inches)
double yy = INITIAL_Y;          // Robot's Y position (in inches)
double theta = 0.0;      // Robot's orientation in degrees (0 to 360)
double wheelDiameter = 4.0;  // Diameter of the wheels (in inches)
double wheelBase = 8.0;     // Distance between the left and right wheels (in inches)
int encoderCountsPerRevolution = 360;  // Number of encoder counts per wheel revolution
double distancePerCount = (M_PI * wheelDiameter) / encoderCountsPerRevolution;  // Distance covered per encoder tick

// put target behind the robot (no turn is needed)
double TARGET_X = INITIAL_X - 50; // Target X coordinate
double TARGET_Y = INITIAL_Y; // Target Y coordinate
// const double MOVE_SPEED_PCT = 30; // Base speed for movement (% of max) - now mostly for max speed in PID
const double MOVE_SPEED_PCT = 30;
// STOP_DISTANCE_INCHES now applies to the REAR-FACING sensor
const double STOP_DISTANCE_INCHES = 10; // Distance to stop from obstacle (inches)
// const double STOP_DISTANCE_INCHES = 15; // make it large for testing.
const double TARGET_REACHED_THRESHOLD = 5; // Distance to consider target reached (inches)
const double OBSTACLE_AVOIDANCE_TIME = 4000; // Max time to spend avoiding an obstacle (ms)
const int MOVE_TIMEOUT = 5000; // Maximum time for turn functions, in milliseconds.
const double SMALL_FORWARD_DIST_INCHES = 10.0; // Inches to move forward for obstacle avoidance

// Function to update robot's position based on motor encoders and IMU
void updateOdometry() {
 float leftEncoder = LeftMotor.position(rev);
 float rightEncoder = RightMotor.position(rev);

 float deltaLeft = leftEncoder - lastLeftEncoder;
 float deltaRight = rightEncoder - lastRightEncoder;

 float leftTravel = deltaLeft * M_PI * wheelDiameter;
 float rightTravel = deltaRight * M_PI * wheelDiameter;

 lastLeftEncoder = leftEncoder;
 lastRightEncoder = rightEncoder;

 float deltaDistance = (leftTravel + rightTravel) / 2.0;

 theta = IMU.heading(degrees); // Always use IMU for current absolute heading


 // This is CORRECT for odometry: it tracks the robot's center based on its heading.
 // The 'direction' of deltaDistance determines if it moves along or against theta.
 // For forward movement, deltaDistance is positive and we add.
 // For backward movement, deltaDistance will be negative, and we still add (resulting in subtraction from x,y).
 xx += deltaDistance * cos(theta * M_PI / 180.0);
 yy += deltaDistance * sin(theta * M_PI / 180.0);
}


// Static variables to store the last encoder values
static float lastLeftEncoder = 0.0f;
static float lastRightEncoder = 0.0f;

int D = 4; //Defining wheel diameter
float pi = 3.14; //pi
int stage = 1; //starting stage

float leftDist = leftDistance.objectDistance(inches);
float frontDist = frontDistance.objectDistance(inches);

float detectedObjectWidthColor = 0; // Use width to estimate distance to the object

//--- FLAGS ---
bool missionCompleted = false; // Flag to indicate when the main mission is done

bool tubeColorWasDetected = false;
bool obstacleDetected = false;
bool wallDetected = false; // Flag for wall detection (uncomment if using wall detection)

float detectedObjectWidth = 0; // Use width to estimate distance to the object

//for motions
double distanceToTarget;
double angleToTarget;
double angleDifference;

// Obstacle timer
timer obstacleTimer;

// generating and setting random seed
void initializeRandomSeed(){
  int systemTime = Brain.Timer.systemHighResolution();
  double batteryCurrent = Brain.Battery.current();
  double batteryVoltage = Brain.Battery.voltage(voltageUnits::mV);

  // Combine these values into a single integer
  int seed = int(batteryVoltage + batteryCurrent * 100) + systemTime;

  // Set the seed
  srand(seed);
}

void vexcodeInit() {
  //Initializing random seed.
  initializeRandomSeed();
}

//---VISION SENSOR RELATED---
void detect_tube_color() {
  Camera.takeSnapshot(tubeColor); // Take a snapshot looking for the specific tube color signature
}

// ---MOTION RELATED---
void drive(int lspeed, int rspeed, int wt) {
  LeftMotor.spin(forward, lspeed * 120, voltageUnits::mV);
  RightMotor.spin(forward, rspeed * 120, voltageUnits::mV);
  wait(wt, msec);
}

//drive brake func
void driveBrake(){
  drive(0,0,0);
}

int inchDrive(double targetDistanceInches, directionType dir, int maxSpeed) {
    Brain.Screen.print("Driving %.1f inches %s...\n", targetDistanceInches, (dir == directionType::fwd ? "forward" : "backward"));
    LeftMotor.resetPosition(); // Reset encoder positions for accurate distance measurement
    RightMotor.resetPosition();

    dist_drive_integral = 0.0; // Reset PID state for new drive
    dist_drive_prevError = 0.0;

    int driveTimeoutTimer = 0;
    double currentDistanceTraveled = 0.0;

    do {
      // hmmm we may not go full speed.
      if (frontDistance.objectDistance(inches) < STOP_DISTANCE_INCHES) {
        break;
      }
        // Calculate current distance traveled based on average encoder position
        // Encoders are in revolutions, convert to inches
        currentDistanceTraveled = ((fabs(LeftMotor.position(rev)) + fabs(RightMotor.position(rev))) / 2.0) * (M_PI * wheelDiameter);
       
        double error = targetDistanceInches - currentDistanceTraveled;

        dist_drive_integral += error;
        // Integral wind-up prevention and clamping
        if (fabs(error) > 10) dist_drive_integral = 0; // Reset integral if error is too large
        if (dist_drive_integral > 500) dist_drive_integral = 500;
        if (dist_drive_integral < -500) dist_drive_integral = -500;

        double derivative = error - dist_drive_prevError;
        dist_drive_prevError = error;

        double driveSpeed = (error * DIST_DRIVE_KP) + (dist_drive_integral * DIST_DRIVE_KI) + (derivative * DIST_DRIVE_KD);

        // Apply deadband and minimum/maximum speeds
        if (fabs(error) < DIST_DRIVE_DEADBAND) {
            driveSpeed = 0;
        } else if (fabs(driveSpeed) < DIST_DRIVE_MIN_SPEED) {
            driveSpeed = (driveSpeed > 0 ? DIST_DRIVE_MIN_SPEED : -DIST_DRIVE_MIN_SPEED);
        }

        if (fabs(driveSpeed) > maxSpeed) { // Use provided maxSpeed
            driveSpeed = (driveSpeed > 0 ? maxSpeed : -maxSpeed);
        }
       
        // Ensure speed direction matches desired drive direction
        // The `dir` parameter already specifies the direction, so we just need the magnitude of speed.
        LeftMotor.spin(dir, fabs(driveSpeed), velocityUnits::pct);
        RightMotor.spin(dir, fabs(driveSpeed), velocityUnits::pct);

        updateOdometry();
        displayOdometry();
        wait(20, msec);
        driveTimeoutTimer += 20;

    } while (fabs(targetDistanceInches - currentDistanceTraveled) > DIST_DRIVE_DEADBAND && driveTimeoutTimer < MOVE_TIMEOUT);

    LeftMotor.stop(brake);
    RightMotor.stop(brake);
    vex::wait(100, msec);

    if (driveTimeoutTimer >= MOVE_TIMEOUT) {
        Brain.Screen.print("Drive Distance Timed Out\n");
        return 2;
    } else if (fabs(targetDistanceInches - currentDistanceTraveled) > DIST_DRIVE_DEADBAND) {
        Brain.Screen.print("Obstacle Detected\n");
        return 1;
    } else {
        Brain.Screen.print("Drive Distance Completed\n");
        return 0;
    }
}

// Function to drive forward a small, fixed distance
void moveBackwardSmallDistance(double distanceInches, int speed) {
    // Now uses the PID controlled driveDistancePID function
    inchDrive(distanceInches, directionType::rev, speed);
}


int turnToHeading(double targetHeading) {
  Brain.Screen.newLine();
  Brain.Screen.print("Turning to Heading: %.1f\n", targetHeading);
  int turnTimeoutTimer = 0;

  heading_integral = 0.0; // Reset integral term for each new turn
  heading_prevError = 0.0; // Reset previous error

  do {
    theta = IMU.heading(degrees); // Get latest heading
    double error = targetHeading - theta;

    // Normalize angle error to be between -180 and 180
    while (error > 180) error -= 360;
    while (error <= -180) error += 360;

    heading_integral += error; // Accumulate error
    // Integral wind-up prevention: only accumulate integral if error is within a reasonable range
    if (fabs(error) > 50) heading_integral = 0; // Reset integral if error is too large
    // Also, clamp integral to prevent excessive build-up
    if (heading_integral > 1000) heading_integral = 1000;
    if (heading_integral < -1000) heading_integral = -1000;


    double derivative = error - heading_prevError; // Calculate derivative
    heading_prevError = error; // Update previous error

    double turnSpeed = (error * HEADING_TURN_KP) + (heading_integral * HEADING_TURN_KI) + (derivative * HEADING_TURN_KD);

    // Apply deadband and minimum/maximum speeds
    if (fabs(error) < HEADING_ANGLE_DEADBAND) { // If within deadband, stop
      turnSpeed = 0;
    } else if (fabs(turnSpeed) < HEADING_MIN_TURN_SPEED) { // Ensure minimum speed to overcome friction
      turnSpeed = (turnSpeed > 0 ? HEADING_MIN_TURN_SPEED : -HEADING_MIN_TURN_SPEED);
    }

    if (fabs(turnSpeed) > HEADING_MAX_TURN_SPEED) { // Limit max turning speed
      turnSpeed = (turnSpeed > 0 ? HEADING_MAX_TURN_SPEED : -HEADING_MAX_TURN_SPEED);
    }

    LeftMotor.spin(directionType::fwd, turnSpeed, velocityUnits::pct);
    RightMotor.spin(directionType::fwd, -turnSpeed, velocityUnits::pct); // Opposing speeds for in-place turn

    updateOdometry();
    displayOdometry();
    wait(20, msec);
    turnTimeoutTimer += 20;

  } while (fabs(targetHeading - theta) > HEADING_ANGLE_DEADBAND && turnTimeoutTimer < MOVE_TIMEOUT);
  driveBrake();
  vex::wait(100, msec);

  if (turnTimeoutTimer >= MOVE_TIMEOUT) {
    Brain.Screen.print("Turn To Heading Timed Out\n");
    return 2;
  } else {
    Brain.Screen.print("Turn To Heading Completed\n");
    return 0;
  }
}

// Function to display the odometry data on the controller screen
void displayOdometry() {
  float dist = frontDistance.objectDistance(inches);
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("stage: %d, targetx %.1f in, targety %.1f in\n", stage, TARGET_X, TARGET_Y);
  Brain.Screen.setCursor(2, 1); // Set cursor to the top-left corner
  Brain.Screen.print("X: %.1f in, Y: %.1f in, Theta: %.1f deg\n", xx, yy, theta); // More compact odometry
  Brain.Screen.setCursor(3, 1);
  Brain.Screen.print("Dist: %.1f in", dist);
}

// Target distance for side sensors during obstacle avoidance (how far to keep from the wall)
const double SIDE_TARGET_DIST_BACKWARD = 14.0; // Aim to keep 8 inches from the wall/obstacle when backing up


// Function prototypes to resolve circular dependencies
void obstacleDetection(); // Forward declaration
void obstacleMoveBackward(); // Forward declaration for new backward avoidance function
void turnLeftBackward(); // Forward declaration for new backward turn
void turnRightBackward(); // Forward declaration for new backward turn


/**
* @brief Drives the robot a specific distance using PID control.
* @param targetDistanceInches The absolute distance in inches to drive.
* @param dir The direction to drive (fwd or rev).
* @param maxSpeed The maximum speed percentage to use for driving.
* @return 0 if completed, 2 if timed out.
*/

/**
* @brief Drives the robot backward while attempting to maintain a specific offset
* from a side obstacle, until the rear-facing sensor (frontDistance) clears.
* This is the "reverse wall-following" part of avoidance.
* NOTE: This function is currently not used in the new obstacle avoidance strategy,
* but is kept for potential future use or other scenarios.
*/
// --OBSTACLE DETECTION--
void obstacleMoveBackward() {
  obstacleTimer.clear();
  Brain.Screen.print("Obstacle Move Backward...\n");

  avoid_integral = 0.0; // Reset integral for new avoidance
  avoid_prevError = 0.0; // Reset previous error

  // Keep moving backward and correcting until rear (frontDistance) is clear OR avoidance time is up
  while (frontDistance.objectDistance(inches) < STOP_DISTANCE_INCHES + 2 && obstacleTimer.time(msec) < OBSTACLE_AVOIDANCE_TIME) {
    // float currentRearDist = frontDistance.objectDistance(inches);
    float leftDist = leftDistance.objectDistance(inches);
    float rightDist = rightDistance.objectDistance(inches);

    double targetSideDist = SIDE_TARGET_DIST_BACKWARD;
    double error = 0.0;

    // Determine error based on which side we are avoiding
    if (turnLeftOrRight == 1) { // Pivoting rear to the LEFT, keeping obstacle on physical RIGHT
      error = targetSideDist - rightDist; // Positive error means too far from wall, need to turn more left
    } else if (turnLeftOrRight == 2) { // Pivoting rear to the RIGHT, keeping obstacle on physical LEFT
      error = leftDist - targetSideDist; // Positive error means too far from wall, need to turn more right
    } else {
      Brain.Screen.print("Error: turnLeftOrRight not set for backward avoidance!\n");
      // Fallback: If no specific turn direction, try to center
      error = leftDist - rightDist; // If positive, left is further, turn right
    }

    avoid_integral += error;
    // Integral wind-up prevention and clamping
    if (fabs(error) > 10) avoid_integral = 0; // Reset integral if error is too large
    if (avoid_integral > 500) avoid_integral = 500;
    if (avoid_integral < -500) avoid_integral = -500;

    double derivative = error - avoid_prevError;
    avoid_prevError = error;

    double avoidanceCorrection = (error * AVOID_KP) + (avoid_integral * AVOID_KI) + (derivative * AVOID_KD);

    // Apply deadband for correction
    if (fabs(error) < AVOID_DIST_DEADBAND) {
      avoidanceCorrection = 0;
    }

    // Cap the correction to prevent overly aggressive turns
    if (avoidanceCorrection > AVOID_MAX_SPEED / 2) avoidanceCorrection = AVOID_MAX_SPEED / 2;
    if (avoidanceCorrection < -AVOID_MAX_SPEED / 2) avoidanceCorrection = -AVOID_MAX_SPEED / 2;

    double baseSpeed = -MOVE_SPEED_PCT; // Always negative for reverse

    double leftSpeed = baseSpeed;
    double rightSpeed = baseSpeed;

    // Apply the correction to motor speeds for backward movement
    // Positive avoidanceCorrection means need to turn REAR to the LEFT (robot rotates CCW)
    // To turn CCW while moving backward: Left motor faster backward, Right motor slower backward
    leftSpeed -= avoidanceCorrection;
    rightSpeed += avoidanceCorrection;

    // Clamp motor speeds to -100 to 100 percent
    leftSpeed = fmax(-100, fmin(100, leftSpeed));
    rightSpeed = fmax(-100, fmin(100, rightSpeed));

    // Ensure a minimum speed for movement if not at target
    if (fabs(leftSpeed) < AVOID_MIN_SPEED) leftSpeed = (leftSpeed <= 0 ? -AVOID_MIN_SPEED : AVOID_MIN_SPEED);
    if (fabs(rightSpeed) < AVOID_MIN_SPEED) rightSpeed = (rightSpeed <= 0 ? -AVOID_MIN_SPEED : AVOID_MIN_SPEED);

    LeftMotor.spin(directionType::rev, fabs(leftSpeed), velocityUnits::pct);
    RightMotor.spin(directionType::rev, fabs(rightSpeed), velocityUnits::pct);

    updateOdometry();
    displayOdometry();
    wait(20, msec); // Loop delay
  }
  driveBrake();
  obstacleDetected = false; // Once out of the loop, assume obstacle is cleared or time is up
  turnLeftOrRight = 0; // Reset avoidance direction
  Brain.Screen.print("Obstacle Move Backward Done\n");
}

void turnLeftBackward() {
 obstacleTimer.clear();
 Brain.Screen.newLine();
 Brain.Screen.print("Turn Left Backward Triggered (Initial Pivot)\n");
 turnLeftOrRight = 1; // Indicate that we are trying to avoid by moving rear to the left


 // Pivot rear to the left (robot rotates CCW) until the rear sensor reports clear (or timeout)
 // Left motor reverses, Right motor forwards (relative to its own forward direction)
 while (frontDistance.objectDistance(inches) < STOP_DISTANCE_INCHES && obstacleTimer.time(msec) < OBSTACLE_AVOIDANCE_TIME / 2) {
   LeftMotor.spin(directionType::rev, 20, velocityUnits::pct); // Faster backward (CCW pivot)
   RightMotor.spin(directionType::fwd, 20, velocityUnits::pct); // Slower backward / effectively forward to pivot
   updateOdometry();
   displayOdometry();
   wait(20, msec);
 }
 LeftMotor.stop(brake);
 RightMotor.stop(brake);
 wait(100, msec); // Small pause


 // Now, attempt to move backward and follow the path around the obstacle
 obstacleMoveBackward();
}

void turnRightBackward() {
 obstacleTimer.clear();
 Brain.Screen.newLine();
 Brain.Screen.print("Turn Right Backward Triggered (Initial Pivot)\n");
 turnLeftOrRight = 2; // Indicate that we are trying to avoid by moving rear to the right


 // Pivot rear to the right (robot rotates CW) until the rear sensor reports clear (or timeout)
 // Right motor reverses, Left motor forwards
 while (frontDistance.objectDistance(inches) < STOP_DISTANCE_INCHES && obstacleTimer.time(msec) < OBSTACLE_AVOIDANCE_TIME / 2) {
   RightMotor.spin(directionType::rev, 20, velocityUnits::pct); // Faster backward (CW pivot)
   LeftMotor.spin(directionType::fwd, 20, velocityUnits::pct);  // Slower backward / effectively forward to pivot
   updateOdometry();
   displayOdometry();
   wait(20, msec);
 }
 RightMotor.stop(brake);
 LeftMotor.stop(brake);
 wait(100, msec); // Small pause


 // Now, attempt to move backward and follow the path around the obstacle
 obstacleMoveBackward();
}

void obstacleDetection() {
    float currentRearDist = frontDistance.objectDistance(inches); // Rear-facing sensor
    float leftDist = leftDistance.objectDistance(inches);
    float rightDist = rightDistance.objectDistance(inches);

    Brain.Screen.print("Obstacle Detection (Rear): Rear=%.1f, Left=%.1f, Right=%.1f\n", currentRearDist, leftDist, rightDist);

    // Stop motors immediately upon detection
    LeftMotor.stop(brake);
    RightMotor.stop(brake);
    obstacleDetected = true; // Set the flag

    // If the rear sensor is suddenly clear, exit avoidance (this might happen after moving forward)
    if (currentRearDist > STOP_DISTANCE_INCHES + 2) { // Add buffer
        Brain.Screen.print("Rear clear, exiting obstacle detection.\n");
        obstacleDetected = false;
        return;
    }

    // Determine new heading for avoidance
    double avoidanceTurnHeading = theta; // Start with current heading

    // Logic to decide which way to turn after moving forward
    // If left side is clearer, turn robot's front to the left (CCW turn)
    // If right side is clearer, turn robot's front to the right (CW turn)
    if (leftDist > (rightDist + 5)) { // Left side is significantly clearer (add a threshold for significance)
        Brain.Screen.print("Left side clearer. Turning front Left (CCW).\n");
        avoidanceTurnHeading = theta + 90; // Turn 90 degrees left
    } else if (rightDist > (leftDist + 5)) { // Right side is significantly clearer
        Brain.Screen.print("Right side clearer. Turning front Right (CW).\n");
        avoidanceTurnHeading = theta - 90; // Turn 90 degrees right
    } else { // Both sides are equally blocked or clear, or not significantly different
        Brain.Screen.print("Sides similar. Defaulting to turning front Right (CW).\n");
        avoidanceTurnHeading = theta - 90; // Default to turning right if no clear preference
    }

    // Normalize avoidanceTurnHeading to be between 0 and 360
    while (avoidanceTurnHeading >= 360) avoidanceTurnHeading -= 360;
    while (avoidanceTurnHeading < 0) avoidanceTurnHeading += 360;

    Brain.Screen.print("Turning to avoid obstacle to heading: %.1f\n", avoidanceTurnHeading);
    turnToHeading(avoidanceTurnHeading); // Turn to the calculated avoidance heading
    // for (int i = 0; i < 3; i++) {
    //   Brain.Screen.newLine();
    //   Brain.Screen.print("%d of 3 tries to turn to heading", i+1);
    //   int ret = turnToHeading(avoidanceTurnHeading); // Turn to the calculated avoidance heading
    //   if (ret == 0) break;
    //   wait(2000, msec);
    // }

    // NEW BEHAVIOR: Move forward a bit to create space
    moveBackwardSmallDistance(SMALL_FORWARD_DIST_INCHES, DIST_DRIVE_MAX_SPEED); // Use PID for this movement

    // theta = IMU.heading(degrees);

    obstacleDetected = false; // Once avoidance turn is complete, reset flag
    Brain.Screen.print("Obstacle Avoidance Turn Done.\n");
}
// Wall Color Description (Signature 2). Uncomment and use if needed.
// Ensure your Signature 2 in the Vision Utility matches 'wall'.
aivision::colordesc wall = {2, 201, 182, 160, 10.0, 0.20};

void detectWall() {
 Camera.takeSnapshot(wall); // Take snapshot looking for the wall signature
 if (Camera.objectCount > 0) {
   wallDetected = true; // Wall detected
   Brain.Screen.setCursor(1,1);
   Brain.Screen.print("Wall Detected!");
 } else {
   wallDetected = false; // No wall detected
   Brain.Screen.setCursor(1,1);
   Brain.Screen.print("No Wall Seen.");
 }
}

bool completed = false;
float tagHeight = 0;

void compute_tracking_turn_speed(float& turn_speed, float& fwd_speed) {
  // Reset PID state for AprilTag tracking at the start of each new tracking attempt
  static bool firstCall = true;
  if (firstCall) {
    tag_turn_integral = 0.0;
    tag_turn_prevError = 0.0;
    tag_fwd_integral = 0.0;
    tag_fwd_prevError = 0.0;
    firstCall = false;
  }

  Brain.Screen.clearScreen();
  if (Camera.objectCount > 0) {
    Brain.Screen.print("AprilTags detected!");
    auto obj = Camera.objects[0]; // Assuming we track the first detected tag
    tagHeight = obj.height;

    // --- PID for Turning (Centering X-axis) ---
    float object_x_error = obj.centerX - 160; // Offset by half the camera width (160 for 320px image)
    tag_turn_integral += object_x_error;
    // Integral wind-up prevention and clamping
    if (fabs(object_x_error) > 100) tag_turn_integral = 0;
    if (tag_turn_integral > 500) tag_turn_integral = 500;
    if (tag_turn_integral < -500) tag_turn_integral = -500;

    double tag_turn_derivative = object_x_error - tag_turn_prevError;
    tag_turn_prevError = object_x_error;

    turn_speed = (object_x_error * TAG_TURN_KP) + (tag_turn_integral * TAG_TURN_KI) + (tag_turn_derivative * TAG_TURN_KD);

    // --- PID for Forward Movement (Distance based on Height) ---
    float desired_object_height = 250; // TUNE THIS!
    float height_error = desired_object_height - tagHeight;

    tag_fwd_integral += height_error;
    // Integral wind-up prevention and clamping
    if (fabs(height_error) > 50) tag_fwd_integral = 0;
    if (tag_fwd_integral > 500) tag_fwd_integral = 500;
    if (tag_fwd_integral < -500) tag_fwd_integral = -500;

    double tag_fwd_derivative = height_error - tag_fwd_prevError;
    tag_fwd_prevError = height_error;

    fwd_speed = (height_error * TAG_FWD_KP) + (tag_fwd_integral * TAG_FWD_KI) + (tag_fwd_derivative * TAG_FWD_KD);

    // --- Apply Deadbands and Speed Limits ---
    if (fabs(object_x_error) < TAG_POS_DEADBAND) { // If centered enough
        turn_speed = 0;
    } else if (fabs(turn_speed) < TAG_MIN_SPEED) { // Ensure minimum speed
        turn_speed = (turn_speed > 0 ? TAG_MIN_SPEED : -TAG_MIN_SPEED);
    }
    if (fabs(turn_speed) > TAG_MAX_SPEED) { // Limit max speed
        turn_speed = (turn_speed > 0 ? TAG_MAX_SPEED : -TAG_MAX_SPEED);
    }

    if (fabs(height_error) < TAG_HEIGHT_DEADBAND) { // If at desired distance
        fwd_speed = 0;
    } else if (fabs(fwd_speed) < TAG_MIN_SPEED) { // Ensure minimum speed
        fwd_speed = (fwd_speed > 0 ? TAG_MIN_SPEED : -TAG_MIN_SPEED);
    }
    if (fabs(fwd_speed) > TAG_MAX_SPEED) { // Limit max speed
        fwd_speed = (fwd_speed > 0 ? TAG_MAX_SPEED : -TAG_MAX_SPEED);
    }

    firstCall = false; // Not the first call anymore
  } else {
    Brain.Screen.print("No AprilTags detected.");
    turn_speed = 0;
    fwd_speed = 0;
    firstCall = true; // Reset for next detection
  }
}

void stage1(){
  while (stage==1){
    if (leftDist<35 && frontDist<50)

  }
}
// Function to detect and track AprilTags
void detect_tag(int id) {
 Camera.takeSnapshot(aivision::ALL_TAGS); // Take a snapshot looking for tags
}

void pre_auton(void) {

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}


void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
}

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
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

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}