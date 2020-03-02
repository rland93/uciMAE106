#include <Wire.h>       // Magnetometer
#include <Servo.h>      // Servo
#include <String.h>
#include <LSM303.h>     // Magnetometer
#include <SPI.h>        // SD
#include <SD.h>         // SD
File logFile;
char SDfileName[] = "log.txt"; // SD library only supports up to 8.3 names
const uint8_t SDindicator = 7;
const uint8_t SDchipSelect = 8;
const uint8_t SDcardDetect = 9;
enum SDstates: uint8_t { NORMAL, E, EO };
uint8_t SDstate = NORMAL;
bool SDalreadyBegan = false;  // SD.begin() misbehaves if not first call 
bool SDcardPresent;
// 2-Component vector class.
class Vec2 {
    public:
    float x, y;
    Vec2(float x, float y) : x(x), y(y) {};
    Vec2() : x(0.0), y(0.0) {};
    // add 2 vectors together
    Vec2 operator + (const Vec2 &u) const {
        return Vec2(x + u.x, y + u.y);}
    // subtract 2 vectors
    Vec2 operator - (const Vec2 &u) const {
        return Vec2(x - u.x, y - u.y);}
    // get mag^2
    const float mag2() const {
        return x*x + y*y;}
    // return [x,y] vector as r, theta.
    Vec2 xy_to_rtheta() const {
        if(x != 0.0f) return Vec2(sqrt(this->mag2()), tan(y/x));
        else return Vec2(sqrt(this->mag2()), M_PI_2);}
    // return [r,t] vector as x, y
    Vec2 rtheta_to_xy() const {
        return Vec2(x * cos(y), x * cos(y));}
    // vector cross product 
    float cross(const Vec2 &u) const {
        return (x * u.y - y * u.x);}
    // vector dot product
    float dot(const Vec2 &u) const {
        return x * u.x + y * u.y;}
    // return vector as string
    String info() const {
        return String("[" + String(x) + ", " + String(y) + "]");
    }
};

// Robot Attributes.
class Robot {
public:
    // dist rear axle -> front axle
    float wheelbase;
    // dist between 2 front or 2 rear wheels
    float track;
    // wheel radius
    float wheelR;
    // number of magnets
    unsigned nMagnets;
    // (rad/ms) [EXPERIMENT] max rotational speed to steering mechanism
    float servoAngleDeltaMax;
    // (radians) [EXPERIMENT] max rotation angle of steering mechanism
    float servoAngleMax;
    // (ms) piston "low" time
    unsigned pistonLowT;
    // (ms) piston "high" time
    unsigned pistonHighT;
    // robot's current position
    Vec2 vCurrentPosition;
    // robot's current heading
    float vCurrentHeading;
    Robot(float wheelbase, float track, float wheelR, unsigned nMagnets, float servoAngleDeltaMax, float servoAngleMax, unsigned pistonLowT, unsigned pistonHighT, Vec2 initialPosition) :
    wheelbase(wheelbase),
    track(track),
    wheelR(wheelR),
    nMagnets(nMagnets),
    servoAngleDeltaMax(servoAngleDeltaMax),
    servoAngleMax(servoAngleMax),
    pistonLowT(pistonLowT),
    pistonHighT(pistonHighT),
    vCurrentPosition(initialPosition) {}
};

// Variable prefixes:
// e denotes an Experimentally determined value.
// p denotes a user-settable parameter.
// v denotes a dyanamic variable.

// Units:
// All angle units are given in RADS.
// All distance units are given in INCHES.
// All time units are given in (ms)

// Hardware pins, interfaces, etc.
Servo servo;
LSM303 compass;
const uint8_t pServoPin = 3;
const uint8_t pReedSwitchPin = 4;
const uint8_t pSolenoidPin = 2;
const uint8_t pLEDpin = 7;
const bool pLog = true;
// Robot Parameters
const float eWheelbase = 9;
const float eTrack = 9;
const float eWheelR = 3.149606; // 80mm wheels
const unsigned enMagnets = 4;
const float eServoAngleDeltaMax = 10 * M_PI/180;
const float eServoAngleMax = 90 * M_PI/180;
unsigned ePistonLowT = 400;
unsigned ePistonHighT = 400;
float pKp = .8;
// misc
uint16_t LEDt1, LEDt0;
bool LEDstatus;
int vSolenoidState = LOW;
unsigned long vTimeSolOn, vTimeSolOff;
float vServoAngle;
bool vSwitchStateLast, vSwitchState;
unsigned long vMagnetPassed0, vMagnetPassed1;
float r, dr;
unsigned long vOldTime, vCurrentTime;
const unsigned long pLoopPeriod = 40;  // loop period, ms
unsigned long vbeginTime; // time that the main loop will start
const unsigned long pMaxTime = 40000; // max time, in ms
// List of Targets
const uint8_t noTargets = 3; // bugs will come if this isn't equal to the len of targets[] below.
const Vec2 targets [noTargets] = { Vec2(60, 60), Vec2(150, 100), Vec2(100, -40) };
uint8_t vtargetIndex = 0;
const float ptargetOKDist = 10.00;
bool vTerminate = false;

// create a robot and place it at [0,0]
Robot robot = Robot(eWheelbase, eTrack, eWheelR, enMagnets, eServoAngleDeltaMax, eServoAngleMax, ePistonLowT, ePistonHighT, Vec2(0.0, 0.0) );

void setup() {  
    // SD Card Setup
    // Note: To satisfy the AVR SPI gods the SD library takes care of setting
    // SS_PIN as an output. We don't need to.
    pinMode(SDcardDetect, INPUT);

    // if sd card is not present we set flag
    if(SD.begin(SDchipSelect)) {
        SDcardPresent = true;
    }
    else {
        SDcardPresent = false;
    }

    // set LED pin
    pinMode(pLEDpin, OUTPUT);

    if (pLog) {
        // write data column labels to data file
        logFile = SD.open(SDfileName, FILE_WRITE);
        if (logFile) {
            logFile.println(String("time\tR.T.A.\tS.A.T.\tS.A.\tcurPos\ttargPos\theading"));
            logFile.close();
        }
    }
    // initialize Servo
    servo.attach(3); 
    // initialize Compass
    Wire.begin();
    compass.init();
    compass.enableDefault();
    compass.writeReg(0x24, 0x74);
    // Calibrated by ms on 2/25
    //                                  min: { -2448,  -2456,  -1208}    
    //                                  max: { +1959,  +2412,  +3001}
    compass.m_min = (LSM303::vector<int16_t>){ -2448,  -2456,  -1208};
    compass.m_max = (LSM303::vector<int16_t>){ +1959,  +2412,  +3001};
    // initialize reed switch
    pinMode(pReedSwitchPin, INPUT_PULLUP);

    // this should always be the last entry in setup()
    vbeginTime = vCurrentTime;
}

void loop() {
    if (not vTerminate) {
        vCurrentTime = millis();
        // SD card indicator Light:
        // SD card absent - constant light
        // SD card present and logging - fast blink
        // SD card present and no logging - slow blink
        if (SDcardPresent && pLog) ledIndicate(vCurrentTime, 50, 150, pLEDpin);
        else if (SDcardPresent && !pLog) ledIndicate(vCurrentTime, 50, 950, pLEDpin);
        else digitalWrite(pLEDpin, HIGH);

        // activate the piston
        vSolenoidState = actuatePiston(vCurrentTime, vTimeSolOn, vTimeSolOff);
        digitalWrite(pSolenoidPin, vSolenoidState);  

        // find distance traveled from veh. speed.
        vSwitchStateLast = vSwitchState;
        vSwitchState = digitalRead(pReedSwitchPin);
        if(vSwitchStateLast == 1 && vSwitchState == 0) {
                // store times for each pulse
                vMagnetPassed1 = vMagnetPassed0;
                vMagnetPassed0 = vCurrentTime;
                // vehicle traveled 1/4 of wheel radius
                dr += .5 * M_PI_2 * robot.wheelR;
                // total dist traveled
                r += .5 * M_PI_2 * robot.wheelR;
        }    


        // Control loop
        if( (vCurrentTime - vOldTime) >= pLoopPeriod) {

            // update the current heading
            compass.read();
            float headingDegrees = compass.heading();
            robot.vCurrentHeading = headingDegrees * M_PI/180;
            // update the robot position
            robot.vCurrentPosition = robot.vCurrentPosition + Vec2(dr, robot.vCurrentHeading).rtheta_to_xy();
            dr = 0;
            // check distance to target, then either update target or vTerminate
            float targetDist = (robot.vCurrentPosition - targets[vtargetIndex]).mag2();
            if (targetDist < ptargetOKDist && vtargetIndex < noTargets) vtargetIndex ++;
            else if (targetDist < ptargetOKDist && vtargetIndex == noTargets) vTerminate = true;

            // update the servo angle and get info
            // controlInfo --> robot to target angle, servo angle target, servo angle
            String controlInfo;
            controlInfo = updateServo(targets[vtargetIndex], servo, robot, pKp);

            // only write file if we're logging
            if (pLog) {
                // open the file
                logFile = SD.open(SDfileName, FILE_WRITE);
                // position, target position, heading
                String positionInfo = String(robot.vCurrentPosition.info() + "\t" + targets[vtargetIndex].info() + "\t" + String(headingDegrees));
                // write all info to logFile and then close the file
                if (logFile) {
                    logFile.println(String(vCurrentTime) + "\t" + controlInfo + positionInfo);
                    logFile.close();
                }
            }
            vOldTime = vCurrentTime;
        }
        // terminate if we exceed time
        if (vCurrentTime - vbeginTime > pMaxTime) vTerminate = true;
    }
}

// steer the servo towards the desired heading.
// ** note. sat stands for Servo Angle Target.
// **      dsat stands for Servo Angle Target Delta.
// returns the data of the update process in a string.
String updateServo(const Vec2 &target, Servo &servo, const Robot &robot, const float Kp) {
    float sat, dsat, targetDist, robotToTargetAngle, servoAngle, currentServoAngle;
    Vec2 robotToTargetVector;
    robotToTargetVector = target - robot.vCurrentPosition;
    targetDist = sqrt( robotToTargetVector.mag2() );
    // position vector from robot to target
    robotToTargetAngle = wrapPi(robot.vCurrentHeading - atan2(robotToTargetVector.y, robotToTargetVector.x));
    // apply gain
    sat = Kp * robotToTargetAngle;
    sat = sign(sat) * min(robot.servoAngleMax, abs(sat));
    currentServoAngle = servo.read() * M_PI/180;
    // servo needs to travel this distance
    dsat = sat - currentServoAngle;
    // servo can only turn so fast
    servoAngle = currentServoAngle + sign(dsat) * min(robot.servoAngleDeltaMax, abs(dsat));
    // make sure we don't go out of bounds
    servoAngle = sign(servoAngle) * min(robot.servoAngleMax, abs(servoAngle));
    servo.write(servoAngle * 180 / M_PI);
    // return info
    return String(String(robotToTargetAngle * 180 * M_1_PI,3) + "\t" + String(sat * 180 * M_1_PI) + "\t" + String(servoAngle * 180 * M_1_PI) + "\t");
}

// method for actuating piston
int actuatePiston(const unsigned long &vCurrentTime, unsigned long &prevTimeOn, unsigned long prevTimeOff) {
    bool pistonState;
    // if it's on we turn it off...
    if(pistonState && (vCurrentTime - prevTimeOff) > prevTimeOn) {
        pistonState = false;
        prevTimeOn = vCurrentTime;
        return LOW;
    }
    // if it's off we turn it on...
    if( (vCurrentTime - prevTimeOn) > prevTimeOff) {
        pistonState = true;
        prevTimeOff = vCurrentTime;
        return HIGH;
    }
}

// return -1 or 1 corresponding to sign of a number
// 0 is a positive number :P
float sign(float u) {
    if (u >= 0) return 1.0F;
    else return -1.0F;
}
float wrapPi(float u) {
    if (u > M_PI) return (u-2*M_PI);
    else return u;
}

// LED Indicator Light 
void ledIndicate(unsigned long &time, const uint16_t LEDon, const uint16_t LEDoff, const uint8_t LEDpin) {
    LEDt0 = (time % 65536);
    // if it's off turn it on
    if ( !LEDstatus && (LEDt0-LEDt1) >= LEDoff) {
        digitalWrite(LEDpin, HIGH);
        LEDstatus = true;
        LEDt1 = LEDt0;
    } 
    if (LEDstatus && (LEDt0-LEDt1) >= LEDon) {
    digitalWrite(LEDpin, LOW);
    LEDstatus = false;
    LEDt1 = LEDt0;
    }
}