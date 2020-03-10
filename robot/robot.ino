#include <Wire.h>       // Magnetometer
#include <Servo.h>      // Servo
#include <String.h>
#include <LSM303.h>     // Magnetometer
#include <SPI.h>        // SD
#include <SD.h>         // SD

// Variable prefixes:
// e denotes an Experimentally determined value.
// p denotes a user-settable parameter.
// v denotes a dyanamic variable.

// Units:
// All angle units are given in RADS.
// All distance units are given in INCHES.
// All time units are given in (ms)

File logFile;
char SDfileName[] = "log.txt"; // SD library only supports up to 8.3 names
const uint8_t pSDindicator = 7;
const uint8_t pSDchipSelect = 8;
const uint8_t pSDcardDetect = 9;
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
        return String(String(x) + "\t" + String(y));
    }
};

// Robot Attributes.
class Robot {
public:
    float wheelbase;// dist rear axle -> front axle
    float track;// dist between 2 front or 2 rear wheels
    float wheelR;// wheel radius
    unsigned nMagnets;// number of magnets
    float servoAngleDeltaMax;// (rad/ms) [EXPERIMENT] max rotational speed to steering mechanism
    float servoAngleMax;// (radians) [EXPERIMENT] max rotation angle of steering mechanism
    unsigned pistonLowT;// (ms) piston "low" time
    unsigned pistonHighT;// (ms) piston "high" time
    Vec2 vCurrentPosition;
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
// Hardware pins, interfaces, etc.
Servo servo;
LSM303 compass;
// ######## PINS ############
const uint8_t pServoPin = 3;
const uint8_t pReedSwitchPin = 4;
const uint8_t pSolenoidPin = 2;
const uint8_t pLEDpin = 7;
const uint8_t pStartButtonPin = 6;
const uint8_t pGainPin = A1; // potentiometer @ A1
const uint8_t pSteerBiasPin = A2; // potentiometer @ A2
// ######## USER-SET PARAMS ###########
const bool pLog = true;
const unsigned long pLoopPeriod = 25;  // loop period, ms
const unsigned long pMaxTime = 25 * 1000; // max run time, in ms
const uint8_t noTargets = 3; // bugs will come if this isn't equal to the len of targets[] below.
const Vec2 targets [noTargets] = { Vec2(60, 60), Vec2(150, 100), Vec2(100, -40) };
const float ptargetOKDist = 10.00;
const float pFilterParam = 0.7;
// ######## PHYSICAL PARAMS ######
const float eWheelbase = 9;
const float eTrack = 9;
const float eWheelR = 3.149606; // 80mm wheels
const unsigned enMagnets = 4;
const float eServoAngleDeltaMax = 1 * M_PI/180;
const float eServoAngleMax = 40 * M_PI/180;
unsigned ePistonLowT = 400;
unsigned ePistonHighT = 400;
// ######## VARS #############
float pKp;
float steerBias;
uint16_t LEDt1, LEDt0;
bool LEDstatus;
int vSolenoidState = LOW;
unsigned long vTimeSolOn, vTimeSolOff;
float vServoAngle;
bool vSwitchStateLast, vSwitchState;
unsigned long vMagnetPassed0, vMagnetPassed1;
float r, dr;
unsigned long vOldTime, vCurrentTime;
unsigned long vbeginTime; // time that the main loop will start
uint8_t vtargetIndex = 0;
bool vTerminate = false;
bool vStart = false;
bool vPistonState;

// create a robot and place it at [0,0]
Robot robot = Robot(eWheelbase, eTrack, eWheelR, enMagnets, eServoAngleDeltaMax, eServoAngleMax, ePistonLowT, ePistonHighT, Vec2(0.0, 0.0) );

void setup() { 
    servo.attach(3); 
    Wire.begin();
    compass.init();
    compass.enableDefault();
    compass.writeReg(0x24, 0x74);
    // Calibrated by ms on 3/2
    // min: { -3305,  -2375,  -2478}    max: { +1771,  +2824,  +2951}
    compass.m_min = (LSM303::vector<int16_t>){ -3305,  -2375,  -2478};
    compass.m_max = (LSM303::vector<int16_t>){ +1771,  +2824,  +2951};
    pinMode(pSDcardDetect, INPUT);
    pinMode(pLEDpin, OUTPUT);
    pinMode(pReedSwitchPin, INPUT_PULLUP);
    pinMode(pGainPin, INPUT);
    pinMode(pSteerBiasPin, INPUT);
    pinMode(pStartButtonPin, INPUT_PULLUP);
    vCurrentTime = 0;
    // if sd card is not present we set flag
    if(SD.begin(pSDchipSelect)) SDcardPresent = true;
    else SDcardPresent = false;
    if (pLog && SDcardPresent) {
        // write data column labels to data file
        logFile = SD.open(SDfileName, FILE_WRITE);
        if (logFile) {
            logFile.print(String("t\trta\tsat\tsa\tposx\tposy\ttposx\ttposy\theading\t"));
            logFile.close();
        }
    }
    
    // in this loop till the button is pressed
    while(vStart == false) {
        pKp = analogRead(pGainPin)/float(1024);
        steerBias = analogRead(pSteerBiasPin)/float(1024) * eServoAngleMax;
        servo.write(steerBias * 180/M_PI);
        vCurrentTime = millis();
        ledIndicate(vCurrentTime, 900, 150, pLEDpin);
        if(digitalRead(pStartButtonPin) == HIGH) vStart = true;
    }
    if (pLog && SDcardPresent) {
        logFile = SD.open(SDfileName, FILE_WRITE);
            if (logFile) {
                logFile.println(String("Gain = " + String(pKp) + "\tSteerBias = " + String(steerBias * 180/M_PI)));
                logFile.close();
            }
    }
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
        vSolenoidState = actuatePiston(vCurrentTime, vTimeSolOn, vTimeSolOff, ePistonLowT,ePistonLowT, vPistonState);
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


        // currently compass.read() hangs.
        // read and filter heading
        compass.read();

        float headingDegrees = compass.heading();
        Serial.println(headingDegrees);
        // Control loop
        if( (vCurrentTime - vOldTime) >= pLoopPeriod) {
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
            controlInfo = updateServo(targets[vtargetIndex], servo, robot, pKp, steerBias);
            // only write file if we're logging
            if (pLog && SDcardPresent) {
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
String updateServo(const Vec2 &target, Servo &servo, const Robot &robot, const float Kp, const float angleBias) {
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
    servo.write( (servoAngle + angleBias) * 180 / M_PI);
    // return info
    return String(String(robotToTargetAngle * 180 * M_1_PI,3) + "\t" + String(sat * 180 * M_1_PI) + "\t" + String(servoAngle * 180 * M_1_PI) + "\t");
}

// method for actuating piston
int actuatePiston(const unsigned long &vCurrentTime, 
                  unsigned long &prevTimeOn, 
                  unsigned long &prevTimeOff, 
                  unsigned pistonLowT,
                  unsigned pistonHighT,
                  bool pistonState) {
    // if it's on we turn it off...
    Serial.println("Piston Actuated:");
    if(pistonState && (vCurrentTime - prevTimeOff) > pistonHighT) {
        Serial.println("\tPiston Off");

        pistonState = false;
        prevTimeOn = vCurrentTime;
        return LOW;
    }
    // if it's off we turn it on...
    if( (vCurrentTime - prevTimeOn) > pistonLowT) {
        Serial.println("\tPiston On");

        pistonState = true;
        prevTimeOff = vCurrentTime;
        return HIGH;
    }
}

float sign(float u) {
    if (u >= 0) return 1.0F;
    else return -1.0F;
}
float wrapPi(float u) {
    if (u > M_PI) return (u-2*M_PI);
    else return u;
}
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
// x0: current reading
// x1: prev. reading
// a : filter coeff
float filter1stOrder(const float x0, const float x1, const float a) {
    return x1 * (1 - a) + x0 * (a);
}
