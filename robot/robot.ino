#include <Wire.h>
#include <Servo.h>
#include <String.h>
#include <LSM303.h>

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
    const float mag2() {
        return x*x + y*y;}
    // return [x,y] vector as r, theta.
    Vec2 xy_to_rtheta() {
        if(x != 0.0f) return Vec2(sqrt(this->mag2()), tan(y/x));
        else return Vec2(sqrt(this->mag2()), M_PI_2);}
    // return [r,t] vector as x, y
    Vec2 rtheta_to_xy() {
        return Vec2(x * cos(y), x * cos(y));}
    // vector cross product 
    float cross(const Vec2 &u) {
        return (x * u.y - y * u.x);}
    // vector dot product
    float dot(const Vec2 &u) {
        return x * u.x + y * u.y;}
    // return vector as string
    String info() {
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
const int pServoPin = 3;
const int pReedSwitchPin = 4;
const int pSolenoidPin = 2;
const bool pPrintInfo = true;
// Robot Parameters
const float eWheelbase = 9;
const float eTrack = 9;
const float eWheelR = 3.149606; // 80mm wheels
const unsigned enMagnets = 4;
const float eServoAngleDeltaMax = 5 * M_PI/180;
const float eServoAngleMax = 45 * M_PI/180;
unsigned ePistonLowT = 400;
unsigned ePistonHighT = 400;
float pKp = .45;
// Solenoid
int vSolenoidState = LOW;
unsigned long vTimeSolOn, vTimeSolOff;
// Servo
float vservoAngle;
// Switch
bool vSwitchStateLast, vSwitchState;
unsigned long vMagnetPassed0, vMagnetPassed1;
float r, dr;
// Time vars
unsigned long oldTime, currentTime;
const unsigned long pLoopPeriod = 200;  // loop period, ms
// Targets
Vec2 target = Vec2(60, 60);
// create a robot and place it at [0,0]
Robot robot = Robot(eWheelbase, eTrack, eWheelR, enMagnets, eServoAngleDeltaMax, eServoAngleMax, ePistonLowT, ePistonHighT, Vec2(0.0, 0.0) );

void setup() {
    Serial.begin(9600);
    // initialize Servo
    servo.attach(3); 
    testServo(servo, 0, 90, false, 2000);
    // initialize Compass
    Wire.begin();
    compass.init();
    compass.enableDefault();
    compass.writeReg(0x24, 0x74);
    compass.m_min = (LSM303::vector<int16_t>){-32767, -32767, -32767};
    compass.m_max = (LSM303::vector<int16_t>){+32767, +32767, +32767};
    // initialize reed switch
    pinMode(pReedSwitchPin, INPUT_PULLUP);
    currentTime = oldTime;
}

void loop() {
    currentTime = millis();
    // activate the piston
    vSolenoidState = actuatePiston(currentTime, vTimeSolOn, vTimeSolOff);
    digitalWrite(pSolenoidPin, vSolenoidState);  
    // find distance traveled from veh. speed.
    vSwitchStateLast = vSwitchState;
    vSwitchState = digitalRead(pReedSwitchPin);
    if(vSwitchStateLast == 1 && vSwitchState == 0) {
            // store times for each pulse
            vMagnetPassed1 = vMagnetPassed0;
            vMagnetPassed0 = currentTime;
            // vehicle traveled 1/4 of wheel radius
            dr += .5 * M_PI_2 * robot.wheelR;
            // total dist traveled
            r += .5 * M_PI_2 * robot.wheelR;
    }    
    // Control loop
    if( (currentTime - oldTime) >= pLoopPeriod) {
        // update the current heading
        compass.read();
        robot.vCurrentHeading = compass.heading() * M_PI/180;
        // update the robot position
        robot.vCurrentPosition = robot.vCurrentPosition + Vec2(dr, robot.vCurrentHeading).rtheta_to_xy();
        dr = 0;
        if(pPrintInfo) Serial.print(String("p:\t" + robot.vCurrentPosition.info() + "\t"));
        // update the servo angle
        updateServo(target, servo, robot, pKp, pPrintInfo);
        if(pPrintInfo) Serial.print(String("h:\t" + String(robot.vCurrentHeading) + "\ta:\t" + String(servo.read()) + "\n"));
        oldTime = currentTime;
    }
}

// A function to test servo travel.
void testServo(Servo &servo, const int low, const int hig, const bool detailed, const unsigned long period) {
    int servoAngle;
    bool steppingUp;
    Serial.print("Servo test started...\n");
    int iterations = int(float(period)/20.0);
    for(int t = 0; t < iterations; t++) {
        bool steppingUp;
        if(servoAngle <= low) steppingUp = true;
        if(servoAngle >= hig) steppingUp = false;
        if(servoAngle <= hig && steppingUp == true) servoAngle++;
        else servoAngle--;
        servo.write(servoAngle);
        if(detailed) Serial.print(String("Angle:\t" + String(servo.read()) + "\n"));
        delay(20);
    }
    Serial.print("Servo test finished...\n");
}

// steer the servo towards the desired heading.
// ** note. sat stands for Servo Angle Target.
// **      dsat stands for Servo Angle Target Delta.
void updateServo(const Vec2 &target, Servo &servo, const Robot &robot, const float Kp, const bool showinfo) {
    float sat, dsat, targetDist, robotToTargetAngle, servoAngle, currentServoAngle;
    Vec2 robotToTargetVector;
    robotToTargetVector = target - robot.vCurrentPosition;
    targetDist = sqrt( robotToTargetVector.mag2() );
    // position vector from robot to target
    robotToTargetAngle = wrapPi(robot.vCurrentHeading - atan2(robotToTargetVector.y, robotToTargetVector.x));
        if(showinfo) Serial.print("\t" + String(robotToTargetAngle * 180 * M_1_PI,3) + "\t");
    // apply gain
    sat = Kp * robotToTargetAngle;
    sat = sign(sat) * min(robot.servoAngleMax, abs(sat));
        if(showinfo) Serial.print("\t" + String(sat * 180 * M_1_PI) + "\t");
    currentServoAngle = servo.read() * M_PI/180;
    // servo needs to travel this distance
    dsat = sat - currentServoAngle;
    // servo can only turn so fast
    servoAngle = currentServoAngle + sign(dsat) * min(robot.servoAngleDeltaMax, abs(dsat));
        if(showinfo) Serial.print("\t" + String(servoAngle * 180 * M_1_PI) + "\t");
    // make sure we don't go out of bounds
    servoAngle = sign(servoAngle) * min(robot.servoAngleMax, abs(servoAngle));
    servo.write(servoAngle * 180 / M_PI);
}

// method for actuating piston
int actuatePiston(const unsigned long &currentTime, unsigned long &prevTimeOn, unsigned long prevTimeOff) {
    bool pistonState;
    // if it's on we turn it off...
    if(pistonState) {
        if(pistonState && (currentTime - prevTimeOff) > prevTimeOn) {
            pistonState = false;
            prevTimeOn = currentTime;
            return LOW;
        }
    }
    // if it's off we turn it on...
    else {
        if( (currentTime - prevTimeOn) > prevTimeOff) {
            pistonState = true;
            prevTimeOff = currentTime;
            return HIGH;
        }
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