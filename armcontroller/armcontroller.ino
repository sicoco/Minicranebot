#include "MPU6050_tockn.h"
#include <Wire.h>
#include <MsTimer2.h>
#include <AccelStepper.h>
#include <Metro.h>
#include "Position.h"
#include <Servo.h>

#define DEVICE_NAME "Arm"

//Arm Control
//Includes waist, shoulder, elbow, counterweight stepper and gripper servos and IMU@arm

String inputString = "";
boolean stringComplete = false;

MPU6050 mpu6050(Wire);

// Define steppers and the pins it will use
AccelStepper shoulderstepper(1, 2, 5);//X-axis DRV8825 Full step
AccelStepper elbowstepper(1, 3, 6); //Y-axis DRV8825 Full step
AccelStepper waiststepper(1, 4, 7); //Z-axis A4988  16 microstep
AccelStepper counterweightstepper(1, 12, 13); //A-axis A4988 * 2 16 microstep
Servo gripper_L;
Servo gripper_R;
const int servogripper_L_pin = 9; //X-endstop
const int servogripper_R_pin = 10; //Y-endstop
const int elbow_potentiometer_pin = A0; //Abort
const int shoulder_potentiometer_pin = A1; //Hold
const int counterweight_endstop_pin = 11;//Z-endstop

const int max_shoulder_sp = 1000;
const int max_elbow_sp = 1000;
const int max_waist_sp = 1000;
const int max_counterweight_sp = 1000;

Metro spcontrolMetro = Metro(50);//50ms
Metro poscontrolMetro = Metro(200);//200ms

int waist_sp = 500;
int counterweight_sp;
int elbow_sp;
int shoulder_sp;

float elbow_proportion = 1;
float shoulder_proportion = 1;
float waist_proportion = 1;
int elbow_allow_error_half_width = 5;
int shoulder_allow_error_half_width = 5;
int waist_allow_error_half_width = 1;

float filter_a = 0.3;

float current_elbow_value;
float current_shoulder_value;
int current_elbow_angle;
int current_shoulder_angle;
int current_waist_angle;
int current_counterweight_position = 0;

int set_elbow_angle = 90;
int set_shoulder_angle = 90;
int set_waist_angle = 0;
int set_counterweight_position = 0;

int set_gripper_L = 90;
int set_gripper_R = 90;

const int max_elbow_angle = 170;
const int min_elbow_angle = 0;
const int max_shoulder_angle = 190;
const int min_shoulder_angle = 75;
const int max_waist_angle = 180;
const int min_waist_angle = -180;
const int max_counterweight_range = 48;//mm
const int min_counterweight_range = 0;

const int elbow_offset = -110;
const int shoulder_offset = 0;
const int waist_offset = 0;

const int upper_arm_length = 280;
const int lower_arm_length = 386;//include gripper length
Position setpoint = {100, 0, 0};
Position currentpoint;
float pos_allow_error_half_width = 10;

boolean counterweight_init = false;

void setup()
{
  inputString.reserve(20);
  Serial.begin(115200);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  shoulderstepper.setEnablePin(8);
  shoulderstepper.setPinsInverted(false, false, true);
  shoulderstepper.setMaxSpeed(max_shoulder_sp);
  shoulderstepper.setAcceleration(1000);
  shoulderstepper.enableOutputs();

  elbowstepper.setEnablePin(8);
  elbowstepper.setPinsInverted(false, false, true);
  elbowstepper.setMaxSpeed(max_elbow_sp);
  elbowstepper.setAcceleration(1000);
  elbowstepper.enableOutputs();

  waiststepper.setEnablePin(8);
  waiststepper.setPinsInverted(false, false, true);
  waiststepper.setMaxSpeed(max_waist_sp);
  waiststepper.setAcceleration(50);
  waiststepper.enableOutputs();

  counterweightstepper.setEnablePin(8);
  counterweightstepper.setPinsInverted(false, false, true);
  counterweightstepper.setMaxSpeed(max_counterweight_sp);
  counterweightstepper.setAcceleration(50);
  counterweightstepper.enableOutputs();

  pinMode(counterweight_endstop_pin, INPUT);
  gripper_L.attach(servogripper_L_pin);
  gripper_R.attach(servogripper_R_pin);
  gripper_L.write(set_gripper_L);
  gripper_R.write(set_gripper_R);
  //Serial.println("Counterweight reset position");
  //  while (digitalRead(counterweight_endstop_pin))
  //  {
  //    if (counterweight_init = false)
  //    {
  //      counterweightstepper.setSpeed(30);
  //      counterweight_init = true;
  //    }
  //
  //    counterweightstepper.runSpeed();
  //  }
  //  counterweightstepper.stop();
  //Serial.println("Counterweight reset done");

  MsTimer2::set(20, getAngle); // 20ms period
  MsTimer2::start();
}

void getAngle()
{
  current_elbow_value = (1 - filter_a) * current_elbow_value + filter_a * analogRead(elbow_potentiometer_pin);
  current_shoulder_value = (1 - filter_a) * current_shoulder_value + filter_a * analogRead(shoulder_potentiometer_pin);
  current_elbow_angle = map(current_elbow_value, 0, 1023, 0, 295) + elbow_offset;
  current_shoulder_angle = map(current_shoulder_value, 0, 1023, 300, 0) + shoulder_offset;

  current_waist_angle = mpu6050.getAngleZ();
  current_counterweight_position = (float)counterweightstepper.currentPosition() * (2 * PI * 7 / 16 / 200);
}

void calccurPos()
{
  float radb = (current_elbow_angle + current_shoulder_angle) / 180 * PI;
  float rada = current_shoulder_angle / 180 * PI;

  currentpoint = {lower_arm_length * cos(radb) + upper_arm_length * cos(rada), 0, lower_arm_length * sin(radb) + upper_arm_length * sin(rada)};
}
Position posdiff()
{
  return {setpoint.x - currentpoint.x, setpoint.y - currentpoint.y, setpoint.z - currentpoint.z};
}
void loop()
{
  if (spcontrolMetro.check() == 1)
  {
    mpu6050.update();
    //    Serial.println(current_elbow_angle);
    //    shoulderstepper.stop();
    //    elbowstepper.stop();
    //    waiststepper.stop();
  }
  if (poscontrolMetro.check() == 1)
  {
    calccurPos();
    set_gripper_L = constrain(set_gripper_L, 5, 185);
    set_gripper_R = constrain(set_gripper_R, 5, 185);
    gripper_L.write(set_gripper_L);
    gripper_R.write(set_gripper_R);
  }


  if (stringComplete)
  {
    if (inputString == "name?")
    {
      Serial.println(DEVICE_NAME);
    }
    else if (inputString == "power off")
    {
      elbowstepper.disableOutputs();
    }
    else if (inputString == "power on")
    {
      elbowstepper.enableOutputs();
    }
    else if (inputString == "pose reset")
    {
      Serial.println("Done");
    }
    else if (inputString == "w")
    {
      elbowstepper.move(500);
    }
    else if (inputString == "s")
    {
      elbowstepper.move(-500);
    }
    else if (inputString == "a")
    {
      shoulderstepper.move(500);
    }
    else if (inputString == "d")
    {
      shoulderstepper.move(-500);
    }
    else if (inputString == "z")
    {
      waiststepper.move(1000);
    }
    else if (inputString == "x")
    {
      waiststepper.move(-1000);
    }
    else if (inputString == "c")
    {
      counterweightstepper.move(1000);
    }
    else if (inputString == "v")
    {
      counterweightstepper.move(-1000);
    }
    else if (inputString == "t")
    {
      set_gripper_L += 20;
    }
    else if (inputString == "g")
    {
      set_gripper_L -= 20;
    }
    else if (inputString == "y")
    {
      set_gripper_R += 20;
    }
    else if (inputString == "h")
    {
      set_gripper_R -= 20;
    }
    else //received any illegal message, stop the motion
    {

    }
    stringComplete = false;
    inputString = "";
  }


  shoulderstepper.run();
  elbowstepper.run();
  waiststepper.run();
  counterweightstepper.run();
}
void serialEvent()
{
  while (Serial.available())
  {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      stringComplete = true;
    }
    else {
      inputString += c;
    }
  }
}

