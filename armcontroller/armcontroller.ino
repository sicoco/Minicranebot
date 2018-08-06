#include "MPU6050_tockn.h"
#include <Wire.h>
#include <MsTimer2.h>
#include <AccelStepper.h>
#include <Metro.h>
#include <Servo.h>

#define DEVICE_NAME "Arm"

#define DEFAULT_ELBOW_ANGLE 175
#define DEFAULT_SHOULDER_ANGLE 110
#define DEFAULT_WAIST_YAW_ANGLE 0
#define DEFAULT_GRIPPER_L_ANGLE 180
#define DEFAULT_GRIPPER_R_ANGLE 0

//Arm Control
//Includes waist, shoulder, elbow, counterweight stepper and gripper servos and IMU@arm

String inputString = "";
boolean stringComplete = false;

MPU6050 mpu6050(Wire);

// Define steppers and the pins it will use
AccelStepper shoulderstepper(1, 2, 5);//X-axis DRV8825 Full step
AccelStepper elbowstepper(1, 3, 6); //Y-axis DRV8825 Full step
AccelStepper waistyawstepper(1, 4, 7); //Z-axis A4988  8 microstep
AccelStepper counterweightstepper(1, 12, 13); //A-axis A4988 * 2 8 microstep
Servo gripper_L;
Servo gripper_R;
const int servogripper_L_pin = 9; //X-endstop
const int servogripper_R_pin = 10; //Y-endstop
const int elbow_potentiometer_pin = A0; //Abort
const int shoulder_potentiometer_pin = A1; //Hold
const int counterweight_endstop_pin = 11;//Z-endstop

const int max_shoulder_sp = 600;
const int max_elbow_sp = 500;
const int max_waist_yaw_sp = 1000;
const int max_counterweight_sp = 1000;



Metro spcontrolMetro = Metro(50);//50ms
Metro reportMetro = Metro(1000);//1000ms

int set_waist_yaw_sp;
int set_elbow_sp;
int set_shoulder_sp;

int waist_yaw_sp;
int elbow_sp;
int shoulder_sp;

int waist_yaw_sp_diff;
int elbow_sp_diff;
int shoulder_sp_diff;

int waist_yaw_acc = 100;
int elbow_acc = 100;
int shoulder_acc = 20;

float filter_a = 0.3;

float current_elbow_value;
float current_shoulder_value;
int current_elbow_angle;
int current_shoulder_angle;
int current_waist_yaw_angle;
int current_waist_pitch_angle;
//int current_counterweight_position = 0;

float shoulder_effectivity;
float elbow_effectivity;
float shoulder_stall_threshold = 1000;
float elbow_stall_threshold = 4000;

int set_elbow_angle = DEFAULT_ELBOW_ANGLE;
int set_shoulder_angle = DEFAULT_SHOULDER_ANGLE;
int set_waist_yaw_angle = DEFAULT_WAIST_YAW_ANGLE;
int set_counterweight_position = 0;

int set_gripper_L = DEFAULT_GRIPPER_L_ANGLE;
int set_gripper_R = DEFAULT_GRIPPER_R_ANGLE;

const int max_elbow_angle = 175;
const int min_elbow_angle = 0;
const int max_shoulder_angle = 190;
const int min_shoulder_angle = 75;
const int max_waist_yaw_angle = 180;
const int min_waist_yaw_angle = -180;
const int max_counterweight_range = 45 / 2 / PI / 7 * 8 * 200; //pulses for 47mm range of motion
const int min_counterweight_range = 0;

const int max_gripper_angle = 180;
const int min_gripper_angle = 0;
const int gripper_L_offset = 5;
const int gripper_R_offset = 5;

const int elbow_offset = -110;
const int shoulder_offset = -20;
//const int waist_yaw_offset = 0;
int offset_yaw = 0;

int error_shoulder;
int error_elbow;
float error_waist_yaw;
float error_waist_pitch;

int compensation_waist_pitch;

int kp_elbow = 40;
int kp_shoulder = 40;
int kp_waist_yaw = 100;
int kp_waist_pitch = 100;

int error_width_elbow = 2;
int error_width_shoulder = 1;
int error_width_waist_yaw = 1;
int error_width_waist_pitch = 1;

//const int upper_arm_length = 280;
//const int lower_arm_length = 386;//include gripper length
//Position setpoint = {100, 0, 0};
//Position currentpoint;
//float pos_allow_error_half_width = 10;

void setup()
{
  inputString.reserve(60);
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(50000);
  mpu6050.begin();
  mpu6050.calcGyroOffsets(false);
//mpu6050.setGyroOffsets(-1.57,1.87,-0.31);
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

  waistyawstepper.setEnablePin(8);
  waistyawstepper.setPinsInverted(false, false, true);
  waistyawstepper.setMaxSpeed(max_waist_yaw_sp);
  waistyawstepper.setAcceleration(50);
  waistyawstepper.enableOutputs();

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
  //  Serial.println(F("Counterweight reset position"));
//  resetCounterweightPosition();
  //  Serial.println(F("Counterweight reset done"));

  MsTimer2::set(40, getAngle); // 40ms period
  MsTimer2::start();
  delay(1000);
}

void getAngle()
{
  sei();
  mpu6050.update();
  current_elbow_value = (1 - filter_a) * current_elbow_value + filter_a * analogRead(elbow_potentiometer_pin);
  current_shoulder_value = (1 - filter_a) * current_shoulder_value + filter_a * analogRead(shoulder_potentiometer_pin);
  current_elbow_angle = map(current_elbow_value, 0, 1023, 0, 335) + elbow_offset;
  current_shoulder_angle = map(current_shoulder_value, 0, 1023, 330, 0) + shoulder_offset;

  current_waist_yaw_angle = mpu6050.getAngleZ() + offset_yaw;
  current_waist_pitch_angle = mpu6050.getAngleY();
//  current_counterweight_position = counterweightstepper.currentPosition();
}

//void calccurPos()
//{
//  float radb = (current_elbow_angle + current_shoulder_angle) / 180 * PI;
//  float rada = current_shoulder_angle / 180 * PI;
//
//  currentpoint = {lower_arm_length * cos(radb) + upper_arm_length * cos(rada), 0, lower_arm_length * sin(radb) + upper_arm_length * sin(rada)};
//}
//Position posdiff()
//{
//  return {setpoint.x - currentpoint.x, setpoint.y - currentpoint.y, setpoint.z - currentpoint.z};
//}
void loop()
{
  if (spcontrolMetro.check() == 1)  //50ms
  {
    set_gripper_L = constrain(set_gripper_L, min_gripper_angle + gripper_L_offset, max_gripper_angle + gripper_L_offset);
    set_gripper_R = constrain(set_gripper_R, min_gripper_angle + gripper_R_offset, max_gripper_angle + gripper_R_offset);
    gripper_L.write(set_gripper_L);
    gripper_R.write(set_gripper_R);

    error_shoulder = set_shoulder_angle - current_shoulder_angle;
    error_elbow = set_elbow_angle - current_elbow_angle;
    error_waist_yaw = set_waist_yaw_angle - current_waist_yaw_angle;
    error_waist_pitch = -current_waist_pitch_angle;

    if (abs(error_shoulder) > error_width_shoulder)
    {
      set_shoulder_sp = error_shoulder * kp_shoulder;
    }
    else
    {
      set_shoulder_sp = 0;
    }
    if (abs(error_elbow) > error_width_elbow)
    {
      set_elbow_sp = error_elbow * kp_elbow;
    }
    else
    {
      set_elbow_sp = 0;
    }
    if (abs(error_waist_yaw) > error_width_waist_yaw)
    {
      set_waist_yaw_sp = error_waist_yaw * kp_waist_yaw;
    }
    else
    {
      set_waist_yaw_sp = 0;
    }
    if (abs(error_waist_pitch) > error_width_waist_pitch)
    {
      compensation_waist_pitch = error_waist_pitch * kp_waist_pitch;
    }
    else
    {
      compensation_waist_pitch = 0;
    }

    set_waist_yaw_sp = constrain(set_waist_yaw_sp, -max_waist_yaw_sp, max_waist_yaw_sp);
    set_shoulder_sp = constrain(set_shoulder_sp, -max_shoulder_sp, max_shoulder_sp);
    set_elbow_sp = constrain(set_elbow_sp, -max_elbow_sp, max_shoulder_sp);

    waist_yaw_sp_diff = set_waist_yaw_sp - waist_yaw_sp;
    shoulder_sp_diff = set_shoulder_sp - shoulder_sp;
    elbow_sp_diff = set_elbow_sp - elbow_sp;

    if (waist_yaw_sp_diff > waist_yaw_acc)
    {
      waist_yaw_sp += waist_yaw_acc;
    }
    else if (waist_yaw_sp_diff < (-1)*waist_yaw_acc)
    {
      waist_yaw_sp -= waist_yaw_acc;
    }
    else
    {
      waist_yaw_sp = set_waist_yaw_sp;
    }

    if (shoulder_sp_diff > shoulder_acc)
    {
      shoulder_sp += shoulder_acc;
    }
    else if (shoulder_sp_diff < (-1)*shoulder_acc)
    {
      shoulder_sp -= shoulder_acc;
    }
    else
    {
      shoulder_sp = set_shoulder_sp;
    }

    if (elbow_sp_diff > elbow_acc)
    {
      elbow_sp += elbow_acc;
    }
    else if (elbow_sp_diff < (-1)*elbow_acc)
    {
      elbow_sp -= elbow_acc;
    }
    else
    {
      elbow_sp = set_elbow_sp;
    }

    if (counterweightstepper.targetPosition() -  compensation_waist_pitch >= max_counterweight_range)
    {
      counterweightstepper.moveTo(max_counterweight_range);
    }
    else if (counterweightstepper.targetPosition() -  compensation_waist_pitch <= min_counterweight_range)
    {
      counterweightstepper.moveTo(min_counterweight_range);
    }
    else
    {
      counterweightstepper.move(compensation_waist_pitch);
    }

    shoulderstepper.setSpeed(-shoulder_sp);
    elbowstepper.setSpeed(elbow_sp);
    waistyawstepper.setSpeed(waist_yaw_sp);
  }

  //  if (reportMetro.check() == 1) //1sec
  //  {
  //        Serial.print(current_shoulder_angle);
  //        Serial.print("->");
  //        Serial.print(set_shoulder_angle);
  //        Serial.print("(");
  //        Serial.print(set_shoulder_sp);
  //        Serial.print(") ");
  //        Serial.print(current_elbow_angle);
  //        Serial.print("->");
  //        Serial.print(set_elbow_angle);
  //        Serial.print("(");
  //        Serial.print(set_elbow_sp);
  //        Serial.print(") ");
  //        Serial.print(current_waist_yaw_angle);
  //        Serial.print("->");
  //        Serial.print(set_waist_yaw_angle);
  //        Serial.print("(");
  //        Serial.print(set_waist_yaw_sp);
  //        Serial.print(") ");
  //        Serial.print(current_waist_pitch_angle);
  //        Serial.print("(");
  //        Serial.print(counterweightstepper.targetPosition());
  //        Serial.println(")");
  //  }

  if (stringComplete)
  {
    int arg1, arg2, arg3, arg4, arg5;
    if (sscanf(inputString.c_str(), "A%d %d %d %d %d", &arg1, &arg2, &arg3, &arg4, &arg5) == 5)
    {
      set_shoulder_angle = arg1;
      set_elbow_angle = arg2;
      set_waist_yaw_angle = arg3;
      set_gripper_L = arg4;
      set_gripper_R = arg5;
    }
    else if (sscanf(inputString.c_str(), "A%d %d %d", &arg1, &arg2, &arg3) == 3)
    {
      set_shoulder_angle = arg1;
      set_elbow_angle = arg2;
      set_waist_yaw_angle = arg3;
    }
    else if (sscanf(inputString.c_str(), "G%d %d", &arg4, &arg5) == 2)
    {
      set_gripper_L = arg4;
      set_gripper_R = arg5;
    }
    else if (sscanf(inputString.c_str(), "F%d", &arg1) == 1)
    {
      offset_yaw = arg1;
    }
    else if (inputString == "name?")
    {
      Serial.println(DEVICE_NAME);
    }
    else if (inputString == "pwroff")//motor power off
    {
      elbowstepper.disableOutputs();
    }
    else if (inputString == "pwron")//motor power on
    {
      elbowstepper.enableOutputs();
    }
    else if (inputString == "psrst")//reset pose
    {
//      resetCounterweightPosition();
      set_gripper_L = DEFAULT_GRIPPER_L_ANGLE + gripper_L_offset;
      set_gripper_R = DEFAULT_GRIPPER_R_ANGLE + gripper_R_offset;
      set_elbow_angle = DEFAULT_ELBOW_ANGLE;
      set_shoulder_angle = DEFAULT_SHOULDER_ANGLE;
      set_waist_yaw_angle = DEFAULT_WAIST_YAW_ANGLE;
      // Serial.println("Done");
    }
    else if (inputString == "st") //report current angle status
    {
      Serial.print(current_shoulder_angle);
      Serial.print(" ");
      Serial.print(current_elbow_angle);
      Serial.print(" ");
      Serial.println(current_waist_yaw_angle);
    }
    //    else if (inputString == "w")
    //    {
    //      elbowstepper.move(500);
    //    }
    //    else if (inputString == "s")
    //    {
    //      elbowstepper.move(-500);
    //    }
    //    else if (inputString == "a")
    //    {
    //      shoulderstepper.move(500);
    //    }
    //    else if (inputString == "d")
    //    {
    //      shoulderstepper.move(-500);
    //    }
    //    else if (inputString == "z")
    //    {
    //      waistyawstepper.move(1000);
    //    }
    //    else if (inputString == "x")
    //    {
    //      waistyawstepper.move(-1000);
    //    }
    //    else if (inputString == "c")
    //    {
    //      counterweightstepper.move(1000);
    //      if (counterweightstepper.targetPosition() > max_counterweight_range)
    //      {
    //        counterweightstepper.moveTo(max_counterweight_range);
    //      }
    //    }
    //    else if (inputString == "v")
    //    {
    //      counterweightstepper.move(-1000);
    //      if (counterweightstepper.targetPosition() < min_counterweight_range)
    //      {
    //        counterweightstepper.moveTo(min_counterweight_range);
    //      }
    //    }
    //    else if (inputString == "t")
    //    {
    //      set_gripper_L += 10;
    //    }
    //    else if (inputString == "g")
    //    {
    //      set_gripper_L -= 10;
    //    }
    //    else if (inputString == "y")
    //    {
    //      set_gripper_R += 10;
    //    }
    //    else if (inputString == "h")
    //    {
    //      set_gripper_R -= 10;
    //    }
    else //received any illegal message, stop the motion
    {

      set_elbow_angle = current_elbow_angle;
      set_shoulder_angle = current_shoulder_angle;
      set_waist_yaw_angle = current_waist_yaw_angle;
      //  set_counterweight_position =  current_counterweight_position;
    }

    set_elbow_angle = constrain(set_elbow_angle, min_elbow_angle, max_elbow_angle);
    set_shoulder_angle = constrain(set_shoulder_angle, min_shoulder_angle, max_shoulder_angle);
    set_waist_yaw_angle = constrain(set_waist_yaw_angle, min_waist_yaw_angle, max_waist_yaw_angle);

    stringComplete = false;
    inputString = "";
  }


  shoulderstepper.runSpeed();
  elbowstepper.runSpeed();
  waistyawstepper.runSpeed();
//  counterweightstepper.run();
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

void resetCounterweightPosition()
{
  while (digitalRead(counterweight_endstop_pin))
  {

    digitalWrite(13, HIGH);
    digitalWrite(12, HIGH);
    delay(3);
    digitalWrite(12, LOW);
  }
  counterweightstepper.setCurrentPosition(min_counterweight_range);
}

