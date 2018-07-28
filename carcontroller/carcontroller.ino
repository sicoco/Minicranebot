#include <MsTimer2.h>
#include "MPU6050_tockn.h"
#include <Wire.h>
#include <AccelStepper.h>
#include <Metro.h>

#define DEVICE_NAME "Car"

//Car Control
//Includes left and right wheel stepper
//MPU6050 not used due to not enough processing speed to do both accelstepper and MPU6050 feedback

MPU6050 mpu6050(Wire);

String inputString = "";
boolean stringComplete = false;

// Define steppers and the pins it will use
AccelStepper leftstepper(1, 2, 5);//X-axis DRV8825 8 microstep
AccelStepper rightstepper(1, 3, 6); //Y-axis DRV8825 8 microstep

const int max_sp = 1000;

Metro spcontrolMetro = Metro(50);//50ms

int left_sp;
int right_sp;

int set_left_speed = 0;
int set_right_speed = 0;
int set_speed = 0;
int acceleration = 100;
boolean rotation = false;

volatile float current_yaw = 0;
float set_yaw = 0;
float yaw_error_width = 2.0;
float kp = 20.0;

void setup()
{
  inputString.reserve(15);
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  // Change these to suit your stepper if you want
  leftstepper.setEnablePin(8);
  leftstepper.setPinsInverted(false, false, true);
  leftstepper.setMaxSpeed(max_sp);
  leftstepper.setAcceleration(300);
  leftstepper.enableOutputs();

  rightstepper.setEnablePin(8);
  rightstepper.setPinsInverted(false, false, true);
  rightstepper.setMaxSpeed(max_sp);
  rightstepper.setAcceleration(300);
  rightstepper.enableOutputs();

  MsTimer2::set(20, getAngle); // 20ms period
  MsTimer2::start();
}

void getAngle()
{
  current_yaw = mpu6050.getAngleZ();
}

void loop()
{
  if (spcontrolMetro.check() == 1)
  {
    mpu6050.update();
    float error = set_yaw - current_yaw;
    float speed_compensation = 0;
    if (error > yaw_error_width || error < -yaw_error_width) //need compensation for yaw angle;
    {
      speed_compensation = error * kp;
    }
    if (!rotation)
    {
      set_left_speed = set_speed;
      set_right_speed = set_speed;
    }
    else
    {

      set_left_speed = set_speed;
      set_right_speed = (-1) * set_speed;

    }

    set_left_speed -= speed_compensation;
    set_right_speed += speed_compensation;


    int left_sp_diff = set_left_speed - left_sp;
    int right_sp_diff = set_right_speed - right_sp;

    if (left_sp_diff > acceleration)
    {
      left_sp += acceleration;
    }
    else if (left_sp_diff < (-1)*acceleration)
    {
      left_sp -= acceleration;
    }
    else
    {
      left_sp = set_left_speed;
    }

    if (right_sp_diff > acceleration)
    {
      right_sp += acceleration;
    }
    else if (right_sp_diff < (-1)*acceleration)
    {
      right_sp -= acceleration;
    }
    else
    {
      right_sp = set_right_speed;
    }

    leftstepper.setSpeed(left_sp);
    rightstepper.setSpeed(right_sp);
  }
  if (stringComplete)
  {
    int argyaw;
    int argsp;
    if (sscanf(inputString.c_str(), "T%d", &argsp) == 1)//go forward or backward at certain speed. Yaw compensation will keep the same yaw angle while running
    {
      set_yaw = current_yaw;
      if (argsp < max_sp && argsp > (-1) * max_sp)
      {
        set_speed = argsp;
      }
      else if (argsp >= max_sp)
      {
        set_speed = max_sp;
      }
      else if (argsp <= (-1) * max_sp)
      {
        set_speed = (-1) * max_sp;
      }
      rotation = false;
    }
    else if (sscanf(inputString.c_str(), "R%d %d", &argyaw, &argsp) == 2)//rotate with certain speed
    {
      set_yaw += (float)argyaw;
      if (argsp < max_sp && argsp > (-1) * max_sp)
      {
        set_speed = argsp;
      }
      else if (argsp >= max_sp)
      {
        set_speed = max_sp;
      }
      else if (argsp <= (-1) * max_sp)
      {
        set_speed = (-1) * max_sp;
      }
      rotation = true;
    }

    else if (inputString == "name?")
    {
      Serial.println(DEVICE_NAME);
    }
    else if (inputString == "power off")
    {
      leftstepper.disableOutputs();
    }
    else if (inputString == "power on")
    {
      leftstepper.enableOutputs();
    }
    else //received any illegal message, stop the motion
    {
      set_yaw = current_yaw;
      set_speed = 0;
      rotation = false;
    }

    stringComplete = false;
    inputString = "";
  }

  leftstepper.runSpeed();
  rightstepper.runSpeed();
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

