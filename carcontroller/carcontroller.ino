#include <MPU6050_tockn.h>
#include <MsTimer2.h>
#include <AccelStepper.h>
#include <Metro.h>

#define DEVICE_NAME "Car"

//Car Control
//Includes left and right wheel stepper and MPU6050

MPU6050 mpu6050(Wire);

String inputString = "";
boolean stringComplete = false;

// Define steppers and the pins it will use
AccelStepper leftstepper(1, 2, 5);//X-axis DRV8825 16 microstep
AccelStepper rightstepper(1, 3, 6); //Y-axis DRV8825 16 microstep

const int max_sp = 1000;

Metro spcontrolMetro = Metro(25);//25ms

int left_sp;
int right_sp;

int set_left_speed = 0;
int set_right_speed = 0;
int set_speed = 0;
int acceleration = 300;

float current_yaw = 0;
float set_yaw = 0;
float yaw_error_width = 1.0;
float kp = 1.0;

void setup()
{
  inputString.reserve(15);
  Serial.begin(115200);

  Wire.begin();
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
  mpu6050.update();
  current_yaw = mpu6050.getAngleZ();
}


void loop()
{
  if (spcontrolMetro.check() == 1)
  {
    float error = set_yaw - current_yaw;
    float speed_compensation = 0;
    if (error > yaw_error_width || error < -yaw_error_width) //need compensation for yaw angle;
    {
      speed_compensation = error * kp;
    }
  }
  if (stringComplete == true)
  {
    float argyaw;
    int argsp;
    if (sscanf(inputString.c_str(), "T%d", &argsp) == 1)//go forward or backward at certain speed. Yaw compensation will keep the same yaw angle while running
    {
      set_yaw = current_yaw;
      set_speed = argsp;
    }
    else if (sscanf(inputString.c_str(), "R%f %d", &argyaw, &argsp) == 2)//rotate to certain yaw angle with certain speed
    {
      set_yaw = argyaw;
      set_speed = argsp;
    }
    else if (sscanf(inputString.c_str(), "R%f", &argyaw, &argsp) == 1)//rotate to certain yaw angle with constant speed
    {
      set_yaw = argyaw;
      set_speed = 100;
    }
    else if (inputString == "name?")
    {
      Serial.println(DEVICE_NAME);
    }
    else //received any illegal message, stop the motion
    {
      set_yaw = current_yaw;
      set_speed = 0;
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

