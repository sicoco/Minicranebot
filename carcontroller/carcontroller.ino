#include <MsTimer2.h>
#include <AccelStepper.h>
#include <Metro.h>

#define DEVICE_NAME "Car"

//Car Control
//Includes left and right wheel stepper

// Define steppers and the pins it will use
AccelStepper leftstepper(1, 2, 5);//X-axis DRV8825 16 microstep
AccelStepper rightstepper(1, 3, 6); //Y-axis DRV8825 16 microstep

const int max_sp = 1000;

Metro spcontrolMetro = Metro(25);//25ms
Metro poscontrolMetro = Metro(200);//200ms

int left_sp;
int right_sp;

int set_left_speed = 100;
int set_right_speed = 100;
void setup()
{
  Serial.begin(115200);
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
}

void calccurPos()
{
}

void loop()
{
  if (spcontrolMetro.check() == 1)
  {
  }
  if (poscontrolMetro.check() == 1)
  {
    calccurPos();
  }

  leftstepper.run();
  rightstepper.run();
}
void serialEvent()
{
  while (Serial.available())
  {
    char c = Serial.read();
    switch (c)
    {
      case 'w':
        leftstepper.move(1500);
        rightstepper.move(1500);
        break;
      case 's':
        leftstepper.move(-1500);
        rightstepper.move(-1500);
        break;
      case 'a':
        leftstepper.move(-1500);
        rightstepper.move(1500);
        break;
      case 'd':
        leftstepper.move(1500);
        rightstepper.move(-1500);
        break;
      case '\n':
        break;
      case '\r':
        break;
      default:
        break;
    }
  }
}

