import controlP5.*;
import processing.serial.*;
ControlP5 cp5;
Button Left, Right, Forward, Backward, Stop, PWR_ON, PWR_OFF, Pose_Reset;
Knob CarSpKnob, ElbowKnob, ShoulderKnob, WaistYawKnob, GripperKnob;
CallbackListener cb;

Serial arm;
Serial car;

String armreceive = null;
String carreceive = null;
String armsendbuf = null;
String carsendbuf = null;

String get_status = "st";
String arm_pose_reset = "psrst";
String power_on = "pwron";
String power_off = "pwroff";
String identify = "name?";
int lf = '\n';

long lasttime;
long nowtime;
long cycle = 200;

boolean switchserial = false;

float armscale = 0.2;

int car_turn_angle;
int car_sp;
int shoulder_angle;
int elbow_angle;
int waist_yaw_angle;
int gripper_angle;
int current_shoulder_angle, current_elbow_angle, current_waist_yaw_angle, current_gripper_angle;
int armbasex=400,armbasey=300;
boolean carturnleftpressed = false;
boolean carturnrightpressed = false;
boolean carbackwardpressed = false;
boolean carforwardpressed = false;
boolean carstoppressed = false;
void setup()
{
  size(600, 400);
  background(0);

  cp5 = new ControlP5(this);
  Left = cp5.addButton("Left").setLabel("Left").setPosition(0, 70).setSize(50, 19);
  Right = cp5.addButton("Right").setLabel("Right").setPosition(100, 70).setSize(50, 20);
  Forward = cp5.addButton("Forward").setLabel("Forward").setPosition(50, 50).setSize(50, 20);
  Backward = cp5.addButton("Backward").setLabel("Backward").setPosition(50, 90).setSize(50, 20);
  Stop = cp5.addButton("Stop").setLabel("Stop").setPosition(50, 70).setSize(50, 20);
  PWR_ON = cp5.addButton("PWR_ON").setLabel("PWR ON").setPosition(0, 0).setSize(50, 20);
  PWR_OFF = cp5.addButton("PWR_OFF").setLabel("PWR OFF").setPosition(50, 0).setSize(50, 20);
  Pose_Reset = cp5.addButton("Pose_Reset").setLabel("Pose Reset").setPosition(100, 0).setSize(50, 20);
  CarSpKnob = cp5.addKnob("CarSpKnob").setLabel("Car Sp").setRange(0, 1000).setValue(400).setPosition(0, 120).setRadius(50);
  ShoulderKnob = cp5.addKnob("ShoulderKnob").setLabel("Shoulder").setRange(75, 190).setValue(110).setPosition(0, 250).setRadius(25);
  ElbowKnob = cp5.addKnob("ElbowKnob").setLabel("Elbow").setRange(0, 175).setValue(175).setPosition(50, 250).setRadius(25);
  WaistYawKnob = cp5.addKnob("WaistYawKnob").setLabel("Waist").setRange(-170, 170).setValue(0).setPosition(100, 250).setRadius(25);
  GripperKnob = cp5.addKnob("GripperKnob").setLabel("Gripper").setRange(0, 90).setValue(0).setPosition(150, 250).setRadius(25);
  cb = new CallbackListener()
  {
    public void controlEvent(CallbackEvent theEvent)
    {
      switch(theEvent.getAction())
      {
        case(ControlP5.ACTION_ENTER):
        cursor(HAND);
        break;
        case(ControlP5.ACTION_LEAVE):
        case(ControlP5.ACTION_RELEASEDOUTSIDE):
        cursor(ARROW);
        break;
      }
    }
  };
  cp5.addCallback(cb);
  printArray(Serial.list());
  arm = new Serial(this, "/dev/ttyACM0", 115200);
  car = new Serial(this, "/dev/ttyACM1", 115200);
  delay(20000);
  arm.clear();
  car.clear();
  armsendbuf = identify;
  carsendbuf = identify;
  sendcommand();
  delay(100);
  while (arm.available() > 0)
  {
    armreceive = arm.readStringUntil(lf);
  }
  while (car.available() > 0)
  {
    carreceive = car.readStringUntil(lf);
  }
  println(armreceive);
  println(carreceive);
  if (armreceive == "Car\n" || carreceive == "Arm\n")
  {
    switchserial = true;
    println("Swap serial");
  } else
  {
    switchserial = false;
    println("No swap serial");
  }
  armreceive = null;
  carreceive = null;

  lasttime = millis();
}

void draw()
{
  background(#CEE0DD);
  pushStyle();
  stroke(255, 0, 0);
  strokeWeight(5);
  noFill();
  beginShape();
  vertex(armbasex,armbasey);
  vertex(armbasex+280*armscale*cos(radians(-shoulder_angle)), armbasey+280*armscale*sin(radians(-shoulder_angle)));
  vertex(armbasex+280*armscale*cos(radians(-shoulder_angle))+386*armscale*cos(radians(-shoulder_angle-elbow_angle)), armbasey+280*armscale*sin(radians(-shoulder_angle))+386*armscale*sin(radians(-shoulder_angle-elbow_angle)));
  endShape();
  //line(armbasex, armbasey, armbasex+280*armscale*cos(radians(-shoulder_angle)), armbasey+280*armscale*sin(radians(-shoulder_angle)));
  //line(armbasex+280*armscale*cos(radians(-shoulder_angle)), armbasey+280*armscale*sin(radians(-shoulder_angle)), armbasex+280*armscale*cos(radians(-shoulder_angle))+386*armscale*cos(radians(-shoulder_angle-elbow_angle)), armbasey+280*armscale*sin(radians(-shoulder_angle))+386*armscale*sin(radians(-shoulder_angle-elbow_angle)));
  popStyle();
  pushStyle();
  noFill();
  stroke(0, 0, 255);
  strokeWeight(3);
  beginShape();
  vertex(armbasex,armbasey);
  vertex(armbasex+280*armscale*cos(radians(-current_shoulder_angle)), armbasey+280*armscale*sin(radians(-current_shoulder_angle)));
  vertex(armbasex+280*armscale*cos(radians(-current_shoulder_angle))+386*armscale*cos(radians(-current_shoulder_angle-current_elbow_angle)), armbasey+280*armscale*sin(radians(-current_shoulder_angle))+386*armscale*sin(radians(-current_shoulder_angle-current_elbow_angle)));
  endShape();
  //line(armbasex, armbasey, armbasex+280*armscale*cos(radians(-current_shoulder_angle)), 200+280*armscale*sin(radians(-current_shoulder_angle)));
  //line(armbasex+280*armscale*cos(radians(-current_shoulder_angle)), 200+280*armscale*sin(radians(-current_shoulder_angle)), 300+280*armscale*cos(radians(-current_shoulder_angle))+386*armscale*cos(radians(-current_shoulder_angle-current_elbow_angle)), 200+280*armscale*sin(radians(-current_shoulder_angle))+386*armscale*sin(radians(-current_shoulder_angle-current_elbow_angle)));
  popStyle();
  nowtime = millis();
  if (nowtime - lasttime > cycle)
  {
    armsendbuf = get_status;
    carsendbuf = get_status;
    sendcommand();
    armsendbuf = "A"+shoulder_angle + " " + elbow_angle + " " + waist_yaw_angle + " " + (180-gripper_angle) + " " + gripper_angle;
    sendarmcommand();
    lasttime = nowtime;
  }

  while (arm.available() > 0)
  {
    armreceive = arm.readStringUntil(lf);
    if (armreceive != null)
    {
      int[] vals = int(split(armreceive, " "));
      if (vals.length == 3) {
        current_shoulder_angle = vals[0];
        current_elbow_angle = vals[1];
        current_waist_yaw_angle = vals[2];
      }
    }
    //print(armreceive);
  }
  while (car.available() > 0)
  {
    carreceive = car.readStringUntil(lf);
    //print(carreceive);
  }
}

void sendcommand()
{
  if (switchserial == false)
  {
    arm.write(armsendbuf);
    arm.write(lf);
    car.write(carsendbuf);
    car.write(lf);
  } else
  {
    arm.write(carsendbuf);
    arm.write(lf);
    car.write(armsendbuf);
    car.write(lf);
  }
}
void sendcarcommand()
{
  if (switchserial == false)
  {

    car.write(carsendbuf);
    car.write(lf);
  } else
  {
    arm.write(carsendbuf);
    arm.write(lf);
  }
}
void sendarmcommand()
{
  if (switchserial == false)
  {

    arm.write(armsendbuf);
    arm.write(lf);
  } else
  {
    car.write(armsendbuf);
    car.write(lf);
  }
}
public void Left()
{
  car_turn_angle = 20;
  //car_sp = 500;
  carsendbuf = "R"+ car_turn_angle + " " + car_sp;
  sendcarcommand();
}
public void Right()
{
  car_turn_angle = -20;
  //car_sp = 500;
  carsendbuf = "R" + car_turn_angle + " " + car_sp;
  sendcarcommand();
}
public void Forward()
{
  //car_sp = 500;
  carsendbuf = "T" + car_sp;
  sendcarcommand();
}
public void Backward()
{
  //car_sp = -500;
  carsendbuf = "T" + (-1)*car_sp;
  sendcarcommand();
}
public void Stop()
{
  car_turn_angle = 0;
  carsendbuf = "T" + 0;
  sendcarcommand();
}

public void PWR_ON()
{
  armsendbuf = power_on;
  carsendbuf = power_on;
  sendcommand();
}
public void PWR_OFF()
{
  armsendbuf = power_off;
  carsendbuf = power_off;
  sendcommand();
}

public void Pose_Reset()
{
  armsendbuf = arm_pose_reset;
  sendarmcommand();
}

public void CarSpKnob(int theValue)
{
  car_sp = theValue;
}
public void ShoulderKnob(int theValue)
{
  shoulder_angle= theValue;
}
public void ElbowKnob(int theValue)
{
  elbow_angle = theValue;
}
public void WaistYawKnob(int theValue)
{
  waist_yaw_angle = theValue;
}
public void GripperKnob(int theValue)
{
  gripper_angle = theValue;
}

void keyReleased()
{
  if (key==CODED)
  {
    if (keyCode == UP)
    {
      Forward();
    } else if (keyCode == DOWN)
    {
      Backward();
    } else if (keyCode == LEFT)
    {
      Left();
    } else if (keyCode == RIGHT)
    {
      Right();
    }
  } else if (key == ENTER)
  {
    Stop();
  }
}
//void mousePressed()
//{
//  if (mouseX>300 && mouseY < 200)
//  {
//  }
//}
