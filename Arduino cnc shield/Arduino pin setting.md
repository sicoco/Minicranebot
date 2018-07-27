# Arm Controller Arduino pin setting
|Arduino pin|CNC Shield V3 port Name|Connect to|Mode|
|--|--|--|--|
|D0|TX|RPi Serial|UART TX|
|D1|RX|RPi Serial|UART RX|
|D2|X-STEP|shoulder step|Accelstepper step|
|D3|Y-STEP|elbow step|Accelstepper step|
|D4|Z-STEP|waist step|Accepstepper step|
|D5|X-DIR|shoulder dir|Accelstepper dir|
|D6|Y-DIR|elbow dir|Accelstepper dir|
|D7|Z-DIR|waist dir|Accelstepper dir|
|D8|EN|All stepper driver's EN pin|Accelstepper enable|
|D9|X-Endstop|Gripper servo L|PWM|
|D10|Y-Endstop|Gripper servo R|PWM|
|D11|Z-Endstop|counterweight endstop|GPIO input|
|D12|SpnEn|counterweight step(also connects to another CNC shield's A-axis step)|Accelstepper step|
|D13|SpnDir|counterweight dir(also connects to another CNC shield's A-axis dir)|Accelstepper dir|
|A0|Abort|elbow potentiometer|ADC|
|A1|Home|shoulder potentiometer|ADC|
|A2|Resume|||
|A3|CoolEn|||
|A4|SDA|MPU6050 I2C|I2C SDA|
|A5|SCL|MPU6050 I2C|I2C SCL|

E-stop is reset button

Used libraries:
Metro: speed(25ms) and position(100ms) control
MsTimer2: get MPU6050 and potentiometer data and do some filtering to get angles updated(20ms)
Accelstepper: control all stepping motors
MPU6050_tockn: MPU605 driver
Wire: I2C library

3，5，6，9，10，11 pin have PWM function



# Car Controller Arduino pin setting
|Arduino pin|CNC Shield V3 port Name|Connect to|Mode|
|--|--|--|--|
|D0|TX|RPi Serial|UART TX|
|D1|RX|RPi Serial|UART RX|
|D2|X-STEP|Left wheel step|Accelstepper step|
|D3|Y-STEP|Right wheel step|Accelstepper step|
|D4|Z-STEP|||
|D5|X-DIR|Left wheel dir|Accelstepper dir|
|D6|Y-DIR|Right wheel dir|Accelstepper dir|
|D7|Z-DIR|||
|D8|EN|All stepper driver's EN pin(Also controls A-Axis driver's EN pin)|Accelstepper enable|
|D9|X-Endstop|||
|D10|Y-Endstop|||
|D11|Z-Endstop|||
|D12|SpnEn|should not connect to A-Axis as A-Axis driver is controlled by Arm controller Arduino now||
|D13|SpnDir|should not connect to A-Axis as A-Axis driver is controlled by Arm controller Arduino now||
|A0|Abort|||
|A1|Home|||
|A2|Resume|||
|A3|CoolEn|||
|A4|SDA|MPU6050 I2C(hopefully, if wiring can be solved)|I2C SDA|
|A5|SCL|MPU6050 I2C(hopefully, if wiring can be solved)|I2C SCL|


## Problems right now

 1. Servo are in failure: possibly overdriven too much? change new one. First print a new twin servo holder. fixed
 2. Add glue gun on gripper to increase friction. fixed
 3. Is Cooling enough? 
 4. Endstop need to get closer a little to the counterweight. fixed
 5. Why counterweight is not moving in initialization? Using runSpeed wrongly? 
 6. Connect servo L and R correctly 
 7. Waist timing belt getting loose
 8. Cable entangling between car and arm. Make waist a little taller 
 9. Waist plate is bending due to heavy counterweight. Make thicker waist plate.　fixed
 10. Why MPU6050_tockn not working properly? Check thoroughly.  fixed
 11.Complete feedback control on arm 
