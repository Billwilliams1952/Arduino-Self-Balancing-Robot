# Arduino-Self-Balancing-Robot
 Self balancing robot using the Arduino and an LSM9DSO 9 degrees of freedom sensor
 
## Synopsis

This sketch demonstrates a self-balancing robot.

## Theory of Operation

## Hardware

The following hardware is used in the design:

- The Motor Shield (motor drive interface) is the SMAKN dual motor driver.
- The Sensor is the LSM9DSO 9 degrees of freedom sensor. 3 axis acceleration, three axis gyroscope, 3 axis magnetic field, and temp sensor.
- The Rotary Encoder is a KY-040.
- The switch is a SPST pushbutton.
- The display is a 128x32 OLED display. 
- The motors are 12V motors <TODO: part number>. 
- Wheels are Pololu <TODO: size and part number>.
- The processor is an Arduino Nano.
- The battery is a <TODO: description and part number>.

## API

Several key libraries are used:

- 9DOF Sensor found at:  https://github.com/adafruit/Adafruit_LSM9DS0_Library
- OLED Display found at: https://github.com/adafruit/Adafruit_SSD1306
- Graphic library found at: https://github.com/adafruit/Adafruit-GFX-Library
- Kalman found at: https://github.com/TKJElectronics/KalmanFilter
- PID_V1 found at: https://github.com/br3ttb/Arduino-PID-Library
- KY-040 found at: https://github.com/Billwilliams1952/KY-040-Encoder-Library---Arduino
- LPF found at: https://github.com/Billwilliams1952/Low-Pass-Filter


