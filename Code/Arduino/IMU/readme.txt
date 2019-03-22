This code works with the teensy 3.6 with 3 MPU 9250 on 3 different i2c Pins with 4.7k pull up resistors. It prints the roll pitch yaw from the Earth->Sensor Transformation to the serial connection to be read by either the serial monitor/Simulink running on C/Simulink running on the Pi.
Its mostly self written but contains parts from https://github.com/arduino-ibraries/MadgwickAHRS , https://github.com/kriswiner/MPU9250 and https://github.com/nox771/i2c_t3

Interesting functionalities:
There is a I2c Scanner method which can be used to check the I2c network for devices (useful for debugging purposes). The code also includes a send2Simulink method which is used to send data to Simulink to be read via its serial read. Serial.print does not work with simulink!! But its necessary for the serial monitor. Outside of this the code should be self explanatory. If not check the sources and githubs provided for more information.

Possible Code Improvements: 
MultiThreading. currently we ask get values from one sensor before we ask the next. This process involves waiting on sensor response->multiple threads to increase efficiency
general code quality(eg enums,std::array/std::vector, etc.)
optimization for speed
EEPROM to save calibration values: currently calibration values are entered directly in the code  
Add some more method especially to access sensor setup from the main. Might be handy 
