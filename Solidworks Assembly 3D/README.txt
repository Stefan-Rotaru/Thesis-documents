Roll stabilization lab
==================================

Prerequisites
-------------
• Python 3.12+  
• Arduino IDE  
• Servo.h Arduino Library (https://www.arduino.cc/reference/en/libraries/servo/)

Usage
-----
1. Upload the controller to the Arduino:  
   - Open "PID_Arduino.ino" in the Arduino IDE  
   - Select your board and port  
   - Click Upload

2. Record data to a text file:  
   - python script.py

3. Plot the data :
  - python plots.py --trim= a test_b.txt
  - Replace a by the trim needed
  - Replace b by the test to plot

Configuration
-------------
# In "PID_Arduino.ino" (line 178) tweak "Kp", "Ki", "Kd" to tune your controller.
# On line 157 of "PID_Arduino.ino" you can change the “setpoint” for your target roll angle.
# In "script.py" make sure the serial "port = 'COM3'" matches your board.

Encoder error
-------------
Spikes might occur due to encoder's measurement error