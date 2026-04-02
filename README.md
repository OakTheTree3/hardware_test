## Instruction for building the project
Extract to `{your workspace}/src/`
Build in your workspace using `colcon build`.
Before trying to launch the node remember to `source ./install/setup.bash`!
Make sure that the ports for the RFT Sensor and the Arduino are set correctly. Refer to the ```config/para,s.yaml``` and ```motor.cpp``` for the port setting of the RFT Sensor and the Arduino respectively.

## Instructions for running RFT Sensor
Credit to Robotous for the RFT Sensor code

Set BAUD rate as ros params in ```config/params.yaml```
Only BAUD rate (for connecting only), Force Divider, Torque Divider, dev name is recognised for now

To run the node with the params in config/params.yaml:
```ros2 launch rft_sensor_serial rft_sensor_launch.py```

## Instruction for motor control
The project is assuming that an Arduino is used for lower level control of the motor. The code for the Arduino is also provided in the ```motorController``` directory. 

The PID speed controller for the motor is tuned to the Sha Yang Ye 12V 1400RPM 2kgfcm motor. A different implementation of the lower level control can be used, but refer to the ```arduino_comms.hpp``` and ```cpp_motor.cpp``` files for necessary commands. Credit to joshnewans as the code for ```arduino_comms.hpp``` is adapted from his ```arduino_comms.cpp```.

## Instruction for PID control
Although ```K_P```, ```K_D``` and ```K_I``` are not all in use, the code for all three terms are left in so these values can easily be tuned at a later date depending on the physical setup.
