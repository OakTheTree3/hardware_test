## Instructions for running RFT Sensor
Extract to `{your workspace}/src/`
Build in your workspace using `colcon build`.
Before trying to launch the node remember to `source ./install/setup.bash`!

Set BAUD rate as ros params in config/params.yaml
Only BAUD rate (for connecting only), Force Divider, Torque Divider, dev name is recognised for now
To set these and the Baud rate, use the Windows utility (set the desired baud rate with it before connecting with this node).

To run the node with the params in config/params.yaml:
```ros2 launch rft_sensor_serial rft_sensor_launch.py```
