# **Oreo**

Source code for control of the 3-wheeled omnidirectional robot. Current features include command execution and closed loop motor velocity control. 


## Frequent commands

### Run keyboard node and convert to robot commands

```
cd ~/Oreo
ros2 run gogo_keyboard ros_node & ros2 run motor_control keyboard_listener.py
```


| Key | Command |
| :-------- | --------: |
| W | Forward |
| A | Left |
| S | Backward |
| D | Right |
| Right | Rotate CW |
| Left | Rotate CCW |
| Space | Brake |
| Up | Increase velocity |
| Down | Decrease velocity |
| 1 | Test motor 1 |
| 2 | Test motor 2 |
| 3 | Test motor 3 |


### Read motor encoders

Local:

```
ssh daniel@pi.local
```

Pi:

```
cd ~/Oreo
./ros shell
ros2 run motor_control motor_state.py
```


### Start motor control

Local:

```
ssh daniel@pi.local
```

Pi:

```
cd ~/Oreo
./ros shell
ros2 run motor_control motor_control.py
```


### Plot estimated motor states

```
cd ~/Oreo
ros2 run motor_control joint_state_plotter.py
```