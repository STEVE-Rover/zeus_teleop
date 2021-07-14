# steve_teleop
This package is for teleoperating the rover.

## teleop_gamepad
For controlling the rover with a logitech gamepad.

### How to use
1. Setup your gamepad with [this guide](http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick)
2. In `launch/teleop_gamepad.launch` change `js0` for your gamepad ID
3.
    ```bash
    roslaunch steve_teleop teleop_gamepad.launch
    ```

### Button mapping
| Button       | action |
| ---          |:---:   |
|Left joystick | Move forward / backward and turn |
|RB            | Deadman switch |
|A             | Turbo |


### Subscribed topics
* joy (sensor_msgs/Joy): Gamepad joy message.
### Published topics
* cmd_vel (geometry_msgs/Twist): output velocity command for the mobile base.

### Parameters
* ~linear_vel_axis (int, default: 1): axis number for linear velocity.
* ~angular_vel_axis (int, default: 0): axis number for angular velocity.
* ~deadman_switch (int, default: 5): button number for the deadman switch.
* ~turbo_button (int, default: 1): button number for the turbo.
* ~max_linear_vel (float, default: 1.5): max linear velocity (m/s) that can be output.
* ~max_angular_vel (float, default: 1): max angular velocity (rad/s) that can be output.
* ~turbo_max_linear_vel (float, default: 1.5): max linear velocity (m/s) that can be output when in turbo mode.
* ~turbo_max_angular_vel (float, default: 1): max angular velocity (rad/s) that can be output when in turbo mode.

## zero_twist_publisher
Publishes a twist message set to zero at a constant rate.

### Published topics
* zero_twist (geometry_msgs/Twist)

### Parameters
* rate (float, default 10): Publishing rate

