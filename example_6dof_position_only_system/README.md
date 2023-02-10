# WIP ros2_control Kuka Driver

## Kuka Driver:
The minimal ros2 fork of kuka_experimental has a, somewhat rough, adaptation of the RSI simulator. It can be used to test the driver by callig:
```
ros2 run kuka_rsi_simulator kuka_rsi_simulator
```

In combination with launching the robot hardware (see https://github.com/ros-controls/ros2_control_demos for details on starting the driver).

# How to connect with a real robot
## Before you start:
Make sure you followed the "[How to use](https://github.com/StoglRobotics-forks/kuka_experimental#installation)" section and installed all necessary dependencies.
## Establish a connection:

**_NOTE:_** The below provided ip addresses and how to turn on the robot are only an example and could differ for yours.

### Laptop:
1. Plugin the wire and add a wired connection as follows:
    Setting->Connections->Add new connection->Wired Ethernet:
    The inside the IPv4 setting set the following:
    ```
    Address: 172.20.19.101
    Netmask: 255.255.255.0
    Gateway:
    ```
=> 172.20.19.101 is ip address of laptop (server)

2. In kuka_6dof_system_position_only.ros2_control.xacro:
    ```
    <hardware>
        <plugin>ros2_control_kuka_driver/KukaSystemPositionOnlyHardware</plugin>
        <param name="listen_address">172.20.19.101</param> # This need to be the ip address of your laptop
        <param name="listen_port">49152</param>
    </hardware>
    ```

### Robot:
Ip address: 172.20.19.5 (if you cannot find it you can search via `nmap -sn 172.20.19.0/255`)

On the robot:
1. Turn on the switch on the electric control box
   + you should then after a few moments be able to ping your robot: `ping 172.20.19.5` which should return some bytes.
2. Switch to automatic mode (Turn small key on control panel and select aut)
3. Turn on power: On control panel press on "O"/"0" and select "I"/"1"

## Execute a program:
1. select program to run (e.g. ros_rsi_101) -> press _Anwählen_
2. Press the run button and then in the terminal start the rsi:
    ```
    ros2 launch kuka_kr16_support kuka_kr16_2.launch.py use_mock_hw:=false
    ```



