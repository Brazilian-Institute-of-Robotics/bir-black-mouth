# Caramel
Caramel is a project of an open-source quadruped robot for research, developed at the Centro Universitário SENAI CIMATEC in partnership with the Centro de Competência em Robótica e Sistemas Autônomos.

**CARAMEL IMAGE**

The robot is entirely teleoperated by a Joystick, and is governed by a state machine.

**STATE MACHINE IMAGE**

### Moving Body
**VIDEO**

### Controlling Body
**VIDEO**

### Walking
**VIDEO**

# How to use Caramel

The robot is configured to automatically connect to a network with the name *caramel* and password *caramel123*. So, you can change the network name and password on your router or use your mobile device's hotspot.

Make sure that your notebook or PC is connected to the same network, and to identify the address where Caramel's RaspberryPi is connected, you can run the command:

```bash
$ sudo nmap -sn $YOUR_IP_ADDRESS$/24
```

Then, connect your computer to the raspberry via SSH. The password is *caramel123*.

```bash
$ ssh ubuntu@$RASPBERRY_IP$
```

In you PC, run:
```bash
$ ros2 run joy joy_node --ros-args -p autorepeat_rate:=0.0
```

In the terminal connected to the Caramel Raspberry, run:
```bash
$ . /rsource.sh
$ ros2 launch caramel_bringup bringup.launch.py
```

You can specify the joystick you are using by including `joy_type=(ps4, x360 or generic)' in the line above.

# Instalation
## Packages contents
The Caramel packages are organized as follow:
- **caramel_bringup:** Package for starting the Caramel robot
- **caramel_control:** Contains the body control and hardware interface
- **caramel_data_analysis:** Package used only for data collection and analysis
- **caramel_description:** Caramel description, URDF and mesh files
- **caramel_gait_planner:** Contains gait trajectory planner and trot gait executor
- **caramel_gazebo:** Package for simulation in Gazebo
- **caramel_kinematics:** Contains quadruped's inverse kinematics
- **caramel_teleop:** Package teleoperation with a joystick 

## Install with Docker
#### Create workspace and build docker image
```bash
$ mkdir -p ~/workspaces/caramel_humble/src/
$ cd ~/workspaces/caramel_humble/src/
$ git clone git@github.com:Brazilian-Institute-of-Robotics/bir-black-mouth.git
$ cd bir-black-mouth
$ . build_docker.sh
```

#### Run docker container
```bash
$ . run_docker.sh
```

#### Inside docker container
```bash
$ source /rsource.sh
```

## Install without Docker
#### Create a workspace and clone the repository
```bash
$ mkdir -p ~/caramel_ws/src/
$ cd ~/caramel_ws/src/
$ git clone git@github.com:Brazilian-Institute-of-Robotics/bir-black-mouth.git
```

#### Build the packages
```bash
$ cd ~/caramel_ws/
$ colcon build
```

# Authors
This project was developed as a Course Completion Work of the Electrical Engineering course. To see the final paper, check [this repository](https://github.com/Brazilian-Institute-of-Robotics/bir-black-mouth-docs/tree/fix/spelling)

**Advisor:** MSc. Paulo Andrade Souza

**Co-advisor:** MSc. Marco Antonio dos Reis

**Authors:** Brenda Silva de Alencar, 
             Felipe Mohr Santos Muniz Barreto, 
             Lucas Lins Souza
