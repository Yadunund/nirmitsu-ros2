# Workshop: How do robots move?

![nirmitsu_overview](https://user-images.githubusercontent.com/13482049/188319370-118fe2cb-7392-4363-9d3f-b7c31992d366.png)

## Overview
In this session, we will assemble a simple two-wheeled robot and learn how it moves around. The goal is to develop an intuition of how changing the direction and speeds of individual wheels can steer such a robot around a space. There are two components to this session:
* [Hardware assembly](#hardware-assembly)
* [Software assembly](#software-assembly)
* [Network setup](#network-setup)
* [How to run the session](#session)

## Hardware assembly
![assembly_step_0](https://user-images.githubusercontent.com/13482049/188320468-87b06a8e-ca89-410f-881c-680a3175ddeb.jpg)

Each robot has requires the following components:
* [Chassis (to be 3D printed)](https://cad.onshape.com/documents/54308b759bf3a45674879fa2/w/3fe7569d8381a7214a214808/e/deaf99848626555afa48ef71?renderMode=0&uiState=6314bc778027b126a0aff285)
* [2x DC motor + wheel](https://www.amazon.com/uxcell-Yellow-Black-Single-Geared/dp/B00BG7PFVA)
* [Motor driver (L9110 or L298N)](https://www.elecrow.com/l9110-dualchannel-hbridge-motor-driver-module-12v-800ma-p-826.html)
* [Raspberry Pi (Model 4 and above preferred)](https://www.raspberrypi.com/products/raspberry-pi-4-model-b/)
* [Jumper wires](https://sg.cytron.io/p-female-to-female-jumper-wire)
* Any powerbank
* M3 screws, M3 nuts, M2 screws, thread inserts
Tools required: Allen keys, phillips screw driver, soldering iron

### Step 1: Motors
![image](https://user-images.githubusercontent.com/13482049/188322050-b4433a0e-c5cf-45f9-b6af-1ba13e01b42a.png)
Fix the motors to the chassis with the help of the M3 screws and nuts as shown above. There is a small cylindrical extrustion on the motor that will fit into the hole in center hole. Then push the wheels into the motor shaft.

### Step 2: Electronics
![image](https://user-images.githubusercontent.com/13482049/188323279-d1797512-5ec4-4dcb-865f-6d20f92725e5.png)
Mount the RPi and motor driver to the chassis using M2 screws.

### Step 3: Wiring
![image](https://user-images.githubusercontent.com/13482049/188323321-79af2cee-2f75-4efb-9357-37338d6731e4.png)
TODO

### Step 4: Configuration
The following configuration on the RPi are needed:
1. Install Ubuntu 22.04 + ROS 2 Humble on the RPi
2. Clone this repository `git clone https://github.com/yadunund/nirmitsu-ros2`

## Software assembly
On a laptop, follow insturctions from [here](https://github.com/Yadunund/nirmitsu-ros2#setup) to install the programming interface.
Launch the software with these commands.
```bash
cd ws_nirmitsu
source install/setup.bash
ros2 run nirmitsu_ros2_flowgraph nirmitsu 
```

A grey canvas will open up. If you right click on the canvas, a list of items which can be added will open up. They are grouped into 3 categories: Input, Display and Robot. We will not "assemble" the robot in software following the instructions below.

![image](https://user-images.githubusercontent.com/13482049/188323755-7cc4fb59-7967-46f0-a02d-c7bc7b478f88.png)
![image](https://user-images.githubusercontent.com/13482049/188323777-71d0f11a-773b-41d0-8029-356d631874d3.png)
![image](https://user-images.githubusercontent.com/13482049/188323800-e8d9aa7c-ecdc-4875-a821-2e77082f6c1e.png)
![image](https://user-images.githubusercontent.com/13482049/188323808-0431b5a6-4b47-4724-a5cd-acc680e3e490.png)
![image](https://user-images.githubusercontent.com/13482049/188323817-e0a8629e-0577-44b7-aae5-02e0b9ed12a5.png)
![image](https://user-images.githubusercontent.com/13482049/188323825-13926bb7-1699-4409-abff-7a4bee782372.png)
![image](https://user-images.githubusercontent.com/13482049/188323885-5558247f-7302-4bdf-a29b-223d6dd80f7f.png)

## Network setup
TODO

## Session
After assembling the robot in hardware and software, it's time to learn how the robot moves.
First, move the slider connected to the `Left Wheel` and observe how the speed of rotation changes as you slide along. Click on the `Reverse` switch to flip the direction of rotation. Then try the same with the `Right Wheel`.

### Things to try out:
1. Set the sliders for both wheels to the same value and turn on the robot. What direction does the robot move?
2. Now flip the direction of both wheels. Does the direction of the robot change?
3. What happens when one wheel is not in reverse but the other is?
4. What happens when both wheels have the same direction but one is spinning faster than the other?

Now that we're familair with how each individual wheel should spin to move the robot, let's have some fun. Right click on the canvas and add the `Joystick` input. Then connect the output of the `Joystick` to the `Joystick` input on the robot node as shown below. As you move the joystick around, the robot should follow the same direction and speed. You will notice that the joystick automatically sets the direction and speed of each wheel to follow the requested motion. But in principle, it does exactly what you did in the steps above 🙂

![image](https://user-images.githubusercontent.com/13482049/188324257-b9e2d47d-c80f-45a4-ab9b-c06819211946.png)


