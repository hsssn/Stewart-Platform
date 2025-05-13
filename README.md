## Stewart-Platform
A servo actuated stewart platform implementation using 3d printed parts and Arduino.


![Screenshot (81)](https://github.com/user-attachments/assets/653ce345-0adc-4684-9c4c-713abd3bb4f6)

## About
[Stewart Platforms](https://en.wikipedia.org/wiki/Stewart_platform) is a 6DOF parallel manipulator that has its applications in many fields like motion simulation for aircrafts.

# Design
There are various designs with different arrangemetns for the servo motors, most of them work as long as the main principles of the platform are not violated. [This article](https://raw.org/research/inverse-kinematics-of-a-stewart-platform/) shows the possible confugrations, as well as the mathematics behind the linear and rotary actuation. There are also more sources that describe the mathematics, which will be discussed along.
The platform design showed in the implementation is not the best, mechanically, but it works as a first version.



## Parts
- 6 MG995R Servos
- PCA9685 Servo Driver
- Arduino Uno
- Ball Joints
- Metal Rods
- M3 and M4 screws
- 3d printed base and platform, can be found.


## Inverse Kinematics and Maths
For regural platforms using linear actuators, the inverse kinematics process is straight forward vector algebra that calculates the leg lengths needed to reach a specific position and orientation. When adding servo motors, the same lenghts are used and combined with some servo parameters to produce required servo angles. This process is elegantly described in [The Mathematics of The Steart Platform](https://web.archive.org/web/20130506134518/http://www.wokinghamu3a.org.uk/Maths%20of%20the%20Stewart%20Platform%20v5.pdf), and it also describes some basic concepts in robotics like rotation matrices. 


## Code
Before writing the code in Arduino's IDE, it is a good idea to quickly test how everything works. Pyhton was used in this process, and [this repo](https://github.com/Yeok-c/Stewart_Py) by [Yeok-C](https://github.com/Yeok-c) was very helpful in visulaising the result.
The main Arduino code uses [BasicLinearAlgebra](https://github.com/tomstewart89/BasicLinearAlgebra) library, as well as [Adafruit_PWMServoDriver](https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library) library. It starts by defining base and platform points, as well as servo arm length and rod length. Then, the required leg lengths are calculated, and used to get the servo angles. The angles are then converted to PWMs to write and move the servos. 

## Flaws Future Improvemets
The flaws in the cureent implementation are mainly mechanical and related to the design, so another iterative design process is required to eleminate them.
This implementation serves as a base for future work. For now, the desired configuration is achieved by hard-coding it, but any input device like joysticks and potentiometers can be added easiely. 
I plan to get inputs from any flight simulation software and use them to move the platform.
