# Cam Arm

## Project Overview
The project aims to develop a real-time hand gesture recognition system that detects and tracks hand movements using Mediapipe and Opencv and communicates the recognized gestures to an Arduino for further actions. It also demonstrates the integration of computer vision for gesture recognition with hardware control based on hand gestures.

![image1](circuit.png)
![image2](landmarks.png)


## HARDWARE:
The Arduino UNO controls the robotic arm by sending command signals to the servo motor driver. 
The servo motor driver interprets these signals and supplies the necessary current to the servo motors, which then deflect the fingers of the arm according to the specified motion. 
Springs facilitate the fingers ‘back-and-forth’ motion, and an SMPS provides the main power supply to the motors.
For efficient working of our robotic arm we have replaced our Arduino UNO by a Raspberry Pi
Raspberry Pi Pico is a low-cost, high-performance microcontroller board with flexible digital interfaces. 

## CONNECTIONS:
● The PWM pins from the Raspberry Pi are connected to the signal ports of the motor driver.
● The servo motors’ pins are also connected to the motor driver with respect to their power, signal, and ground pins.
● The whole circuit is powered using an SMPS which supplies 12V connected to the customized motor driver.
Raspberry pico W helps to enable wireless communication.
Pin configuration

(The pins can be connected according to your choice but they must all be PWM signal pins and corresponding servo motor pins must be connected accordingly.)
| Finger   | PWM Pins   | Motor Driver Pins |
| Index    | 3          |c1                 |
| Ring     |5           |D5                 |
| Thumb    |9           |D4                 |
| Middle   |10          |C0                 |
| Pinky    |11          |D6                 |

## CIRCUIT 

![image1](circuit.png)

## Budget And Resource
| Component              | Price  |
| Raspberry Pi Pico W    | ₹700   |
| MG995 Servo Motor [5]  | ₹1475  |
| Acrylic Sheet          | ₹599   |
| Spring                 | ₹299   | 
| Servo Motor Driver     | ₹700   |
| Jumper Cable           | ₹150   |
| SMPS                   | ₹570   |
# Total Budget :₹4493

# SOFTWARE:
## Hand Tracking and Communication via socket (Python):
The Python script uses OpenCV to capture and process video from a webcam and Mediapioe to detect and track hand landmarks in real time.This script demonstrates the setup for image processing and hardware communication applications using openCV,Mediapipe,and Socket.
● OpenCV: open source computer vision library for computer vision
● Mediapipe: a cross-platform framework for building multimodal.
● Socket : Socket library in Raspberry Pi Pico W are used for network communication
●Raspberry Pi Pico W :  wireless communication using Wifi Module.

## Wi-Fi Controlled Servo Motors using ESP32 And Python Sockets:
The code demonstrates how to connect an ESP32 to a Wi-Fi network, with setting up     a socket server, and  to control servo motors based on commands received over the   network. The software handles the Wi-Fi connection, initializes the PWM for servos,   sets up the socket server, and processes incoming data to control the servos .

=> Set_servo_angle function : maps the angle to the corresponding pulse width and sets the duty cycle for the servo. If the state is 1,then it is bent or it is straight.
=> Socket setup : Bonds the socket to all available network interfaces on port 8080.
=> Handling connections :  Accepts a client connection and prints the client's address.Reads data from  the client connection and processes it line by line.
=> Decodes and splits the received data, and updates the servos based on the received hand state.Handles any exceptions that occur during data processing.
=> Connection to Wi-Fi : specify the SSID and Password for initializing the connection.
=> Set Up PWM for Servos : Initialize the PWM for each servo pin with frequency te.Handles any exceptions that occur during data processing.


## Goals and objectives   
# Precision and efficient:
The robotic arm can be used to achieve unparalleled precision in material handling, sorting, and minimizing operational errors in dynamic and unstructured environments. Additionally, it aims to enhance safety and efficiency by automating complex tasks that traditionally require skilled manual labor.
# Guide and control:
The deployment of robotic systems for crowd management could offer significant advantages. These robots can be strategically positioned to direct crowds efficiently to various areas. This solution is cost-effective compared to employing a large number of police officers, as robots can operate interminably
# Adaptability:
The robotic arm can also include the ability to learn and adapt to new tasks through machine learning algorithms, thus continuously improving its performance over time. Additionally, it aims to provide real-time feedback and diagnostics, enabling proactive maintenance and reducing downtime in industrial operations.


## Key Features
# Real-Time Hand Gesture Recognition:
Utilizes OpenCV and Mediapipe to capture and process video from a webcam, enabling real-time detection and tracking of hand landmarks.
Recognizes and interprets hand gestures instantly, ensuring responsive control of the robotic arm.
# Integration of Computer Vision and Hardware Control:
Combines computer vision techniques for gesture recognition with hardware control via Pico W.
Demonstrates seamless communication between software (Python scripts) and hardware (Raspberry Pi Pico W and servo motors).
# Serial Communication:
Employs Socket for reliable serial communication between the computer and the Pico Board.
Transmits digital data representing hand gestures to the Pico in real-time, enabling precise control of the servo motors.
# Servo Motor Control:
Thonny code receives commands from the Python script and adjusts the servo motors based on the recognized hand gestures.
Five servo motors correspond to the five fingers, allowing for individual finger movement and complex gestures.



