# Cam Arm Project

## Project Overview

The Cam Arm project aims to develop a real-time hand gesture recognition system using Mediapipe and OpenCV. This system detects and tracks hand movements, recognizing various gestures and communicating them wirelessly to a Raspberry Pi Pico W. The Pico W processes these gestures to control a robotic arm, demonstrating the integration of computer vision for gesture recognition with hardware control.

![Hand Landmarks](landmarks.png)

## Hardware Components

### Raspberry Pi Pico W
- Controls the robotic arm by sending command signals to the servo motor driver.
- Uses sockets for wireless communication.
- Low-cost, high-performance microcontroller board with flexible digital interfaces.

### Servo Motor Driver
- Interprets signals from the Raspberry Pi Pico W.
- Supplies necessary current to the servo motors.
- Controls the deflection of the robotic arm's fingers.

### Servo Motors
- MG995 Servo Motors [5 units]
- Connect to the motor driver for finger movement.

### Power Supply
- LiPo Battery 12V 2200mAh.
- Provides main power supply to the motors.

### Additional Components
- Acrylic Sheet
- Springs (for finger back-and-forth motion)
- Jumper Cables

### Hardware Connections
- PWM pins from the Raspberry Pi are connected to the signal ports of the motor driver.
- Servo motor pins are connected to the motor driver (power, signal, ground).
- The circuit is powered by a 12V LiPo Battery.

### Pin Configuration

| Finger   | PWM Pin | Motor Driver Pin |
|----------|---------|------------------|
| Index    | 3       | C1               |
| Ring     | 5       | D5               |
| Thumb    | 9       | D4               |
| Middle   | 10      | C0               |
| Pinky    | 11      | D6               |

## Circuit Diagram



## Budget and Resources

| Component                | Price (₹)   |
|--------------------------|------------ |
| Raspberry Pi Pico W      | ₹700        |
| MG995 Servo Motor [5]    | ₹1000       |
| Acrylic Sheet            | ₹599        |
| Spring                   | ₹299        |
| Servo Motor Driver       | ₹500        |
| Jumper Cable             | ₹150        |
| LiPo Battery 12V 2200mAh | ₹570        |
    ##Total Budget : ₹3818   

## Software Components

### Hand Tracking and Communication (Python)

The Python script captures and processes video from a webcam using OpenCV, detects and tracks hand landmarks using Mediapipe, and communicates the recognized gestures via sockets to the Raspberry Pi Pico W.

#### Key Libraries
- OpenCV: Open source computer vision library.
- Mediapipe: Cross-platform framework for building multimodal machine learning applications.
- Socket: Network communication library for Raspberry Pi Pico W.

### Python Code

python
import cv2
import mediapipe as mp
import math
import socket

def calculate_angle(p1, p2, p3):
    angle_rad = math.atan2(p3[1] - p2[1], p3[0] - p2[0]) - math.atan2(p1[1] - p2[1], p1[0] - p2[0])
    angle_deg = math.degrees(angle_rad)
    return angle_deg + 360 if angle_deg < 0 else angle_deg

def update_hand_state(thumb_bent, index_bent, middle_bent, ring_bent, pinky_bent):
    hand_state = [int(thumb_bent), int(index_bent), int(middle_bent), int(ring_bent), int(pinky_bent)]
    return hand_state

host = '192.168.253.40'
port = 8080
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((host, port))

mp_hands = mp.solutions.hands
hands = mp_hands.Hands(static_image_mode=False, max_num_hands=1, min_detection_confidence=0.5, min_tracking_confidence=0.5)
mp_drawing = mp.solutions.drawing_utils

cap = cv2.VideoCapture(0)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        print("Failed to read from camera.")
        break

    height, width, _ = frame.shape
    box_left, box_top, box_right, box_bottom = width // 4, height // 4, 3 * width // 4, 3 * height // 4

    cv2.rectangle(frame, (box_left, box_top), (box_right, box_bottom), (255, 0, 0), 2)
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = hands.process(frame_rgb)

    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            landmarks = [(int(lm.x * width), int(lm.y * height)) for lm in hand_landmarks.landmark]
            hand_x, hand_y = landmarks[0]

            if box_left < hand_x < box_right and box_top < hand_y < box_bottom:
                mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

                thumb_bent = landmarks[4][0] >= landmarks[3][0]
                index_bent = landmarks[8][1] < landmarks[6][1]
                middle_bent = landmarks[12][1] < landmarks[10][1]
                ring_bent = landmarks[16][1] < landmarks[14][1]
                pinky_bent = landmarks[20][1] < landmarks[18][1]

                hand_state = update_hand_state(thumb_bent, index_bent, middle_bent, ring_bent, pinky_bent)
                print("Hand State:", hand_state)

                hand_state_str = ''.join(map(str, hand_state)) + '\n'
                client_socket.send(hand_state_str.encode('utf-8'))

                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(frame, f'Thumb: {"Straight" if thumb_bent else "Bent"}', (10, 30), font, 0.7, (0, 255, 0), 2, cv2.LINE_AA)
                cv2.putText(frame, f'Index: {"Straight" if index_bent else "Bent"}', (10, 60), font, 0.7, (0, 255, 0), 2, cv2.LINE_AA)
                cv2.putText(frame, f'Middle: {"Straight" if middle_bent else "Bent"}', (10, 90), font, 0.7, (0, 255, 0), 2, cv2.LINE_AA)
                cv2.putText(frame, f'Ring: {"Straight" if ring_bent else "Bent"}', (10, 120), font, 0.7, (0, 255, 0), 2, cv2.LINE_AA)
                cv2.putText(frame, f'Pinky: {"Straight" if pinky_bent else "Bent"}', (10, 150), font, 0.7, (0, 255, 0), 2, cv2.LINE_AA)
                cv2.putText(frame, f'Hand State: {hand_state}', (10, 180), font, 0.7, (0, 255, 255), 2, cv2.LINE_AA)

    cv2.imshow('Hand Tracking', frame)

    if cv2.waitKey(5) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
client_socket.close()


## Wi-Fi Controlled Servo Motors Using ESP32 and Python Sockets

The code below demonstrates how to connect an ESP32 to a Wi-Fi network, set up a socket server, and control servo motors based on commands received over the network.

### Key Functions
- set_servo_angle: Maps the angle to the corresponding pulse width and sets the duty cycle for the servo.
- Socket setup: Binds the socket to all available network interfaces on port 8080.
- Handling connections: Accepts a client connection and processes incoming data to control the servos.
- Wi-Fi connection: Connects to the specified SSID and password.
- PWM for servos: Initializes the PWM for each servo pin.

### Raspberry Pi Pico W Code (Thonny)

python
import network
import socket
from machine import Pin, PWM
from time import sleep

ssid = 'Sow'
password = '11111111'

wlan = network.WLAN(network.STA_IF)
wlan.active(True)
print('Connecting to Wi-Fi...')
wlan.connect(ssid, password)
print(f'Attempting to connect to {ssid}...')

while not wlan.isconnected():
    sleep(1)
    print('Waiting for connection...')

print('Connected to Wi-Fi')

if wlan.isconnected():
    ip = wlan.ifconfig()[0]
    print(f'IP Address: {ip}')
else:
    print('Failed to get IP address')

servo_pins = [0, 1, 2, 3, 4]
servos = [PWM(Pin(pin)) for pin in servo_pins]
for servo in servos:
    servo.freq(50)

def set_servo_angle(servo, state):
    angle = 100 if state == 1 else 0
    min_pulse = 1000


    max_pulse = 9000
    pulse_width = min_pulse + (angle / 180.0) * (max_pulse - min_pulse)
    duty_cycle = int(pulse_width / 20000.0 * 1023)
    servo.duty(duty_cycle)

host = '0.0.0.0'
port = 8080

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
    server_socket.bind((host, port))
    server_socket.listen(1)
    print(f'Server listening on {host}:{port}')

    client_socket, client_address = server_socket.accept()
    print(f'Client connected: {client_address}')

    while True:
        data = client_socket.recv(1024).decode('utf-8').strip()
        if data:
            print(f'Received data: {data}')
            for i, state in enumerate(data):
                set_servo_angle(servos[i], int(state))


### Pin Configuration for Servo Motors

| Servo Motor | Raspberry Pi Pico Pin |
|-------------|-----------------------|
| Servo 1     | GPIO 0                |
| Servo 2     | GPIO 1                |
| Servo 3     | GPIO 2                |
| Servo 4     | GPIO 3                |
| Servo 5     | GPIO 4                |

## Implementation and Testing
- The code was developed and tested to ensure reliable real-time communication between the hand gesture recognition system and the Raspberry Pi Pico W.
- Various gestures were successfully recognized and transmitted to control the robotic arm.

## Challenges and Future Work
- Challenges: Ensuring low-latency communication and accurate gesture recognition in various lighting conditions.
- Future Work: Enhancing the gesture recognition system to support more complex gestures and integrating additional functionalities into the robotic arm.

## Conclusion
The Cam Arm project successfully demonstrates the feasibility of real-time hand gesture recognition and wireless communication for robotic arm control. This integration of computer vision and hardware control showcases the potential for developing more advanced and interactive robotic systems in the future.
