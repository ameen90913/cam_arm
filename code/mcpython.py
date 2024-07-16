import network
import socket
from machine import Pin, PWM
from time import sleep

# Wi-Fi connection details
ssid = 'narzo'
password = '11111111'

# Connect to Wi-Fi
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
print('Connecting to Wi-Fi...')
wlan.connect(ssid, password)

# Debug statement to indicate connection attempt
print(f'Attempting to connect to {ssid}...')

# Wait for connection
while not wlan.isconnected():
    sleep(1)
    print('Waiting for connection...')

# Debug statement to confirm connection
print('Connected to Wi-Fi')

# Check if connected and get IP address
if wlan.isconnected():
    ip = wlan.ifconfig()[0]
    print(f'IP Address: {ip}')
else:
    print('Failed to get IP address')

# Set up PWM for servos
servo_pins = [0, 1, 2, 3, 4]  # GPIO pins connected to the servos
servos = [PWM(Pin(pin)) for pin in servo_pins]
for servo in servos:
    servo.freq(50)  # Typical servo frequency

def set_servo_angle(servo, state):
    if state == 1:
        angle = 100  # Bent position
    else:
        angle = 0  # Straight position
    min_pulse = 1000
    max_pulse = 9000
    pulse_width = min_pulse + (max_pulse - min_pulse) * (angle / 180)
    servo.duty_u16(int(pulse_width))

# Socket setup
addr = socket.getaddrinfo('0.0.0.0', 8080)[0][-1]
s = socket.socket()
s.bind(addr)
s.listen(1)

print('Listening on', addr)

# Main loop
while True:
    cl, addr = s.accept()
    print('Client connected from', addr)
    cl_file = cl.makefile('rwb', 0)
    while True:
        line = cl_file.readline()
        if not line or line == b'\r\n':
            break
        try:
            # Decode and strip newline characters
            data = line.decode().strip()wwwwwww
            print(f"Received raw data: {data}")  # Print the raw data for debugging
            
            # Convert data to a list of integers
            hand_state = list(map(int, data))
            if len(hand_state) == 5:
                print(f"Received hand state: {hand_state}")  # Print the received hand state
                for i, state in enumerate(hand_state):
                    set_servo_angle(servos[i], state)
        except Exception as e:
            print('Error:', e)
    cl.close()