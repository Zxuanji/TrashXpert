import grovepi
import time
import json
import requests
from datetime import datetime

# Sensor Ports
PROXIMITY_SENSOR_PORT = 2  # D2
WASTE_LEVEL_SENSOR_PORT = 3  # D3
SERVO_PORT = 6  # A0 (PWM)
LED_PORT = 4  # D4

# Timer variables for bin full condition
FULL_THRESHOLD_DISTANCE = 5  # cm to consider bin full
FULL_HOLD_TIME = 3  # seconds to confirm bin is full                 ######################## CHANGE
full_bin_start_time = None

# Ubuntu Server IP
UBUNTU_IP = "192.168.0.105"
UPLOAD_URL = "http://{}:5000/upload".format(UBUNTU_IP)

last_sent_time = time.time()

def get_distance(sensor_port):
    try:
        return grovepi.ultrasonicRead(sensor_port)
    except IOError:
        return float('inf')

def move_servo_to_angle(port, angle):                              ############################ ADD
    for angle in range(0, 181, 10):
        grovepi.analogWrite(port, angle)
        time.sleep(0.05)
    time.sleep(2)
    for angle in range(180, -1, -10):
        grovepi.analogWrite(port, angle)
        time.sleep(0.05)
    grovepi.analogWrite(port, 0)  # Final stop at 0°
    time.sleep(1)

# Function to initialize servo with a sweep
def initialize_servo_to_real_zero(port):                           ############################ ADD
    print("Initializing servo to real 0° position...")
    for angle in range(0, 181, 10):
        grovepi.analogWrite(port, angle)
        time.sleep(0.05)
    time.sleep(2)
    for angle in range(180, -1, -10):
        grovepi.analogWrite(port, angle)
        time.sleep(0.05)
    grovepi.analogWrite(port, 0)  # Final stop at 0°
    print("✅ Servo set to real 0° position.")
    time.sleep(1)

def open_bin_lid():                                                  ###################### ADD
    print("Opening bin lid...")
    move_servo_to_angle(SERVO_PORT, 90)

def send_waste_data(waste_level):
    data = {
        "device": "Pi3",
        "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "sensor_data": {
            "waste_level": waste_level
        },
    }
    try:
        response = requests.post(UPLOAD_URL, json=data, timeout=5)
        print("✅Sent waste level, Server Response: {}".format(response.json()))
    except Exception as e:
        print("❌ Failed to send waste level JSON: {}".format(e))
        
def send_bin_full_data():
    data = {
        "bin_full": "Bin is full"
    }
    try: 
        response = requests.post(UPLOAD_URL, json=data, timeout=5)
        print("✅Sent bin full, Server Response: {}".format(response.json()))
        
    except Exception as e:
        print("❌ Failed to send bin full JSON: {}".format(e))

try:
    grovepi.pinMode(LED_PORT, "OUTPUT")
    bin_full_notified = False
    
    # Initialize servo at 0°
    initialize_servo_to_real_zero(SERVO_PORT)                  #############################
    
    while True:
        distance_near = get_distance(PROXIMITY_SENSOR_PORT)
        waste_level = get_distance(WASTE_LEVEL_SENSOR_PORT)
        print("Proximity Sensor: {} cm, Trash Level: {} cm".format(distance_near, waste_level))

        if distance_near < 20:
            open_bin_lid()

        current_time = time.time()
        if current_time - last_sent_time >= 30:
            send_waste_data(waste_level)
            last_sent_time = current_time
            
        if waste_level < FULL_THRESHOLD_DISTANCE:
            if full_bin_start_time is None:
                full_bin_start_time = time.time()
            elif time.time() - full_bin_start_time >= FULL_HOLD_TIME:
                grovepi.digitalWrite(LED_PORT, 1)  # Turn LED ON
                if not bin_full_notified:
                    send_bin_full_data()
                    print("Bin is full, sending move command to mobility system.")
                    bin_full_notified = True  # Set flag to True
        else:
            full_bin_start_time = None
            grovepi.digitalWrite(LED_PORT, 0)  # Turn LED OFF
            bin_full_notified = False  # Reset flag when bin is not full

        time.sleep(1)  
        
except KeyboardInterrupt:
    print("\nProgram stopped by user")
    grovepi.analogWrite(SERVO_PORT, 0)  # Reset servo to 0°
