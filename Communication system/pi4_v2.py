import grovepi
import time
import json
import os
import requests
from datetime import datetime
from camera_detection import detect_object  # Import from camera script
from picamera2 import Picamera2  # ✅ Import Picamera2 for initialization

# Ubuntu 服务器 IP（替换为你的 Ubuntu 局域网 IP）
UBUNTU_IP = "192.168.0.105"
UPLOAD_URL = "http://{}:5000/upload".format(UBUNTU_IP)

# Grove Pi+ Port Assignments
ULTRASONIC_PAPER = 2  # D2
ULTRASONIC_PLASTIC = 8  # D3
ULTRASONIC_METAL = 4  # D4
ULTRASONIC_TRIGGER = 7  # D7 (New ultrasonic sensor to activate PiCam)

SERVO_PAPER = 5  # D5
SERVO_PLASTIC = 6  # D6
SERVO_METAL = 3  # D7

LED_PAPER_FULL = 14  # A0
LED_PLASTIC_FULL = 15  # A1
LED_METAL_FULL = 16  # A2

# Bin full detection settings
FULL_THRESHOLD_DISTANCE = 10  # cm (consider bin full if distance < 10 cm from experiment)
FULL_HOLD_TIME = 3  # Bin must be full for 3 seconds (determined from experiment)
TRIGGER_THRESHOLD_DISTANCE = 15  # cm (Activate camera when an object is detected within 15 cm)
full_bin_start_time = {"paper": None, "plastic": None, "metal": None}

# Function to measure distance using a Grove ultrasonic sensor
def get_distance(sensor_port):
    try:
        return grovepi.ultrasonicRead(sensor_port)
    except IOError:
        return float('inf')  # Return an invalid reading on error

# Function to move servo to a specific angle
def move_servo_to_angle(port, angle):
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
def initialize_servo_to_real_zero(port):
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

# Function to send bin level data
def send_three_bin_data(levels):
    data = {
        "device": "Pi4",
        "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "sensor_data": {
            "paper_level": levels["paper"],
            "plastic_level": levels["plastic"],
            "metal_level": levels["metal"],
        },
    }

    try:
        response = requests.post(UPLOAD_URL, json=data, timeout=5)
        print("✅Sent waste level, Server Response: {}".format(response.json()))
    except Exception as e:
        print("❌ Failed to send waste level JSON: {}".format(e))

# Function to send bin full notification
def send_bin_full_data(bin_type):
    data = {
        "bin_full": bin_type + " Bin is full"
    }
    try:
        response = requests.post(UPLOAD_URL, json=data, timeout=5)
        print("✅Sent bin full, Server Response: {}".format(response.json()))
    except Exception as e:
        print("❌ Failed to send bin full JSON: {}".format(e))

# ✅ Initialize Picamera2 outside the loop and pass it to detect_object
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (320, 240), "format": "RGB888"})
picam2.configure(config)
picam2.start()
time.sleep(2)

grovepi.pinMode(ULTRASONIC_TRIGGER, "INPUT")  # Set ultrasonic trigger sensor as input

try:
    bin_full_notified = False
    grovepi.pinMode(LED_PAPER_FULL, "OUTPUT")
    grovepi.pinMode(LED_PLASTIC_FULL, "OUTPUT")
    grovepi.pinMode(LED_METAL_FULL, "OUTPUT")
    grovepi.pinMode(SERVO_PAPER, "OUTPUT")
    grovepi.pinMode(SERVO_PLASTIC, "OUTPUT")
    grovepi.pinMode(SERVO_METAL, "OUTPUT")

    # Initialize all servos
    initialize_servo_to_real_zero(SERVO_PAPER)
    initialize_servo_to_real_zero(SERVO_PLASTIC)
    initialize_servo_to_real_zero(SERVO_METAL)

    while True:
        # Check if an object is detected near the trigger ultrasonic sensor
        if get_distance(ULTRASONIC_TRIGGER) < TRIGGER_THRESHOLD_DISTANCE:
            detected_class = detect_object(picam2)
        else:
            detected_class = None

        if detected_class:
            print(f"Detected: {detected_class}")
            if detected_class.lower() in ["paper", "plastic", "metal"]:
                servo_port = SERVO_PAPER if detected_class.lower() == "paper" else SERVO_PLASTIC if detected_class.lower() == "plastic" else SERVO_METAL
                move_servo_to_angle(servo_port, 180)
                time.sleep(2)
                move_servo_to_angle(servo_port, 0)
            else:
                print("Unknown material, use general waste bin.")
        else:
            print("No object detected.")

        trash_levels = {
            "paper": get_distance(ULTRASONIC_PAPER),
            "plastic": get_distance(ULTRASONIC_PLASTIC),
            "metal": get_distance(ULTRASONIC_METAL)
        }

        print("Trash Levels - Paper: {} cm, Plastic: {} cm, Metal: {} cm".format(trash_levels['paper'], trash_levels['plastic'], trash_levels['metal']))
        send_three_bin_data(trash_levels)

        for bin_type in ["paper", "plastic", "metal"]:
            if trash_levels[bin_type] < FULL_THRESHOLD_DISTANCE:
                if full_bin_start_time[bin_type] is None:
                    full_bin_start_time[bin_type] = time.time()
                elif time.time() - full_bin_start_time[bin_type] >= FULL_HOLD_TIME:
                    led_port = LED_PAPER_FULL if bin_type == "paper" else LED_PLASTIC_FULL if bin_type == "plastic" else LED_METAL_FULL
                    grovepi.analogWrite(led_port, 255)
                    if not bin_full_notified:
                        send_bin_full_data(bin_type)
                        bin_full_notified = True
            else:
            	led_port = LED_PAPER_FULL if bin_type == "paper" else LED_PLASTIC_FULL if bin_type == "plastic" else LED_METAL_FULL
                full_bin_start_time[bin_type] = None
                grovepi.analogWrite(led_port, 0)
                bin_full_notified = False

        time.sleep(1)

except KeyboardInterrupt:
    print("\nProgram stopped by user")
finally:
    picam2.stop()
    picam2.close()
    print("✅ Camera shut down successfully.")
