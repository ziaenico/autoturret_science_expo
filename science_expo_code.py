import threading
import time
import cv2
import numpy as np
import RPi.GPIO as GPIO
import pigpio

# ==============================
# Motor Control Setup (RPi.GPIO)
# ==============================

motorA_PinForward = 22  # GPIO number for forward direction of Motor A
motorA_PinReverse = 23  # GPIO number for reverse direction of Motor A
motorB_PinForward = 24  # GPIO number for forward direction of Motor B
motorB_PinReverse = 25  # GPIO number for reverse direction of Motor B

# Global state variables
motorA_running = False
motorB_running = False
# Shared command variable (None, 'activate', or 'deactivate')
motor_command = None

def motor_control():
    global motorA_running, motorB_running, motor_command
    try:
        GPIO.setmode(GPIO.BCM)  # Use Broadcom pin numbering
        GPIO.setup(motorA_PinForward, GPIO.OUT)
        GPIO.setup(motorA_PinReverse, GPIO.OUT)
        GPIO.setup(motorB_PinForward, GPIO.OUT)
        GPIO.setup(motorB_PinReverse, GPIO.OUT)

        def activate_motors():
            global motorA_running, motorB_running
            motorA_running = True
            motorB_running = True
            GPIO.output(motorA_PinForward, GPIO.HIGH)
            GPIO.output(motorB_PinForward, GPIO.HIGH)
            print("Motors activated")

        def deactivate_motors():
            global motorA_running, motorB_running
            motorA_running = False
            motorB_running = False
            GPIO.output(motorA_PinForward, GPIO.LOW)
            GPIO.output(motorB_PinForward, GPIO.LOW)
            print("Motors deactivated")

        # Continuously poll the shared command variable.
        while True:
            if motor_command == 'activate':
                activate_motors()
                motor_command = None
            elif motor_command == 'deactivate':
                deactivate_motors()
                motor_command = None
            time.sleep(0.1)  # Small delay to avoid busy-waiting

    except KeyboardInterrupt:
        print("Motor control stopped by user.")
    finally:
        GPIO.cleanup()  # Clean up GPIO pins on exit

# ==============================
# Vision Tracking & Servo Control
# ==============================

def set_angle(angle):
    """Sets the servo on GPIO 4 to the specified angle."""
    SERVO_PIN = 4
    pi = pigpio.pi()
    if not pi.connected:
        print("Error: Could not connect to pigpio daemon")
        return
    # Calculate pulse width from angle (0 degrees -> 500us, 180 degrees -> 2500us)
    pulse_width = 500 + (angle / 180.0) * 2000
    pi.set_servo_pulsewidth(SERVO_PIN, pulse_width)
    time.sleep(0.5)
    pi.stop()

def vision_tracking():
    PAN_SERVO = 17   # GPIO for left/right servo
    TILT_SERVO = 18  # GPIO for up/down servo

    NEUTRAL_PWM = 1500   # Neutral pulse width (stop)
    MAX_OFFSET = 500     # Maximum speed offset

    pan_gain = 0.005
    tilt_gain = 0.005

    # Initialize pigpio for servo control.
    pi = pigpio.pi()
    if not pi.connected:
        print("Error: Could not connect to pigpio daemon")
        return
    pi.set_mode(PAN_SERVO, pigpio.OUTPUT)
    pi.set_mode(TILT_SERVO, pigpio.OUTPUT)

    def set_servo_speed(pin, speed):
        pulse_width = NEUTRAL_PWM + int(speed * MAX_OFFSET)
        pi.set_servo_pulsewidth(pin, pulse_width)

    pipeline = "libcamerasrc ! video/x-raw, width=320, height=240, framerate=30/1 ! videoconvert ! appsink"
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        print("Error: Failed to open camera")
        return

    global motor_command  # To update motor commands from vision loop

    print("Press 'e' to activate motors, 'z' to deactivate motors,\n'a' to move servo to 130 degrees and back, and ESC to exit.")
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to capture image")
            break

        # Flip the frame 180° to correct upside-down camera input.
        frame = cv2.flip(frame, -1)
        # Alternatively, you could use:
        # frame = cv2.rotate(frame, cv2.ROTATE_180)

        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        low_red = np.array([161, 155, 84])
        high_red = np.array([179, 255, 255])
        red_mask = cv2.inRange(hsv_frame, low_red, high_red)

        contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)

        if contours:
            (x, y, w, h) = cv2.boundingRect(contours[0])
            x_medium = x + w // 2
            y_medium = y + h // 2

            frame_height, frame_width, _ = frame.shape
            x_center, y_center = frame_width // 2, frame_height // 2

            error_x = x_medium - x_center
            error_y = y_medium - y_center

            pan_speed = -pan_gain * error_x
            tilt_speed = -tilt_gain * error_y

            pan_speed = max(-1.0, min(1.0, pan_speed))
            tilt_speed = max(-1.0, min(1.0, tilt_speed))

            set_servo_speed(PAN_SERVO, pan_speed)
            set_servo_speed(TILT_SERVO, tilt_speed)

            cv2.line(frame, (x_medium, 0), (x_medium, frame_height), (0, 255, 0), 2)
            cv2.line(frame, (0, y_medium), (frame_width, y_medium), (255, 0, 0), 2)
        else:
            set_servo_speed(PAN_SERVO, 0)
            set_servo_speed(TILT_SERVO, 0)

        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1) & 0xFF

        # Check for motor commands or exit
        if key == 27:  # ESC key to exit
            break
        elif key == ord('e'):
            motor_command = 'activate'
        elif key == ord('z'):
            motor_command = 'deactivate'
        elif key == ord('a'):
            # Move servo to 130 degrees then return to 0 degrees.
            set_angle(0)
            time.sleep(1)
            set_angle(120)
            time.sleep(1)

    cap.release()
    cv2.destroyAllWindows()
    pi.stop()

# ==============================
# Main: Start Threads
# ==============================

if __name__ == "__main__":
    # Start motor_control in a background thread.
    motor_thread = threading.Thread(target=motor_control, daemon=True)
    motor_thread.start()

    # Run vision_tracking on the main thread.
    vision_tracking()

    print("Exiting program.")
