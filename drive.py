import RPi.GPIO as GPIO
import tkinter as tk
from tkinter import Scale, HORIZONTAL
import time
import threading
from detector import WhiteRectangleDetector

class RobotController:
    def __init__(self):
        # Set GPIO mode and warnings
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # Define GPIO pins for each motor
        self.motor_pins = {
            "motor1": [20, 21],  # IN1, IN2
            "motor2": [16, 12],  # IN3, IN4
        }
        self.enable_pin = 26  # Enable pin for both motors

        # Define GPIO pins for the IR sensor array
        self.ir_pins = {
            "extreme_left": 9,
            "left": 10,
            "center": 22,
            "right": 27,
            "extreme_right": 17,
        }

        # Setup GPIO pins as output
        GPIO.setup(list(sum(self.motor_pins.values(), [])), GPIO.OUT)
        GPIO.setup(self.enable_pin, GPIO.OUT)

        # Setup GPIO pins as input for IR sensors
        GPIO.setup(list(self.ir_pins.values()), GPIO.IN)

        # Setup PWM for controlling motor speed on the enable pin
        self.frequency = 50
        self.pwm = GPIO.PWM(self.enable_pin, self.frequency)
        self.pwm.start(0)

        # Default speed is set to 50%
        self.default_speed = 12

        # Flag variable to track motor state
        self.motor_running = False  # Set to False initially (stopped)

        # Create a Tkinter window
        self.root = tk.Tk()
        self.root.title("Motor Control GUI")
        self.root.geometry("400x300")

        # Setup GUI elements
        forward_button = tk.Button(self.root, text="Forward", command=lambda: self.move_robot("forward"))
        forward_button.pack()

        reverse_button = tk.Button(self.root, text="Reverse", command=lambda: self.move_robot("reverse"))
        reverse_button.pack()

        left_button = tk.Button(self.root, text="Left", command=lambda: self.move_robot("left"))
        left_button.pack()

        right_button = tk.Button(self.root, text="Right", command=lambda: self.move_robot("right"))
        right_button.pack()

        stop_button = tk.Button(self.root, text="Stop", command=self.stop_motor)
        stop_button.pack()

        self.speed_slider = Scale(self.root, from_=0, to=100, orient=HORIZONTAL, label="Speed", command=self.set_speed)
        self.speed_slider.set(self.default_speed)
        self.speed_slider.pack()

        self.detector = WhiteRectangleDetector(camera_index=-1, threshold=50, min_rectangle_area=100)
        self.detector.run_detection()

        # Run the obstacle detection loop in a separate thread
        self.obstacle_thread = threading.Thread(target=self.check_obstacle)
        self.obstacle_thread.daemon = True  # Allow the thread to terminate when the main thread exits
        self.obstacle_thread.start()

    def set_speed(self, new_speed):
        if self.motor_running:
            self.pwm.ChangeDutyCycle(float(new_speed))
            # Update the slider position to reflect the new speed
            self.speed_slider.set(new_speed)

    def motor_control(self, action, speed):
        self.pwm.ChangeDutyCycle(speed)
        if action == "forward":
            GPIO.output(self.motor_pins["motor1"], [0, 1])
            GPIO.output(self.motor_pins["motor2"], [0, 1])
        elif action == "reverse":
            GPIO.output(self.motor_pins["motor1"], [1, 0])
            GPIO.output(self.motor_pins["motor2"], [1, 0])
        elif action == "left":
            GPIO.output(self.motor_pins["motor1"], [1, 0])
            GPIO.output(self.motor_pins["motor2"], [0, 0])
        elif action == "right":
            GPIO.output(self.motor_pins["motor1"], [0, 0])
            GPIO.output(self.motor_pins["motor2"], [1, 0])
        elif action == "stop":
            GPIO.output(self.enable_pin, 1)
            GPIO.output(self.motor_pins["motor1"], [0, 0])
            GPIO.output(self.motor_pins["motor1"], [0, 0])
            GPIO.output(self.motor_pins["motor2"], [0, 0])

    def stop_motor(self):
        self.motor_running = False
        self.motor_control("stop", 0)
        GPIO.output(self.enable_pin, GPIO.HIGH)

    def move_robot(self, action, speed=None):
        if speed is None:
            speed = self.speed_slider.get()
        self.motor_running = True
        print(action)
        #self.motor_control(action, speed)

    def check_obstacle(self):
        while True:
            # Read IR sensor values
            extreme_left = GPIO.input(self.ir_pins["extreme_left"])
            left = GPIO.input(self.ir_pins["left"])
            center = GPIO.input(self.ir_pins["center"])
            right = GPIO.input(self.ir_pins["right"])
            extreme_right = GPIO.input(self.ir_pins["extreme_right"])

            # Check if obstacle detected in front
            if center == 1:
                # Obstacle detected in front, stop and turn
                self.stop_motor()
                self.move_robot("reverse", 60)
                time.sleep(0.25)
                self.move_robot("right", 60)
                time.sleep(0.2)
            else:
                # Check left and right sensors for obstacles
                left_obstacle = left == 1 or extreme_left == 1
                right_obstacle = right == 1 or extreme_right == 1

                if left_obstacle and right_obstacle:
                    # If obstacles detected on both sides, move in reverse for 2 seconds
                    self.move_robot("reverse", 60)
                    time.sleep(0.5)
                elif left_obstacle:
                    # If obstacle detected on the left, turn right until left sensors are clear
                    self.move_robot("reverse", 60)
                    time.sleep(0.25)
                    self.move_robot("right", 60)
                    time.sleep(0.25)

                elif right_obstacle:
                    # If obstacle detected on the right, turn left until right sensors are clear
                    self.move_robot("reverse", 60)
                    time.sleep(0.25)
                    self.move_robot("left", 60)
                    time.sleep(0.25)

                else:
                    print(self.detector.is_rectangle_detected())
                    if self.detector.is_rectangle_detected() > 0:
                        # No obstacle detected in front, move forward
                        self.move_robot("forward")

            # Add a slight delay before checking again
            time.sleep(0.1)

    def run(self):
        # Run the Tkinter event loop
        self.root.mainloop()

    def cleanup(self):
        # Cleanup GPIO on program exit
        GPIO.cleanup()
        self.root.quit()
        self.detector.cleanup()

if __name__ == "__main__":
    robot_controller = RobotController()
    try:
        robot_controller.run()
    except KeyboardInterrupt:
        # Cleanup GPIO on Ctrl+C
        robot_controller.cleanup()
        
