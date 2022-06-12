import RPi.GPIO as GPIO
import time
import threading

left_R = 22
left_G = 23
left_B = 24
right_R = 10
right_G = 9
right_B = 25


class RGBctrl(threading.Thread):
    def __init__(self):
        pass

    def setup(self):
        self.pins = {
            "l_pin_R": left_R,
            "l_pin_G": left_G,
            "l_pin_B": left_B,
            "r_pin_R": right_R,
            "r_pin_G": right_G,
            "r_pin_B": right_B
        }
        GPIO.setmode(GPIO.BCM)
        for i in self.pins:
            GPIO.setup(self.pins[i], GPIO.OUT)  # Set the pin mode to output
            GPIO.output(self.pins[i], GPIO.LOW)  # Set the pin to low(0V) to off led

        self.l_pwmR = GPIO.PWM(self.pins['l_pin_R'], 50)  # Set Frequece to 50Hz
        self.l_pwmG = GPIO.PWM(self.pins['l_pin_G'], 50)
        self.l_pwmB = GPIO.PWM(self.pins['l_pin_B'], 50)
        self.r_pwmR = GPIO.PWM(self.pins['r_pin_R'], 50)
        self.r_pwmG = GPIO.PWM(self.pins['r_pin_G'], 50)
        self.r_pwmB = GPIO.PWM(self.pins['r_pin_B'], 50)

        self.set_left_on()
        self.set_right_on()

    def map(self, x, in_min, in_max, out_min, out_max):  # Map 0-255 to 0-100
        return (x - in_min) / (in_max - in_min) * (out_max - out_min) + out_min

    def off(self):
        for i in self.pins:
            GPIO.output(self.pins[i], GPIO.LOW)

    def set_left_color(self, r, g, b):

        #r_val = self.map(r, 0, 255, 100, 0)  # Map 0-255 to 0-100
        #g_val = self.map(g, 0, 255, 100, 0)
        #b_val = self.map(b, 0, 255, 100, 0)

        r_val = (1-r) * 100
        g_val = (1-g) * 100
        b_val = (1-b) * 100

        self.l_pwmR.ChangeDutyCycle(r_val)  # Change the duty cycle,change rhe color brightness.
        self.l_pwmG.ChangeDutyCycle(g_val)
        self.l_pwmB.ChangeDutyCycle(b_val)

    def set_right_color(self, r, g, b):
        #r_val = self.map(r, 0, 255, 100, 0)  # Map 0-255 to 0-100
        #g_val = self.map(g, 0, 255, 100, 0)
        #b_val = self.map(b, 0, 255, 100, 0)

        r_val = (1-r) * 100
        g_val = (1-g) * 100
        b_val = (1-b) * 100

        self.r_pwmR.ChangeDutyCycle(r_val)  # Change the duty cycle,change rhe color brightness.
        self.r_pwmG.ChangeDutyCycle(g_val)
        self.r_pwmB.ChangeDutyCycle(b_val)

    def set_left_on(self):
        self.l_pwmR.start(0)  # Turn off PWM.
        self.l_pwmG.start(0)
        self.l_pwmB.start(0)

    def set_right_on(self):
        self.r_pwmR.start(0)  # Turn off PWM.
        self.r_pwmG.start(0)
        self.r_pwmB.start(0)

    def set_left_off(self):
        self.l_pwmR.ChangeDutyCycle(100)
        self.l_pwmG.ChangeDutyCycle(100)
        self.l_pwmB.ChangeDutyCycle(100)

    def set_right_off(self):
        self.r_pwmR.ChangeDutyCycle(100)
        self.r_pwmG.ChangeDutyCycle(100)
        self.r_pwmB.ChangeDutyCycle(100)

    def destroy(self):
        self.l_pwmR.stop()  # Turn off PWM.
        self.l_pwmG.stop()
        self.l_pwmB.stop()
        self.off()  # Turn off all leds.
        GPIO.cleanup()  # Reset GPIO status.


if __name__ == "__main__":
    try:
        RGBCtrl_1 = RGBctrl()
        RGBCtrl_1.setup()
        while True:
            RGBCtrl_1.set_left_color(0, 255, 0)
            time.sleep(1)
            RGBCtrl_1.set_left_color(255, 0, 0)
            time.sleep(1)
            RGBCtrl_1.set_left_off()
            time.sleep(1)
    except KeyboardInterrupt:
        RGBCtrl_1.destroy()
