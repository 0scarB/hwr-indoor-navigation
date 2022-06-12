#!/usr/bin/env python3
import time
import threading

import RPi.GPIO as GPIO

Motor_Slot_A_EN = 17

Motor_Slot_A_Pin1 = 27
Motor_Slot_A_Pin2 = 18

Dir_forward = 0
Dir_backward = 1

pwm_B = 0


class Engine:

    def __init__(self):
        self.thread = None

    def motorStop(self):  # Motor stops
        GPIO.output(Motor_Slot_A_Pin1, GPIO.LOW)
        GPIO.output(Motor_Slot_A_Pin2, GPIO.LOW)
        GPIO.output(Motor_Slot_A_EN, GPIO.LOW)

    def setup(self):  # Motor initialization
        global pwm_B
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(Motor_Slot_A_EN, GPIO.OUT)
        GPIO.setup(Motor_Slot_A_Pin1, GPIO.OUT)
        GPIO.setup(Motor_Slot_A_Pin2, GPIO.OUT)

        self.motorStop()
        try:
            pwm_B = GPIO.PWM(Motor_Slot_A_EN, 1000)
        except:
            pass

    def motor(self, status, direction, speed):
        if status == 0:  # stop
            GPIO.output(Motor_Slot_A_Pin1, GPIO.LOW)
            GPIO.output(Motor_Slot_A_Pin2, GPIO.LOW)
            GPIO.output(Motor_Slot_A_EN, GPIO.LOW)
        else:
            if direction == Dir_backward:
                GPIO.output(Motor_Slot_A_Pin1, GPIO.HIGH)
                GPIO.output(Motor_Slot_A_Pin2, GPIO.LOW)
                pwm_B.start(100)
                pwm_B.ChangeDutyCycle(speed)
            elif direction == Dir_forward:
                GPIO.output(Motor_Slot_A_Pin1, GPIO.LOW)
                GPIO.output(Motor_Slot_A_Pin2, GPIO.HIGH)
                pwm_B.start(0)
                pwm_B.ChangeDutyCycle(speed)

    def moveSync(self, speed, direction, duration, callback):
        # speed = 100
        if direction == 'forward':
            self.motor(1, Dir_forward, speed)
            time.sleep(duration)
            self.motorStop()
            if callback is not None:
                callback()
        elif direction == 'backward':
            self.motor(1, Dir_backward, speed)
            time.sleep(duration)
            self.motorStop()
            if callback is not None:
                callback()
        elif direction == 'no':
            self.motorStop()
            if callback is not None:
                callback()
        else:
            pass

    def motorForward(self, speed):
        self.motor(1, Dir_forward, speed)

    def motorBackward(self, speed):
        self.motor(1, Dir_backward, speed)

    def move(self, speed, direction, duration, callback):
        self.thread = threading.Thread(target=self.moveSync, args=(speed, direction, duration, callback))
        self.thread.start()

    def destroy(self):
        self.motorStop()
        GPIO.cleanup()  # Release resource


def callback():
    print('yo.')


if __name__ == '__main__':
    speed_set = 100
    engine = Engine()
    try:
        engine.setup()
        engine.move(speed_set, 'forward', 1.3, None)
        # engine.destroy()
    except KeyboardInterrupt:
        engine.destroy()
