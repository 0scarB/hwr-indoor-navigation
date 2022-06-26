import sys
import time
import asyncio

sys.path.append("../adeept_picar-b/server")

from sshkeyboard import listen_keyboard
from Engine import Engine
import Adafruit_PCA9685


pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(50)

speed = 100
engine = Engine()
engine.setup()

current_pwm = 300
current_keys = set()


async def turn_right():
    global current_pwm

    while True:
        if "d" not in current_keys:
            break

        current_pwm -= 1
        pwm.set_pwm(0, 0, current_pwm)
        await asyncio.sleep(0.01)

async def turn_left():
    global current_pwm

    while True:
        if "a" not in current_keys:
            break

        current_pwm += 1
        pwm.set_pwm(0, 0, current_pwm)
        await asyncio.sleep(0.01)

def move_forward():
    engine.move(speed, "forward", 10000, None)

def move_backward():
    engine.move(speed, "backward", 10000, None)

async def press(key):
    print(f"'{key}' pressed")

    current_keys.add(key)
    print(current_keys)

    if "w" in current_keys:
        move_forward()
    elif "s" in current_keys:
        move_backward()

    if "a" in current_keys:
        await turn_left()
    elif "d" in current_keys:
        await turn_right()

def release(key):
    global current_keys

    print(f"'{key}' released")

    if "w" in current_keys or "s" in current_keys:
        engine.move(0, "forward", 0, None)

    current_keys.remove(key)

def drive():
    listen_keyboard(
        on_press=press,
        on_release=release,
    )

if __name__ == "__main__":
    try:
        drive()
    except KeyboardInterrupt:
        engine.destroy()



