import keyboard

from Engine import Engine


def drive():
    speed = 100
    engine = Engine()
    engine.setup()
    try:
        while True:
            if keyboard.read_key() == "w":
                engine.move(speed, "forward", 1.3, None)
            if keyboard.read_key() == "d":
                engine.move(speed, "backward", 1.3, None)
            # if keyboard.read_key() == "a":
    except KeyboardInterrupt:
        engine.destroy()



if __name__ == "__main__":
    drive()



