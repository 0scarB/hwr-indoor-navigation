import time
import threading
from RBGctrl import RGBctrl

RGBCtrl_1 = RGBctrl()
RGBCtrl_1.setup()


def color_light(command):
    print(command)
    side = command["side"]
    r = command["R"]
    g = command["G"]
    b = command["B"]
    if side == 0:
        RGBCtrl_1.set_left_color(r, g, b)
        RGBCtrl_1.set_right_color(r, g, b)
    elif side == 1:
        RGBCtrl_1.set_left_color(r, g, b)
    elif side == 2:
        RGBCtrl_1.set_right_color(r, g, b)
    else:
        print("invalid side: " + str(side))


class Interval:
    def __init__(self, command):
        self.intervals = command["intervals"]
        self.commands = command["colors"]
        self.index = 0

    def apply_command(self):
        color_light(self.commands[self.index])
        time.sleep(self.intervals[self.index] / 1000)
        self.index = (self.index + 1) % len(self.commands)
        self.apply_command()


def main():
    a = {
        'intervals': [1000, 500],
        'colors': [
            {
                'R': 255, 'G': 255, 'B': 0, 'A': 255,
                'IsKnownColor': True,
                'IsEmpty': False,
                'IsNamedColor': True,
                'IsSystemColor': False,
                'side': 1,
                'Name': 'Yellow'
            },
            {
                'R': 0, 'G': 0, 'B': 0, 'A': 255,
                'IsKnownColor': True,
                'IsEmpty': False,
                'IsNamedColor': True,
                'IsSystemColor': False,
                'side': 1,
                'Name': 'Black'
            }
        ],
        'type': 3
    }
    interval1 = Interval(a)
    t1 = threading.Thread(target=interval1.apply_command)
    t1.start()
    time.sleep(0.5)

    b = {
        'intervals': [1000, 500],
        'colors': [
            {
                'R': 255, 'G': 255, 'B': 0, 'A': 255,
                'IsKnownColor': True,
                'IsEmpty': False,
                'IsNamedColor': True,
                'IsSystemColor': False,
                'side': 2,
                'Name': 'Yellow'
            },
            {
                'R': 0, 'G': 0, 'B': 0, 'A': 255,
                'IsKnownColor': True,
                'IsEmpty': False,
                'IsNamedColor': True,
                'IsSystemColor': False,
                'side': 2,
                'Name': 'Black'
            }
        ],
        'type': 3
    }
    interval2 = Interval(b)
    t2 = threading.Thread(target=interval2.apply_command)
    t2.start()


if __name__ == "__main__":
    main()
