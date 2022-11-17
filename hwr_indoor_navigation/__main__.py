import logging
import os
import sys

import event
import robot
import terminal_controls
from unit import UnitValue

sys.path.append(os.path.dirname(__file__))


import app


app = app.App(app.Config(
    event=event.Config(
        log_level=logging.WARN
    ),
    services=[
        ("robot", robot.Config(use_lidar=True)),
        ("terminal_controls", terminal_controls.Config(
            movement_speed=UnitValue(100, "_drive_motor_pwm_duty_cycle"),
            steering_increment=UnitValue(20, "_steering_motor_pwm_duty_cycle"),
            forward_key="w",
            backward_key="s",
            left_key="a",
            right_key="d",
        ))
    ]
))

app.start()
