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
        log_level=logging.DEBUG
    ),
    services=[
        ("robot", robot.Config()),
        ("terminal_controls", terminal_controls.Config(
            movement_speed=UnitValue(100, "forward_backward_motor_pwm_duty_cycle"),
            steering_increment=UnitValue(20, "steering_motor_pwm_frequency"),
            forward_key="w",
            backward_key="s",
            left_key="a",
            right_key="d",
        ))
    ]
))

app.start()
