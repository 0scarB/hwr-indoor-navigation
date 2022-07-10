import logging
import os
import sys

sys.path.append(os.path.dirname(__file__))

import event
from app import App
from config import Config


app = App(Config(
    event=event.Config(log_level=logging.DEBUG)
))

app.run()
