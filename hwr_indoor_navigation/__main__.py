import logging
import os
import sys

sys.path.append(os.path.dirname(__file__))


import app


app = app.App(app.Config(
    event=app.event.Config(log_level=logging.DEBUG)
))

app.run()
