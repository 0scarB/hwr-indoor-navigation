import event as global_event
import hardware
from ._config import Config


class Ros2Adapter(global_event.Service[Config]):

    def setup(self, broker: global_event.Broker) -> None:
        startup_request_processor = global_event.processor.StartupRequestProcessor()
        shutdown_request_processor = global_event.processor.ShutdownRequestProcessor()