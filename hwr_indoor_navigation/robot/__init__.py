import event as global_event
from . import _event as event
from . import _error as error
from . import _component
from ._config import Config
from ._forward_backward_mover import ForwardBackwardMover
from ._steerer import Steerer


class Robot(global_event.Service):

    def use_event_broker(self, broker: global_event.Broker) -> None:
        startup_request_processor = global_event.processor.StartupRequestProcessor()
        shutdown_request_processor = global_event.processor.ShutdownRequestProcessor()
        set_speed_request_processor = event._processor.SetSpeedRequestProcessor()
        set_heading_request_processor = event._processor.SetHeadingRequestProcessor()

        forward_backward_mover = ForwardBackwardMover(
            _component.ForwardBackwardMotor()
        )
        steerer = Steerer(
            _component.SteeringMotor()
        )
        forward_backward_mover.use_startup_request_event_processor(startup_request_processor)
        forward_backward_mover.use_set_speed_request_event_processor(
            set_speed_request_processor
        )
        steerer.use_set_heading_request_event_processor(
            set_heading_request_processor
        )
        forward_backward_mover.use_shutdown_request_event_processor(shutdown_request_processor)

        broker.add_processor(
            global_event.Topic.REQUEST,
            global_event.Type.STARTUP,
            startup_request_processor,
        )
        broker.add_processor(
            global_event.Topic.REQUEST,
            global_event.Type.SHUTDOWN,
            shutdown_request_processor,
        )
        broker.add_processor(
            global_event.Topic.REQUEST,
            event.Type.SET_SPEED,
            set_speed_request_processor,
        )
        broker.add_processor(
            global_event.Topic.REQUEST,
            event.Type.SET_HEADING,
            set_heading_request_processor,
        )

