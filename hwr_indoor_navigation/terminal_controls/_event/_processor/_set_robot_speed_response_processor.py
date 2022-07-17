import robot
from ._set_robot_unit_value_response_processor import SetRobotUnitValueResponseProcessor


class SetRobotSpeedResponseProcessor(
    SetRobotUnitValueResponseProcessor[robot.event.Type.SET_SPEED]
):
    pass
