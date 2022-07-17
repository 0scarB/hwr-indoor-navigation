from dataclasses import dataclass
from typing import Literal, Iterable, Tuple, Union

import event
import robot
import terminal_controls


Service = Union[
    Tuple[Literal["robot"], robot.Config],
    Tuple[Literal["terminal_controls"], terminal_controls.Config]
]


@dataclass(frozen=True)
class Config:
    event: event.Config
    services: Iterable[Service]
