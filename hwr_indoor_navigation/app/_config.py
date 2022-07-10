from dataclasses import dataclass

from .event import Config as EventConfig


@dataclass(frozen=True)
class Config:
    event: EventConfig
