import logging
from dataclasses import dataclass

from ._type import LogLevel


@dataclass(frozen=True)
class Config:
    log_level: LogLevel = logging.WARNING
