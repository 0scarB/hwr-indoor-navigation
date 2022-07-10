import logging
from dataclasses import dataclass

from ._types import LogLevel


@dataclass(frozen=True)
class Config:
    log_level: LogLevel = logging.WARNING
