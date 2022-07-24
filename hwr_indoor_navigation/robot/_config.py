from dataclasses import dataclass
from xmlrpc.client import Boolean


@dataclass(frozen=True)
class Config:
    use_lidar: bool
