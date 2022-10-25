from typing import Generic, TypeVar


_Config = TypeVar("_Config")


class Service(Generic[_Config]):
    _config: _Config

    def __init__(self, _config: _Config) -> None:
        self._config = _Config
