from __future__ import annotations

from typing import Protocol, Awaitable


class WithStartup(Protocol):

    def startup(self) -> None | Awaitable:
        pass


class WithShutdown(Protocol):

    def shutdown(self) -> None | Awaitable:
        pass
