from typing import Protocol


class WithStartup(Protocol):

    def startup(self) -> None:
        pass


class WithShutdown(Protocol):

    def shutdown(self) -> None:
        pass
