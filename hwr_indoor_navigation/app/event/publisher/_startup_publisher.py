from typing import List, Tuple

from .._types import Publisher, Type


class StartupPublisher(Publisher):

    def publish(self) -> List[Tuple[Type.STARTUP, None]]:
        return [(Type.STARTUP, None)]
