from typing import List, Tuple

from .._types import Publisher, Type


class ShutdownPublisher(Publisher):

    def publish(self) -> List[Tuple[Type.SHUTDOWN, None]]:
        return [(Type.SHUTDOWN, None)]
