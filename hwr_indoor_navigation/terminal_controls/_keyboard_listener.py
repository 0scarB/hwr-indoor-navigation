from __future__ import annotations

from typing import Protocol, List

from sshkeyboard import listen_keyboard, stop_listening

from interface import WithStartup, WithShutdown


class Subscriber(Protocol):

    def handle_key_changed(self, key: str) -> None:
        ...


class KeyboardListener(WithStartup, WithShutdown):
    _current_key: str | None
    _subscribers: List[Subscriber]

    def __init__(self) -> None:
        self._current_key = None
        self._subscribers = []

    def startup(self) -> None:
        listen_keyboard(
            on_press=self._handle_key_press,
            on_release=self._handle_key_release
        )

    def shutdown(self) -> None:
        stop_listening()

    def add_subscriber(self, subscribers: Subscriber) -> None:
        self._subscribers.append(subscribers)

    async def _handle_key_press(self, key: str) -> None:
        self._current_key = key

    async def _handle_key_release(self, key: str) -> None:
        self._handle_key_change(None)

    def _handle_key_change(self, key: str | None) -> None:
        for subscriber in self._subscribers:
            subscriber.handle_key_changed(key)

        self._current_key = key
