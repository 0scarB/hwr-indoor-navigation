from __future__ import annotations

import asyncio
from typing import List, Callable

from sshkeyboard import stop_listening, listen_keyboard_manual

from interface import WithStartup, WithShutdown


class KeyboardListener(WithStartup, WithShutdown):
    _current_key: str | None
    _key_change_handlers: List[Callable[[str], None]]

    def __init__(self) -> None:
        self._current_key = None
        self._key_change_handlers = []

    async def startup(self) -> None:
        asyncio.ensure_future(listen_keyboard_manual(
            on_press=self._handle_key_press,
            on_release=self._handle_key_release
        ))

    def shutdown(self) -> None:
        stop_listening()

    def add_key_change_handler(self, handler: Callable[[str], None]) -> None:
        self._key_change_handlers.append(handler)

    async def _handle_key_press(self, key: str) -> None:
        self._handle_key_change(key)

    async def _handle_key_release(self, key: str) -> None:
        self._handle_key_change(None)

    def _handle_key_change(self, key: str | None) -> None:
        for handler in self._key_change_handlers:
            handler(key)

        self._current_key = key
