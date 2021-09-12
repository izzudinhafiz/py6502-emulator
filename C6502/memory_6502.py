from typing import overload
from .common import MemoryAddressError, Memory
from .debugger_6502 import Debugger6502, DEBUG


class Memory6502(Memory):
    def __init__(self, size: int = 1024*64):
        self.MAX_MEMORY = size
        self.data = [0] * self.MAX_MEMORY

    def register_debugger(self, debugger):
        self.dbg: Debugger6502 = debugger

    @overload
    def __getitem__(self, key: int) -> int: ...

    @overload
    def __getitem__(self, key: slice) -> list[int]: ...

    def __getitem__(self, key: int | slice) -> int | list[int]:
        try:
            temp = self.data[key]
            return temp
        except IndexError:
            raise MemoryAddressError(key)

    def __setitem__(self, key: int, value: int):
        if key >= self.MAX_MEMORY or key < 0:
            raise MemoryAddressError(key)
        self.data[key] = value

        if DEBUG:
            self.dbg.memory_write(key, value)

    def __len__(self):
        return len(self.data)

    def write(self, start_addr: int, *data: int):
        for i, byte in enumerate(data):
            self.data[start_addr + i] = byte
