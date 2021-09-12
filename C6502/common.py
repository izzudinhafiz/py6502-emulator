from typing import Literal, Callable, NamedTuple, overload
from abc import ABC, abstractmethod


class UnknownOpcodeError(Exception):
    pass


class MemoryAddressError(Exception):
    pass


bit = Literal[0] | Literal[1]


class OP(NamedTuple):
    cycles: int
    operation: Callable[[], int]
    addressing_mode: Callable[[], int]


class Registers(NamedTuple):
    PC: int
    SP: int
    A: int
    X: int
    Y: int


class Flags(NamedTuple):
    N: int
    Z: int
    C: int
    I: int
    D: int
    B: int
    O: int


class Trace(NamedTuple):
    op: str
    registers: Registers
    flags: Flags
    clock: int
    last_cycle: int
    mem_access: list[tuple[int, int]] | None
    stack: list[int]
    n_operations: int


class Memory(ABC):
    @overload
    @abstractmethod
    def __getitem__(self, key: int) -> int: ...

    @overload
    @abstractmethod
    def __getitem__(self, key: slice) -> list[int]: ...

    @abstractmethod
    def __getitem__(self, key: int | slice) -> int | list[int]: ...

    @abstractmethod
    def __setitem__(self, key, value): ...

    @abstractmethod
    def __len__(self) -> int: ...

    @abstractmethod
    def write(self, start_addr: int, *data: int): ...

    @abstractmethod
    def register_debugger(self, debugger): ...


class CPU(ABC):
    memory: Memory
    PC = 0
    SP = 0
    A = 0
    X = 0
    Y = 0

    N: bit = 0  # Negative Flag
    Z: bit = 0  # Zero Flag
    C: bit = 0  # Carry Flag
    I: bit = 0  # Interrupt Disable Flag
    D: bit = 0  # Decimal Mode Flag
    B: bit = 0  # Break Command Flag
    V: bit = 0  # Overflow Flag

    dbg: "Debugger"

    @abstractmethod
    def single_step(self): ...

    @abstractmethod
    def single_operation(self): ...

    @abstractmethod
    def get_status_flags(self) -> Flags: ...

    @abstractmethod
    def get_registers(self) -> Registers: ...

    @abstractmethod
    def reset(self): ...

    @abstractmethod
    def disassemble(self, start_addr: int, size: int) -> Trace: ...


class Debugger(ABC):
    @abstractmethod
    def trace(self, func): ...

    @abstractmethod
    def get_stack(self) -> list[int]: ...

    @abstractmethod
    def memory_write(self, addr, value): ...

    @abstractmethod
    def get_memory_write(self) -> list[tuple[int, int]]: ...

    @abstractmethod
    def get_last_trace(self) -> Trace | None: ...

    @abstractmethod
    def tick(self): ...


def twos_comp(val: int, bits: int) -> int:
    """compute the 2's complement of int value val"""
    if (val & (1 << (bits - 1))) != 0:  # if sign bit is set e.g., 8bit: 128-255
        val = val - (1 << bits)        # compute negative value
    return val
