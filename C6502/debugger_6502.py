from collections import deque
from .common import Trace, CPU, Debugger

DEBUG = True


class Debugger6502(Debugger):
    def __init__(self, cpu: CPU):
        self.cpu = cpu
        self.memory = cpu.memory
        self.trace_stack: deque[Trace] = deque(maxlen=1000)
        self.global_tick = 0
        self.n_operations = 0
        self.recent_write: list[tuple[int, int]] = []
        self.memory.register_debugger(self)
        self.stack: list[int] = []

    def trace(self, func):
        op = self.cpu.disassemble(self.cpu.PC, 1)[0]

        cycles = 0
        while func():
            cycles += 1

        self.n_operations += 1
        registers = self.cpu.get_registers()
        flags = self.cpu.get_status_flags()

        trace = Trace(op, registers, flags,
                      self.global_tick, cycles,
                      self.get_memory_write(), self.get_stack(),
                      self.n_operations)

        self.trace_stack.append(trace)

    def get_stack(self) -> list[int]:
        self.stack = self.memory[0x0101 + self.cpu.SP: 0x01FF + 1]
        return self.stack

    def memory_write(self, addr, value):
        self.recent_write.append((addr, value))

    def get_memory_write(self) -> list[tuple[int, int]]:
        temp = self.recent_write
        self.recent_write = []
        return temp

    def get_last_trace(self):
        if len(self.trace_stack) > 0:
            return self.trace_stack[-1]
        else:
            return None

    def tick(self):
        self.global_tick += 1
