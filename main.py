import struct
from CPU6502 import CPU, Memory
from debug_6502 import WebDebugger
import time

with open("6502_functional_test.bin", "rb") as f:
    content = f.read()
data = struct.unpack("B" * len(content), content)

cpu = CPU(Memory())
cpu.memory.write(0x0000, *data)
cpu.set_reset_vector(0x400)
cpu.reset()

wdbg = WebDebugger(cpu)
wdbg.run()
