import struct
from C6502 import CPU6502, WebDebugger

with open("test_binaries/6502_functional_test.bin", "rb") as f:
    content = f.read()
data = struct.unpack("B" * len(content), content)

cpu = CPU6502()
cpu.memory.write(0x0000, *data)
cpu.set_reset_vector(0x400)
cpu.reset()

wdbg = WebDebugger(cpu)
wdbg.run()
