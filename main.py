import struct
from CPU6502 import CPU, Memory
from debug_6502 import WebDebugger

with open("6502_functional_test.bin", "rb") as f:
    # with open("cputest.bin", "rb") as f:
    content = f.read()
data = struct.unpack("B" * len(content), content)

cpu = CPU(Memory())
cpu.memory.write(0x0000, *data)
cpu.set_reset_vector(0x400)
cpu.reset()

# print([hex(x) for x in cpu.memory[0x3708: 0x3708+50]])

wdbg = WebDebugger(cpu)
# for i in range(50):
#     cpu.single_operation()
for i in range(40700):
    cpu.single_operation(False)

wdbg.run()
