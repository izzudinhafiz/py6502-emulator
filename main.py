import struct
from CPU6502 import CPU, Memory
# from debug_6502 import WebDebugger
import time

with open("6502_functional_test.bin", "rb") as f:
    content = f.read()
data = struct.unpack("B" * len(content), content)

cpu = CPU(Memory())
cpu.memory.write(0x0000, *data)
cpu.set_reset_vector(0x400)
cpu.reset()


last_pc = cpu.PC
counter = 0
start_time = time.time_ns()
while counter <= 500_000:
    # if cpu.dbg.n_operations % 10_000 == 0:
    if counter % 100_000 == 0:
        t_time = time.time_ns() - start_time
        start_time = time.time_ns()
        print(f"Ops: {counter:,} Time: {t_time / 100_000} ns/ops")

    cpu.single_operation(False)
    if cpu.PC == last_pc:
        break
    last_pc = cpu.PC

    if cpu.PC == 0x0400:
        print("DONE")
        break

    counter += 1


# # print([hex(x) for x in cpu.memory[0x3708: 0x3708+50]])

# wdbg = WebDebugger(cpu)

# # n_ops = cpu.dbg.n_operations


# # cpu.memory.write(0x0000, *data)
# # cpu.set_reset_vector(0x400)
# # cpu.reset()


# wdbg.run(5_000_000)
