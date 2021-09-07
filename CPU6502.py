from typing import Callable, Literal, overload
from typing import NamedTuple
import traceback

DEBUG = True


class MemoryAddressError(Exception):
    pass


class UnknownOpcodeError(Exception):
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


class Memory:
    def __init__(self, size: int = 1024*64):
        self.MAX_MEMORY = size
        # self.data = np.zeros(self.MAX_MEMORY, dtype=np.uint16)
        self.data = [0] * self.MAX_MEMORY

    def register_debugger(self, debugger):
        self.dbg: Debugger = debugger

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


class CPU:
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

    def __init__(self, memory: Memory):

        self.memory = memory
        self.clock = 0
        self.fetched = 0
        self.absolute_address = 0
        self.relative_address = 0
        self.opcode = OP(1, self.NOP, self.implicit)
        self.lookup = {
            # ADC Opcodes (Add with Carry)
            0x69: OP(2, self.ADC, self.immediate),
            0x65: OP(3, self.ADC, self.zeropage),
            0x75: OP(4, self.ADC, self.zeropageX),
            0x6D: OP(4, self.ADC, self.absolute),
            0x7D: OP(4, self.ADC, self.absoluteX),
            0x79: OP(4, self.ADC, self.absoluteY),
            0x61: OP(6, self.ADC, self.indirectX),
            0x71: OP(5, self.ADC, self.indirectY),

            # AND Opcodes
            0x29: OP(2, self.AND, self.immediate),
            0x25: OP(3, self.AND, self.zeropage),
            0x35: OP(4, self.AND, self.zeropageX),
            0x2D: OP(4, self.AND, self.absolute),
            0x3D: OP(4, self.AND, self.absoluteX),
            0x39: OP(4, self.AND, self.absoluteY),
            0x21: OP(6, self.AND, self.indirectX),
            0x31: OP(5, self.AND, self.indirectY),

            # ASL Opcodes (Arithmetic Shift Left)
            0x0A: OP(2, self.ASL, self.accumulator),
            0x06: OP(5, self.ASL, self.zeropage),
            0x16: OP(6, self.ASL, self.zeropageX),
            0x0E: OP(6, self.ASL, self.absolute),
            0x1E: OP(7, self.ASL, self.absoluteX),

            # BIT Opcodes (test BITS)
            0x24: OP(3, self.BIT, self.zeropage),
            0x2C: OP(4, self.BIT, self.absolute),

            # Branch Opcodes
            0x10: OP(2, self.BPL, self.relative),  # Branch on Plus
            0x30: OP(2, self.BMI, self.relative),  # Branch on Minus
            0x50: OP(2, self.BVC, self.relative),  # Branch on Overflow Clear
            0x70: OP(2, self.BVS, self.relative),  # Branch on Overflow Set
            0x90: OP(2, self.BCC, self.relative),  # Branch on Carry Clear
            0xB0: OP(2, self.BCS, self.relative),  # Branch on Carry Set
            0xD0: OP(2, self.BNE, self.relative),  # Branch on Not Equal
            0xF0: OP(2, self.BEQ, self.relative),  # Branch on Equal

            # BRK Opcodes (Break)
            0x00: OP(7, self.BRK, self.implicit),

            # CMP Opcodes (Compare Accumulator)
            0xC9: OP(2, self.CMP, self.immediate),
            0xC5: OP(3, self.CMP, self.zeropage),
            0xD5: OP(4, self.CMP, self.zeropageX),
            0xCD: OP(4, self.CMP, self.absolute),
            0xDD: OP(4, self.CMP, self.absoluteX),
            0xD9: OP(4, self.CMP, self.absoluteY),
            0xC1: OP(6, self.CMP, self.indirectX),
            0xD1: OP(5, self.CMP, self.indirectY),

            # CPX Opcodes (Compare X)
            0xE0: OP(2, self.CPX, self.immediate),
            0xE4: OP(3, self.CPX, self.zeropage),
            0xEC: OP(4, self.CPX, self.absolute),

            # CPY Opcodes (Compare Y)
            0xC0: OP(2, self.CPY, self.immediate),
            0xC4: OP(3, self.CPY, self.zeropage),
            0xCC: OP(4, self.CPY, self.absolute),

            # DEC Opcodes (Decrement Memory)
            0xC6: OP(5, self.DEC, self.zeropage),
            0xD6: OP(6, self.DEC, self.zeropageX),
            0xCE: OP(6, self.DEC, self.absolute),
            0xDE: OP(7, self.DEC, self.absoluteX),

            # EOR Opcodes (bitwise Exclusive OR)
            0x49: OP(2, self.EOR, self.immediate),
            0x45: OP(3, self.EOR, self.zeropage),
            0x55: OP(4, self.EOR, self.zeropageX),
            0x4D: OP(4, self.EOR, self.absolute),
            0x5D: OP(4, self.EOR, self.absoluteX),
            0x59: OP(4, self.EOR, self.absoluteY),
            0x41: OP(6, self.EOR, self.indirectX),
            0x51: OP(5, self.EOR, self.indirectY),

            # Flag Opcodes
            0x18: OP(2, self.CLC, self.implicit),
            0x38: OP(2, self.SEC, self.implicit),
            0x58: OP(2, self.CLI, self.implicit),
            0x78: OP(2, self.SEI, self.implicit),
            0xB8: OP(2, self.CLV, self.implicit),
            0xD8: OP(2, self.CLD, self.implicit),
            0xF8: OP(2, self.SED, self.implicit),

            # INC Opcodes (Increment memory)
            0xE6: OP(5, self.INC, self.zeropage),
            0xF6: OP(6, self.INC, self.zeropageX),
            0xEE: OP(6, self.INC, self.absolute),
            0xFE: OP(7, self.INC, self.absoluteX),

            # JMP Opcodes (Jump)
            0x4C: OP(3, self.JMP, self.absolute),
            0x6C: OP(5, self.JMP, self.indirect),

            # JSR Opcodes (Jump to Subroutine)
            0x20: OP(6, self.JSR, self.absolute),

            # LDA Opcodes (Load accumulator)
            0xA9: OP(2, self.LDA, self.immediate),
            0xA5: OP(3, self.LDA, self.zeropage),
            0xB5: OP(4, self.LDA, self.zeropageX),
            0xAD: OP(4, self.LDA, self.absolute),
            0xBD: OP(4, self.LDA, self.absoluteX),
            0xB9: OP(4, self.LDA, self.absoluteY),
            0xA1: OP(6, self.LDA, self.indirectX),
            0xB1: OP(5, self.LDA, self.indirectY),

            # LDX Opcodes (Load X)
            0xA2: OP(2, self.LDX, self.immediate),
            0xA6: OP(3, self.LDX, self.zeropage),
            0xB6: OP(4, self.LDX, self.zeropageY),
            0xAE: OP(4, self.LDX, self.absolute),
            0xBE: OP(4, self.LDX, self.absoluteY),

            # LDY Opcodes (Load Y)
            0xA0: OP(2, self.LDY, self.immediate),
            0xA4: OP(3, self.LDY, self.zeropage),
            0xB4: OP(4, self.LDY, self.zeropageX),
            0xAC: OP(4, self.LDY, self.absolute),
            0xBC: OP(4, self.LDY, self.absoluteX),

            # LSR Opcodes (Logical Shift Right)
            0x4A: OP(2, self.LSR, self.accumulator),
            0x46: OP(5, self.LSR, self.zeropage),
            0x56: OP(6, self.LSR, self.zeropageX),
            0x4E: OP(6, self.LSR, self.absolute),
            0x5E: OP(7, self.LSR, self.absoluteX),

            # NOP
            0xEA: OP(2, self.NOP, self.implicit),

            # ORA Opcodes (Bitwise OR with accumulator)
            0x09: OP(2, self.ORA, self.immediate),
            0x05: OP(3, self.ORA, self.zeropage),
            0x15: OP(4, self.ORA, self.zeropageX),
            0x0D: OP(4, self.ORA, self.absolute),
            0x1D: OP(4, self.ORA, self.absoluteX),
            0x19: OP(4, self.ORA, self.absoluteY),
            0x01: OP(6, self.ORA, self.indirectX),
            0x11: OP(5, self.ORA, self.indirectY),

            # Register Instructions
            0xAA: OP(2, self.TAX, self.implicit),
            0x8A: OP(2, self.TXA, self.implicit),
            0xCA: OP(2, self.DEX, self.implicit),
            0xE8: OP(2, self.INX, self.implicit),
            0xA8: OP(2, self.TAY, self.implicit),
            0x98: OP(2, self.TYA, self.implicit),
            0x88: OP(2, self.DEY, self.implicit),
            0xC8: OP(2, self.INY, self.implicit),

            # ROL Opcodes (Rotate Left)
            0x2A: OP(2, self.ROL, self.accumulator),
            0x26: OP(5, self.ROL, self.zeropage),
            0x36: OP(6, self.ROL, self.zeropageX),
            0x2E: OP(6, self.ROL, self.absolute),
            0x3E: OP(7, self.ROL, self.absoluteX),

            # ROR Opcodes (Rotate Right)
            0x6A: OP(2, self.ROR, self.accumulator),
            0x66: OP(5, self.ROR, self.zeropage),
            0x76: OP(6, self.ROR, self.zeropageX),
            0x6E: OP(6, self.ROR, self.absolute),
            0x7E: OP(7, self.ROR, self.absoluteX),

            # RTI Opcode (Return from Interrupt)
            0x40: OP(6, self.RTI, self.implicit),

            # RTS Opcode (Return from Subroutine)
            0x60: OP(6, self.RTS, self.implicit),

            # SBC Opcodes (Subtract with Carry)
            0xE9: OP(2, self.SBC, self.immediate),
            0xE5: OP(3, self.SBC, self.zeropage),
            0xF5: OP(4, self.SBC, self.zeropageX),
            0xED: OP(4, self.SBC, self.absolute),
            0xFD: OP(4, self.SBC, self.absoluteX),
            0xF9: OP(4, self.SBC, self.absoluteY),
            0xE1: OP(6, self.SBC, self.indirectX),
            0xF1: OP(5, self.SBC, self.indirectY),

            # STA Opcodes (Store Accumulator)
            0x85: OP(3, self.STA, self.zeropage),
            0x95: OP(4, self.STA, self.zeropageX),
            0x8D: OP(4, self.STA, self.absolute),
            0x9D: OP(5, self.STA, self.absoluteX),
            0x99: OP(5, self.STA, self.absoluteY),
            0x81: OP(6, self.STA, self.indirectX),
            0x91: OP(6, self.STA, self.indirectY),

            # Stack instructions
            0x9A: OP(2, self.TXS, self.implicit),
            0xBA: OP(2, self.TSX, self.implicit),
            0x48: OP(3, self.PHA, self.implicit),
            0x68: OP(4, self.PLA, self.implicit),
            0x08: OP(3, self.PHP, self.implicit),
            0x28: OP(4, self.PLP, self.implicit),

            # STX Opcodes (Store X)
            0x86: OP(3, self.STX, self.zeropage),
            0x96: OP(4, self.STX, self.zeropageY),
            0x8E: OP(4, self.STX, self.absolute),

            # STY Opcodes (Store Y)
            0x84: OP(3, self.STY, self.zeropage),
            0x94: OP(4, self.STY, self.zeropageX),
            0x8C: OP(4, self.STY, self.absolute)
        }

        # Debug variables
        self.global_clock = 0
        self.global_tick = 0

        self.reset()
        self.dbg = Debugger(self)

    def set_reset_vector(self, addr: int):
        self.memory[0xFFFD] = (addr & 0xFF00) >> 8
        self.memory[0xFFFC] = addr & 0xFF

        if DEBUG:
            self.dbg.get_memory_write()  # Clearing untraced memory for debugging

    def single_step(self):
        """
        Steps the clock of the CPU 1 tick. This is only a clock simulation.
        The real operation will be done in one cycle while the rest of the
        cycles will just be empty 'busy' simulation

        Raises:
            UnknownOpcodeError: Invalid Opcode has been read at the current PC

        Returns:
            None
        """
        if DEBUG:
            self.dbg.tick()

        try:
            if self.clock == 0:
                opcode = self.fetch_byte()

                if opcode not in self.lookup.keys():
                    raise UnknownOpcodeError(opcode)

                self.opcode = self.lookup[opcode]

                self.clock = self.opcode.cycles

                self.clock += self.opcode.addressing_mode()
                self.clock += self.opcode.operation()

                self.global_clock += self.clock
            self.clock -= 1

            if self.clock > 0:
                return True
            else:
                return False

        except:
            print(f"Crashing at PC {self.PC}")
            traceback.print_exc()
            self.reset()
            return False

    def single_operation(self, skip=True):
        if DEBUG:
            self.dbg.trace(self.single_step)
        else:
            while self.single_step():
                pass

    def get_status_flags_byte(self, mode: Literal["instruction", "interrupt"]) -> int:
        if mode == "instruction":
            temp = (self.C << 0) | (self.Z << 1) | (self.I << 2) | (self.D << 3) | (1 << 4) | (1 << 5) | (self.V << 6) | (self.N << 7)
        elif mode == "interrupt":
            temp = (self.C << 0) | (self.Z << 1) | (self.I << 2) | (self.D << 3) | (0 << 4) | (1 << 5) | (self.V << 6) | (self.N << 7)
        else:
            raise NotImplementedError()
        return temp

    def get_status_flags(self) -> Flags:
        return Flags(self.N, self.Z, self.C, self.I, self.D, self.B, self.V)

    def set_status_flags_byte(self, byte: int):
        self.C = 1 if (byte & (1 << 0)) else 0
        self.Z = 1 if (byte & (1 << 1)) else 0
        self.I = 1 if (byte & (1 << 2)) else 0
        self.D = 1 if (byte & (1 << 3)) else 0
        self.B = 1 if (byte & (1 << 4)) else 0
        self.V = 1 if (byte & (1 << 6)) else 0
        self.N = 1 if (byte & (1 << 7)) else 0

    def get_registers(self) -> Registers:
        """
        Mainly debugging function to return all current registers value

        Returns:
            Registers: NamedTuple containing all current registers value
        """

        return Registers(self.PC, self.SP, self.A, self.X, self.Y)

    def set_NZ_flag(self, val):
        if val == 0:
            self.Z = 1
        else:
            self.Z = 0

        if val & (1 << 7):
            self.N = 1
        else:
            self.N = 0

    def fetch_byte(self) -> int:
        value = self.memory[self.PC]
        self.PC += 1
        return value

    def fetch_word(self):
        lo_byte = self.fetch_byte()
        hi_byte = (self.fetch_byte() << 8)

        return hi_byte | lo_byte

    def write_word(self, data, addr: int):
        self.memory[addr] = data & 0xFF
        self.memory[addr + 1] = (data >> 8)

    def read_word(self, addr: int):
        low_b = self.memory[addr]
        high_b = (self.memory[addr + 1] << 8)

        return high_b | low_b

    def fetch(self) -> int:
        if self.opcode.addressing_mode is not self.accumulator:
            self.fetched = self.memory[self.absolute_address]

        return self.fetched

    def stack_push(self, byte: int):
        self.memory[0x0100 + self.SP] = byte
        self.SP = (self.SP - 1) & 0xFF

    def stack_pull(self) -> int:
        self.SP = (self.SP + 1) & 0xFF
        return self.memory[0x0100 + self.SP]

    def reset(self):
        self.fetched = 0
        self.absolute_address = 0
        self.relative_address = 0
        self.opcode = OP(1, self.NOP, self.implicit)
        self.SP = 0xFD
        self.A = self.X = self.Y = 0

        self.N = 0  # Negative Flag
        self.Z = 0  # Zero Flag
        self.C = 0  # Carry Flag
        self.I = 0  # Interrupt Disable Flag
        self.D = 0  # Decimal Mode Flag
        self.B = 0  # Break Command Flag
        self.V = 0  # Overflow Flag

        self.PC = (self.memory[0xFFFD] << 8) | self.memory[0xFFFC]
        self.clock = 0

        return 0

    def IRQ(self):
        if self.I == 0:
            self.stack_push((self.PC >> 8) & 0xFF)  # Write PC High Byte
            self.stack_push(self.PC & 0xFF)         # Write PC Low Byte

            self.I = 1
            self.stack_push(self.get_status_flags_byte("interrupt"))  # Push status register to stack

            self.PC = (self.memory[0xFFFF] << 8) | self.memory[0xFFFE]

            return 7

    def NMI(self):
        self.stack_push((self.PC >> 8) & 0xFF)  # Write PC High Byte
        self.stack_push(self.PC & 0xFF)         # Write PC Low Byte

        self.I = 1
        self.stack_push(self.get_status_flags_byte("interrupt"))  # Push status register to stack

        self.PC = (self.memory[0xFFFB] << 8) | self.memory[0xFFFA]

        return 8

    def dissamble(self, start_addr: int, size=16):
        lo = 0
        hi = 0
        lines = []

        ins_mapper: dict[Callable, tuple[str, int]] = {
            (self.immediate): ("IMM", 1),
            (self.accumulator): ("ACM", 0),
            (self.implicit): ("IMP", 0),
            (self.zeropage): ("ZP0", 1),
            (self.zeropageX): ("ZPX", 1),
            (self.zeropageY): ("ZPY", 1),
            (self.absolute): ("ABS", 2),
            (self.absoluteX): ("ABX", 2),
            (self.absoluteY): ("ABY", 2),
            (self.indirect): ("IND", 2),
            (self.indirectX): ("INX", 1),
            (self.indirectY): ("INY", 1),
            (self.relative): ("REL", 1)
        }

        addr = start_addr
        for _ in range(size):
            ins_addr = addr
            opcode = self.memory[addr]
            addr += 1

            if opcode in self.lookup.keys():
                op = self.lookup[opcode]

                addr_mode, fetch_size = ins_mapper[op.addressing_mode]

                # $ADDR MODE OP OPERAND,OPERAND

                line = f"${ins_addr:04X} [{addr_mode}] {op.operation.__name__} "

                if fetch_size == 0:
                    pass
                elif fetch_size == 1:
                    lo = self.memory[addr]
                    addr += 1
                    hi = 0
                    if addr_mode == "IMM":
                        line += "#"

                    line += f"${lo:02X}"

                    if addr_mode == "REL":
                        if lo & 0x80:
                            lo |= 0xFF00
                        line += f" [${(addr + lo) & 0xFFFF:04X}]"
                else:
                    lo = self.memory[addr]
                    addr += 1
                    hi = self.memory[addr]
                    addr += 1
                    full_addr = (hi << 8) | lo
                    line += f"${full_addr:04X}"

                if addr_mode in ["INX", "ZPX", "ABX"]:
                    line += ", X"

                if addr_mode in ["INY", "ZPY", "ABY"]:
                    line += ", Y"
            else:
                line = f"${ins_addr:04X} [XXX] INVALID OP"

            lines.append(line)

        return lines

    # Addressing Modes

    def accumulator(self):
        self.fetched = self.A
        return 0

    def implicit(self):
        return 0

    def immediate(self):
        self.absolute_address = self.PC
        self.PC += 1

        return 0

    def zeropage(self):
        self.absolute_address = self.fetch_byte()
        self.absolute_address &= 0x00FF
        return 0

    def zeropageX(self):
        self.absolute_address = self.fetch_byte() + self.X
        self.absolute_address &= 0x00FF

        return 0

    def zeropageY(self):
        self.absolute_address = self.fetch_byte() + self.Y
        self.absolute_address &= 0x00FF
        return 0

    def absolute(self):
        self.absolute_address = self.fetch_word()
        return 0

    def absoluteX(self):
        addr = self.fetch_word()
        self.absolute_address = addr + self.X

        # Page crossed, need additional cycle
        if (self.absolute_address & 0xFF00) != (addr & 0xFF00):
            return 1

        return 0

    def absoluteY(self):
        addr = self.fetch_word()
        self.absolute_address = addr + self.Y

        if (self.absolute_address & 0xFF00) != (addr & 0xFF00):
            return 1

        return 0

    def indirect(self):
        addr = self.fetch_word()

        if (addr & 0xFF) == 0xFF:  # Simulate a page boundary hardware bug
            # See https://www.youtube.com/watch?v=8XmxKPJDGU0
            addr &= 0xFF00
            self.absolute_address = self.read_word(addr)
        else:
            self.absolute_address = self.read_word(addr)

        return 0

    def indirectX(self):
        # See https://www.c64-wiki.com/wiki/Indexed-indirect_addressing
        pointer = self.fetch_byte() + self.X
        lo_byte = self.memory[(pointer & 0xFF)]         # bitwise AND is for page wrap-around
        hi_byte = (self.memory[((pointer + 1) & 0xFF)])  # bitwise AND is for page wrap-around

        self.absolute_address = (hi_byte << 8) | lo_byte

        return 0

    def indirectY(self):
        # See https://stackoverflow.com/questions/46262435/indirect-y-indexed-addressing-mode-in-mos-6502
        # Also https://www.c64-wiki.com/wiki/Indirect-indexed_addressing
        vector = self.fetch_byte()
        pointer = self.read_word(vector)

        self.absolute_address = pointer + self.Y

        # Extra cycle if we cross page boundary
        if (self.absolute_address & 0xFF00) != (pointer & 0xFF00):
            return 1

        return 0

    def relative(self):
        self.relative_address = self.fetch_byte()
        temp = self.relative_address

        if self.relative_address & 0x80:
            self.relative_address |= 0xFF00

        return 0

    # Opcode Functions

    def ADC(self):
        val = self.fetch()
        total = self.A + val + self.C

        # Overflow check
        # See http://www.righto.com/2012/12/the-6502-overflow-flag-explained.html
        if ((val ^ total) & (self.A ^ total) & 0x80) != 0:
            self.V = 1
        else:
            self.V = 0

        self.C = 1 if total > 0xFF else 0

        self.A = total & 0xFF  # Truncate to 8 bits
        self.set_NZ_flag(self.A)

        return 0

    def AND(self):
        # Logical AND bitwise with accumulator
        val = self.fetch()
        self.A = val & self.A
        self.set_NZ_flag(self.A)

        return 0

    def ASL(self):
        # Shift bits to the left one bit.
        if self.opcode.addressing_mode.__name__ == self.accumulator.__name__:
            val = self.A
            temp = val << 1
            self.A = temp & 0xFF
        else:
            val = self.fetch()
            temp = val << 1
            self.memory[self.absolute_address] = temp & 0xFF

        # Overflow check
        if temp > 255:
            self.C = 1
        else:
            self.C = 0

        self.set_NZ_flag(temp)

        return 0

    def BCC(self):
        if self.C == 0:
            cycles = 1
            self.absolute_address = (self.PC + self.relative_address) & 0xFFFF

            # If branch to new page, add extra cycle
            if (self.absolute_address & 0xFF00) != (self.PC & 0xFF00):
                cycles += 1

            self.PC = self.absolute_address

            return cycles
        return 0

    def BCS(self):
        if self.C:
            cycles = 1
            self.absolute_address = (self.PC + self.relative_address) & 0xFFFF

            # If branch to new page, add extra cycle
            if (self.absolute_address & 0xFF00) != (self.PC & 0xFF00):
                cycles += 1

            self.PC = self.absolute_address

            return cycles
        return 0

    def BEQ(self):
        if self.Z:
            cycles = 1
            self.absolute_address = (self.PC + self.relative_address) & 0xFFFF

            # If branch to new page, add extra cycle
            if (self.absolute_address & 0xFF00) != (self.PC & 0xFF00):
                cycles += 1

            self.PC = self.absolute_address

            return cycles
        return 0

    def BIT(self):
        val = self.fetch()

        if self.A & val:
            self.Z = 0
        else:
            self.Z = 1

        self.N = 1 if (val & (1 << 7)) else 0
        self.V = 1 if (val & (1 << 6)) else 0

        return 0

    def BMI(self):
        if self.N:
            cycles = 1
            self.absolute_address = (self.PC + self.relative_address) & 0xFFFF

            # If branch to new page, add extra cycle
            if (self.absolute_address & 0xFF00) != (self.PC & 0xFF00):
                cycles += 1

            self.PC = self.absolute_address

            return cycles
        return 0

    def BNE(self):
        if self.Z == 0:
            cycles = 1
            self.absolute_address = (self.PC + self.relative_address) & 0xFFFF

            # If branch to new page, add extra cycle
            if (self.absolute_address & 0xFF00) != (self.PC & 0xFF00):
                cycles += 1

            self.PC = self.absolute_address

            return cycles
        return 0

    def BPL(self):
        if self.N == 0:
            cycles = 1
            self.absolute_address = (self.PC + self.relative_address) & 0xFFFF

            # If branch to new page, add extra cycle
            if (self.absolute_address & 0xFF00) != (self.PC & 0xFF00):
                cycles += 1

            self.PC = self.absolute_address

            return cycles
        return 0

    def BRK(self):
        self.PC += 1
        self.stack_push((self.PC >> 8) & 0xFF)  # Write PC High Byte
        self.stack_push(self.PC & 0xFF)         # Write PC Low Byte

        # On a BRK instruction, the CPU does the same as in the IRQ case,
        # but sets bit #4 (B flag) IN THE COPY of the status register that is saved on the stack.
        self.stack_push(self.get_status_flags_byte("instruction"))  # Push status register to stack
        self.I = 1

        self.PC = (self.memory[0xFFFF] << 8) | self.memory[0xFFFE]

        return 0

    def BVC(self):
        if self.V == 0:
            cycles = 1
            self.absolute_address = (self.PC + self.relative_address) & 0xFFFF

            # If branch to new page, add extra cycle
            if (self.absolute_address & 0xFF00) != (self.PC & 0xFF00):
                cycles += 1

            self.PC = self.absolute_address

            return cycles
        return 0

    def BVS(self):
        if self.V:
            cycles = 1
            self.absolute_address = (self.PC + self.relative_address) & 0xFFFF

            # If branch to new page, add extra cycle
            if (self.absolute_address & 0xFF00) != (self.PC & 0xFF00):
                cycles += 1

            self.PC = self.absolute_address

            return cycles
        return 0

    def CLC(self):
        self.C = 0
        return 0

    def CLD(self):
        self.D = 0
        return 0

    def CLI(self):
        self.I = 0
        return 0

    def CLV(self):
        self.V = 0
        return 0

    def CMP(self):
        # This instruction compares the contents of the accumulator with another memory
        # held value and sets the zero and carry flags as appropriate.
        val = self.fetch()

        if self.A >= val:
            self.C = 1
        else:
            self.C = 0

        self.set_NZ_flag(self.A - val)
        return 0

    def CPX(self):
        val = self.fetch()

        if self.X >= val:
            self.C = 1
        else:
            self.C = 0

        self.set_NZ_flag(self.X - val)
        return 0

    def CPY(self):
        val = self.fetch()

        if self.Y >= val:
            self.C = 1
        else:
            self.C = 0

        self.set_NZ_flag(self.Y - val)
        return 0

    def DEC(self):
        val = self.fetch()
        temp = (val - 1) & 0xFF
        self.memory[self.absolute_address] = temp
        self.set_NZ_flag(temp)

        return 0

    def DEX(self):
        self.X -= 1
        self.X &= 0xFF
        self.set_NZ_flag(self.X)

        return 0

    def DEY(self):
        self.Y -= 1
        self.Y &= 0xFF
        self.set_NZ_flag(self.Y)

        return 0

    def EOR(self):
        val = self.fetch()
        self.A = self.A ^ val
        self.set_NZ_flag(self.A)
        return 0

    def INC(self):
        val = self.fetch()
        temp = (val + 1) & 0xFF
        self.memory[self.absolute_address] = temp
        self.set_NZ_flag(temp)

        return 0

    def INX(self):
        self.X = (self.X + 1) & 0xFF
        self.set_NZ_flag(self.X)

        return 0

    def INY(self):
        self.Y = (self.Y + 1) & 0xFF
        self.set_NZ_flag(self.Y)

        return 0

    def JMP(self):
        self.PC = self.absolute_address
        return 0

    def JSR(self):
        self.PC -= 1
        self.stack_push((self.PC >> 8) & 0xFF)  # Write PC High Byte
        self.stack_push(self.PC & 0xFF)         # Write PC Low Byte

        self.PC = self.absolute_address

        return 0

    def LDA(self):
        val = self.fetch()
        self.A = val
        self.set_NZ_flag(self.A)

        return 0

    def LDX(self):
        val = self.fetch()
        self.X = val
        self.set_NZ_flag(self.X)

        return 0

    def LDY(self):
        val = self.fetch()
        self.Y = val
        self.set_NZ_flag(self.Y)

        return 0

    def LSR(self):

        if self.opcode.addressing_mode.__name__ == self.accumulator.__name__:
            val = self.A
            temp = val >> 1
            self.A = temp

        else:
            val = self.fetch()
            temp = val >> 1
            self.memory[self.absolute_address] = temp

        # Set carry flag = to bit 0 of value
        self.C = 1 if (val & 0x01) else 0
        self.set_NZ_flag(temp)

        return 0

    def NOP(self):
        return 0

    def ORA(self):
        val = self.fetch()

        self.A = self.A | val
        self.set_NZ_flag(self.A)

        return 0

    def PHA(self):
        """Push Accumulator to Stack"""
        self.stack_push(self.A)

        return 0

    def PLA(self):
        """Pull Accumulator from Stack"""
        self.A = self.stack_pull()
        self.set_NZ_flag(self.A)
        return 0

    def PHP(self):
        """Push Status Flags to Stack"""
        self.stack_push(self.get_status_flags_byte("instruction"))
        self.B = 0

        return 0

    def PLP(self):
        """Pull Status Flags from Stack"""
        self.set_status_flags_byte(self.stack_pull())
        self.B = 0
        return 0

    def ROL(self):

        if self.opcode.addressing_mode.__name__ == self.accumulator.__name__:
            val = self.A
            temp = ((val << 1) & 0xFF) | self.C
            self.A = temp
        else:
            val = self.fetch()
            temp = ((val << 1) & 0xFF) | self.C
            self.memory[self.absolute_address] = temp

        # Set carry flag = to bit 0 of value
        self.C = 1 if (val & (1 << 7)) else 0

        self.set_NZ_flag(temp)

        return 0

    def ROR(self):

        if self.opcode.addressing_mode.__name__ == self.accumulator.__name__:
            val = self.A
            temp = (val >> 1) | (self.C << 7)
            self.A = temp
        else:
            val = self.fetch()
            temp = (val >> 1) | (self.C << 7)
            self.memory[self.absolute_address] = temp

        # Set carry flag = to bit 0 of value
        self.C = 1 if (val & 0x01) else 0

        self.set_NZ_flag(temp)

        return 0

    def RTI(self):
        self.set_status_flags_byte(self.stack_pull())
        self.B = 0  # This might have been set by the BRK routine

        low_byte = self.stack_pull()
        high_byte = self.stack_pull()

        self.PC = ((high_byte << 8) | low_byte)

        return 0

    def RTS(self):
        low_byte = self.stack_pull()
        high_byte = self.stack_pull()

        ret_addr = (high_byte << 8) | low_byte
        self.PC = ret_addr + 1
        return 0

    def SBC(self):
        # See http://www.righto.com/2012/12/the-6502-overflow-flag-explained.html
        # val is converted to ones complement and hence we can use ADC logic
        val = self.fetch()
        val = val ^ 0x00FF

        total = self.A + val + self.C

        # Overflow check
        # See http://www.righto.com/2012/12/the-6502-overflow-flag-explained.html
        if ((val ^ total) & (self.A ^ total) & 0x80) != 0:
            self.V = 1
        else:
            self.V = 0

        self.C = 1 if total > 0xFF else 0  # Carry if more than 255

        self.A = total & 0xFF  # Truncate to 8 bits
        self.set_NZ_flag(self.A)

        return 0

    def SEC(self):
        self.C = 1
        return 0

    def SED(self):
        self.D = 1
        return 0

    def SEI(self):
        self.I = 1
        return 0

    def STA(self):
        self.memory[self.absolute_address] = self.A
        return 0

    def STX(self):
        self.memory[self.absolute_address] = self.X
        return 0

    def STY(self):
        self.memory[self.absolute_address] = self.Y
        return 0

    def TAX(self):
        self.X = self.A
        self.set_NZ_flag(self.X)

        return 0

    def TAY(self):
        self.Y = self.A
        self.set_NZ_flag(self.Y)

        return 0

    def TSX(self):
        self.X = self.SP
        self.set_NZ_flag(self.X)
        return 0

    def TXA(self):
        self.A = self.X
        self.set_NZ_flag(self.A)

        return 0

    def TXS(self):
        self.SP = self.X

        return 0

    def TYA(self):
        self.A = self.Y
        self.set_NZ_flag(self.A)

        return 0


class Trace(NamedTuple):
    op: str
    registers: Registers
    flags: Flags
    clock: int
    last_cycle: int
    mem_access: list[tuple[int, int]] | None
    stack: list[int]
    n_operations: int


class Debugger:
    def __init__(self, cpu: CPU):
        self.cpu = cpu
        self.memory = cpu.memory
        self.trace_stack = []
        self.global_tick = 0
        self.n_operations = 0
        self.recent_write: list[tuple[int, int]] = []
        self.memory.register_debugger(self)
        self.stack: list[int] = []

    def trace(self, func):
        op = self.cpu.dissamble(self.cpu.PC, 1)[0]
        start_tick = self.global_tick
        self.n_operations += 1
        while func():
            pass

        cycles = self.global_tick - start_tick
        registers = self.cpu.get_registers()
        flags = self.cpu.get_status_flags()

        trace = Trace(op, registers, flags, self.global_tick, cycles,
                      self.get_memory_write(), self.get_stack(), self.n_operations)
        self.trace_stack.append(trace)
        if len(self.trace_stack) > 1_000:
            self.trace_stack.pop(0)

    def get_stack(self) -> list[int]:
        stack = self.memory[0x0100:0x01FF + 1]
        stack_pointer = self.cpu.SP
        stack = stack[stack_pointer+1:]
        self.stack = stack
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
