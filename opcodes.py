from enum import IntEnum
from typing import NamedTuple


class OpTuple(NamedTuple):
    bytes: int
    cycles: int
    extra: int = 0


class INS(IntEnum):
    # Add with Carry
    ADC_IM = 0x69
    ADC_ZP = 0x65
    ADC_ZPX = 0x75
    ADC_ABS = 0x6D
    ADC_ABSX = 0x7D
    ADC_ABSY = 0x79
    ADC_INDX = 0x61
    ADC_INDY = 0x71

    LDA_IM = 0xA9
    LDA_ZP = 0xA5
    LDA_ZPX = 0xB5
    JSR = 0x20
    RTS = 0x60


opcodes_table: dict[int, OpTuple] = {
    # OPCodes : [Bytes, Cycles, Additional Cycles]
    INS.ADC_IM: OpTuple(2, 2),
    INS.ADC_ZP: OpTuple(2, 3),
    INS.ADC_ZPX: OpTuple(2, 4),
    INS.ADC_ABS: OpTuple(3, 4),
    INS.ADC_ABSX: OpTuple(3, 4, 1),
    INS.ADC_ABSY: OpTuple(3, 4, 1),
    INS.ADC_INDX: OpTuple(2, 6),
    INS.ADC_INDY: OpTuple(2, 5, 1),
    INS.LDA_IM: OpTuple(2, 2),
    INS.LDA_ZP: OpTuple(2, 3),
    INS.LDA_ZPX: OpTuple(2, 4),
    INS.JSR: OpTuple(3, 6),
    INS.RTS: OpTuple(1, 6)
}
