# NMOS 6502 Emulator built in Python

## Introduction
This is a semi-cycle accurate 6502 CPU emulator. It implements all known opcodes _only_. Decimal mode arithmetics are also implemented. The C flag is valid in decimal mode while N, V, Z is not valid as per the original 6502 implementation.

This application also include a web-based debugger to inspect the internal state of the CPU and memory. It is a Flask web app.

![6502 Web Debugger](https://raw.githubusercontent.com/izzudinhafiz/py6502-emulator/master/C6502/images/WebDebugger.png?raw=true)

## Web Debugger Shortcuts
`Enter` - Run Code Continuously

`Space` - Step one instruction

`T` - Skip N operations (without displaying it to the debugger. This makes it run at full speed)

`R` - Reset CPU (Does not reset memory)

`S` - Stop Continuous Run

## References
One Lone Coder's excellent tutorial at [OLC NES](https://github.com/OneLoneCoder/olcNES)

Bruce Clark's decimal mode write up at [6502.org](http://6502.org/tutorials/decimal_mode.html)

Opcodes Reference at [6502.org](http://www.6502.org/tutorials/6502opcodes.html)

Instructions Reference at [obelisk.me.uk](http://www.obelisk.me.uk/6502/reference.html)
