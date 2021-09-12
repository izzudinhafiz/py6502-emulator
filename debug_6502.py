from flask import Flask, render_template
from flask_socketio import SocketIO
from CPU6502 import CPU
import time

app = Flask(__name__)
socket = SocketIO(app)


@app.route("/")
def index():
    return render_template("index.html")


class WebDebugger:
    def __init__(self, cpu: CPU):
        self.cpu = cpu
        self.memory = cpu.memory
        self.connected: bool = False
        self.run_flag = False
        self.last_pc = 0
        socket.on_event("connect", self.handle_connect)
        socket.on_event("disconnect", self.handle_disconnect)
        socket.on_event("keypress", self.handle_keypress)
        socket.on_event("mem-addr-read", self.handle_memory_read)

    def run(self, skip_ops: int = 0):
        socket.run(app)

    def handle_memory_read(self, value: str):
        value = value.split("=")[-1]
        if "x" in value:
            addr = int(value, 16)
        else:
            addr = int(value)
        socket.emit("mem-addr-read", self.memory[addr], broadcast=True)

    def handle_connect(self):
        print("client connected")
        self.connected = True
        self.send_reset()
        self.send_update()

    def handle_disconnect(self):
        print("client disconnected")
        # self.connected = False

    def handle_keypress(self, key: str):
        match key.lower():
            case " ":
                self.cpu.single_operation()
                self.send_update()
            case "r":
                self.cpu.__init__(self.memory)
                self.send_reset()
            case "enter":
                if not self.run_flag:
                    self.run_flag = True
                    socket.start_background_task(self.run_continously)
            case "s":
                self.run_flag = False
                self.send_update()
            case "t":
                self.skip_operations(30_646_000)
            case _:
                pass

    def skip_operations(self, n_ops: int):
        for n in range(n_ops):
            if n % 100_000 == 0:
                print(f"Ops: {n:,} PC: {self.cpu.PC}")
            self.cpu.single_operation(False)
            # if self.cpu.PC == self.last_pc:
            #     break
            # self.last_pc = self.cpu.PC
        self.send_update()

    def run_continously(self):
        while self.run_flag:
            self.cpu.single_operation()
            self.send_update()
            if self.cpu.PC == self.last_pc:
                self.run_flag = False
                break
            self.last_pc = self.cpu.PC
            time.sleep(0.1)

    def send_update(self):
        mem_lower = {}
        for i in range(0, 0x0100, 0x10):
            mem_lower[i] = self.memory[i:i + 16]

        mem_upper = {}
        for i in range(len(self.memory) - 0x10 * 16, len(self.memory), 0x10):
            mem_upper[i] = self.memory[i:i + 16]

        with app.app_context():
            infos = {
                "mem_block": render_template('memory.html', memory_lower=mem_lower, memory_upper=mem_upper),
                "flags": {
                    "N": self.cpu.N,
                    "V": self.cpu.V,
                    "B": self.cpu.B,
                    "D": self.cpu.D,
                    "I": self.cpu.I,
                    "Z": self.cpu.Z,
                    "C": self.cpu.C},
                "registers": {
                    "p_counter": self.cpu.PC,
                    "a_reg": self.cpu.A,
                    "x_reg": self.cpu.X,
                    "y_reg": self.cpu.Y,
                    "s_pointer": self.cpu.SP,
                },
                "programs": self.cpu.dissamble(self.cpu.PC),
                "trace": self.cpu.dbg.get_last_trace()
            }

        socket.emit("update_info", infos, broadcast=True)

    def send_reset(self):
        socket.emit("reset", broadcast=True)
